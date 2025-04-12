#include <ns3/simulator.h>
#include <ns3/seq-ts-header.h>
#include <ns3/udp-header.h>
#include <ns3/ipv4-header.h>
#include "ns3/ppp-header.h"
#include "ns3/boolean.h"
#include "ns3/uinteger.h"
#include "ns3/double.h"
#include "ns3/data-rate.h"
#include "ns3/pointer.h"
#include "rdma-hw.h"
#include "ppp-header.h"
#include "qbb-header.h"
#include "cn-header.h"
#include "rdma-hw-cc.h"

/*******super parameter area*************/
#define CTCCL_SUPA_C_LOG (4)
#define CTCCL_SUPA_BETTA_LOGDENO (1)
#define CTCCL_SUPA_GAMMA (4)
#define CTCCL_SUPA_ETA (7)
#define CTCCL_SUPA_W (20480)
#define CTCCL_SUPA_B (3072)
#define CTCCL_SUPA_TCP_FRIENDLY (0)

// fixed macro area
#define CTCCL_INIT_WIN_MTU_MUL (10)
#define CTCCL_RTT_BASE_MINIMUM (2000)
#define CTCCL_RTT_BASE_MAX (1000000)
#define CTCCL_BIT_TO_BYTE_LOG (3)
#define CTCCL_AVG_RTT_DENO_LOG (5)
#define MAX_UINT64 (0xffffffffffffffff)

namespace ns3{
void RdmaHw::ctcclossy_all_ev_deal(Ptr<RdmaQueuePair> qp,
	cc_ev_type_s ev_type, cc_event_data_s &ev_data)
{
	switch (ev_type) {
		case cc_ev_init:
			ctcclossy_ev_init(qp->ctccLossy, ev_data);
		break;
		case cc_ev_resp:
			ctcclossy_ev_resp(qp, ev_data);
		break;
		default:
		break;
	}
}

void RdmaHw::ctcclossy_ev_init(ctcclossy_algo_s &algo_ctx,
	cc_event_data_s &ev_data)
{
	algo_ctx.cwnd = ctcclossy_unit((CTCCL_INIT_WIN_MTU_MUL * m_mtu));
	// min cwnd = ctcclossy_unit(m_mtu >> CTCCL_SUPA_GAMMA)
	// max cwnd = ctcclossy_unit(m_mtu << CTCCL_SUPA_ETA)
	algo_ctx.base_rtt = CTCCL_RTT_BASE_MAX;
	algo_ctx.prev_rtt = CTCCL_RTT_BASE_MINIMUM;
	algo_ctx.avg_rtt = CTCCL_RTT_BASE_MINIMUM;
	ctcclossy_update_realtime_rate(algo_ctx);
	algo_ctx.last_time = ev_data.ev_time;
	algo_ctx.rate = 0.0;

	algo_ctx.time_len_K = 0;
	algo_ctx.epoch_start_time = 0;
	algo_ctx.origin_point_win = 0;
	algo_ctx.last_win_max = 0;

	algo_ctx.add_resp_pkts_cnt = 0;
	algo_ctx.red_resp_pkts_cnt = 0;

	algo_ctx.tcp_cwnd = algo_ctx.cwnd;
	algo_ctx.tcp_start_time = ev_data.ev_time;
}

void RdmaHw::ctcclossy_update_realtime_rate(ctcclossy_algo_s &algo_ctx)
{
	algo_ctx.rc = (uint32_t)(((uint64_t)algo_ctx.cwnd << (10 +
		CCPFL_EPSILON + CTCCL_BIT_TO_BYTE_LOG))/(uint64_t)algo_ctx.prev_rtt);
	if (algo_ctx.rc == 0) {
		// less than 1Mbps, assign 1Mbps
		algo_ctx.rc = 1;
	}
}

void RdmaHw::ctcclossy_ev_resp(Ptr<RdmaQueuePair> qp, cc_event_data_s &ev_data)
{
	ctcclossy_algo_s &algo_ctx = qp->ctccLossy;
	uint64_t curr_time = ev_data.ev_time;
	uint32_t cwnd_add_resp_cnt = 0; 
	uint32_t cur_rtt = 0;
	uint64_t rtt_target = 0;
	uint64_t rtt_power = 0;
	uint64_t space_time = 0;
    uint32_t rtt_diff = 0;
	uint32_t cwnd_red = 0;
	uint32_t cwnd_inc = 0;
	uint32_t target_cwnd = 0;

	if (curr_time < ev_data.spec.resp.rtt_det_send_time) {
		return;
	}
	cur_rtt = (uint32_t)(curr_time - ev_data.spec.resp.rtt_det_send_time);
	if (ev_data.spec.resp.rtt_det_send_time > curr_time) {
		cur_rtt = (uint32_t)(curr_time +
			(MAX_UINT64 - ev_data.spec.resp.rtt_det_send_time));
	}
	if ((cur_rtt > CTCCL_RTT_BASE_MINIMUM) && (cur_rtt < algo_ctx.base_rtt)) {
		algo_ctx.base_rtt = cur_rtt;
	}

    uint64_t timestamp_diff = (uint64_t)(curr_time - algo_ctx.last_time);
    if (algo_ctx.last_time > curr_time) {
        timestamp_diff = curr_time + (MAX_UINT64 - algo_ctx.last_time);
    }

	if (cur_rtt >= algo_ctx.prev_rtt) {
		rtt_diff = cur_rtt - algo_ctx.prev_rtt;
		rtt_power = (uint64_t)cur_rtt +
			(((uint64_t)rtt_diff * (uint64_t)algo_ctx.base_rtt) / timestamp_diff);
	} else {
		rtt_diff = algo_ctx.prev_rtt - cur_rtt;
		uint64_t tmp = (((uint64_t)rtt_diff * (uint64_t)algo_ctx.base_rtt) / timestamp_diff);
		if ((uint64_t)cur_rtt > tmp) {
			rtt_power = (uint64_t)cur_rtt - tmp;
		} else {
			rtt_power = (uint64_t)cur_rtt;
		}
	}
	rtt_target = algo_ctx.base_rtt + (CTCCL_SUPA_W/sqrt(algo_ctx.rc)) + CTCCL_SUPA_B;
	//if the rtt_power is larger than rtt_target
	if (rtt_power > rtt_target) {
		algo_ctx.add_resp_pkts_cnt = 0;
		algo_ctx.red_resp_pkts_cnt += 1;  // the ptks acked for this event
		if (algo_ctx.epoch_start_time > 0) {  // this is the start point for the redction period
			algo_ctx.epoch_start_time = 0; // epoch_start_time is the k^3 adding start time point, here need to clear it
			/*
				if the redcution starts behind the last_wind_max, means the network not good as before(last_win_max), we need to push the last_win_max 
				even smaller than current window size.
			*/
			if (algo_ctx.cwnd < algo_ctx.last_win_max) {
				algo_ctx.last_win_max = algo_ctx.cwnd -
					(algo_ctx.cwnd >> (CTCCL_SUPA_BETTA_LOGDENO + 1));
			} else {
				algo_ctx.last_win_max = algo_ctx.cwnd;
			}
		}

		if (algo_ctx.cwnd > ctcclossy_unit(m_mtu)) { //reduce pkts for the event, e.g, 5pkts, we reduce 5*0.5. As a result, 1 rtt will reduce total *0.5
			cwnd_red = ((algo_ctx.red_resp_pkts_cnt * m_mtu) >>
				CTCCL_SUPA_BETTA_LOGDENO);
			cwnd_red = ctcclossy_unit(cwnd_red);
		} else {
			cwnd_red = algo_ctx.cwnd >> 1;
		}
		algo_ctx.red_resp_pkts_cnt = 0;  // clear the acked pkts for this event
		if (algo_ctx.cwnd > (cwnd_red +  // reduce the cwnd for this event, and make sure the window is larger than (m_mtu >> CTCCL_SUPA_GAMMA) Byte
			ctcclossy_unit(m_mtu >> CTCCL_SUPA_GAMMA))) {
			algo_ctx.cwnd -= cwnd_red;
		} else {
			algo_ctx.cwnd = ctcclossy_unit(m_mtu >> CTCCL_SUPA_GAMMA);
		}
	} else {
		algo_ctx.red_resp_pkts_cnt = 0;
		algo_ctx.add_resp_pkts_cnt += 1; // the ptks acked for this event
		if (algo_ctx.epoch_start_time <= 0) {  // if this the start point for k^3 increasing
			algo_ctx.epoch_start_time = curr_time; // we record the add epoch start time
			if (algo_ctx.cwnd < algo_ctx.last_win_max) { // as the start point, we calculate the time needs to climb to the last_win_max and store it in time_len_K
				algo_ctx.time_len_K =
					sqrt(((algo_ctx.last_win_max -
					algo_ctx.cwnd) << CTCCL_SUPA_C_LOG));
				algo_ctx.origin_point_win = algo_ctx.last_win_max;  //origin_point_win record the inflection point window
			} else {   
				/*
					Since in the redcution point, we may push the last_win_max even smaller than its window at the time, so here we may face the condition that
					before the window reduce to the last_win_max, the network becomes good, threrefore, we assume that we alread climb to the top of the last_win_max windows which is window
					size for now. What we should is gradually increase the windows size according to the k^3 function(ofcourse, the remianed part curve). 
				*/
				algo_ctx.time_len_K = 0;
				algo_ctx.origin_point_win = algo_ctx.cwnd;
			}
			algo_ctx.tcp_cwnd = algo_ctx.cwnd;
			algo_ctx.tcp_start_time = curr_time;
			std::cout << "calculateK|qpn:" << qp->qp_no << "|lastwinmax:" <<
				algo_ctx.last_win_max << "|curwin:" << algo_ctx.cwnd <<
				"|timeKval:" << algo_ctx.time_len_K << "|rttpower" <<
				rtt_power << "|rtttarget" << rtt_target << std::endl;
		}

		/* curr_time,base_rtt,epoch_start_time,time_len_K,space_time use same unit.
		origin_point_win,target_cwnd use same unit */

		/*
			Here, we calculate the distance between one rtt later  and current rate increasing epoch start point.
			The we get the target_cwnd using the k^3 function. It first denotes the window offset from the origin_point_win, origin_point_win denotes the mathmatical inflection point
			Here, we only use the k^3 predict the far to base rtt instead of direct use time_len_k. Because it cloud be too far, or too close.
		*/
		space_time = (curr_time + algo_ctx.base_rtt) - algo_ctx.epoch_start_time;
		if (space_time < algo_ctx.time_len_K) {   // if the space_time falls into the left of the inflection point
			target_cwnd = (((algo_ctx.time_len_K - space_time) *
				(algo_ctx.time_len_K - space_time)) >> CTCCL_SUPA_C_LOG); // get the target_cwnd using the k^3 function, but with a hyperparameter CTCCL_SUPA_C_LOG
			
			/*
				Here only prevent corner case that the target_cwnd is higer than origin_point_win(which is impossible for math), then target_cwnd  becomes minimal.
				we use k^3 calculate the target_cwnd near the inflection point. The absolute target window is origin_point_win - target_cwnd, 
				and when the space_time smaller, the target_cwnd becomse smaller but with slower speed, the origin_point_win becomes bigger but also with a slower speed. 
			*/
			if (algo_ctx.origin_point_win < (target_cwnd +
				ctcclossy_unit(m_mtu >> CTCCL_SUPA_GAMMA))) {    
				target_cwnd = ctcclossy_unit(m_mtu >> CTCCL_SUPA_GAMMA);
			} else {
				target_cwnd = algo_ctx.origin_point_win - target_cwnd;
			}
		} else {   // if the space_time falls into the right of the inflection point. use k^3 directly. which first slow then fast
			target_cwnd = algo_ctx.origin_point_win + (((space_time - algo_ctx.time_len_K) *
				(space_time - algo_ctx.time_len_K)) >> CTCCL_SUPA_C_LOG);
		}

		// we spray the add to each pkts.
		if (target_cwnd > algo_ctx.cwnd) {
			cwnd_add_resp_cnt = algo_ctx.cwnd/(target_cwnd - algo_ctx.cwnd); //every receive cwnd_add_resp_cnt bytes, we add 1 byte
		} else {
			cwnd_add_resp_cnt = (algo_ctx.cwnd << 7);
		}
		if (cwnd_add_resp_cnt == 0) {
			cwnd_add_resp_cnt = 1;
		}

		cwnd_inc = algo_ctx.add_resp_pkts_cnt/cwnd_add_resp_cnt;
		std::cout<<"cwnd_inc :"<<cwnd_inc<<std::endl;

		algo_ctx.add_resp_pkts_cnt -= (cwnd_inc * cwnd_add_resp_cnt);
		algo_ctx.cwnd += cwnd_inc;
		if (algo_ctx.cwnd > ctcclossy_unit(m_mtu << CTCCL_SUPA_ETA)) {
			algo_ctx.cwnd = ctcclossy_unit(m_mtu << CTCCL_SUPA_ETA);
		}
	}
	algo_ctx.prev_rtt = cur_rtt;
	algo_ctx.avg_rtt = (algo_ctx.prev_rtt >> CTCCL_AVG_RTT_DENO_LOG) +
		(algo_ctx.avg_rtt - (algo_ctx.avg_rtt >> CTCCL_AVG_RTT_DENO_LOG));
	algo_ctx.last_time = curr_time;    //update the last event time
	ctcclossy_update_realtime_rate(algo_ctx);
	algo_ctx.rate = (double)(qp->ctccLossy.cwnd << 3)/(double)qp->ctccLossy.avg_rtt;
	std::cout << " sip: " << qp->sip.Get() << " qpn: " << qp->qp_no << " cwnd: "<< algo_ctx.cwnd << " realrtt: " << algo_ctx.prev_rtt << " rate: " <<
	algo_ctx.rate << " avgrtt: "<< algo_ctx.avg_rtt << std::endl;
}

void RdmaHw::ctcclossy_recv_ack_handle(Ptr<RdmaQueuePair> qp, Ptr<Packet> p,
	CustomHeader &ch, Ptr<QbbNetDevice> dev)
	{
		uint32_t ack_seq = ch.ack.seq;
		cc_event_data_s event_pri_data;
		event_pri_data.spec.resp.rtt_det_send_time = ch.ack.ih.ts;
		event_pri_data.ev_time = Simulator::Now().GetTimeStep();
		ctcclossy_all_ev_deal(qp, cc_ev_resp, event_pri_data);
		qp->HPS.infilight_pkts_limitation = (double)qp->ctccLossy.cwnd/(double)ctcclossy_unit(m_mtu);
		qp->HPS.rtt = qp->ctccLossy.avg_rtt;
		ChangeWindow(qp);
		dev->TriggerTransmit();
	}

}
