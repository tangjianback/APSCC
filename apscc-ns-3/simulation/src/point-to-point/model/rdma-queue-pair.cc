#include <ns3/hash.h>
#include <ns3/uinteger.h>
#include <ns3/seq-ts-header.h>
#include <ns3/udp-header.h>
#include <ns3/ipv4-header.h>
#include <ns3/simulator.h>
#include "ns3/ppp-header.h"
#include "rdma-queue-pair.h"

#define HPS_WINDOW_SIZE 128

namespace ns3 {

/**************************
 * RdmaQueuePair
 *************************/
TypeId RdmaQueuePair::GetTypeId (void)
{
	static TypeId tid = TypeId ("ns3::RdmaQueuePair")
		.SetParent<Object> ()
		;
	return tid;
}

RdmaQueuePair::RdmaQueuePair(uint16_t pg, Ipv4Address _sip, Ipv4Address _dip, uint16_t _sport, uint16_t _dport){
	startTime = Simulator::Now();
	sip = _sip;
	dip = _dip;
	sport = _sport;
	dport = _dport;
	m_size = 0;
	snd_nxt = snd_una = 0;
	m_pg = pg;
	m_ipid = 0;
	m_win = 0;
	m_baseRtt = 0;
	m_max_rate = 0;
	m_var_win = false;
	m_rate = 0;
	m_nextAvail = Time(0);
	mlx.m_alpha = 1;
	mlx.m_alpha_cnp_arrived = false;
	mlx.m_first_cnp = true;
	mlx.m_decrease_cnp_arrived = false;
	mlx.m_rpTimeStage = 0;
	hp.m_lastUpdateSeq = 0;
	for (uint32_t i = 0; i < sizeof(hp.keep) / sizeof(hp.keep[0]); i++)
		hp.keep[i] = 0;
	hp.m_incStage = 0;
	hp.m_lastGap = 0;
	hp.u = 1;
	for (uint32_t i = 0; i < IntHeader::maxHop; i++){
		hp.hopState[i].u = 1;
		hp.hopState[i].incStage = 0;
	}

	tmly.m_lastUpdateSeq = 0;
	tmly.m_incStage = 0;
	tmly.lastRtt = 0;
	tmly.rttDiff = 0;

	dctcp.m_lastUpdateSeq = 0;
	dctcp.m_caState = 0;
	dctcp.m_highSeq = 0;
	dctcp.m_alpha = 1;
	dctcp.m_ecnCnt = 0;
	dctcp.m_batchSizeOfAlpha = 0;

	hpccPint.m_lastUpdateSeq = 0;
	hpccPint.m_incStage = 0;

	//jiange add swift

	tj_increat_ai_forfabric = 5.0;
	tj_decreat_b_forfabric = 0.5;
	tj_decreat_b_max_forfabric = 0.7;

	/*
	target函数的主要参数
	farange   50000
	tj_fs_min_cwnd 0.01
	tj_fs_max_cwnd 30.0
	*/
	qp_no = total_qp_no++;
	tj_min_window_forall = 0.001;
	tj_max_window_forall = HPS_WINDOW_SIZE;
	tj_fs_min_cwnd = 0.01;
	tj_fs_max_cwnd = 0.7*HPS_WINDOW_SIZE;
	tj_fs_range = 50000;
	tj_fs_base_target = 10000; //4178
	base_rtt_unit = 1000;
	path_scale_factor = 100;

	tj_fs_alfa = (double)tj_fs_range / ((1 / sqrt(tj_fs_min_cwnd)) - (1 / sqrt(tj_fs_max_cwnd)));
	tj_fs_beta = -tj_fs_alfa / sqrt(tj_fs_max_cwnd);

	tj_window_now_forfabric = tj_fs_max_cwnd; //TO-DO add init window function
	tj_inflight_packet = 0;
	tj_vague_rtt = 5000;
	tj_last_time_decrease_forfabric = Simulator::Now().GetInteger();
	tj_delay_now_forfabric = 10000; //TO-DO add init delay set up


	/*CTCC*/
	CTCC.m_first_seq_in_cur_rtt = 0;
	CTCC.m_first_seq_in_rtt_meas = 0;
	CTCC.m_rtt = 0;
	CTCC.m_increase_stage_cnt = 0; //20230719 cheny@chinatelecom.cn
	CTCC.cnt_delayinc = 0;
	CTCC.m_flag_was_cnp_during_cur_rtt = false;
	CTCC.m_rate_decrease_alpha = 1;
	CTCC.rttDiff = 0;
	CTCC.first_cnp = 1;
	// CTCC.rtt_cnt = 0;
	CTCC.m_prev_ts = 0;
	CTCC.m_line_rate_diff = 0;
	CTCC.m_last_rtt_req_time = 0;
	CTCC.m_rtt_meas_seq = 0;
	CTCC.m_new_rtt = 0;


	/*HPS Sending QP fields*/
	HPS.next_send_psn = 1;
	HPS.tx_base_psn = 1;

	/*CC Fields*/
	HPS.infilight_pkts_max = HPS_WINDOW_SIZE;
	HPS.infilight_pkts_min = 0.1;
	HPS.rtt_ewma = 0.5;
	HPS.add_step_rate = 3.0; //2pkts
	HPS.base_rtt_percentage = 0.2;
	HPS.base_rtt_unit = 1000;
	HPS.path_scale_fator = 0.1;
	HPS.ooodegreee_factor = 2.0;
	HPS.largest_acked_psn = 0;
	HPS.decrease_factor = 0.4;
	HPS.fairness_fac =0.99;

	HPS.infilight_pkts_limitation = HPS.infilight_pkts_max;
	HPS.rtt = 5000;
	HPS.base_rtt = HPS.rtt;
	HPS.ooo_acc = 0;
	HPS.ack_count = 0;
	HPS.ecn_number = 0;
	HPS.ack_number = 0;
	HPS.last_update_time = Simulator::Now().GetInteger();
	HPS.ecn_ratio = 0;
	for(int i=0;i<HPS.infilight_pkts_max;i++)
		HPS.bitmap.push_back(0);
	HPS.bitmap_head = 0;

	/*PCN*/
	PCN.w_min = 1.0/64.0;
	PCN.w_max = 0.5;
	PCN.min_rate = 100000000; //100Mbps
	
	PCN.w = PCN.w_min;

}

void RdmaQueuePair::SetSize(uint64_t size){
	m_size = size;
}

void RdmaQueuePair::SetWin(uint32_t win){
	m_win = win;
}

void RdmaQueuePair::SetBaseRtt(uint64_t baseRtt){
	m_baseRtt = baseRtt;
}

void RdmaQueuePair::SetVarWin(bool v){
	m_var_win = v;
}

void RdmaQueuePair::SetAppNotifyCallback(Callback<void> notifyAppFinish){
	m_notifyAppFinish = notifyAppFinish;
}

uint64_t RdmaQueuePair::GetBytesLeft(){
	uint64_t BytesLeft = 0;
	if ((!HPS.quick_send_queue.empty()) || HPS.next_send_psn < m_size + 1){
		BytesLeft = 1;
	}
	return BytesLeft;
}

uint32_t RdmaQueuePair::GetHash(void){
	union{
		struct {
			uint32_t sip, dip;
			uint16_t sport, dport;
		};
		char c[12];
	} buf;
	buf.sip = sip.Get();
	buf.dip = dip.Get();
	buf.sport = sport;
	buf.dport = dport;
	return Hash32(buf.c, 12);
}

void RdmaQueuePair::Acknowledge(uint32_t ack_psn){
	//special ptks for PCN
	if (ack_psn == 0)
		return;
	uint32_t acked_psn_index;
	NS_ASSERT_MSG(ack_psn < HPS.tx_base_psn + HPS.bitmap.size(),"ERROR, distance too large");
	if(ack_psn >= HPS.tx_base_psn){
		acked_psn_index = ack_psn - HPS.tx_base_psn;
		HPS.bitmap[(acked_psn_index + HPS.bitmap_head)%HPS.bitmap.size()] = 1;
		while(HPS.bitmap[HPS.bitmap_head]==1){
			HPS.bitmap[HPS.bitmap_head]=0;
			HPS.tx_base_psn +=1;
			HPS.bitmap_head = (HPS.bitmap_head+1)%HPS.bitmap.size();
		}
	}
	HPS.largest_acked_psn = HPS.largest_acked_psn < ack_psn?ack_psn:HPS.largest_acked_psn;
}

uint32_t RdmaQueuePair::GetHPSFly(){
	// std::cout<<std::endl;
	// for(auto i : HPS.bitmap){
	// 	std::cout<<uint32_t(i);
	// }
	// std::cout<<std::endl;
	// std::cout<<"HPS.bitmap_head:"<<HPS.bitmap_head<<" HPS.next_send_psn:"<<HPS.next_send_psn<<" HPS.tx_base_psn:"<<HPS.tx_base_psn<<std::endl;

	uint32_t fly_count = 0;
	uint32_t next_send_psn_index = (HPS.bitmap_head+(HPS.next_send_psn - HPS.tx_base_psn))%HPS.bitmap.size();
	uint32_t head_index = HPS.bitmap_head;
	while(head_index!=next_send_psn_index){
		if(HPS.bitmap[head_index]==0){
			fly_count+=1;
		}
		head_index = (head_index+1)%HPS.bitmap.size();
	}
	// std::cout<<"fly_count:"<<fly_count<<std::endl;
	return fly_count;
}

uint64_t RdmaQueuePair::GetOnTheFly(){
	return snd_nxt - snd_una;
}

bool RdmaQueuePair::IsWinBound(){
	uint64_t w = GetWin();
	return w != 0 && GetOnTheFly() >= w;
}

uint64_t RdmaQueuePair::GetWin(){
	if (m_win == 0)
		return 0;
	uint64_t w;
	if (m_var_win){
		w = m_win * m_rate.GetBitRate() / m_max_rate.GetBitRate();
		if (w == 0)
			w = 1; // must > 0
	}else{
		w = m_win;
	}
	return w;
}

uint64_t RdmaQueuePair::HpGetCurWin(){
	if (m_win == 0)
		return 0;
	uint64_t w;
	if (m_var_win){
		w = m_win * hp.m_curRate.GetBitRate() / m_max_rate.GetBitRate();
		if (w == 0)
			w = 1; // must > 0
	}else{
		w = m_win;
	}
	return w;
}

bool RdmaQueuePair::IsFinished(){
	// return snd_una >= m_size;
	return HPS.tx_base_psn > m_size;
}
uint32_t RdmaQueuePair::total_qp_no = 0;
uint32_t RdmaQueuePair::qp_unfinished = 0;

/*********************
 * RdmaRxQueuePair
 ********************/
TypeId RdmaRxQueuePair::GetTypeId (void)
{
	static TypeId tid = TypeId ("ns3::RdmaRxQueuePair")
		.SetParent<Object> ()
		;
	return tid;
}

RdmaRxQueuePair::RdmaRxQueuePair(){
	sip = dip = sport = dport = 0;
	m_ipid = 0;
	ReceiverNextExpectedSeq = 0;
	m_nackTimer = Time(0);
	m_milestone_rx = 0;
	m_lastNACK = 0;
	rxqp_no = total_rxqp_no;
	total_rxqp_no+=1;
	HPS.next_expect_psn = 1;

	for(int i =0;i<HPS_WINDOW_SIZE;i++){
		HPS.bitmap.push_back(0);
	}
	HPS.base_require_psn = 1;
	HPS.max_received_psn = 0;
	HPS.bitmap_head = 0;

	PCN.cnp_inteval = 50000;//50us
	PCN.first_udp = 1;
	PCN.ecn_count = 0;
	PCN.pkts_received = 0;
}

uint32_t RdmaRxQueuePair::total_rxqp_no = 0;

uint32_t RdmaRxQueuePair::GetHash(void){
	union{
		struct {
			uint32_t sip, dip;
			uint16_t sport, dport;
		};
		char c[12];
	} buf;
	buf.sip = sip;
	buf.dip = dip;
	buf.sport = sport;
	buf.dport = dport;
	return Hash32(buf.c, 12);
}

/*********************
 * RdmaQueuePairGroup
 ********************/
TypeId RdmaQueuePairGroup::GetTypeId (void)
{
	static TypeId tid = TypeId ("ns3::RdmaQueuePairGroup")
		.SetParent<Object> ()
		;
	return tid;
}

RdmaQueuePairGroup::RdmaQueuePairGroup(void){
}

uint32_t RdmaQueuePairGroup::GetN(void){
	return m_qps.size();
}

Ptr<RdmaQueuePair> RdmaQueuePairGroup::Get(uint32_t idx){
	return m_qps[idx];
}

Ptr<RdmaQueuePair> RdmaQueuePairGroup::operator[](uint32_t idx){
	return m_qps[idx];
}

void RdmaQueuePairGroup::AddQp(Ptr<RdmaQueuePair> qp){
	m_qps.push_back(qp);
}

#if 0
void RdmaQueuePairGroup::AddRxQp(Ptr<RdmaRxQueuePair> rxQp){
	m_rxQps.push_back(rxQp);
}
#endif

void RdmaQueuePairGroup::Clear(void){
	m_qps.clear();
}

}
