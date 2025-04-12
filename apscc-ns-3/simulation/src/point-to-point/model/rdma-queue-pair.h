#ifndef RDMA_QUEUE_PAIR_H
#define RDMA_QUEUE_PAIR_H

#include <ns3/object.h>
#include <ns3/packet.h>
#include <ns3/ipv4-address.h>
#include <ns3/data-rate.h>
#include <ns3/event-id.h>
#include <ns3/custom-header.h>
#include <ns3/int-header.h>
#include <vector>
#include <queue>
#include "rdma-hw-cc.h"

namespace ns3 {

/*HPS SEND QP Fields*/
struct SendPKTInfo {
	uint32_t psn;
	SendPKTInfo(uint32_t _psn){
		psn = _psn;
	}
};

class RdmaQueuePair : public Object {
public:
	static uint32_t total_qp_no;
	static uint32_t qp_unfinished;
	uint32_t qp_no;

	Time startTime;
	Ipv4Address sip, dip;
	uint16_t sport, dport;
	uint64_t m_size;
	uint64_t snd_nxt, snd_una; // next seq to send, the highest unacked seq
	uint16_t m_pg;
	uint16_t m_ipid;
	uint32_t m_win; // bound of on-the-fly packets
	uint64_t m_baseRtt; // base RTT of this qp
	DataRate m_max_rate; // max rate
	bool m_var_win; // variable window size
	Time m_nextAvail;	//< Soonest time of next send
	uint32_t wp; // current window of packets
	uint32_t lastPktSize;
	Callback<void> m_notifyAppFinish;

	struct
	{
		double w_min;
		double w_max;
		uint64_t min_rate;
		
		double w;
	} PCN;
	


	struct 
	{
		//used for congestion control
		/*Hyper parameter*/
		double infilight_pkts_max;
		double infilight_pkts_min;
		double rtt_ewma;
		double base_rtt_percentage;
		double add_step_rate;
		double decrease_factor;
		double path_scale_fator;
		uint16_t parallel_path;
		uint64_t base_rtt_unit;
		double ooodegreee_factor;
		double fairness_fac;

		/*Variable*/
		double infilight_pkts_limitation;
		uint64_t rtt;
		uint64_t base_rtt;
		uint64_t ooo_acc;
		uint32_t ack_count;
		uint32_t ecn_number;
		uint32_t ack_number;
		uint64_t last_update_time;
		double ecn_ratio;
		uint32_t largest_acked_psn;

		std::vector<uint8_t> bitmap;
		uint32_t bitmap_head;
		
		//used for transport
		uint32_t next_send_psn;
		uint32_t tx_base_psn;

		//used for ns-3
		std::queue<SendPKTInfo> quick_send_queue;
	} HPS;
	

	//SWFIT CC
	double tj_window_pre_forfabric;
	double tj_window_now_forfabric;
	uint64_t tj_inflight_packet;

	int64_t tj_delay_now_forfabric;
	int64_t tj_delay_pre_forfabric;

	int64_t tj_target_delay_forfabric;

	int64_t tj_vague_rtt;
	int64_t tj_vague_diff;


	int64_t tj_last_time_decrease_forfabric;

	double tj_fs_min_cwnd;
	double tj_fs_max_cwnd;
	int64_t tj_fs_range;

	int64_t tj_fs_base_target;

	double tj_fs_alfa;
	double tj_fs_beta;

	double tj_increat_ai_forfabric;
	double tj_decreat_b_forfabric;

	double tj_decreat_b_max_forfabric;
	double tj_min_window_forall;
	double tj_max_window_forall;
	uint64_t base_rtt_unit;
	uint64_t path_scale_factor;

	/*CTCC*/
	struct {  // 确定数据类型，初始化
		uint32_t m_first_seq_in_cur_rtt;
		uint32_t m_first_seq_in_rtt_meas;
		double m_rtt;
		uint32_t cnt_delayinc;
		uint32_t m_increase_stage_cnt;
		bool m_flag_was_cnp_during_cur_rtt;
		double m_rate_decrease_alpha;
		DataRate m_targetRate;
		double rttDiff;
		// uint32_t rtt_cnt;
		bool first_cnp;
		uint64_t m_prev_ts;
		DataRate m_line_rate_diff;
		uint64_t m_last_rtt_req_time;
		uint32_t m_rtt_meas_seq;
		uint64_t m_new_rtt;
	} CTCC;

	ctcclossy_algo_s ctccLossy;


	// double tj_min_window_forall;
	// double tj_max_window_forall;
	// double tj_fs_min_cwnd;
	// double tj_fs_max_cwnd;
	// uint64_t tj_fs_range;
	// uint64_t tj_fs_very_base_rtt;
	// uint64_t tj_fs_base_target;
	// uint64_t tj_vague_var;
	// uint64_t last_update;

	// double tj_fs_alfa;
	// double tj_fs_beta;

	// double tj_window_now_forfabric;
	// uint32_t tj_inflight_packet;
	// uint64_t tj_vague_rtt;
	// uint64_t tj_last_time_decrease_forfabric;
	// uint64_t tj_delay_now_forfabric;

	/******************************
	 * runtime states
	 *****************************/
	DataRate m_rate;	//< Current rate
	struct {
		DataRate m_targetRate;	//< Target rate
		EventId m_eventUpdateAlpha;
		double m_alpha;
		bool m_alpha_cnp_arrived; // indicate if CNP arrived in the last slot
		bool m_first_cnp; // indicate if the current CNP is the first CNP
		EventId m_eventDecreaseRate;
		bool m_decrease_cnp_arrived; // indicate if CNP arrived in the last slot
		uint32_t m_rpTimeStage;
		EventId m_rpTimer;
	} mlx;
	struct {
		uint32_t m_lastUpdateSeq;
		DataRate m_curRate;
		IntHop hop[IntHeader::maxHop];
		uint32_t keep[IntHeader::maxHop];
		uint32_t m_incStage;
		double m_lastGap;
		double u;
		struct {
			double u;
			DataRate Rc;
			uint32_t incStage;
		}hopState[IntHeader::maxHop];
	} hp;
	struct{
		uint32_t m_lastUpdateSeq;
		DataRate m_curRate;
		uint32_t m_incStage;
		uint64_t lastRtt;
		double rttDiff;
	} tmly;
	struct{
		uint32_t m_lastUpdateSeq;
		uint32_t m_caState;
		uint32_t m_highSeq; // when to exit cwr
		double m_alpha;
		uint32_t m_ecnCnt;
		uint32_t m_batchSizeOfAlpha;
	} dctcp;
	struct{
		uint32_t m_lastUpdateSeq;
		DataRate m_curRate;
		uint32_t m_incStage;
	}hpccPint;

	/***********
	 * methods
	 **********/
	
	static TypeId GetTypeId (void);
	RdmaQueuePair(uint16_t pg, Ipv4Address _sip, Ipv4Address _dip, uint16_t _sport, uint16_t _dport);
	void SetSize(uint64_t size);
	void SetWin(uint32_t win);
	void SetBaseRtt(uint64_t baseRtt);
	void SetVarWin(bool v);
	void SetAppNotifyCallback(Callback<void> notifyAppFinish);

	uint64_t GetBytesLeft();
	uint32_t GetOOODistance();
	uint32_t GetHash(void);
	void Acknowledge(uint32_t ack);
	uint64_t GetOnTheFly();
	uint32_t GetHPSFly();
	bool IsWinBound();
	uint64_t GetWin(); // window size calculated from m_rate
	bool IsFinished();
	uint64_t HpGetCurWin(); // window size calculated from hp.m_curRate, used by HPCC

	//jiange add
	void set_window_size(uint32_t window_size);
	uint32_t get_window_size();
};

class RdmaRxQueuePair : public Object { // Rx side queue pair
public:
	static uint32_t total_rxqp_no;
	struct ECNAccount{
		uint16_t qIndex;
		uint8_t ecnbits;
		uint16_t qfb;
		uint16_t total;

		ECNAccount() { memset(this, 0, sizeof(ECNAccount));}
	};
	ECNAccount m_ecn_source;
	uint32_t sip, dip;
	uint16_t sport, dport;
	uint16_t m_ipid;
	uint32_t ReceiverNextExpectedSeq;
	uint32_t rxqp_no;
	Time m_nackTimer;
	int32_t m_milestone_rx;
	uint32_t m_lastNACK;
	EventId QcnTimerEvent; // if destroy this rxQp, remember to cancel this timer

	/*HPS Receive Headers*/
	struct 
	{
		uint32_t next_expect_psn;
		
		std::vector<uint8_t> bitmap;
		uint32_t bitmap_head;
		uint32_t base_require_psn;
		uint32_t max_received_psn;
	} HPS;

	struct 
	{
		EventId QcnTimerPCNEvent;
		uint32_t ecn_count;
		uint32_t pkts_received;
		uint64_t cnp_inteval;
		double ecn_threshold;
		uint16_t first_udp;
		/* data */
	} PCN;
	

	static TypeId GetTypeId (void);
	RdmaRxQueuePair();
	uint32_t GetHash(void);
};

class RdmaQueuePairGroup : public Object {
public:
	std::vector<Ptr<RdmaQueuePair> > m_qps;
	//std::vector<Ptr<RdmaRxQueuePair> > m_rxQps;

	static TypeId GetTypeId (void);
	RdmaQueuePairGroup(void);
	uint32_t GetN(void);
	Ptr<RdmaQueuePair> Get(uint32_t idx);
	Ptr<RdmaQueuePair> operator[](uint32_t idx);
	void AddQp(Ptr<RdmaQueuePair> qp);
	//void AddRxQp(Ptr<RdmaRxQueuePair> rxQp);
	void Clear(void);
};

}

#endif /* RDMA_QUEUE_PAIR_H */
