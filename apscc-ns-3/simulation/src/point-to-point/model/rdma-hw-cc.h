#ifndef RDMA_HW_CC_H
#define RDMA_HW_CC_H

namespace ns3 {

typedef enum cc_ev_type {
	cc_ev_init = 0,
	cc_ev_resp = 1,
	cc_ev_pkt_drop = 2,
	cc_ev_timeout = 3
} cc_ev_type_s;

typedef struct cc_ev_resp_data {
    uint64_t rtt_det_send_time;
} cc_ev_resp_data_s;

typedef union cc_ev_spec {
	cc_ev_resp_data_s resp;
} cc_ev_spec_s;
typedef struct cc_ev_data {
    uint64_t ev_time;
	cc_ev_spec_s spec;
} cc_event_data_s;

typedef struct ctcclossy_algo {
	uint32_t cwnd; //congestion window, bytes
	uint32_t rc;  //current rate
	uint32_t base_rtt;
	uint32_t avg_rtt;
	uint32_t prev_rtt;
	uint32_t last_win_max;
	uint32_t time_len_K;
    uint32_t add_resp_pkts_cnt;
	uint32_t red_resp_pkts_cnt;
	uint32_t origin_point_win;
	uint32_t tcp_cwnd;
	uint64_t tcp_start_time;
	uint64_t last_time;
	uint64_t epoch_start_time;
	double rate;
} ctcclossy_algo_s;


}

#endif
