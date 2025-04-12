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

// #define DEBUG
#define DEBUG_QP_NUM 5
#define MIN_RATE 6103515
#define DCQCN_FAST_FACTOR 0.75
#define INIT_LINE_RATE_PERCENT 1
#define CUSTOM_RTT_CC_MODE 4   //4(HPS) 5(SWIFT) 6(PCN) 7(TIMELY) 8(WNDONLY) 9(SWIFT-FINE) 10 LossyCC 11 CTCC 12 HPCC

namespace ns3{
TypeId RdmaHw::GetTypeId (void)
{
	static TypeId tid = TypeId ("ns3::RdmaHw")
		.SetParent<Object> ()
		.AddAttribute("MinRate",
				"Minimum rate of a throttled flow",
				DataRateValue(DataRate("100Mb/s")),
				MakeDataRateAccessor(&RdmaHw::m_minRate),
				MakeDataRateChecker())
		.AddAttribute("Mtu",
				"Mtu.",
				UintegerValue(9000),
				MakeUintegerAccessor(&RdmaHw::m_mtu),
				MakeUintegerChecker<uint32_t>())
		.AddAttribute ("CcMode",
				"which mode of DCQCN is running",
				UintegerValue(0),
				MakeUintegerAccessor(&RdmaHw::m_cc_mode),
				MakeUintegerChecker<uint32_t>())
		.AddAttribute("NACK Generation Interval",
				"The NACK Generation interval",
				DoubleValue(500.0),
				MakeDoubleAccessor(&RdmaHw::m_nack_interval),
				MakeDoubleChecker<double>())
		.AddAttribute("L2ChunkSize",
				"Layer 2 chunk size. Disable chunk mode if equals to 0.",
				UintegerValue(0),
				MakeUintegerAccessor(&RdmaHw::m_chunk),
				MakeUintegerChecker<uint32_t>())
		.AddAttribute("L2AckInterval",
				"Layer 2 Ack intervals. Disable ack if equals to 0.",
				UintegerValue(0),
				MakeUintegerAccessor(&RdmaHw::m_ack_interval),
				MakeUintegerChecker<uint32_t>())
		.AddAttribute("L2BackToZero",
				"Layer 2 go back to zero transmission.",
				BooleanValue(false),
				MakeBooleanAccessor(&RdmaHw::m_backto0),
				MakeBooleanChecker())
		.AddAttribute("EwmaGain",
				"Control gain parameter which determines the level of rate decrease",
				DoubleValue(1.0 / 16),
				MakeDoubleAccessor(&RdmaHw::m_g),
				MakeDoubleChecker<double>())
		.AddAttribute ("RateOnFirstCnp",
				"the fraction of rate on first CNP",
				DoubleValue(1.0),
				MakeDoubleAccessor(&RdmaHw::m_rateOnFirstCNP),
				MakeDoubleChecker<double> ())
		.AddAttribute("ClampTargetRate",
				"Clamp target rate.",
				BooleanValue(false),
				MakeBooleanAccessor(&RdmaHw::m_EcnClampTgtRate),
				MakeBooleanChecker())
		.AddAttribute("RPTimer",
				"The rate increase timer at RP in microseconds",
				DoubleValue(1500.0),
				MakeDoubleAccessor(&RdmaHw::m_rpgTimeReset),
				MakeDoubleChecker<double>())
		.AddAttribute("RateDecreaseInterval",
				"The interval of rate decrease check",
				DoubleValue(4.0),
				MakeDoubleAccessor(&RdmaHw::m_rateDecreaseInterval),
				MakeDoubleChecker<double>())
		.AddAttribute("FastRecoveryTimes",
				"The rate increase timer at RP",
				UintegerValue(5),
				MakeUintegerAccessor(&RdmaHw::m_rpgThreshold),
				MakeUintegerChecker<uint32_t>())
		.AddAttribute("AlphaResumInterval",
				"The interval of resuming alpha",
				DoubleValue(55.0),
				MakeDoubleAccessor(&RdmaHw::m_alpha_resume_interval),
				MakeDoubleChecker<double>())
		.AddAttribute("RateAI",
				"Rate increment unit in AI period",
				DataRateValue(DataRate("5Mb/s")),
				MakeDataRateAccessor(&RdmaHw::m_rai),
				MakeDataRateChecker())
		.AddAttribute("RateHAI",
				"Rate increment unit in hyperactive AI period",
				DataRateValue(DataRate("50Mb/s")),
				MakeDataRateAccessor(&RdmaHw::m_rhai),
				MakeDataRateChecker())
		.AddAttribute("VarWin",
				"Use variable window size or not",
				BooleanValue(false),
				MakeBooleanAccessor(&RdmaHw::m_var_win),
				MakeBooleanChecker())
		.AddAttribute("FastReact",
				"Fast React to congestion feedback",
				BooleanValue(true),
				MakeBooleanAccessor(&RdmaHw::m_fast_react),
				MakeBooleanChecker())
		.AddAttribute("MiThresh",
				"Threshold of number of consecutive AI before MI",
				UintegerValue(5),
				MakeUintegerAccessor(&RdmaHw::m_miThresh),
				MakeUintegerChecker<uint32_t>())
		.AddAttribute("CtccRateDecreaseGamma",
				"CTCC's rate decrease factor gamma",
				DoubleValue(0.05),        
				MakeDoubleAccessor(&RdmaHw::m_gammaCtcc),
				MakeDoubleChecker<double>())
		.AddAttribute("CtccRateIncreaseDelta",
				"Rate increment unit in AI period",
				// DataRateValue(DataRate("25Mb/s")), // 1/128/16 * 50Gbps
				// DataRateValue(DataRate("50Mb/s")), // 1/128/16 * 100Gbps
				DataRateValue(DataRate("100Mb/s")), // 1/128/16 * 50Gbps
				// DataRateValue(DataRate("1600Mb/s")), // 1/128/16 * 50Gbps
				MakeDataRateAccessor(&RdmaHw::m_rateDeltaCtcc),
				MakeDataRateChecker())
		.AddAttribute("TargetUtil",
				"The Target Utilization of the bottleneck bandwidth, by default 95%",
				DoubleValue(0.95),
				MakeDoubleAccessor(&RdmaHw::m_targetUtil),
				MakeDoubleChecker<double>())
		.AddAttribute("UtilHigh",
				"The upper bound of Target Utilization of the bottleneck bandwidth, by default 98%",
				DoubleValue(0.98),
				MakeDoubleAccessor(&RdmaHw::m_utilHigh),
				MakeDoubleChecker<double>())
		.AddAttribute("RateBound",
				"Bound packet sending by rate, for test only",
				BooleanValue(true),
				MakeBooleanAccessor(&RdmaHw::m_rateBound),
				MakeBooleanChecker())
		.AddAttribute("MultiRate",
				"Maintain multiple rates in HPCC",
				BooleanValue(true),
				MakeBooleanAccessor(&RdmaHw::m_multipleRate),
				MakeBooleanChecker())
		.AddAttribute("SampleFeedback",
				"Whether sample feedback or not",
				BooleanValue(false),
				MakeBooleanAccessor(&RdmaHw::m_sampleFeedback),
				MakeBooleanChecker())
		.AddAttribute("TimelyAlpha",
				"Alpha of TIMELY",
				DoubleValue(0.875),
				MakeDoubleAccessor(&RdmaHw::m_tmly_alpha),
				MakeDoubleChecker<double>())
		.AddAttribute("TimelyBeta",
				"Beta of TIMELY",
				DoubleValue(0.8),
				MakeDoubleAccessor(&RdmaHw::m_tmly_beta),
				MakeDoubleChecker<double>())
		.AddAttribute("CtccTargetRttWeight",
				"CTCC's target rtt weight",
				DoubleValue(4600),        
				MakeDoubleAccessor(&RdmaHw::m_targetWeightCtcc),
				MakeDoubleChecker<double>())
		.AddAttribute("CtccTargetRttBias",
				"CTCC's target rtt bias",
				DoubleValue(-5000),        
				MakeDoubleAccessor(&RdmaHw::m_targetBiasCtcc),
				MakeDoubleChecker<double>())
		.AddAttribute("TimelyTLow",
				"TLow of TIMELY (ns)",
				UintegerValue(50000),
				MakeUintegerAccessor(&RdmaHw::m_tmly_TLow),
				MakeUintegerChecker<uint64_t>())
		.AddAttribute("TimelyTHigh",
				"THigh of TIMELY (ns)",
				UintegerValue(500000),
				MakeUintegerAccessor(&RdmaHw::m_tmly_THigh),
				MakeUintegerChecker<uint64_t>())
		.AddAttribute("TimelyMinRtt",
				"MinRtt of TIMELY (ns)",
				UintegerValue(20000),
				MakeUintegerAccessor(&RdmaHw::m_tmly_minRtt),
				MakeUintegerChecker<uint64_t>())
		.AddAttribute("DctcpRateAI",
				"DCTCP's Rate increment unit in AI period",
				DataRateValue(DataRate("1000Mb/s")),
				MakeDataRateAccessor(&RdmaHw::m_dctcp_rai),
				MakeDataRateChecker())
		.AddAttribute("PintSmplThresh",
				"PINT's sampling threshold in rand()%65536",
				UintegerValue(65536),
				MakeUintegerAccessor(&RdmaHw::pint_smpl_thresh),
				MakeUintegerChecker<uint32_t>())
		;
	return tid;
}

RdmaHw::RdmaHw(){
	m_minRate = 6103515;
}

void RdmaHw::SetNode(Ptr<Node> node){
	m_node = node;
}
void RdmaHw::Setup(QpCompleteCallback cb){
	for (uint32_t i = 0; i < m_nic.size(); i++){
		Ptr<QbbNetDevice> dev = m_nic[i].dev;
		if (dev == NULL)
			continue;
		// share data with NIC
		dev->m_rdmaEQ->m_qpGrp = m_nic[i].qpGrp;
		// setup callback
		dev->m_rdmaReceiveCb = MakeCallback(&RdmaHw::Receive, this);
		dev->m_rdmaLinkDownCb = MakeCallback(&RdmaHw::SetLinkDown, this);
		dev->m_rdmaPktSent = MakeCallback(&RdmaHw::PktSent, this);
		// config NIC
		dev->m_rdmaEQ->m_rdmaGetNxtPkt = MakeCallback(&RdmaHw::GetNxtPacket, this);
	}
	// setup qp complete callback
	m_qpCompleteCallback = cb;
}

uint32_t RdmaHw::GetNicIdxOfQp(Ptr<RdmaQueuePair> qp){
	auto &v = m_rtTable[qp->dip.Get()];
	if (v.size() > 0){
		return v[qp->GetHash() % v.size()];
	}else{
		NS_ASSERT_MSG(false, "We assume at least one NIC is alive");
	}
}
uint64_t RdmaHw::GetQpKey(uint32_t dip, uint16_t sport, uint16_t pg){
	return ((uint64_t)dip << 32) | ((uint64_t)sport << 16) | (uint64_t)pg;
}
Ptr<RdmaQueuePair> RdmaHw::GetQp(uint32_t dip, uint16_t sport, uint16_t pg){
	uint64_t key = GetQpKey(dip, sport, pg);
	auto it = m_qpMap.find(key);
	if (it != m_qpMap.end())
		return it->second;
	return NULL;
}
void RdmaHw::AddQueuePair(uint64_t size, uint16_t pg, Ipv4Address sip, Ipv4Address dip, uint16_t sport, uint16_t dport, uint32_t win, uint64_t baseRtt, Callback<void> notifyAppFinish){
	// create qp
	Ptr<RdmaQueuePair> qp = CreateObject<RdmaQueuePair>(pg, sip, dip, sport, dport);
	qp->SetSize(size);
	qp->SetWin(win);
	//std::cout<<"win "<<win<<std::endl;
	qp->SetBaseRtt(baseRtt);
	qp->SetVarWin(m_var_win);
	qp->SetAppNotifyCallback(notifyAppFinish);

	// add qp
	uint32_t nic_idx = GetNicIdxOfQp(qp);
	m_nic[nic_idx].qpGrp->AddQp(qp);
	uint64_t key = GetQpKey(dip.Get(), sport, pg);
	m_qpMap[key] = qp;

	// set init variables
	DataRate m_bps = m_nic[nic_idx].dev->GetDataRate();
	qp->m_rate = m_bps;
	qp->m_max_rate = m_bps;
	if (m_cc_mode == 1){
		qp->mlx.m_targetRate = m_bps;
	}else if (m_cc_mode == 3){
		qp->hp.m_curRate = m_bps;
		if (m_multipleRate){
			for (uint32_t i = 0; i < IntHeader::maxHop; i++)
				qp->hp.hopState[i].Rc = m_bps;
		}
	}else if (m_cc_mode == 7){
		qp->tmly.m_curRate = m_bps;
	}else if (m_cc_mode == 10){
		qp->hpccPint.m_curRate = m_bps;
	}

	qp->m_rate = qp->m_rate*INIT_LINE_RATE_PERCENT;

	// ctcclossy init event
	cc_event_data_s ev_data;
	ev_data.ev_time = Simulator::Now().GetTimeStep();
	ctcclossy_all_ev_deal(qp, cc_ev_init, ev_data);

	// Notify Nic
	m_nic[nic_idx].dev->NewQp(qp);
}

void RdmaHw::DeleteQueuePair(Ptr<RdmaQueuePair> qp){
	// remove qp from the m_qpMap
	uint64_t key = GetQpKey(qp->dip.Get(), qp->sport, qp->m_pg);
	m_qpMap.erase(key);
}

Ptr<RdmaRxQueuePair> RdmaHw::GetRxQp(uint32_t sip, uint32_t dip, uint16_t sport, uint16_t dport, uint16_t pg, bool create){
	uint64_t key = ((uint64_t)dip << 32) | ((uint64_t)pg << 16) | (uint64_t)dport;
	auto it = m_rxQpMap.find(key);
	if (it != m_rxQpMap.end())
		return it->second;
	if (create){
		// create new rx qp
		Ptr<RdmaRxQueuePair> q = CreateObject<RdmaRxQueuePair>();
		// init the qp
		q->sip = sip;
		q->dip = dip;
		q->sport = sport;
		q->dport = dport;
		q->m_ecn_source.qIndex = pg;
		// store in map
		m_rxQpMap[key] = q;
		return q;
	}
	return NULL;
}
uint32_t RdmaHw::GetNicIdxOfRxQp(Ptr<RdmaRxQueuePair> q){
	auto &v = m_rtTable[q->dip];
	if (v.size() > 0){
		return v[q->GetHash() % v.size()];
	}else{
		NS_ASSERT_MSG(false, "We assume at least one NIC is alive");
	}
}
void RdmaHw::DeleteRxQp(uint32_t dip, uint16_t pg, uint16_t dport){
	uint64_t key = ((uint64_t)dip << 32) | ((uint64_t)pg << 16) | (uint64_t)dport;
	if(m_cc_mode == 7 && CUSTOM_RTT_CC_MODE == 6)
		Simulator::Cancel(m_rxQpMap[key]->PCN.QcnTimerPCNEvent); 
	m_rxQpMap.erase(key);
}


void RdmaHw::PCNGenerateCNP(Ptr<RdmaRxQueuePair> rxQp){
	if(rxQp->PCN.pkts_received> 0){
		double ecn_ratio = (double)rxQp->PCN.ecn_count / (double)rxQp->PCN.pkts_received;
		DataRate receiveing_rate;
		if (ecn_ratio > rxQp->PCN.ecn_threshold){
			receiveing_rate = DataRate(1000000000 / rxQp->PCN.cnp_inteval * rxQp->PCN.pkts_received * m_mtu * 8);
		} else{
			receiveing_rate = 0;
		}
		qbbHeader seqh;
		seqh.SetAckPsn(0);
		seqh.SetPG(rxQp->m_ecn_source.qIndex);
		seqh.SetSport(rxQp->sport);
		seqh.SetSport(rxQp->sport);
		seqh.SetDport(rxQp->dport);
		seqh.SetReceiveingRate(receiveing_rate.GetBitRate());
		// seqh.SetIntHeader(ch.udp.ih);
		seqh.SetCnp();

		Ptr<Packet> newp = Create<Packet>(std::max(60-14-20-(int)seqh.GetSerializedSize(), 0));
		newp->AddHeader(seqh);

		Ipv4Header head;	// Prepare IPv4 header
		head.SetDestination(Ipv4Address(rxQp->dip));
		head.SetSource(Ipv4Address(rxQp->sip));
		head.SetProtocol(0xFC); //ack=0xFC nack=0xFD
		head.SetTtl(64);
		head.SetPayloadSize(newp->GetSize());
		head.SetIdentification(rxQp->m_ipid++);

		newp->AddHeader(head);
		AddHeader(newp, 0x800);	// Attach PPP header
		// send
		uint32_t nic_idx = GetNicIdxOfRxQp(rxQp);
		m_nic[nic_idx].dev->RdmaEnqueueHighPrioQ(newp);
		m_nic[nic_idx].dev->TriggerTransmit();
	}
	rxQp->PCN.ecn_count = 0;
	rxQp->PCN.pkts_received = 0;
	
	rxQp->PCN.QcnTimerPCNEvent = Simulator::Schedule(NanoSeconds(rxQp->PCN.cnp_inteval), &RdmaHw::PCNGenerateCNP, this, rxQp);
}

int RdmaHw::ReceiveUdp(Ptr<Packet> p, CustomHeader &ch){
	m_node->bandwidth_queue.push(Simulator::Now().GetInteger());
	
	//std::cout<<"get UDP packet " <<Simulator::Now().GetInteger()<<std::endl;
	uint8_t ecnbits = ch.GetIpv4EcnBits();

	uint32_t payload_size = p->GetSize() - ch.GetSerializedSize();

	// TODO find corresponding rx queue pair
	Ptr<RdmaRxQueuePair> rxQp = GetRxQp(ch.dip, ch.sip, ch.udp.dport, ch.udp.sport, ch.udp.pg, true);
	if (ecnbits != 0){
		rxQp->m_ecn_source.ecnbits |= ecnbits;
		rxQp->m_ecn_source.qfb++;
	}
	rxQp->m_ecn_source.total++;
	rxQp->m_milestone_rx = m_ack_interval;

	if(m_cc_mode ==7 && CUSTOM_RTT_CC_MODE == 6 ){
		if (rxQp->PCN.first_udp == 1) {
			rxQp->PCN.first_udp = 0;
			rxQp->PCN.QcnTimerPCNEvent = Simulator::Schedule(NanoSeconds(rxQp->PCN.cnp_inteval), &RdmaHw::PCNGenerateCNP, this, rxQp);
		} 
		if(ecnbits)
			rxQp->PCN.ecn_count +=1;
		rxQp->PCN.pkts_received +=1;
	} 
	/*HPS Protocal in Reveiver End*/
	// std::cout<<"Receiver Get PSN" << ch.udp.HPS.psn<<std::endl;
	int x = 1; //default ACK

	uint32_t receive_psn_index;
	uint32_t receive_psn = ch.udp.HPS.psn;

	NS_ASSERT_MSG(receive_psn < rxQp->HPS.base_require_psn + rxQp->HPS.bitmap.size(),"ERROR, distance too large");
	if(receive_psn >= rxQp->HPS.base_require_psn){
		receive_psn_index = receive_psn - rxQp->HPS.base_require_psn;
		rxQp->HPS.bitmap[(receive_psn_index + rxQp->HPS.bitmap_head)%rxQp->HPS.bitmap.size()] = 1;
		while(rxQp->HPS.bitmap[rxQp->HPS.bitmap_head]==1){
			rxQp->HPS.bitmap[rxQp->HPS.bitmap_head]=0;
			rxQp->HPS.base_require_psn +=1;
			rxQp->HPS.bitmap_head = (rxQp->HPS.bitmap_head+1)%rxQp->HPS.bitmap.size();
		}
	}
	if(rxQp->HPS.max_received_psn < receive_psn)
		rxQp->HPS.max_received_psn = receive_psn;

	// if((rxQp->HPS.max_received_psn +1 > rxQp->HPS.base_require_psn) && (rxQp->HPS.base_require_psn%128 == 0) && rxQp->rxqp_no < 10)
	// 	std::cout<<"qp:"<<rxQp->rxqp_no<<" max_receive_psn:"<<rxQp->HPS.max_received_psn<< " ooo_distance:"<<rxQp->HPS.max_received_psn - rxQp->HPS.base_require_psn<<std::endl;

	if (x == 1 || x == 2){ //generate ACK or NACK
		qbbHeader seqh;
		seqh.SetAckPsn(ch.udp.HPS.psn);
		// std::cout<<"max_received_psn:"<<rxQp->HPS.max_received_psn<< " base_require_psn:"<<rxQp->HPS.base_require_psn<< " ooo-distance:"<<rxQp->HPS.max_received_psn - rxQp->HPS.base_require_psn -1<<std::endl;
		seqh.SetOooDistance(rxQp->HPS.max_received_psn + 1 - rxQp->HPS.base_require_psn);
		seqh.SetPG(ch.udp.pg);
		seqh.SetSport(ch.udp.dport);
		seqh.SetDport(ch.udp.sport);
		seqh.SetIntHeader(ch.udp.ih);
		if (ecnbits)
			seqh.SetCnp();

		Ptr<Packet> newp = Create<Packet>(std::max(60-14-20-(int)seqh.GetSerializedSize(), 0));
		newp->AddHeader(seqh);

		Ipv4Header head;	// Prepare IPv4 header
		head.SetDestination(Ipv4Address(ch.sip));
		head.SetSource(Ipv4Address(ch.dip));
		head.SetProtocol(x == 1 ? 0xFC : 0xFD); //ack=0xFC nack=0xFD
		head.SetTtl(64);
		head.SetPayloadSize(newp->GetSize());
		head.SetIdentification(rxQp->m_ipid++);

		newp->AddHeader(head);
		AddHeader(newp, 0x800);	// Attach PPP header
		// send
		uint32_t nic_idx = GetNicIdxOfRxQp(rxQp);
		m_nic[nic_idx].dev->RdmaEnqueueHighPrioQ(newp);
		m_nic[nic_idx].dev->TriggerTransmit();
	}
	return 0;
}

int RdmaHw::ReceiveCnp(Ptr<Packet> p, CustomHeader &ch){
	// QCN on NIC
	// This is a Congestion signal
	// Then, extract data from the congestion packet.
	// We assume, without verify, the packet is destinated to me
	uint32_t qIndex = ch.cnp.qIndex;
	if (qIndex == 1){		//DCTCP
		std::cout << "TCP--ignore\n";
		return 0;
	}
	uint16_t udpport = ch.cnp.fid; // corresponds to the sport
	uint8_t ecnbits = ch.cnp.ecnBits;
	uint16_t qfb = ch.cnp.qfb;
	uint16_t total = ch.cnp.total;

	uint32_t i;
	// get qp
	Ptr<RdmaQueuePair> qp = GetQp(ch.sip, udpport, qIndex);
	if (qp == NULL)
		std::cout << "ERROR: QCN NIC cannot find the flow\n";
	// get nic
	uint32_t nic_idx = GetNicIdxOfQp(qp);
	Ptr<QbbNetDevice> dev = m_nic[nic_idx].dev;

	if (qp->m_rate == 0)			//lazy initialization	
	{
		qp->m_rate = dev->GetDataRate();
		if (m_cc_mode == 1){
			qp->mlx.m_targetRate = dev->GetDataRate();
		}else if (m_cc_mode == 3){
			qp->hp.m_curRate = dev->GetDataRate();
			if (m_multipleRate){
				for (uint32_t i = 0; i < IntHeader::maxHop; i++)
					qp->hp.hopState[i].Rc = dev->GetDataRate();
			}
		}else if (m_cc_mode == 7){
			qp->tmly.m_curRate = dev->GetDataRate();
		}else if (m_cc_mode == 10){
			qp->hpccPint.m_curRate = dev->GetDataRate();
		}
	}
	return 0;
}

int RdmaHw::ReceiveAck(Ptr<Packet> p, CustomHeader &ch){
	uint16_t qIndex = ch.ack.pg;
	uint16_t port = ch.ack.dport;
	uint32_t ack_psn = ch.ack.HPS.ack_psn; //这个是当前期望收到的下一个psn
	int i;
	Ptr<RdmaQueuePair> qp = GetQp(ch.sip, port, qIndex);
	if (qp == NULL){
		std::cout << "ERROR: " << "node:" << m_node->GetId() << ' ' << (ch.l3Prot == 0xFC ? "ACK" : "NACK") << " NIC cannot find the flow\n";
		return 0;
	}

	uint32_t nic_idx = GetNicIdxOfQp(qp);
	Ptr<QbbNetDevice> dev = m_nic[nic_idx].dev;

	//update ack, seq info for the related QP
	qp->Acknowledge(ack_psn);
	// if(qp->qp_no == 0){
	// 	std::cout<<"ack:psn_:"<<ack_psn<<" ";
	// 	for(auto i :qp->HPS.bitmap){
	// 		std::cout<<uint16_t(i);
	// 	}
	// 	std::cout<<std::endl;
	// }
	if (qp->IsFinished()){
		QpComplete(qp);
	}
	// if (ch.l3Prot == 0xFD) // NACK, currently go back n
	// 	RecoverQueue(qp);


	//Other HPS operations
	// std::cout<<"Sender Get ACKED PSN "<<ack_psn<<std::endl;

	//update congestion controls
	if(m_cc_mode == 7){
		if(CUSTOM_RTT_CC_MODE == 4)
			HandleAckHPS_CC_ACK(qp, p, ch,dev);
		else if (CUSTOM_RTT_CC_MODE == 5)
			HandleAckSwift(qp,p,ch,dev);
		else if (CUSTOM_RTT_CC_MODE == 6){
			HandleAckPCN(qp,p,ch,dev);
		}
		else if (CUSTOM_RTT_CC_MODE == 7){
			HandleAckTimely(qp,p, ch);
			dev->TriggerTransmit();
		}else if (CUSTOM_RTT_CC_MODE == 8){
			ChangeWindow(qp);
			dev->TriggerTransmit();
		}else if(CUSTOM_RTT_CC_MODE == 9 ){
			HandleAckSwift_finegrained(qp,p,ch,dev);
		} else if (CUSTOM_RTT_CC_MODE == 10){
			ctcclossy_recv_ack_handle(qp, p, ch, dev);
		} else if (CUSTOM_RTT_CC_MODE == 11){
			HandleAckCTCC(qp, p, ch);
			dev->TriggerTransmit();
		} else if (CUSTOM_RTT_CC_MODE == 12){
			HandleAckHp(qp, p, ch);
			dev->TriggerTransmit();
		} 
		
	}
	else if (m_cc_mode == 1){
		uint8_t cnp = (ch.ack.flags >> qbbHeader::FLAG_CNP) & 1;
		if (cnp){
			// std::cout<<"qp_no:"<<qp->qp_no<<" time:"<<Simulator::Now().GetTimeStep()<<" rate:"<<qp->m_rate<<" cnp"<<(uint16_t)cnp<<std::endl;
			cnp_received_mlx(qp);
		}
		dev->TriggerTransmit();
	} 
	return 0;
}

int RdmaHw::Receive(Ptr<Packet> p, CustomHeader &ch){
	if (ch.l3Prot == 0x11){ // UDP
		ReceiveUdp(p, ch);
	}else if (ch.l3Prot == 0xFF){ // CNP
		ReceiveCnp(p, ch);
	}else { //ACK or NACK
		ReceiveAck(p, ch);	
	}
	return 0;
}

int RdmaHw::ReceiverCheckSeq(uint32_t seq, Ptr<RdmaRxQueuePair> q, uint32_t size){
	uint32_t expected = q->HPS.next_expect_psn;
	if (seq == expected){
		q->ReceiverNextExpectedSeq +=1;
		if (q->ReceiverNextExpectedSeq >= q->m_milestone_rx){
			q->m_milestone_rx += m_ack_interval;
			return 1; //Generate ACK
		}else if (q->ReceiverNextExpectedSeq % m_chunk == 0){
			return 1;
		}else {
			return 5;
		}
	} else if (seq > expected) {
		// Generate NACK
		if (Simulator::Now() >= q->m_nackTimer || q->m_lastNACK != expected){
			q->m_nackTimer = Simulator::Now() + MicroSeconds(m_nack_interval);
			q->m_lastNACK = expected;
			if (m_backto0){
				q->ReceiverNextExpectedSeq = q->ReceiverNextExpectedSeq / m_chunk*m_chunk;
			}
			return 2;
		}else
			return 4;
	}else {
		// Duplicate. 
		return 3;
	}
}
void RdmaHw::AddHeader (Ptr<Packet> p, uint16_t protocolNumber){
	PppHeader ppp;
	ppp.SetProtocol (EtherToPpp (protocolNumber));
	p->AddHeader (ppp);
}
uint16_t RdmaHw::EtherToPpp (uint16_t proto){
	switch(proto){
		case 0x0800: return 0x0021;   //IPv4
		case 0x86DD: return 0x0057;   //IPv6
		default: NS_ASSERT_MSG (false, "PPP Protocol number not defined!");
	}
	return 0;
}

void RdmaHw::RecoverQueue(Ptr<RdmaQueuePair> qp){
	qp->HPS.next_send_psn = qp->HPS.tx_base_psn;
}

void RdmaHw::QpComplete(Ptr<RdmaQueuePair> qp){
	NS_ASSERT(!m_qpCompleteCallback.IsNull());
	if (m_cc_mode == 1){
		Simulator::Cancel(qp->mlx.m_eventUpdateAlpha);
		Simulator::Cancel(qp->mlx.m_eventDecreaseRate);
		Simulator::Cancel(qp->mlx.m_rpTimer);
	}

	// This callback will log info
	// It may also delete the rxQp on the receiver
	m_qpCompleteCallback(qp);

	qp->m_notifyAppFinish();

	RdmaQueuePair::qp_unfinished--;
	std::cout<<Simulator::Now().GetInteger()<<" "<<qp->startTime<<" "<<qp->qp_no<<" FINISH "<<std::endl;

	// delete the qp
	DeleteQueuePair(qp);
}

void RdmaHw::SetLinkDown(Ptr<QbbNetDevice> dev){
	printf("RdmaHw: node:%u a link down\n", m_node->GetId());
}

void RdmaHw::AddTableEntry(Ipv4Address &dstAddr, uint32_t intf_idx){
	uint32_t dip = dstAddr.Get();
	m_rtTable[dip].push_back(intf_idx);
}

void RdmaHw::ClearTable(){
	m_rtTable.clear();
}

void RdmaHw::RedistributeQp(){
	// clear old qpGrp
	for (uint32_t i = 0; i < m_nic.size(); i++){
		if (m_nic[i].dev == NULL)
			continue;
		m_nic[i].qpGrp->Clear();
	}

	// redistribute qp
	for (auto &it : m_qpMap){
		Ptr<RdmaQueuePair> qp = it.second;
		uint32_t nic_idx = GetNicIdxOfQp(qp);
		m_nic[nic_idx].qpGrp->AddQp(qp);
		// Notify Nic
		m_nic[nic_idx].dev->ReassignedQp(qp);
	}
}

Ptr<Packet> RdmaHw::GetNxtPacket(Ptr<RdmaQueuePair> qp){
	Ptr<Packet> p = Create<Packet> (m_mtu);
	// add SeqTsHeader
	SeqTsHeader seqTs;
	// seqTs.SetSeq (qp->snd_nxt);
	if(!qp->HPS.quick_send_queue.empty()){
		seqTs.SetPsn(qp->HPS.quick_send_queue.front().psn);
		qp->HPS.quick_send_queue.pop();
	} else{
		seqTs.SetPsn (qp->HPS.next_send_psn);
		qp->HPS.next_send_psn +=1;
	}
	
	seqTs.SetPG (qp->m_pg);
	p->AddHeader (seqTs);
	// add udp header
	UdpHeader udpHeader;
	udpHeader.SetDestinationPort (qp->dport);
	udpHeader.SetSourcePort (qp->sport);
	p->AddHeader (udpHeader);
	// add ipv4 header
	Ipv4Header ipHeader;
	ipHeader.SetSource (qp->sip);
	ipHeader.SetDestination (qp->dip);
	ipHeader.SetProtocol (0x11);
	ipHeader.SetPayloadSize (p->GetSize());
	ipHeader.SetTtl (64);
	ipHeader.SetTos (0);
	ipHeader.SetIdentification (qp->m_ipid);
	p->AddHeader(ipHeader);
	// add ppp header
	PppHeader ppp;
	ppp.SetProtocol (0x0021); // EtherToPpp(0x800), see point-to-point-net-device.cc
	p->AddHeader (ppp);

	// update state
	qp->m_ipid++;
	
	// return
	return p;
}

void RdmaHw::PktSent(Ptr<RdmaQueuePair> qp, Ptr<Packet> pkt, Time interframeGap){
	qp->lastPktSize = pkt->GetSize();
	UpdateNextAvail(qp, interframeGap, pkt->GetSize());
}

void RdmaHw::UpdateNextAvail(Ptr<RdmaQueuePair> qp, Time interframeGap, uint32_t pkt_size){
	Time sendingTime;
	//window for swift, hps, windowonly
	if(m_cc_mode == 7 && ( CUSTOM_RTT_CC_MODE == 4 || CUSTOM_RTT_CC_MODE == 5 || CUSTOM_RTT_CC_MODE == 8 || CUSTOM_RTT_CC_MODE == 9 || CUSTOM_RTT_CC_MODE == 10))
	{
		//if the quick queue is not empty
		if(!qp->HPS.quick_send_queue.empty()){
			qp->m_nextAvail = interframeGap+Simulator::Now();
		}
		else{
			double should_send_temp = qp->HPS.infilight_pkts_limitation - (qp->HPS.next_send_psn -  qp->HPS.tx_base_psn);
			//if there is too much packet in the link, set the avail large and stop sending packets
			if(should_send_temp<=0)
				qp->m_nextAvail = interframeGap+Simulator::Now() + Seconds(10000.0);
			else if(should_send_temp < 1)
				qp->m_nextAvail = interframeGap+Simulator::Now()+ NanoSeconds(qp->HPS.rtt / qp->HPS.infilight_pkts_limitation);
			else
				qp->m_nextAvail = interframeGap+Simulator::Now();	
		}
	}
	else{
		if (m_rateBound)
			sendingTime = interframeGap + Seconds(qp->m_rate.CalculateTxTime(pkt_size));
		else
			sendingTime = interframeGap + Seconds(qp->m_max_rate.CalculateTxTime(pkt_size));
		qp->m_nextAvail = Simulator::Now() + sendingTime;
	}
}

void RdmaHw::ChangeRate(Ptr<RdmaQueuePair> qp, DataRate new_rate){
	#if 1
	Time sendingTime = Seconds(qp->m_rate.CalculateTxTime(qp->lastPktSize));
	Time new_sendintTime = Seconds(new_rate.CalculateTxTime(qp->lastPktSize));
	qp->m_nextAvail = qp->m_nextAvail + new_sendintTime - sendingTime;
	// update nic's next avail event
	uint32_t nic_idx = GetNicIdxOfQp(qp);
	m_nic[nic_idx].dev->UpdateNextAvail(qp->m_nextAvail);
	#endif
	// change to new rate
	qp->m_rate = new_rate;
}

#define PRINT_LOG 0
/******************************
 * Mellanox's version of DCQCN
 *****************************/
void RdmaHw::UpdateAlphaMlx(Ptr<RdmaQueuePair> q){
	#if PRINT_LOG
	//std::cout << Simulator::Now() << " alpha update:" << m_node->GetId() << ' ' << q->mlx.m_alpha << ' ' << (int)q->mlx.m_alpha_cnp_arrived << '\n';
	//printf("%lu alpha update: %08x %08x %u %u %.6lf->", Simulator::Now().GetTimeStep(), q->sip.Get(), q->dip.Get(), q->sport, q->dport, q->mlx.m_alpha);
	#endif
	if (q->mlx.m_alpha_cnp_arrived){
		q->mlx.m_alpha = (1 - m_g)*q->mlx.m_alpha + m_g; 	//binary feedback
	}else {
		q->mlx.m_alpha = (1 - m_g)*q->mlx.m_alpha; 	//binary feedback
	}
	#if PRINT_LOG
	//printf("%.6lf\n", q->mlx.m_alpha);
	#endif
	q->mlx.m_alpha_cnp_arrived = false; // clear the CNP_arrived bit
	ScheduleUpdateAlphaMlx(q);
}
void RdmaHw::ScheduleUpdateAlphaMlx(Ptr<RdmaQueuePair> q){
	q->mlx.m_eventUpdateAlpha = Simulator::Schedule(MicroSeconds(m_alpha_resume_interval), &RdmaHw::UpdateAlphaMlx, this, q);
}

void RdmaHw::cnp_received_mlx(Ptr<RdmaQueuePair> q){
	q->mlx.m_alpha_cnp_arrived = true; // set CNP_arrived bit for alpha update
	q->mlx.m_decrease_cnp_arrived = true; // set CNP_arrived bit for rate decrease
	if (q->mlx.m_first_cnp){
		// init alpha
		q->mlx.m_alpha = 1;
		q->mlx.m_alpha_cnp_arrived = false;
		// schedule alpha update
		ScheduleUpdateAlphaMlx(q);
		// schedule rate decrease
		ScheduleDecreaseRateMlx(q, 1); // add 1 ns to make sure rate decrease is after alpha update
		// set rate on first CNP
		q->mlx.m_targetRate = q->m_rate = m_rateOnFirstCNP * q->m_rate;
		q->mlx.m_first_cnp = false;
	}
}

void RdmaHw::HandleAckPCN(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch,Ptr<QbbNetDevice> dev){
	uint32_t ack_psn = ch.ack.HPS.ack_psn; 
	uint8_t cnp = (ch.ack.flags >> qbbHeader::FLAG_CNP) & 1;
	DataRate receive_rate(ch.ack.PCN.receiveing_rate);
	//ack_psn=0 use to identify the CNP packet
	if(ack_psn == 0)
	{
#ifdef DEBUG
		std::cout<<"ack_psn:"<< ack_psn<<" r_rate:"<<(double)receive_rate.GetBitRate() / 1000000000.0;
#endif
		if(receive_rate.GetBitRate()!=0){
			receive_rate = DataRate(receive_rate.GetBitRate()*(1.0- qp->PCN.w_min));
			qp->m_rate = receive_rate<qp->m_rate?receive_rate:qp->m_rate;
			qp->PCN.w = qp->PCN.w_min;
		} else{
			qp->m_rate = DataRate(qp->m_rate.GetBitRate() * (1-qp->PCN.w) + qp->m_max_rate.GetBitRate() *(qp->PCN.w));
			qp->PCN.w = qp->PCN.w*(1-qp->PCN.w) + qp->PCN.w_max*qp->PCN.w;
		}

		if(qp->m_rate < DataRate(qp->PCN.min_rate))
			qp->m_rate = DataRate(qp->PCN.min_rate);
		if(qp->m_rate > qp->m_max_rate)
			qp->m_rate = qp->m_max_rate;
#ifdef DEBUG
		std::cout<<" m_rate:"<< (double)qp->m_rate.GetBitRate()/1000000000<<std::endl;
#endif
	}

	ChangeRate(qp, qp->m_rate);
	dev->TriggerTransmit();
}

void RdmaHw::CheckRateDecreaseMlx(Ptr<RdmaQueuePair> q){
	ScheduleDecreaseRateMlx(q, 0);
	if (q->mlx.m_decrease_cnp_arrived){
		#if PRINT_LOG
		printf("%lu rate dec: %08x %08x %u %u (%0.3lf %.3lf)->", Simulator::Now().GetTimeStep(), q->sip.Get(), q->dip.Get(), q->sport, q->dport, q->mlx.m_targetRate.GetBitRate() * 1e-9, q->m_rate.GetBitRate() * 1e-9);
		#endif
		bool clamp = true;
		if (!m_EcnClampTgtRate){
			if (q->mlx.m_rpTimeStage == 0)
				clamp = false;
		}
		if (clamp)
			q->mlx.m_targetRate = q->m_rate;
		q->m_rate = std::max(m_minRate, q->m_rate * (1 - q->mlx.m_alpha / 2));
		// reset rate increase related things
		q->mlx.m_rpTimeStage = 0;
		q->mlx.m_decrease_cnp_arrived = false;
		Simulator::Cancel(q->mlx.m_rpTimer);
		q->mlx.m_rpTimer = Simulator::Schedule(MicroSeconds(m_rpgTimeReset), &RdmaHw::RateIncEventTimerMlx, this, q);
		#if PRINT_LOG
		printf("(%.3lf %.3lf)\n", q->mlx.m_targetRate.GetBitRate() * 1e-9, q->m_rate.GetBitRate() * 1e-9);
		#endif
	}
}
void RdmaHw::ScheduleDecreaseRateMlx(Ptr<RdmaQueuePair> q, uint32_t delta){
	q->mlx.m_eventDecreaseRate = Simulator::Schedule(MicroSeconds(m_rateDecreaseInterval) + NanoSeconds(delta), &RdmaHw::CheckRateDecreaseMlx, this, q);
}

void RdmaHw::RateIncEventTimerMlx(Ptr<RdmaQueuePair> q){
	q->mlx.m_rpTimer = Simulator::Schedule(MicroSeconds(m_rpgTimeReset), &RdmaHw::RateIncEventTimerMlx, this, q);
	RateIncEventMlx(q);
	q->mlx.m_rpTimeStage++;
}
void RdmaHw::RateIncEventMlx(Ptr<RdmaQueuePair> q){
	
	// check which increase phase: fast recovery, active increase, hyper increase
	if (q->mlx.m_rpTimeStage < m_rpgThreshold){ // fast recovery
		FastRecoveryMlx(q);
	}else if (q->mlx.m_rpTimeStage == m_rpgThreshold){ // active increase
		ActiveIncreaseMlx(q);
	}else { // hyper increase
		HyperIncreaseMlx(q);
	}
	ChangeRate(q, q->m_rate);
	// std::cout<<q->qp_no <<" RateIncEventMlx "<< q->m_rate/1000000000<<std::endl;
}

void RdmaHw::FastRecoveryMlx(Ptr<RdmaQueuePair> q){
	#if PRINT_LOG
	printf("%lu fast recovery: %08x %08x %u %u (%0.3lf %.3lf)->", Simulator::Now().GetTimeStep(), q->sip.Get(), q->dip.Get(), q->sport, q->dport, q->mlx.m_targetRate.GetBitRate() * 1e-9, q->m_rate.GetBitRate() * 1e-9);
	#endif
	q->m_rate = (q->m_rate * DCQCN_FAST_FACTOR) + (q->mlx.m_targetRate *(1-DCQCN_FAST_FACTOR));
	#if PRINT_LOG
	printf("(%.3lf %.3lf)\n", q->mlx.m_targetRate.GetBitRate() * 1e-9, q->m_rate.GetBitRate() * 1e-9);
	#endif
}
void RdmaHw::ActiveIncreaseMlx(Ptr<RdmaQueuePair> q){
	#if PRINT_LOG
	printf("%lu active inc: %08x %08x %u %u (%0.3lf %.3lf)->", Simulator::Now().GetTimeStep(), q->sip.Get(), q->dip.Get(), q->sport, q->dport, q->mlx.m_targetRate.GetBitRate() * 1e-9, q->m_rate.GetBitRate() * 1e-9);
	#endif
	// get NIC
	uint32_t nic_idx = GetNicIdxOfQp(q);
	Ptr<QbbNetDevice> dev = m_nic[nic_idx].dev;
	// increate rate
	q->mlx.m_targetRate += m_rai;
	if (q->mlx.m_targetRate > dev->GetDataRate())
		q->mlx.m_targetRate = dev->GetDataRate();
	q->m_rate = q->m_rate * DCQCN_FAST_FACTOR + q->mlx.m_targetRate *(1-DCQCN_FAST_FACTOR);
	#if PRINT_LOG
	printf("(%.3lf %.3lf)\n", q->mlx.m_targetRate.GetBitRate() * 1e-9, q->m_rate.GetBitRate() * 1e-9);
	#endif
}
void RdmaHw::HyperIncreaseMlx(Ptr<RdmaQueuePair> q){
	#if PRINT_LOG
	printf("%lu hyper inc: %08x %08x %u %u (%0.3lf %.3lf)->", Simulator::Now().GetTimeStep(), q->sip.Get(), q->dip.Get(), q->sport, q->dport, q->mlx.m_targetRate.GetBitRate() * 1e-9, q->m_rate.GetBitRate() * 1e-9);
	#endif
	// get NIC
	uint32_t nic_idx = GetNicIdxOfQp(q);
	Ptr<QbbNetDevice> dev = m_nic[nic_idx].dev;
	// increate rate
	q->mlx.m_targetRate += m_rhai;
	if (q->mlx.m_targetRate > dev->GetDataRate())
		q->mlx.m_targetRate = dev->GetDataRate();
	q->m_rate = q->m_rate * DCQCN_FAST_FACTOR + q->mlx.m_targetRate *(1-DCQCN_FAST_FACTOR);
	#if PRINT_LOG
	printf("(%.3lf %.3lf)\n", q->mlx.m_targetRate.GetBitRate() * 1e-9, q->m_rate.GetBitRate() * 1e-9);
	#endif
}

/***********************
 * High Precision CC
 ***********************/
void RdmaHw::HandleAckHp(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch){
	uint32_t ack_seq = ch.ack.seq;
	//std::cout<<Simulator::Now().GetInteger()<<" "<<qp->m_ipid<<" "<< ack_seq<<std::endl;
	// update rate
	if (ack_seq > qp->hp.m_lastUpdateSeq){ // if full RTT feedback is ready, do full update
		UpdateRateHp(qp, p, ch, false);
	}else{ // do fast react
		FastReactHp(qp, p, ch);
	}
}

// 定义一个函数，返回 [min, max] 范围内的随机整数
int RdmaHw::getRandomInteger(int min, int max) {
	// 使用系统时钟作为随机数种子
	std::mt19937 engine(static_cast<unsigned long>(std::chrono::system_clock::now().time_since_epoch().count()));
	// 定义一个均匀整数分布
	std::uniform_int_distribution<int> distribution(min, max);
	// 返回分布生成的随机整数
	return distribution(engine);
}

void RdmaHw::UpdateRateHp(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch, bool fast_react){
	uint32_t next_seq = qp->snd_nxt;
	bool print = !fast_react || true;
	if (qp->hp.m_lastUpdateSeq == 0){ // first RTT
		qp->hp.m_lastUpdateSeq = next_seq;
		// store INT
		IntHeader &ih = ch.ack.ih;
		NS_ASSERT(ih.nhop <= IntHeader::maxHop);
		for (uint32_t i = 0; i < ih.nhop; i++)
			qp->hp.hop[i] = ih.hop[i];
		#if PRINT_LOG
		if (print){
			printf("%lu %s %08x %08x %u %u [%u,%u,%u]", Simulator::Now().GetTimeStep(), fast_react? "fast" : "update", qp->sip.Get(), qp->dip.Get(), qp->sport, qp->dport, qp->hp.m_lastUpdateSeq, ch.ack.seq, next_seq);
			for (uint32_t i = 0; i < ih.nhop; i++)
				printf(" %u %lu %lu", ih.hop[i].GetQlen(), ih.hop[i].GetBytes(), ih.hop[i].GetTime());
			printf("\n");
		}
		#endif
	}else {
		// check packet INT
		IntHeader &ih = ch.ack.ih;
		if (ih.nhop <= IntHeader::maxHop){
			double max_c = 0;
			bool inStable = false;
			#if PRINT_LOG
			if (print)
				printf("%lu %s %08x %08x %u %u [%u,%u,%u]", Simulator::Now().GetTimeStep(), fast_react? "fast" : "update", qp->sip.Get(), qp->dip.Get(), qp->sport, qp->dport, qp->hp.m_lastUpdateSeq, ch.ack.seq, next_seq);
			#endif
			// check each hop
			double U = 0;
			uint64_t dt = 0;
			bool updated[IntHeader::maxHop] = {false}, updated_any = false;
			NS_ASSERT(ih.nhop <= IntHeader::maxHop);
			for (uint32_t i = 0; i < ih.nhop; i++){
				if (m_sampleFeedback){
					if (ih.hop[i].GetQlen() == 0 && fast_react)
						continue;
				}
				updated[i] = updated_any = true;
				#if PRINT_LOG
				if (print)
					printf(" %u(%u) %lu(%lu) %lu(%lu)", ih.hop[i].GetQlen(), qp->hp.hop[i].GetQlen(), ih.hop[i].GetBytes(), qp->hp.hop[i].GetBytes(), ih.hop[i].GetTime(), qp->hp.hop[i].GetTime());
				#endif
				uint64_t tau = ih.hop[i].GetTimeDelta(qp->hp.hop[i]);;
				double duration = tau * 1e-9;
				double txRate = (ih.hop[i].GetBytesDelta(qp->hp.hop[i])) * 8 / duration;
				double u = txRate / ih.hop[i].GetLineRate() + (double)std::min(ih.hop[i].GetQlen(), qp->hp.hop[i].GetQlen()) * qp->m_max_rate.GetBitRate() / ih.hop[i].GetLineRate() /qp->m_win;
				#if PRINT_LOG
				if (print)
					printf(" %.3lf %.3lf", txRate, u);
				#endif
				if (!m_multipleRate){
					// for aggregate (single R)
					if (u > U){
						U = u;
						dt = tau;
					}
				}else {
					// for per hop (per hop R)
					if (tau > qp->m_baseRtt)
						tau = qp->m_baseRtt;
					qp->hp.hopState[i].u = (qp->hp.hopState[i].u * (qp->m_baseRtt - tau) + u * tau) / double(qp->m_baseRtt);
				}
				qp->hp.hop[i] = ih.hop[i];
			}

			DataRate new_rate;
			int32_t new_incStage;
			DataRate new_rate_per_hop[IntHeader::maxHop];
			int32_t new_incStage_per_hop[IntHeader::maxHop];
			if (!m_multipleRate){
				// for aggregate (single R)
				if (updated_any){
					if (dt > qp->m_baseRtt)
						dt = qp->m_baseRtt;
					qp->hp.u = (qp->hp.u * (qp->m_baseRtt - dt) + U * dt) / double(qp->m_baseRtt);
					max_c = qp->hp.u / m_targetUtil;

					if (max_c >= 1 || qp->hp.m_incStage >= m_miThresh){
						new_rate = qp->hp.m_curRate / max_c + m_rai;
						new_incStage = 0;
					}else{
						new_rate = qp->hp.m_curRate + m_rai;
						new_incStage = qp->hp.m_incStage+1;
					}
					if (new_rate < m_minRate)
						new_rate = m_minRate;
					if (new_rate > qp->m_max_rate)
						new_rate = qp->m_max_rate;
					#if PRINT_LOG
					if (print)
						printf(" u=%.6lf U=%.3lf dt=%u max_c=%.3lf", qp->hp.u, U, dt, max_c);
					#endif
					#if PRINT_LOG
					if (print)
						printf(" rate:%.3lf->%.3lf\n", qp->hp.m_curRate.GetBitRate()*1e-9, new_rate.GetBitRate()*1e-9);
					#endif
				}
			}else{
				// for per hop (per hop R)
				new_rate = qp->m_max_rate;
				for (uint32_t i = 0; i < ih.nhop; i++){
					if (updated[i]){
						double c = qp->hp.hopState[i].u / m_targetUtil;
						if (c >= 1 || qp->hp.hopState[i].incStage >= m_miThresh){
							new_rate_per_hop[i] = qp->hp.hopState[i].Rc / c + m_rai;
							new_incStage_per_hop[i] = 0;
						}else{
							new_rate_per_hop[i] = qp->hp.hopState[i].Rc + m_rai;
							new_incStage_per_hop[i] = qp->hp.hopState[i].incStage+1;
						}
						// bound rate
						if (new_rate_per_hop[i] < m_minRate)
							new_rate_per_hop[i] = m_minRate;
						if (new_rate_per_hop[i] > qp->m_max_rate)
							new_rate_per_hop[i] = qp->m_max_rate;
						// find min new_rate
						if (new_rate_per_hop[i] < new_rate)
							new_rate = new_rate_per_hop[i];
						#if PRINT_LOG
						if (print)
							printf(" [%u]u=%.6lf c=%.3lf", i, qp->hp.hopState[i].u, c);
						#endif
						#if PRINT_LOG
						if (print)
							printf(" %.3lf->%.3lf", qp->hp.hopState[i].Rc.GetBitRate()*1e-9, new_rate.GetBitRate()*1e-9);
						#endif
					}else{
						if (qp->hp.hopState[i].Rc < new_rate)
							new_rate = qp->hp.hopState[i].Rc;
					}
				}
				#if PRINT_LOG
				printf("\n");
				#endif
			}
			if (updated_any)
				ChangeRate(qp, new_rate);
			if (!fast_react){
				if (updated_any){
					qp->hp.m_curRate = new_rate;
					qp->hp.m_incStage = new_incStage;
				}
				if (m_multipleRate){
					// for per hop (per hop R)
					for (uint32_t i = 0; i < ih.nhop; i++){
						if (updated[i]){
							qp->hp.hopState[i].Rc = new_rate_per_hop[i];
							qp->hp.hopState[i].incStage = new_incStage_per_hop[i];
						}
					}
				}
			}
		}
		if (!fast_react){
			if (next_seq > qp->hp.m_lastUpdateSeq)
				qp->hp.m_lastUpdateSeq = next_seq; //+ rand() % 2 * m_mtu;
		}
	}
}

void RdmaHw::FastReactHp(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch){
	if (m_fast_react)
		UpdateRateHp(qp, p, ch, true);
}

/**********************
 * TIMELY
 *********************/
void RdmaHw::HandleAckTimely(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch){
	uint32_t ack_seq = ch.ack.seq;
	uint64_t rtt = Simulator::Now().GetTimeStep() - ch.ack.ih.ts;
	// update rate
	if (ack_seq > qp->tmly.m_lastUpdateSeq){ // if full RTT feedback is ready, do full update
		UpdateRateTimely(qp, p, ch, false);
	}else{ // do fast react
		FastReactTimely(qp, p, ch);
	}
	ChangeRate(qp,qp->m_rate);
}


#include <cmath>

double sigmoid(double x) {
    return 1 / (1 + exp(-x));
}

void RdmaHw::UpdateRateCtccC(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch){
	m_minRate = MIN_RATE;
	uint64_t timestamp = Simulator::Now().GetTimeStep();
	uint64_t new_rtt =  Simulator::Now().GetTimeStep() - ch.ack.ih.ts; 
	int64_t update_ts_diff = (timestamp - qp->CTCC.m_prev_ts)/2;  // 使用receive time 差值计算梯度
	int64_t new_rtt_diff = (int64_t)new_rtt - (int64_t)qp->CTCC.m_rtt;

	double rtt_power = 0;
	if (new_rtt < qp->m_baseRtt){
		qp->m_baseRtt = new_rtt;
	}
	if (qp->CTCC.m_rtt ){
		rtt_power = (int64_t)new_rtt + new_rtt_diff * (int64_t)(new_rtt - qp->m_baseRtt) / update_ts_diff;
	} else {
		rtt_power = new_rtt;
	}

    double rtt_target = qp->m_baseRtt +6000 + 12000 / sqrt(qp->m_rate.GetBitRate()/1e9);
	double max_target = qp->m_baseRtt +6000 + 12000 / sqrt(m_minRate.GetBitRate()/1e9);
	rtt_target = rtt_target > max_target?max_target:rtt_target;

#ifdef DEBUG
	if(qp->qp_no < DEBUG_QP_NUM)
		std::cout<<"flow_id:"<<qp->qp_no<< " rate:"<<qp->m_rate.GetBitRate()/1000000<<"mpbs rtt:"<<new_rtt<< " target:"<<rtt_target<<" power:"<<rtt_power;
#endif
	if (rtt_power >= rtt_target){
#ifdef DEBUG
	if(qp->qp_no < DEBUG_QP_NUM)
		std::cout<<" dec_f:"<<m_gammaCtcc*(rtt_power - rtt_target)/rtt_power;
#endif	
		qp->m_rate = std::max(m_minRate, qp->m_rate * (1 - m_gammaCtcc*(rtt_power - rtt_target)/(rtt_power- qp->m_baseRtt)));  // MD
		qp->CTCC.m_increase_stage_cnt = 0;
	} else {
		qp->CTCC.m_increase_stage_cnt = std::min(qp->CTCC.m_increase_stage_cnt + 1, (uint32_t)10);
		
		qp->m_rate = std::min(qp->m_max_rate, qp->m_rate+ std::min(qp->m_rate/4, m_rateDeltaCtcc* qp->CTCC.m_increase_stage_cnt));  	
	}
#ifdef DEBUG
if(qp->qp_no < DEBUG_QP_NUM)
	std::cout<<" Crate:"<<qp->m_rate.GetBitRate()/1000000<<"mbps"<<std::endl;
#endif
	qp->CTCC.m_rtt = new_rtt;
	qp->CTCC.m_prev_ts = timestamp;
}

void RdmaHw::HandleAckCTCC(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch){
    /***********************************************************************
	 每发送4K数据探测RTT，两次调速间隔至少发送m_updateRateBytesIntervalCtcc数据
	 ***********************************************************************/
	uint32_t sent_bytes_from_last_rtt_meas = ch.udp.seq - qp->CTCC.m_rtt_meas_seq;
	uint32_t next_seq = qp->snd_nxt;
	uint32_t sent_bytes_from_last_update = next_seq - qp->CTCC.m_first_seq_in_rtt_meas;
	// uint64_t time_from_last_update = Simulator::Now().GetTimeStep() - qp->CTCC.m_prev_ts;
	uint32_t m_rttReqBytesIntervalCtcc = 9000; //最小探测间隔为发送4KB数据
	m_updateRateBytesIntervalCtcc = 9000;
	// uint64_t m_rttReqInterval = 0;
	// printf("srcip:%lu, udp_seq:%lu, sent_bytes_from_last_rtt_meas:%u\n",qp->sip.Get(), ch.udp.seq, sent_bytes_from_last_rtt_meas);
	if (ch.udp.seq<=m_mtu || sent_bytes_from_last_rtt_meas >= m_rttReqBytesIntervalCtcc){
		if (sent_bytes_from_last_update >= m_updateRateBytesIntervalCtcc){  //seq为已发送数据量 
			UpdateRateCtccC(qp, p, ch);
			qp->CTCC.m_first_seq_in_rtt_meas = next_seq;
		}
	    qp->CTCC.m_rtt_meas_seq = ch.udp.seq;
		qp->CTCC.m_flag_was_cnp_during_cur_rtt = false;
	}
	ChangeRate(qp,qp->m_rate);
}

void RdmaHw::HandleAckHPS_CC_ACK(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch, Ptr<QbbNetDevice> dev){
	uint8_t cnp = (ch.ack.flags >> qbbHeader::FLAG_CNP) & 1;
	uint32_t ack_psn = ch.ack.HPS.ack_psn; 
	// uint16_t ooo_distance = ch.ack.HPS.ooo_distance; 
	uint64_t rtt = Simulator::Now().GetTimeStep() - ch.ack.ih.ts;
	uint64_t time_from_last_update;

	uint64_t rate_change_window_rtt;
	double temp_ooo_degree;
	double temp_add_step;
	double temp_ecn_threshold;

	qp->HPS.rtt = qp->HPS.rtt*qp->HPS.rtt_ewma + rtt*(1-qp->HPS.rtt_ewma);
	if (qp->HPS.base_rtt > rtt)
		qp->HPS.base_rtt = rtt;
	qp->HPS.ooo_acc += qp->HPS.largest_acked_psn + 1 - qp->HPS.tx_base_psn;
	qp->HPS.ack_count +=1; 
	qp->HPS.ack_number +=1;
	if (cnp)
		qp->HPS.ecn_number +=1;
	time_from_last_update = Simulator::Now().GetTimeStep() -  qp->HPS.last_update_time;
	
	if(time_from_last_update > qp->HPS.rtt){
		temp_ooo_degree = (double)(qp->HPS.ooo_acc * qp->HPS.ooodegreee_factor)/ (double)(qp->HPS.infilight_pkts_max * qp->HPS.ack_count);
		if (temp_ooo_degree > 1.0)
			temp_ooo_degree = 0.99;

		qp->HPS.ecn_ratio =(double)qp->HPS.ecn_number/(double)qp->HPS.ack_number;
		temp_ecn_threshold = Get_ecn_threshold_hps(qp);

#ifdef DEBUG
		if(qp->qp_no<DEBUG_QP_NUM){
			std::cout<<"qp:"<<qp->qp_no<<"time:"<<Simulator::Now().GetTimeStep()<<" rate:"<<qp->GetHPSFly()*m_mtu*8/qp->HPS.rtt<<" psn:"<< ack_psn<< " rtt:"<<rtt<<" ecn_r:"<<qp->HPS.ecn_ratio<<" ecn_t:"<<temp_ecn_threshold<<" fly:"<<qp->GetHPSFly();
		}
#endif
		if (qp->HPS.ecn_ratio >= temp_ecn_threshold){
			//case for last-hop
			rate_change_window_rtt = qp->HPS.base_rtt_percentage*qp->HPS.base_rtt + (1-qp->HPS.base_rtt_percentage)*qp->HPS.rtt;
			double quick_result = (double) (qp->HPS.ack_number) * rate_change_window_rtt/(double)(time_from_last_update)* qp->HPS.fairness_fac;
			// double quick_result = (double) (qp->HPS.ack_number) * qp->HPS.fairness_fac;
			
			//case for normal
			double normal_result = (1.0 - qp->HPS.decrease_factor* ( (qp->HPS.ecn_ratio - temp_ecn_threshold) / qp->HPS.ecn_ratio))*qp->HPS.infilight_pkts_limitation;
			
			double combine_result = quick_result * (1-temp_ooo_degree) + normal_result * temp_ooo_degree;

#ifdef DEBUG
		if(qp->qp_no<DEBUG_QP_NUM) {
			std::cout<<" q_w:"<<quick_result<< " n_w:"<<normal_result<<" c_w:"<<combine_result<<" od:"<<temp_ooo_degree;
			if(qp->HPS.infilight_pkts_limitation>combine_result)
				std::cout<<" old_limi:"<<qp->HPS.infilight_pkts_limitation<<" new_limi:"<<combine_result <<" ↓"<<qp->HPS.infilight_pkts_limitation - combine_result;
			else
				std::cout<<" old_limi:"<<qp->HPS.infilight_pkts_limitation<<" new_limi:"<<qp->HPS.infilight_pkts_limitation <<" ↓0";
		}
#endif
			qp->HPS.infilight_pkts_limitation = qp->HPS.infilight_pkts_limitation>combine_result?combine_result:qp->HPS.infilight_pkts_limitation;
		} else{
			rate_change_window_rtt = qp->HPS.base_rtt_percentage*qp->HPS.base_rtt + (1-qp->HPS.base_rtt_percentage)*qp->HPS.rtt;
			temp_add_step = rate_change_window_rtt * qp->HPS.add_step_rate / (m_mtu*8);
			qp->HPS.infilight_pkts_limitation += temp_add_step;
			// qp->HPS.infilight_pkts_limitation += qp->HPS.add_step_rate;
#ifdef DEBUG
		if(qp->qp_no<DEBUG_QP_NUM) {
			std::cout<<" old_limi:"<<qp->HPS.infilight_pkts_limitation-temp_add_step<<" new_limi:"<<qp->HPS.infilight_pkts_limitation<<" ↑"<<temp_add_step;
		}
#endif
		}

		if(qp->HPS.infilight_pkts_limitation > qp->HPS.infilight_pkts_max){
			qp->HPS.infilight_pkts_limitation = qp->HPS.infilight_pkts_max;
		}
		if(qp->HPS.infilight_pkts_limitation < qp->HPS.infilight_pkts_min){
			qp->HPS.infilight_pkts_limitation = qp->HPS.infilight_pkts_min;
		}
		
		qp->HPS.last_update_time = Simulator::Now().GetTimeStep();
		qp->HPS.ooo_acc=0;
		qp->HPS.ack_count = 0;
		qp->HPS.ack_number = 0;
		qp->HPS.ecn_number = 0;
#ifdef DEBUG
	if(qp->qp_no<DEBUG_QP_NUM) {
		std::cout<<std::endl;
	}
#endif
	}
	
	ChangeWindow(qp);
	dev->TriggerTransmit();
}

void RdmaHw::HandleAckHPS_CC_TIMEER(Ptr<RdmaQueuePair> qp,Ptr<QbbNetDevice> dev){
	// std::cout<<"HandleAckHPS_CC: timeout:"<<std::endl;
	ChangeWindow(qp);
	dev->TriggerTransmit();
}


void RdmaHw::HandleAckSwift_finegrained(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch,Ptr<QbbNetDevice> dev){
	uint64_t rtt = Simulator::Now().GetTimeStep() - ch.ack.ih.ts;
	uint32_t ack_psn = ch.ack.HPS.ack_psn; 
#ifdef DEBUG
	std::cout<<Simulator::Now().GetInteger()<<" qp_no:"<<qp->qp_no<<" ack_psn:"<< ack_psn<<" rtt:"<<rtt<<" rate:"<<qp->tj_window_now_forfabric*m_mtu*8/qp->tj_vague_rtt<<"(Gbps) window:"<<qp->tj_window_now_forfabric<< " pre_window:"<<qp->tj_window_pre_forfabric<<" target "<<qp->tj_target_delay_forfabric<< " inflights:"<< qp->GetHPSFly();
#endif

	//update the base rtt
	if(qp->tj_fs_base_target>rtt)
		qp->tj_fs_base_target=rtt;

	//first round for pre_rtt
	if (ack_psn == 1){
		qp->tj_delay_pre_forfabric = rtt;
		qp->tj_vague_rtt = rtt;
	}
	else{
		qp->tj_delay_pre_forfabric = qp->tj_delay_now_forfabric;
		qp->tj_vague_rtt = 0.8 * qp->tj_vague_rtt + 0.2* rtt;
	}

	qp->tj_delay_now_forfabric = rtt;

	//update the target
	qp->tj_target_delay_forfabric = Get_targetdelay_forfabric_swift(qp);

	//recode the last windowsize
	qp->tj_window_pre_forfabric = qp->tj_window_now_forfabric;

	//update the window size
	UpdateWindow_forfabric_finegrained(qp);

	qp->HPS.infilight_pkts_limitation = qp->tj_window_now_forfabric;
	qp->HPS.rtt = qp->tj_vague_rtt;

	//update the next qp-avail
	ChangeWindow(qp);
	dev->TriggerTransmit();
}


void RdmaHw::HandleAckSwift(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch,Ptr<QbbNetDevice> dev){
	uint64_t rtt = Simulator::Now().GetTimeStep() - ch.ack.ih.ts;
	uint32_t ack_psn = ch.ack.HPS.ack_psn; 

	if (Simulator::Now().GetInteger() - qp->tj_last_time_decrease_forfabric < qp->tj_vague_rtt){
		ChangeWindow(qp);
		return;
	}else
		qp->tj_last_time_decrease_forfabric = Simulator::Now().GetInteger();

#ifdef DEBUG
	std::cout<<Simulator::Now().GetInteger()<<" qp_no:"<<qp->qp_no<<" ack_psn:"<< ack_psn<<" rtt:"<<rtt<<" rate:"<<qp->tj_window_now_forfabric*m_mtu*8/qp->tj_vague_rtt<<"(Gbps) window:"<<qp->tj_window_now_forfabric<< " pre_window:"<<qp->tj_window_pre_forfabric<<" target "<<qp->tj_target_delay_forfabric<< " inflights:"<< qp->GetHPSFly();
#endif

	//update the base rtt
	if(qp->tj_fs_base_target>rtt)
		qp->tj_fs_base_target=rtt;

	//first round for pre_rtt
	if (ack_psn == 1){
		qp->tj_delay_pre_forfabric = rtt;
		qp->tj_vague_rtt = rtt;
	}
	else{
		qp->tj_delay_pre_forfabric = qp->tj_delay_now_forfabric;
		qp->tj_vague_rtt = 0.8 * qp->tj_vague_rtt + 0.2* rtt;
	}
		
	qp->tj_delay_now_forfabric = rtt;

	//update the target
	qp->tj_target_delay_forfabric = Get_targetdelay_forfabric_swift(qp);

	//recode the last windowsize
	qp->tj_window_pre_forfabric = qp->tj_window_now_forfabric;

	//update the window size
	UpdateWindow_forfabric(qp);

	//mark if it is decreasing
	qp->HPS.infilight_pkts_limitation = qp->tj_window_now_forfabric;
	qp->HPS.rtt = qp->tj_vague_rtt;
	//update the next qp-avail
	ChangeWindow(qp);
	dev->TriggerTransmit();
}

void RdmaHw::ChangeWindow(Ptr<RdmaQueuePair> qp)
{
	//Update the next availe
	if(!qp->HPS.quick_send_queue.empty()){
		qp->m_nextAvail = Simulator::Now();
	} else{
		double should_send_temp = qp->HPS.infilight_pkts_limitation - (qp->HPS.next_send_psn -  qp->HPS.tx_base_psn);
		if(should_send_temp<=0)
			qp->m_nextAvail = Simulator::Now() + Seconds(10000.0);
		else if(should_send_temp < 1)
			qp->m_nextAvail = Simulator::Now()+ NanoSeconds(qp->HPS.rtt / qp->HPS.infilight_pkts_limitation);
		else
			qp->m_nextAvail = Simulator::Now();	
	}

	// update nic's next avail event
	uint32_t nic_idx = GetNicIdxOfQp(qp);
	m_nic[nic_idx].dev->UpdateNextAvail(qp->m_nextAvail);
}


void RdmaHw::UpdateWindow_forfabric_finegrained(Ptr<RdmaQueuePair> qp)
{
	// if current delay is smaller than the target ,we should add
	if (qp->tj_delay_now_forfabric <= qp->tj_target_delay_forfabric)
	{
		double temp_ai_forfabric = qp->tj_increat_ai_forfabric;
		if (qp->tj_window_now_forfabric >= 1)
			qp->tj_window_now_forfabric += temp_ai_forfabric / qp->tj_window_now_forfabric;
		else
			qp->tj_window_now_forfabric += temp_ai_forfabric;
	}
	//otherwise we should decrease(but only if the pause time is greater than one rtt)
	if (Simulator::Now().GetInteger() - qp->tj_last_time_decrease_forfabric >= qp->tj_vague_rtt)
	{
		double normal_decrease = 1.0 - qp->tj_decreat_b_forfabric* ( ((double)(qp->tj_delay_now_forfabric) - (double)(qp->tj_target_delay_forfabric)) / (double)(qp->tj_delay_now_forfabric));
		double thresh_decrease = 1.0 - qp->tj_decreat_b_max_forfabric;
		//if (flow_id <= 200)
		//std::cout << " normal decrease " << normal_decrease << "  " << thresh_decrease << std::endl;
		qp->tj_window_now_forfabric = (normal_decrease > thresh_decrease ? normal_decrease : thresh_decrease)* qp->tj_window_now_forfabric;
	}

	//bound the result
	if (qp->tj_window_now_forfabric < qp->tj_min_window_forall)
		qp->tj_window_now_forfabric = qp->tj_min_window_forall;

	else if (qp->tj_window_now_forfabric > qp->tj_max_window_forall)
		qp->tj_window_now_forfabric = qp->tj_max_window_forall;

#ifdef DEBUG
	if (qp->tj_window_pre_forfabric > qp->tj_window_now_forfabric)
		std::cout<<"  post_window:"<<qp->tj_window_now_forfabric<< " ↓"<< qp->tj_window_pre_forfabric - qp->tj_window_now_forfabric<< std::endl;
	else if (qp->tj_window_pre_forfabric < qp->tj_window_now_forfabric)
		std::cout<<"  post_window:"<<qp->tj_window_now_forfabric<< " ↑"<< qp->tj_window_now_forfabric - qp->tj_window_pre_forfabric<< std::endl;
	else
		std::cout<<"  post_window:"<<qp->tj_window_now_forfabric<< " ="<<std::endl;
#endif

	//mark if it is decreasing
	if (qp->tj_window_now_forfabric < qp->tj_window_pre_forfabric)
		qp->tj_last_time_decrease_forfabric = Simulator::Now().GetInteger();
	
	qp->HPS.infilight_pkts_limitation = qp->tj_window_now_forfabric;
}


void RdmaHw::UpdateWindow_forfabric(Ptr<RdmaQueuePair> qp)
{
	// if current delay is smaller than the target ,we should add
	if (qp->tj_delay_now_forfabric <= qp->tj_target_delay_forfabric)
	{
		qp->tj_window_now_forfabric +=  qp->tj_increat_ai_forfabric;
	} else{
		double normal_decrease = 1.0 - qp->tj_decreat_b_forfabric* ( ((double)(qp->tj_delay_now_forfabric) - (double)(qp->tj_target_delay_forfabric)) / (double)(qp->tj_delay_now_forfabric));
		double thresh_decrease = 1.0 - qp->tj_decreat_b_max_forfabric;
		//if (flow_id <= 200)
		//std::cout << " normal decrease " << normal_decrease << "  " << thresh_decrease << std::endl;
		qp->tj_window_now_forfabric = (normal_decrease > thresh_decrease ? normal_decrease : thresh_decrease)* qp->tj_window_now_forfabric;
	}

	//bound the result
	if (qp->tj_window_now_forfabric < qp->tj_min_window_forall)
		qp->tj_window_now_forfabric = qp->tj_min_window_forall;

	else if (qp->tj_window_now_forfabric > qp->tj_max_window_forall)
		qp->tj_window_now_forfabric = qp->tj_max_window_forall;

#ifdef DEBUG
	if (qp->tj_window_pre_forfabric > qp->tj_window_now_forfabric)
		std::cout<<"  post_window:"<<qp->tj_window_now_forfabric<< " ↓"<< qp->tj_window_pre_forfabric - qp->tj_window_now_forfabric<< std::endl;
	else if (qp->tj_window_pre_forfabric < qp->tj_window_now_forfabric)
		std::cout<<"  post_window:"<<qp->tj_window_now_forfabric<< " ↑"<< qp->tj_window_now_forfabric - qp->tj_window_pre_forfabric<< std::endl;
	else
		std::cout<<"  post_window:"<<qp->tj_window_now_forfabric<< " ="<<std::endl;
#endif

}


uint64_t RdmaHw::Get_targetdelay_forfabric_swift(Ptr<RdmaQueuePair> qp)
{
	if (qp->tj_window_now_forfabric < qp->tj_fs_min_cwnd)
		return qp->tj_fs_range + qp->tj_fs_base_target;
	else if (qp->tj_window_now_forfabric > qp->tj_fs_max_cwnd)
		return qp->tj_fs_base_target;

	uint32_t path_scale_hop = qp->tj_fs_base_target/qp->base_rtt_unit/2 -1;
	uint64_t temp_target = qp->tj_fs_base_target + path_scale_hop*qp->path_scale_factor + (uint64_t)Max(0, Mix(qp->tj_fs_alfa / sqrt(qp->tj_window_now_forfabric) + qp->tj_fs_beta, qp->tj_fs_range));
#ifdef DEBUG
	std::cout<<" base_target:"<<qp->tj_fs_base_target<<" path_scale:"<<path_scale_hop*qp->path_scale_factor    <<" curve_target:"<<(uint64_t)Max(0, Mix(qp->tj_fs_alfa / sqrt(qp->tj_window_now_forfabric) + qp->tj_fs_beta, qp->tj_fs_range));
#endif
	return temp_target;
}


double RdmaHw::Get_ecn_threshold_hps(Ptr<RdmaQueuePair> qp)
{
	uint32_t path_scale_hop = qp->HPS.base_rtt/qp->HPS.base_rtt_unit/2 - 1;
	double temp = 1.0 - log2(qp->HPS.infilight_pkts_limitation/qp->HPS.infilight_pkts_min) / log2(qp->HPS.infilight_pkts_max/qp->HPS.infilight_pkts_min) +  path_scale_hop * qp->HPS.path_scale_fator;
	return temp > 1.0 ? 1.0:temp;
}

uint64_t RdmaHw::Mix(double a, double b)
{
	return a < b ? a : b;
}
uint64_t RdmaHw::Max(double a, double b)
{
	return a > b ? a : b;
}




void RdmaHw::UpdateRateTimely(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch, bool us){
	uint32_t next_seq = qp->snd_nxt;
	uint64_t rtt = Simulator::Now().GetTimeStep() - ch.ack.ih.ts;
	bool print = !us;
	if (qp->tmly.m_lastUpdateSeq != 0){ // not first RTT
		int64_t new_rtt_diff = (int64_t)rtt - (int64_t)qp->tmly.lastRtt;
		double rtt_diff = (1 - m_tmly_alpha) * qp->tmly.rttDiff + m_tmly_alpha * new_rtt_diff;
		double gradient = rtt_diff / m_tmly_minRtt;
		bool inc = false;
		double c = 0;
		#if PRINT_LOG
		if (print)
			printf("%lu node:%u rtt:%lu rttDiff:%.0lf gradient:%.3lf rate:%.3lf", Simulator::Now().GetTimeStep(), m_node->GetId(), rtt, rtt_diff, gradient, qp->tmly.m_curRate.GetBitRate() * 1e-9);
		#endif
		if (rtt < m_tmly_TLow){
			inc = true;
		}else if (rtt > m_tmly_THigh){
			c = 1 - m_tmly_beta * (1 - (double)m_tmly_THigh / rtt);
			inc = false;
		}else if (gradient <= 0){
			inc = true;
		}else{
			c = 1 - m_tmly_beta * gradient;
			if (c < 0)
				c = 0;
			inc = false;
		}
		if (inc){
			if (qp->tmly.m_incStage < 5){
				qp->m_rate = qp->tmly.m_curRate + m_rai;
			}else{
				qp->m_rate = qp->tmly.m_curRate + m_rhai;
			}
			if (qp->m_rate > qp->m_max_rate)
				qp->m_rate = qp->m_max_rate;
			if (!us){
				qp->tmly.m_curRate = qp->m_rate;
				qp->tmly.m_incStage++;
				qp->tmly.rttDiff = rtt_diff;
			}
		}else{
			qp->m_rate = std::max(m_minRate, qp->tmly.m_curRate * c); 
			if (!us){
				qp->tmly.m_curRate = qp->m_rate;
				qp->tmly.m_incStage = 0;
				qp->tmly.rttDiff = rtt_diff;
			}
		}
		#if PRINT_LOG
		if (print){
			printf(" %c %.3lf\n", inc? '^':'v', qp->m_rate.GetBitRate() * 1e-9);
		}
		#endif
	}
	if (!us && next_seq > qp->tmly.m_lastUpdateSeq){
		qp->tmly.m_lastUpdateSeq = next_seq;
		// update
		qp->tmly.lastRtt = rtt;
	}
}
void RdmaHw::FastReactTimely(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch){
	qp->m_rate+= m_rhai;
}

/**********************
 * DCTCP
 *********************/
void RdmaHw::HandleAckDctcp(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch){
	uint32_t ack_seq = ch.ack.seq;
	uint8_t cnp = (ch.ack.flags >> qbbHeader::FLAG_CNP) & 1;
	bool new_batch = false;

	// update alpha
	qp->dctcp.m_ecnCnt += (cnp > 0);
	if (ack_seq > qp->dctcp.m_lastUpdateSeq){ // if full RTT feedback is ready, do alpha update
		#if PRINT_LOG
		printf("%lu %s %08x %08x %u %u [%u,%u,%u] %.3lf->", Simulator::Now().GetTimeStep(), "alpha", qp->sip.Get(), qp->dip.Get(), qp->sport, qp->dport, qp->dctcp.m_lastUpdateSeq, ch.ack.seq, qp->snd_nxt, qp->dctcp.m_alpha);
		#endif
		new_batch = true;
		if (qp->dctcp.m_lastUpdateSeq == 0){ // first RTT
			qp->dctcp.m_lastUpdateSeq = qp->snd_nxt;
			qp->dctcp.m_batchSizeOfAlpha = qp->snd_nxt / m_mtu + 1;
		}else {
			double frac = std::min(1.0, double(qp->dctcp.m_ecnCnt) / qp->dctcp.m_batchSizeOfAlpha);
			qp->dctcp.m_alpha = (1 - m_g) * qp->dctcp.m_alpha + m_g * frac;
			qp->dctcp.m_lastUpdateSeq = qp->snd_nxt;
			qp->dctcp.m_ecnCnt = 0;
			qp->dctcp.m_batchSizeOfAlpha = (qp->snd_nxt - ack_seq) / m_mtu + 1;
			#if PRINT_LOG
			printf("%.3lf F:%.3lf", qp->dctcp.m_alpha, frac);
			#endif
		}
		#if PRINT_LOG
		printf("\n");
		#endif
	}

	// check cwr exit
	if (qp->dctcp.m_caState == 1){
		if (ack_seq > qp->dctcp.m_highSeq)
			qp->dctcp.m_caState = 0;
	}

	// check if need to reduce rate: ECN and not in CWR
	if (cnp && qp->dctcp.m_caState == 0){
		#if PRINT_LOG
		printf("%lu %s %08x %08x %u %u %.3lf->", Simulator::Now().GetTimeStep(), "rate", qp->sip.Get(), qp->dip.Get(), qp->sport, qp->dport, qp->m_rate.GetBitRate()*1e-9);
		#endif
		qp->m_rate = std::max(m_minRate, qp->m_rate * (1 - qp->dctcp.m_alpha / 2));
		#if PRINT_LOG
		printf("%.3lf\n", qp->m_rate.GetBitRate() * 1e-9);
		#endif
		qp->dctcp.m_caState = 1;
		qp->dctcp.m_highSeq = qp->snd_nxt;
	}

	// additive inc
	if (qp->dctcp.m_caState == 0 && new_batch)
		qp->m_rate = std::min(qp->m_max_rate, qp->m_rate + m_dctcp_rai);
}

/*********************
 * HPCC-PINT
 ********************/
void RdmaHw::SetPintSmplThresh(double p){
       pint_smpl_thresh = (uint32_t)(65536 * p);
}
void RdmaHw::HandleAckHpPint(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch){
       uint32_t ack_seq = ch.ack.seq;
       if (rand() % 65536 >= pint_smpl_thresh)
               return;
       // update rate
       if (ack_seq > qp->hpccPint.m_lastUpdateSeq){ // if full RTT feedback is ready, do full update
               UpdateRateHpPint(qp, p, ch, false);
       }else{ // do fast react
               UpdateRateHpPint(qp, p, ch, true);
       }
}

void RdmaHw::UpdateRateHpPint(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch, bool fast_react){
       uint32_t next_seq = qp->snd_nxt;
       if (qp->hpccPint.m_lastUpdateSeq == 0){ // first RTT
               qp->hpccPint.m_lastUpdateSeq = next_seq;
       }else {
               // check packet INT
               IntHeader &ih = ch.ack.ih;
               double U = Pint::decode_u(ih.GetPower());

               DataRate new_rate;
               int32_t new_incStage;
               double max_c = U / m_targetUtil;

               if (max_c >= 1 || qp->hpccPint.m_incStage >= m_miThresh){
                       new_rate = qp->hpccPint.m_curRate / max_c + m_rai;
                       new_incStage = 0;
               }else{
                       new_rate = qp->hpccPint.m_curRate + m_rai;
                       new_incStage = qp->hpccPint.m_incStage+1;
               }
               if (new_rate < m_minRate)
                       new_rate = m_minRate;
               if (new_rate > qp->m_max_rate)
                       new_rate = qp->m_max_rate;
               ChangeRate(qp, new_rate);
               if (!fast_react){
                       qp->hpccPint.m_curRate = new_rate;
                       qp->hpccPint.m_incStage = new_incStage;
               }
               if (!fast_react){
                       if (next_seq > qp->hpccPint.m_lastUpdateSeq)
                               qp->hpccPint.m_lastUpdateSeq = next_seq; //+ rand() % 2 * m_mtu;
               }
       }
}

}
