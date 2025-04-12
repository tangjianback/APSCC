#include <stdint.h>
#include <iostream>
#include "qbb-header.h"
#include "ns3/buffer.h"
#include "ns3/address-utils.h"
#include "ns3/log.h"

NS_LOG_COMPONENT_DEFINE("qbbHeader");

namespace ns3 {

	NS_OBJECT_ENSURE_REGISTERED(qbbHeader);

	qbbHeader::qbbHeader(uint16_t pg)
		: m_pg(pg), sport(0), dport(0), flags(0), m_seq(0)
	{
	}

	qbbHeader::qbbHeader()
		: m_pg(0), sport(0), dport(0), flags(0), m_seq(0)
	{}

	qbbHeader::~qbbHeader()
	{}

	void qbbHeader::SetPG(uint16_t pg)
	{
		m_pg = pg;
	}

	void qbbHeader::SetSeq(uint32_t seq)
	{
		m_seq = seq;
	}

	void qbbHeader::SetSport(uint32_t _sport){
		sport = _sport;
	}
	void qbbHeader::SetDport(uint32_t _dport){
		dport = _dport;
	}

	void qbbHeader::SetTs(uint64_t ts){
		NS_ASSERT_MSG(IntHeader::mode == 1, "qbbHeader cannot SetTs when IntHeader::mode != 1");
		ih.ts = ts;
	}
	void qbbHeader::SetCnp(){
		flags |= 1 << FLAG_CNP;
	}
	void qbbHeader::SetIntHeader(const IntHeader &_ih){
		ih = _ih;
	}

	void qbbHeader::SetAckPsn(uint32_t _psn){
		HPS.ack_psn = _psn;
	}
	void qbbHeader::SetOooDistance(uint16_t distance){
		HPS.ooo_distance = distance;
	}

	uint64_t qbbHeader::SetReceiveingRate(uint64_t receiveing_rate){
		PCN.receiveing_rate = receiveing_rate;
	}

	uint64_t qbbHeader::GetReceiveingRate() const{
		return PCN.receiveing_rate;
	}
	uint16_t qbbHeader::GetOooDistance() const{
		return  HPS.ooo_distance;
	}

	uint32_t qbbHeader::GetAckPsn() const
	{
		return HPS.ack_psn;
	}

	uint16_t qbbHeader::GetPG() const
	{
		return m_pg;
	}

	uint32_t qbbHeader::GetSeq() const
	{
		return m_seq;
	}

	uint16_t qbbHeader::GetSport() const{
		return sport;
	}
	uint16_t qbbHeader::GetDport() const{
		return dport;
	}

	uint64_t qbbHeader::GetTs() const {
		NS_ASSERT_MSG(IntHeader::mode == 1, "qbbHeader cannot GetTs when IntHeader::mode != 1");
		return ih.ts;
	}
	uint8_t qbbHeader::GetCnp() const{
		return (flags >> FLAG_CNP) & 1;
	}

	TypeId
		qbbHeader::GetTypeId(void)
	{
		static TypeId tid = TypeId("ns3::qbbHeader")
			.SetParent<Header>()
			.AddConstructor<qbbHeader>()
			;
		return tid;
	}
	TypeId
		qbbHeader::GetInstanceTypeId(void) const
	{
		return GetTypeId();
	}
	void qbbHeader::Print(std::ostream &os) const
	{
		os << "qbb:" << "pg=" << m_pg << ",seq=" << m_seq <<std::endl;
	}
	uint32_t qbbHeader::GetSerializedSize(void)  const
	{
		return GetBaseSize() + IntHeader::GetStaticSize();
	}
	uint32_t qbbHeader::GetBaseSize() {
		qbbHeader tmp;
		return sizeof(tmp.sport) + sizeof(tmp.dport) + sizeof(tmp.flags) + sizeof(tmp.m_pg) + sizeof(tmp.m_seq)+sizeof(tmp.HPS.ack_psn)+sizeof(tmp.HPS.ooo_distance)+sizeof(tmp.PCN.receiveing_rate);
	}
	void qbbHeader::Serialize(Buffer::Iterator start)  const
	{
		Buffer::Iterator i = start;
		i.WriteU16(sport);
		i.WriteU16(dport);
		i.WriteU16(flags);
		i.WriteU16(m_pg);
		i.WriteU32(m_seq);
		i.WriteU16(HPS.ooo_distance);
		i.WriteU32(HPS.ack_psn);

		i.WriteU64(PCN.receiveing_rate);

		// write IntHeader
		ih.Serialize(i);
	}

	uint32_t qbbHeader::Deserialize(Buffer::Iterator start)
	{
		Buffer::Iterator i = start;
		sport = i.ReadU16();
		dport = i.ReadU16();
		flags = i.ReadU16();
		m_pg = i.ReadU16();
		m_seq = i.ReadU32();
		HPS.ooo_distance = i.ReadU16();
		HPS.ack_psn = i.ReadU32();

		PCN.receiveing_rate = i.ReadU64();

		// read IntHeader
		ih.Deserialize(i);
		return GetSerializedSize();
	}
}; // namespace ns3
