/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011, 2012, 2013 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Lluis Parcerisa <lparcerisa@cttc.cat> (cv2x_TestUtils from test-asn1-encoding.cc)
 *         Nicola Baldo <nbaldo@cttc.es> (actual test)
 */

#include "ns3/log.h"
#include "ns3/test.h"
#include "ns3/ptr.h"
#include "ns3/packet.h"

#include "ns3/cv2x_lte-rlc-am-header.h"

#include <list>
#include <bitset>
#include <iomanip>


NS_LOG_COMPONENT_DEFINE ("cv2x_TestLteRlcHeader");

namespace ns3 {

/**
 * \ingroup lte-test
 * \ingroup tests
 *
 * \brief Test Utils
 */
class cv2x_TestUtils
{
public:
  /**
   * Function to convert packet contents in hex format
   * \param pkt the packet
   * \returns a text string
   */
  static std::string sprintPacketContentsHex (Ptr<Packet> pkt)
  {
    uint32_t psize = pkt->GetSize ();
    uint8_t buffer[psize];
    std::ostringstream oss (std::ostringstream::out);
    pkt->CopyData (buffer, psize);
    for (uint32_t i = 0; i < psize; i++)
      {
        oss << std::setfill ('0') << std::setw (2) << std::hex << (uint32_t) buffer[i];
      }
    return oss.str ();
  }

  /**
   * Function to convert packet contents in binary format
   * \param pkt the packet
   * \returns a text string
   */
  static std::string sprintPacketContentsBin (Ptr<Packet> pkt)
  {
    uint32_t psize = pkt->GetSize ();
    uint8_t buffer[psize];
    std::ostringstream oss (std::ostringstream::out);
    pkt->CopyData (buffer, psize);
    for (uint32_t i = 0; i < psize; i++)
      {
        oss << (std::bitset<8> (buffer[i]));
      }
    return std::string (oss.str () + "\n");
  }

  /**
   * Function to log packet contents
   * \param pkt the packet
   */
  static void LogPacketContents (Ptr<Packet> pkt)
  {
    NS_LOG_DEBUG ("---- SERIALIZED PACKET CONTENTS (HEX): -------");
    NS_LOG_DEBUG ("Hex: " << cv2x_TestUtils::sprintPacketContentsHex (pkt));
    NS_LOG_DEBUG ("Bin: " << cv2x_TestUtils::sprintPacketContentsBin (pkt));
  }

  /**
   * Log packet info function
   * \param source T
   * \param s text string to log
   */
  template <class T>
  static void LogPacketInfo (T source,std::string s)
  {
    NS_LOG_DEBUG ("--------- " << s.data () << " INFO: -------");
    std::ostringstream oss (std::ostringstream::out);
    source.Print (oss);
    NS_LOG_DEBUG (oss.str ());
  }
};


/**
 * \ingroup lte-test
 * \ingroup tests
 *
 * \brief Rlc Am Status Pdu Test Case
 */
class cv2x_RlcAmStatusPduTestCase : public TestCase
{
public:
  /**
   * Constructor
   *
   * \param ackSn the sequence number
   * \param nackSnList list of nack sequence numbers
   * \param hex string
   */
  cv2x_RlcAmStatusPduTestCase (cv2x_SequenceNumber10 ackSn, 
			  std::list<cv2x_SequenceNumber10> nackSnList,
			  std::string hex);

protected:  
  virtual void DoRun (void);
  
  cv2x_SequenceNumber10 m_ackSn; ///< ack sequence number  
  std::list<cv2x_SequenceNumber10> m_nackSnList; ///< list of nack sequence numbers
  std::string m_hex; ///< hex string
  
};


cv2x_RlcAmStatusPduTestCase::cv2x_RlcAmStatusPduTestCase (cv2x_SequenceNumber10 ackSn, 
						std::list<cv2x_SequenceNumber10> nackSnList ,
						std::string hex)
  : TestCase (hex), 
    m_ackSn (ackSn),
    m_nackSnList (nackSnList),
    m_hex (hex)
{
  NS_LOG_FUNCTION (this << hex);
}

void

cv2x_RlcAmStatusPduTestCase::DoRun ()
{
  NS_LOG_FUNCTION (this);
  
  Ptr<Packet> p = Create<Packet> ();
  cv2x_LteRlcAmHeader h;
  h.SetControlPdu (cv2x_LteRlcAmHeader::STATUS_PDU);
  h.SetAckSn (m_ackSn);
  for (std::list<cv2x_SequenceNumber10>::iterator it = m_nackSnList.begin ();
       it != m_nackSnList.end ();
       ++it)
    {
      h.PushNack (it->GetValue ());
    }
  p->AddHeader (h);  

  cv2x_TestUtils::LogPacketContents (p);
  std::string hex = cv2x_TestUtils::sprintPacketContentsHex (p);
  NS_TEST_ASSERT_MSG_EQ (m_hex, hex, "serialized packet content " << hex << " differs from test vector " << m_hex);
  
  cv2x_LteRlcAmHeader h2;
  p->RemoveHeader (h2);
  cv2x_SequenceNumber10 ackSn = h2.GetAckSn ();
  NS_TEST_ASSERT_MSG_EQ (ackSn, m_ackSn, "deserialized ACK SN differs from test vector");
  
  for (std::list<cv2x_SequenceNumber10>::iterator it = m_nackSnList.begin ();
       it != m_nackSnList.end ();
       ++it)
    {
      int nackSn = h2.PopNack ();
      NS_TEST_ASSERT_MSG_GT (nackSn, -1, "not enough elements in deserialized NACK list");
      NS_TEST_ASSERT_MSG_EQ (nackSn, it->GetValue (), "deserialized NACK SN  differs from test vector");
    }
  int retVal = h2.PopNack ();
  NS_TEST_ASSERT_MSG_LT (retVal, 0, "too many elements in deserialized NACK list");
}


/**
 * \ingroup lte-test
 * \ingroup tests
 *
 * \brief Lte Rlc Header Test Suite
 */
class cv2x_LteRlcHeaderTestSuite : public TestSuite
{
public:
  cv2x_LteRlcHeaderTestSuite ();
} staticCv2xLteRlcHeaderTestSuiteInstance ; ///< the test suite

cv2x_LteRlcHeaderTestSuite::cv2x_LteRlcHeaderTestSuite ()
  : TestSuite ("lte-rlc-header", UNIT)
{
  NS_LOG_FUNCTION (this);

  {
    cv2x_SequenceNumber10 ackSn (8);
    std::list<cv2x_SequenceNumber10> nackSnList;
    std::string hex ("0020");
    AddTestCase (new cv2x_RlcAmStatusPduTestCase (ackSn, nackSnList, hex), TestCase::QUICK);
  }

  {
    cv2x_SequenceNumber10 ackSn (873);
    std::list<cv2x_SequenceNumber10> nackSnList;
    std::string hex ("0da4");
    AddTestCase (new cv2x_RlcAmStatusPduTestCase (ackSn, nackSnList, hex), TestCase::QUICK);
  }

  {
    cv2x_SequenceNumber10 ackSn (2);
    std::list<cv2x_SequenceNumber10> nackSnList;
    nackSnList.push_back (cv2x_SequenceNumber10 (873));
    std::string hex ("000bb480");
    AddTestCase (new cv2x_RlcAmStatusPduTestCase (ackSn, nackSnList, hex), TestCase::QUICK);
  }


  {
    cv2x_SequenceNumber10 ackSn (2);
    std::list<cv2x_SequenceNumber10> nackSnList;
    nackSnList.push_back (cv2x_SequenceNumber10 (1021));
    nackSnList.push_back (cv2x_SequenceNumber10 (754));
    std::string hex ("000bfed790");
    AddTestCase (new cv2x_RlcAmStatusPduTestCase (ackSn, nackSnList, hex), TestCase::QUICK);
  }


  {
    cv2x_SequenceNumber10 ackSn (2);
    std::list<cv2x_SequenceNumber10> nackSnList;
    nackSnList.push_back (cv2x_SequenceNumber10 (1021));
    nackSnList.push_back (cv2x_SequenceNumber10 (754));
    nackSnList.push_back (cv2x_SequenceNumber10 (947));
    std::string hex ("000bfed795d980");
    AddTestCase (new cv2x_RlcAmStatusPduTestCase (ackSn, nackSnList, hex), TestCase::QUICK);
  }


  {
    cv2x_SequenceNumber10 ackSn (2);
    std::list<cv2x_SequenceNumber10> nackSnList;
    nackSnList.push_back (cv2x_SequenceNumber10 (1021));
    nackSnList.push_back (cv2x_SequenceNumber10 (754));
    nackSnList.push_back (cv2x_SequenceNumber10 (947));
    nackSnList.push_back (cv2x_SequenceNumber10 (347));
    std::string hex ("000bfed795d9cad8");
    AddTestCase (new cv2x_RlcAmStatusPduTestCase (ackSn, nackSnList, hex), TestCase::QUICK);
  }

}


} // namespace ns3
