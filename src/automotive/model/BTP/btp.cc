/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
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

 * Edited by Carlos Risma, Politecnico di Torino
 * (carlosrisma@gmail.com)
*/
#include "btp.h"


namespace ns3
{
  NS_LOG_COMPONENT_DEFINE("btp");

  TypeId
  btp::GetTypeId ()
  {
    static TypeId tid = TypeId("ns3::btp")
        .SetParent <Object>()
        .AddConstructor <btp>();
    return tid;
  }

  btp::~btp ()
  {
    NS_LOG_FUNCTION(this);
  }

  btp::btp()
  {
    m_geonet = NULL;

    m_cam_ReceiveCallback = nullptr;
    m_denm_ReceiveCallback = nullptr;
    m_ivim_ReceiveCallback = nullptr;
    m_vam_ReceiveCallback = nullptr;
  }

  void
  btp::cleanup ()
  {
    // Cleanup the GeoNet object
    m_geonet->cleanup();
  }

  void
  btp::setSocketRx (Ptr<Socket> socket_rx)
  {
    socket_rx->SetRecvCallback (MakeCallback(&GeoNet::receiveGN, m_geonet));
    m_geonet->addRxCallback(std::bind(static_cast<void(btp::*)(GNDataIndication_t,Address)>(&btp::receiveBTP),this,std::placeholders::_1,std::placeholders::_2));
  }

  std::tuple<GNDataConfirm_t, MessageId_t>
  btp::sendBTP(BTPDataRequest_t dataRequest, int priority, MessageId_t message_id)
  {
    GNDataConfirm_t dataConfirm;
    GNDataRequest_t GnDataRequest = {};
    btpHeader header;
    header.SetDestinationPort (dataRequest.destPort);
    GnDataRequest._messagePort = dataRequest.destPort;

    if(dataRequest.BTPType==BTP_A) // BTP-A
    {
      header.SetSourcePort (dataRequest.sourcePort);
      GnDataRequest.upperProtocol = BTP_A;
    }
    else    //  BTP-B
    {
      header.SetDestinationPortInfo (dataRequest.destPInfo);
      GnDataRequest.upperProtocol = BTP_B;
    }
    dataRequest.data->AddHeader (header);

    //Filling the GN-dataRequest
    GnDataRequest.GNType = dataRequest.GNType;
    GnDataRequest.GnAddress = dataRequest.GnAddress;
    GnDataRequest.GNCommProfile = dataRequest.GNCommProfile;
    GnDataRequest.GNRepInt = dataRequest.GNRepInt;
    GnDataRequest.GNMaxRepTime = dataRequest.GNMaxRepInt;
    GnDataRequest.GNMaxLife = dataRequest.GNMaxLife;
    GnDataRequest.GNMaxHL = dataRequest.GNMaxHL;
    GnDataRequest.GNTraClass = dataRequest.GNTraClass;
    GnDataRequest.data = dataRequest.data;
    GnDataRequest.lenght = dataRequest.lenght + 4;

    std::tuple<GNDataConfirm_t, MessageId_t> status = m_geonet->sendGN(GnDataRequest, priority, message_id);
    dataConfirm = std::get<0>(status);
    if(dataConfirm != ACCEPTED && dataConfirm != BLOCKED_BY_GK) {
        std::cerr << "Error! GeoNetworking could not send any packet." << std::endl;
      }
    return status;

  }

  void
  btp::receiveBTP(GNDataIndication_t dataIndication,Address address)
  {
    btpHeader header;
    BTPDataIndication_t btpDataIndication = {};
    dataIndication.data->RemoveHeader (header, 4);

    btpDataIndication.BTPType = dataIndication.upperProtocol;
    btpDataIndication.destPort = header.GetDestinationPort ();
    if(btpDataIndication.BTPType == BTP_A)
    {
      btpDataIndication.sourcePort = header.GetSourcePort ();
      btpDataIndication.destPInfo = 0;
    }
    else //BTP-B
    {
      btpDataIndication.destPInfo = header.GetDestinationPortInfo ();
      btpDataIndication.sourcePort = 0;
    }
    btpDataIndication.GnAddress = dataIndication.GnAddressDest;
    btpDataIndication.GNTraClass = dataIndication.GNTraClass;
    btpDataIndication.GNRemPLife = dataIndication.GNRemainingLife;
    btpDataIndication.GNPositionV = dataIndication.SourcePV;
    btpDataIndication.GNAddressSource = dataIndication.SourcePV.GnAddress;
    btpDataIndication.data = dataIndication.data;
    btpDataIndication.lenght = dataIndication.data->GetSize ();

    if(btpDataIndication.destPort == CA_PORT) {
      if(m_cam_ReceiveCallback!=nullptr) {
        m_cam_ReceiveCallback(btpDataIndication,address);
      }
    } else if(btpDataIndication.destPort == DEN_PORT) {
      if(m_denm_ReceiveCallback!=nullptr) {
        m_denm_ReceiveCallback(btpDataIndication,address);
      }
    } else if(btpDataIndication.destPort == IVIM_PORT) {
      if(m_ivim_ReceiveCallback!=nullptr) {
        m_ivim_ReceiveCallback(btpDataIndication,address);
      }
    }
    else if(btpDataIndication.destPort == CP_PORT){
      if(m_cpm_ReceiveCallback!=nullptr){
          m_cpm_ReceiveCallback(btpDataIndication,address);
        }
      } else if(btpDataIndication.destPort == VA_PORT) {
        if(m_vam_ReceiveCallback!=nullptr) {
          m_vam_ReceiveCallback(btpDataIndication,address);
        }
    } else if (btpDataIndication.destPort == MC_PORT) {
        if (m_mcm_ReceiveCallback != nullptr) {
            m_mcm_ReceiveCallback(btpDataIndication, address);
          }
    } else {
    NS_LOG_ERROR("BTP : Unknown port");
    }
  }

}
