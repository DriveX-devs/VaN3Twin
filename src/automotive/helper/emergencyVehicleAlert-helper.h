#ifndef EMERGENCYVEHICLEALERT_HELPER_H
#define EMERGENCYVEHICLEALERT_HELPER_H
#include "ns3/OpenCDAClient.h"
#include <stdint.h>
#include "ns3/application-container.h"
#include "ns3/node-container.h"
#include "ns3/object-factory.h"
#include "ns3/ipv4-address.h"
#include "ns3/ipv6-address.h"
#include "ns3/traci-client.h"
#include "ns3/inet-socket-address.h"
#include "ns3/string.h"
#include "ns3/names.h"

namespace ns3 {

/**
 * \ingroup TrafficInfo
 * \brief Create an application which sends a UDP packet and waits for an echo of this packet
 */
class emergencyVehicleAlertHelper
{
public:
  /**
   * Create TrafficInfoClientHelper which will make life easier for people trying
   * to set up simulations with echos.
   *
   * \param ip The IP address of the remote Traffic Info server
   * \param port The port number of the remote Traffic Info server
   */
  emergencyVehicleAlertHelper ();

  void SetAttribute (std::string name, const AttributeValue &value);

  /**
   * Create a TrafficInfoServerApplication on the specified Node.
   *
   * \param node The node on which to create the Application.  The node is
   *             specified by a Ptr<Node>.
   *
   * \returns An ApplicationContainer holding the Application created,
   */
  ApplicationContainer Install (Ptr<Node> node) const;

  /**
   * Create a Traffic Info client application on the specified node.  The Node
   * is provided as a string name of a Node that has been previously 
   * associated using the Object Name Service.
   *
   * \param nodeName The name of the node on which to create the TrafficInfoClientApplication
   *
   * \returns An ApplicationContainer that holds a Ptr<Application> to the 
   *          application created
   */
  ApplicationContainer Install (std::string nodeName) const;

  /**
   * \param c the nodes
   *
   * Create one Traffic Info client application on each of the input nodes
   *
   * \returns the applications created, one application per input node.
   */
  ApplicationContainer Install (NodeContainer c) const;

private:
  /**
   * Install an ns3::TrafficInfoClient on the node configured with all the
   * attributes set with SetAttribute.
   *
   * \param node The node on which an TrafficInfoClient will be installed.
   * \returns Ptr to the application installed.
   */
  Ptr<Application> InstallPriv (Ptr<Node> node) const;
  ObjectFactory m_factory; //!< Object factory.
};

} // namespace ns3

#endif /* EMERGENCYVEHICLEALERT_HELPER_H */
