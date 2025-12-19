//
// Copyright (C) 2023 Tobias Hardes <tobias.hardes@uni-paderborn.de>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// SPDX-License-Identifier: GPL-2.0-or-later
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
#ifndef OPENCDA_CLIENT_H
#define OPENCDA_CLIENT_H
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "google/protobuf/empty.pb.h"
#include "grpcpp/grpcpp.h"
#include <signal.h>
#include <unistd.h>


#include "ns3/carla.grpc.pb.h"

#define STARTUP_FCN std::function<Ptr<Node>(std::string)>
#define SHUTDOWN_FCN std::function<void(Ptr<Node>,std::string)>

namespace ns3
{

    class OpenCDAClient : public Object {
    public:
      static TypeId GetTypeId (void);
      OpenCDAClient (void);
      ~OpenCDAClient (void);
      void startCarlaAdapter(STARTUP_FCN includeNode, SHUTDOWN_FCN excludeNode);
      void startSimulation();
      void testInsertVehicle();

      void stopSimulation();

      carla::ActorIds GetManagedHostIds();
      carla::Vehicle GetManagedActorById(int actorId);
      carla::Actor GetActorById(int actorId);
      carla::ActorIds GetAllActorsIds();
      carla::Transform getRandomSpawnPoint();

      carla::Vector getCartesian(double lon, double lat);
      carla::Vector getGeo(double x, double y);
      double getSpeed(int id);
      double getHeading(int id);
      bool hasCARLALDM(int id);
      carla::Objects getDetectedObjects(int id);
      void setControl(int id, double speed, Vector position, double acceleration);
      carla::Waypoint getWaypoint(Vector location);
      carla::Waypoint getNextWaypoint(Vector location);
      carla::ActorIds GetManagedCAVsIds();
      double InsertObjects (carla::ObjectsIn object);
      double InsertCV(carla::ObjectIn object);

      double getGTaccuracy(double x, double y, double length, double width, double yaw, int id);

      int getVehicleID(Ptr<Node> node);
      bool InsertObject(carla::ObjectIn object);

      std::vector<int> getManagedConnectedIds();
      std::map<std::string,std::string> getManagedConnectedNodes();//returns map < obj_id, node_id >

      std::string getObjectIdByNodeId(uint32_t targetNodeId);


    private:
      void executeOneTimestep();
      void insertVehicle(carla::Vehicle request);
      void printVehicle(carla::Vehicle vehicle);
      void processVehicleSubscription(int actorId, Vector location, double speed, double angle);

      // update output files with sumo vehicles' info
      void UpdateVehicleFileMap(void);

    private:
      std::shared_ptr<grpc::Channel> m_channel;
      std::unique_ptr<carla::CarlaAdapter::Stub> m_stub;
      double m_updateInterval;
      double m_simTimeLimit;

      int m_port;
      std::string m_opencda_host; //!< Host of OpenCDA CI
      int m_opencda_port; //!< Port of OpenCDA CI
      std::string m_opencda_user; //!< User name for OpenCDA CI ssh connection
      std::string m_opencda_password; //!< Password for OpenCDA CI ssh connection
      std::string m_carla_host;  //!< Host for CARLA server
      int m_carla_port;  //!< Port for CARLA server
      std::string m_carla_user;  //!< User name for CARLA server ssh connection (to be used only in remote mode)
      std::string m_carla_password;  //!< Password for CARLA server ssh connection (to be used only in remote mode)
      bool m_carla_gui; //!< Flag to start CARLA server with GUI
      int m_carla_gpu; //!< GPU ID to be used by CARLA server
      int m_openCDA_gpu; //!< GPU ID to be used by OpenCDA
      int m_carla_tm_port; //!< Port for CARLA Traffic Manager
      std::string m_opencda_config;
      std::string m_carla_home;
      std::string m_opencda_home;
      std::string m_python_interpreter;
      bool m_apply_ml;
      bool m_carla_manual;
      bool m_opencda_manual;

      std::map<int, Ptr<Node>> m_vehMap;
      std::vector<int> m_nonCVIDs;

      std::map<uint32_t, int> m_nodeIdToObjectIdMap;


        EventId m_executeOneTimestepTrigger;

      // function pointers to node include/exclude functions
      STARTUP_FCN m_includeNode;
      SHUTDOWN_FCN m_excludeNode;

      pid_t m_pid;
      pid_t m_carla_pid;
      pid_t m_opencda_pid;
      bool m_adapter_debug; // Flag for toggle execution of adapter within ms-van3t or from outside (for debugging purposes)

      double m_penetration_rate;
      double m_penetration_rate_vru;
      Ptr<UniformRandomVariable> m_randVar; // For penetration rate

      // map every vehicle to an output file
      std::map<std::string, std::ofstream> m_fileMap;
    };

}
#endif // OPENCDA_CLIENT_H
