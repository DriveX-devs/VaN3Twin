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
#include <string>
#include "OpenCDAClient.h"
#include <unistd.h> // For usleep
#include <fcntl.h>
#include <libssh/libssh.h>
#include <cstdlib>

#include "../proto/carla.pb.h"
#include "../proto/carla.pb.h"
#include "../proto/carla.pb.h"

namespace ns3
{
  NS_LOG_COMPONENT_DEFINE("OpenCDAClient");

  TypeId
  OpenCDAClient::GetTypeId(void)
  {
    static TypeId tid =
        TypeId("ns3::OpenCDAClient")
        .SetParent<Object>()
        .SetGroupName ("OpenCDAClient")
    .AddAttribute ("UpdateInterval",
                  "CARLA update interval",
                  DoubleValue (1.0),
                  MakeDoubleAccessor (&OpenCDAClient::m_updateInterval),
                  MakeDoubleChecker <double>())
    .AddAttribute ("CARLAHost",
                  "CARLA host",
                  StringValue ("localhost"),
                  MakeStringAccessor (&OpenCDAClient::m_carla_host),
                  MakeStringChecker ())
    .AddAttribute("CARLAPort",
                  "CARLA port",
                  UintegerValue(3000),
                  MakeUintegerAccessor(&OpenCDAClient::m_carla_port),
                  MakeUintegerChecker<uint32_t>())
    .AddAttribute ("CARLAUser",
                  "CARLA ssh user",
                  StringValue ("carla"),
                  MakeStringAccessor (&OpenCDAClient::m_carla_user),
                  MakeStringChecker ())
    .AddAttribute ("CARLAPassword",
                    "CARLA ssh password",
                    StringValue ("password"),
                    MakeStringAccessor (&OpenCDAClient::m_carla_password),
                    MakeStringChecker ())
    .AddAttribute ("PenetrationRate",
                  "Rate of vehicles, equipped with wireless communication devices",
                  DoubleValue (1.0),
                  MakeDoubleAccessor (&OpenCDAClient::m_penetration_rate),
                  MakeDoubleChecker <double>())
    .AddAttribute ("VRUPenetrationRate",
                   "Rate of vulnerable road users, equipped with wireless communication devices",
                   DoubleValue (1.0),
                   MakeDoubleAccessor (&OpenCDAClient::m_penetration_rate_vru),
                   MakeDoubleChecker <double>())
    .AddAttribute ("OpenCDA_HOME",
                   "OpenCDA",
                   StringValue("ms_van3t_example"),
                   MakeStringAccessor (&OpenCDAClient::m_opencda_home),
                   MakeStringChecker ())
      .AddAttribute("OpenCDACIHost",
                    "Host of OpenCDAClient",
                    StringValue("localhost"),
                    MakeStringAccessor(&OpenCDAClient::m_opencda_host),
                    MakeStringChecker())
      .AddAttribute("CARLATMPort",
                    "CARLA Traffic Manager Port",
                    UintegerValue(8000),
                    MakeUintegerAccessor(&OpenCDAClient::m_carla_tm_port),
                    MakeUintegerChecker<uint32_t>())
      .AddAttribute("OpenCDACIPort",
                    "Port of OpenCDAClient",
                    UintegerValue(1337),
                    MakeUintegerAccessor(&OpenCDAClient::m_opencda_port),
                    MakeUintegerChecker<uint32_t>())
      .AddAttribute("OpenCDACIUser",
                    "User of OpenCDAClient",
                    StringValue("opencda"),
                    MakeStringAccessor(&OpenCDAClient::m_opencda_user),
                    MakeStringChecker())
      .AddAttribute("OpenCDACIPassword",
                    "Password of OpenCDAClient",
                    StringValue("password"),
                    MakeStringAccessor(&OpenCDAClient::m_opencda_password),
                    MakeStringChecker())
    .AddAttribute ("PythonInterpreter",
                   "Python interpreter used to execute OpenCDA",
                   StringValue("python3.7"),
                   MakeStringAccessor (&OpenCDAClient::m_python_interpreter),
                   MakeStringChecker ())
    .AddAttribute ("OpenCDA_config",
                   "Name of opencda script",
                   StringValue("ms_van3t_example"),
                   MakeStringAccessor (&OpenCDAClient::m_opencda_config),
                   MakeStringChecker ())
    .AddAttribute ("ApplyML",
                   "Apply machine learning model for OpenCDA perception module.",
                   BooleanValue (false),
                   MakeBooleanAccessor(&OpenCDAClient::m_apply_ml),
                   MakeBooleanChecker ())
    .AddAttribute ("CARLA_HOME",
                   "Location of CARLA installation",
                   StringValue("CARLA_0.9.12"),
                   MakeStringAccessor (&OpenCDAClient::m_carla_home),
                   MakeStringChecker ())
      .AddAttribute("CARLAGUI",
                    "CARLA GUI",
                    BooleanValue(true),
                    MakeBooleanAccessor(&OpenCDAClient::m_carla_gui),
                    MakeBooleanChecker())
      .AddAttribute("CARLAManual",
                    "CARLA manual execution",
                    BooleanValue(false),
                    MakeBooleanAccessor(&OpenCDAClient::m_carla_manual),
                    MakeBooleanChecker())
      .AddAttribute("OpenCDAManual",
              "OpenCDA manual execution",
              BooleanValue(false),
              MakeBooleanAccessor(&OpenCDAClient::m_opencda_manual),
              MakeBooleanChecker())
      .AddAttribute("CARLAGPU",
                    "CARLA GPU",
                    UintegerValue(0),
                    MakeUintegerAccessor(&OpenCDAClient::m_carla_gpu),
                    MakeUintegerChecker<uint32_t>())
      .AddAttribute("OpenCDAGPU",
                    "OpenCDA GPU",
                    UintegerValue(0),
                    MakeUintegerAccessor(&OpenCDAClient::m_openCDA_gpu),
                    MakeUintegerChecker<uint32_t>());

  ;
    return tid;
  }

  OpenCDAClient::OpenCDAClient(void)
  {
      NS_LOG_FUNCTION(this);
      m_updateInterval = 0.05;
      m_carla_host = "localhost";
      m_opencda_host = "localhost";
      m_opencda_port = 1337;
      m_adapter_debug = false;
      m_carla_port = 2000;
      m_carla_gui = true;
      m_carla_gpu = 0;
      m_openCDA_gpu = 0;
      m_opencda_manual = false;
      m_carla_manual = false;
      m_randVar = CreateObject<UniformRandomVariable>();
      m_randVar->SetAttribute("Min", DoubleValue(0.0));
      m_randVar->SetAttribute("Max", DoubleValue(1.0));
  }

  OpenCDAClient::~OpenCDAClient(void)
  {
      this->stopSimulation ();
      std::cout<<"OpenCDAClient object destroyed." << std::endl;
      NS_LOG_INFO("OpenCDAClient object destroyed.");
  }

  void
  OpenCDAClient::stopSimulation ()
  {
      if (m_opencda_host == "localhost") {
          kill(-m_opencda_pid, SIGKILL);
      }
      else {
          std::string kill_cmd = "ssh " + m_opencda_user + "@" + m_opencda_host + " 'kill -9 " +
              std::to_string(m_opencda_pid) + "'";
          system(kill_cmd.c_str());
      }
      if (m_carla_host == "localhost") {
          kill(-m_carla_pid, SIGKILL);
      }
      else {
          std::string kill_cmd = "ssh " + m_carla_user + "@" + m_carla_host + " 'kill -9 " +
              std::to_string(m_carla_pid) + "'";
          system(kill_cmd.c_str());
          // ssh user@host 'pids=$(lsof -ti :$1); if [ -n "$pids" ]; then for pid in $pids; do echo "Killing process with PID: $pid using port $1"; kill -9 $pid; echo "Process $pid killed."; done; else echo "No process found using port $1."; fi'
          std::string kill_port_cmd =  "ssh " + m_carla_user + "@" + m_carla_host + " 'pids=$(lsof -ti :" + std::to_string(m_carla_port) + "); if [ -n \"$pids\" ]; then for pid in $pids; do echo \"Killing process with PID: $pid using port " + std::to_string(m_carla_port) + "\"; kill -9 $pid; echo \"Process $pid killed.\"; done; else echo \"No process found using port " + std::to_string(m_carla_port) + "\"; fi'";
            system(kill_port_cmd.c_str());
      }
  }
  void
  OpenCDAClient::startCarlaAdapter(std::function<Ptr<Node>(std::string)> includeNode, std::function<void (Ptr<Node>,std::string)> excludeNode)
  {
      m_includeNode = includeNode;
      m_excludeNode = excludeNode;
      if(!m_adapter_debug)
        {
          if (!m_carla_manual) {
              m_carla_pid = fork();

              if (m_carla_pid == 0) {

                  std::string carla_command = "cd "+ m_carla_home +" && ./CarlaUE4.sh -prefernvidia -carla-port=" + std::to_string(m_carla_port) ;
                  if (!m_carla_gui) {
                      carla_command += " -RenderOffScreen";
                  }
                  if(m_carla_gpu != 0 && !m_carla_gui) {
                      carla_command += " -graphicsadapter=" + std::to_string(m_carla_gpu);
                  }
                  setpgid(0, 0); // Create a new process group

                  std::ifstream file("/tmp/carla_output.txt");
                  if (file.good()) {
                      file.close();
                      remove("/tmp/carla_output.txt");
                  }

                  if(m_carla_host != "localhost" ) {
                      std::string ssh_command = "ssh -X " + m_carla_user + "@" + m_carla_host + " '" +  carla_command + " & echo $!'";
                      std::cout <<"Executing: " << (ssh_command).c_str() << std::endl;


                      // Open a temporary file for CARLA output
                      int fd = open("/tmp/carla_output.txt", O_WRONLY | O_CREAT | O_TRUNC, 0644);
                      if (fd < 0) {
                          perror("Opening temp file for CARLA output failed");
                          exit(1);
                      }
                      dup2(fd, STDOUT_FILENO);
                      dup2(fd, STDERR_FILENO);
                      close(fd);

                      int r = execl("/bin/sh", "sh", "-c", (ssh_command).c_str(), NULL);

                      std::cout << "execl returned: " << r << std::endl;

                      if (r == -1) {
                          NS_FATAL_ERROR("Error.system failed");
                      }
                      if (WEXITSTATUS(r) != 0) {
                          NS_FATAL_ERROR("Error.cannot run");
                      }
                      NS_FATAL_ERROR("Error.returned from exec");
                      exit(1);
                  }
                  else {
                      std::cout <<"Executing: " << (carla_command).c_str() << std::endl;

                      // Open a temporary file for CARLA output
                      int fd = open("/tmp/carla_output.txt", O_WRONLY | O_CREAT | O_TRUNC, 0644);
                      if (fd < 0) {
                          perror("Opening temp file for CARLA output failed");
                          exit(1);
                      }
                      dup2(fd, STDOUT_FILENO);
                      dup2(fd, STDERR_FILENO);
                      close(fd);

                      int r = execl("/bin/sh", "sh", "-c", (carla_command).c_str(), NULL);

                      std::cout << "execl returned: " << r << std::endl;

                      if (r == -1) {
                          NS_FATAL_ERROR("Error.system failed");
                      }
                      if (WEXITSTATUS(r) != 0) {
                          NS_FATAL_ERROR("Error.cannot run");
                      }
                      NS_FATAL_ERROR("Error.returned from exec");
                      exit(1);
                  }
              }
          }

          usleep(10000000);

          if (!m_opencda_manual) {
              m_opencda_pid = fork();

              if (m_opencda_pid == 0) {

                  std::string opencda_command = "cd " + m_opencda_home +" && " + m_python_interpreter +" -u opencda.py -t " + m_opencda_config + " -v 0.9.12";
                  opencda_command += " -p "
                                  + std::to_string(m_carla_port)
                                  + " -g " + std::to_string(m_openCDA_gpu)
                                  + " -r " +std::to_string(m_opencda_port)
                                  + " --tm_port " + std::to_string(m_carla_tm_port);
                  if (m_apply_ml)
                  {
                      opencda_command = opencda_command + " --apply_ml";
                  }

                  if (m_carla_host != m_opencda_host) {
                      opencda_command += " -s " + m_carla_host;
                  }

                  if(m_opencda_host != "localhost") {
                      opencda_command = "ssh -X -L "+ std::to_string(m_opencda_port) + ":localhost:" + std::to_string(m_opencda_port) + " "
                          + m_opencda_user + "@" + m_opencda_host + " 'export SUMO_HOME=usr/bin/sumo; " +  opencda_command + " & echo $!'";
                  }

                  setpgid(0, 0); // Create a new process group


                  std::ifstream file("/tmp/opencdaCI_ready");
                  if (file.good()) {
                      file.close();
                      remove("/tmp/opencdaCI_ready");
                  }
                  std::cout << "Executing: " << (opencda_command).c_str() << std::endl;
                  // Open a temporary file for CARLA output
                  std::string file_name = "/tmp/opencda_output_" + std::to_string(m_opencda_port) + ".txt";
                  int fd = open(file_name.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
                  if (fd < 0) {
                      perror("Opening temp file for OpenCDA output failed");
                      exit(1);
                  }
                  dup2(fd, STDOUT_FILENO);
                  dup2(fd, STDERR_FILENO);
                  close(fd);

                  int r = execl("/bin/sh", "sh", "-c", (opencda_command).c_str(), NULL);
                  std::cout << "execl returned: " << r << std::endl;

                  if (r == -1) {
                      NS_FATAL_ERROR("Error.system failed");
                  }
                  if (WEXITSTATUS(r) != 0) {
                      NS_FATAL_ERROR("Error.cannot run");
                  }
                  NS_FATAL_ERROR("Error.returned from exec");

                  exit(1);
              }
          }

        }
      Simulator::Schedule(Seconds(m_updateInterval), &OpenCDAClient::startSimulation, this);
  }

  void
  OpenCDAClient::startSimulation()
  {
      std::cout <<"OpenCDAClient::startSimulation()" << std::endl;
      usleep(10000000);
      bool server_ready = false;
      if (m_opencda_manual) {
          server_ready = true;
      }
      while (!server_ready) {
          // read /tmp/opencda_output.txt to see OpenCDA logs and check if there's a "Server ready" message
          std::string file_name = "/tmp/opencda_output_" + std::to_string(m_opencda_port) + ".txt";
          std::ifstream file(file_name.c_str());
            if (file.good()) {
                std::string line;
                while (std::getline(file, line)) {
                    if (line.find("Server ready") != std::string::npos) {
                        server_ready = true;
                        break;
                    }
                }
                file.close();
            }
          else {
              // File does not exist, wait for 1 second before checking again
              std::cout << "Waiting for OpenCDA Control Interface to be ready." << std::endl;
              usleep(10000000);
          }

      }

      if(m_opencda_host != "localhost") {
          std::string kill_cmd;
          if (m_apply_ml)
            {
              kill_cmd = "ssh " + m_opencda_user + "@" + m_opencda_host
                                     + " 'nvidia-smi --query-compute-apps=pid,name --format=csv,noheader | grep msvan3t | cut -d , -f1 | sort -n | tail -n 1' > /tmp/opencda_pid.txt";
            }
          else
            {
              kill_cmd = "ssh " + m_opencda_user + "@" + m_opencda_host
                                     + " 'ps -u " + m_opencda_user + " -o pid,cmd --no-header | grep python3 | head -n 1 | cut -d \" \" -f1 ' > /tmp/opencda_pid.txt";
            }

          system(kill_cmd.c_str());
          std::ifstream file("/tmp/opencda_pid.txt");
          std::string line;
          if (file.good()) {
              std::getline(file, line);
              file.close();
          }
          m_opencda_pid = std::stoi(line);
      }
      if (m_carla_host != "localhost" && !m_carla_manual) {
          std::string kill_cmd = "ssh " + m_carla_user + "@" + m_carla_host
          + " 'nvidia-smi --query-compute-apps=pid,name --format=csv,noheader | grep Carla | cut -d , -f1 | sort -n | tail -n 1' > /tmp/carla_pid.txt";
          system(kill_cmd.c_str());
          std::ifstream file("/tmp/carla_pid.txt");
          std::string line;
          if (file.good()) {
              std::getline(file, line);
              file.close();
          }
          m_carla_pid = std::stoi(line);
      }

      std::string hostAndPort = "localhost:" + std::to_string(m_opencda_port);

       // ssh tunneling
      m_channel = CreateChannel(hostAndPort, grpc::InsecureChannelCredentials());
      m_stub = carla::CarlaAdapter::NewStub(m_channel);
      auto state = m_channel->GetState(true);

      while (state != GRPC_CHANNEL_READY) {
          if (!m_channel->WaitForStateChange(state, std::chrono::system_clock::now() + std::chrono::seconds(150))) {
              if (m_opencda_host == "localhost") {
                  kill(-m_opencda_pid, SIGKILL);
              }
              else {
                  std::string kill_cmd = "ssh " + m_opencda_user + "@" + m_opencda_host + " 'kill -9 " +
                      std::to_string(m_opencda_pid) + "'";
                  system(kill_cmd.c_str());
              }
              if (m_carla_host == "localhost") {
                  kill(-m_carla_pid, SIGKILL);
              }
              else {
                  std::string kill_cmd = "ssh " + m_carla_user + "@" + m_carla_host + " 'kill -9 " +
                      std::to_string(m_carla_pid) + "'";
                  system(kill_cmd.c_str());
              }
              NS_FATAL_ERROR("Could not connect to gRPC");
          }
          state = m_channel->GetState(true);
          if (state == GRPC_CHANNEL_IDLE) {
              std::cout << "I do have a GRPC_CHANNEL_IDLE in WHILE -> " << state << std::endl;
          }
          else if (state == GRPC_CHANNEL_CONNECTING) {
              std::cout << "I do have a GRPC_CHANNEL_CONNECTING in WHILE -> " << state << std::endl;
          }
          else if (state == GRPC_CHANNEL_READY) {
              std::cout << "I do have a GRPC_CHANNEL_READY in WHILE -> " << state << std::endl;
          }
          else if (state == GRPC_CHANNEL_TRANSIENT_FAILURE) {
              std::cout << "I do have a GRPC_CHANNEL_TRANSIENT_FAILURE in WHILE -> " << state << std::endl;
          }
          else if (state == GRPC_CHANNEL_SHUTDOWN) {
              std::cout << "I do have a GRPC_CHANNEL_SHUTDOWN in WHILE -> " << state << std::endl;
          }
      }

      // Add all the Actors that are already part of the simulation
      //carla::ActorIds actorIds = GetManagedHostIds();
      carla::ActorIds actorIds = GetAllActorsIds();
      for (int i = 0; i < actorIds.actorid_size(); i++) {
          int actorId = actorIds.actorid(i);
          carla::Actor actor = GetActorById(actorId);
          if (actor.itstype() == 0){
              if (m_randVar->GetValue() <= m_penetration_rate_vru || hasCARLALDM (actorIds.actorid(i)))
              {
                  //printVehicle(vehicle);
                  Vector location;
                  location.x = actor.location().x();
                  location.y = actor.location().y();
                  location.z = actor.location().z();

                  std::cout << "Adding pedestrian (" << actorId << ") node with location x = " << location.x
                            << " y = " << location.y << " z = " << location.z << " YAW = " << actor.transform().rotation().yaw() << std::endl;

                  Ptr<ns3::Node> inNode = m_includeNode("ped" + std::to_string(actorId));
                  m_vehMap.insert(std::pair<int, Ptr<Node>>(actorId, inNode));
                  m_nodeIdToObjectIdMap.insert(std::pair<uint32_t, int>(inNode->GetId(), actorId));

                  Ptr<MobilityModel> mob = m_vehMap.at(actorId)->GetObject<MobilityModel>();
                  // set ns3 node position
                  mob->SetPosition(Vector(location.x, location.y, 1.5));
              }
              else
              {
                  m_nonCVIDs.push_back (actorIds.actorid(i));  // To keep track of non Connected Vehicles
              }
          }
          else {
              if (m_randVar->GetValue() <= m_penetration_rate || hasCARLALDM(actorIds.actorid(i))) {
                  //printVehicle(vehicle);
                  Vector location;
                  location.x = actor.location().x();
                  location.y = actor.location().y();
                  location.z = actor.location().z();

                  std::cout << "Adding vehicle (" << actorId << ") node with location x = " << location.x << " y = "
                            << location.y << " z = " << location.z << std::endl;

                  Ptr<ns3::Node> inNode = m_includeNode("veh" + std::to_string(actorId));
                  m_vehMap.insert(std::pair<int, Ptr<Node>>(actorId, inNode));
                  m_nodeIdToObjectIdMap.insert(std::pair<uint32_t, int>(inNode->GetId(), actorId));

                  Ptr<MobilityModel> mob = m_vehMap.at(actorId)->GetObject<MobilityModel>();
                  // set ns3 node position
                  mob->SetPosition(Vector(location.x, location.y, 1.5));
              } else {
                  m_nonCVIDs.push_back(actorIds.actorid(i));  // To keep track of non Connected Vehicles
              }
          }
      }
      executeOneTimestep();
      //this->testInsertVehicle ();
  }

    void
    OpenCDAClient::testInsertVehicle() {
      carla::Vehicle vehicle;
      carla::Transform v = getRandomSpawnPoint();
      carla::Vector* pos = vehicle.mutable_location();

      pos->set_x(v.location().x());
      pos->set_y(v.location().y());
      pos->set_z(v.location().z());

      insertVehicle(vehicle);
  }


  void
  OpenCDAClient::UpdateVehicleFileMap(void)
  {
    NS_LOG_FUNCTION(this);

    try
      {
        // iterate over all sumo vehicles in map
        for (std::map<int, Ptr<Node> >::iterator it = m_vehMap.begin(); it != m_vehMap.end(); ++it)
          {
            // get current sumo vehicle from map
            std::string veh(std::to_string (it->first));

            auto it_files = m_fileMap.find (veh);

            if(it_files == m_fileMap.end())
              {
                std::string filename = "carla_data_" + veh + ".csv";
                // Create and open a new file stream and store it in the map
                std::ofstream file(filename);
                if(!file.is_open()) {
                    std::cerr << "Failed to create file: " << filename << std::endl;
                }
                m_fileMap[veh] = std::move(file);
                m_fileMap[veh]  << "time,posX,posY,speed,acceleration" << std::endl;
              }
            else{
                // get vehicle position from carla
                carla::Vehicle egoVehicle = GetManagedActorById (it->first);

                it_files->second << (double) Simulator::Now().GetMilliSeconds ()<< ","
                                 << egoVehicle.location ().x ()<< ","
                                 << egoVehicle.location ().y () << ","
                                 << sqrt(pow(egoVehicle.speed ().x (),2) + pow(egoVehicle.speed ().y (),2))<< ","
                                 << sqrt(pow(egoVehicle.acceleration ().x (),2) + pow(egoVehicle.acceleration ().y (),2))
                                 << std::endl;
              }
          }
      }
    catch (std::exception& e)
      {
        NS_FATAL_ERROR("SUMO was closed unexpectedly while writing vehicle info into file: " << e.what());
      }

  }

  void
  OpenCDAClient::printVehicle(carla::Vehicle vehicle)
  {
      std::cout << "=========================================" << std::endl;
      std::cout << vehicle.acceleration().x() << std::endl;
      std::cout << "Vehicle" << vehicle.id() << "\n"
          << "Acceleration.x = " << vehicle.acceleration().x() << "\n"
          << "Acceleration.y = " << vehicle.acceleration().y() << "\n"
          << "Acceleration.z = " << vehicle.acceleration().z() << "\n"
          << "Speed.x = " << vehicle.speed().x() << "\n"
          << "Speed.y = " << vehicle.speed().y() << "\n"
          << "Speed.z = " << vehicle.speed().z() << "\n"
          << "Position.x = " << vehicle.location().x() << "\n"
          << "Position.y = " << vehicle.location().y() << "\n"
          << "Position.z = " << vehicle.location().z() <<std::endl;
      std::cout << "=========================================" << std::endl;
  }

  carla::Vehicle
  OpenCDAClient::GetManagedActorById(int actorId)
  {
      carla::Vehicle vehicle;
      carla::Number vehicleId;
      grpc::ClientContext clientContext;
      vehicleId.set_num(actorId);

      grpc::Status status = m_stub->GetManagedActorById(&clientContext, vehicleId, &vehicle);
      if (status.ok()) {
          return vehicle;
      }
      else {
          NS_FATAL_ERROR((std::string("OpenCDAClient::GetManagedActorById() failed with error: " + std::string(status.error_message())).c_str()));
      }
  }
  carla::Actor
  OpenCDAClient::GetActorById(int actorId)
  {
     carla::Actor actor;
     carla::Number vehicleId;
     grpc::ClientContext clientContext;
     vehicleId.set_num(actorId);

     grpc::Status status = m_stub->GetActorById(&clientContext, vehicleId, &actor);
     if (status.ok()) {
         return actor;
     }
     else {
         NS_FATAL_ERROR((std::string("OpenCDAClient::GetActorById() failed with error: " + std::string(status.error_message())).c_str()));
     }
  }
  void
  OpenCDAClient::insertVehicle(carla::Vehicle request)
  {
      carla::Number vehicleId;
      grpc::ClientContext clientContext;
      std::cout << "Inserting vehicle at location: " << request.location().x() << "/" << request.location().y() << "/" << request.location().z() << std::endl;
      grpc::Status status = m_stub->InsertVehicle(&clientContext, request, &vehicleId);
      if (status.ok()) {
          std::cout << "New vehicle inserted succesfully" << std::endl;
      }
      else {
          NS_FATAL_ERROR((std::string("OpenCDAClient::insertVehicle() failed with error: " + std::string(status.error_message())).c_str()));
      }
      Vector pos;
      pos.x = request.location().x();
      pos.y = request.location().y();
      pos.z = request.location().z();

      Ptr<ns3::Node> inNode = m_includeNode(std::to_string(vehicleId.num()));
      m_vehMap.insert(std::pair<int, Ptr<Node>>(vehicleId.num(), inNode));
      m_nodeIdToObjectIdMap.insert(std::pair<uint32_t, int>(inNode->GetId(), vehicleId.num()));
      Ptr<MobilityModel> mob = m_vehMap.at(vehicleId.num())->GetObject<MobilityModel>();
      // set ns3 node position
      mob->SetPosition(Vector(pos.x, pos.y, pos.z));

  }

  carla::Transform
  OpenCDAClient::getRandomSpawnPoint()
  {
      google::protobuf::Empty request;
      carla::Transform response;
      grpc::ClientContext clientContext;
      grpc::Status status = m_stub->GetRandomSpawnPoint(&clientContext, request, &response);
      if (status.ok()) {
          std::cout << "getRandomSpawnPoint:" << response.location().x() << "/" << response.location().y() << "/" << response.location().z() << std::endl;
          return response;
      }
      else {
          NS_FATAL_ERROR((std::string("OpenCDAClient::getRandomSpawnPoint() failed with error: " + std::string(status.error_message())).c_str()));
      }
  }


  void
  OpenCDAClient::executeOneTimestep()
  {
      //std::cout << Simulator::Now().GetSeconds () << ": executeOneTimestep()" << std::endl;
      google::protobuf::Empty responseEmpty;
      carla::Boolean retval;
      google::protobuf::Empty empty;
      grpc::ClientContext clientContext;
      grpc::Status status = m_stub->ExecuteOneTimeStep(&clientContext, empty, &retval);

      //if (status.ok())
      if(!status.ok())
        {
          NS_FATAL_ERROR((std::string("OpenCDAClient::executeOneTimestep() failed with error: " + std::string(status.error_message())).c_str()));
        }
      if (retval.value ()) {

          //carla::ActorIds actors = GetManagedHostIds(); //get only vehicles
          carla::ActorIds actors = GetAllActorsIds(); //
          for (int i = 0; i < actors.actorid_size(); i++) {
              auto actorId = actors.actorid(i);
              //carla::Vehicle v = GetManagedActorById(actorId);
              carla::Actor a = GetActorById(actorId);
              Vector location;
              location.x = a.location().x();
              location.y = a.location().y();
              location.z = a.location().z();
//              double speed = 0; // speed and angle not considered by current ms-van3t channel models
//              double angle = 0;
              double speed = std::sqrt (
                      std::pow(a.speed().x(), 2) + std::pow(a.speed().y(), 2) + std::pow(a.speed().z(), 2)
              ); // speed and angle not considered by current ms-van3t channel models
              double angle = a.heading();
              if (a.itstype()==0){}
              processVehicleSubscription(a.id(), location, speed, angle);
          }
          UpdateVehicleFileMap();
          m_executeOneTimestepTrigger = Simulator::Schedule(Seconds(m_updateInterval), &OpenCDAClient::executeOneTimestep, this);

      }
      else {
          if (m_opencda_host == "localhost") {
              kill(-m_opencda_pid, SIGKILL);
          }
          else {
              std::string kill_cmd = "ssh " + m_opencda_user + "@" + m_opencda_host + " 'kill -9 " +
                  std::to_string(m_opencda_pid) + "'";
              system(kill_cmd.c_str());
          }
          if (m_carla_host == "localhost") {
              kill(-m_carla_pid, SIGKILL);
          }
          else {
              std::string kill_cmd = "ssh " + m_carla_user + "@" + m_carla_host + " 'kill -9 " +
                  std::to_string(m_carla_pid) + "'";
              system(kill_cmd.c_str());
          }
          Simulator::Stop ();

          std::string file_name = "/tmp/opencda_output_" + std::to_string(m_opencda_port) + ".txt";
          std::cout << "OpenCDA simulation finished unexpectedly, check " << file_name.c_str() <<" to see OpenCDA logs" << std::endl;
        }
  }

  void
  OpenCDAClient::processVehicleSubscription(int actorId, Vector location, double speed, double angle)
  {

      if (m_vehMap.find(actorId) == m_vehMap.end()) {
          //New vehicle
          if(std::find(m_nonCVIDs.begin(), m_nonCVIDs.end(), actorId) == m_nonCVIDs.end())
            {
              if (m_randVar->GetValue() <= m_penetration_rate || hasCARLALDM (actorId))
                {
                  carla::Vehicle vehicle = GetManagedActorById(actorId);
                  printVehicle(vehicle);
                  Vector location;
                  location.x = vehicle.location().x();
                  location.y = vehicle.location().y();
                  location.z = vehicle.location().z();
                  //std::cout << "Adding vehicle with location x = " << location.x << " y = " << location.y << " z = " << location.z << std::endl;

                  Ptr<ns3::Node> inNode = m_includeNode(std::to_string(actorId));
                  m_vehMap.insert(std::pair<int, Ptr<Node>>(actorId, inNode));
                    m_nodeIdToObjectIdMap.insert(std::pair<uint32_t, int>(inNode->GetId(), actorId));

                  Ptr<MobilityModel> mob = m_vehMap.at(actorId)->GetObject<MobilityModel>();
                  // set ns3 node position
                  mob->SetPosition(Vector(location.x, location.y, location.z));
                }
              else
                {
                  m_nonCVIDs.push_back (actorId);  // To keep track of non Connected Vehicles
                }
            }
          // IF found in nonCVIDs --> we skip
      }
      else {
          // Node existed - update position
          Ptr<MobilityModel> mob = m_vehMap.at(actorId)->GetObject<MobilityModel>();
          // set ns3 node position
          mob->SetPosition(Vector(location.x, location.y, location.z));
      }
  }

  carla::ActorIds
  OpenCDAClient::GetManagedHostIds()
  {
      carla::ActorIds actorIds;
      google::protobuf::Empty empty;
      grpc::ClientContext clientContext;
      grpc::Status status = m_stub->GetManagedActorsIds(&clientContext, empty, &actorIds);

      if (!status.ok()) {
          NS_FATAL_ERROR((std::string("OpenCDAClient::GetManagedHostIds() failed with error: " + std::string(status.error_message())).c_str()));
      }
      return actorIds;
  }

  carla::ActorIds
  OpenCDAClient::GetAllActorsIds()
  {
      carla::ActorIds actorIds;
      google::protobuf::Empty empty;
      grpc::ClientContext clientContext;
      grpc::Status status = m_stub->GetAllActorsIds(&clientContext, empty, &actorIds);

      if (!status.ok()) {
          NS_FATAL_ERROR((std::string("OpenCDAClient::GetAllActorsIds() failed with error: " + std::string(status.error_message())).c_str()));
      }
      return actorIds;
  }


  carla::Vector
  OpenCDAClient::getCartesian (double lon, double lat)
  {
      carla::Vector pos, geoPos;
      grpc::ClientContext clientContext;
      geoPos.set_x (lat);
      geoPos.set_y (lon);
      geoPos.set_z (0.0);

      grpc::Status status = m_stub->GetCartesian (&clientContext, geoPos, &pos);
      if (status.ok()) {
          return pos;
      }
      else {
          NS_FATAL_ERROR((std::string("OpenCDAClient::GetCartesian() failed with error: " + std::string(status.error_message())).c_str()));
      }
  }

  carla::Vector
  OpenCDAClient::getGeo (double x, double y)
  {
      carla::Vector pos, cartesianPos;
      grpc::ClientContext clientContext;
      cartesianPos.set_x (x);
      cartesianPos.set_y (y);
      cartesianPos.set_z (0.0);

      grpc::Status status = m_stub->GetGeo (&clientContext, cartesianPos, &pos);
      if (status.ok()) {
          return pos;
      }
      else {
          NS_FATAL_ERROR((std::string("OpenCDAClient::GetGeo() failed with error: " + std::string(status.error_message())).c_str()));
      }
  }

carla::Waypoint
OpenCDAClient::getWaypoint(Vector location)
{
      carla::Waypoint waypoint;
      grpc::ClientContext client_context;
      carla::Vector carla_position;
      carla_position.set_x(location.x);
      carla_position.set_y(location.y);
      carla_position.set_z(location.z);

      grpc::Status status = m_stub->GetCarlaWaypoint(&client_context, carla_position, &waypoint);

      if (status.ok()) {
          return waypoint;
      }
      else {
          NS_FATAL_ERROR((std::string("OpenCDAClient::GetCarlaWaypoint() failed with error: " + std::string(status.error_message())).c_str()));
      }
}

carla::Waypoint
OpenCDAClient::getNextWaypoint(Vector location) {
      carla::Waypoint waypoint;
      grpc::ClientContext client_context;
      carla::Vector carla_position;
      carla_position.set_x(location.x);
      carla_position.set_y(location.y);
      carla_position.set_z(location.z);

      grpc::Status status = m_stub->GetNextCarlaWaypoint(&client_context, carla_position, &waypoint);

      if (status.ok()) {
          return waypoint;
      }
      else {
          NS_FATAL_ERROR((std::string("OpenCDAClient::GetCarlaWaypointNext() failed with error: " + std::string(status.error_message())).c_str()));
      }
}

  double
  OpenCDAClient::getSpeed (int id)
  {
    carla::Vehicle vehicle = GetManagedActorById(id);
    double speed = sqrt(pow(vehicle.speed ().x (),2) + pow(vehicle.speed ().y (),2));
    return speed;
  }

  double
  OpenCDAClient::getHeading (int id)
  {
    carla::Vehicle vehicle = GetManagedActorById(id);
    return vehicle.heading ();
  }

  bool
  OpenCDAClient::hasCARLALDM (int id)
  {
    carla::Boolean retval;
    carla::Number vehicleId;
    vehicleId.set_num (id);
    grpc::ClientContext clientContext;

    grpc::Status status = m_stub->hasLDM (&clientContext, vehicleId, &retval);

    return retval.value ();
  }

  carla::Objects
  OpenCDAClient::getDetectedObjects (int id)
  {
    carla::Objects retObjects;
    carla::Number vehicleId;
    vehicleId.set_num (id);
    grpc::ClientContext clientContext;

    grpc::Status status = m_stub->GetActorLDM (&clientContext, vehicleId, &retObjects);

    return retObjects;
  }

    double
    OpenCDAClient::getGTaccuracy(double x, double y, double length, double width, double yaw, int id) {
      carla::ObjectMinimal object;
      carla::Vector* location = object.mutable_transform()->mutable_location();
      carla::Rotation* rotation = object.mutable_transform()->mutable_rotation();
      grpc::ClientContext clientContext;
      carla::DoubleValue accuracy;
      double ret_accuracy = 0.0;

      location->set_x(x);
      location->set_y(y);
      location->set_z(1.5);
      rotation->set_pitch(0);
      rotation->set_yaw(yaw);
      rotation->set_roll(0);
      object.set_id(id);
      object.set_length(length);
      object.set_width(width);

      grpc::Status status = m_stub->GetGTaccuracy(&clientContext, object, &accuracy);

      if (!status.ok()) {
          NS_FATAL_ERROR((std::string("OpenCDAClient::getGTaccuracy() failed with error: " + std::string(status.error_message())).c_str()));
      }

      ret_accuracy = accuracy.value();
      return ret_accuracy;
  }

  int
  OpenCDAClient::getVehicleID (Ptr<Node> node)
  {
    int retval = -1;
    for (std::map<int, Ptr<Node>>::iterator it = m_vehMap.begin (); it != m_vehMap.end (); it++)
      {
        if (it->second == node)
          {
            retval = it->first;
            break;
          }
      }
    return retval;
  }

  bool
  OpenCDAClient::InsertObject (carla::ObjectIn object)
  {
    grpc::ClientContext clientContext;
    carla::Number ret;
    bool ret_b = false;
    grpc::Status status = m_stub->InsertObject (&clientContext, object, &ret);
    if (!status.ok()) {
        NS_FATAL_ERROR((std::string("OpenCDAClient::InsertObject () failed with error: " + std::string(status.error_message())).c_str()));
    }
    if(ret.num () == 1)
      ret_b = true;

    return ret_b;
  }

    double
  OpenCDAClient::InsertCV(carla::ObjectIn object)
  {
      grpc::ClientContext clientContext;
      carla::DoubleValue ret;

      grpc::Status status = m_stub->InsertCV (&clientContext, object, &ret);
      if (!status.ok()) {
          NS_FATAL_ERROR((std::string("OpenCDAClient::InsertObject () failed with error: " + std::string(status.error_message())).c_str()));
      }
      return ret.value ();
  }
    double
  OpenCDAClient::InsertObjects (carla::ObjectsIn objects)
  {
      grpc::ClientContext clientContext;
      carla::DoubleValue ret;

      grpc::Status status = m_stub->InsertObjects (&clientContext, objects, &ret);
      if (!status.ok()) {
          NS_FATAL_ERROR((std::string("OpenCDAClient::InsertObject () failed with error: " + std::string(status.error_message())).c_str()));
      }

      return ret.value ();
  }

    void
    OpenCDAClient::setControl(int id, double speed, Vector position, double acceleration) {
      grpc::ClientContext clientContext;
      carla::Control control;
      carla::Vector* pos = control.mutable_waypoint();

      pos->set_x(position.x);
      pos->set_y(position.y);
      pos->set_z(position.z);

      control.set_id(id);
      control.set_speed(speed);
      control.set_acceleration(acceleration);

      google::protobuf::Empty responseEmpty;
      grpc::Status status = m_stub->SetControl(&clientContext, control, &responseEmpty);

      if (!status.ok()) {
          NS_FATAL_ERROR((std::string("OpenCDAClient::setControl() failed with error: " + std::string(status.error_message())).c_str()));
      }
  }


  carla::ActorIds
  OpenCDAClient::GetManagedCAVsIds()
  {
    carla::ActorIds actorIds;
    google::protobuf::Empty empty;
    grpc::ClientContext clientContext;
    grpc::Status status = m_stub->GetManagedCAVsIds (&clientContext, empty, &actorIds);

    if (!status.ok()) {
        NS_FATAL_ERROR((std::string("OpenCDAClient::GetManagedCAVsIds() failed with error: " + std::string(status.error_message())).c_str()));
    }
    return actorIds;

  }

  std::vector<int>
  OpenCDAClient::getManagedConnectedIds()
  {
    std::vector<int> ret_ids;

    for(auto it = m_vehMap.begin (); it != m_vehMap.end (); it++){
        ret_ids.push_back (it->first);
      }

    return ret_ids;
  }

  std::map<std::string,std::string>
  OpenCDAClient::getManagedConnectedNodes()
  {
      std::map<std::string,std::string> ret_map;

      for(auto it = m_vehMap.begin (); it != m_vehMap.end (); it++){
          ret_map.insert({std::to_string(it->first), std::to_string (it->second->GetId())});
      }

      return ret_map;
  }
  std::string
  OpenCDAClient::getObjectIdByNodeId(uint32_t targetNodeId) {
        // Using m_nodeIdToObjectIdMap for efficient lookup
        auto it = m_nodeIdToObjectIdMap.find(targetNodeId);

        if (it != m_nodeIdToObjectIdMap.end()) {
            // Found: it->first is the node ID, it->second is the object ID
            return std::to_string(it->second);
        }

        // Not found
        return std::to_string(-1);
    }
}

