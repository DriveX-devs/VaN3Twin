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

* Created by:
    *  Carlos Mateo Risma Carletti, Politecnico di Torino (carlosrisma@gmail.com)
*/


#include "opencda-sensor.h"
#include <cmath>
#include <sys/types.h>
#include <sys/stat.h>
#include "ns3/dbscan.h"
#include <iostream>
#include <fstream>
#include <string>

namespace ns3 {


    OpenCDASensor::OpenCDASensor() {
        m_id = 0;
        m_string_id = "";
        m_opencda_client = NULL;

    }

    OpenCDASensor::OpenCDASensor(Ptr<OpenCDAClient> opencda_client, std::string id) {
        m_id = std::stoi(id);
        m_string_id = id;
        m_opencda_client = opencda_client;

    }

    OpenCDASensor::OpenCDASensor(Ptr<OpenCDAClient> opencda_client, int id) {
        m_id = id;
        m_string_id = std::to_string(id);
        m_opencda_client = opencda_client;

    }

    OpenCDASensor::~OpenCDASensor() {
        Simulator::Cancel(m_event_updateDetectedObjects);
    }

    double
    OpenCDASensor::insertCV(vehicleData_t vehData) {

        double ret = m_opencda_client->InsertCV(createCARLAObjectIn(vehData));
        m_lastInserted[vehData.stationID] = vehData.timestamp_us;

        return ret;
    }

    double OpenCDASensor::insertPOs(std::vector<vehicleData_t> POs, uint64_t fromId) {

        carla::ObjectsIn objectsIn;
        for (auto it = POs.begin(); it != POs.end(); it++) {
            carla::Object object;
            carla::Vector speed;
            carla::Vector acc;
            carla::Vector location;
            carla::Rotation rotation;
            carla::Transform transform;

            object.set_id(it->stationID);
            object.set_dx(static_cast<double>(it->xDistance.getData()));
            object.set_dy(static_cast<double>(it->yDistance.getData()));

            acc.set_x(0.0);
            acc.set_y(0.0);
            acc.set_z(0.0);
            object.set_allocated_acceleration(&acc);  // Temporary assignment

            speed.set_x(it->speed_ms * cos(it->heading * M_PI / 180.0));
            speed.set_y(it->speed_ms * sin(it->heading * M_PI / 180.0));
            speed.set_z(0.0);
            object.set_allocated_speed(&speed);  // Temporary assignment

            object.set_length(static_cast<double>(it->vehicleLength.getData()) / DECI);
            object.set_width(static_cast<double>(it->vehicleWidth.getData()) / DECI);
            object.set_onsight(false);
            object.set_tracked(false);
            object.set_timestamp(it->timestamp_us / 1000);
            object.set_confidence(it->confidence.getData());

            if (it->heading > 180) {
                object.set_yaw(it->heading - 360);
            } else {
                object.set_yaw(it->heading);
            }

            object.set_detected(it->detected);

            carla::Vector pos = m_opencda_client->getCartesian(it->lon, it->lat);
            location.set_x(pos.x());
            location.set_y(pos.y());
            location.set_z(pos.z());
            rotation.set_pitch(0);
            rotation.set_yaw(it->heading);
            rotation.set_roll(0);
            transform.set_allocated_location(&location);  // Temporary assignment
            transform.set_allocated_rotation(&rotation);  // Temporary assignment
            object.set_allocated_transform(&transform);  // Temporary assignment

            objectsIn.add_cpmobjects()->CopyFrom(object);

            // Remove temporary assignments to avoid memory management issues
            object.release_acceleration();
            object.release_speed();
            transform.release_location();
            transform.release_rotation();
            object.release_transform();

            m_lastInserted[it->stationID] = it->timestamp_us;
        }

        // Print values of cpmobjects for debugging
        // for (int i = 0; i < objectsIn.cpmobjects_size(); i++) {
        //     carla::Object obj = objectsIn.cpmobjects(i);
        //     std::cout << "Object " << i << ":\n";
        //     std::cout << "ID: " << obj.id() << "\n";
        //     std::cout << "dx: " << obj.dx() << "\n";
        //     std::cout << "dy: " << obj.dy() << "\n";
        //     std::cout << "acceleration: (" << obj.acceleration().x() << ", " << obj.acceleration().y() << ", " << obj.acceleration().z() << ")\n";
        //     std::cout << "speed: (" << obj.speed().x() << ", " << obj.speed().y() << ", " << obj.speed().z() << ")\n";
        //     std::cout << "length: " << obj.length() << "\n";
        //     std::cout << "width: " << obj.width() << "\n";
        //     std::cout << "onsight: " << obj.onsight() << "\n";
        //     std::cout << "tracked: " << obj.tracked() << "\n";
        //     std::cout << "timestamp: " << obj.timestamp() << "\n";
        //     std::cout << "confidence: " << obj.confidence() << "\n";
        //     std::cout << "yaw: " << obj.yaw() << "\n";
        //     std::cout << "detected: " << obj.detected() << "\n";
        //     std::cout << "location: (" << obj.transform().location().x() << ", " << obj.transform().location().y() << ", " << obj.transform().location().z() << ")\n";
        //     std::cout << "rotation: (" << obj.transform().rotation().pitch() << ", " << obj.transform().rotation().yaw() << ", " << obj.transform().rotation().roll() << ")\n";
        // }

        objectsIn.set_fromid(fromId);
        objectsIn.set_egoid(m_id);
        double ret = m_opencda_client->InsertObjects(objectsIn);
        return ret;

    }

    const char *itsTypeToString(itsType_t type) {
        switch (type) {
            case itsType_vehicle:
                return "Vehicle";
            case itsType_pedestrian:
                return "Pedestrian";
            case itsType_bicycle:
                return "Bicycle";
            case itsType_motorcycle:
                return "Motorcycle";
            case itsType_unknown:
                return "Unknown";
            default:
                return "Invalid Type";
        }
    }

    void
    OpenCDASensor::updateDetectedObjects() {
        if (m_opencda_client->hasCARLALDM(m_id)) {
            // carla::Objects objects = m_opencda_client->getDetectedObjects(m_id);
            carla::Vehicle egoVehicle = m_opencda_client->GetManagedActorById(m_id);
            std::vector<uint64_t> carla_ids;
            std::vector<LDM::returnedVehicleData_t> LDM_POs, LDM_CVs, LDM_CVRUs;


            // First we insert all entries of received data (CAM, CPM or VAM) so they get matched with the entries in OpenCDA's LDM
            if (m_LDM->getAllPOs(LDM_POs)) {
                std::vector<LDM::returnedVehicleData_t>::iterator it;
                for (it = LDM_POs.begin(); it != LDM_POs.end(); it++) {
                    if (it->vehData.perceivedBy.getData() != m_id) {
                        if (m_lastInserted.find(it->vehData.stationID) == m_lastInserted.end()) {
                            insertObject(it->vehData);
                            m_lastInserted[it->vehData.stationID] = it->vehData.timestamp_us;
                        } else if (m_lastInserted[it->vehData.stationID] < it->vehData.timestamp_us) {
                            insertObject(it->vehData);
                            m_lastInserted[it->vehData.stationID] = it->vehData.timestamp_us;
                        }
                        // If the timestamp in m_lastInserted is the same as current one, we skip
                    }

                }
            }
            if (m_LDM->getAllCVs(LDM_CVs)) {
                std::vector<LDM::returnedVehicleData_t>::iterator it;
                for (it = LDM_CVs.begin(); it != LDM_CVs.end(); it++) {
                    if (m_lastInserted.find(it->vehData.stationID) == m_lastInserted.end()) {
                        insertObject(it->vehData);
                        m_lastInserted[it->vehData.stationID] = it->vehData.timestamp_us;
                    } else if (m_lastInserted[it->vehData.stationID] < it->vehData.timestamp_us) {
                        insertObject(it->vehData);
                        m_lastInserted[it->vehData.stationID] = it->vehData.timestamp_us;
                    }
                    // If the timestamp in m_lastInserted is the same as current one, we skip
                }
            }
            if (m_LDM->getAllCVRUs(LDM_CVRUs)) {
                std::vector<LDM::returnedVehicleData_t>::iterator it;
                for (it = LDM_CVRUs.begin(); it != LDM_CVRUs.end(); it++) {
                    if (m_lastInserted.find(it->vehData.stationID) == m_lastInserted.end()) {
                        insertObject(it->vehData);
                        m_lastInserted[it->vehData.stationID] = it->vehData.timestamp_us;
                    } else if (m_lastInserted[it->vehData.stationID] < it->vehData.timestamp_us) {
                        insertObject(it->vehData);
                        m_lastInserted[it->vehData.stationID] = it->vehData.timestamp_us;
                    }
                    // If the timestamp in m_lastInserted is the same as current one, we skip
                }
            }

            int num_perceived_ped = 0;
            carla::Objects objects = m_opencda_client->getDetectedObjects(m_id);
            // Once all entries from V2X messages are inserted and matched in OpenCDA's LDM, we sync both LDMs
            for (int i = 0; i < objects.objects_size(); i++) {
                carla::Object obj = objects.objects(i);
                carla_ids.push_back((uint64_t) obj.id());
                LDM::returnedVehicleData_t retveh = {0};
                LDM::LDM_error_t retval = m_LDM->lookup(obj.id(), retveh);


                if (retval == LDM::LDM_ITEM_NOT_FOUND || (retval == LDM::LDM_OK && retveh.vehData.detected) || (retval == LDM::LDM_OK && retveh.vehData.VAM) ){
                    vehicleData_t objectData = {0};
                    objectData.detected = obj.detected();
                    objectData.ID = std::to_string(obj.id());
                    objectData.stationID = obj.id();


                    objectData.elevation = AltitudeValue_unavailable;

                    objectData.speed_ms = sqrt(pow(obj.speed().x(), 2) + pow(obj.speed().y(), 2));
                    objectData.xSpeedAbs = OptionalDataItem<long>(long(obj.speed().x() * CENTI));
                    objectData.ySpeedAbs = OptionalDataItem<long>(long(obj.speed().y() * CENTI));
                    //objectData.heading = atan2(obj.speed ().y (),obj.speed ().x ()) * (180.0 / M_PI);
                    objectData.heading = obj.yaw();
                    objectData.angle.setData(objectData.heading * DECI);
                    // double yawDiff = obj.yaw () - egoVehicle.heading ();
                    // if(yawDiff < 0)
                    //   objectData.angle.setData ((yawDiff+360)*DECI);
                    // else
                    //   objectData.angle.setData (yawDiff*DECI);
                    objectData.itsType = (itsType_t) obj.itstype();
                    objectData.timestamp_us = obj.timestamp() * 1000; //timestamp from CARLA is in milliseconds

                    if (objectData.itsType != itsType_pedestrian){
                        objectData.camTimestamp = objectData.timestamp_us;
                    } else {
                        objectData.camTimestamp = 0;
                    }
                    objectData.vehicleWidth = OptionalDataItem<long>(long(obj.width() * DECI));
                    objectData.vehicleLength = OptionalDataItem<long>(long(obj.length() * DECI));
                    objectData.xDistAbs = OptionalDataItem<long>(long(obj.dx() * CENTI));//X distance in centimeters
                    objectData.yDistAbs = OptionalDataItem<long>(long(obj.dy() * CENTI));//Y Distance in centimeters


                    objectData.x = obj.transform().location().x();
                    objectData.y = obj.transform().location().y();
                    carla::Vector objPos = m_opencda_client->getGeo(obj.transform().location().x(),
                                                                    obj.transform().location().y());
                    objectData.lat = objPos.x();
                    objectData.lon = objPos.y();


                    objectData.xSpeed = OptionalDataItem<long>(
                            (long) (obj.speed().x() - egoVehicle.speed().x()) * CENTI);
                    objectData.ySpeed = OptionalDataItem<long>(
                            (long) (obj.speed().y() - egoVehicle.speed().y()) * CENTI);
                    //speed_ms was* a relative velocity
//                    objectData.speed_ms = sqrt(pow(obj.speed().x() - egoVehicle.speed().x(), 2) +
//                                               pow(obj.speed().y() - egoVehicle.speed().y(), 2));

                    objectData.speed_ms = sqrt(pow(obj.speed().x(), 2) + pow(obj.speed().y(), 2));
                    objectData.longitudinalAcceleration = OptionalDataItem<long>(
                            long(sqrt(pow(obj.acceleration().x(), 2) + pow(obj.acceleration().y(), 2))));
                    objectData.confidence = long(obj.confidence() * CENTI); //Distance based confidence


                    // if retval is OK and perceivedBy is different from m_id, we update the perceivedBy field
                    if (retval == LDM::LDM_OK &&
                        (retveh.vehData.perceivedBy.getData() != m_id || obj.perceivedby() == -1)) {
                        objectData.perceivedBy = retveh.vehData.perceivedBy;
                    } else {
                        objectData.perceivedBy = OptionalDataItem<long>((long) obj.perceivedby());
                    }

                    if (objectData.perceivedBy.getData() == -1) {
                        continue;
                    }
                    //objectData.perceivedBy = OptionalDataItem<long> ((long) obj.perceivedby ());
                    if (objectData.itsType == itsType_pedestrian) {
                        objectData.stationType = StationType_pedestrian;
                    } else if(objectData.itsType == itsType_vehicle) {
                        objectData.stationType = StationType_passengerCar;
                    } else if(objectData.itsType == itsType_motorcycle) {
                        objectData.stationType = StationType_unknown;
                    } else if(objectData.itsType == itsType_bicycle) {
                         objectData.stationType = StationType_unknown;
                     }

                    if (retveh.vehData.lastCPMincluded.isAvailable())
                        objectData.lastCPMincluded.setData(retveh.vehData.lastCPMincluded.getData());

                    if (retveh.vehData.associatedCVs.isAvailable())
                        objectData.associatedCVs = OptionalDataItem<std::vector<long>>(
                                retveh.vehData.associatedCVs.getData());

                    objectData.GTaccuracy = OptionalDataItem<double>(
                            m_opencda_client->getGTaccuracy(obj.transform().location().x(),
                                                            obj.transform().location().y(),
                                                            obj.length(), obj.width(), obj.yaw(), obj.id()));

                    retval = m_LDM->insert(objectData);

                    float distance = sqrt(
                            pow(obj.transform().location().x() - egoVehicle.transform().location().x(), 2) +
                            pow(obj.transform().location().y() - egoVehicle.transform().location().y(), 2));
                    float objectVelocity = sqrt(pow(obj.speed().x(), 2) + pow(obj.speed().y(), 2));
                    if (objectData.detected && retveh.vehData.perceivedBy.getData() == m_id) {
                        std::cout << "[" << Simulator::Now().GetSeconds() << "] " << "Vehicle "
                                  << m_id << " detected, new LDM object:"
                                  << "[" << objectData.stationID
                                  << " - " << itsTypeToString(objectData.itsType) << "] -- > Heading: "
                                  << objectData.heading
                                  << ", Pos: [" << obj.transform().location().x() << ", "
                                  << obj.transform().location().y()
                                  << "], Distance from ego vehicle: " << std::fixed << std::setprecision(2) << distance
                                  << " m, Relative Speed: " << objectData.speed_ms << " m/s, "
                                  << itsTypeToString(objectData.itsType)
                                  << " speed: " << objectVelocity << " m/s " << std::endl;
                        if (objectData.itsType == itsType_pedestrian && retveh.vehData.perceivedBy.getData() == m_id) {
                            num_perceived_ped++;
                        }
                    }

                    if (retval != LDM::LDM_OK && retval != LDM::LDM_UPDATED) {
                        std::cerr << "Warning! Insert on the database for detected object " << objectData.ID
                                  << "failed!" << std::endl;
                    }
                }
            }
            g_perceived_pedestrians[m_string_id].push_back(num_perceived_ped);
            // After sync, we delete from ns-3 LDM leftover detected objects
            LDM_POs = {};
            if (m_LDM->getAllPOs(LDM_POs)) {
                std::vector<LDM::returnedVehicleData_t>::iterator it;
                for (it = LDM_POs.begin(); it != LDM_POs.end(); it++) {
                if (it->vehData.itsType == itsType_vehicle) {
                    if (std::find(carla_ids.begin(), carla_ids.end(), it->vehData.stationID) == carla_ids.end()) {
                        LDM::LDM_error_t retval = m_LDM->remove(it->vehData.stationID);
                        // We asume that this Perceived Objects has been matched with another one
                        if (retval == LDM::LDM_OK) {
                            std::cout << "Removed from ns3 LDM a PO that we are not perceiving anymore -> ID: "
                                      << it->vehData.stationID << std::endl;
                        }
                    }
                }
                }
            }
            if (m_clustering) {
                createCluster();
            }
            if (m_GUI) {
                updateGUI();
            }

            m_event_updateDetectedObjects = Simulator::Schedule(MilliSeconds(100),
                                                                &OpenCDASensor::updateDetectedObjects, this);
        }
    }

    void OpenCDASensor::insertObject(vehicleData_t vehData) {
        carla::ObjectIn toSend;
        carla::Object *object = toSend.mutable_object();

        object->set_id(vehData.stationID);
//    object->set_dx(((double)vehData.xDistance.getData()) / CENTI);
//    object->set_dy(((double)vehData.yDistance.getData()) / CENTI);
        object->set_dx(((double) vehData.xDistAbs.getData()) / CENTI);
        object->set_dy(((double) vehData.yDistAbs.getData()) / CENTI);
        carla::Vector *acc = object->mutable_acceleration();
        acc->set_x(0.0);
        acc->set_y(0.0);
        acc->set_z(0.0);

        carla::Vector *speed = object->mutable_speed();
        speed->set_x(vehData.speed_ms * cos(vehData.heading * M_PI / 180.0));
        speed->set_y(vehData.speed_ms * sin(vehData.heading * M_PI / 180.0));
        speed->set_z(0.0);

        object->set_length(((double) vehData.vehicleLength.getData()) / DECI);
        object->set_width(((double) vehData.vehicleWidth.getData()) / DECI);
        object->set_onsight(false); // We always insert either objects from CPM or CAMs
        object->set_tracked(false);
        object->set_timestamp(vehData.timestamp_us / 1000);
        object->set_confidence(vehData.confidence.getData());

        if (vehData.heading > 180) {
            object->set_yaw(vehData.heading - 360);
        } else {
            object->set_yaw(vehData.heading);
        }

        object->set_detected(vehData.detected);

        object->set_itstype(vehData.itsType);

        object->set_vam(vehData.VAM);

        carla::Vector pos = m_opencda_client->getCartesian(vehData.lon, vehData.lat);

        carla::Transform *transform = object->mutable_transform();
        carla::Vector *location = transform->mutable_location();
        carla::Rotation *rotation = transform->mutable_rotation();

        location->set_x(pos.x());
        location->set_y(pos.y());
        location->set_z(pos.z());

        rotation->set_pitch(0);
        rotation->set_yaw(vehData.heading);
        rotation->set_roll(0);


        if (vehData.detected && vehData.perceivedBy.isAvailable()) {
            toSend.set_fromid(vehData.perceivedBy.getData());
        } else if (!vehData.detected) {
            toSend.set_fromid(vehData.stationID);
        } else {
            toSend.set_fromid(m_id);
        }

        toSend.set_egoid(m_id);
        m_opencda_client->InsertObject(toSend);
    }


    carla::ObjectIn OpenCDASensor::createCARLAObjectIn(vehicleData_t vehData) {
        carla::ObjectIn toSend;
        carla::Object *object = toSend.mutable_object();

        object->set_id(vehData.stationID);
        object->set_dx(((double) vehData.xDistance.getData()) / CENTI);
        object->set_dy(((double) vehData.yDistance.getData()) / CENTI);

        carla::Vector *acc = object->mutable_acceleration();
        acc->set_x(0.0);
        acc->set_y(0.0);
        acc->set_z(0.0);

        carla::Vector *speed = object->mutable_speed();
        speed->set_x(vehData.speed_ms * cos(vehData.heading * M_PI / 180.0));
        speed->set_y(vehData.speed_ms * sin(vehData.heading * M_PI / 180.0));
        speed->set_z(0.0);

        object->set_length(((double) vehData.vehicleLength.getData()) / DECI);
        object->set_width(((double) vehData.vehicleWidth.getData()) / DECI);
        object->set_onsight(false); // We always insert either objects from CPM or CAMs
        object->set_tracked(false);
        object->set_timestamp(vehData.timestamp_us / 1000);
        object->set_confidence(vehData.confidence.getData());

        if (vehData.heading > 180) {
            object->set_yaw(vehData.heading - 360);
        } else {
            object->set_yaw(vehData.heading);
        }

        object->set_detected(vehData.detected);

        carla::Vector pos = m_opencda_client->getCartesian(vehData.lon, vehData.lat);

        carla::Transform *transform = object->mutable_transform();
        carla::Vector *location = transform->mutable_location();
        carla::Rotation *rotation = transform->mutable_rotation();

        location->set_x(pos.x());
        location->set_y(pos.y());
        location->set_z(pos.z());

        rotation->set_pitch(0);
        rotation->set_yaw(vehData.heading);
        rotation->set_roll(0);

        if (vehData.detected && vehData.perceivedBy.isAvailable()) {
            toSend.set_fromid(vehData.perceivedBy.getData());
        } else if (!vehData.detected) {
            toSend.set_fromid(vehData.stationID);
        } else {
            toSend.set_fromid(m_id);
        }

        toSend.set_egoid(m_id);
        return toSend;
    }


    carla::Object OpenCDASensor::createCARLAObject(vehicleData_t vehData) {
        carla::Object object;
        object.set_id(vehData.stationID);
        object.set_dx(((double) vehData.xDistance.getData()) / CENTI);
        object.set_dy(((double) vehData.yDistance.getData()) / CENTI);

        carla::Vector *acc = object.mutable_acceleration();
        acc->set_x(0.0);
        acc->set_y(0.0);
        acc->set_z(0.0);

        carla::Vector *speed = object.mutable_speed();
        speed->set_x(vehData.speed_ms * cos(vehData.heading * M_PI / 180.0));
        speed->set_y(vehData.speed_ms * sin(vehData.heading * M_PI / 180.0));
        speed->set_z(0.0);

        object.set_length(((double) vehData.vehicleLength.getData()) / DECI);
        object.set_width(((double) vehData.vehicleWidth.getData()) / DECI);
        object.set_onsight(false);
        object.set_tracked(false);
        object.set_timestamp(vehData.timestamp_us / 1000);
        object.set_confidence(vehData.confidence.getData());

        if (vehData.heading > 180) {
            object.set_yaw(vehData.heading - 360);
        } else {
            object.set_yaw(vehData.heading);
        }

        object.set_detected(vehData.detected);

        carla::Vector pos = m_opencda_client->getCartesian(vehData.lon, vehData.lat);
        carla::Transform *transform = object.mutable_transform();
        carla::Vector *location = transform->mutable_location();
        carla::Rotation *rotation = transform->mutable_rotation();

        location->set_x(pos.x());
        location->set_y(pos.y());
        location->set_z(pos.z());
        rotation->set_pitch(0);
        rotation->set_yaw(vehData.heading);
        rotation->set_roll(0);

        return object;
    }


    // Function to calculate the intersection area of two polygons
    double
    OpenCDASensor::calculateIoU(const cv::RotatedRect &rect1, const cv::RotatedRect &rect2) {
        std::vector<cv::Point2f> intersection;
        int result = cv::rotatedRectangleIntersection(rect1, rect2, intersection);

        if (result == cv::INTERSECT_NONE) {
            return 0.0; // No intersection
        } else if (result == cv::INTERSECT_PARTIAL || result == cv::INTERSECT_FULL) {
            double intersectionArea = cv::contourArea(intersection);
            double unionArea = rect1.size.area() + rect2.size.area() - intersectionArea;
            return intersectionArea / unionArea;
        } else {
            // Handle unexpected result
            return 0.0;
        }
    }

    void OpenCDASensor::enableGUI(bool visualize) {
        m_GUI = true;
        m_pldm = false;
        m_visualize = visualize;
        m_cv_image = cv::Mat::zeros(1000, 1000, CV_8UC3);
        std::string windowName = "LDM vehicle " + m_string_id;

        //to avoid creating unnecessary image window
       carla::ActorIds cav_ids = m_opencda_client->GetManagedCAVsIds();
       size_t num_cavs = cav_ids.actorid_size();
       for (size_t i = 0; i<num_cavs; ++i){
          if (m_id == cav_ids.actorid(i)){
              m_visualize = visualize;
              break;
          } else {
              m_visualize = false;
          }
       }

        if (m_visualize) {
            cv::namedWindow(windowName, cv::WINDOW_NORMAL);
            cv::imshow(windowName, m_cv_image);
        }

        // Create folder for simulation frames if it does not exist
        struct stat info;
        if (stat("sim_frames_LDM", &info) != 0 || !(info.st_mode & S_IFDIR)) {
            std::string command = "mkdir sim_frames_LDM";
            system(command.c_str());
        }

        // Create subfolder
        std::string subfolderPath = "sim_frames_LDM/vehicle_" + m_string_id;
        if (stat(subfolderPath.c_str(), &info) != 0 || !(info.st_mode & S_IFDIR)) {
            std::string command = "mkdir " + subfolderPath;
            system(command.c_str());
        }

        m_count = 0;
    }

    void
    OpenCDASensor::updateGUI() {
        if (m_GUI) {
            double x_center = 500;
            double y_center = 500;
            double scale_x = 6;
            double scale_y = 6;
            auto egoVehicle = m_opencda_client->GetManagedActorById(m_id);
            double x_offset = egoVehicle.location().x();
            double y_offset = egoVehicle.location().y();

            m_cv_image.setTo(cv::Scalar(0, 0, 0));

            std::map<int, cv::RotatedRect> PO_boxes, GT_boxes;

            std::string windowName = "LDM vehicle " + m_string_id;

            std::vector<LDM::returnedVehicleData_t> LDM_POs, LDM_CVs, LDM_CVRUs;
            std::map<size_t, ClusterInfo> cluster_map;
            std::vector<cv::Scalar> colors{};
            // if there is a cluster we create a range of colors
            if (m_LDM->getClusterMap(cluster_map)) {
                size_t num_colors = 6;
                cv::Scalar startColor(255, 0, 0);
                cv::Scalar endColor(0, 255, 0);
                for (int i = 0; i < num_colors; ++i) {
                    float t = static_cast<float>(i) / (num_colors - 1);
//                int blue = (255 / (num_colors + 1)) * (i + 1);  // Make sure blue is within range 0-255
//                int green = (255 / (num_colors + 1)) * (i + 1);  // Keep green at 0 for blue tones
//                int red = 0;    // Keep red at 0 for blue tones
                    int blue = static_cast<int>(startColor[0] + (endColor[0] - startColor[0]) * t);
                    int green = static_cast<int>(startColor[1] + (endColor[1] - startColor[1]) * t);
                    int red = static_cast<int>(startColor[2] + (endColor[2] - startColor[2]) * t);

                    // Create a cv::Scalar color and push to the colors vector
                    colors.push_back(cv::Scalar(blue, green, red));
                }
            }

            if (m_LDM->getAllPOs(LDM_POs)) {
                for (auto &it: LDM_POs) {
                    carla::Vector pos = m_opencda_client->getCartesian(it.vehData.lon, it.vehData.lat);
                    double x = (pos.x() - x_offset) * scale_x + x_center;
                    double y = (pos.y() - y_offset) * scale_y + y_center;
                    double length = it.vehData.vehicleLength.getData() * scale_x / 10;
                    double width = it.vehData.vehicleWidth.getData() * scale_y / 10;
                    double heading = it.vehData.heading;

                    auto color = cv::Scalar(0, 0, 255);
                    // set color for each cluster
                    for (const auto &pair: cluster_map) {
                        for (const auto &ID: pair.second.IDs) {
                            if (ID == it.vehData.stationID) {
                                color = colors[pair.first - 1];
                            }
                        }
                    }

                    if (it.vehData.itsType != itsType_pedestrian) {
                        // Define the rotated rectangle
                        cv::RotatedRect rotatedRect(cv::Point2f(x, y), cv::Size2f(length, width), heading);
                        PO_boxes[it.vehData.stationID] = rotatedRect;
                        cv::Point2f vertices[4];
                        rotatedRect.points(vertices);

                        // Draw the rotated rectangle
                        for (int j = 0; j < 4; j++) {
                            cv::line(m_cv_image, vertices[j], vertices[(j + 1) % 4], cv::Scalar(0, 0, 255), 2);
                        }

                        // Find the top-left corner of the rotated rectangle
                        cv::Point2f topLeft = vertices[0];
                        for (int j = 1; j < 4; j++) {
                            if (vertices[j].x < topLeft.x ||
                                (vertices[j].x == topLeft.x && vertices[j].y < topLeft.y)) {
                                topLeft = vertices[j];
                            }
                        }
                        // Draw the text at the top-left corner
                        cv::putText(m_cv_image, std::to_string(it.vehData.stationID), topLeft, cv::FONT_HERSHEY_SIMPLEX,
                                    0.4, cv::Scalar(0, 0, 255), 1);
                    } else {
                        // Define the center and radius for the circle
                        cv::Point2f center(x, y);
                        //float radius = std::min(length, width) / 2; // Use an appropriate radius based on your requirements
                        float radius{7.0};
                        // Draw the circle
                        cv::circle(m_cv_image, center, radius, color, 2);

                        // Draw the text (ID) inside the circle
                        std::string text = std::to_string(it.vehData.stationID);
                        int fontFace = cv::FONT_HERSHEY_SIMPLEX;
                        double fontScale = 0.4;
                        int thickness = 1;
                        int baseline = 0;

                        // Get the text size
                        cv::Size textSize = cv::getTextSize(text, fontFace, fontScale, thickness, &baseline);
                        baseline += thickness;

                        // Center the text
                        cv::Point textOrg((x - textSize.width / 2), (y - radius - textSize.height / 2));

                        // Put the text inside the circle
                        cv::putText(m_cv_image, text, textOrg, fontFace, fontScale, cv::Scalar(0, 0, 255), thickness);
                    }

                    std::string type_str;
                    type_str = "vehicle";

                }
            }
            if (m_LDM->getAllCVs(LDM_CVs)) {
                for (auto &it: LDM_CVs) {
                    carla::Vector pos = m_opencda_client->getCartesian(it.vehData.lon, it.vehData.lat);
                    double x = (pos.x() - x_offset) * scale_x + x_center;
                    double y = (pos.y() - y_offset) * scale_y + y_center;
                    double length = it.vehData.vehicleLength.getData() * scale_x / 10;
                    double width = it.vehData.vehicleWidth.getData() * scale_y / 10;
                    double heading = it.vehData.heading;


                    // Define the rotated rectangle
                    cv::RotatedRect rotatedRect(cv::Point2f(x, y), cv::Size2f(length, width), heading);
                    cv::Point2f vertices[4];
                    rotatedRect.points(vertices);

                    // Draw the rotated rectangle - green
                    for (int j = 0; j < 4; j++) {
                        cv::line(m_cv_image, vertices[j], vertices[(j + 1) % 4], cv::Scalar(0, 255, 0), 2);
                    }

                    // Find the top-left corner of the rotated rectangle
                    cv::Point2f topLeft = vertices[0];
                    for (int j = 1; j < 4; j++) {
                        if (vertices[j].x < topLeft.x || (vertices[j].x == topLeft.x && vertices[j].y < topLeft.y)) {
                            topLeft = vertices[j];
                        }
                    }

                    // Draw the text in the top-left corner
                    cv::putText(m_cv_image, std::to_string(it.vehData.stationID), topLeft, cv::FONT_HERSHEY_SIMPLEX,
                                0.4, cv::Scalar(0, 255, 0), 1);
                }
            }
            //retrieve connected pedestrians that already sent a VAM
            if (m_LDM->getAllCVRUs(LDM_CVRUs)) {
                for (auto &it: LDM_CVRUs) {
                    carla::Vector pos = m_opencda_client->getCartesian(it.vehData.lon, it.vehData.lat);
                    double x = (pos.x() - x_offset) * scale_x + x_center;
                    double y = (pos.y() - y_offset) * scale_y + y_center;

                    // Define the center and radius for the circle
                    cv::Point2f center(x, y);
                    //float radius = std::min(length, width) / 2;
                    float radius{8.0};
                    // Draw the circle
                    auto color = cv::Scalar(0, 255, 255); //yellow
                    cv::circle(m_cv_image, center, radius, color, 2);

                    // Draw the text (ID) inside the circle
                    std::string text = std::to_string(it.vehData.stationID);
                    int fontFace = cv::FONT_HERSHEY_SIMPLEX;
                    double fontScale = 0.4;
                    int thickness = 1;
                    int baseline = 0;

                    // Get the text size
                    cv::Size textSize = cv::getTextSize(text, fontFace, fontScale, thickness, &baseline);
                    baseline += thickness;

                    // Center the text
                    cv::Point textOrg((x - textSize.width / 2), (y - radius - textSize.height / 2));

                    // Put the text inside the circle
                    cv::putText(m_cv_image, text, textOrg, fontFace, fontScale, cv::Scalar(0, 0, 255), thickness);

                }
            }

            auto GTactors = m_opencda_client->GetAllActorsIds(); //use GetManagedHostIds() to retrieve only vehicles
            for (int i = 0; i < GTactors.actorid_size(); i++) {
                // carla::Vehicle vehicle = m_opencda_client->GetManagedActorById(GTactors.actorid(i));
                carla::Actor actor = m_opencda_client->GetActorById(GTactors.actorid(i));
                double x = (actor.location().x() - x_offset) * scale_x + x_center;
                double y = (actor.location().y() - y_offset) * scale_y + y_center;
                double length = actor.length() * scale_x;
                double width = actor.width() * scale_y;
                if (width == 0)
                    width = 1;
                double heading = actor.heading();

                if (actor.itstype() != itsType_pedestrian) {
                    // Define the rotated rectangle
                    cv::RotatedRect rotatedRect(cv::Point2f(x, y), cv::Size2f(length, width), heading);
                    GT_boxes[GTactors.actorid(i)] = rotatedRect;
                    cv::Point2f vertices[4];
                    rotatedRect.points(vertices);

                    // Draw the rotated rectangle
                    for (int j = 0; j < 4; j++) {
                        cv::line(m_cv_image, vertices[j], vertices[(j + 1) % 4], cv::Scalar(128, 128, 128), 1);
                    }

                    // Draw the text (ID) inside the rectangle
                    std::string text = std::to_string(GTactors.actorid(i));
                    int fontFace = cv::FONT_HERSHEY_SIMPLEX;
                    double fontScale = 0.4;
                    int thickness = 1;
                    int baseline = 0;

                    // Get the text size
                    cv::Size textSize = cv::getTextSize(text, fontFace, fontScale, thickness, &baseline);
                    baseline += thickness;

                    // Center the text
                    cv::Point textOrg((x - textSize.width / 2), (y + textSize.height / 2));

                    // Put the text inside the rectangle
                    cv::putText(m_cv_image, text, textOrg, fontFace, fontScale, cv::Scalar(128, 128, 128), thickness);
                } else if (actor.itstype() == itsType_pedestrian) {
                    // Define the center and radius for the circle
                    cv::Point2f center(x, y);
                    float radius{2};

                    // Draw the circle
                    cv::circle(m_cv_image, center, radius, cv::Scalar(255, 255, 255), 1);

                    // Draw the text (ID) inside the circle
                    std::string text = std::to_string(GTactors.actorid(i));
                    int fontFace = cv::FONT_HERSHEY_SIMPLEX;
                    double fontScale = 0.3;
                    int thickness = 1;
                    int baseline = 0;

                    // Get the text size
                    cv::Size textSize = cv::getTextSize(text, fontFace, fontScale, thickness, &baseline);
                    baseline += thickness;

                    // Center the text
                    cv::Point textOrg((x - textSize.width / 2), (y - radius - textSize.height / 2));

                    // Put the text inside the circle
                    cv::putText(m_cv_image, text, textOrg, fontFace, fontScale, cv::Scalar(255, 255, 255), thickness);

                }
            }
            // draw cluster if present
            if (m_LDM->getClusterMap(cluster_map)) {
                for (auto it = cluster_map.begin(); it != cluster_map.end(); it++) {
                    cv::Point2f center{it->second.center};
                    float radius{it->second.radius};
                    auto cluster_index{it->first};

                    center.x = (center.x - x_offset) * scale_x + x_center;
                    center.y = (center.y - y_offset) * scale_y + y_center;
                    radius = radius * scale_x + 2; //2 is the radius of the pedestrian

                    cv::circle(m_cv_image, center, static_cast<int>(radius), colors[cluster_index - 1], 1);

                    // Draw the text (ID) inside the circle
                    std::string text = std::to_string(it->second.cardinality);
                    int fontFace = cv::FONT_HERSHEY_SIMPLEX;
                    double fontScale = 0.5;
                    int thickness = 1;
                    int baseline = 0;

                    // Get the text size
                    cv::Size textSize = cv::getTextSize(text, fontFace, fontScale, thickness, &baseline);
                    baseline += thickness;

                    // Center the text
                    cv::Point textOrg((center.x - textSize.width / 2), (center.y - radius - textSize.height / 2));

                    // Put the text inside the circle
                    cv::putText(m_cv_image, text, textOrg, fontFace, fontScale, cv::Scalar(0, 0, 255), thickness);


                };
            }

            auto time = Simulator::Now().GetSeconds();
            cv::putText(m_cv_image, "Time: " + std::to_string(time), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1,
                        cv::Scalar(255, 255, 255), 2);

            // Show list of POs with their IDs, accuracy and respPMID

            // if (LDM_POs.size() > 0) {
            //     int i = 0;
            //     int text_x_offset = 10;
            //     for (auto &it : LDM_POs) {
            //         std::string po_string = "PO ID: " + std::to_string(it.vehData.stationID)
            //             + " / Accuracy: " + std::to_string(it.vehData.GTaccuracy.getData())
            //             + " / RespPMID: " + std::to_string(it.vehData.respPMID.getData());
            //         int offset = 60 + 30 * i;
            //         cv::putText(m_cv_image, po_string, cv::Point(text_x_offset, offset), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);
            //         i++;
            //         if(i > 4) {
            //           text_x_offset += 500;
            //           i = 0;
            //       }
            //     }
            // }


            int i = 0;
            int text_x_offset = 10;
            double avg_accuracy = 0.0;
            for (const auto &PO: PO_boxes) {
                int id = PO.first;
                if (GT_boxes.find(id) != GT_boxes.end()) {

                    double IoU = calculateIoU(PO.second, GT_boxes[id]);
                    avg_accuracy += IoU;
                    //std::cout << "ID: " << id << " IoU: " << IoU << std::endl;
                    int offset = 60 + 30 * i;
                    std::string po_string = "PO ID: " + std::to_string(id)
                                            + " / Accuracy: " + std::to_string(IoU);
                    cv::putText(m_cv_image, po_string, cv::Point(text_x_offset, offset), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                                cv::Scalar(0, 0, 255), 2);
                    i++;
                    if (i > 4) {
                        text_x_offset += 500;
                        i = 0;
                    }
                }
            }
            avg_accuracy /= PO_boxes.size();
            //m_csv_file << time << "," << avg_accuracy << "\n";
            std::cout << "Time: " << time << " Avg. Accuracy: " << avg_accuracy << std::endl;

            if (m_visualize) {
                cv::imshow(windowName, m_cv_image);
            }
            std::string filename = "sim_frames_LDM/vehicle_" + m_string_id + "/" + std::to_string(m_count) + ".png";
            cv::imwrite(filename, m_cv_image);
            m_count++;
            cv::waitKey(1);
        }
    }

    void
    OpenCDASensor::createCluster() {
        std::vector<vehicleData_t> LDM_VRUs{};
        std::vector<LDM::returnedVehicleData_t> LDM_POs, LDM_VAMs;
        std::vector<LDM::returnedVehicleData_t>::iterator it;
        std::vector<ns3::point2> input_points;
        std::vector<uint64_t> vruIDs;
        bool fromPerception{true};
        const double HEADING_DEVIATION_THRESH = 30;
        if (m_LDM->getAllPOs(LDM_POs)) {
            for (it = LDM_POs.begin(); it != LDM_POs.end(); it++) {
                if ((it->vehData.itsType == itsType_pedestrian && it->vehData.perceivedBy.getData() == static_cast<long>(m_id))
                && it->vehData.VAM == false) {
                    LDM_VRUs.push_back(it->vehData);
                    ns3::point2 pt;
                    pt.x = it->vehData.x;
                    pt.y = it->vehData.y;
                    input_points.push_back(pt);
                    vruIDs.push_back(it->vehData.stationID);
                } //we are not considering CPM objects (PO from another CAV)
            }
            if (m_LDM->getAllCVRUs(LDM_VAMs)){
                for (auto ped = LDM_VAMs.begin(); ped != LDM_VAMs.end(); ++ped){
                    LDM_VRUs.push_back(ped->vehData);
                    ns3::point2 pt;
                    pt.x = ped->vehData.x;
                    pt.y = ped->vehData.y;
                    input_points.push_back(pt);
                    vruIDs.push_back(ped->vehData.stationID);
                }
            }
            // we also consider connected pedestrian to form a cluster

            float epsilon = 5.0;  // Maximum distance between points to be considered neighbors
            int minPts = 3;        // Minimum number of points to form a cluster

            // Apply DBSCAN clustering
            auto clusters = dbscan(input_points, epsilon, minPts);
            auto flat_clusters = std::vector<size_t>(input_points.size());
            auto cluster_map = std::map<size_t, ClusterInfo>();

            for (size_t i = 0; i < clusters.size(); i++) {
                for (auto p: clusters[i]) {
                    flat_clusters[p] = i + 1;
                }
            }
            for (size_t i = 0; i < flat_clusters.size(); i++) {
                size_t cluster_index = flat_clusters[i];
                size_t ID = vruIDs[i];
                if (cluster_index == 0) {
                    continue;
                }
                cluster_map[cluster_index].IDs.emplace_back(ID);
            }
            //check pedestrians for heading deviation
            double cluster_mean_heading {};
            std::vector<double> member_headings; // Just headings for mean calculation
            if (!cluster_map.empty()) {
                for (auto it = cluster_map.begin(); it != cluster_map.end();) {
                    auto &[key, cluster] = *it;
                    int cluster_index = key;
                    std::vector<unsigned long> &point_ids = cluster.IDs;
                    for (auto id: point_ids) {
                        LDM::returnedVehicleData_t retVRUdata;
                        m_LDM->lookup(id, retVRUdata);
                        member_headings.push_back(retVRUdata.vehData.heading);
//                        std::cout << "ID: " << id << " Heading: "<< retVRUdata.vehData.heading << std::endl;
                    }
                    cluster_mean_heading = get_circular_mean_heading(member_headings);
//                    std::cout << "Cluster mean heading: " << cluster_mean_heading << " deg" << std::endl;

                    bool outlier_removed_in_iteration = true;

                    while(outlier_removed_in_iteration){
                        outlier_removed_in_iteration = false;
                        int most_outlier_id = -1;
                        double highest_diff = 0;
                        // Find the most outlier
                        for (unsigned long current_id : point_ids) {
                            LDM::returnedVehicleData_t retVRUdata;
                            m_LDM->lookup(current_id, retVRUdata);
//                            double diff = get_angular_difference(retVRUdata.vehData.heading, cluster_mean_heading);
                            double diff = std::fmod(cluster_mean_heading - retVRUdata.vehData.heading, 360.0);
                            if (diff > 180.0) {
                                diff -= 360.0;
                            } else if (diff < -180.0) {
                                diff += 360.0;
                            }
                            diff = std::abs(diff);
                            if (diff > highest_diff) {
                                highest_diff = diff;
                                most_outlier_id = current_id;
                            }
                        }
                        //if higher difference is higher than threshold means we need to remove an object from the list
                        if (highest_diff > HEADING_DEVIATION_THRESH) {
                            std::cout << "Outlier ID: " << most_outlier_id << std::endl;
                            //remove outlier from point_ids
                            auto new_end = std::remove(point_ids.begin(),point_ids.end(),most_outlier_id);
                            point_ids.erase(new_end, point_ids.end());
                            //clear member_headings
                            member_headings.clear();
                            for (auto id: point_ids) {
                                LDM::returnedVehicleData_t retVRUdata;
                                m_LDM->lookup(id, retVRUdata);
                                member_headings.push_back(retVRUdata.vehData.heading);
//                                std::cout << "ID: " << id << " Heading: " << retVRUdata.vehData.heading << std::endl;
                            }
                            cluster_mean_heading = get_circular_mean_heading(member_headings);
                            std::cout << "New Cluster mean heading: " << cluster_mean_heading << " deg" << std::endl;

                            outlier_removed_in_iteration = true;
                            }
//                            else {
//                                std::cout << "No more outliers found" << std::endl;
//                            }
                    }

                    if (point_ids.size() < minPts) {
//                        std::cout << "Removing cluster " << key << " due to low cardinality: " << point_ids.size()
//                                  << std::endl;
                        it = cluster_map.erase(it);
                    } else {
                        ++it;
                    }
                }
            }


            std::cout << "  CAV " << m_id << std::endl;
            if (!cluster_map.empty()) {
                for(auto it = cluster_map.begin(); it != cluster_map.end();) {

                    std::cout << "\n---FROM DBSCAN---" << std::endl;
                    auto& cluster_entry = *it;
                    int cluster_index = cluster_entry.first;
                    std::cout << "Cluster " << cluster_entry.first << std::endl;
                    const std::vector<unsigned long> &point_ids = cluster_entry.second.IDs;
                    auto cardinality = point_ids.size();
                    int count_connected_ped = 0;

                    // define the points inside the cluster (retrieve then by the Ids)
                    uint64_t timestamp{};
                    member_headings.clear();
                    std::vector<cv::Point2f> cluster_points{};
                    float mean_cluster_speed{};
                    for (auto id: point_ids) {
                        LDM::returnedVehicleData_t retVRUdata;
                        m_LDM->lookup(id, retVRUdata);
                        cluster_points.push_back(cv::Point2f(retVRUdata.vehData.x, retVRUdata.vehData.y));
                        double speedAbs = sqrt(pow(retVRUdata.vehData.xSpeedAbs.getData(),2)+pow(retVRUdata.vehData.ySpeedAbs.getData(),2))/CENTI;
                        std::cout << "  Object ID: " << id << " - timestamp: " << retVRUdata.vehData.timestamp_us
                                  << " - heading: " << retVRUdata.vehData.heading << " - Velocity: "<< speedAbs <<"\n";
                        member_headings.push_back(retVRUdata.vehData.heading);
                        if (retVRUdata.vehData.timestamp_us > timestamp) {
                            timestamp = retVRUdata.vehData.timestamp_us;
                            //check for the latest perceived pedestrian on the cluster
                        };
                        if (retVRUdata.vehData.VAM){ ++count_connected_ped;}
                        mean_cluster_speed += speedAbs;
                    }
                    cluster_mean_heading = get_circular_mean_heading(member_headings);
                    mean_cluster_speed /= cardinality;
                    std::cout << "Cluster mean speed: " << mean_cluster_speed << " m/s" << std::endl;
                    std::cout << "Cluster mean heading: " << cluster_mean_heading << " degrees" << std::endl;

                    cv::Point2f center;
                    float radius;
                    cv::minEnclosingCircle(cluster_points, center, radius);

                    std::cout << "(x = " << center.x << ", y = " << center.y << ")" << std::endl;

                    //avoid creating cluster using only connected pedestrians
                    if (count_connected_ped != cardinality){
                        cluster_map[cluster_index].radius = radius;
                        cluster_map[cluster_index].center = center;
                        cluster_map[cluster_index].perceivedBy = m_id;
                        cluster_map[cluster_index].timestamp_us = timestamp;
                        cluster_map[cluster_index].last_predicted_timestamp = timestamp;
                        cluster_map[cluster_index].cardinality = cardinality;
                        cluster_map[cluster_index].heading = cluster_mean_heading;
                        cluster_map[cluster_index].speed_ms = mean_cluster_speed;
                        ++it;
                    }else{
                        std::cout << "Cluster " << cluster_index << " removed due to all members being VAM-connected." << std::endl;
                        it = cluster_map.erase(it);
                    }


                }
                m_LDM->updateClusterMap(cluster_map, fromPerception);
            } else {
                std::cout << "No cluster from DBSCAN" << std::endl;
                auto empty_cluster_map = std::map<size_t, ClusterInfo>();
                m_LDM->updateClusterMap(empty_cluster_map, fromPerception);

            }
        }
    }

    void
    OpenCDASensor::logPerceivedPedestrians() {
        // Define the time step duration (assuming 0.05 based on previous code)
        const double time_step_duration = 0.1;

        // Check if there is any data in the main map
        if (g_perceived_pedestrians.empty()) {
            std::cout << "No perceived pedestrian data available to log." << std::endl;
            return;
        }

        int files_created_count = 0;

        // Iterate through each vehicle's data in the map
        for (const auto &pair: g_perceived_pedestrians) {
            const std::string &vehicle_id = pair.first;
            const std::vector<int> &object_counts = pair.second;

            // Only create a log file if the vehicle has recorded any perceived pedestrians over time
            if (!object_counts.empty()) {
                // Construct the output filename using the vehicle ID
                std::string filename = "PerceivedPedestrians_" + vehicle_id + ".csv";

                // Open the output file for writing. This will create a new file or overwrite an existing one.
                std::ofstream log_file(filename);

                // Check if the file was opened successfully
                if (log_file.is_open()) {
                    // Write the header row (Time and the Vehicle ID)
                    log_file << "Time," << vehicle_id << std::endl; // Using comma as the delimiter

                    // Write the data rows for each time step
                    for (size_t i = 0; i < object_counts.size(); ++i) {
                        double current_time = static_cast<double>(i) * time_step_duration + 0.15;
                        int count = object_counts[i];

                        // Log the time and the count for this time step
                        log_file << current_time << "," << count << std::endl; // Using comma as the delimiter
                    }

                    // Close the file for this specific vehicle
                    log_file.close();
                    std::cout << "Successfully logged data for vehicle " << vehicle_id << " to " << filename
                              << std::endl;
                    files_created_count++;

                } else {
                    // Error opening the file
                    std::cerr << "Error: Unable to open log file for writing: " << filename << std::endl;
                }
            } else {
                // Optional: provide feedback for vehicles with no data
                // std::cout << "Vehicle " << vehicle_id << " has no perceived pedestrian data to log." << std::endl;
            }
        }

        // Final feedback after processing all vehicles
        if (files_created_count == 0) {
            // This case is covered if perceived_pedestrians_data.empty() was true,
            // but also if all vehicles in the map had empty vectors.
            std::cout << "No files were created as no vehicles had non-empty perceived pedestrian data." << std::endl;
        } else {
            std::cout << "Finished logging data for " << files_created_count << " vehicle(s)." << std::endl;
        }
    }

     double
    OpenCDASensor::get_circular_mean_heading(const std::vector<double> &headings_deg) {
        if (headings_deg.empty()) return 0.0;

        double sum_cos = 0.0;
        double sum_sin = 0.0;

        for (double heading_deg: headings_deg) {
            double heading_rad = heading_deg * M_PI / 180.0;
            sum_cos += std::cos(heading_rad);
            sum_sin += std::sin(heading_rad);
        }

        // Handle sum near zero vector
        if (std::abs(sum_cos) < 1e-9 && std::abs(sum_sin) < 1e-9) {
            // If all headings were identical, return that heading
            bool all_identical = true;
            if (headings_deg.size() > 1) {
                for (size_t i = 1; i < headings_deg.size(); ++i) {
                    double diff = std::fmod(headings_deg[i] - headings_deg[0], 360.0);
                    if (diff > 180.0) {
                        diff -= 360.0;
                    } else if (diff < -180.0) {
                        diff += 360.0;
                    }
                    if (std::abs(diff) > 1e-9) {
                        all_identical = false;
                        break;
                    }
                }
            }
            if (all_identical && !headings_deg.empty()) return headings_deg[0];

            return 0.0; // Fallback for sum near zero vector
        }


        double mean_heading_rad = std::atan2(sum_sin, sum_cos);

        double mean_heading_deg = mean_heading_rad * 180.0 / M_PI;
        if (mean_heading_deg < 0) {
            mean_heading_deg += 360.0;
        }

        return mean_heading_deg;
    }

//    double
//    OpenCDASensor::get_angular_difference(double heading1_deg, double heading2_deg) {
//        double diff = std::fmod(heading2_deg - heading1_deg, 360.0);
//        if (diff > 180.0) {
//            diff -= 360.0;
//        } else if (diff < -180.0) {
//            diff += 360.0;
//        }
//        return std::abs(diff);
//    }
}


