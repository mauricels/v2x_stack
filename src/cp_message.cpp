#include "cp_message.h"
#include <boost/make_shared.hpp>
#include <ros_etsi_its_msgs/msg/cpm.hpp>
#include <vanetza/asn1/cpm.hpp>
#include <algorithm>

namespace v2x_stack_btp
{


inline uint8_t reverse_byte(uint8_t byte)
{
    byte = (byte & 0xf0) >> 4 | (byte & 0x0f) << 4;
    byte = (byte & 0xcc) >> 2 | (byte & 0x33) << 2;
    byte = (byte & 0xaa) >> 1 | (byte & 0x55) << 1;
    return byte;
}


boost::shared_ptr<ros_etsi_its_msgs::msg::CPM> convertCpm(const vanetza::asn1::Cpm& asn1, std::string* error_msg)
{
    auto msg = boost::make_shared<ros_etsi_its_msgs::msg::CPM>();

    // etsi_its_msgs/CPM header fields
    //msg.header.stamp = ros::Time::now();
    msg->its_header.protocol_version = asn1->header.protocolVersion;
    msg->its_header.station_id = asn1->header.stationID;
    msg->generation_delta_time = asn1->cpm.generationDeltaTime;

    // CpmManagementContainer           
    msg->station_type.value = asn1->cpm.cpmParameters.managementContainer.stationType;
    
    msg->reference_position.altitude.value = asn1->cpm.cpmParameters.managementContainer.referencePosition.altitude.altitudeValue;
    msg->reference_position.altitude.confidence = asn1->cpm.cpmParameters.managementContainer.referencePosition.altitude.altitudeConfidence;
    msg->reference_position.latitude = asn1->cpm.cpmParameters.managementContainer.referencePosition.latitude;
    msg->reference_position.longitude = asn1->cpm.cpmParameters.managementContainer.referencePosition.longitude;
    msg->reference_position.position_confidence.semi_major_confidence = asn1->cpm.cpmParameters.managementContainer.referencePosition.positionConfidenceEllipse.semiMajorConfidence;
    msg->reference_position.position_confidence.semi_minor_confidence = asn1->cpm.cpmParameters.managementContainer.referencePosition.positionConfidenceEllipse.semiMinorConfidence;
    msg->reference_position.position_confidence.semi_major_orientation = asn1->cpm.cpmParameters.managementContainer.referencePosition.positionConfidenceEllipse.semiMajorOrientation;
    
    // Station Data Container
    msg->originating_vehicle_container.heading.value = asn1->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer.heading.headingValue;
    msg->originating_vehicle_container.heading.confidence = asn1->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer.heading.headingConfidence;
    msg->originating_vehicle_container.speed.value = asn1->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer.speed.speedValue;
    msg->originating_vehicle_container.speed.confidence = asn1->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer.speed.speedConfidence;
    //Error Segmentation fault for next declarations
    //msg->originating_vehicle_container.vehicle_orientation_angle.value = asn1->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer.vehicleOrientationAngle->value;
    //msg->originating_vehicle_container.vehicle_orientation_angle.confidence = asn1->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer.vehicleOrientationAngle->confidence;
    msg->originating_vehicle_container.drive_direction.value = asn1->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer.driveDirection;
   
    
    if (asn1->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer.vehicleLength) 
    {
        msg->originating_vehicle_container.has_vehicle_length = true;
        msg->originating_vehicle_container.vehicle_length.value = asn1->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer.vehicleLength->vehicleLengthValue;
        msg->originating_vehicle_container.vehicle_length.confidence_indication = asn1->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer.vehicleLength->vehicleLengthConfidenceIndication;
    }

    if (asn1->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer.vehicleWidth)
    {
        msg->originating_vehicle_container.has_vehicle_width = true;
        //msg->originating_vehicle_container.vehicle_width.value = asn1->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer.vehicleWidth;
    }

    // TODO: extend the Originating Vehicle Container


    // perceivedObjectContainer
    msg->number_of_perceived_objects = asn1->cpm.cpmParameters.numberOfPerceivedObjects;
    
    if (msg->number_of_perceived_objects > 0)
    {    
        ros_etsi_its_msgs::msg::PerceivedObject perceived_object;
        
        using namespace vanetza;
        PerceivedObjectContainer_t *objectsContainer = asn1->cpm.cpmParameters.perceivedObjectContainer;

        for (int i = 0; objectsContainer != nullptr && i < objectsContainer->list.count; i++)
        {                       
            PerceivedObject *objCont = objectsContainer->list.array[i];

            perceived_object.object_id = objCont->objectID;            
            perceived_object.time_of_measurement = objCont->timeOfMeasurement;

            if (objCont->objectAge)
                perceived_object.object_age = *(objCont->objectAge);
            
            perceived_object.object_confidence = objCont->objectConfidence;

            perceived_object.x_distance.value = objCont->xDistance.value;
            perceived_object.x_distance.confidence = objCont->xDistance.confidence;
            perceived_object.y_distance.value = objCont->yDistance.value;
            perceived_object.y_distance.confidence = objCont->yDistance.confidence;
            //perceived_object.z_distance.value = objCont->zDistance->value;
            //perceived_object.z_distance.confidence = objCont->zDistance->confidence;
            
            perceived_object.x_speed.value = objCont->xSpeed.value;
            perceived_object.x_speed.confidence = objCont->xSpeed.confidence;
            perceived_object.y_speed.value = objCont->ySpeed.value;
            perceived_object.y_speed.confidence = objCont->ySpeed.confidence;
            //perceived_object.z_speed.value = objCont->zSpeed->value;
            //perceived_object.z_speed.confidence = objCont->zSpeed->confidence;

            //perceived_object.x_acceleration.value = objCont->xAcceleration->longitudinalAccelerationValue;
            //perceived_object.x_acceleration.confidence = objCont->xAcceleration->longitudinalAccelerationConfidence;
            //perceived_object.y_acceleration.value = objCont->yAcceleration->lateralAccelerationValue;
            //perceived_object.y_acceleration.confidence = objCont->yAcceleration->lateralAccelerationConfidence;

            //perceived_object.object_ref_point.value = objCont->objectRefPoint;

            //perceived_object.dynamic_status.value = objCont->dynamicStatus;

            if (objCont->classification != nullptr && objCont->classification->list.count > 0)
            {
                if(objCont->classification->list.array[0]->Class.present == ObjectClass__class_PR_person)
                {
                    PersonSubclass_t &personSubclass = objCont->classification->list.array[0]->Class.choice.person;
                    perceived_object.classification.value = ros_etsi_its_msgs::msg::StationType::PEDESTRIAN;
                } else {
                    VehicleSubclass_t &vehicleSubclass = objCont->classification->list.array[0]->Class.choice.vehicle;
                    switch (vehicleSubclass.type) {

                        case VehicleSubclassType_moped:
                            perceived_object.classification.value = ros_etsi_its_msgs::msg::StationType::MOPED;
                            break;
                        case VehicleSubclassType_motorcycle:
                            perceived_object.classification.value = ros_etsi_its_msgs::msg::StationType::MOTORCYCLE;
                            break;
                        case VehicleSubclassType_passengerCar:
                            perceived_object.classification.value = ros_etsi_its_msgs::msg::StationType::PASSENGER_CAR;
                            break;
                        case VehicleSubclassType_bus:
                            perceived_object.classification.value = ros_etsi_its_msgs::msg::StationType::BUS;
                            break;
                        case VehicleSubclassType_lightTruck:
                            perceived_object.classification.value = ros_etsi_its_msgs::msg::StationType::LIGHT_TRUCK;
                            break;
                        case VehicleSubclassType_heavyTruck:
                            perceived_object.classification.value = ros_etsi_its_msgs::msg::StationType::HEAVY_TRUCK;
                            break;
                        case VehicleSubclassType_trailer:
                            perceived_object.classification.value = ros_etsi_its_msgs::msg::StationType::TRAILER;
                            break;
                        case VehicleSubclassType_specialVehicles:
                            perceived_object.classification.value = ros_etsi_its_msgs::msg::StationType::SPECIAL_VEHICLE;
                            break;
                        case VehicleSubclassType_tram:
                            perceived_object.classification.value = ros_etsi_its_msgs::msg::StationType::TRAM;
                            break;
                        default:
                            perceived_object.classification.value = ros_etsi_its_msgs::msg::StationType::UNKNOWN;
                    }
                }
            }

            //perceived_object.matched_position.lane_id = objCont->matchedPosition->laneID;*/
            
            msg->list_of_perceived_objects.perceived_object_container.push_back(perceived_object);
        }
            
    }

    return msg;
}
/*
vanetza::asn1::Cam convertCam(ros_etsi_its_msgs::msg::CAM::ConstSharedPtr ptr)
{
    vanetza::asn1::Cam msg;

    ItsPduHeader_t& header = msg->header;
    header.protocolVersion = ptr->its_header.protocol_version;
    header.messageID = ptr->its_header.message_id;
    header.stationID = ptr->its_header.station_id;

    CoopAwareness_t& ca = msg->cam;
    ca.generationDeltaTime = ptr->generation_delta_time;

    BasicContainer_t& basic = ca.camParameters.basicContainer;
    basic.stationType = ptr->station_type.value;
    basic.referencePosition.altitude.altitudeValue = ptr->reference_position.altitude.value;
    basic.referencePosition.altitude.altitudeConfidence = ptr->reference_position.altitude.confidence;
    basic.referencePosition.latitude = ptr->reference_position.latitude;
    basic.referencePosition.longitude = ptr->reference_position.longitude;
    basic.referencePosition.positionConfidenceEllipse.semiMajorConfidence =
        ptr->reference_position.position_confidence.semi_major_confidence;
    basic.referencePosition.positionConfidenceEllipse.semiMinorConfidence =
        ptr->reference_position.position_confidence.semi_minor_confidence;
    basic.referencePosition.positionConfidenceEllipse.semiMajorOrientation =
        ptr->reference_position.position_confidence.semi_major_orientation;

    HighFrequencyContainer_t& hfc = ca.camParameters.highFrequencyContainer;
    hfc.present = HighFrequencyContainer_PR_basicVehicleContainerHighFrequency;

    BasicVehicleContainerHighFrequency_t& bvc = hfc.choice.basicVehicleContainerHighFrequency;
    bvc.heading.headingValue = ptr->high_frequency_container.heading.value;
    bvc.heading.headingConfidence = ptr->high_frequency_container.heading.confidence;
    bvc.speed.speedValue = ptr->high_frequency_container.speed.value;
    bvc.speed.speedConfidence = ptr->high_frequency_container.speed.confidence;
    bvc.driveDirection = ptr->high_frequency_container.drive_direction.value;
    bvc.longitudinalAcceleration.longitudinalAccelerationValue =
        ptr->high_frequency_container.longitudinal_acceleration.value;
    bvc.longitudinalAcceleration.longitudinalAccelerationConfidence =
        ptr->high_frequency_container.longitudinal_acceleration.confidence;
    bvc.curvature.curvatureValue = ptr->high_frequency_container.curvature.value;
    bvc.curvature.curvatureConfidence = ptr->high_frequency_container.curvature.confidence;
    bvc.curvatureCalculationMode = ptr->high_frequency_container.curvature_calculation_mode.value;
    bvc.yawRate.yawRateValue = ptr->high_frequency_container.yaw_rate.value;
    bvc.yawRate.yawRateConfidence = ptr->high_frequency_container.yaw_rate.confidence;
    bvc.vehicleLength.vehicleLengthValue = ptr->high_frequency_container.vehicle_length.value;
    bvc.vehicleLength.vehicleLengthConfidenceIndication =
        ptr->high_frequency_container.vehicle_length.confidence_indication;
    bvc.vehicleWidth = ptr->high_frequency_container.vehicle_width.value;
    if (ptr->high_frequency_container.has_acceleration_control)
    {
        bvc.accelerationControl = vanetza::asn1::allocate<AccelerationControl_t>();
        bvc.accelerationControl->buf = static_cast<uint8_t*>(vanetza::asn1::allocate(1));
        bvc.accelerationControl->size = 1;
        bvc.accelerationControl->buf[0] = reverse_byte(ptr->high_frequency_container.acceleration_control.value);
    }

    if (ptr->has_low_frequency_container)
    {
        ca.camParameters.lowFrequencyContainer = vanetza::asn1::allocate<LowFrequencyContainer_t>();
        ca.camParameters.lowFrequencyContainer->present = LowFrequencyContainer_PR_basicVehicleContainerLowFrequency;

        BasicVehicleContainerLowFrequency& bvcl = ca.camParameters.lowFrequencyContainer->choice.basicVehicleContainerLowFrequency;
        bvcl.vehicleRole = ptr->low_frequency_container.vehicle_role.value;
        bvcl.exteriorLights.buf = static_cast<uint8_t*>(vanetza::asn1::allocate(1));
        bvcl.exteriorLights.size = 1;
        bvcl.exteriorLights.buf[0] = reverse_byte(ptr->low_frequency_container.exterior_lights.value);

        for (const ros_etsi_its_msgs::msg::PathPoint& path_point : ptr->low_frequency_container.path_history.points)
        {
            PathPoint_t* pathPoint = vanetza::asn1::allocate<PathPoint_t>();
            if (path_point.path_delta_time.value != ros_etsi_its_msgs::msg::PathDeltaTime::UNAVAILABLE)
            {
                pathPoint->pathDeltaTime = vanetza::asn1::allocate<PathDeltaTime_t>();
                *pathPoint->pathDeltaTime = path_point.path_delta_time.value;
            }
            pathPoint->pathPosition.deltaAltitude = path_point.path_position.delta_altitude;
            pathPoint->pathPosition.deltaLatitude = path_point.path_position.delta_latitude;
            pathPoint->pathPosition.deltaLongitude = path_point.path_position.delta_longitude;
            ASN_SEQUENCE_ADD(&bvcl.pathHistory, pathPoint);
        }
    }

    return msg;
}*/

} // namespace v2x_stack_btp

