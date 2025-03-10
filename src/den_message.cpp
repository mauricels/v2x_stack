#include "den_message.h"
#include <boost/make_shared.hpp>
#include <ros_etsi_its_msgs/msg/denm.hpp>
#include <vanetza/asn1/support/asn_application.h>
#include <vanetza/asn1/denm.hpp>

namespace v2x_stack_btp
{

namespace
{

inline uint8_t reverse_byte(uint8_t byte)
{
    byte = (byte & 0xf0) >> 4 | (byte & 0x0f) << 4;
    byte = (byte & 0xcc) >> 2 | (byte & 0x33) << 2;
    byte = (byte & 0xaa) >> 1 | (byte & 0x55) << 1;
    return byte;
}

} // namespace

boost::shared_ptr<ros_etsi_its_msgs::msg::DENM> convertDenm(const vanetza::asn1::r1::Denm& asn1, std::string* error_msg)
{
    auto msg = boost::make_shared<ros_etsi_its_msgs::msg::DENM>();
    //ros_etsi_its_msgs::msg::DENM msg;

    // etsi_its_msgs/DENM header fields
    //msg.header.stamp = ros::Time::now();
    msg->its_header.protocol_version = asn1->header.protocolVersion;
    msg->its_header.station_id = asn1->header.stationID;

    // management container
    const auto& mngmt = asn1->denm.management;
    msg->management.action_id.station_id = mngmt.actionID.originatingStationID;
    msg->management.action_id.sequence_number = mngmt.actionID.sequenceNumber;
    asn_INTEGER2uint64(&mngmt.detectionTime, &msg->management.detection_time);
    asn_INTEGER2uint64(&mngmt.referenceTime, &msg->management.reference_time);
    //msg->management.termination = *mngmt.termination; // OPTIONAL
    msg->management.event_position.latitude = mngmt.eventPosition.latitude;
    msg->management.event_position.longitude = mngmt.eventPosition.longitude;
    msg->management.event_position.position_confidence.semi_major_confidence = mngmt.eventPosition.positionConfidenceEllipse.semiMajorConfidence;
    msg->management.event_position.position_confidence.semi_minor_confidence = mngmt.eventPosition.positionConfidenceEllipse.semiMinorConfidence;
    msg->management.event_position.position_confidence.semi_major_orientation = mngmt.eventPosition.positionConfidenceEllipse.semiMajorOrientation;
    msg->management.event_position.altitude.value = mngmt.eventPosition.altitude.altitudeValue;
    msg->management.event_position.altitude.confidence = mngmt.eventPosition.altitude.altitudeConfidence;
    //msg->management.relevance_distance.value = *mngmt.relevanceDistance; // OPTIONAL
    //msg->management.relevance_traffic_direction.value = *mngmt.relevanceTrafficDirection; // OPTIONAL
    msg->management.validity_duration = *mngmt.validityDuration; 
    //msg->management.transmission_interval = *mngmt.transmissionInterval; // OPTIONAL
    msg->management.station_type.value = mngmt.stationType;
    
    // situation container (optional)
    if(asn1->denm.situation)
    {
        msg->has_situation = true;
        msg->situation.information_quality.value = asn1->denm.situation->informationQuality;
        msg->situation.event_type.cause_code = asn1->denm.situation->eventType.causeCode;
        msg->situation.event_type.sub_cause_code = asn1->denm.situation->eventType.subCauseCode;
        if(asn1->denm.situation->linkedCause)
        {
            msg->situation.has_linked_cause = true;
            msg->situation.linked_cause.cause_code = asn1->denm.situation->linkedCause->causeCode;
            msg->situation.linked_cause.sub_cause_code = asn1->denm.situation->linkedCause->subCauseCode;
        }
        if(asn1->denm.situation->eventHistory)
        {
            for (int i = 0; i < asn1->denm.situation->eventHistory->list.count; ++i)
            {
                const EventPoint_t* asn1_event_point = asn1->denm.situation->eventHistory->list.array[i];
                ros_etsi_its_msgs::msg::EventPoint event_point;

                event_point.event_position.delta_latitude = asn1_event_point->eventPosition.deltaLatitude;
                event_point.event_position.delta_longitude = asn1_event_point->eventPosition.deltaLongitude;
                event_point.event_position.delta_altitude = asn1_event_point->eventPosition.deltaAltitude;
                // event_point.event_delta_time.value = asn1_event_point->eventDeltaTime; // OPTIONAL
                event_point.information_quality.value = asn1_event_point->informationQuality;

                msg->situation.event_history.push_back(event_point);
            }
        }
    }

    // location container (optional)
    if(asn1->denm.location)
    {
        msg->has_location = true;
        if(asn1->denm.location->eventSpeed)
        {
            msg->location.event_speed.value = asn1->denm.location->eventSpeed->speedValue;
            msg->location.event_speed.confidence = asn1->denm.location->eventSpeed->speedConfidence;
        }
        
        if(asn1->denm.location->eventPositionHeading)
        {
            msg->location.event_position_heading.value = asn1->denm.location->eventPositionHeading->headingValue;
            msg->location.event_position_heading.confidence = asn1->denm.location->eventPositionHeading->headingConfidence;
        }
        
        for(int i = 0; i < asn1->denm.location->traces.list.count; ++i)
        {
            const PathHistory_t* asn1_path_history = asn1->denm.location->traces.list.array[i];
            ros_etsi_its_msgs::msg::PathHistory path_history;

            for (int j = 0; j < asn1_path_history->list.count; ++j)
            {
                const PathPoint_t* asn1_path_point = asn1_path_history->list.array[j];
                ros_etsi_its_msgs::msg::PathPoint path_point;

                path_point.path_position.delta_latitude = asn1_path_point->pathPosition.deltaLatitude;
                path_point.path_position.delta_longitude = asn1_path_point->pathPosition.deltaLongitude;
                path_point.path_position.delta_altitude = asn1_path_point->pathPosition.deltaAltitude;

                path_point.path_delta_time.value = ros_etsi_its_msgs::msg::PathDeltaTime::UNAVAILABLE;
                if (asn1_path_point->pathDeltaTime)
                {
                    path_point.path_delta_time.value = *(asn1_path_point->pathDeltaTime);
                }

                path_history.points.push_back(path_point);
            }

            msg->location.traces.push_back(path_history);
        }

        if(asn1->denm.location->roadType)
        {
            msg->location.road_type = *asn1->denm.location->roadType;
        }
            
    }
   
    return msg;
}

// vanetza::asn1::Cam convertCam(ros_etsi_its_msgs::msg::CAM::ConstSharedPtr ptr)
// {
//     vanetza::asn1::Cam msg;

//     ItsPduHeader_t& header = msg->header;
//     header.protocolVersion = ptr->its_header.protocol_version;
//     header.messageID = ptr->its_header.message_id;
//     header.stationID = ptr->its_header.station_id;

//     CoopAwareness_t& ca = msg->cam;
//     ca.generationDeltaTime = ptr->generation_delta_time;

//     BasicContainer_t& basic = ca.camParameters.basicContainer;
//     basic.stationType = ptr->station_type.value;
//     basic.referencePosition.altitude.altitudeValue = ptr->reference_position.altitude.value;
//     basic.referencePosition.altitude.altitudeConfidence = ptr->reference_position.altitude.confidence;
//     basic.referencePosition.latitude = ptr->reference_position.latitude;
//     basic.referencePosition.longitude = ptr->reference_position.longitude;
//     basic.referencePosition.positionConfidenceEllipse.semiMajorConfidence =
//         ptr->reference_position.position_confidence.semi_major_confidence;
//     basic.referencePosition.positionConfidenceEllipse.semiMinorConfidence =
//         ptr->reference_position.position_confidence.semi_minor_confidence;
//     basic.referencePosition.positionConfidenceEllipse.semiMajorOrientation =
//         ptr->reference_position.position_confidence.semi_major_orientation;

//     HighFrequencyContainer_t& hfc = ca.camParameters.highFrequencyContainer;
//     hfc.present = HighFrequencyContainer_PR_basicVehicleContainerHighFrequency;

//     BasicVehicleContainerHighFrequency_t& bvc = hfc.choice.basicVehicleContainerHighFrequency;
//     bvc.heading.headingValue = ptr->high_frequency_container.heading.value;
//     bvc.heading.headingConfidence = ptr->high_frequency_container.heading.confidence;
//     bvc.speed.speedValue = ptr->high_frequency_container.speed.value;
//     bvc.speed.speedConfidence = ptr->high_frequency_container.speed.confidence;
//     bvc.driveDirection = ptr->high_frequency_container.drive_direction.value;
//     bvc.longitudinalAcceleration.longitudinalAccelerationValue =
//         ptr->high_frequency_container.longitudinal_acceleration.value;
//     bvc.longitudinalAcceleration.longitudinalAccelerationConfidence =
//         ptr->high_frequency_container.longitudinal_acceleration.confidence;
//     bvc.curvature.curvatureValue = ptr->high_frequency_container.curvature.value;
//     bvc.curvature.curvatureConfidence = ptr->high_frequency_container.curvature.confidence;
//     bvc.curvatureCalculationMode = ptr->high_frequency_container.curvature_calculation_mode.value;
//     bvc.yawRate.yawRateValue = ptr->high_frequency_container.yaw_rate.value;
//     bvc.yawRate.yawRateConfidence = ptr->high_frequency_container.yaw_rate.confidence;
//     bvc.vehicleLength.vehicleLengthValue = ptr->high_frequency_container.vehicle_length.value;
//     bvc.vehicleLength.vehicleLengthConfidenceIndication =
//         ptr->high_frequency_container.vehicle_length.confidence_indication;
//     bvc.vehicleWidth = ptr->high_frequency_container.vehicle_width.value;
//     if (ptr->high_frequency_container.has_acceleration_control)
//     {
//         bvc.accelerationControl = vanetza::asn1::allocate<AccelerationControl_t>();
//         bvc.accelerationControl->buf = static_cast<uint8_t*>(vanetza::asn1::allocate(1));
//         bvc.accelerationControl->size = 1;
//         bvc.accelerationControl->buf[0] = reverse_byte(ptr->high_frequency_container.acceleration_control.value);
//     }

//     if (ptr->has_low_frequency_container)
//     {
//         ca.camParameters.lowFrequencyContainer = vanetza::asn1::allocate<LowFrequencyContainer_t>();
//         ca.camParameters.lowFrequencyContainer->present = LowFrequencyContainer_PR_basicVehicleContainerLowFrequency;

//         BasicVehicleContainerLowFrequency& bvcl = ca.camParameters.lowFrequencyContainer->choice.basicVehicleContainerLowFrequency;
//         bvcl.vehicleRole = ptr->low_frequency_container.vehicle_role.value;
//         bvcl.exteriorLights.buf = static_cast<uint8_t*>(vanetza::asn1::allocate(1));
//         bvcl.exteriorLights.size = 1;
//         bvcl.exteriorLights.buf[0] = reverse_byte(ptr->low_frequency_container.exterior_lights.value);

//         for (const ros_etsi_its_msgs::msg::PathPoint& path_point : ptr->low_frequency_container.path_history.points)
//         {
//             PathPoint_t* pathPoint = vanetza::asn1::allocate<PathPoint_t>();
//             if (path_point.path_delta_time.value != ros_etsi_its_msgs::msg::PathDeltaTime::UNAVAILABLE)
//             {
//                 pathPoint->pathDeltaTime = vanetza::asn1::allocate<PathDeltaTime_t>();
//                 *pathPoint->pathDeltaTime = path_point.path_delta_time.value;
//             }
//             pathPoint->pathPosition.deltaAltitude = path_point.path_position.delta_altitude;
//             pathPoint->pathPosition.deltaLatitude = path_point.path_position.delta_latitude;
//             pathPoint->pathPosition.deltaLongitude = path_point.path_position.delta_longitude;
//             ASN_SEQUENCE_ADD(&bvcl.pathHistory, pathPoint);
//         }
//     }

//     return msg;
// }

} // namespace v2x_stack_btp


