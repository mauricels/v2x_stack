#include "cp_ts_message.h"
#include <boost/make_shared.hpp>
#include <etsi_its_cpm_ts_msgs/msg/collective_perception_message.hpp>
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

boost::shared_ptr<etsi_its_cpm_ts_msgs::msg::CollectivePerceptionMessage> convertCpm(const vanetza::asn1::r2::Cpm& asn1, std::string* error_msg)
{
    auto msg = boost::make_shared<etsi_its_cpm_ts_msgs::msg::CollectivePerceptionMessage>();

    // Header
    msg->header.protocol_version.value = asn1->header.protocolVersion;
    msg->header.message_id.value = asn1->header.messageId;
    msg->header.station_id.value = asn1->header.stationId;

    // Management Container
    asn_INTEGER2uint64(&asn1->payload.managementContainer.referenceTime, &msg->payload.management_container.reference_time.value);
    msg->payload.management_container.reference_position.altitude.altitude_value.value = asn1->payload.managementContainer.referencePosition.altitude.altitudeValue;
    msg->payload.management_container.reference_position.altitude.altitude_confidence.value = asn1->payload.managementContainer.referencePosition.altitude.altitudeConfidence;
    msg->payload.management_container.reference_position.latitude.value =  asn1->payload.managementContainer.referencePosition.latitude;
    msg->payload.management_container.reference_position.longitude.value = asn1->payload.managementContainer.referencePosition.longitude;
    msg->payload.management_container.reference_position.position_confidence_ellipse.semi_major_confidence.value = asn1->payload.managementContainer.referencePosition.positionConfidenceEllipse.semiMajorConfidence;
    msg->payload.management_container.reference_position.position_confidence_ellipse.semi_minor_confidence.value = asn1->payload.managementContainer.referencePosition.positionConfidenceEllipse.semiMinorConfidence;
    msg->payload.management_container.reference_position.position_confidence_ellipse.semi_major_orientation.value = asn1->payload.managementContainer.referencePosition.positionConfidenceEllipse.semiMajorOrientation;
    // TODO: Add optional MessageSegmentationInfo and MessageRateRange datafields

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

