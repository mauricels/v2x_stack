#pragma once

#include <boost/shared_ptr.hpp>
#include <ros_etsi_its_msgs/msg/cam.hpp>
#include <string>

// forward declaration
namespace vanetza { namespace asn1 { namespace r1 {class Cam;} } }

namespace v2x_stack_btp
{

/**
 * Convert ASN.1 data structure to ROS etsi_its_msg CAM
 * \param cam ASN.1 data structure
 * \param msg optional error string
 * \return converted CAM (or nullptr on error)
 */
boost::shared_ptr<ros_etsi_its_msgs::msg::CAM> convertCam(const vanetza::asn1::r1::Cam& cam, std::string* msg = nullptr);

/**
 * Convert ROS etsi_its_msg CAM to ASN.1 data structure
 * \param ptr etsi_its_msgs CAM
 * \return converted CAM
 */
vanetza::asn1::r1::Cam convertCam(ros_etsi_its_msgs::msg::CAM::ConstSharedPtr ptr);

} // namespace v2x_stack_btp