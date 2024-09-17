#pragma once

#include <boost/shared_ptr.hpp>
#include <ros_etsi_its_msgs/msg/denm.hpp>
#include <string>

// forward declaration
namespace vanetza { namespace asn1 { class Denm; } }

namespace v2x_stack_btp
{

/**
 * Convert ASN.1 data structure to ROS etsi_its_msg CPM
 * \param denm ASN.1 data structure
 * \param msg optional error string
 * \return converted DENM (or nullptr on error)
 */
boost::shared_ptr<ros_etsi_its_msgs::msg::DENM> convertDenm(const vanetza::asn1::Denm& denm, std::string* msg = nullptr);

/**
 * Convert ROS etsi_its_msg DENM to ASN.1 data structure
 * \param ptr etsi_its_msgs DENM
 * \return converted DENM
 */
//vanetza::asn1::Denm convertDenm(ros_etsi_its_msgs::msg::DENM::ConstSharedPtr ptr);

} // namespace v2x_stack_btp
