#pragma once

#include <rclcpp/rclcpp.hpp>
#include <vanetza/asn1/cam.hpp>
#include <vanetza/asn1/vam.hpp>
#include <v2x_stack_btp/msg/btp_data_indication.hpp>
#include <ros_etsi_its_msgs/msg/cam.hpp>
#include <ros_etsi_its_msgs/msg/vam.hpp>
#include <string>

namespace v2x_stack_btp
{

class CaRxNode : public rclcpp::Node
{
public:
    explicit CaRxNode(const rclcpp::NodeOptions & options);

    void onIndication(msg::BtpDataIndication::ConstSharedPtr indication);

private:
    void publishCam(const vanetza::asn1::r1::Cam& asn1_cam);
    void publishVam(const vanetza::asn1::r2::Vam& asn1_vam);

    rclcpp::Publisher<ros_etsi_its_msgs::msg::CAM>::SharedPtr pub_cam_;
    rclcpp::Publisher<ros_etsi_its_msgs::msg::VAM>::SharedPtr pub_vam_;
};

} // namespace v2x_stack_btp