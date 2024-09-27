#pragma once

#include <rclcpp/rclcpp.hpp>
#include <vanetza/asn1/denm.hpp>
#include <v2x_stack_btp/msg/btp_data_indication.hpp>
#include <ros_etsi_its_msgs/msg/denm.hpp>
#include <cstdint>

namespace v2x_stack_btp
{

class DenRxNode : public rclcpp::Node
{
public:
    explicit DenRxNode(const rclcpp::NodeOptions & options);
    void onIndication(const msg::BtpDataIndication::ConstSharedPtr);

private:
    
    void publish(const vanetza::asn1::Denm);

    uint16_t port_;
    rclcpp::Subscription<msg::BtpDataIndication>::SharedPtr sub_btp_;
    //rclcpp::Publisher<ros_etsi_its_msgs::msg::DENM>pub_denm_;
    std::shared_ptr<rclcpp::Publisher<ros_etsi_its_msgs::msg::DENM>> pub_denm_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Time last_publish_time_;
    double publish_cooldown_;
};

} // namespace v2x_stack_btp