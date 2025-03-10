#pragma once

#include <rclcpp/rclcpp.hpp>
#include <vanetza/asn1/cpm.hpp>
#include <v2x_stack_btp/msg/btp_data_indication.hpp>
#include <cstdint>

namespace v2x_stack_btp
{

class CpTsRxNode : public rclcpp::Node
{
public:
    explicit CpTsRxNode(const rclcpp::NodeOptions & options);
    void onIndication(const msg::BtpDataIndication::ConstSharedPtr);

private:

    void publish(const vanetza::asn1::r2::Cpm&);

    uint16_t port_;
    rclcpp::Subscription<msg::BtpDataIndication>::SharedPtr sub_btp_;
    std::shared_ptr<rclcpp::Publisher<etsi_its_cpm_ts_msgs::msg::CollectivePerceptionMessage>> pub_cpm_ts_;
    rclcpp::Node::SharedPtr node_;
};

} // namespace v2x_stack_btp