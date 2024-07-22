#include "cp_message.h"
#include "cp_rx.h"
#include <vanetza/btp/ports.hpp>

namespace v2x_stack_btp
{

CpRxNode::CpRxNode(const rclcpp::NodeOptions & options)
: Node("cp_rx_node", options)
{
}

void CpRxNode::onIndication(msg::BtpDataIndication::ConstSharedPtr indication)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Indication CP");
    if (indication->btp_type == msg::BtpDataIndication::BTP_TYPE_B && indication->destination_port == 2009)
    {
        /*
        vanetza::asn1::Cpm cpm;
        if (cpm.decode(indication->data))
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Well decoded CP");
            publish(cpm);
        }*/

        vanetza::asn1::Cpm cpm;// = nullptr;
        const std::vector<unsigned char>& payload = indication->data;
        const uint8_t* buffer = payload.data();


        auto ok = vanetza::asn1::decode_per((asn_TYPE_descriptor_t&)(asn_DEF_CPM), (void**)(&cpm), (const void*)(buffer), indication->data.size());

        if (ok)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Well decoded CPM");
            publish(cpm);
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "ASN.1 decoding failed, drop received CPM");            
        }
    }
}

void CpRxNode::publish(const vanetza::asn1::Cpm& asn1)
{
    std::string error_msg;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Convert CPM");
    auto msg = convertCpm(asn1, &error_msg);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Publish CP");


    if (msg)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "There is CP");
        node_ = std::make_shared<rclcpp::Node>("cpm_rx");
        pub_cpm_ = node_->create_publisher<ros_etsi_its_msgs::msg::CPM>("cpm_received", 20);
    
        pub_cpm_->publish(*msg);
    }
}

} // namespace v2x_stack_btp

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting CP RX");

   auto node = std::make_shared<v2x_stack_btp::CpRxNode>(rclcpp::NodeOptions());
   auto subscription = node->create_subscription<v2x_stack_btp::msg::BtpDataIndication>("btp_data", 20, std::bind(&v2x_stack_btp::CpRxNode::onIndication, node, std::placeholders::_1));

   rclcpp::spin(node);
   rclcpp::shutdown();

    return 0;
}