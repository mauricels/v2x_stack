#include "cp_ts_message.h"
#include "cp_ts_rx.h"
#include <vanetza/btp/ports.hpp>

namespace v2x_stack_btp
{

CpTsRxNode::CpTsRxNode(const rclcpp::NodeOptions & options)
: Node("cp_ts_rx_node", options)
{
}

void CpTsRxNode::onIndication(msg::BtpDataIndication::ConstSharedPtr indication)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Indication CP TS");
    if (indication->btp_type == msg::BtpDataIndication::BTP_TYPE_B && indication->destination_port == 2009)
    {
        /*
        vanetza::asn1::r1::Cpm cpm;
        if (cpm.decode(indication->data))
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Well decoded CP");
            publish(cpm);
        }*/

        vanetza::asn1::r2::Cpm cpm = vanetza::asn1::r2::Cpm();// = nullptr;
        const std::vector<unsigned char>& payload = indication->data;
        const uint8_t* buffer = payload.data();

        auto ok = vanetza::asn1::decode_per((asn_TYPE_descriptor_t&)(asn_DEF_Vanetza_ITS2_CollectivePerceptionMessage), (void**)(&cpm), (const void*)(buffer), indication->data.size());

        if (ok)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Well decoded CPM TS");
            publish(cpm);
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "ASN.1 decoding failed, drop received CPM");            
        }
    }
}

void CpTsRxNode::publish(const vanetza::asn1::r2::Cpm& asn1)
{
    std::string error_msg;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Convert CPM TS");
    auto msg = convertCpm(asn1, &error_msg);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Publish CPM TS");


    if (msg)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "There is CP TS");
        node_ = std::make_shared<rclcpp::Node>("cpm_ts_rx");
        pub_cpm_ts_ = node_->create_publisher<etsi_its_cpm_ts_msgs::msg::CollectivePerceptionMessage>("cpm_received_ts", 20);
    
        pub_cpm_ts_->publish(*msg);
    }
}


} // namespace v2x_stack_btp

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting CP TS RX");

   auto node = std::make_shared<v2x_stack_btp::CpTsRxNode>(rclcpp::NodeOptions());
   auto subscription = node->create_subscription<v2x_stack_btp::msg::BtpDataIndication>("btp_data", 20, std::bind(&v2x_stack_btp::CpTsRxNode::onIndication, node, std::placeholders::_1));

   rclcpp::spin(node);
   rclcpp::shutdown();

    return 0;
}