#include "den_message.h"
#include "den_rx.h"
#include <vanetza/btp/ports.hpp>
#include <chrono>

namespace v2x_stack_btp
{

DenRxNode::DenRxNode(const rclcpp::NodeOptions & options)
: Node("den_rx_node", options)
{
}

void DenRxNode::onIndication(msg::BtpDataIndication::ConstSharedPtr indication)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Indication DEN");
    if (indication->btp_type == msg::BtpDataIndication::BTP_TYPE_B && indication->destination_port == 2002)
    {

        auto current_time = this->get_clock()->now();
        auto time_since_last_publish = current_time.seconds() - last_publish_time_.seconds();

        get_parameter("cooldown", publish_cooldown_);

        /*
        vanetza::asn1::Denm denm;
        if (denm.decode(indication->data))
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Well decoded DEN");
            publish(denm);
        }*/
        vanetza::asn1::Denm denm;// = nullptr;
        const std::vector<unsigned char>& payload = indication->data;
        const uint8_t* buffer = payload.data();


        auto ok = vanetza::asn1::decode_per((asn_TYPE_descriptor_t&)(asn_DEF_DENM), (void**)(&denm), (const void*)(buffer), indication->data.size());
    
        if (ok)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Well decoded DEN");
            if(time_since_last_publish >= publish_cooldown_)
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Time interval OK - publishing DEN");
                publish(denm);
                last_publish_time_ = current_time;
            }
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "ASN.1 decoding failed, drop received DENM");            
        }
    }
}

void DenRxNode::publish(const vanetza::asn1::Denm asn1)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Entering publish method");
    std::string error_msg;
    auto msg = convertDenm(asn1, &error_msg);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Try to Publish DEN");

    

    if (msg)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "There is DEN");
        node_ = std::make_shared<rclcpp::Node>("denm_rx");
        pub_denm_ = node_->create_publisher<ros_etsi_its_msgs::msg::DENM>("denm_received", 20);
    
        pub_denm_->publish(*msg);
    } else {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "DENM not correct");
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Error cause: %s", error_msg.c_str());
    }
}

} // namespace v2x_stack_btp

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting DEN RX");

   auto node = std::make_shared<v2x_stack_btp::DenRxNode>(rclcpp::NodeOptions());
   auto subscription = node->create_subscription<v2x_stack_btp::msg::BtpDataIndication>("btp_data", 20, std::bind(&v2x_stack_btp::DenRxNode::onIndication, node, std::placeholders::_1));
   node->declare_parameter("cooldown", 0.0); // No cooldown by default

   rclcpp::spin(node);
   rclcpp::shutdown();

    return 0;
}