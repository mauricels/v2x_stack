#include "ca_message.h"
#include "ca_rx.h"
#include <vanetza/btp/ports.hpp>

namespace v2x_stack_btp
{

CaRxNode::CaRxNode(const rclcpp::NodeOptions & options)
: Node("ca_rx_node", options)
{
    pub_cam_ = this->create_publisher<ros_etsi_its_msgs::msg::CAM>("cam_received", 20);
    pub_vam_ = this->create_publisher<ros_etsi_its_msgs::msg::VAM>("vam_received", 20);
}

void CaRxNode::onIndication(msg::BtpDataIndication::ConstSharedPtr indication)
{
    RCLCPP_INFO(this->get_logger(), "Indication received");
    if (indication->btp_type == msg::BtpDataIndication::BTP_TYPE_B)
    {
        const uint8_t* buffer = indication->data.data();

        if (indication->destination_port == 2001) // CAM
        {
            vanetza::asn1::r1::Cam cam;
            auto ok = vanetza::asn1::decode_per((asn_TYPE_descriptor_t&)(asn_DEF_CAM), (void**)(&cam), (const void*)(buffer), indication->data.size());

            if (ok)
            {
                RCLCPP_INFO(this->get_logger(), "Well decoded CAM");
                publishCam(cam);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "ASN.1 decoding failed, drop received CAM");
            }
        }
        else if (indication->destination_port == 3001) // VAM
        {
            vanetza::asn1::r2::Vam vam;
            auto ok = vanetza::asn1::decode_per((asn_TYPE_descriptor_t&)(asn_DEF_Vanetza_ITS2_VAM), (void**)(&vam), (const void*)(buffer), indication->data.size());

            if (ok)
            {
                RCLCPP_INFO(this->get_logger(), "Well decoded VAM");
                publishVam(vam);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "ASN.1 decoding failed, drop received VAM");
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Unknown destination port: %d", indication->destination_port);
        }
    }
}

void CaRxNode::publishCam(const vanetza::asn1::r1::Cam& asn1_cam)
{
    RCLCPP_INFO(this->get_logger(), "Entering publishCam method");
    std::string error_msg;

    // Konvertiere ASN.1-CAM in eine ROS 2-kompatible Nachricht
    auto msg = convertCam(asn1_cam, &error_msg);
    RCLCPP_INFO(this->get_logger(), "Try to Publish CAM");

    if (msg)
    {
        RCLCPP_INFO(this->get_logger(), "Publishing CAM");
        pub_cam_->publish(*msg);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "CAM not correct");
        RCLCPP_ERROR(this->get_logger(), "Error cause: %s", error_msg.c_str());
    }
}
void CaRxNode::publishVam(const vanetza::asn1::r2::Vam& asn1_vam)
{
    RCLCPP_INFO(this->get_logger(), "Entering publishVam method");
    std::string error_msg;

    // Konvertiere ASN.1-VAM in eine ROS 2-kompatible Nachricht
    auto msg = convertVam(asn1_vam, &error_msg);
    RCLCPP_INFO(this->get_logger(), "Try to Publish VAM");

    if (msg)
    {
        RCLCPP_INFO(this->get_logger(), "Publishing VAM");
        pub_vam_->publish(*msg);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "VAM not correct");
        RCLCPP_ERROR(this->get_logger(), "Error cause: %s", error_msg.c_str());
    }
}

} // namespace v2x_stack_btp

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting CA RX");

    auto node = std::make_shared<v2x_stack_btp::CaRxNode>(rclcpp::NodeOptions());
    auto subscription = node->create_subscription<v2x_stack_btp::msg::BtpDataIndication>("btp_data", 20, std::bind(&v2x_stack_btp::CaRxNode::onIndication, node, std::placeholders::_1));

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}