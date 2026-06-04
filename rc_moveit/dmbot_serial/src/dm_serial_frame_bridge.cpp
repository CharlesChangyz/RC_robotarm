#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "arm_msgs/msg/can_frame.hpp"
#include "dmbot_serial/protocol/usb_class.h"
#include "rclcpp/rclcpp.hpp"

namespace
{

class DmSerialFrameBridge : public rclcpp::Node
{
public:
  DmSerialFrameBridge()
  : Node("dm_serial_frame_bridge")
  {
    declare_parameter<std::string>("sn", "9940F4E149D904A69924737E3DE6629F");
    declare_parameter<int64_t>("nom_baud", 1000000);
    declare_parameter<int64_t>("dat_baud", 2000000);
    declare_parameter<std::string>("rx_topic", "/rc_arm_2/dm_serial_rx");
    declare_parameter<std::string>("tx_topic", "/rc_arm_2/dm_serial_tx");

    const auto sn = get_parameter("sn").as_string();
    const auto nom_baud = static_cast<uint32_t>(get_parameter("nom_baud").as_int());
    const auto dat_baud = static_cast<uint32_t>(get_parameter("dat_baud").as_int());
    const auto rx_topic = get_parameter("rx_topic").as_string();
    const auto tx_topic = get_parameter("tx_topic").as_string();

    rx_pub_ = create_publisher<arm_msgs::msg::CanFrame>(rx_topic, 50);
    tx_sub_ = create_subscription<arm_msgs::msg::CanFrame>(
      tx_topic,
      50,
      std::bind(&DmSerialFrameBridge::onTxFrame, this, std::placeholders::_1));

    usb_hw_ = std::make_shared<usb_class>(nom_baud, dat_baud, sn);
    usb_hw_->setFrameCallback(
      [this](can_value_type & frame)
      {
        publishRxFrame(frame);
      });

    RCLCPP_INFO(
      get_logger(),
      "dm_serial_frame_bridge ready: sn=%s nom_baud=%u dat_baud=%u rx=%s tx=%s",
      sn.c_str(),
      nom_baud,
      dat_baud,
      rx_topic.c_str(),
      tx_topic.c_str());
  }

private:
  void publishRxFrame(const can_value_type & frame)
  {
    arm_msgs::msg::CanFrame msg;
    msg.id = frame.head.id;
    msg.is_extended = frame.head.id_type != 0;
    msg.is_remote = frame.head.fram_type != 0;
    msg.is_fd = frame.head.can_type != 0;
    msg.dlc = std::min<uint8_t>(frame.head.dlc, static_cast<uint8_t>(msg.data.size()));
    std::fill(msg.data.begin(), msg.data.end(), 0);
    std::copy(frame.data, frame.data + msg.dlc, msg.data.begin());
    rx_pub_->publish(msg);
  }

  void onTxFrame(const arm_msgs::msg::CanFrame::SharedPtr msg)
  {
    if (!msg || !usb_hw_) {
      return;
    }

    const auto dlc = std::min<uint8_t>(msg->dlc, static_cast<uint8_t>(msg->data.size()));
    std::vector<uint8_t> data(msg->data.begin(), msg->data.begin() + dlc);
    usb_hw_->fdcanFrameSend(data, msg->id);
    RCLCPP_INFO(get_logger(), "sent dmserial frame id=0x%X dlc=%u", msg->id, dlc);
  }

  std::shared_ptr<usb_class> usb_hw_;
  rclcpp::Publisher<arm_msgs::msg::CanFrame>::SharedPtr rx_pub_;
  rclcpp::Subscription<arm_msgs::msg::CanFrame>::SharedPtr tx_sub_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DmSerialFrameBridge>());
  rclcpp::shutdown();
  return 0;
}
