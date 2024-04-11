// Copyright 2024 Antonio Bono
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAO_LED_SERVER__LED_ACTION_SERVER_HPP_
#define NAO_LED_SERVER__LED_ACTION_SERVER_HPP_

#include <functional>
#include <memory>
#include <thread>
//#include <vector>
#include <array>
//#include <chrono>
#include <cstring>
#include <iostream>  // std::cout
#include <string>    // std::string, std::stof

#include "nao_led_interfaces/action/leds_play.hpp"
#include "nao_led_interfaces/msg/led_indexes.hpp"
#include "nao_led_interfaces/msg/led_modes.hpp"
#include "nao_lola_command_msgs/msg/chest_led.hpp"
#include "nao_lola_command_msgs/msg/head_leds.hpp"
#include "nao_lola_command_msgs/msg/left_ear_leds.hpp"
#include "nao_lola_command_msgs/msg/left_eye_leds.hpp"
#include "nao_lola_command_msgs/msg/left_foot_led.hpp"
#include "nao_lola_command_msgs/msg/right_ear_leds.hpp"
#include "nao_lola_command_msgs/msg/right_eye_leds.hpp"
#include "nao_lola_command_msgs/msg/right_foot_led.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/color_rgba.hpp"

namespace nao_led_action_server
{

using LedsPlay = nao_led_interfaces::action::LedsPlay;
using GoalHandleLedsPlay = rclcpp_action::ServerGoalHandle<LedsPlay>;
using LedIndexes = nao_led_interfaces::msg::LedIndexes;
using LedModes = nao_led_interfaces::msg::LedModes;

class LedsPlayActionServer : public rclcpp::Node
{
public:
  explicit LedsPlayActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});
  virtual ~LedsPlayActionServer();

private:
  rclcpp_action::Server<nao_led_interfaces::action::LedsPlay>::SharedPtr action_server_;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<nao_led_interfaces::action::LedsPlay>>
  goal_handle_;

  rclcpp::Publisher<nao_lola_command_msgs::msg::HeadLeds>::SharedPtr head_pub_;
  rclcpp::Publisher<nao_lola_command_msgs::msg::RightEyeLeds>::SharedPtr right_eye_pub_;
  rclcpp::Publisher<nao_lola_command_msgs::msg::LeftEyeLeds>::SharedPtr left_eye_pub_;
  rclcpp::Publisher<nao_lola_command_msgs::msg::RightEarLeds>::SharedPtr right_ear_pub_;
  rclcpp::Publisher<nao_lola_command_msgs::msg::LeftEarLeds>::SharedPtr left_ear_pub_;
  rclcpp::Publisher<nao_lola_command_msgs::msg::ChestLed>::SharedPtr chest_pub_;
  rclcpp::Publisher<nao_lola_command_msgs::msg::RightFootLed>::SharedPtr right_foot_pub_;
  rclcpp::Publisher<nao_lola_command_msgs::msg::LeftFootLed>::SharedPtr left_foot_pub_;

  std_msgs::msg::ColorRGBA color_off_;
  // color_off_.r=0.0; color_off_.g=0.0; color_off_.b=0.0;
  nao_led_interfaces::msg::LedIndexes led_indexes_msg_;
  nao_led_interfaces::msg::LedModes led_modes_msg_;

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const nao_led_interfaces::action::LedsPlay::Goal> goal);

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<nao_led_interfaces::action::LedsPlay>>
    goal_handle);

  void handleAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<nao_led_interfaces::action::LedsPlay>>
    goal_handle);

  void execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<nao_led_interfaces::action::LedsPlay>>
    goal_handle);

  void steadyMode(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<nao_led_interfaces::action::LedsPlay>>
    goal_handle,
    std::array<uint8_t, 2> & leds, float frequency,
    std::array<std_msgs::msg::ColorRGBA, 8> & colors, std::array<float, 12> & intensities);

  bool blinkingMode(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<nao_led_interfaces::action::LedsPlay>>
    goal_handle,
    std::array<uint8_t, 2> & leds, float frequency,
    std::array<std_msgs::msg::ColorRGBA, 8> & colors);

  bool loopMode(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<nao_led_interfaces::action::LedsPlay>>
    goal_handle,
    std::array<uint8_t, 2> & leds, float frequency,
    std::array<std_msgs::msg::ColorRGBA, 8> & colors);
};

}  // namespace nao_led_action_server

#endif  // NAO_LED_SERVER__LED_ACTION_SERVER_HPP_
