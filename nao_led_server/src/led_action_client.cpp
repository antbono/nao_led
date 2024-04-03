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

#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "boost/filesystem.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "nao_lola_command_msgs/msg/head_leds.hpp"
#include "nao_lola_command_msgs/msg/right_eye_leds.hpp"
#include "nao_lola_command_msgs/msg/left_eye_leds.hpp"
#include "nao_lola_command_msgs/msg/right_ear_leds.hpp"
#include "nao_lola_command_msgs/msg/left_ear_leds.hpp"
#include "nao_lola_command_msgs/msg/chest_led.hpp"
#include "nao_lola_command_msgs/msg/right_foot_led.hpp"
#include "nao_lola_command_msgs/msg/left_foot_led.hpp"

#include "nao_led_interfaces/action/leds_play.hpp"
#include "nao_led_interfaces/msg/led_indexes.hpp"
#include "nao_led_interfaces/msg/led_modes.hpp"
#include "nao_led_interfaces/action/leds_play.hpp"
#include "nao_led_server/led_action_client.hpp"

#include "std_msgs/msg/color_rgba.hpp"


namespace nao_led_action_client {

using LedsPlay = nao_led_interfaces::action::LedsPlay;
using GoalHandleLedsPlay = rclcpp_action::ClientGoalHandle<LedsPlay>;

using LedIndexes = nao_led_interfaces::msg::LedIndexes;
using LedModes = nao_led_interfaces::msg::LedModes;


LedsPlayActionClient::LedsPlayActionClient(const rclcpp::NodeOptions & options)
  : rclcpp::Node("led_action_client_node", options) {

  this->client_ptr_ = rclcpp_action::create_client<LedsPlay>(
                        this,
                        "leds_play");

  RCLCPP_INFO(this->get_logger(), "LedsPlayActionClient initialized");

}

LedsPlayActionClient::~LedsPlayActionClient() {}


void LedsPlayActionClient::eyesStatic( bool flag ) {

  using namespace std::placeholders;
  if (!this->client_ptr_->wait_for_action_server()) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    rclcpp::shutdown();
  }

  auto goal_msg = LedsPlay::Goal();

  goal_msg.leds = {LedIndexes::REYE, LedIndexes::LEYE};
  goal_msg.mode = LedModes::STEADY;
  std_msgs::msg::ColorRGBA color;
  if (flag) {
    color.r = 1.0; color.g = 1.0; color.b = 1.0;
  } else {
    color.r = 0.0; color.g = 0.0; color.b = 0.0;
  }

  for (unsigned i = 0; i < nao_lola_command_msgs::msg::RightEyeLeds::NUM_LEDS; ++i) {
    goal_msg.colors[i] = color;
  }

  auto send_goal_options = rclcpp_action::Client<LedsPlay>::SendGoalOptions();

  send_goal_options.goal_response_callback =
    std::bind(&LedsPlayActionClient::goalResponseCallback, this, _1);

  send_goal_options.feedback_callback =
    std::bind(&LedsPlayActionClient::feedbackCallback, this, _1, _2);

  send_goal_options.result_callback =
    std::bind(&LedsPlayActionClient::resultCallback, this, _1);

  RCLCPP_INFO(this->get_logger(), "Sending goal:" );

  this->client_ptr_->async_send_goal(goal_msg, send_goal_options);

}

void LedsPlayActionClient::chestStatic( bool flag ) {

  using namespace std::placeholders;
  if (!this->client_ptr_->wait_for_action_server()) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    rclcpp::shutdown();
  }

  auto goal_msg = LedsPlay::Goal();

  goal_msg.leds = {LedIndexes::CHEST};
  goal_msg.mode = LedModes::STEADY;
  std_msgs::msg::ColorRGBA color;
  if (flag) {
    color.r = 1.0; color.g = 1.0; color.b = 1.0;
  } else {
    color.r = 0.0; color.g = 0.0; color.b = 0.0;
  }

  goal_msg.colors[0] = color;

  auto send_goal_options = rclcpp_action::Client<LedsPlay>::SendGoalOptions();

  send_goal_options.goal_response_callback =
    std::bind(&LedsPlayActionClient::goalResponseCallback, this, _1);

  send_goal_options.feedback_callback =
    std::bind(&LedsPlayActionClient::feedbackCallback, this, _1, _2);

  send_goal_options.result_callback =
    std::bind(&LedsPlayActionClient::resultCallback, this, _1);

  RCLCPP_INFO(this->get_logger(), "Sending goal:" );

  this->client_ptr_->async_send_goal(goal_msg, send_goal_options);

}

void LedsPlayActionClient::headStatic(bool flag) {
  using namespace std::placeholders;

  if (!this->client_ptr_->wait_for_action_server()) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    rclcpp::shutdown();
  }

  auto goal_msg = LedsPlay::Goal();

  goal_msg.leds = {LedIndexes::HEAD};
  goal_msg.mode = LedModes::STEADY;
  for (unsigned i = 0; i < nao_lola_command_msgs::msg::HeadLeds::NUM_LEDS; ++i) {
    if (flag) {
      goal_msg.intensities[i] = 1.0;
    } else {goal_msg.intensities[i] = 0.0;}
  }

  auto send_goal_options = rclcpp_action::Client<LedsPlay>::SendGoalOptions();

  send_goal_options.goal_response_callback =
    std::bind(&LedsPlayActionClient::goalResponseCallback, this, _1);

  send_goal_options.feedback_callback =
    std::bind(&LedsPlayActionClient::feedbackCallback, this, _1, _2);

  send_goal_options.result_callback =
    std::bind(&LedsPlayActionClient::resultCallback, this, _1);

  RCLCPP_INFO(this->get_logger(), "Sending goal:" );

  this->client_ptr_->async_send_goal(goal_msg, send_goal_options);

}

void LedsPlayActionClient::earsStatic(bool flag) {
  using namespace std::placeholders;

  if (!this->client_ptr_->wait_for_action_server()) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    rclcpp::shutdown();
  }

  auto goal_msg = LedsPlay::Goal();

  goal_msg.leds = {LedIndexes::REAR, LedIndexes::LEAR};
  goal_msg.mode = LedModes::STEADY;
  for (unsigned i = 0; i < nao_lola_command_msgs::msg::RightEarLeds::NUM_LEDS; ++i) {
    if (flag) {
      goal_msg.intensities[i] = 1.0;
    } else {goal_msg.intensities[i] = 0.0;}
  }

  auto send_goal_options = rclcpp_action::Client<LedsPlay>::SendGoalOptions();

  send_goal_options.goal_response_callback =
    std::bind(&LedsPlayActionClient::goalResponseCallback, this, _1);

  send_goal_options.feedback_callback =
    std::bind(&LedsPlayActionClient::feedbackCallback, this, _1, _2);

  send_goal_options.result_callback =
    std::bind(&LedsPlayActionClient::resultCallback, this, _1);

  RCLCPP_INFO(this->get_logger(), "Sending goal:" );

  this->client_ptr_->async_send_goal(goal_msg, send_goal_options);

}

void LedsPlayActionClient::headLoop( bool flag )  {
  using namespace std::placeholders;

  if (!this->client_ptr_->wait_for_action_server()) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    rclcpp::shutdown();
  }

  if (flag) {
    auto goal_msg = LedsPlay::Goal();

    goal_msg.leds = {LedIndexes::HEAD};
    goal_msg.mode = LedModes::LOOP;
    for (unsigned i = 0; i < nao_lola_command_msgs::msg::HeadLeds::NUM_LEDS; ++i) {
      goal_msg.intensities[i] = 1.0;
    }
    goal_msg.frequency = 10.0;

    auto send_goal_options = rclcpp_action::Client<LedsPlay>::SendGoalOptions();

    send_goal_options.goal_response_callback =
      std::bind(&LedsPlayActionClient::goalResponseCallback, this, _1);

    send_goal_options.feedback_callback =
      std::bind(&LedsPlayActionClient::feedbackCallback, this, _1, _2);

    send_goal_options.result_callback =
      std::bind(&LedsPlayActionClient::resultCallback, this, _1);

    RCLCPP_INFO(this->get_logger(), "Sending goal:" );

    auto goal_handle_future = client_ptr_->async_send_goal(goal_msg, send_goal_options);
    head_goal_handle_ = goal_handle_future.get();
    //head_goal_handle_ = client_ptr_->async_send_goal(goal_msg, send_goal_options);
  } else {
    //rclcpp_action::ClientGoalHandle<nao_led_interfaces::action::LedsPlay>::SharedPtr handle = head_goal_handle_.get();
    //auto cancel_result_future = client_ptr_->async_cancel_goal(handle);
    auto cancel_result_future = client_ptr_->async_cancel_goal(head_goal_handle_);
  }
}

void LedsPlayActionClient::earsLoop(bool flag)  {
  using namespace std::placeholders;

  if (flag) {
    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = LedsPlay::Goal();

    goal_msg.leds = {LedIndexes::REAR, LedIndexes::LEAR};
    goal_msg.mode = LedModes::LOOP;
    for (unsigned i = 0; i < nao_lola_command_msgs::msg::RightEarLeds::NUM_LEDS; ++i) {
      goal_msg.intensities[i] = 1.0;
    }
    goal_msg.frequency = 10.0;

    auto send_goal_options = rclcpp_action::Client<LedsPlay>::SendGoalOptions();

    send_goal_options.goal_response_callback =
      std::bind(&LedsPlayActionClient::goalResponseCallback, this, _1);

    send_goal_options.feedback_callback =
      std::bind(&LedsPlayActionClient::feedbackCallback, this, _1, _2);

    send_goal_options.result_callback =
      std::bind(&LedsPlayActionClient::resultCallback, this, _1);

    RCLCPP_INFO(this->get_logger(), "Sending goal:" );

    auto goal_handle_future = client_ptr_->async_send_goal(goal_msg, send_goal_options);
    ears_goal_handle_ = goal_handle_future.get();
  } else {
    auto cancel_result_future = client_ptr_->async_cancel_goal(ears_goal_handle_);
  }
}


void LedsPlayActionClient::eyesLoop(bool flag)  {
  using namespace std::placeholders;

  if (flag) {
    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = LedsPlay::Goal();

    goal_msg.leds = {LedIndexes::REYE, LedIndexes::LEYE};
    goal_msg.mode = LedModes::LOOP;
    
    std_msgs::msg::ColorRGBA color;
    color.r = 1.0; color.g = 1.0; color.b = 1.0;

    for (unsigned i = 0; i < nao_lola_command_msgs::msg::RightEyeLeds::NUM_LEDS; ++i) {
      goal_msg.colors[i] = color;
    }

    goal_msg.frequency = 10.0;

    auto send_goal_options = rclcpp_action::Client<LedsPlay>::SendGoalOptions();

    send_goal_options.goal_response_callback =
      std::bind(&LedsPlayActionClient::goalResponseCallback, this, _1);

    //send_goal_options.feedback_callback =
    //  std::bind(&LedsPlayActionClient::feedbackCallback, this, _1, _2);

    send_goal_options.result_callback =
      std::bind(&LedsPlayActionClient::resultCallback, this, _1);

    RCLCPP_INFO(this->get_logger(), "Sending goal:" );

    auto goal_handle_future = client_ptr_->async_send_goal(goal_msg, send_goal_options);
    eyes_goal_handle_ = goal_handle_future.get();
  } else {
    auto cancel_result_future = client_ptr_->async_cancel_goal(eyes_goal_handle_);
  }
}



void LedsPlayActionClient::goalResponseCallback(const GoalHandleLedsPlay::SharedPtr & goal_handle) {
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  }
}

void LedsPlayActionClient::feedbackCallback(
  GoalHandleLedsPlay::SharedPtr,
  const std::shared_ptr<const LedsPlay::Feedback> feedback) {

  //TODO

}

void LedsPlayActionClient::resultCallback(const GoalHandleLedsPlay::WrappedResult & result) {
  switch (result.code) {
  case rclcpp_action::ResultCode::SUCCEEDED:
    break;
  case rclcpp_action::ResultCode::ABORTED:
    RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
    return;
  case rclcpp_action::ResultCode::CANCELED:
    RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
    return;
  default:
    RCLCPP_ERROR(this->get_logger(), "Unknown result code");
    return;
  }

  if (result.result->success)
    RCLCPP_INFO(this->get_logger(), "Leds regulary played.");

  rclcpp::shutdown();
}


}  // namespace nao_led_action_client

RCLCPP_COMPONENTS_REGISTER_NODE(nao_led_action_client::LedsPlayActionClient)