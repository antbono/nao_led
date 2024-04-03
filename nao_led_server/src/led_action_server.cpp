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
#include <memory>
#include <thread>
//#include <vector>
#include <array>
//#include <chrono>
#include <string>     // std::string, std::stof
#include <cstring>
#include <iostream>   // std::cout

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "std_msgs/msg/color_rgba.hpp"

#include "nao_lola_command_msgs/msg/head_leds.hpp"
#include "nao_lola_command_msgs/msg/right_eye_leds.hpp"
#include "nao_lola_command_msgs/msg/left_eye_leds.hpp"
#include "nao_lola_command_msgs/msg/right_ear_leds.hpp"
#include "nao_lola_command_msgs/msg/left_ear_leds.hpp"
#include "nao_lola_command_msgs/msg/chest_led.hpp"
#include "nao_lola_command_msgs/msg/right_foot_led.hpp"
#include "nao_lola_command_msgs/msg/left_foot_led.hpp"

#include "nao_led_server/led_action_server.hpp"

#include "nao_led_interfaces/msg/led_indexes.hpp"
#include "nao_led_interfaces/msg/led_modes.hpp"
#include "nao_led_interfaces/action/leds_play.hpp"




namespace nao_led_action_server {

LedsPlayActionServer::LedsPlayActionServer(const rclcpp::NodeOptions & options)
    : rclcpp::Node("leds_play_action_server_node", options) {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<nao_led_interfaces::action::LedsPlay>(
                               this,
                               "leds_play",
                               std::bind(&LedsPlayActionServer::handleGoal, this, _1, _2),
                               std::bind(&LedsPlayActionServer::handleCancel, this, _1),
                               std::bind(&LedsPlayActionServer::handleAccepted, this, _1));

    this->head_pub_ = this->create_publisher<nao_lola_command_msgs::msg::HeadLeds>(
                          "effectors/head_leds", 10);
    this->right_eye_pub_ = this->create_publisher<nao_lola_command_msgs::msg::RightEyeLeds>(
                               "effectors/right_eye_leds", 10);
    this->left_eye_pub_ = this->create_publisher<nao_lola_command_msgs::msg::LeftEyeLeds>(
                              "effectors/left_eye_leds", 10);
    this->right_ear_pub_ = this->create_publisher<nao_lola_command_msgs::msg::RightEarLeds>(
                               "effectors/right_ear_leds", 10);
    this->left_ear_pub_ = this->create_publisher<nao_lola_command_msgs::msg::LeftEarLeds>(
                              "effectors/left_ear_leds", 10);
    this->chest_pub_ = this->create_publisher<nao_lola_command_msgs::msg::ChestLed>(
                           "effectors/chest_led", 10);
    this->right_foot_pub_ = this->create_publisher<nao_lola_command_msgs::msg::RightFootLed>(
                                "effectors/right_foot_led", 10);
    this->left_foot_pub_ = this->create_publisher<nao_lola_command_msgs::msg::LeftFootLed>(
                               "effectors/left_foot_led", 10);

    this->color_off_ = std_msgs::msg::ColorRGBA();
    this->color_off_.r = 0.0;
    this->color_off_.g = 0.0;
    this->color_off_.b = 0.0;

    RCLCPP_INFO(this->get_logger(), "LedsPlayActionServer Initialized");

}

LedsPlayActionServer::~LedsPlayActionServer() {}


rclcpp_action::GoalResponse LedsPlayActionServer::handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const nao_led_interfaces::action::LedsPlay::Goal> goal) {

    RCLCPP_INFO( this->get_logger(), "Received LED goal request");
    (void)uuid;

    //TODO checks on goal integrity

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

}

rclcpp_action::CancelResponse LedsPlayActionServer::handleCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<nao_led_interfaces::action::LedsPlay>> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel LED goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void LedsPlayActionServer::handleAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<nao_led_interfaces::action::LedsPlay>> goal_handle) {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&LedsPlayActionServer::execute, this, _1), goal_handle} .detach();
}

void LedsPlayActionServer::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<nao_led_interfaces::action::LedsPlay>> goal_handle) {

    RCLCPP_INFO(this->get_logger(), "Executing LED goal");

    auto goal = goal_handle->get_goal();
    bool canceled = false;

    std::array<uint8_t, 2> leds = goal->leds;
    uint8_t mode = goal->mode;
    float frequency = goal->frequency;
    std::array<std_msgs::msg::ColorRGBA, 8> colors = goal->colors;
    std::array<float, 12> intensities = goal->intensities;
    float duration = goal->duration;

    auto feedback = std::make_shared<nao_led_interfaces::action::LedsPlay::Feedback>();
    auto result = std::make_shared<nao_led_interfaces::action::LedsPlay::Result>();


    if (mode == nao_led_interfaces::msg::LedModes::STEADY) {
        steadyMode(goal_handle, leds, frequency, colors, intensities);

    } else if (mode == nao_led_interfaces::msg::LedModes::BLINKING) {

        canceled = blinkingMode(goal_handle, leds, frequency, colors );
        if (canceled) {
            result->success = true;
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "LED Goal canceled");
            return;
        }
        //

    } else if (mode == nao_led_interfaces::msg::LedModes::LOOP) {
        canceled = loopMode(goal_handle, leds, frequency, colors);
        if (canceled) {
            result->success = true;
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "LED Goal canceled");
            return;
        }
    }

    if (rclcpp::ok()) {
        result->success = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "LED Goal succeeded");
    }

}


void LedsPlayActionServer::steadyMode(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<nao_led_interfaces::action::LedsPlay>> goal_handle,
    std::array<uint8_t, 2> & leds, float frequency,
    std::array<std_msgs::msg::ColorRGBA, 8> & colors,
    std::array<float, 12> & intensities) {

    auto result = std::make_shared<nao_led_interfaces::action::LedsPlay::Result>();

    if (leds[0] == nao_led_interfaces::msg::LedIndexes::HEAD) {
        nao_lola_command_msgs::msg::HeadLeds head_leds_msg;
        for (unsigned i = 0; i < nao_lola_command_msgs::msg::HeadLeds::NUM_LEDS; ++i) {
            head_leds_msg.intensities[i] = intensities[i];
        }
        head_pub_->publish(head_leds_msg);
    }

    if ((leds[0] == nao_led_interfaces::msg::LedIndexes::REYE && leds[1] == nao_led_interfaces::msg::LedIndexes::LEYE) ||
        (leds[0] == nao_led_interfaces::msg::LedIndexes::LEYE && leds[1] == nao_led_interfaces::msg::LedIndexes::REYE))  {
        nao_lola_command_msgs::msg::RightEyeLeds right_eye_leds_msg;
        nao_lola_command_msgs::msg::LeftEyeLeds left_eye_leds_msg;
        for (unsigned i = 0; i < nao_lola_command_msgs::msg::RightEyeLeds::NUM_LEDS; ++i) {
            right_eye_leds_msg.colors[i] = colors[i];
            left_eye_leds_msg.colors[i] = colors[i];
        }

        right_eye_pub_->publish(right_eye_leds_msg);
        left_eye_pub_->publish(left_eye_leds_msg);

    }

    if ((leds[0] == nao_led_interfaces::msg::LedIndexes::REAR && leds[1] == nao_led_interfaces::msg::LedIndexes::LEAR) ||
        (leds[0] == nao_led_interfaces::msg::LedIndexes::LEAR && leds[1] == nao_led_interfaces::msg::LedIndexes::REAR)) {
        nao_lola_command_msgs::msg::RightEarLeds right_ear_leds_msg;
        nao_lola_command_msgs::msg::LeftEarLeds left_ear_leds_msg;
        for (unsigned i = 0; i < nao_lola_command_msgs::msg::RightEarLeds::NUM_LEDS; ++i) {
            right_ear_leds_msg.intensities[i] = intensities[i];
            left_ear_leds_msg.intensities[i] = intensities[i];
        }

        right_ear_pub_->publish(right_ear_leds_msg);
        left_ear_pub_->publish(left_ear_leds_msg);
    }

    if (leds[0] == nao_led_interfaces::msg::LedIndexes::CHEST) {
        nao_lola_command_msgs::msg::ChestLed chest_led_msg;
        chest_led_msg.color = colors[0];
        chest_pub_->publish(chest_led_msg);
    }

}


bool LedsPlayActionServer::blinkingMode(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<nao_led_interfaces::action::LedsPlay>> goal_handle,
    std::array<uint8_t, 2> & leds, float frequency,
    std::array<std_msgs::msg::ColorRGBA, 8> & colors) {

    rclcpp::Rate loop_rate(frequency);
    bool canceled = false;

    if (leds[0] == nao_led_interfaces::msg::LedIndexes::HEAD) {

        nao_lola_command_msgs::msg::HeadLeds head_leds_on;
        nao_lola_command_msgs::msg::HeadLeds head_leds_off;
        for (unsigned i = 0; i < nao_lola_command_msgs::msg::HeadLeds::NUM_LEDS; ++i) {
            head_leds_on.intensities[i] = 1.0;
            head_leds_off.intensities[i] = 0.0;
        }
        uint8_t c = 0;
        while (rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
                return canceled = true;
            }
            c = (c + 1) % 2;

            if (c == 0) {
                head_pub_->publish(head_leds_on);
            } else {
                head_pub_->publish(head_leds_off);
            }
            loop_rate.sleep();
        }
    }


    if ((leds[0] == nao_led_interfaces::msg::LedIndexes::REYE && leds[1] == nao_led_interfaces::msg::LedIndexes::LEYE)||
        (leds[0] == nao_led_interfaces::msg::LedIndexes::LEYE && leds[1] == nao_led_interfaces::msg::LedIndexes::REYE)) {
        nao_lola_command_msgs::msg::RightEyeLeds right_eye_leds_off;
        nao_lola_command_msgs::msg::LeftEyeLeds left_eye_leds_off;
        nao_lola_command_msgs::msg::RightEyeLeds right_eye_leds_on;
        nao_lola_command_msgs::msg::LeftEyeLeds left_eye_leds_on;
        for (unsigned i = 0; i < nao_lola_command_msgs::msg::RightEyeLeds::NUM_LEDS; ++i) {
            right_eye_leds_off.colors[i] = color_off_;
            left_eye_leds_off.colors[i] = color_off_;
            right_eye_leds_on.colors[i] = colors[i];
            left_eye_leds_on.colors[i] = colors[i];
        }
        uint8_t c = 0;
        while (rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
                return canceled = true;
            }
            c = (c + 1) % 2;

            if (c == 0) {
                right_eye_pub_->publish(right_eye_leds_off);
                left_eye_pub_->publish(left_eye_leds_off);

            } else {
                right_eye_pub_->publish(right_eye_leds_on);
                left_eye_pub_->publish(left_eye_leds_on);
            }
            loop_rate.sleep();
        }
    }

    if ((leds[0] == nao_led_interfaces::msg::LedIndexes::REAR && leds[1] == nao_led_interfaces::msg::LedIndexes::LEAR) ||
        (leds[0] == nao_led_interfaces::msg::LedIndexes::LEAR && leds[1] == nao_led_interfaces::msg::LedIndexes::REAR)) {
        nao_lola_command_msgs::msg::RightEarLeds right_ear_leds_off;
        nao_lola_command_msgs::msg::LeftEarLeds left_ear_leds_off;
        nao_lola_command_msgs::msg::RightEarLeds right_ear_leds_on;
        nao_lola_command_msgs::msg::LeftEarLeds left_ear_leds_on;
        for (unsigned i = 0; i < nao_lola_command_msgs::msg::RightEarLeds::NUM_LEDS; ++i) {
            right_ear_leds_off.intensities[i] = 0.0;
            left_ear_leds_off.intensities[i] = 0.0;
            right_ear_leds_on.intensities[i] = 1.0;
            left_ear_leds_on.intensities[i] = 1.0;
        }
        uint8_t c = 0;
        while (rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
                return canceled = true;
            }
            c = (c + 1) % 2;

            if (c == 0) {
                right_ear_pub_->publish(right_ear_leds_off);
                left_ear_pub_->publish(left_ear_leds_off);

            } else {
                right_ear_pub_->publish(right_ear_leds_on);
                left_ear_pub_->publish(left_ear_leds_on);
            }
            loop_rate.sleep();
        }
    }

    if (leds[0] == nao_led_interfaces::msg::LedIndexes::CHEST) {

        nao_lola_command_msgs::msg::ChestLed chest_led_on;
        nao_lola_command_msgs::msg::ChestLed chest_led_off;
        chest_led_on.color = colors[0];
        chest_led_off.color = color_off_;
        uint8_t c = 0;
        while (rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
                return canceled = true;
            }
            c = (c + 1) % 2;

            if (c == 0) {
                chest_pub_->publish(chest_led_on);
            } else {
                chest_pub_->publish(chest_led_off);
            }
            loop_rate.sleep();
        }
    }
    RCLCPP_ERROR(this->get_logger(), "LED blinking mode not executed");
    return canceled = false;
}


bool LedsPlayActionServer::loopMode(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<nao_led_interfaces::action::LedsPlay>> goal_handle,
    std::array<uint8_t, 2> & leds,
    float frequency,
    std::array<std_msgs::msg::ColorRGBA, 8> & colors) {

    rclcpp::Rate loop_rate(frequency);
    bool canceled = false;

    //HEAD
    if (leds[0] == nao_led_interfaces::msg::LedIndexes::HEAD) {

        nao_lola_command_msgs::msg::HeadLeds head_leds_msg;

        uint8_t c = 0;
        while (rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
                return canceled = true;
            }
            c = (c + 1) % nao_lola_command_msgs::msg::HeadLeds::NUM_LEDS;

            for (unsigned i = 0; i < nao_lola_command_msgs::msg::HeadLeds::NUM_LEDS; ++i) {
                head_leds_msg.intensities[i] = 1.0;
            }
            head_leds_msg.intensities[c] = 0.0;

            head_pub_->publish(head_leds_msg);

            loop_rate.sleep();
        }
    }

    // EYES
    if ((leds[0] == nao_led_interfaces::msg::LedIndexes::REYE && leds[1] == nao_led_interfaces::msg::LedIndexes::LEYE) ||
        (leds[0] == nao_led_interfaces::msg::LedIndexes::LEYE && leds[1] == nao_led_interfaces::msg::LedIndexes::REYE)) {
        nao_lola_command_msgs::msg::RightEyeLeds right_eye_leds_msg;
        nao_lola_command_msgs::msg::LeftEyeLeds left_eye_leds_msg;
        short int cw = 0, cw_succ = 0;
        short int ccw = 0, ccw_succ = 0;
        const short int num_eye_leds = nao_lola_command_msgs::msg::RightEyeLeds::NUM_LEDS;
        while (rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
                return canceled = true;
            }
            
            for (unsigned i = 0; i < num_eye_leds; ++i) {
                right_eye_leds_msg.colors[i] = colors[i];
                left_eye_leds_msg.colors[i] = colors[i];
            }

            cw = (cw + 1) % num_eye_leds;
            cw_succ = (cw+1) % num_eye_leds;

            ccw = (cw != 0) ? num_eye_leds-cw : 0;
            ccw_succ = (ccw != 0) ? ccw-1 : 7;

            right_eye_leds_msg.colors[cw] = color_off_;
            right_eye_leds_msg.colors[cw_succ] = color_off_;
            left_eye_leds_msg.colors[ccw] = color_off_;
            left_eye_leds_msg.colors[ccw_succ] = color_off_;
            right_eye_pub_->publish(right_eye_leds_msg);
            left_eye_pub_->publish(left_eye_leds_msg);

            loop_rate.sleep();
        }
    }
    if (leds[0] == nao_led_interfaces::msg::LedIndexes::REYE ) {
        nao_lola_command_msgs::msg::RightEyeLeds right_eye_leds_msg;
        uint8_t c = 0;
        while (rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
                return canceled = true;
            }
            c = (c + 1) % nao_lola_command_msgs::msg::RightEyeLeds::NUM_LEDS;

            for (unsigned i = 0; i < nao_lola_command_msgs::msg::RightEyeLeds::NUM_LEDS; ++i) {
                right_eye_leds_msg.colors[i] = colors[i];
            }
            right_eye_leds_msg.colors[c] = color_off_;
            right_eye_pub_->publish(right_eye_leds_msg);

            loop_rate.sleep();
        }
    }
    if (leds[0] == nao_led_interfaces::msg::LedIndexes::LEYE) {
        nao_lola_command_msgs::msg::LeftEyeLeds left_eye_leds_msg;
        uint8_t c = 0;
        while (rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
                return canceled = true;
            }
            c = (c + 1) % nao_lola_command_msgs::msg::RightEyeLeds::NUM_LEDS;

            for (unsigned i = 0; i < nao_lola_command_msgs::msg::RightEyeLeds::NUM_LEDS; ++i) {
                left_eye_leds_msg.colors[i] = colors[i];
            }
            left_eye_leds_msg.colors[c] = color_off_;
            left_eye_pub_->publish(left_eye_leds_msg);

            loop_rate.sleep();
        }
    }

    //EARS
    if ( (leds[0] == nao_led_interfaces::msg::LedIndexes::REAR && leds[1] == nao_led_interfaces::msg::LedIndexes::LEAR) ||
         (leds[0] == nao_led_interfaces::msg::LedIndexes::LEAR && leds[1] == nao_led_interfaces::msg::LedIndexes::REAR)) {

        nao_lola_command_msgs::msg::RightEarLeds right_ear_leds_msg;
        nao_lola_command_msgs::msg::LeftEarLeds left_ear_leds_msg;

        uint8_t c = 0;
        while (rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
                return canceled = true;
            }
            c = (c + 1) % nao_lola_command_msgs::msg::RightEarLeds::NUM_LEDS;
            for (unsigned i = 0; i < nao_lola_command_msgs::msg::RightEarLeds::NUM_LEDS; ++i) {
                right_ear_leds_msg.intensities[i] = 1.0;
                left_ear_leds_msg.intensities[i] = 1.0;
            }
            right_ear_leds_msg.intensities[c] = 0.0;
            left_ear_leds_msg.intensities[c] = 0.0;
            right_ear_pub_->publish(right_ear_leds_msg);
            left_ear_pub_->publish(left_ear_leds_msg);

            loop_rate.sleep();
        }
    }
    if (leds[0] == nao_led_interfaces::msg::LedIndexes::REAR) {

        nao_lola_command_msgs::msg::RightEarLeds right_ear_leds_msg;

        uint8_t c = 0;
        while (rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
                return canceled = true;
            }
            c = (c + 1) % nao_lola_command_msgs::msg::RightEarLeds::NUM_LEDS;
            for (unsigned i = 0; i < nao_lola_command_msgs::msg::RightEarLeds::NUM_LEDS; ++i) {
                right_ear_leds_msg.intensities[i] = 1.0;
            }
            right_ear_leds_msg.intensities[c] = 0.0;
            right_ear_pub_->publish(right_ear_leds_msg);

            loop_rate.sleep();
        }
    }
    if (leds[0] == nao_led_interfaces::msg::LedIndexes::LEAR) {

        nao_lola_command_msgs::msg::LeftEarLeds left_ear_leds_msg;

        uint8_t c = 0;
        while (rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
                return canceled = true;
            }
            c = (c + 1) % nao_lola_command_msgs::msg::RightEarLeds::NUM_LEDS;
            for (unsigned i = 0; i < nao_lola_command_msgs::msg::RightEarLeds::NUM_LEDS; ++i) {
                left_ear_leds_msg.intensities[i] = 1.0;
            }
            left_ear_leds_msg.intensities[c] = 0.0;
            left_ear_pub_->publish(left_ear_leds_msg);

            loop_rate.sleep();
        }
    }

    RCLCPP_ERROR(this->get_logger(), "LED loop mode not executed");
    return canceled = false;
}


}  // namespace nao_led_action_server

RCLCPP_COMPONENTS_REGISTER_NODE(nao_led_action_server::LedsPlayActionServer)