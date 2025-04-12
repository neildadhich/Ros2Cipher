// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/rclcpp.hpp"
#include "cipher_interfaces/msg/cipher_message.hpp"
#include "cipher_interfaces/srv/cipher_answer.hpp"
using std::placeholders::_1;
using namespace std::chrono_literals;

std::string caesar_decipher(const std::string &text, int key)
{
  std::string result = text;
  for (char &c : result) {
    if (std::isalpha(c)) {
      // Determine the base (depending on if the character is lowercase or uppercase).
      char base = std::islower(c) ? 'a' : 'A';
      // Subtract the key from the character's position while handling wrap-around.
      c = base + (c - base - key + 26) % 26;
    }
  }
  return result;
}

class CipherSubscriber : public rclcpp::Node
{
public:
  CipherSubscriber()
  : Node("cipher_subscriber")
  {
    subscription_ = this->create_subscription<cipher_interfaces::msg::CipherMessage>(
      "cipher_topic", 10, std::bind(&CipherSubscriber::topic_callback, this, _1));

    client_ = this->create_client<cipher_interfaces::srv::CipherAnswer>("verify_decoded");
  }

private:
  void topic_callback(const cipher_interfaces::msg::CipherMessage & msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Recieved encoded message: '%s', key: %d", msg.message.c_str(), msg.key);
  
    std::string decoded_message = caesar_decipher(msg.message, msg.key);
    RCLCPP_INFO(this->get_logger(), "Decoded message: '%s'", decoded_message.c_str());

    // Wait for the verification service to be available.
    if (!client_->wait_for_service(1s)) {
      RCLCPP_WARN(this->get_logger(), "Service 'verify_decoded' not available, skipping verification.");
      return;
    }

    // Create a service request with the decoded message.
    auto request = std::make_shared<cipher_interfaces::srv::CipherAnswer::Request>();
    request->answer = decoded_message;

    // Asynchronously send the request to the verification service.
    auto future_result = client_->async_send_request(
      request,
      [this, decoded_message](rclcpp::Client<cipher_interfaces::srv::CipherAnswer>::SharedFuture future)
      {
        auto response = future.get();
        if (response->result) {
          RCLCPP_INFO(this->get_logger(), "Verification: The decoded message '%s' is CORRECT.",
                      decoded_message.c_str());
        } else {
          RCLCPP_INFO(this->get_logger(), "Verification: The decoded message '%s' is INCORRECT.",
                      decoded_message.c_str());
        }
      });
  }
  rclcpp::Subscription<cipher_interfaces::msg::CipherMessage>::SharedPtr subscription_;
  rclcpp::Client<cipher_interfaces::srv::CipherAnswer>::SharedPtr client_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CipherSubscriber>());
  rclcpp::shutdown();
  return 0;
}
