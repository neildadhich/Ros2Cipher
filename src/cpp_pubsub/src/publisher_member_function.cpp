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
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <cstdlib>
#include <cctype>

#include "rclcpp/rclcpp.hpp"
#include "cipher_interfaces/msg/cipher_message.hpp"
#include "cipher_interfaces/srv/cipher_answer.hpp" 

using namespace std::chrono_literals;

// Simple Caesar cipher function
std::string caesar_cipher(const std::string &text, int key)
{
  std::string result = text;
  for (char &c : result) {
    if (std::isalpha(c)) {
      // Determine base depending on lowercase or uppercase
      char base = std::islower(c) ? 'a' : 'A';
      // Rotate the character by key positions (wrap-around using modulo)
      c = base + (c - base + key) % 26;
    }
  }
  return result;
}

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class CipherPublisher : public rclcpp::Node
{
public:
  CipherPublisher(const std::string &input_message, int key)
  : Node("cipher_publisher"), original_message_(input_message)
  {
    publisher_ = this->create_publisher<cipher_interfaces::msg::CipherMessage>("cipher_topic", 10);
    std::string encoded = caesar_cipher(input_message, key);
    
    auto message = cipher_interfaces::msg::CipherMessage();
    message.header.stamp = this->get_clock()->now();
    message.message = encoded;
    message.key = static_cast<int8_t>(key); 
    publisher_->publish(message);

    service_ = this->create_service<cipher_interfaces::srv::CipherAnswer>(
      "verify_decoded",
      std::bind(&CipherPublisher::verify_answer_callback, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "Service 'verify_decoded' is ready.");
  }

private:
  void verify_answer_callback(
    const std::shared_ptr<cipher_interfaces::srv::CipherAnswer::Request> request,
    std::shared_ptr<cipher_interfaces::srv::CipherAnswer::Response> response)
  {
    // Check if the answer provided in the request matches the original message.
    if (request->answer == original_message_) {
      response->result = true;
      RCLCPP_INFO(this->get_logger(), "Verification successful: answer is correct.");
    } else {
      response->result = false;
      RCLCPP_INFO(this->get_logger(), "Verification failed: received answer '%s' (expected '%s')",
                  request->answer.c_str(), original_message_.c_str());
    }
  }
  rclcpp::Publisher<cipher_interfaces::msg::CipherMessage>::SharedPtr publisher_;
  rclcpp::Service<cipher_interfaces::srv::CipherAnswer>::SharedPtr service_;
  std::string original_message_;
};

int main(int argc, char * argv[])
{
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " <message> <key>" << std::endl;
    return 1;
  }
  std::string input_message = argv[1];
  int key = std::atoi(argv[2]);

  rclcpp::init(argc, argv);
  auto node = std::make_shared<CipherPublisher>(input_message, key);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
