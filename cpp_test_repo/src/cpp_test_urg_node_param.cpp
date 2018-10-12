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
#include <iomanip>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    uint64_t nanosec = (uint64_t)msg->header.stamp.sec * 1e9 + (uint64_t)msg->header.stamp.nanosec;
    rclcpp::Time now(std::chrono::duration_cast< std::chrono::nanoseconds >(std::chrono::system_clock::now().time_since_epoch()).count());
    rclcpp::Time test(nanosec);
    rclcpp::Duration diff = test - now;

    RCLCPP_INFO(this->get_logger(), "I heard: [%d] secs + [%lu] nanosecs = [%llu] diff to real time : [%lld] - [%lld] = [%lld]", msg->header.stamp.sec, msg->header.stamp.nanosec, nanosec, now.nanoseconds() ,test.nanoseconds(), diff.nanoseconds());
    const auto msg_time = std::chrono::nanoseconds(nanosec);
    const std::chrono::time_point<std::chrono::system_clock> msg_time_point(msg_time);
    std::time_t msg_time_t = std::chrono::system_clock::to_time_t(msg_time_point);
    RCLCPP_INFO(this->get_logger(), "[%s]", std::ctime(&msg_time_t));
  }
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
