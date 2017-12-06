/*
 * Copyright 2015 Aldebaran
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef BASIC_PUBLISHER_HPP
#define BASIC_PUBLISHER_HPP

#include <string>

/*
* ROS includes
*/
#include <ros/ros.h>

/*
* ROS2 includes
*/
#include <rclcpp/rclcpp.hpp>

namespace naoqi
{
namespace publisher
{

template < class T >
class is_ros2_message
 {
private:
  using Yes = char[2];
  using  No = char[1];

  struct Fallback { int UniquePtr; };
  struct Derived : T, Fallback { };

  template < class U >
  static No& test ( decltype(U::UniquePtr)* );
  template < typename U >
  static Yes& test ( U* );
 
 public:
  static constexpr bool value = sizeof(test<Derived>(nullptr)) == sizeof(Yes);
  using value_type = typename std::integral_constant<bool, value>::type;
};

template<class T>
class BasicPublisher
{

public:
  BasicPublisher( const std::string& topic ):
    topic_( topic ),
    is_initialized_( false )
  {}

  virtual ~BasicPublisher() {}

  inline std::string topic() const
  {
    return topic_;
  }

  inline bool isInitialized() const
  {
    return is_initialized_;
  }

  virtual inline bool isSubscribed() const
  {
    if (is_initialized_ == false) return false;
    return is_ros2_ ? false : pub_.getNumSubscribers() > 0;
  }

  virtual void publish( const T& msg )
  {
    publishImpl(msg);
  }

  virtual void reset( ros::NodeHandle& nh )
  {
    resetROS1Impl(nh);
  }

  virtual void reset( std::shared_ptr<rclcpp::Node> node )
  {
    resetROS2Impl(node);
  }

protected:
  std::string topic_;

  bool is_initialized_;

  const bool is_ros2_ = is_ros2_message<T>::value;

  /** Publisher */
  ros::Publisher pub_;

  std::shared_ptr<rclcpp::Publisher<T>> publisher_;

  template<
    typename U = T
  >
  void publishImpl(const U& msg, typename std::enable_if<!is_ros2_message<U>::value>::type * = nullptr)
  {
    pub_.publish( msg );
  }

  template<
    typename U = T
  >
  void publishImpl(const U& msg, typename std::enable_if<is_ros2_message<U>::value>::type * = nullptr)
  {
    publisher_->publish(msg);
  }

  template<
    typename U = T
  >
  void resetROS1Impl(ros::NodeHandle& nh, typename std::enable_if<!is_ros2_message<U>::value>::type * = nullptr)
  {
    pub_ = nh.advertise<T>( this->topic_, 10 );
    is_initialized_ = true;
  }

  template<
    typename U = T
  >
  void resetROS1Impl(ros::NodeHandle& nh, typename std::enable_if<is_ros2_message<U>::value>::type * = nullptr)
  {
    //static_assert(false, "This should never be reachable");
  }

  template<
    typename U = T
  >
  void resetROS2Impl(std::shared_ptr<rclcpp::Node> node, typename std::enable_if<!is_ros2_message<U>::value>::type * = nullptr)
  {
    //static_assert(false, "This should never be reachable");
  }

  template<
    typename U = T
  >
  void resetROS2Impl(std::shared_ptr<rclcpp::Node> node, typename std::enable_if<is_ros2_message<U>::value>::type * = nullptr)
  {
    publisher_ = node->create_publisher<U>(this->topic_, 10);
    is_initialized_ = true;
  }

}; // class

} // publisher
} // naoqi

#endif
