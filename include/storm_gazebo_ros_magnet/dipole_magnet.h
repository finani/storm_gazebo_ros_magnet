/*
 * Copyright (c) 2016, Vanderbilt University
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
 * Author: Addisu Z. Taddese
 */

#ifndef INCLUDE_MAC_GAZEBO_DIPOLE_MAGNET_DIPOLE_MAGNET_H_
#define INCLUDE_MAC_GAZEBO_DIPOLE_MAGNET_DIPOLE_MAGNET_H_


#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/MagneticField.h>
#include <std_msgs/Bool.h>

#include <memory>

#include "storm_gazebo_ros_magnet/dipole_magnet_container.h"

using namespace gazebo;
using namespace ignition;

class DipoleMagnet : public ModelPlugin {
 public:
  DipoleMagnet();

  ~DipoleMagnet();

  /// \brief Loads the plugin
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

  /// \brief Callback for when subscribers connect
  void Connect();

  /// \brief Callback for when subscribers disconnect
  void Disconnect();

  /// \brief Thread to interact with ROS
  void QueueThread();

  /// \brief Called by the world update start event
  void OnUpdate(const common::UpdateInfo & /*_info*/);



  void LimitVelocity(math::Vector3d& v_cmd,
                     math::Vector3d& v_limit);

  void Magnet_CB(const std_msgs::Bool& msg);

  // Pointer to the model
 private:
  physics::ModelPtr model;
  physics::LinkPtr link;
  physics::InertialPtr inertial;
  physics::WorldPtr world;

  std::shared_ptr<DipoleMagnetContainer::Magnet> mag;

  std::string link_name;
  std::string robot_namespace;
  std::string topic_ns;

  bool debug;
  double mass;
  double Ixx;
  double Iyy;
  double Izz;
  ros::NodeHandle* rosnode;
  ros::Subscriber magnet_sub;

  geometry_msgs::WrenchStamped wrench_msg;

  private: boost::mutex lock;
  int connect_count;

  // Custom Callback Queue
  ros::CallbackQueue queue;
  boost::thread callback_queue_thread;

  // Pointer to the update event connection
  event::ConnectionPtr update_connection;
};

#endif // INCLUDE_MAC_GAZEBO_DIPOLE_MAGNET_DIPOLE_MAGNET_H_