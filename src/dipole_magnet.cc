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

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/advertise_options.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <iostream>
#include <vector>
#include <cstdint>

#include "storm_gazebo_ros_magnet/dipole_magnet.h"

using namespace gazebo;
using namespace ignition;

DipoleMagnet::DipoleMagnet(): ModelPlugin() {
  this->connect_count = 0;
}

DipoleMagnet::~DipoleMagnet() {
  this->update_connection.reset();
  if (this->mag->controllable) {
    this->queue.clear();
    this->queue.disable();
    this->rosnode->shutdown();
    this->callback_queue_thread.join();
    delete this->rosnode;
  }
  if (this->mag){
    DipoleMagnetContainer::Get().Remove(this->mag);
  }
}

void DipoleMagnet::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
  // Store the pointer to the model
  this->model = _parent;
  this->world = _parent->GetWorld();
  gzdbg << "Loading DipoleMagnet plugin" << std::endl;

  this->mag = std::make_shared<DipoleMagnetContainer::Magnet>();

  // load parameters
  this->robot_namespace = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robot_namespace = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  if(!_sdf->HasElement("bodyName")) {
    gzerr << "DipoleMagnet plugin missing <bodyName>, cannot proceed" << std::endl;
    return;
  }else {
    this->link_name = _sdf->GetElement("bodyName")->Get<std::string>();
  }

  this->link = this->model->GetLink(this->link_name);
  if(!this->link){
    gzerr << "Error: link named " << this->link_name << " does not exist" << std::endl;
    return;
  }

  this->inertial = this->link->GetInertial();
  if(!this->inertial){
    gzerr << "Error: inertial of the link named " << this->link_name << " does not loaded" << std::endl;
    return;
  }
  else {
    this->mag->mass = this->inertial->Mass();
  }

  this->mag->controllable = false;
  if (_sdf->HasElement("controllable"))
  {
    this->mag->controllable = _sdf->GetElement("controllable")->Get<bool>();
  }

  if (_sdf->HasElement("calculate")){
    this->mag->calculate = _sdf->Get<bool>("calculate");
  } else
    this->mag->calculate = true;

  if (_sdf->HasElement("xyzOffset")){
    this->mag->offset.Pos() = _sdf->Get<math::Vector3d>("xyzOffset");
  }

  if (_sdf->HasElement("xyzRange")){
    this->mag->range.Pos() = _sdf->Get<math::Vector3d>("xyzRange");
  }

  if (_sdf->HasElement("xyzVelLimit")){
    this->mag->vel_limit.Pos() = _sdf->Get<math::Vector3d>("xyzVelLimit");
  }

  if (this->mag->controllable) {
    if (!_sdf->HasElement("topicNs"))
    {
      gzmsg << "DipoleMagnet plugin missing <topicNs>," 
          "will publish on namespace " << this->link_name << std::endl;
    }
    else {
      this->topic_ns = _sdf->GetElement("topicNs")->Get<std::string>();
    }

    if (!ros::isInitialized())
    {
      gzerr << "A ROS node for Gazebo has not been initialized, unable to load "
        "plugin. Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in "
        "the gazebo_ros package. If you want to use this plugin without ROS, "
        "set <shouldPublish> to false" << std::endl;
      return;
    }

    this->rosnode = new ros::NodeHandle(this->robot_namespace);
    this->rosnode->setCallbackQueue(&this->queue);

    this->magnet_sub = this->rosnode->subscribe(
        this->topic_ns + "/cmd", 1, &DipoleMagnet::Magnet_CB, this);

    // Custom Callback Queue
    this->callback_queue_thread = boost::thread( boost::bind( &DipoleMagnet::QueueThread,this ) );
  }

  this->debug = false;
  if (_sdf->HasElement("debug")) {
    this->debug = _sdf->GetElement("debug")->Get<bool>();
  }

  this->mag->model_id = this->model->GetId();

  gzmsg << "Loaded Gazebo dipole magnet plugin on " << this->model->GetName() << std::endl;

  DipoleMagnetContainer::Get().Add(this->mag);

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&DipoleMagnet::OnUpdate, this, _1));
}

void DipoleMagnet::Connect() {
  this->connect_count++;
}

void DipoleMagnet::Disconnect() {
  this->connect_count--;
}

void DipoleMagnet::QueueThread() {
  static const double timeout = 0.01;

  while (this->rosnode->ok())
  {
    this->queue.callAvailable(ros::WallDuration(timeout));
  }
}

// Called by the world update start event
void DipoleMagnet::OnUpdate(const common::UpdateInfo & /*_info*/) {

  if (!this->mag->calculate)
    return;

  // Calculate the force from all other magnets
  math::Pose3d p_self = this->link->WorldPose();
  this->mag->pose = p_self;
  double mass_self = this->mag->mass;

  DipoleMagnetContainer& dp = DipoleMagnetContainer::Get();

  for(DipoleMagnetContainer::MagnetPtrV::iterator it = dp.magnets.begin(); it < dp.magnets.end(); it++){
    std::shared_ptr<DipoleMagnetContainer::Magnet> mag_other = *it;
    if (mag_other->model_id != this->mag->model_id && !this->mag->controllable && mag_other->controllable) {
      math::Pose3d p_other = mag_other->pose;
      math::Pose3d offset_other = mag_other->offset;
      math::Pose3d range_other = mag_other->range;
      double mass_other = mag_other->mass;

      math::Vector3d v_self = this->link->WorldLinearVel();
      math::Vector3d v_cmd(0, 0, 0);
      math::Vector3d force(0, 0, 0);
      math::Vector3d w_self = this->link->WorldAngularVel();
      math::Vector3d w_cmd(0, 0, 0);
      math::Vector3d torque(0, 0, 0);

      math::Vector3d p_err = p_self.Pos() + p_other.Rot().RotateVector(offset_other.Pos()) - p_other.Pos();
      math::Vector3d p_err_inv = p_other.Rot().Inverse().RotateVector(p_err);
      math::Vector3d e_err = (p_self.Rot() * p_other.Rot().Inverse()).Euler();
      
      // velocity, torque are only working on the magnet with no controllable
      if (mag_other->magnet_cmd
          && std::abs(p_err_inv.X()) < range_other.Pos().X()
          && std::abs(p_err_inv.Y()) < range_other.Pos().Y()
          && std::abs(p_err_inv.Z()) < range_other.Pos().Z()) {
        v_cmd = 10 * -p_err;
        this->LimitVelocity(v_cmd, mag_other->vel_limit.Pos());
        force = 1000 * (v_cmd - v_self);
        force.Z() += mass_self * 9.8;

        w_cmd = 10 * -e_err;
        torque = 0.00001 * (w_cmd - w_self);
      }
      else {
        force = 0;
        torque = 0;
      }
      this->link->AddForce(force);
      this->link->AddTorque(torque);

      if (this->debug) {
        system("clear");
        std::cout << std::setprecision(3);
        std::cout << "[magnet_msg] \t" << (this->mag->magnet_cmd ? "true" : "false") << std::endl;
        std::cout << "[mass_self] \t" << mass_self << std::endl;
        std::cout << "[mass_other] \t" << mass_other << std::endl;
        std::cout << "[position] \t" << p_self << std::endl;
        std::cout << "[vel_cmd] \t" << v_cmd << std::endl;
        std::cout << "[velocity] \t" << v_self << std::endl;
        std::cout << "[euler_self] \t" << p_self.Rot().Euler() << std::endl;
        std::cout << "[euler_other] \t" << p_other.Rot().Euler() << std::endl;
        std::cout << "[w_cmd] \t" << w_cmd << std::endl;
        std::cout << "[Ang. Vel.] \t" << w_self << std::endl;
        std::cout << "[force] \t" << force << std::endl;
        std::cout << "[torque] \t" << torque << std::endl
                  << std::endl;
      }
    }
  }
}

void DipoleMagnet::LimitVelocity(math::Vector3d& v_cmd,
                                 math::Vector3d& v_limit) {
  if (v_cmd.X() > v_limit.X())
    v_cmd.X() = v_limit.X();
  if (v_cmd.X() < -v_limit.X())
    v_cmd.X() = -v_limit.X();

  if (v_cmd.Y() > v_limit.Y())
    v_cmd.Y() = v_limit.Y();
  if (v_cmd.Y() < -v_limit.Y())
    v_cmd.Y() = -v_limit.Y();

  if (v_cmd.Z() > v_limit.Z())
    v_cmd.Z() = v_limit.Z();
  if (v_cmd.Z() < -v_limit.Z())
    v_cmd.Z() = -v_limit.Z();
}

void DipoleMagnet::Magnet_CB(const std_msgs::Bool& msg) {
  this->mag->magnet_cmd = msg.data;
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(DipoleMagnet)
