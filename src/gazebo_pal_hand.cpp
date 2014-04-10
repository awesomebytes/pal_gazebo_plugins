///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2014, PAL Robotics S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
// * Neither the name of PAL Robotics S.L. nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/*
 *  gazebo_pal_hand.cpp
 *  Copyright (c) 2014 PAL Robotics sl. All Rights Reserved
 *  Created on: 7 Jan 2014
 *      Author: luca
 *
 * \brief A plugin for gazebo for controlling the pal underactuated hand in simulation
 * \author  Luca Marchionni (luca.marchionni@pal-robotics.com)
 */

#include <algorithm>
#include <assert.h>

#include <pal_gazebo_plugins/gazebo_pal_hand.h>

#include <gazebo/math/gzmath.hh>
#include <sdf/sdf.hh>

#include "pal_gazebo_plugins/PalHandPlugin.h"
#include <ros/ros.h>

namespace gazebo {

  GazeboPalHand::GazeboPalHand() {

  }

  // Destructor
  GazeboPalHand::~GazeboPalHand() {
  }

  // Load the controller
  void GazeboPalHand::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {

    this->parent = _parent;
    this->world = _parent->GetWorld();

    this->robot_namespace_ = "";
    if (!_sdf->HasElement("robotNamespace")) {
      ROS_INFO("GazeboPalHand Plugin missing <robotNamespace>, defaults to \"%s\"",
          this->robot_namespace_.c_str());
    } else {
      this->robot_namespace_ =
        _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
    }

    this->finger_joint_name_ = "actuated_finger_joint";
    if (!_sdf->HasElement("actuatedJoint")) {
      ROS_WARN("GazeboPalHand Plugin (ns = %s) missing <actuatedJoint>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->finger_joint_name_.c_str());
    } else {
      this->finger_joint_name_ = _sdf->GetElement("actuatedJoint")->Get<std::string>();
    }

    this->finger_1_joint_name_ = "finger_joint_1";
    if (!_sdf->HasElement("fingerJoint1")) {
      ROS_WARN("GazeboPalHand Plugin (ns = %s) missing <fingerJoint1>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->finger_1_joint_name_.c_str());
    } else {
      this->finger_1_joint_name_ = _sdf->GetElement("fingerJoint1")->Get<std::string>();
    }

    this->finger_2_joint_name_ = "finger_joint_2";
    if (!_sdf->HasElement("fingerJoint1")) {
      ROS_WARN("GazeboPalHand Plugin (ns = %s) missing <fingerJoint2>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->finger_2_joint_name_.c_str());
    } else {
      this->finger_2_joint_name_ = _sdf->GetElement("fingerJoint2")->Get<std::string>();
    }

    this->finger_3_joint_name_ = "finger_joint_3";
    if (!_sdf->HasElement("fingerJoint3")) {
      ROS_WARN("GazeboPalHand Plugin (ns = %s) missing <fingerJoint3>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->finger_3_joint_name_.c_str());
    } else {
      this->finger_3_joint_name_ = _sdf->GetElement("fingerJoint3")->Get<std::string>();
    }

    physics::Joint_V all_joints = parent->GetJoints();
    for(unsigned int i=0; i < all_joints.size(); ++i)
    {
      ROS_INFO("Joint name %s", all_joints[i]->GetName().c_str());
    }
    joints[0] = this->parent->GetJoint(finger_joint_name_);
    joints[1] = this->parent->GetJoint(finger_1_joint_name_);
    joints[2] = this->parent->GetJoint(finger_2_joint_name_);
    joints[3] = this->parent->GetJoint(finger_3_joint_name_);

    if (!joints[0]) {
      char error[200];
      snprintf(error, 200,
          "GazeboPalHand Plugin (ns = %s) couldn't get actuated finger hinge joint named \"%s\"",
          this->robot_namespace_.c_str(), this->finger_joint_name_.c_str());
      gzthrow(error);
    }
    if (!joints[1]) {
      char error[200];
      snprintf(error, 200,
          "GazeboPalHand Plugin (ns = %s) couldn't get  finger 1 hinge joint named \"%s\"",
          this->robot_namespace_.c_str(), this->finger_1_joint_name_.c_str());
      gzthrow(error);
    }

    if (!joints[2]) {
      char error[200];
      snprintf(error, 200,
          "GazeboPalHand Plugin (ns = %s) couldn't get  finger 2 hinge joint named \"%s\"",
          this->robot_namespace_.c_str(), this->finger_2_joint_name_.c_str());
      gzthrow(error);
    }

    if (!joints[3]) {
      char error[200];
      snprintf(error, 200,
          "GazeboPalHand Plugin (ns = %s) couldn't get  finger 3 hinge joint named \"%s\"",
          this->robot_namespace_.c_str(), this->finger_3_joint_name_.c_str());
      gzthrow(error);
    }

    //TODO: Load upper_joint_limit and lower_joint_limit from the upper and lower atribute of the limit element of the joint in the sdf
    this->lower_joint_limit = 0.02;
    this->upper_joint_limit = 4.5;
    closing_angle.SetFromRadian(this->upper_joint_limit);


//    this->old_forces[0] = 0.0;
//    this->old_forces[1] = 0.0;
//    this->old_forces[2] = 0.0;
//    this->old_forces[3] = 0.0;

    // ros callback queue for processing subscription
    this->deferredLoadThread_ = boost::thread(
      boost::bind(&GazeboPalHand::DeferredLoad, this));

  }

  ////////////////////////////////////////////////////////////////////////////////
  void GazeboPalHand::DeferredLoad()
  {
    // initialize ros
    if (!ros::isInitialized())
    {
      gzerr << "Not loading plugin since ROS hasn't been "
            << "properly initialized.  Try starting gazebo with ros plugin:\n"
            << "  gazebo -s libgazebo_ros_api_plugin.so\n";
      return;
    }

    // ros stuff
    this->rosNode_ = new ros::NodeHandle("");

    this->publisher_ = this->rosNode_->advertise<pal_gazebo_plugins::PalHandPlugin>("/" + this->robot_namespace_+ "/pal_hand/" + this->finger_joint_name_, 10);

    // ros callback queue for processing subscription
    this->callbackQueeuThread_ = boost::thread(
      boost::bind(&GazeboPalHand::RosQueueThread, this));

    // listen to the update event (broadcast every simulation iteration)
    this->update_connection_ =
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboPalHand::UpdateChild, this));
}

  void GazeboPalHand::RosQueueThread()
  {
  //  static const double timeout = 0.01;
    ros::Rate rate(1000);

    while (this->rosNode_->ok())
    {
      this->rosQueue_.callAvailable(/*ros::WallDuration(timeout)*/);
      rate.sleep();
    }
  }

  // Update the controller
  void GazeboPalHand::UpdateChild() {

    pal_gazebo_plugins::PalHandPlugin message;
    for(unsigned int i=0; i<4; ++i)
    {
      message.positions[i] =  joints[i]->GetAngle(0u).Radian();
      message.forces[i]    = joints[i]->GetForce(0u);
      message.forces_dt[i] = (float)(message.forces[i] - this->old_forces[i])/(0.001d);
//      if (i == 0 && this->finger_joint_name_ ==   "hand_right_index_joint"){
//        gzwarn << this->finger_joint_name_ << ":\n"
//                 << " (message.forces[i] - old_forces[i])/(0.001d)\n"
//                 << " (" << message.forces[i] << " - " <<  this->old_forces[i] << ")/(0.001d)\n"
//                 << " (" << message.forces[i] - this->old_forces[i] << ")/(0.001d)\n"
//                 << " " << (message.forces[i] - this->old_forces[i])/(0.001d) << "\n"
//                 << " " << (float)(message.forces[i] - this->old_forces[i])/(0.001d) << "\n";

//        gzwarn << "previous value of old_forces[0]: " <<  this->old_forces[0] << "\n"
//               << "now we set the value: " << message.forces[0] << "\n";
//        }
      this->old_forces[i] =  message.forces[i];
      for(unsigned int j=0; j<3; ++j)
      {
        message.joint_body1forces[i].forces.force[j]   =  joints[i]->GetForceTorque(0u).body1Force[j];
        message.joint_body1forces[i].torques.torque[j] =  joints[i]->GetForceTorque(0u).body1Torque[j];

        message.joint_body2forces[i].forces.force[j]   =  joints[i]->GetForceTorque(0u).body2Force[j];
        message.joint_body2forces[i].torques.torque[j] =  joints[i]->GetForceTorque(0u).body2Torque[j];

        message.joint_link_force_torques[i].forces.force[j] = joints[i]->GetLinkForce(0u)[j];
        message.joint_link_force_torques[i].torques.torque[j] = joints[i]->GetLinkTorque(0u)[j];

        message.child_link_world_force_torques[i].forces.force[j] = joints[i]->GetChild()->GetWorldForce()[j];
        message.child_link_world_force_torques[i].torques.torque[j] = joints[i]->GetChild()->GetWorldTorque()[j];
      }
    }



    publisher_.publish(message);
    math::Angle actuator_angle = joints[0]->GetAngle(0u);

    /// TODO: expose a parametr in SDF for max joint force
    double max_joint_force = 10.0; // empirically found, when the phallanges start to fly and jump out of position a value over 10 triggers
    /* If we find out this value while closing, i.e. grasping an object, set the current angle as the maximum one */
    if(fabs(message.forces_dt[0]) > max_joint_force )
    {
        closing_angle = actuator_angle;
        closing_angle = (closing_angle > this->lower_joint_limit) ? closing_angle : this->lower_joint_limit;
    }
    /* If the angle requested is lower than the angle where we blocked the finger, set up the max angle back to normal */
    if(actuator_angle < closing_angle)
        closing_angle.SetFromRadian(this->upper_joint_limit);

    if(actuator_angle < closing_angle)
    {

        if( actuator_angle > this->lower_joint_limit)
        {
            math::Angle index_1_angle = ( actuator_angle/2.5 > joints[1]->GetUpperLimit(0u) ) ? joints[1]->GetUpperLimit(0u) : actuator_angle/2.5;
            joints[1]->SetAngle(0u, index_1_angle);

            math::Angle index_2_angle = ( actuator_angle/3.2 > joints[2]->GetUpperLimit(0u) ) ? joints[2]->GetUpperLimit(0u) : actuator_angle/3.2;
            joints[2]->SetAngle(0u, index_2_angle);

            math::Angle index_3_angle = ( actuator_angle/3.2 > joints[3]->GetUpperLimit(0u) ) ? joints[3]->GetUpperLimit(0u) : actuator_angle/3.2;
            joints[3]->SetAngle(0u, index_3_angle);
        }
        else
        {
            joints[1]->SetAngle(0u, this->lower_joint_limit);
            joints[2]->SetAngle(0u, this->lower_joint_limit);
            joints[3]->SetAngle(0u, this->lower_joint_limit);
        }
    }
  }

  GZ_REGISTER_MODEL_PLUGIN(GazeboPalHand)
}


