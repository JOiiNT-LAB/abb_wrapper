/***********************************************************************************************************************
Copyright (c) 2021, JOiiNT LAB, Fondazione Istituto Italiano di Tecnologia, Intellimech Consorzio per la Meccatronica.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***********************************************************************************************************************
*
***********************************************************************************************************************
* 
* Authors: Gianluca Lentini, Ugo Alberto Simioni
* Date:18/01/2022
* Version 1.0
***********************************************************************************************************************
*/

#include <ros/ros.h>
#include <ros/package.h>
#include <rosbag/view.h>
#include <sensor_msgs/JointState.h>

#include <eigen3/Eigen/Eigen>


Eigen::VectorXd joints;

void callback_joints(const sensor_msgs::JointState::ConstPtr& msg)
{

  for(int i = 0; i < 7; i++)
  {
    joints(i) = msg->position[i];
  }

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "single_yumi_joints_rviz_publisher_node");
  ros::NodeHandle n;
  ros::Rate rate(100);

  joints = Eigen::VectorXd::Zero(7);

  ros::Subscriber sub_joints = n.subscribe("joint_states", 1, callback_joints);
  ros::Publisher  pub_joint_states = n.advertise<sensor_msgs::JointState>("/joint_states", 1);

  sensor_msgs::JointState joint_state;


 
  while(ros::ok())
    {

      joint_state.name.resize(9);
      joint_state.position.resize(9);
      
      joint_state.name[0] ="single_yumi_joint_1";
      joint_state.name[1] ="single_yumi_joint_2";
      joint_state.name[2] ="single_yumi_joint_3";
      joint_state.name[3] ="single_yumi_joint_4";
      joint_state.name[4] ="single_yumi_joint_5";
      joint_state.name[5] ="single_yumi_joint_6";
      joint_state.name[6] ="single_yumi_joint_7";

      joint_state.name[7] ="gripper_joint";
      joint_state.name[8] ="gripper_joint_m";

      //update joint_state
      joint_state.header.stamp = ros::Time::now();
      joint_state.position[0] = joints(0);
      joint_state.position[1] = joints(1);
      joint_state.position[2] = joints(2);
      joint_state.position[3] = joints(3);
      joint_state.position[4] = joints(4);
      joint_state.position[5] = joints(5);
      joint_state.position[6] = joints(6);


      joint_state.position[7] = 0.0;
      joint_state.position[8] = 0.0;


      pub_joint_states.publish(joint_state);

      ros::spinOnce();
      rate.sleep();
    }

  return 0;
}