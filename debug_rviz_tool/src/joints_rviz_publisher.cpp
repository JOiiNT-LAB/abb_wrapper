/***********************************************************************************************************************
BSD 3-Clause License
Author: Gianluca Lentini, Ugo Alberto Simioni
Copyright (c) 2021, JOiiNT LAB, Istituto italiano di Tecnologia, Intellimech.
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
 */
#include <ros/ros.h>
#include <ros/package.h>
#include <rosbag/view.h>
#include <sensor_msgs/JointState.h>

#include <eigen3/Eigen/Eigen>


Eigen::VectorXd joints_R;
Eigen::VectorXd joints_L;

void callback_joints_R(const sensor_msgs::JointState::ConstPtr& msg)
{

  for(int i = 0; i < 7; i++)
  {
    joints_R(i) = msg->position[i];
  }

}

void callback_joints_L(const sensor_msgs::JointState::ConstPtr& msg)
{
  for(int i = 0; i < 7; i++)
  {
    joints_L(i) = msg->position[i];
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "joints_rviz_publisher_node");
  ros::NodeHandle n;
  ros::Rate rate(100);

  joints_R = Eigen::VectorXd::Zero(7);
  joints_L = Eigen::VectorXd::Zero(7);

  ros::Subscriber sub_joints_R = n.subscribe("/Yumi_RIGHT/joint_states", 1, callback_joints_R);
  ros::Subscriber sub_joints_L = n.subscribe("/Yumi_LEFT/joint_states", 1, callback_joints_L);
  ros::Publisher  pub_joint_states = n.advertise<sensor_msgs::JointState>("/joint_states", 1);

  sensor_msgs::JointState joint_state;


 
  while(ros::ok())
    {

      joint_state.name.resize(18);
      joint_state.position.resize(18);
      
      joint_state.name[0] ="yumi_joint_1_r";
      joint_state.name[1] ="yumi_joint_2_r";
      joint_state.name[2] ="yumi_joint_3_r";
      joint_state.name[3] ="yumi_joint_4_r";
      joint_state.name[4] ="yumi_joint_5_r";
      joint_state.name[5] ="yumi_joint_6_r";
      joint_state.name[6] ="yumi_joint_7_r";

      joint_state.name[7] ="yumi_joint_1_l";
      joint_state.name[8] ="yumi_joint_2_l";
      joint_state.name[9] ="yumi_joint_3_l";
      joint_state.name[10] ="yumi_joint_4_l";
      joint_state.name[11] ="yumi_joint_5_l";
      joint_state.name[12] ="yumi_joint_6_l";
      joint_state.name[13] ="yumi_joint_7_l";



      joint_state.name[14] ="gripper_r_joint";
      joint_state.name[15] ="gripper_r_joint_m";

      joint_state.name[16] ="gripper_l_joint";
      joint_state.name[17] ="gripper_l_joint_m";
      //update joint_state
      joint_state.header.stamp = ros::Time::now();
      joint_state.position[0] = joints_R(0);
      joint_state.position[1] = joints_R(1);
      joint_state.position[2] = joints_R(2);
      joint_state.position[3] = joints_R(3);
      joint_state.position[4] = joints_R(4);
      joint_state.position[5] = joints_R(5);
      joint_state.position[6] = joints_R(6);

      joint_state.position[7] = joints_L(0);
      joint_state.position[8] = joints_L(1);
      joint_state.position[9] = joints_L(2);
      joint_state.position[10] = joints_L(3);
      joint_state.position[11] = joints_L(4);
      joint_state.position[12] = joints_L(5);
      joint_state.position[13] = joints_L(6);


      joint_state.position[14] = 0.0;
      joint_state.position[15] = 0.0;
      joint_state.position[16] = 0.0;
      joint_state.position[17] = 0.0;

      pub_joint_states.publish(joint_state);

      ros::spinOnce();
      rate.sleep();
    }

  return 0;
}