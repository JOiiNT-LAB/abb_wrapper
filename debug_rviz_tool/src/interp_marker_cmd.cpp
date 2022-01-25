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
#include <ros/rate.h>
#include <iostream>
#include <eigen3/Eigen/Eigen>

#include <geometry_msgs/Pose.h>
#include <std_srvs/Empty.h>


Eigen::Vector3d pos_cmd_, pos_curr_yumi_;
Eigen::Quaterniond quat_cmd_, quat_curr_yumi_;
int state = 0;

//Linear position interpolation
Eigen::Vector3d linear_inter(Eigen::Vector3d pos_in, Eigen::Vector3d pos_fin, double time, double dur)
{
  return pos_in + (((pos_fin - pos_in) / dur) * time); 
}

void callback_marker_pose(const geometry_msgs::Pose::ConstPtr& msg)
{
  pos_cmd_ << msg->position.x, msg->position.y, msg->position.z; 
  quat_cmd_.w() = msg->orientation.w;
  quat_cmd_.vec() << msg->orientation.x, msg->orientation.y, msg->orientation.z;
}

void callback_curr_pose(const geometry_msgs::Pose::ConstPtr& msg)
{
  pos_curr_yumi_ << msg->position.x, msg->position.y, msg->position.z; 
  quat_curr_yumi_.w() = msg->orientation.w;
  quat_curr_yumi_.vec() << msg->orientation.x, msg->orientation.y, msg->orientation.z;

}



bool callback_go(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
{
  state = 1;
  return true;
}


//-----------------------------------------------------
//                                                 main
//-----------------------------------------------------
int main(int argc, char **argv)
{

  ros::init(argc, argv, "interp_marker_cmd_node");
  ros::NodeHandle n;

  double rate_100Hz = 100.0;
  ros::Rate r_100HZ(rate_100Hz);
  double dt = 1 / rate_100Hz;
  
  int step_inter_home = 0;
  // double time_home = 0.5;
  double time_home = 3.0;
  bool flag_first_step = true;
  Eigen::Vector3d pos_init_L;
  Eigen::Quaterniond quat_init_L;

  ros::Publisher pub_pos_des = n.advertise<geometry_msgs::Pose>("/Gofa/pose_des", 1);

  ros::Subscriber sub_marker_pose = n.subscribe("/cube_pose", 1, callback_marker_pose);
  ros::Subscriber sub_curr_pose = n.subscribe("/Gofa/curr_pose", 1, callback_curr_pose);

  ros::ServiceServer service_go = n.advertiseService("go_makert", callback_go);


  while(ros::ok())
  {
    // std::cout<<"sono qui"<<std::endl;
    switch(state)
    {
      case 0:
      {
        // if(getchar() == ' ')
        // {
        //   std::cout<<"spacebar pressed"<<std::endl;
        //   state = 1;
        // }
        break;
      }

      case 1:
      {
        std::cout <<"position: "<< pos_cmd_.x()<< " "<< pos_cmd_.y()<< " "<< pos_cmd_.z()<<std::endl;
        // state = 0;
        if(step_inter_home <= (time_home/dt))
        {
          if(flag_first_step)
          {
            pos_init_L << pos_curr_yumi_;
            quat_init_L = quat_curr_yumi_;
            flag_first_step = false;
          }

          double time = step_inter_home * dt;
          Eigen::Vector3d pos_inter_L = linear_inter(pos_init_L, pos_cmd_, time, time_home);
          Eigen::Quaterniond quat_inter_L = quat_init_L.slerp((time/time_home), quat_cmd_);
          step_inter_home ++;

          geometry_msgs::Pose send_pose;
          send_pose.position.x = pos_inter_L(0);
          send_pose.position.y = pos_inter_L(1);
          send_pose.position.z = pos_inter_L(2);
          send_pose.orientation.w = quat_inter_L.w();
          send_pose.orientation.x = quat_inter_L.x();
          send_pose.orientation.y = quat_inter_L.y();
          send_pose.orientation.z = quat_inter_L.z();

          pub_pos_des.publish(send_pose);
        }
        else 
          {
            state = 0;
            step_inter_home = 0;
            flag_first_step = true;
          }

        break;
      }
    }



    ros::spinOnce();
    r_100HZ.sleep();
        
  }// end while()
return 0;
}