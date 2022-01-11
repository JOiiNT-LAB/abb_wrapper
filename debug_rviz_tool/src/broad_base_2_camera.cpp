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



#include <eigen3/Eigen/Eigen>
#include <tf/transform_broadcaster.h>



int main(int argc, char **argv)
{
	ros::init(argc, argv, "broad_base_2_camera_node");
	ros::NodeHandle n;
	ros::Rate rate(100);

	std::vector<double> translation, rotation;
	n.getParam("/translation", translation);
	n.getParam("/rotation", rotation);
  	tf::TransformBroadcaster br_base_2_des;


 
	while(ros::ok())
	{

		tf::Transform transform;
		transform.setOrigin( tf::Vector3(translation[0], translation[1], translation[2]));
		tf::Quaternion q(rotation[0], rotation[1], rotation[2], rotation[3]);
		transform.setRotation(q);
		std::string tf_name_des, tf_base_robot;
		tf_name_des =  "/yumi_base_link";
		tf_base_robot = "/camera_color_optical_frame";
		br_base_2_des.sendTransform(tf::StampedTransform(transform, ros::Time::now(), tf_base_robot, tf_name_des));


		ros::spinOnce();
		rate.sleep();
	}

  return 0;
}