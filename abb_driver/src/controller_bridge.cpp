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

#include <controller_bridge.h>

controller_bridge::~controller_bridge()
{
	// Perform a clean shutdown.
	io_service_.stop();
	thread_group_.join_all();

}

void controller_bridge::callback_joint_cmd(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	joint_array_cmd_.data.clear();
	joint_array_cmd_.data = msg->data;
}


void controller_bridge::callback_test_joint(const std_msgs::Float64::ConstPtr& msg)
{
	// test_joint_ = msg->data;
	joint_array_cmd_.data[6] = msg->data;

	std::cout << "callback_test_joint" << joint_array_cmd_.data[6] << std::endl;
}

bool controller_bridge::callback_bridge_com(abb_driver::bridge_com::Request  &req, abb_driver::bridge_com::Response &res)
{
	first_msg_ = true;
	if (req.start) state_ = req.controller;
	else state_ = 2; //stop-> solo lettura
	// usleep(1000000);
	return true;
}


void controller_bridge::run()
{
	// Wait for a new EGM message from the EGM client (with a timeout of 500 ms).
	if (egm_interface_.waitForMessage(500))
	{
		// Read the message received from the EGM client.
		input_.Clear();
		// if (first_msg_ == true) std::cout << "input_" << input_.feedback().robot().joints().position().values_size()<< std::endl;
		egm_interface_.read(&input_);

		joint_curr_.header.stamp = ros::Time::now();

		int j = 0;
		for (int i = 0; i < kdl_chain_.getNrOfJoints(); i++)
		{
			if ((kdl_chain_.getNrOfJoints() == 7) && (i == 2))
			{
				q_msr_(i) = joint_curr_.position[i] = input_.feedback().external().joints().position().values(0) * 3.14159265358979323846 / 180.0;
			}
			else
			{
				q_msr_(i) = joint_curr_.position[i] = input_.feedback().robot().joints().position().values(j) * 3.14159265358979323846 / 180.0;
				j++;
			}
		}

		// computing forward kinematics
		fk_pos_solver_->JntToCart(q_msr_, EE_pose_);

		Eigen::Vector3d pos_curr;
		Eigen::Matrix3d orient_curr;
		Eigen::Quaterniond quat_curr;

		// Position and rot matrix from kdl to Eigen
		for (int i = 0; i < 3; i++)
		{
			pos_curr(i) = EE_pose_.p(i);

			for (int j = 0; j < 3; j++)
			{
				orient_curr(i, j) = EE_pose_.M(i, j);
			}
		}
		quat_curr = orient_curr;
		quat_curr.normalize();

		if (first_quat_)
		{
			first_quat_ = false;
			quat_old_ = quat_curr;
		}

		// rotation to quaternion issue , "Sign Flip" , check  http://www.dtic.mil/dtic/tr/fulltext/u2/1043624.pdf
		double sign_check = quat_curr.w() * quat_old_.w() + quat_curr.x() * quat_old_.x() + quat_curr.y() * quat_old_.y() + quat_curr.z() * quat_old_.z();
		if (sign_check < 0.0)
		{
			quat_curr.w() = quat_curr.w() * (-1);
			quat_curr.vec() = quat_curr.vec() * (-1);
		}

		quat_old_ = quat_curr;

		geometry_msgs::Pose curr_pose;
		curr_pose.position.x = pos_curr.x();
		curr_pose.position.y = pos_curr.y();
		curr_pose.position.z = pos_curr.z();
		curr_pose.orientation.w = quat_curr.w();
		curr_pose.orientation.x = quat_curr.x();
		curr_pose.orientation.y = quat_curr.y();
		curr_pose.orientation.z = quat_curr.z();

		switch (state_)
		{
			case 0: //joint controller
			{
				if (first_msg_ == true)
				{
					std::cout << "prima lettura" << std::endl;
					std::cout << "input_" << input_.feedback().robot().joints().position().values_size()<< std::endl;
					std::cout << "output_" << output_.mutable_robot()->mutable_joints()->mutable_position()->values_size()<< std::endl;

					// Reset all references, if it is the first message.
					output_.mutable_robot()->mutable_joints()->mutable_position()->Clear();
					output_.mutable_robot()->mutable_joints()->Clear();
					output_.mutable_robot()->Clear();
					output_.Clear();

					if(kdl_chain_.getNrOfJoints() == 7) output_.mutable_external()->mutable_joints()->mutable_position()->Clear();
					
					// initial_positions_.CopyFrom(input_.feedback().robot().joints().position());
					// output_.mutable_robot()->mutable_joints()->mutable_position()->CopyFrom(initial_positions_);
					output_.mutable_robot()->mutable_joints()->mutable_position()->CopyFrom(input_.feedback().robot().joints().position());
					output_.mutable_external()->mutable_joints()->mutable_position()->CopyFrom(input_.feedback().external().joints().position());
					// egm_interface_.write(output_);

					joint_array_cmd_.data.clear();
					int j = 0;
					for (int i = 0; i < kdl_chain_.getNrOfJoints(); i++)
					{
						if ((kdl_chain_.getNrOfJoints() == 7) && (i == 2))
						{
							joint_array_cmd_.data[i] = input_.feedback().external().joints().position().values(0) * 3.14159265358979323846 / 180.0;
						}
						else
						{
							joint_array_cmd_.data[i] = input_.feedback().robot().joints().position().values(j) * 3.14159265358979323846 / 180.0;
							j++;
						}
					}

					
					first_msg_ = false;
					
					
				}
				else
				{
					double tmp_input = input_.feedback().robot().joints().position().values(0) * 3.14159265358979323846 / 180.0;
					

					if (output_.mutable_robot()->mutable_joints()->mutable_position()->values_size() > 1)
					{

						int j = 0;
						for (int i = 0; i < kdl_chain_.getNrOfJoints(); i++)
						{
							if ((kdl_chain_.getNrOfJoints() == 7) && (i == 2))
							{
								if (!std::isnan(joint_array_cmd_.data[i]) && !std::isinf(joint_array_cmd_.data[i])) output_.mutable_external()->mutable_joints()->mutable_position()->set_values(0, joint_array_cmd_.data[i] / 3.14159265358979323846 * 180.0);
							}
							else
							{
								if (!std::isnan(joint_array_cmd_.data[i]) && !std::isinf(joint_array_cmd_.data[i])) output_.mutable_robot()->mutable_joints()->mutable_position()->set_values(j, joint_array_cmd_.data[i] / 3.14159265358979323846 * 180.0);
								j++;
							}
						}

						egm_interface_.write(output_);
					}

					
				}
				break;
			}

			case 1:	//pose controller
			{
				if (first_msg_ == true)
				{
					// Reset all references, if it is the first message.
					output_.Clear();
					initial_pose_.CopyFrom(input_.feedback().robot().cartesian().pose());
					output_.mutable_robot()->mutable_cartesian()->mutable_pose()->CopyFrom(initial_pose_);


					pose_cmd_.data.clear();

					pose_cmd_.data[0] = input_.feedback().robot().cartesian().pose().position().x() / 1000.0;
					pose_cmd_.data[1] = input_.feedback().robot().cartesian().pose().position().y() / 1000.0;
					pose_cmd_.data[2] = input_.feedback().robot().cartesian().pose().position().z() / 1000.0;
					pose_cmd_.data[3] = input_.feedback().robot().cartesian().pose().euler().x() * 3.14159265358979323846 / 180.0;
					pose_cmd_.data[4] = input_.feedback().robot().cartesian().pose().euler().y() * 3.14159265358979323846 / 180.0;
					pose_cmd_.data[5] = input_.feedback().robot().cartesian().pose().euler().z() * 3.14159265358979323846 / 180.0;

					first_msg_ = false;
					std::cout << "prima lettura" << std::endl;
				}
				else
				{
					if (output_.mutable_robot()->mutable_joints()->mutable_position()->values_size() > 1)
					{
						if (!std::isnan(pose_cmd_.data[0]) && !std::isinf(pose_cmd_.data[0])) output_.mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_position()->set_x(pose_cmd_.data[0] * 1000.0);
						if (!std::isnan(pose_cmd_.data[1]) && !std::isinf(pose_cmd_.data[1])) output_.mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_position()->set_y(pose_cmd_.data[1] * 1000.0);
						if (!std::isnan(pose_cmd_.data[2]) && !std::isinf(pose_cmd_.data[2])) output_.mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_position()->set_z(pose_cmd_.data[2] * 1000.0);

						if (!std::isnan(pose_cmd_.data[3]) && !std::isinf(pose_cmd_.data[3])) output_.mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_euler()->set_x(pose_cmd_.data[3] / 3.14159265358979323846 * 180.0);
						if (!std::isnan(pose_cmd_.data[4]) && !std::isinf(pose_cmd_.data[4])) output_.mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_euler()->set_y(pose_cmd_.data[4] / 3.14159265358979323846 * 180.0);
						if (!std::isnan(pose_cmd_.data[5]) && !std::isinf(pose_cmd_.data[5])) output_.mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_euler()->set_z(pose_cmd_.data[5] / 3.14159265358979323846 * 180.0);
					}
					egm_interface_.write(output_);
				}
				break;
			}

			case 2: //LT
			{
				output_.Clear();
				egm_interface_.write(output_);
				break;
			}
		}

		pub_curr_pose_.publish(curr_pose);
		pub_joint_.publish(joint_curr_);
	}
	else
	{
		std::cout << "wait for message joint controller bridge" << std::endl;
	}

}


