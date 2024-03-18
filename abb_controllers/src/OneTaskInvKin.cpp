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
#include <OneTaskInvKin.h>
OneTaskInvKin::OneTaskInvKin()
{

	std::string robot_description, root_name, tip_name;
	robot_description = n_.getNamespace() + "/robot_description";

	if (!n_.getParam("root_name", root_name))
	{
		ROS_ERROR_STREAM("OneTaskInvKin: No root name found on parameter server (" << n_.getNamespace() << "/root_name)");
	}

	if (!n_.getParam("tip_name", tip_name))
	{
		ROS_ERROR_STREAM("OneTaskInvKin: No tip name found on parameter server (" << n_.getNamespace() << "/tip_name)");
	}

	// Construct an URDF model from the xml string
	std::string xml_string;

	if (n_.hasParam(robot_description))
		n_.getParam(robot_description.c_str(), xml_string);
	else
	{
		ROS_ERROR("Parameter %s not set, shutting down node...", robot_description.c_str());
		n_.shutdown();
	}

	if (xml_string.size() == 0)
	{
		ROS_ERROR("Unable to load robot model from parameter %s", robot_description.c_str());
		n_.shutdown();
	}

	ROS_DEBUG("%s content\n%s", robot_description.c_str(), xml_string.c_str());

	// Get urdf model out of robot_description
	urdf::Model model;
	if (!model.initString(xml_string))
	{
		ROS_ERROR("Failed to parse urdf file");
		n_.shutdown();
	}
	ROS_INFO("Successfully parsed urdf file");

	KDL::Tree kdl_tree_;
	if (!kdl_parser::treeFromUrdfModel(model, kdl_tree_))
	{
		ROS_ERROR("Failed to construct kdl tree");
		n_.shutdown();
	}

	// Populate the KDL chain
	if (!kdl_tree_.getChain(root_name, tip_name, kdl_chain_))
	{
		ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
		ROS_ERROR_STREAM("  " << root_name << " --> " << tip_name);
		ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfJoints() << " joints");
		ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfSegments() << " segments");
		ROS_ERROR_STREAM("  The segments are:");

		KDL::SegmentMap segment_map = kdl_tree_.getSegments();
		KDL::SegmentMap::iterator it;

		for (it = segment_map.begin(); it != segment_map.end(); it++)
			ROS_ERROR_STREAM("    " << (*it).first);
	}

	ROS_DEBUG("Number of segments: %d", kdl_chain_.getNrOfSegments());
	ROS_DEBUG("Number of joints in chain: %d", kdl_chain_.getNrOfJoints());

	jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
	fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));

	q_msr_.resize(kdl_chain_.getNrOfJoints());
	q_.resize(kdl_chain_.getNrOfJoints());
	J_.resize(kdl_chain_.getNrOfJoints());
	q_eig_ = Eigen::VectorXd::Zero(kdl_chain_.getNrOfJoints());

	k_ = Eigen::MatrixXd::Identity(kdl_chain_.getNrOfJoints(), kdl_chain_.getNrOfJoints()) * 20.0;
	W_ = Eigen::MatrixXd::Identity(kdl_chain_.getNrOfJoints(), kdl_chain_.getNrOfJoints());
	W_(4, 4) = 0.1;
	W_(5, 5) = 0.1;
	// W_(6,6) = 0.1;
	step_ = 2;
	flag_joint_msr_ = false;
	first_quat_ = true;
	first_des_ = false;

	//Subscriber
	sub_joint_states_ = n_.subscribe("joint_states", 1, &OneTaskInvKin::callback_joint_states, this);
	sub_des_pos_ = n_.subscribe("pose_des", 10, &OneTaskInvKin::callback_des_pose, this);

	//Publisher
	pub_joint_cmd_ = n_.advertise<std_msgs::Float64MultiArray>("joint_cmd", 1);
	pub_orient_error_ = n_.advertise<geometry_msgs::Vector3>("orient_error", 1);

	//server
	server_inv_kin_ = n_.advertiseService("inv_kin", &OneTaskInvKin::callback_inv_kin, this);
}

OneTaskInvKin::~OneTaskInvKin()
{
}

void OneTaskInvKin::callback_joint_states(const sensor_msgs::JointState::ConstPtr &msg)
{
	for (int i = 0; i < kdl_chain_.getNrOfJoints(); i++)
	{
		q_msr_(i) = msg->position[i];
	}

	if (flag_joint_msr_ == false)
	{
		step_ = 0;
		flag_joint_msr_ = true;
	}
}

void OneTaskInvKin::callback_des_pose(const geometry_msgs::Pose::ConstPtr &msg)
{
	pos_d_(0) = msg->position.x;
	pos_d_(1) = msg->position.y;
	pos_d_(2) = msg->position.z;

	quat_d_.w() = msg->orientation.w;
	quat_d_.x() = msg->orientation.x;
	quat_d_.y() = msg->orientation.y;
	quat_d_.z() = msg->orientation.z;
}

bool OneTaskInvKin::callback_inv_kin(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	step_ = 2;
	flag_joint_msr_ = false;
	first_quat_ = true;
	return true;
}

void OneTaskInvKin::init()
{
	q_ = q_msr_;

	for (int i = 0; i < kdl_chain_.getNrOfJoints(); i++)
		q_eig_(i) = q_msr_(i);

	// computing forward kinematics
	fk_pos_solver_->JntToCart(q_msr_, x_);
	Eigen::Matrix3d orient_d;
	for (int i = 0; i < 3; i++)
	{
		pos_d_(i) = x_.p(i);

		for (int j = 0; j < 3; j++)
		{
			orient_d(i, j) = x_.M(i, j);
		}
	}
	quat_d_ = orient_d;

	std::cout << "q_msr_: " << q_msr_(0) << ", " << q_msr_(1) << ", " << q_msr_(2) << ", " << q_msr_(3) << ", " << q_msr_(4) << ", " << q_msr_(5) << std::endl;
}

void OneTaskInvKin::update()
{
	KDL::Frame x_curr_yumi;
	// Compute the forward kinematics and Jacobian (at this location).
	fk_pos_solver_->JntToCart(q_, x_);
	jnt_to_jac_solver_->JntToJac(q_, J_);

	Eigen::Vector3d pos, pos_curr_yumi, e_pos, e_quat;
	Eigen::Matrix3d orient, orient_curr_yumi, skew;
	Eigen::MatrixXd Jac_, Jac_pinv, Jac_pinv_W, tmp_matrix, tmp_matrix_pinv;
	Eigen::Quaterniond quat, quat_curr_yumi;
	Eigen::VectorXd e(Eigen::VectorXd::Zero(6));
	Eigen::VectorXd qdot(Eigen::VectorXd::Zero(kdl_chain_.getNrOfJoints()));
	KDL::Vector quat_d_vec;
	std_msgs::Float64MultiArray joint_cmd;
	joint_cmd.data.clear();

	Jac_.resize(6, kdl_chain_.getNrOfJoints());

	for (int i = 0; i < 3; i++)
	{
		pos(i) = x_.p(i);

		for (int j = 0; j < 3; j++)
		{
			orient(i, j) = x_.M(i, j);
		}
	}

	for (int i = 0; i < 6; i++)
	{
		for (int j = 0; j < kdl_chain_.getNrOfJoints(); j++)
		{
			Jac_(i, j) = J_(i, j);
		}
	}

	//from matrix to quat
	quat = orient;
	quat.normalize();

	if (first_quat_)
	{
		first_quat_ = false;
		quat_old_ = quat;
	}

	// rotation to quaternion issue , "Sign Flip" , check  http://www.dtic.mil/dtic/tr/fulltext/u2/1043624.pdf
	double sign_check = quat.w() * quat_old_.w() + quat.x() * quat_old_.x() + quat.y() * quat_old_.y() + quat.z() * quat_old_.z();
	if (sign_check < 0.0)
	{
		quat.w() = quat.w() * (-1);
		quat.vec() = quat.vec() * (-1);
	}

	quat_old_ = quat;

	// rotation to quaternion issue , "Sign Flip" , check  http://www.dtic.mil/dtic/tr/fulltext/u2/1043624.pdf
	double sign_check_curr_des = quat.w() * quat_d_.w() + quat.x() * quat_d_.x() + quat.y() * quat_d_.y() + quat.z() * quat_d_.z();
	if (sign_check_curr_des < 0.0)
		sign_curr_des_ = (-1.0);
	else
		sign_curr_des_ = (1.0);

	quat_d_.w() = quat_d_.w() * sign_curr_des_;
	quat_d_.vec() = quat_d_.vec() * sign_curr_des_;

	quat_d_vec(0) = quat_d_.x();
	quat_d_vec(1) = quat_d_.y();
	quat_d_vec(2) = quat_d_.z();

	skew_symmetric(quat_d_vec, skew);

	e_pos = pos_d_ - pos;
	e_quat = (quat.w() * quat_d_.vec()) - (quat_d_.w() * quat.vec()) - (skew * quat.vec());

	e << e_pos, e_quat;

	pseudo_inverse(Jac_, Jac_pinv, true);

	qdot = k_ * Jac_pinv * e;

	q_eig_ += qdot * dt_;

	for (int i = 0; i < kdl_chain_.getNrOfJoints(); i++)
	{
		q_(i) = q_eig_(i);
		joint_cmd.data.push_back(q_(i));
	}
	geometry_msgs::Vector3 msg_orient_error;
	msg_orient_error.x = e_quat.x();
	msg_orient_error.y = e_quat.y();
	msg_orient_error.z = e_quat.z();

	pub_orient_error_.publish(msg_orient_error);
	pub_joint_cmd_.publish(joint_cmd);
}

void OneTaskInvKin::run()
{
	switch (step_)
	{
	case 0:
	{
		init();
		step_ = 1;
		break;
	}
	case 1:
	{
		update();
		break;
	}
	case 2:
	{
		// nothing
		break;
	}
	}
}