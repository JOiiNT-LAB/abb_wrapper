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

#include <control_msgs/JointControllerState.h> 

#include <urdf/model.h>
#include <controller_interface/controller.h>
#include <ros/node_handle.h>
#include <ros/ros.h>

#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/chaindynparam.hpp> //this to compute the gravity vector
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <eigen3/Eigen/Eigen>
#include <skew_symmetric.h>
#include <pseudo_inversion.h>

#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/Empty.h>



class OneTaskInvKin
{
	public:
		OneTaskInvKin();
		~OneTaskInvKin();

		void run();
		double dt_;

	private:

		void callback_joint_states(const sensor_msgs::JointState::ConstPtr& msg);
		void callback_des_pose(const geometry_msgs::Pose::ConstPtr& msg);
		bool callback_inv_kin(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);



		void init();
		void update();


	
		ros::NodeHandle n_;
		ros::Subscriber sub_joint_states_, sub_des_pos_;
		ros::Publisher pub_joint_cmd_, pub_curr_pose_, pub_joint_, pub_orient_error_;
		ros::ServiceServer server_inv_kin_;

		KDL::Chain kdl_chain_;

		KDL::JntArray  q_msr_;          // Joint measured positions
		KDL::JntArray  q_;        		// Joint computed positions
		KDL::Jacobian  J_;            // Jacobian
		KDL::Frame     x_;            // Tip pose                                                                                                                                       

		boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
		boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;

		Eigen::VectorXd joint_location_, q_eig_;
		Eigen::Vector3d pos_d_;
		Eigen::Quaterniond quat_d_, quat_old_;
		Eigen::MatrixXd k_;
		int step_, sign_curr_des_;
		bool flag_joint_msr_, first_quat_, first_des_;
		Eigen::MatrixXd W_;
};