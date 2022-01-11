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

// #ifndef CONTROLLER_BRIDGE_H
// #define CONTROLLER_BRIDGE_H

#include <ros/node_handle.h>
#include <ros/ros.h>
#include <abb_libegm/egm_controller_interface.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>

#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/chaindynparam.hpp> //this to compute the gravity vector
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>

#include <eigen3/Eigen/Eigen>
#include <abb_driver/bridge_com.h>




class controller_bridge
{ 
	public:
		controller_bridge(const unsigned short robot_port_number) : egm_interface_(io_service_, robot_port_number)
		{

			std::cout<<"Port_number: "<<robot_port_number << std::endl;
			wait_ = true;
			first_msg_ = true;
			first_quat_ = true;
		  	
		  	pose_cmd_.data.resize(6);
		  	state_ = 2;

		  	count_test_ = 0;

			sub_joint_cmd_ = n_.subscribe("joint_cmd", 1, &controller_bridge::callback_joint_cmd, this);
			sub_test_joint_ = n_.subscribe("test_joint", 1, &controller_bridge::callback_test_joint, this);
    		pub_joint_ = n_.advertise<sensor_msgs::JointState>("joint_states", 1);
    		pub_curr_pose_ = n_.advertise<geometry_msgs::Pose>("curr_pose", 1);
    		

    		server_bridge_com_ = n_.advertiseService("bridge_com", &controller_bridge::callback_bridge_com, this);

			if(!egm_interface_.isInitialized())
			{
				ROS_ERROR("EGM interface failed to initialize (e.g. due to port already bound)");
			}
			else std::cout<<"egm_interface INITIALIZED"<<std::endl;

			// Spin up a thread to run the io_service.
			thread_group_.create_thread(boost::bind(&boost::asio::io_service::run, &io_service_));

			ROS_INFO("1: Wait for an EGM communication session to start...");
			while(ros::ok() && wait_)
			{
				std::cout<< "wait_: "<< wait_ << std::endl;
				// wait_ = false;
				if(egm_interface_.isConnected())
				{
				  if(egm_interface_.getStatus().rapid_execution_state() == abb::egm::wrapper::Status_RAPIDExecutionState_RAPID_UNDEFINED)
				  {
				  	ROS_WARN("RAPID execution state is UNDEFINED (might happen first time after controller start/restart). Try to restart the RAPID program.");
				  }
				  else
				  {
				    wait_ = egm_interface_.getStatus().rapid_execution_state() != abb::egm::wrapper::Status_RAPIDExecutionState_RAPID_RUNNING;
				  }
				}

				usleep(50000);
			}
			//-----------------------------------------------------
			//	KDL
			//-----------------------------------------------------
			std::string robot_description, root_name, tip_name;
			robot_description = n_.getNamespace() + "/robot_description";


			if (!n_.getParam("root_name", root_name))
		    {
		        ROS_ERROR_STREAM("OneTaskInvKin: No root name found on parameter server ("<<n_.getNamespace()<<"/root_name)");
		    }

		    if (!n_.getParam("tip_name", tip_name))
		    {
		        ROS_ERROR_STREAM("OneTaskInvKin: No tip name found on parameter server ("<<n_.getNamespace()<<"/tip_name)");
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
		        ROS_ERROR("Unable to load robot model from parameter %s",robot_description.c_str());
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
		    if(!kdl_tree_.getChain(root_name, tip_name, kdl_chain_))
		    {
		        ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
		        ROS_ERROR_STREAM("  "<<root_name<<" --> "<<tip_name);
		        ROS_ERROR_STREAM("  Tree has "<<kdl_tree_.getNrOfJoints()<<" joints");
		        ROS_ERROR_STREAM("  Tree has "<<kdl_tree_.getNrOfSegments()<<" segments");
		        ROS_ERROR_STREAM("  The segments are:");

		        KDL::SegmentMap segment_map = kdl_tree_.getSegments();
		        KDL::SegmentMap::iterator it;

		        for( it=segment_map.begin(); it != segment_map.end(); it++ )
		          ROS_ERROR_STREAM( "    "<<(*it).first);

		    }

			q_msr_.resize(kdl_chain_.getNrOfJoints());
			joint_curr_.name.resize(kdl_chain_.getNrOfJoints());
			joint_curr_.position.resize(kdl_chain_.getNrOfJoints());


			for (unsigned int i=0; i < kdl_chain_.getNrOfSegments(); ++i)
			{
				KDL::Segment segment = kdl_chain_.getSegment(i);
				KDL::Joint joint = segment.getJoint();
				if (joint.getType() == KDL::Joint::JointType::None) continue;
				joint_curr_.name[i] = joint.getName();
			}

		  	joint_array_cmd_.data.resize(kdl_chain_.getNrOfJoints());
			fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));

		}
		~controller_bridge();

		double dt_;
		void run();

	private:
		ros::NodeHandle n_;
		void callback_joint_cmd(const std_msgs::Float64MultiArray::ConstPtr& msg);
		void callback_test_joint(const std_msgs::Float64::ConstPtr& msg);
		bool callback_bridge_com(abb_driver::bridge_com::Request  &req, abb_driver::bridge_com::Response &res);



		// Boost components for managing asynchronous UDP socket(s).
		boost::thread_group thread_group_;
		boost::asio::io_service io_service_;
		// const unsigned short robot_port_numberc= 0;

		abb::egm::EGMControllerInterface egm_interface_;

  		bool wait_, first_msg_, first_quat_;
  		abb::egm::wrapper::Input input_;
  		abb::egm::wrapper::Joints initial_positions_;
  		abb::egm::wrapper::CartesianPose initial_pose_;
  		abb::egm::wrapper::Output output_;

  		ros::Subscriber sub_joint_cmd_, sub_test_joint_;
  		ros::Publisher pub_joint_, pub_curr_pose_;
  		ros::ServiceServer server_bridge_com_;

  		std_msgs::Float64MultiArray joint_array_cmd_;
  		std_msgs::Float64MultiArray pose_cmd_;
  		sensor_msgs::JointState joint_curr_;

  		// bool only_read_;
		KDL::Chain kdl_chain_;
		boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
		KDL::JntArray  q_msr_;          // Joint measured positions
		KDL::Frame     EE_pose_;        // Tip pose

		Eigen::Quaterniond quat_old_;  
		int state_;                                                                                                                                 



  		// cancella
  		double test_joint_;
  		int count_test_;

};

// #endif