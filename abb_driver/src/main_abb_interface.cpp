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
#include <abb_librws/rws_state_machine_interface.h>
#include <abb_librws/rws_interface.h>

#include <stdio.h>
#include <sys/select.h>
#include <termios.h>
// #include <stropts.h>
#include <sys/ioctl.h>


#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Float64MultiArray.h>
#include <abb_driver/srv_abb_controller.h>
#include <abb_driver/bridge_com.h>


int state_ = 0;
int previous_state_ = 0;
ros::ServiceClient client_bridge_com_, client_inv_kin_;

bool callback_controller(abb_driver::srv_abb_controller::Request  &req, abb_driver::srv_abb_controller::Response &res)
{
	state_ = req.controller;

	//reset inv kin
	std_srvs::Empty srv_ik;
	if(state_ == 0) if (!client_inv_kin_.call(srv_ik))ROS_ERROR("Failed to call service client_inv_kin_"); 

	abb_driver::bridge_com srv;
	srv.request.controller = state_;
	srv.request.start = false;
	if (!client_bridge_com_.call(srv))ROS_ERROR("Failed to call service client_bridge_com_");

	return true;
}


int main(int argc, char** argv)
{
	//----------------------------------------------------------
	// Preparations
	//----------------------------------------------------------

	// Initialize the node.
	ros::init(argc, argv, "abb_interface_node");
	ros::NodeHandle nh;
	ros::Rate rate(100);

	ros::ServiceServer srv_controller = nh.advertiseService("abb_controller", callback_controller);

	client_bridge_com_ = nh.serviceClient<abb_driver::bridge_com>("bridge_com");
	client_inv_kin_ = nh.serviceClient<std_srvs::Empty>("inv_kin");

    ros::Publisher pub_torque = nh.advertise<std_msgs::Float64MultiArray>("torque_meas", 1);
    std_msgs::Float64MultiArray torque_meas;


	int pos_corr_gain;
	int max_speed_deviation;
	int port_robot;

	std::string ip_robot, name_robot, task_robot;
	nh.getParam("ip_robot", ip_robot);
	nh.getParam("name_robot", name_robot);
	nh.getParam("task_robot", task_robot);
	nh.getParam("pos_corr_gain", pos_corr_gain);
	nh.getParam("max_speed_deviation", max_speed_deviation);
	nh.getParam("port_robot", port_robot);

	const Poco::Net::Context::Ptr ptrContext(new Poco::Net::Context( Poco::Net::Context::CLIENT_USE, "", "", "", Poco::Net::Context::VERIFY_NONE));
	abb::rws::RWSStateMachineInterface rws_interface(ip_robot, port_robot, ptrContext);
	abb::rws::RWSStateMachineInterface::EGMSettings egm_settings;

	rws_interface.stopRAPIDExecution();
	usleep(250000);
	rws_interface.requestMasterShip();
	usleep(250000);
	rws_interface.resetRAPIDProgramPointer();
	usleep(250000);
	rws_interface.releaseMasterShip();
	usleep(250000);
	rws_interface.startRAPIDExecution();
	// usleep(1000000);
	usleep(250000);

	if (rws_interface.services().egm().getSettings(task_robot, &egm_settings)) // safer way to apply settings
	{
		rws_interface.requestMasterShip();
		egm_settings.activate.max_speed_deviation.value = max_speed_deviation;
		egm_settings.allow_egm_motions.value = true; // brief Flag indicating if EGM motions are allowed to start
		egm_settings.run.pos_corr_gain.value = pos_corr_gain; // 0=velocity 1=position
		egm_settings.run.cond_time.value = 599.0;
		rws_interface.services().egm().setSettings(task_robot, egm_settings);
		rws_interface.releaseMasterShip();
	}
	else ROS_ERROR("EGM SETTING ERROR");

	// usleep(1000000);
	usleep(250000);


	previous_state_ = -1;

	while (ros::ok())
	{


		switch (state_)
		{
			case 0:
			{
				

				switch (previous_state_)
				{
					case 1:
						rws_interface.pulseIOSignal("EGM_STOP", 500000);
						break;
					case 2:
						rws_interface.requestMasterShip();
						usleep(250000);
						rws_interface.startRAPIDExecution();
						usleep(250000);
						rws_interface.setLeadThroughOff(name_robot);
						usleep(250000);
						rws_interface.pulseIOSignal("EGM_STOP_STREAM", 500000);
						usleep(250000);
						rws_interface.releaseMasterShip();
						break;
				}
				usleep(1000000);


				ROS_INFO("Joint Position");

				rws_interface.pulseIOSignal("EGM_START_JOINT", 500000);

				abb_driver::bridge_com srv;
				srv.request.controller = state_;
				srv.request.start = true;
				if (!client_bridge_com_.call(srv)) ROS_ERROR("Failed to call service client_bridge_com_");

				previous_state_ = state_;
				state_ = 100;

				break;
			}
			case 1:
			{
				switch (previous_state_)
				{
					case 0:
						rws_interface.pulseIOSignal("EGM_STOP", 500000);
						break;
					case 2:
						rws_interface.requestMasterShip();
						rws_interface.startRAPIDExecution();
						rws_interface.setLeadThroughOff(name_robot);
						rws_interface.pulseIOSignal("EGM_STOP_STREAM", 500000);
						rws_interface.releaseMasterShip();
						break;
				}
				usleep(1000000);

				ROS_INFO("Cartesian Position");

				rws_interface.pulseIOSignal("EGM_START_POSE", 500000);

				abb_driver::bridge_com srv;
				srv.request.controller = state_;
				srv.request.start = true;
				if (!client_bridge_com_.call(srv)) ROS_ERROR("Failed to call service client_bridge_com_");

				previous_state_ = state_;
				state_ = 100;

				break;
			}

			case 2:
			{
				ROS_INFO("LeadThrough");
				rws_interface.pulseIOSignal("EGM_STOP", 500000);
				usleep(1000000);
				rws_interface.pulseIOSignal("EGM_START_STREAM", 500000);

				usleep(1000000);
				rws_interface.requestMasterShip();
				rws_interface.setLeadThroughOn(name_robot);
				rws_interface.stopRAPIDExecution();
				rws_interface.releaseMasterShip();

				previous_state_ = state_;
				state_ = 100;
				break;
			}
			default: break;
		}
		ros::spinOnce();
		rate.sleep();
	}

	rws_interface.requestMasterShip();
	rws_interface.setLeadThroughOff(name_robot);
	rws_interface.pulseIOSignal("EGM_STOP", 500000);
	rws_interface.releaseMasterShip();
	usleep(500000);
	rws_interface.stopRAPIDExecution();

	return 0;
}
