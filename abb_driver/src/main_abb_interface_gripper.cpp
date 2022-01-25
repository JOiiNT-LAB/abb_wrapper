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
#include <stropts.h>
#include <sys/ioctl.h>


#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>


bool gripper_cmd_ = false;
bool old_gripper_cmd_ = false;
bool callback_gripper(std_srvs::SetBool::Request  &req, std_srvs::SetBool::Response &res)
{
	
	gripper_cmd_ = req.data;
	return true;
}



int main(int argc, char** argv)
{
  //----------------------------------------------------------
  // Preparations
  //----------------------------------------------------------



  // Initialize the node.
	ros::init(argc, argv, "abb_interface_gripper_node");
	ros::NodeHandle nh;
	ros::Rate rate(50);

	ros::ServiceServer srv_gripper = nh.advertiseService("gripper", callback_gripper);

	int port_robot;

	std::string ip_robot;
	nh.getParam("ip_robot", ip_robot);
	nh.getParam("port_robot", port_robot);

	const Poco::Net::Context::Ptr ptrContext( new Poco::Net::Context( Poco::Net::Context::CLIENT_USE, "", "", "", Poco::Net::Context::VERIFY_NONE) );
	abb::rws::RWSStateMachineInterface rws_interface(ip_robot, port_robot, ptrContext);

	usleep(500000);
	rws_interface.services().sg().JogOut();
	rws_interface.services().sg().JogIn();
	rws_interface.services().sg().Calibrate();
	while(ros::ok())
	{
		if(gripper_cmd_ != old_gripper_cmd_)
		{
			if(gripper_cmd_)rws_interface.services().sg().GripIn();
			else if(!gripper_cmd_) rws_interface.services().sg().GripOut();
			old_gripper_cmd_ = gripper_cmd_;
		}

		ros::spinOnce();
		rate.sleep();
	}


return 0;
}
