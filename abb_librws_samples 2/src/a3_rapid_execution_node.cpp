/***********************************************************************************************************************
 *
 * Copyright (c) 2018, ABB Schweiz AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with
 * or without modification, are permitted provided that
 * the following conditions are met:
 *
 *    * Redistributions of source code must retain the
 *      above copyright notice, this list of conditions
 *      and the following disclaimer.
 *    * Redistributions in binary form must reproduce the
 *      above copyright notice, this list of conditions
 *      and the following disclaimer in the documentation
 *      and/or other materials provided with the
 *      distribution.
 *    * Neither the name of ABB nor the names of its
 *      contributors may be used to endorse or promote
 *      products derived from this software without
 *      specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ***********************************************************************************************************************
 */

#include <ros/ros.h>
#include <abb_librws/rws_interface.h>

#include "abb_librws_samples/utility.h"

int main(int argc, char** argv)
{
  //----------------------------------------------------------
  // Preparations
  //----------------------------------------------------------
  // Initialize the node.
  ros::init(argc, argv, "rapid_execution_node");
  ros::NodeHandle node_handle;

  // Retrive the ROS parameter for the RWS server's IP-address.
  std::string rws_ip_address;
  if(!getRWSIpAddress(rws_ip_address))
  {
    return 0;
  }

  // Create a RWS interface:
  // * Sets up a RWS client (that can connect to the robot controller's RWS server).
  // * Provides APIs to the user (for accessing the RWS server's services).
  //
  // Note: It is important to set the correct IP-address here, to the RWS server.
  abb::rws::RWSInterface rws_interface(rws_ip_address);

  //----------------------------------------------------------
  // Stop, reset, and start RAPID execution
  //----------------------------------------------------------
  ROS_INFO("========== Stop, reset, and start RAPID execution sample ==========");
  ROS_WARN("Note: It is assumed that the robot controller is in 'Auto Mode' and has 'Motors On'");

  // Stop RAPID execution.
  ROS_INFO("1: Press Enter to stop RAPID execution");
  std::cin.get();
  ROS_INFO_STREAM("RAPID running (before): " << rws_interface.isRAPIDRunning());
  rws_interface.stopRAPIDExecution();
  ROS_INFO_STREAM("RAPID running (after): " << rws_interface.isRAPIDRunning());

  ROS_INFO("==========");

  // Reset RAPID program pointer to main.
  ROS_INFO("2: Press Enter to reset the RAPID program pointer to main");
  std::cin.get();
  rws_interface.resetRAPIDProgramPointer();

  ROS_INFO("==========");

  // Start RAPID execution.
  ROS_INFO("3: Press Enter to start RAPID execution");
  std::cin.get();
  ROS_INFO_STREAM("RAPID running (before): " << rws_interface.isRAPIDRunning());
  rws_interface.startRAPIDExecution();
  ROS_INFO_STREAM("RAPID running (after): " << rws_interface.isRAPIDRunning());

  return 0;
}
