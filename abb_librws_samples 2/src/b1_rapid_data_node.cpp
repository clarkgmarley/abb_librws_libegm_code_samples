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

// Custom RAPID Record.
struct TestCustomRecord : public abb::rws::RAPIDRecord
{
  TestCustomRecord() : RAPIDRecord("TestCustomRecord")
  {
    // Important: Order MUST match the RAPID record definition in the RAPID program (in the robot controller system).
    components_.push_back(&counter);
    components_.push_back(&joint_target);
    components_.push_back(&rob_target);
  }

  abb::rws::RAPIDNum counter;
  abb::rws::JointTarget joint_target;
  abb::rws::RobTarget rob_target;
};

int main(int argc, char** argv)
{
  //----------------------------------------------------------
  // Preparations
  //----------------------------------------------------------
  // Initialize the node.
  ros::init(argc, argv, "rapid_data_node");
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
  // Reading/writing of RAPID data
  //----------------------------------------------------------
  ROS_INFO("========== Reading/writing of RAPID data sample ==========");
  ROS_WARN("Note: It is assumed that the robot controller's is in 'Auto Mode'");

  // RAPID task and module (i.e. the "path" to the RAPID data location in the robot system).
  std::string rapid_task = "T_ROB1";
  std::string rapid_module = "TRobMain";

  ROS_INFO("1: Press Enter to read and write RAPID data");
  std::cin.get();

  //-----------------------------
  // Basic RAPID data
  //-----------------------------
  abb::rws::RAPIDNum test_num;
  abb::rws::RAPIDDnum test_dnum;
  abb::rws::RAPIDBool test_bool;
  abb::rws::RAPIDString test_string;

  ROS_INFO("Read basic RAPID data...");
  rws_interface.getRAPIDSymbolData(rapid_task, rapid_module, "test_num", &test_num);
  rws_interface.getRAPIDSymbolData(rapid_task, rapid_module, "test_dnum", &test_dnum);
  rws_interface.getRAPIDSymbolData(rapid_task, rapid_module, "test_bool", &test_bool);
  rws_interface.getRAPIDSymbolData(rapid_task, rapid_module, "test_string", &test_string);
  ROS_INFO_STREAM("'test_num': " << test_num.constructString());
  ROS_INFO_STREAM("'test_dnum': " << test_dnum.constructString());
  ROS_INFO_STREAM("'test_bool': " << test_bool.constructString());
  ROS_INFO_STREAM("'test_string': " << test_string.constructString());

  ROS_INFO("Write basic RAPID data...");
  test_num.value = test_num.value + 1.0;
  test_dnum.value = test_dnum.value + 2.2;
  test_bool.value = !test_bool.value;
  test_string.value = "HELLO";
  rws_interface.setRAPIDSymbolData(rapid_task, rapid_module, "test_num", test_num);
  rws_interface.setRAPIDSymbolData(rapid_task, rapid_module, "test_dnum", test_dnum);
  rws_interface.setRAPIDSymbolData(rapid_task, rapid_module, "test_bool", test_bool);
  rws_interface.setRAPIDSymbolData(rapid_task, rapid_module, "test_string", test_string);

  ROS_INFO("Read basic RAPID data...");
  rws_interface.getRAPIDSymbolData(rapid_task, rapid_module, "test_num", &test_num);
  rws_interface.getRAPIDSymbolData(rapid_task, rapid_module, "test_dnum", &test_dnum);
  rws_interface.getRAPIDSymbolData(rapid_task, rapid_module, "test_bool", &test_bool);
  rws_interface.getRAPIDSymbolData(rapid_task, rapid_module, "test_string", &test_string);
  ROS_INFO_STREAM("'test_num': " << test_num.constructString());
  ROS_INFO_STREAM("'test_dnum': " << test_dnum.constructString());
  ROS_INFO_STREAM("'test_bool': " << test_bool.constructString());
  ROS_INFO_STREAM("'test_string': " << test_string.constructString());

  //-----------------------------
  // Predefined RAPID record data
  //-----------------------------
  ROS_INFO("==========");
  abb::rws::Pose test_pose;

  ROS_INFO("Read predefined RAPID record data...");
  rws_interface.getRAPIDSymbolData(rapid_task, rapid_module, "test_pose", &test_pose);
  ROS_INFO_STREAM("'test_pose': " << test_pose.constructString());

  ROS_INFO("Write predefined RAPID record data...");
  test_pose.pos.x.value = test_pose.pos.x.value + 3.0;
  test_pose.pos.y.value = test_pose.pos.y.value + 6.0;
  test_pose.pos.z.value = test_pose.pos.z.value + 9.0;
  rws_interface.setRAPIDSymbolData(rapid_task, rapid_module, "test_pose", test_pose);

  ROS_INFO("Read predefined RAPID record data...");
  rws_interface.getRAPIDSymbolData(rapid_task, rapid_module, "test_pose", &test_pose);
  ROS_INFO_STREAM("'test_pose': " << test_pose.constructString());

  //-----------------------------
  // Custom RAPID record data
  //-----------------------------
  ROS_INFO("==========");
  TestCustomRecord test_custom_record;

  ROS_INFO("Read custom RAPID record data...");
  rws_interface.getRAPIDSymbolData(rapid_task, rapid_module, "test_custom_record", &test_custom_record);
  ROS_INFO_STREAM("'test_custom_record': " << test_custom_record.constructString());

  ROS_INFO("Write custom RAPID record data...");
  test_custom_record.counter.value = test_custom_record.counter.value + 4.0;
  test_custom_record.joint_target.robax.rax_1.value = test_custom_record.joint_target.robax.rax_1.value + 5.0;
  test_custom_record.rob_target.pos.x.value = test_custom_record.rob_target.pos.x.value + 6.0;
  rws_interface.setRAPIDSymbolData(rapid_task, rapid_module, "test_custom_record", test_custom_record);

  ROS_INFO("Read custom RAPID record data...");
  rws_interface.getRAPIDSymbolData(rapid_task, rapid_module, "test_custom_record", &test_custom_record);
  ROS_INFO_STREAM("'test_custom_record': " << test_custom_record.constructString());

  return 0;
}
