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
  ros::init(argc, argv, "subscription_node");
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
  // Subscription service
  //----------------------------------------------------------
  ROS_INFO("========== Subscription service sample ==========");
  ROS_WARN("Note: It is assumed that the robot controller's RAPID program is running");
  std::string io_signal = "TEST_IO_SIGNAL_2";

  // Specify which resource(s) to subscribe to, and the priority level.
  abb::rws::RWSClient::SubscriptionResources subscription_resources;
  subscription_resources.addIOSignal(io_signal, abb::rws::RWSClient::SubscriptionResources::HIGH);

  ROS_INFO_STREAM("1: Press Enter to start a subscription on '" << io_signal << "'");
  std::cin.get();

  if(rws_interface.startSubscription(subscription_resources))
  {
    ROS_INFO("Wait for subscription events...");
    for(int i = 0; ros::ok() && i < 5; ++i)
    {
      if(rws_interface.waitForSubscriptionEvent())
      {
        ROS_INFO_STREAM("Event " << i + 1 << " of 5 received");
      }
      else
      {
        ROS_WARN_STREAM("Event " << i + 1 << " of 5 failed (e.g. due to timeout)");
      }
    }

    ROS_INFO("End subscription...");
    rws_interface.endSubscription();
  }
  else
  {
    ROS_WARN("Failed to register the subscription (e.g. due to no connection with the controller)");
  }

  return 0;
}
