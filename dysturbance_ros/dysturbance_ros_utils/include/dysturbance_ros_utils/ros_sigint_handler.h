/***
 *  Software License Agreement: BSD 3-Clause License
 *
 *  Copyright (c) 2020, qbrobotics®
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 *  following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this list of conditions and the
 *    following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *    following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 *  * Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef DYSTURBANCE_ROS_SIGINT_HANDLER_H
#define DYSTURBANCE_ROS_SIGINT_HANDLER_H

// specific for custom SIGINT handler
#include <signal.h>
#include <ros/xmlrpc_manager.h>

namespace ros_sigint_handler {
// signal-safe flag for whether shutdown is requested
static volatile sig_atomic_t g_request_shutdown = 0;

// replacement SIGINT handler
static void mySigIntHandler(int _) {
  (void)_;
  g_request_shutdown = 1;  // set flag
}

// replacement "shutdown" XMLRPC callback
static void shutdownCallback(XmlRpc::XmlRpcValue &params, XmlRpc::XmlRpcValue &result) {
  int num_params = 0;
  if (params.getType() == XmlRpc::XmlRpcValue::TypeArray) {
    num_params = params.size();
  }
  if (num_params > 1) {
    std::string reason = params[1];
    ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
    g_request_shutdown = 1;  // set flag
  }

  result = ros::xmlrpc::responseInt(1, "", 0);
}

int isShuttingDown() {
  return g_request_shutdown;
}

void overrideHandlers() {
  // override SIGINT handler
  signal(SIGINT, mySigIntHandler);
  // override XMLRPC shutdown
  ros::XMLRPCManager::instance()->unbind("shutdown");
  ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);
}
}  // namespace ros_sigint_handler

#endif // DYSTURBANCE_ROS_SIGINT_HANDLER_H