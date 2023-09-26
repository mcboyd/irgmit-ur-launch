// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2022 Universal Robots A/S
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// -- END LICENSE BLOCK ------------------------------------------------

// In a real-world example it would be better to get those values from command line parameters / a
// better configuration system such as Boost.Program_options



#include <ur_client_library/ur/dashboard_client.h>
#include <ur_client_library/ur/ur_driver.h>
#include <ur_client_library/types.h>

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <thread>
#include "ur_client_library/control/reverse_interface.h"

#include <boost/process.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <unistd.h>

using namespace boost::process;
using namespace urcl;

const std::string DEFAULT_ROBOT_IP = "172.16.0.10";
const std::string SCRIPT_FILE = "/home/irg/catkin_ws/src/irgmit_ur_launch/resources/external_control.urscript";
const std::string OUTPUT_RECIPE = "/home/irg/catkin_ws/src/irgmit_ur_launch/resources/rtde_output_recipe.txt";
const std::string INPUT_RECIPE = "/home/irg/catkin_ws/src/irgmit_ur_launch/resources/rtde_input_recipe.txt";
const std::string CALIBRATION_CHECKSUM = "calib_1919144596799550491";
const std::string SOCAT_CMD = "socat pty,link=/tmp/ttyUR,raw,ignoreeof,waitslave tcp:172.16.0.10:54321";
const std::string GRIPPER_CMD = "python /home/irg/catkin_ws/src/robotiq/robotiq_2f_gripper_control/scripts/robotiq_2f_standalone_client.py";


std::unique_ptr<DashboardClient> g_my_dashboard;

// We need a callback function to register. See UrDriver's parameters for details.
void handleRobotProgramState(bool program_running)
{
  // Print the text in green so we see it better
  std::cout << "\033[1;32mProgram running: " << std::boolalpha << program_running << "\033[0m\n" << std::endl;
}

int main(int argc, char* argv[])
{
  urcl::setLogLevel(urcl::LogLevel::INFO);

  std::string robot_ip = DEFAULT_ROBOT_IP;

  // Parse how many seconds to run
  auto second_to_run = std::chrono::seconds(0);
  if (argc > 1)
  {
    second_to_run = std::chrono::seconds(std::stoi(argv[1]));
  }

  // Making the robot ready for the program by:
  // Connect the robot Dashboard
  g_my_dashboard.reset(new DashboardClient(robot_ip));
  if (!g_my_dashboard->connect())
  {
    URCL_LOG_ERROR("Could not connect to dashboard");
    return 1;
  }

  // // Stop program, if there is one running
  if (!g_my_dashboard->commandStop())
  {
    URCL_LOG_ERROR("Could not send stop program command");
    return 1;
  }

  boost::asio::io_service ios;
  boost::process::async_pipe p_in(ios);
  std::deque<std::string> input_queue;
  boost::mutex in_mutex;

  input_queue.push_back("o\n");

  child socat(SOCAT_CMD);
  child gripper(GRIPPER_CMD, std_in < p_in);
  in_mutex.lock();
  boost::asio::async_write(p_in,boost::asio::buffer(input_queue.front()),[&](const boost::system::error_code & ec, std::size_t n){ });
  in_mutex.unlock();
  input_queue.pop_front();


  // Now the robot is ready to receive a program
  std::unique_ptr<ToolCommSetup> tool_comm_setup;
  tool_comm_setup.reset(new urcl::ToolCommSetup());
  tool_comm_setup->setBaudRate(115200);
  tool_comm_setup->setToolVoltage(ToolVoltage::_24V);
  tool_comm_setup->setParity(Parity::NONE);
  tool_comm_setup->setStopBits(1);
  tool_comm_setup->setRxIdleChars(1.5);
  tool_comm_setup->setTxIdleChars(3.5);

  sleep(1);

  const bool HEADLESS = true;
  // Using this scope so g_my_driver is properly destroyed
  {
    std::unique_ptr<UrDriver> g_my_driver;
    g_my_driver.reset(new UrDriver(robot_ip, SCRIPT_FILE, OUTPUT_RECIPE, INPUT_RECIPE, &handleRobotProgramState, HEADLESS,
                                   std::move(tool_comm_setup), CALIBRATION_CHECKSUM));

    g_my_driver->startRTDECommunication();
    URCL_LOG_INFO("RTDE communication started.");
    g_my_driver->writeKeepalive();
  }  // End of Scope for g_my_driver
  
  sleep(1);

  pid_t pid = gripper.id();
  kill(pid, SIGINT);

  pid = socat.id();
  kill(pid, SIGINT);

  return 0;  

}
