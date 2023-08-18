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
const std::string SAVE_FILE = "/home/irg/museum_recordings/txts/users/output.txt";
const double CUBE_DIM = 0.035;  // 35mm
const double SINK_X = 0.4330208551070603; // + CUBE_DIM;
const double SINK_Y = -0.4609244169054163; // - CUBE_DIM;
const double SINK_Z = 0.3358450828078722; // + CUBE_DIM;


std::unique_ptr<DashboardClient> g_my_dashboard;

// We need a callback function to register. See UrDriver's parameters for details.
void handleRobotProgramState(bool program_running)
{
  // Print the text in green so we see it better
  std::cout << "\033[1;32mProgram running: " << std::boolalpha << program_running << "\033[0m\n" << std::endl;
}

void sendFreedriveMessageOrDie(std::unique_ptr<UrDriver> & g_my_driver, const control::FreedriveControlMessage freedrive_action)
{
  bool ret = g_my_driver->writeFreedriveControlMessage(freedrive_action);
  if (!ret)
  {
    URCL_LOG_ERROR("Could not send joint command. Is the robot in remote control?");
    exit(1);
  }
}

// Callback function for trajectory execution.
bool g_trajectory_running(false);
void handleTrajectoryState(control::TrajectoryResult state)
{
  URCL_LOG_INFO("In callback.");
  // trajectory_state = state;
  g_trajectory_running = false;
  std::string report = "?";
  switch (state)
  {
    case control::TrajectoryResult::TRAJECTORY_RESULT_SUCCESS:
      report = "success";
      break;
    case control::TrajectoryResult::TRAJECTORY_RESULT_CANCELED:
      report = "canceled";
      break;
    case control::TrajectoryResult::TRAJECTORY_RESULT_FAILURE:
    default:
      report = "failure";
      break;
  }
  std::cout << "\033[1;32mTrajectory report: " << report << "\033[0m\n" << std::endl;
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

  std::list<std::string> csv_data;

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

  sleep(5);

  int counter = 0;

  const bool HEADLESS = true;
  // Using this scope so g_my_driver is properly destroyed after the freedrive / RTDE loop
  {
    std::unique_ptr<UrDriver> g_my_driver;
    g_my_driver.reset(new UrDriver(robot_ip, SCRIPT_FILE, OUTPUT_RECIPE, INPUT_RECIPE, &handleRobotProgramState, HEADLESS,
                                   std::move(tool_comm_setup), CALIBRATION_CHECKSUM));

    g_my_driver->registerTrajectoryDoneCallback(&handleTrajectoryState);

    // Setup cartesian moves to pickup cube and move to starting location
    urcl::vector6d_t grasp, inspection_start, g_tcp_pose;
    grasp[0] = -0.359541785191785;
    grasp[1] = -0.43876006460910113;
    grasp[2] = 0.4009993552206437;
    grasp[3] = 2.0812724;
    grasp[4] = -2.3146799;
    grasp[5] = 0.027137563;
    inspection_start[0] = -0.1008749399845134;
    inspection_start[1] = -0.4609244169054163;
    inspection_start[2] = 0.3358450828078722;
    inspection_start[3] = 2.0812724;
    inspection_start[4] = -2.3146799;
    inspection_start[5] = 0.027137563;


    // Once RTDE communication is started, we have to make sure to read from the interface buffer, as
    // otherwise we will get pipeline overflows. Therefore, do this directly before starting your main
    // loop.
    g_my_driver->startRTDECommunication();
    URCL_LOG_INFO("RTDE communication started.");
    g_my_driver->writeKeepalive();

    // First, go to grasp location
    g_trajectory_running = true;
    g_my_driver->writeTrajectoryControlMessage(urcl::control::TrajectoryControlMessage::TRAJECTORY_START, 1);
    g_my_driver->writeTrajectoryPoint(grasp, true);
    while (g_trajectory_running)
    {
      std::unique_ptr<rtde_interface::DataPackage> data_pkg = g_my_driver->getDataPackage();
      if (data_pkg)
      {        
        bool ret = g_my_driver->writeJointCommand(vector6d_t(), comm::ControlMode::MODE_FORWARD);

        if (!ret)
        {
          URCL_LOG_ERROR("Could not send joint command. Is the robot in remote control?");
          return 1;
        }
      }
    }
    URCL_LOG_INFO("First move completed.");

    // Second, move -Z to pickup cube
    grasp[2] -= 0.05;
    g_trajectory_running = true;
    g_my_driver->writeTrajectoryControlMessage(urcl::control::TrajectoryControlMessage::TRAJECTORY_START, 1);
    g_my_driver->writeTrajectoryPoint(grasp, true);
    URCL_LOG_INFO("Send trajectory 2 completed.");
    while (g_trajectory_running)
    {
      std::unique_ptr<rtde_interface::DataPackage> data_pkg = g_my_driver->getDataPackage();
      if (data_pkg)
      {
        bool ret = g_my_driver->writeJointCommand(vector6d_t(), comm::ControlMode::MODE_FORWARD);

        if (!ret)
        {
          URCL_LOG_ERROR("Could not send joint command. Is the robot in remote control?");
          return 1;
        }
      }
    }
    URCL_LOG_INFO("Second move completed.");

    // Third close the gripper on the cube
    input_queue.push_back("x\n");  // "x" (or any input other than o, c, or q) tells it to close to 30mm stroke, slightly smaller than the 35mm cube
    in_mutex.lock();
    boost::asio::async_write(p_in,boost::asio::buffer(input_queue.front()),[&](const boost::system::error_code & ec, std::size_t n){ });
    in_mutex.unlock();
    input_queue.pop_front();
    // Basically pause for 5 seconds to allow the gripper time to close
    // It would be better if this was watching the output queue of the gripper process for feedback
    // ... OR if the python gripper driver class was imported and used directly in this code...
    while (counter<2500) {
      std::unique_ptr<rtde_interface::DataPackage> data_pkg = g_my_driver->getDataPackage();
      if (data_pkg)
      {
        g_my_driver->writeKeepalive();  // REQUIRED to keep the reverse interface connection open when not sending another command in this loop
        counter += 1;
      }
    }
    URCL_LOG_INFO("Third move completed.");


    // Fourth, move +Z back to grasp location
    grasp[2] += 0.05;
    g_trajectory_running = true;
    g_my_driver->writeTrajectoryControlMessage(urcl::control::TrajectoryControlMessage::TRAJECTORY_START, 1);
    g_my_driver->writeTrajectoryPoint(grasp, true);
    URCL_LOG_INFO("Send trajectory 4 completed.");
    while (g_trajectory_running)
    {
      std::unique_ptr<rtde_interface::DataPackage> data_pkg = g_my_driver->getDataPackage();
      if (data_pkg)
      {
        bool ret = g_my_driver->writeJointCommand(vector6d_t(), comm::ControlMode::MODE_FORWARD);

        if (!ret)
        {
          URCL_LOG_ERROR("Could not send joint command. Is the robot in remote control?");
          return 1;
        }
      }
    }
    URCL_LOG_INFO("Fourth move completed.");

    // Fifth, move to inspection start lcoation
    g_trajectory_running = true;
    g_my_driver->writeTrajectoryControlMessage(urcl::control::TrajectoryControlMessage::TRAJECTORY_START, 1);
    g_my_driver->writeTrajectoryPoint(inspection_start, true);
    URCL_LOG_INFO("Send trajectory 5 completed.");
    while (g_trajectory_running)
    {
      std::unique_ptr<rtde_interface::DataPackage> data_pkg = g_my_driver->getDataPackage();
      if (data_pkg)
      {
        bool ret = g_my_driver->writeJointCommand(vector6d_t(), comm::ControlMode::MODE_FORWARD);

        if (!ret)
        {
          URCL_LOG_ERROR("Could not send joint command. Is the robot in remote control?");
          return 1;
        }
      }
    }
    URCL_LOG_INFO("Fifth move completed.");

    std::chrono::duration<double> time_done(0);
    std::chrono::duration<double> timeout(second_to_run);
    std::chrono::duration<double> proctimeout(50);
    bool killed=false;
    auto stopwatch_last = std::chrono::steady_clock::now();
    auto stopwatch_now = stopwatch_last;
    g_my_driver->writeKeepalive();
    sendFreedriveMessageOrDie(g_my_driver, control::FreedriveControlMessage::FREEDRIVE_START);

    // std::cout << grasp[3] << "," << grasp[4] << "," << grasp[5] << std::endl;

    while (true)
    {
      // URCL_LOG_INFO("In while loop...");
      // Read latest RTDE package. This will block for a hard-coded timeout (see UrDriver), so the
      // robot will effectively be in charge of setting the frequency of this loop.
      // In a real-world application this thread should be scheduled with real-time priority in order
      // to ensure that this is called in time.
      std::unique_ptr<rtde_interface::DataPackage> data_pkg = g_my_driver->getDataPackage();
      urcl::vector6d_t test;
      if (data_pkg)
      {
        // URCL_LOG_INFO("In while loop...1");
        sendFreedriveMessageOrDie(g_my_driver, control::FreedriveControlMessage::FREEDRIVE_NOOP);
        data_pkg->getData("actual_TCP_pose", test);
        csv_data.push_back(data_pkg->toCsv());
        // URCL_LOG_INFO("In while loop...2");
        // std::cout << data_pkg->toCsv();

        if (time_done > timeout && second_to_run.count() != 0)
        {
          // csv_data.assign({ data_pkg->toCsv() });
          // std::cout << data_pkg->toCsv();
          URCL_LOG_INFO("Timeout reached.");
          // std::cout << data_pkg->toString();
          // std::cout << "tcp data: " << test[0] << ", " << test[1] <<  std::endl;
          break;
        }
        
        if (test[0] > SINK_X && test[1] < SINK_Y && test[2] > SINK_Z)
        {
          input_queue.push_back("o\n");
          in_mutex.lock();
          boost::asio::async_write(p_in,boost::asio::buffer(input_queue.front()),[&](const boost::system::error_code & ec, std::size_t n){ });
          in_mutex.unlock();
          input_queue.pop_front();
          // std::cout << data_pkg->toCsv();
          URCL_LOG_INFO("Sink reached.");
          // std::cout << data_pkg->toString();
          // std::cout << "tcp data: " << test[0] << ", " << test[1] <<  std::endl;
          break;
        }
      }
      else
      {
        URCL_LOG_WARN("Could not get fresh data package from robot");
      }

      stopwatch_now = std::chrono::steady_clock::now();
      time_done += stopwatch_now - stopwatch_last;
      stopwatch_last = stopwatch_now;
    }
    sendFreedriveMessageOrDie(g_my_driver, control::FreedriveControlMessage::FREEDRIVE_STOP);
  }  // End of Scope for g_my_driver
  
  sleep(1);

  pid_t pid = gripper.id();
  kill(pid, SIGINT);

  pid = socat.id();
  kill(pid, SIGINT);

  // Outputting data in list captured during teaching to disk
  std::cout << "The elements of the TCP pose list are being written to disk..." << std::endl;
  std::ofstream output(SAVE_FILE);
  for (std::string& a : csv_data)
    output << a << "\n";

  return 0;  

}
