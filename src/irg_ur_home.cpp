// ------------------------------------------------
// This file moves an IRG-related UR5e to the designated "home" pose.
// Pose includes wrist rotation to normalize an attached Hand-E gripper.
// Author: Matt Boyd
// Author Email: mcboyd@mit.eu
// ------------------------------------------------


#include <ur_client_library/ur/dashboard_client.h>
#include <ur_client_library/ur/ur_driver.h>
#include <ur_client_library/types.h>

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <thread>
#include "ur_client_library/control/reverse_interface.h"

using namespace urcl;

const std::string DEFAULT_ROBOT_IP = "172.16.0.10";
const std::string SCRIPT_FILE = "/home/irg/catkin_ws/src/irgmit_ur_launch/resources/external_control.urscript";
const std::string OUTPUT_RECIPE = "/home/irg/catkin_ws/src/irgmit_ur_launch/resources/rtde_output_recipe.txt";
const std::string INPUT_RECIPE = "/home/irg/catkin_ws/src/irgmit_ur_launch/resources/rtde_input_recipe.txt";
const std::string CALIBRATION_CHECKSUM = "calib_1919144596799550491";

std::unique_ptr<DashboardClient> g_my_dashboard;

// We need a callback function to register. See UrDriver's parameters for details.
void handleRobotProgramState(bool program_running)
{
  // Print the text in green so we see it better
  std::cout << "\033[1;32mProgram running: " << std::boolalpha << program_running << "\033[0m\n" << std::endl;
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
  urcl::setLogLevel(urcl::LogLevel::NONE);

  std::string robot_ip = DEFAULT_ROBOT_IP;

  // Making the robot ready for the program by:
  // Connect the robot Dashboard
  g_my_dashboard.reset(new DashboardClient(robot_ip));
  if (!g_my_dashboard->connect())
  {
    URCL_LOG_ERROR("Could not connect to dashboard");
    return 1;
  }

  // Stop program, if there is one running
  if (!g_my_dashboard->commandStop())
  {
    URCL_LOG_ERROR("Could not send stop program command");
    return 1;
  }

  // Now the robot is ready to receive a program
  std::unique_ptr<ToolCommSetup> tool_comm_setup;

  const bool HEADLESS = true;
  
  std::unique_ptr<UrDriver> g_my_driver;
  g_my_driver.reset(new UrDriver(robot_ip, SCRIPT_FILE, OUTPUT_RECIPE, INPUT_RECIPE, &handleRobotProgramState, HEADLESS,
	                               std::move(tool_comm_setup), CALIBRATION_CHECKSUM));

  g_my_driver->registerTrajectoryDoneCallback(&handleTrajectoryState);


  // Setup vector of joint "home" angles
  urcl::vector6d_t home_angles;
  home_angles[0] = 1.5370889902114868;
  home_angles[1] = -2.45106901744985;
  home_angles[2] = 2.0207517782794397 ;
  home_angles[3] = -1.1723517936519166;
  home_angles[4] = 4.706076622009277;
  home_angles[5] = -1.500333611165182;

  // Once RTDE communication is started, we have to make sure to read from the interface buffer, as
  // otherwise we will get pipeline overflows. Therefore, do this directly before starting your main
  // loop.
  g_my_driver->startRTDECommunication();
  URCL_LOG_INFO("RTDE communication started.");
  g_my_driver->writeKeepalive();

  // First, go to home location
  g_trajectory_running = true;
  g_my_driver->writeTrajectoryControlMessage(urcl::control::TrajectoryControlMessage::TRAJECTORY_START, 1);
  // "false" below indicates the input vector is joint-based rather than cartesian 
  g_my_driver->writeTrajectoryPoint(home_angles, false);
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
  URCL_LOG_INFO("Move home completed.");

  return 0;  

}
