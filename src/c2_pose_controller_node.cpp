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
#include <abb_libegm/egm_controller_interface.h>

int main(int argc, char** argv)
{
  bool getInitial;
  getInitial = false;
  //----------------------------------------------------------
  // Preparations
  //----------------------------------------------------------
  // Initialize the node.
  ros::init(argc, argv, "pose_controller_node");
  ros::NodeHandle node_handle;

  // Boost components for managing asynchronous UDP socket(s).
  boost::asio::io_service io_service;
  boost::thread_group thread_group;

  // Create an EGM interface:
  // * Sets up an EGM server (that the robot controller's EGM client can connect to).
  // * Provides APIs to the user (for setting motion references, that are sent in reply to the EGM client's request).
  //
  // Note: It is important to set the correct port number here,
  //       as well as configuring the settings for the EGM client in thre robot controller.
  //       If using the included RobotStudio Pack&Go file, then port 6511 = ROB_1, 6512 = ROB_2, etc.
  abb::egm::EGMControllerInterface egm_interface(io_service, 6510);

  if(!egm_interface.isInitialized())
  {
    ROS_ERROR("EGM interface failed to initialize (e.g. due to port already bound)");
    return 0;
  }

  // Spin up a thread to run the io_service.
  thread_group.create_thread(boost::bind(&boost::asio::io_service::run, &io_service));

  //----------------------------------------------------------
  // Execute a pose controller loop.
  //
  // Note: The EGM communication session is started by the
  //       EGMRunPose RAPID instruction.
  //----------------------------------------------------------
  ROS_INFO("========== Pose controller loop (open-loop) sample ==========");
  bool wait = true;
  abb::egm::wrapper::Input input;
  abb::egm::wrapper::CartesianPose initial_pose;
  abb::egm::wrapper::CartesianSpace current_CartesianS;
  abb::egm::wrapper::JointSpace current_jointS;
  // abb::egm::wrapper::CartesianPose current_joint;

  const int egm_rate = 250.0; // [Hz] (EGM communication rate, specified by the EGMActPose RAPID instruction).
  int sequence_number = 0;    // [-] (sequence number of a received EGM message).
  double time = 0.0;          // [seconds] (elapsed time during an EGM communication session).

  abb::egm::wrapper::Output output;
  double position_reference_z = 0.0;      // [mm].
  double position_reference_y = 0.0;      // [mm].
  double orientation_reference = 0.0;   // [degrees].
  double position_amplitude = 200.0;    // [mm].
  double orientation_amplitude = -10.0; // [degrees].
  double frequency = 0.25;              // [Hz].

  ROS_INFO("1: Wait for an EGM communication session to start...");
  while(ros::ok() && wait)
  {
    if(egm_interface.isConnected())
    {
      if(egm_interface.getStatus().rapid_execution_state() == abb::egm::wrapper::Status_RAPIDExecutionState_RAPID_UNDEFINED)
      {
        ROS_WARN("RAPID execution state is UNDEFINED (might happen first time after controller start/restart). Try to restart the RAPID program.");
      }
      else
      {
        wait = egm_interface.getStatus().rapid_execution_state() != abb::egm::wrapper::Status_RAPIDExecutionState_RAPID_RUNNING;
      }
    }

    ros::Duration(0.5).sleep();
  }

  while(ros::ok())
  {
    // Wait for a new EGM message from the EGM client (with a timeout of 500 ms).
    if(egm_interface.waitForMessage(500))
    {
      // Read the message received from the EGM client.
      egm_interface.read(&input);
      sequence_number = input.header().sequence_number();
      // ROS_INFO("sequence_number: %d", sequence_number);


      // if(sequence_number == 0)
      if( getInitial == false)
      {
        // Reset all references, if it is the first message.
        output.Clear();
        initial_pose.CopyFrom(input.feedback().robot().cartesian().pose());
        output.mutable_robot()->mutable_cartesian()->mutable_pose()->CopyFrom(initial_pose);
        // ROS_INFO_STREAM("initial configuration: " <<
        //          "Y position = " << initial_pose.position().y() << " [mm] | " <<
        //          "Y orientation (Euler) = " << initial_pose.euler().y() << " [degrees]");
                 
        
        getInitial = true;
      }
      else
      {
        time = sequence_number/((double) egm_rate);

        // Compute references for position (along X-axis), and orientation (around Y-axis).
        // position_reference = initial_pose.position().y() + position_amplitude*(0.0 + std::sin(2.0*M_PI*frequency*time - 0.5*M_PI));
        // orientation_reference = initial_pose.euler().y() + orientation_amplitude*(0.0 + std::sin(2.0*M_PI*frequency*time - 0.5*M_PI));
        position_reference_y = position_amplitude*(0.0 + std::sin(2.0*M_PI*frequency*time - 0.5*M_PI));
        position_reference_z = 850 + position_amplitude*(0.0 + std::cos(2.0*M_PI*frequency*time - 0.5*M_PI));

        // Set references.
        // Note: The references are relative to the frames specified by the EGMActPose RAPID instruction.
        output.mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_position()->set_y(position_reference_y);
        output.mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_position()->set_z(position_reference_z);
        // output.mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_euler()->set_y(orientation_reference);

        if(sequence_number%egm_rate == 0)
        {
          // ROS_INFO_STREAM("References: " <<
          //                 "X position = " << position_reference << " [mm] | " <<
          //                 "Y orientation (Euler) = " << orientation_reference << " [degrees]");


          current_CartesianS.CopyFrom(input.feedback().robot().cartesian());
          current_jointS.CopyFrom(input.feedback().robot().joints());
          output.mutable_robot()->mutable_cartesian()->CopyFrom(current_CartesianS);
          output.mutable_robot()->mutable_joints()->CopyFrom(current_jointS);

          ROS_INFO_STREAM("[ROBOT][CARTESINA SPACE][POSE]: " << 
                        "\n x:  " << current_CartesianS.pose().position().x() <<
                        "\n y:  " << current_CartesianS.pose().position().y() <<
                        "\n z:  " << current_CartesianS.pose().position().z() <<
                        "\n rx:  " << current_CartesianS.pose().euler().x() <<
                        "\n ry:  " << current_CartesianS.pose().euler().y() <<
                        "\n rz:  " << current_CartesianS.pose().euler().z() );

 
          ROS_INFO_STREAM("[ROBOT][CARTESINA SPACE][VELOCITY] " << 
                        "\n x:  " << current_CartesianS.velocity().linear().x() <<
                        "\n y:  " << current_CartesianS.velocity().linear().y() <<
                        "\n z:  " << current_CartesianS.velocity().linear().z() <<
                        "\n rx: " << current_CartesianS.velocity().angular().x() <<
                        "\n ry: " << current_CartesianS.velocity().angular().y() <<
                        "\n rz:   " << current_CartesianS.velocity().angular().z() );


          ROS_INFO_STREAM("[ROBOT][JOINT SPACE][POSITION][in degree]:" << 
                          "\n 1:  " << current_jointS.position().values(0) <<
                          "\n 2:  " << current_jointS.position().values(1) <<
                          "\n 3:  " << current_jointS.position().values(2) <<
                          "\n 4:  " << current_jointS.position().values(3) <<
                          "\n 5:  " << current_jointS.position().values(4) <<
                          "\n 6:  " << current_jointS.position().values(5) );
          
          ROS_INFO_STREAM("[ROBOT INFORMATION][JOINT SPACE][velocity]: " << 
                          "\n 1:  " << current_jointS.velocity().values(0) <<
                          "\n 2:  " << current_jointS.velocity().values(1) <<
                          "\n 3:  " << current_jointS.velocity().values(2) <<
                          "\n 4:  " << current_jointS.velocity().values(3) <<
                          "\n 5:  " << current_jointS.velocity().values(4) <<
                          "\n 6:  " << current_jointS.velocity().values(5));

        }

        
      }

      // Write references back to the EGM client.
      egm_interface.write(output);
    }
  }

  // Perform a clean shutdown.
  io_service.stop();
  thread_group.join_all();

  return 0;
}
