#include "myarm_hardware_interface/myarm_hardware_interface.h"

myarm_hardware_interface::myarm_hardware_interface(ros::NodeHandle& nh) : nh_(nh) {

    // Declare all JointHandles, JointInterfaces and JointLimitInterfaces of the robot.
    init();
    
    // Create the controller manager
    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    
    // Set the frequency of the control loop.
    loop_hz_ = 10;
    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
    
    // Run the control loop
    my_control_loop_ = nh_.createTimer(update_freq, &myarm_hardware_interface::update, this);

    this->cmd_pub = nh_.advertise<std_msgs::Int16MultiArray>("/lower_controller/command", 10);
}

myarm_hardware_interface::~myarm_hardware_interface() {
}

void myarm_hardware_interface::init() {

  num_joints_ = 4;
	joint_names_[0]="joint1";	
	joint_names_[1]="joint2";
	joint_names_[2]="joint3";
  joint_names_[3]="joint4";

  for (int i = 0; i < num_joints_; ++i)
  {

    // Create joint state interface
    hardware_interface::JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
    joint_state_interface_.registerHandle(jointStateHandle);

    // Create position joint interface
    hardware_interface::JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]);

    position_joint_interface_.registerHandle(jointPositionHandle);
  }
  registerInterface(&joint_state_interface_);
  registerInterface(&position_joint_interface_);
}

// This is the control loop
void myarm_hardware_interface::update(const ros::TimerEvent &e)
{
  elapsed_time_ = ros::Duration(e.current_real - e.last_real);
  read();
  controller_manager_->update(ros::Time::now(), elapsed_time_);
  write(elapsed_time_);
}

void myarm_hardware_interface::read() {
  // Write the protocol (I2C/CAN/ros_serial/ros_industrial)used to get the current joint position and/or velocity and/or effort       

  // from robot.
  // and fill JointStateHandle variables joint_position_[i], joint_velocity_[i] and joint_effort_[i]

  // We don't have feedback from the low level controller(Arduino), so we assume the command is executed ideally
  for (int i = 0; i < num_joints_; i++)
  {
    joint_position_[i] = joint_position_command_[i];
  }
  
}

int myarm_hardware_interface::caculate_pulse(const double radians)
{
  return (steps / (2 * M_PI)) * radians;
}

void myarm_hardware_interface::write(ros::Duration elapsed_time) {
  // Safety
  positionJointSaturationInterface.enforceLimits(elapsed_time); // enforce limits for joint1 joint2 joint3 joint4

  // Write the protocol (I2C/CAN/ros_serial/ros_industrial)used to send the commands to the robot's actuators.
  // The output commands need to send joint_position_command_[0] for joint1, joint_position_command_[1] for joint2 ...
  
  // Publish the command to the low level controller through ros_serial
  std_msgs::Int16MultiArray cmd;
  cmd.data.clear();
  for (int i = 0; i < 4; i++)
  {
    cmd.data.push_back(caculate_pulse(joint_position_command_[i] - joint_position_[i]));
  }
  
  // transmission ratio is 2:1
  cmd.data[1] = cmd.data[1] * 2;
  cmd.data[2] = cmd.data[2] * 2;
  cmd.data[3] = cmd.data[3] * 2;

  cmd_pub.publish(cmd);
}

int main(int argc, char **argv)
{

  // Initialze the ROS node.
  ros::init(argc, argv, "myarm_hardware_interface_node");
  ros::NodeHandle nh;

  // Separate Sinner thread for the Non-Real time callbacks such as service callbacks to load controllers
  ros::MultiThreadedSpinner spinner(2);

  // Create the object of the robot hardware_interface class and spin the thread.
  myarm_hardware_interface myarm(nh);
  spinner.spin();

  return 0;
}