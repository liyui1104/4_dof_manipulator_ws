#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <controller_manager/controller_manager.h>
#include <std_msgs/Int16MultiArray.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>

#define steps 6400 // one circle --> 6400 pulses

class myarm_hardware_interface : public hardware_interface::RobotHW 
{
    public:
        myarm_hardware_interface(ros::NodeHandle& nh);
        ~myarm_hardware_interface();
        void init();
        void update(const ros::TimerEvent& e);
        void read();
        void write(ros::Duration elapsed_time);
        int caculate_pulse(const double radians);
        
    protected:
        // Depend your tpyes of interfaces
        hardware_interface::JointStateInterface joint_state_interface_;
        hardware_interface::PositionJointInterface position_joint_interface_;
        
        joint_limits_interface::JointLimits limits;
        joint_limits_interface::PositionJointSaturationInterface positionJointSaturationInterface;
        
        int num_joints_;
        std::string joint_names_[4];
        double joint_position_[4];
        double joint_velocity_[4];
        double joint_effort_[4];
        double joint_position_command_[4];
        
        ros::NodeHandle nh_;
        ros::Timer my_control_loop_;
        ros::Duration elapsed_time_;
        ros::Publisher cmd_pub;
        double loop_hz_;
        boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
};