#ifndef IMP_CONTROLLER_H
#define IMP_CONTROLLER_H

// Ros
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
// PluginLib
#include <pluginlib/class_list_macros.hpp>
// Ros control
#include <controller_interface/controller.h>
#include <controller_interface/multi_interface_controller.h>
// Hardware interfaces
#include <dls_hardware_interface/joint_command_adv_interface.h> // custom hw
// Eigen
#include <Eigen/Dense>


namespace ros_impedance_controller
{

class Controller : public controller_interface::MultiInterfaceController<hardware_interface::JointCommandAdvInterface>
{
public:
    /** @brief Constructor function */
    Controller();

    /** @brief Destructor function */
    ~Controller();

    /**
         * @brief Initializes sample controller
         * @param hardware_interface::RobotHW* robot hardware interface
         * @param ros::NodeHandle& Root node handle
         * @param ros::NodeHandle& Supervisor node handle
         */
    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);

    /**
         * @brief Starts the sample controller when controller manager request it
         * @param const ros::Time& time Time
         */
    void starting(const ros::Time& time);

    /**
         * @brief Updates the sample controller according to the control
         * frequency (task frequency)
         * @param const ros::Time& time Time
         * @param const ros::Duration& Period
         */
    void update(const ros::Time& time, const ros::Duration& period);

    /**
         * @brief Stops the sample controller when controller manager request it
         * @param const ros::time& Time
         */
    void stopping(const ros::Time& time);

private:

    void commandCallback(const sensor_msgs::JointState &msg);

    ros::Subscriber sub_;

    /** @brief Number of joints */
    unsigned int num_joints_;
    /** @brief Joint names */
    std::vector<std::string> joint_names_;
    /** @brief Joint states for input and output */
    std::vector<hardware_interface::JointCommandAdvHandle> joint_states_;
    /** @brief Desired joint efforts */
    Eigen::VectorXd des_joint_efforts_;
    /** @brief Desired joint positions */
    Eigen::VectorXd des_joint_positions_;
    /** @brief Desired joint velocities */
    Eigen::VectorXd des_joint_velocities_;
    /** @brief Actual P value for the joints PID controller */
    std::vector<double> joint_p_gain_;
    /** @brief Actual I value for the joints PID controller */
    std::vector<double> joint_i_gain_;
    /** @brief Actual D value for the joints PID controller */
    std::vector<double> joint_d_gain_;


};


PLUGINLIB_EXPORT_CLASS(ros_impedance_controller::Controller, controller_interface::ControllerBase);

} //@namespace ros_impedance_controller

#endif //IMP_CONTROLLER_H
