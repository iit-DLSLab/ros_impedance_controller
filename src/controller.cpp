/**
 * @file controller.cpp
 * @author Gennaro Raiola, Michele Focchi
 * @date 12 June, 2018
 * @brief Ros impedance controller.
 */

#include <ros_impedance_controller/controller.h>


namespace ros_impedance_controller {


Controller::Controller()
{
}

Controller::~Controller()
{

}
   
bool Controller::init(hardware_interface::RobotHW* robot_hw,
                      ros::NodeHandle& root_nh,
                      ros::NodeHandle& controller_nh)
{
    // getting the names of the joints from the ROS parameter server
    ROS_DEBUG("Initialize Controller");

    assert(robot_hw);


    hardware_interface::EffortJointInterface* eff_hw = robot_hw->get<hardware_interface::EffortJointInterface>();

    if(!eff_hw)
    {
        ROS_ERROR("hardware_interface::EffortJointInterface not found");
        return false;
    }
    
    if (!controller_nh.getParam("joints", joint_names_))
    {
        ROS_ERROR("No joints given in the namespace: %s.", controller_nh.getNamespace().c_str());
        return false;
    }

    // Setting up handles:
    for (unsigned int i = 0; i < joint_names_.size(); i++)
    {
        // Getting joint state handle
        try
        {
            ROS_DEBUG_STREAM("Found joint: "<<joint_names_[i]);
            joint_states_.push_back(eff_hw->getHandle(joint_names_[i]));
        }
        catch(...)
        {
            ROS_ERROR("Error loading joint_states_");
            return false;
        }
    }
    assert(joint_states_.size()>0);

    joint_p_gain_.resize(joint_states_.size());
    joint_i_gain_.resize(joint_states_.size());
    joint_d_gain_.resize(joint_states_.size());
    for (unsigned int i = 0; i < joint_states_.size(); i++)
    {
       // Getting PID gains
       if (!controller_nh.getParam("gains/" + joint_names_[i] + "/p", joint_p_gain_[i]))
       {
           ROS_ERROR("No P gain given in the namespace: %s. ", controller_nh.getNamespace().c_str());
           return false;
       }
       if (!controller_nh.getParam("gains/" + joint_names_[i] + "/i", joint_i_gain_[i]))
       {
           ROS_ERROR("No D gain given in the namespace: %s. ", controller_nh.getNamespace().c_str());
           return false;
       }
       if (!controller_nh.getParam("gains/" + joint_names_[i] + "/d", joint_d_gain_[i]))
       {
           ROS_ERROR("No I gain given in the namespace: %s. ", controller_nh.getNamespace().c_str());
           return false;
       }
       // Check if the values are positive
       if(joint_p_gain_[i]<0.0 || joint_i_gain_[i]<0.0 || joint_d_gain_[i]<0.0)
       {
           ROS_ERROR("PID gains must be positive!");
           return false;
       }
       ROS_DEBUG("P value for joint %i is: %d",i,joint_p_gain_[i]);
       ROS_DEBUG("I value for joint %i is: %d",i,joint_i_gain_[i]);
       ROS_DEBUG("D value for joint %i is: %d",i,joint_d_gain_[i]);

    }

    // Resize the variables
    des_joint_positions_.resize(joint_states_.size());
    des_joint_positions_.fill(0.0);
    des_joint_velocities_.resize(joint_states_.size());
    des_joint_velocities_.fill(0.0);
    des_joint_efforts_.resize(joint_states_.size());
    des_joint_efforts_.fill(0.0);

    // Create the subscriber
    sub_ = controller_nh.subscribe("command", 1, &Controller::commandCallback, this);

    //subscriber to the ground truth
    std::string robot_name = "hyq";
    controller_nh.getParam("robot_name", robot_name);


    gt_sub_ = controller_nh.subscribe("/"+robot_name + "/ground_truth", 100, &Controller::baseGroundTruthCB, this);


    // Create the PID set service
    set_pids_srv_ = controller_nh.advertiseService("set_pids", &Controller::setPidsCallback, this);



    return true;
}


void Controller::starting(const ros::Time& time)
{
    ROS_DEBUG("Starting Controller");
    des_joint_positions_.fill(0.0);
    des_joint_velocities_.fill(0.0);
    des_joint_efforts_.fill(0.0);
}



bool Controller::setPidsCallback(set_pids::Request& req,
                                 set_pids::Response& res)
{
    res.ack = true;

    for(unsigned int i = 0; i < req.data.size(); i++)
    {
        for(unsigned int j = 0; j < joint_names_.size(); j++)
            if(!std::strcmp(joint_names_[j].c_str(),req.data[i].joint_name.c_str()))
            {
                if(req.data[i].p_value>=0.0)
                {
                    joint_p_gain_[j] = req.data[i].p_value;
                    ROS_INFO_STREAM("Set P gain for joint "<< joint_names_[j] << " to: "<<joint_p_gain_[j]);
                }
                else
                {
                   ROS_WARN("P value has to be positive");
                   res.ack = false;
                }

                if(req.data[i].i_value>=0.0)
                {
                    joint_i_gain_[j] = req.data[i].i_value;
                    ROS_INFO_STREAM("Set I gain for joint "<< joint_names_[j] << "to: "<<joint_i_gain_[j]);
                }
                else
                {
                   ROS_WARN("I value has to be positive");
                   res.ack = false;
                }

                if(req.data[i].d_value>=0.0)
                {
                    joint_d_gain_[j] = req.data[i].d_value;
                    ROS_INFO_STREAM("Set D gain for joint "<< joint_names_[j] << "to: "<<joint_d_gain_[j]);
                }
                else
                {
                   ROS_WARN("D value has to be positive");
                   res.ack = false;
                }
            }

    }

    return true;
}

void Controller::commandCallback(const sensor_msgs::JointState& msg)
{

    if(joint_states_.size() == msg.position.size() && joint_states_.size() == msg.velocity.size() && joint_states_.size() == msg.effort.size())
    {
            //des_joint_efforts_(i) = msg.data[i];
            des_joint_positions_ = Eigen::Map<const Eigen::VectorXd>(&msg.position[0],joint_states_.size());
            des_joint_velocities_ = Eigen::Map<const Eigen::VectorXd>(&msg.velocity[0],joint_states_.size());
            des_joint_efforts_ = Eigen::Map<const Eigen::VectorXd>(&msg.effort[0],joint_states_.size());
    }

    else
        ROS_WARN("Wrong dimension!");
}


void Controller::baseGroundTruthCB(const nav_msgs::OdometryConstPtr &msg)
{

    static tf::TransformBroadcaster br;
    tf::Transform b_transform_w;


    //orientation of base frame
    q_base.setX(msg->pose.pose.orientation.x);
    q_base.setY(msg->pose.pose.orientation.y);
    q_base.setZ(msg->pose.pose.orientation.z);
    q_base.setW(msg->pose.pose.orientation.w);
    //position of base frame
    base_pos_w = tf::Vector3(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);


    //the vector of the base is in the world frame, so to apply to the base frame I should rotate it to the base frame before
    tf::Vector3 world_origin_w(-msg->pose.pose.position.x,-msg->pose.pose.position.y,-msg->pose.pose.position.z);
    tf::Vector3 world_origin_b = tf::quatRotate(q_base.inverse(), world_origin_w);

    b_transform_w.setRotation(q_base.inverse());
    b_transform_w.setOrigin(world_origin_b);
    br.sendTransform(tf::StampedTransform(b_transform_w, ros::Time::now(), "/base_link", "/world" ));
}

void Controller::update(const ros::Time& time, const ros::Duration& period)
{
    ROS_DEBUG_STREAM("des_joint_efforts_: " << des_joint_efforts_.transpose());
    ROS_DEBUG_STREAM("des_joint_velocities_: " << des_joint_velocities_.transpose());
    ROS_DEBUG_STREAM("des_joint_positions_: " << des_joint_positions_.transpose());

    // Write to the hardware interface
    for (unsigned int i = 0; i < joint_states_.size(); i++)
    {      
        joint_states_[i].setCommand(des_joint_efforts_(i));
    }


    //publish hyq pose for the mapper node
    geometry_msgs::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp =ros::Time::now();
    pose_msg.header.frame_id = "world";
    pose_msg.pose.pose.position.x = base_pos_w.x();
    pose_msg.pose.pose.position.y = base_pos_w.y();
    pose_msg.pose.pose.position.z = base_pos_w.z();
    pose_msg.pose.pose.orientation.w = q_base.w();
    pose_msg.pose.pose.orientation.x = q_base.x();
    pose_msg.pose.pose.orientation.y = q_base.y();
    pose_msg.pose.pose.orientation.z = q_base.z();
    pose_pub_.publish(pose_msg);

}



void Controller::stopping(const ros::Time& time)
{
    ROS_DEBUG("Stopping Controller");
}

} //namespace
