/**
 * @file controller.cpp
 * @author Gennaro Raiola, Michele Focchi
 * @date 12 June, 2018
 * @brief Ros impedance controller.
 */

#include <ros_impedance_controller/controller.h>


namespace ros_impedance_controller {

const std::string red("\033[0;31m");
const std::string green("\033[1;32m");
const std::string yellow("\033[1;33m");
const std::string cyan("\033[0;36m");
const std::string magenta("\033[0;35m");
const std::string reset("\033[0m");

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
    std::cout<< red<< "Initialize Ros Impedance Controller framework independent" << reset <<std::endl;
    root_nh_ = &root_nh;
    assert(robot_hw);

//TODO
//    std::string srdf, urdf;
//    if(!controller_nh.getParam("/robot_description",urdf))
//    {
//        ROS_ERROR_NAMED(CLASS_NAME,"robot_description not available in the ros param server");

//    }
//    if(!controller_nh.getParam("/robot_semantic_description",srdf))
//    {
//        ROS_ERROR_NAMED(CLASS_NAME,"robot_description_semantic not available in the ros param server");

//    }


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
    } else
    {
         std::cout<< green<< "Found  " <<joint_names_.size()<< " joints"<< reset <<std::endl;
    }

     // Setting up handles:
    for ( int i = 0; i < joint_names_.size(); i++)
    {

        // Getting joint state handle
        try
        {
            std::cout<< green<< "Loading effort interface for joint " <<joint_names_[i]<< reset <<std::endl;
            joint_states_.push_back(eff_hw->getHandle(joint_names_[i]));

        }
        catch(...)
        {
            ROS_ERROR("Error loading the effort interfaces");
            return false;
        }
    }
    assert(joint_states_.size()>0);


    // Resize the variables
    des_joint_positions_.resize(joint_states_.size());
    des_joint_positions_.fill(0.0);
    des_joint_velocities_.resize(joint_states_.size());
    des_joint_velocities_.fill(0.0);
    des_joint_efforts_.resize(joint_states_.size());
    des_joint_efforts_.fill(0.0);
    des_joint_efforts_pids_.resize(joint_states_.size());
    des_joint_efforts_.fill(0.0);


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
       ROS_DEBUG("P value for joint %i is: %f",i,joint_p_gain_[i]);
       ROS_DEBUG("I value for joint %i is: %f",i,joint_i_gain_[i]);
       ROS_DEBUG("D value for joint %i is: %f",i,joint_d_gain_[i]);

      //get statrup go0 position from yaml (TODO srdf)
       controller_nh.getParam("home/" + joint_names_[i], des_joint_positions_[i]);

    }


//TODO
//    srdf::Model s;
//    urdf::ModelInterfaceSharedPtr u = urdf::parseURDF(urdf);
//    s.initString(*u,srdf);
//    auto group_states = s.getGroupStates();

//    for(unsigned int i=0;i<group_states.size();i++)
//        if(group_states[i].name_ == "home") // look for the home group state and get the names of the joints in there
//            for(auto & tmp : group_states[i].joint_values_)
//                des_joint_positions_.push_back(tmp.second);




//contact sensor TODO
    //foot switch
//    foot_sensors_.resize(4);
//    std::vector<std::string> foot_sensor_names(4);
//    foot_sensor_names[0] = std::string("lf_foot_contact_sensor");
//    foot_sensor_names[1] = std::string("rf_foot_contact_sensor");
//    foot_sensor_names[2] = std::string("lh_foot_contact_sensor");
//    foot_sensor_names[3] = std::string("rh_foot_contact_sensor");
//    for (int n = 0; n < foot_sensors_.size(); n++) {
//        foot_sensors_[n] = std::dynamic_pointer_cast<gazebo::sensors::ContactSensor>
//                (gazebo::sensors::SensorManager::Instance()->GetSensor(foot_sensor_names[n]));
//        if (!this->foot_sensors_[n]) 	{
//            ROS_ERROR_STREAM("Could not find foot sensor \"" << foot_sensor_names[n] << "\".");
//        }
//    }

    // Create the subscriber
    sub_ = root_nh.subscribe("/command", 1, &Controller::commandCallback, this);

    //subscriber to the ground truth
    std::string robot_name = "hyq";
    ros::NodeHandle param_node;
    param_node.getParam("/robot_name", robot_name);

    std::cout<< red<< "ROBOT NAME IS : "<< robot_name<<reset <<std::endl;
     // Create the PID set service
    set_pids_srv_ = param_node.advertiseService("/set_pids", &Controller::setPidsCallback, this);


    gt_sub_ = param_node.subscribe("/"+robot_name + "/ground_truth", 1, &Controller::baseGroundTruthCB, this, ros::TransportHints().tcpNoDelay());



    //rt publisher (uncomment if you need them)
    //pose_pub_rt_.reset(new realtime_tools::RealtimePublisher<BaseState>(param_node, "/"+robot_name + "/base_state", 1));
    //contact_state_pub_rt_.reset(new realtime_tools::RealtimePublisher<gazebo_msgs::ContactsState>(param_node, "/"+robot_name + "/contacts_state", 1));


    return true;
}


void Controller::starting(const ros::Time& time)
{
    ROS_DEBUG("Starting Controller");

}



bool Controller::setPidsCallback(set_pids::Request& req,
                                 set_pids::Response& res)
{
    //get params from parameter server
    root_nh_->getParam("/verbose", verbose);
    res.ack = true;

    for(unsigned int i = 0; i < req.data.size(); i++)
    {
        for(unsigned int j = 0; j < joint_names_.size(); j++)
            if(!std::strcmp(joint_names_[j].c_str(),req.data[i].joint_name.c_str()))
            {
                if(req.data[i].p_value>=0.0)
                {
                    joint_p_gain_[j] = req.data[i].p_value;
                    if (verbose)
                        std::cout<<"Set P gain for joint "<< joint_names_[j] << " to: "<<joint_p_gain_[j]<<std::endl;
                }
                else
                {
                   ROS_WARN("P value has to be positive");
                   res.ack = false;
                }

                if(req.data[i].i_value>=0.0)
                {
                    joint_i_gain_[j] = req.data[i].i_value;
                    if (verbose)
                       std::cout<<"Set I gain for joint "<< joint_names_[j] << " to: "<<joint_i_gain_[j]<<std::endl;
                }
                else
                {
                   ROS_WARN("I value has to be positive");
                   res.ack = false;
                }

                if(req.data[i].d_value>=0.0)
                {
                    joint_d_gain_[j] = req.data[i].d_value;
                    if (verbose)
                       std::cout<<"Set D gain for joint "<< joint_names_[j] << " to: "<<joint_d_gain_[j]<<std::endl;
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
    tf::Transform w_transform_b;

    //orientation of base frame
    q_base.setX(msg->pose.pose.orientation.x);
    q_base.setY(msg->pose.pose.orientation.y);
    q_base.setZ(msg->pose.pose.orientation.z);
    q_base.setW(msg->pose.pose.orientation.w);
    //position of base frame
    base_pos_w = tf::Vector3(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);

    //get twist
    base_twist_w.linear.x= msg->twist.twist.linear.x;
    base_twist_w.linear.y = msg->twist.twist.linear.y;
    base_twist_w.linear.z = msg->twist.twist.linear.z;
    base_twist_w.angular.x = msg->twist.twist.angular.x;
    base_twist_w.angular.y = msg->twist.twist.angular.y;
    base_twist_w.angular.z = msg->twist.twist.angular.z;


    //the vector of the base is in the world frame, so to apply to the base frame I should rotate it to the base frame before
    tf::Vector3 world_origin_w(-msg->pose.pose.position.x,-msg->pose.pose.position.y,-msg->pose.pose.position.z);
    tf::Vector3 world_origin_b = tf::quatRotate(q_base.inverse(), world_origin_w);

    //this is the transform from base to world to publish the world transform for rviz
    w_transform_b.setRotation(q_base.inverse());
    w_transform_b.setOrigin(world_origin_b);
    br.sendTransform(tf::StampedTransform(w_transform_b, ros::Time::now(), "/base_link", "/world" ));
}

void Controller::update(const ros::Time& time, const ros::Duration& period)
{

    //if task_period is smaller than sim max_step_size (in world file) period it is clamped to that value!!!!!
    //std::cout<<period.toSec()<<std::endl;
//    std::cout<<"des_joint_efforts_: " << des_joint_efforts_.transpose()<<std::endl;
//    std::cout<<"des_joint_velocities_: " << des_joint_velocities_.transpose()<<std::endl;
//    std::cout<<"des_joint_positions_: " << des_joint_positions_.transpose()<<std::endl;


    // Write to the hardware interface
    //(NB this is not the convention
    //of ros but the convention of robcogen  that we define in ros_impedance_controller_XX.yaml!!!!
    for (unsigned int i = 0; i < joint_states_.size(); i++)
    {      
        //compute PID
        des_joint_efforts_pids_(i) = joint_p_gain_[i]*(des_joint_positions_(i)-joint_states_[i].getPosition()) +
                                     joint_d_gain_[i]*(des_joint_velocities_(i)-joint_states_[i].getVelocity());
        //add PID + FFWD
        joint_states_[i].setCommand(des_joint_efforts_(i) +  des_joint_efforts_pids_(i));
        //std::cout<<"DEBUG: joint effort joint " <<i <<"   " <<joint_states_[i].getEffort()<<std::endl;
    }


      //publish hyq pose for the mapper node
      //uncomment if you want to use BaseState message instead than /solo/groundtruth in python
//      BaseState pose_msg;
//      pose_msg.header.stamp =ros::Time::now();
//      pose_msg.header.frame_id = "world";
//      pose_msg.pose.position.x = base_pos_w.x();
//      pose_msg.pose.position.y = base_pos_w.y();
//      pose_msg.pose.position.z = base_pos_w.z();
//      pose_msg.pose.orientation.w = q_base.w();
//      pose_msg.pose.orientation.x = q_base.x();
//      pose_msg.pose.orientation.y = q_base.y();
//      pose_msg.pose.orientation.z = q_base.z();

//      pose_msg.twist.linear.x = base_twist_w.linear.x;
//      pose_msg.twist.linear.y = base_twist_w.linear.y;
//      pose_msg.twist.linear.z = base_twist_w.linear.z;
//      pose_msg.twist.angular.x = base_twist_w.angular.x;
//      pose_msg.twist.angular.y = base_twist_w.angular.y;
//      pose_msg.twist.angular.z = base_twist_w.angular.z;


//      if (pose_pub_rt_->trylock()){
//        pose_pub_rt_->msg_ = pose_msg;
//        pose_pub_rt_->unlockAndPublish();
//      }

    //get virtual foot switch

//    std::vector<gazebo::physics::LinkPtr> lowerleg_link(4);
//    lowerleg_link[0] = sim_model_->GetLink("lf_lowerleg");
//    lowerleg_link[1] = sim_model_->GetLink("rf_lowerleg");
//    lowerleg_link[2] = sim_model_->GetLink("lh_lowerleg");
//    lowerleg_link[3] = sim_model_->GetLink("rh_lowerleg");

//     for (int n = 0; n < foot_sensors_.size(); n++) {
//            gazebo::msgs::Contacts contacts;
//            contacts = foot_sensors_[n]->Contacts();

//        //the wrench is in the last link where the foot is lumped that is the lowerleg! so it is expressed in the lowerleg
//        //map from lowerleg frame to world
//        gazebo::math::Pose link_pose = lowerleg_link[n]->GetWorldPose();
//        gazebo::math::Vector3 forceW = link_pose.rot.RotateVector(
//                                      gazebo::math::Vector3( contacts.contact(0).wrench(0).body_1_wrench().force().x(),
//                                                            contacts.contact(0).wrench(0).body_1_wrench().force().y(),
//                                                                contacts.contact(0).wrench(0).body_1_wrench().force().z()));


//        //these forces are in the world frame!
//        force_[n][0] = forceW.x;
//        force_[n][1] = forceW.y;
//        force_[n][2] = forceW.z;
//        //std::cout<<"force :"<<force_[n][0]<<"  "<< force_[n][1]<<"  "<< force_[n][2] <<std::endl;

//        //the noraml is already in the world!!!
//        normal_[n][0]  = contacts.contact(0).normal(0).x();
//        normal_[n][1]  = contacts.contact(0).normal(0).y();
//        normal_[n][2]  = contacts.contact(0).normal(0).z();
//        //std::cout<<"normal :"<<normal.transpose()<<std::endl;
//    } else {
//        contact_[n] = false;
//        force_[n][0]=0.0;
//        force_[n][1]=0.0;
//        force_[n][2]=0.0;}
//    }

//    //publish contactstate
//    std::vector<gazebo_msgs::ContactState> contacts_state(4);
//    std::vector<geometry_msgs::Wrench> tmp_wrench(1);
//    std::vector<geometry_msgs::Vector3> tmp_normal(1);
//    std::vector<double> tmp_contact_bool(1);
//    contacts_state[0].info = "LF";
//    contacts_state[1].info = "RF";
//    contacts_state[2].info = "LH";
//    contacts_state[3].info = "RH";

//    for (int leg = 0; leg < 4; leg++)
//    {
//            tmp_wrench[0].force.x = force_[leg][0];
//            tmp_wrench[0].force.y = force_[leg][1];
//            tmp_wrench[0].force.z = force_[leg][2];
//            contacts_state[leg].wrenches = tmp_wrench;

//            tmp_normal[0].x = normal_[leg][0];
//            tmp_normal[0].y = normal_[leg][1];
//            tmp_normal[0].z = normal_[leg][2];
//            contacts_state[leg].contact_normals = tmp_normal;

//            tmp_contact_bool[0] = (double)contact_[leg];
//            contacts_state[leg].depths = tmp_contact_bool;
//     }

//        contact_state_pub_->msg_.states = contacts_state;
//        contact_state_pub_->msg_.header.stamp = ros::Time::now();
//
    std::vector<gazebo_msgs::ContactState> contacts_message(4);
    gazebo_msgs::ContactsState contact_msg;
    contact_msg.states = contacts_message;
    contact_msg.header.stamp = ros::Time::now();
    //uncomment if you want to subscribe to the ground truth in python receivecontact
//    if (contact_state_pub_rt_->trylock()){
//      contact_state_pub_rt_->msg_ = contact_msg;
//      contact_state_pub_rt_->unlockAndPublish();
//    }
}



void Controller::stopping(const ros::Time& time)
{
    ROS_DEBUG("Stopping Controller");
}

} //namespace
