/**
 * @file plot_node.cpp
 * @author Gennaro Raiola, Michele Focchi
 * @date 12 June, 2019
 * @brief plot node.
 */

#include <stdio.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/sensors/ContactSensor.hh>
#include <tf/transform_listener.h>

namespace ros_impedance_controller
{

/** @brief Visual tools */
rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

ros::Subscriber subscriber_;
//void callbackContactForces(const gazebo_msgs::ContactsState &msg);
unsigned int decimate = 10;
unsigned long long cnt = 0;
Eigen::Affine3d pose;
Eigen::Vector3d vector;
double norm;
Eigen::Matrix3d R;
std::vector<std::shared_ptr<gazebo::sensors::ContactSensor> > foot_sensors_;
tf::StampedTransform lf_transform;
tf::StampedTransform rf_transform;
tf::StampedTransform lh_transform;
tf::StampedTransform rh_transform;

bool init(ros::NodeHandle& nh)
{

    ROS_INFO("Init");


    //foot switch
    foot_sensors_.resize(4);
    std::vector<std::string> foot_sensor_names(4);
    foot_sensor_names[0] = std::string("lf_foot_contact_sensor");
    foot_sensor_names[1] = std::string("rf_foot_contact_sensor");
    foot_sensor_names[2] = std::string("lh_foot_contact_sensor");
    foot_sensor_names[3] = std::string("rh_foot_contact_sensor");


    for (int n = 0; n < foot_sensors_.size(); n++) {
        foot_sensors_[n] = std::dynamic_pointer_cast<gazebo::sensors::ContactSensor>
            (gazebo::sensors::SensorManager::Instance()->GetSensor(foot_sensor_names[n]));
        if (!foot_sensors_[n]) 	{
            ROS_ERROR_STREAM("Could not find foot sensor \"" << foot_sensor_names[n] << "\".");
        }
    }

    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("world","grfs"));

    return true;
}

void createArrow(const geometry_msgs::Vector3& force, const geometry_msgs::Vector3& position,  rviz_visual_tools::colors color)
{
    vector(0) = force.x;
    vector(1) = force.y;
    vector(2) = force.z;
    norm = vector.norm();

    //find rotation matrix to align 1 0  0 to force direction
    R = Eigen::Quaterniond().setFromTwoVectors(Eigen::Vector3d::UnitX(),vector/norm).toRotationMatrix();
    pose.linear() = R;
    pose.translation().x() = position.x;
    pose.translation().y() = position.y;
    pose.translation().z() = position.z;

    visual_tools_->publishArrow(pose, color, rviz_visual_tools::LARGE, norm/500.0);
    visual_tools_->trigger();
}

//void callbackContactForces(const gazebo_msgs::ContactsState& msg)
//{
//    if(cnt++%decimate==0)
//    {
//        visual_tools_->deleteAllMarkers();

////        string info                                   # text info on this contact
////        string collision1_name                        # name of contact collision1
////        string collision2_name                        # name of contact collision2
////        geometry_msgs/Wrench[] wrenches               # list of forces/torques
////        geometry_msgs/Wrench total_wrench             # sum of forces/torques in every DOF
////        geometry_msgs/Vector3[] contact_positions     # list of contact position
////        geometry_msgs/Vector3[] contact_normals       # list of contact normals
////        float64[] depths                              # list of penetration depths
////msg.states[0].wrenches[0].force.x
////        gazebo_msgs::ContactState leg_contact;
////        //Display an arrow along the x-axis of a pose.
////        for(unsigned int i=0; i<msg.msg.state[i].size();i++)
////        {
////                leg_contact = msg.state[i];
////                Vector
////                createArrow(leg_contact.wrenches[0].force.x,leg_contact.wrenches[0].force.y,rviz_visual_tools::BLUE);
////                createArrow(msg.contact_forces[i].force,msg.contact_positions[i],rviz_visual_tools::GREEN);
////        }
////    }

//}

void listen_tf(tf::TransformListener & listener)
{
    try{

        listener.lookupTransform("/world", "/lf_lowerleg", ros::Time(0), lf_transform);
        listener.lookupTransform("/world", "/rf_lowerleg", ros::Time(0), rf_transform);
        listener.lookupTransform("/world", "/lh_lowerleg", ros::Time(0), lh_transform);
        listener.lookupTransform("/world", "/rh_lowerleg", ros::Time(0), rh_transform);
        tf::Vector3 origin =  lf_transform.getOrigin();
        std::cout<<"Transform X "<<origin.x()<<" Y "<< origin.y()<<" Z "<<origin.z()<<std::endl;
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
}

void getContactSensors()
{
    //  //get virtual foot switch
    //  for (int n = 0; n < foot_sensors_.size(); n++) {
    //      gazebo::msgs::Contacts contacts;
    //      contacts = foot_sensors_[n]->Contacts();
    //      //std::cout << "Sensor " << n << " Contacts " <<  contacts.contact_size() << std::endl;
    //      if (contacts.contact_size()>=1)
    //      {

    //          //the wrench is in the last link where the foot is lumped that is the lowerleg! so it is expressed in the lowerleg
    //          //map from lowerleg frame to world
    //          gazebo::math::Pose link_pose = lowerleg_link[n]->GetWorldPose();
    //          gazebo::math::Vector3 forceW = link_pose.rot.RotateVector(
    //                                        gazebo::math::Vector3( contacts.contact(0).wrench(0).body_1_wrench().force().x(),
    //                                                              contacts.contact(0).wrench(0).body_1_wrench().force().y(),
    //                                                                  contacts.contact(0).wrench(0).body_1_wrench().force().z()));


    //          //these forces are in the world frame!
    //          force_[n][0] = forceW.x;
    //          force_[n][1] = forceW.y;
    //          force_[n][2] = forceW.z;
              //std::cout<<"force :"<<force_[n][0]<<"  "<< force_[n][1]<<"  "<< force_[n][2] <<std::endl;

    //          //the noraml is already in the world!!!
    //          normal_[n][0]  = contacts.contact(0).normal(0).x();
    //          normal_[n][1]  = contacts.contact(0).normal(0).y();
    //          normal_[n][2]  = contacts.contact(0).normal(0).z();
              //std::cout<<"normal :"<<normal.transpose()<<std::endl;

     //          force_[n][0]=0.0;
    //          force_[n][1]=0.0;
    //          force_[n][2]=0.0;
    //      }


    //  }
}

} // namespace

int main(int argc, char **argv)
{

  ros::init(argc, argv, "ros_impedance_controller");
  //put the node hanlde inside the specific plot_node name space rather than just the root "robot_name"
  //name space specified in the launch file

  ros::NodeHandle node("ros_impedance_controller");
  ros_impedance_controller::init(node);
  ros::Rate rate(10.0);

  //you need to instantiate this here after creating the node otherwise segfault
  tf::TransformListener listener;
  while (node.ok())
  {
      ros_impedance_controller::listen_tf(listener);
      ros_impedance_controller::getContactSensors();
      rate.sleep();
  }
  ROS_INFO("Ready to plot");
  ros::spin();

  return 0;
}


