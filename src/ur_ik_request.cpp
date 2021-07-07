#include "ros/ros.h"
#include "moveit_msgs/GetPositionIK.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"
#include <iostream>

geometry_msgs::PoseStamped end_effector_pos = geometry_msgs::PoseStamped();

class ik_request
{
private:
  sensor_msgs::JointState current_Joint;
  moveit_msgs::GetPositionIK _request;
  ros::NodeHandle _nh;
  ros::ServiceClient client;
public:
  ik_request(ros::NodeHandle nh)
  {
    current_Joint = sensor_msgs::JointState();
    _request.request.ik_request.robot_state.joint_state = current_Joint;
    _request.request.ik_request.group_name = "manipulator";
    
    _request.request.ik_request.pose_stamped = geometry_msgs::PoseStamped();
    _request.request.ik_request.pose_stamped.header.frame_id = "world";
    _nh = nh;
    client = _nh.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");
  }

  sensor_msgs::JointState IK_compute(geometry_msgs::PoseStamped end_effector_position)
  {
    _request.request.ik_request.pose_stamped = end_effector_position;
    _request.request.ik_request.pose_stamped.header.frame_id = "world";
    if (client.call(_request))
    {
      _request.request.ik_request.robot_state.joint_state = _request.response.solution.joint_state;
      std::cout << _request.response.error_code << std::endl;
    }
    return _request.request.ik_request.robot_state.joint_state;
  }

};




void eeCb(geometry_msgs::PoseStamped msg)
{
  end_effector_pos = msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ur5_ik");
  ros::NodeHandle _nh;

  ros::Publisher joint_pub = _nh.advertise<sensor_msgs::JointState>("query_joint_state", 1);
  ros::Subscriber sub = _nh.subscribe("unity/end_effector_goal", 1000, eeCb);
  ik_request ik_computation = ik_request(_nh);

  sensor_msgs::JointState joint_to_send = sensor_msgs::JointState();
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    joint_to_send = ik_computation.IK_compute(end_effector_pos);
    joint_pub.publish(joint_to_send);
    loop_rate.sleep();
    ros::spinOnce();
  }
  

  return 0;
}
