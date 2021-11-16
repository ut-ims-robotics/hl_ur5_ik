#include "ros/ros.h"
#include "moveit_msgs/GetPositionIK.h"
#include "moveit_msgs/GetMotionPlan.h"
#include "moveit_msgs/Constraints.h"
#include "moveit_msgs/JointConstraint.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"
#include <iostream>
#include "pluginlib/class_loader.h"


#include "moveit/robot_model_loader/robot_model_loader.h"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include <moveit/planning_scene/planning_scene.h>
#include "moveit/trajectory_execution_manager/trajectory_execution_manager.h"


geometry_msgs::PoseStamped end_effector_pos = geometry_msgs::PoseStamped();

static const std::string PLANNING_GROUP = "manipulator";

class ik_request
{
private:
  sensor_msgs::JointState _current_Joint;
  moveit_msgs::GetPositionIK _request_ik;
  moveit_msgs::GetMotionPlan _request_trajectory;
  moveit_msgs::JointConstraint _goalPosition;

  ros::NodeHandle _nh;
  ros::ServiceClient _client_ik;
  ros::ServiceClient _client_trajectory;
public:
  ik_request(ros::NodeHandle nh)
  {
    _current_Joint = sensor_msgs::JointState();
    _request_ik.request.ik_request.robot_state.joint_state = _current_Joint;
    _request_ik.request.ik_request.group_name = "manipulator";    
    _request_ik.request.ik_request.pose_stamped = geometry_msgs::PoseStamped();
    _request_ik.request.ik_request.pose_stamped.header.frame_id = "world";

    _request_trajectory.request.motion_plan_request.group_name = "manipulator";
    //_request_trajectory.request.motion_plan_request.start_state.joint_state = _current_Joint;

    _nh = nh;
    _client_ik = _nh.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");
    _client_trajectory = _nh.serviceClient<moveit_msgs::GetMotionPlan>("plan_kinematic_path");
    
  }

  sensor_msgs::JointState IK_compute(geometry_msgs::PoseStamped end_effector_position)
  {
    _request_ik.request.ik_request.pose_stamped = end_effector_position;
    _request_ik.request.ik_request.pose_stamped.header.frame_id = "world";
    if (_client_ik.call(_request_ik))
    {
      _request_ik.request.ik_request.robot_state.joint_state = _request_ik.response.solution.joint_state;
      std::cout << _request_ik.response.error_code << std::endl;
    }
    return _request_ik.request.ik_request.robot_state.joint_state;
  }

  void Trajectory_execute(sensor_msgs::JointState goalState)
  {
      //TODO
      /*
       * use /plan_kinematic_path service to create a plan(kinda done?) an then execute it.
       */


    int size_of_vector = goalState.name.size();
    
    /*
    name: 
  - shoulder_pan_joint
  - shoulder_lift_joint
  - elbow_joint
  - wrist_1_joint
  - wrist_2_joint
  - wrist_3_joint
     */
    moveit_msgs::Constraints goal;


    for (size_t i = 0; i < size_of_vector; i++)
    {
      _goalPosition.joint_name = goalState.name[i];
      _goalPosition.position = goalState.position[i];
      _goalPosition.weight = 1;
      goal.joint_constraints.push_back(_goalPosition);
    }
    


    _request_trajectory.request.motion_plan_request.goal_constraints.push_back(goal);

    if (_client_trajectory.call(_request_trajectory))
    {
      //make the push and execute of trajectory
    }
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
    if (end_effector_pos.header.stamp.sec == 1)
    {
      //TODO
      /*
      * function to use generated joint states to make a trajectory and execute
      */
    }
    else
    {
      joint_to_send = ik_computation.IK_compute(end_effector_pos);
      joint_pub.publish(joint_to_send);
    }
    loop_rate.sleep();
    ros::spinOnce();
  }
  

  return 0;
}
