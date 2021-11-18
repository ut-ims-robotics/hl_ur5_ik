#include "ros/ros.h"
#include "moveit_msgs/GetPositionIK.h"
#include "moveit_msgs/GetMotionPlan.h"
#include "moveit_msgs/Constraints.h"
#include "moveit_msgs/JointConstraint.h"
#include "moveit_msgs/RobotTrajectory.h"
#include "moveit_msgs/PlanningScene.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"

#include <iostream>
#include "pluginlib/class_loader.h"

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>


#include <moveit/move_group_interface/move_group_interface.h>

#include <boost/scoped_ptr.hpp>

geometry_msgs::PoseStamped end_effector_pos = geometry_msgs::PoseStamped();

planning_interface::MotionPlanRequest req;
planning_interface::MotionPlanResponse res;



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
    //_current_Joint.name = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
    //_current_Joint.position = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    _request_ik.request.ik_request.robot_state.joint_state = _current_Joint;
    _request_ik.request.ik_request.group_name = "manipulator";    
    _request_ik.request.ik_request.pose_stamped = geometry_msgs::PoseStamped();
    _request_ik.request.ik_request.pose_stamped.header.frame_id = "world";
    _request_ik.request.ik_request.robot_state.is_diff = true;

    _request_trajectory.request.motion_plan_request.group_name = "manipulator";
    //_request_trajectory.request.motion_plan_request.start_state.joint_state = _current_Joint;

    _nh = nh;
    _client_ik = _nh.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");
    _client_trajectory = _nh.serviceClient<moveit_msgs::GetMotionPlan>("plan_kinematic_path");
    
  }

  sensor_msgs::JointState IK_compute(geometry_msgs::PoseStamped end_effector_position)
  {
    _request_ik.request.ik_request.pose_stamped = end_effector_position;
    ROS_INFO_STREAM(_request_ik.request);
    _request_ik.request.ik_request.pose_stamped.header.frame_id = "world";
    if (_client_ik.call(_request_ik))
    {
      _request_ik.request.ik_request.robot_state.joint_state = _request_ik.response.solution.joint_state;
      std::cout << _request_ik.response.error_code << std::endl;
    }
    return _request_ik.request.ik_request.robot_state.joint_state;
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

  static const std::string PLANNING_GROUP = "manipulator";

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Publisher joint_pub = _nh.advertise<sensor_msgs::JointState>("query_joint_state", 1);
  ros::Subscriber sub = _nh.subscribe("unity/end_effector_goal", 1000, eeCb);
  ik_request ik_computation = ik_request(_nh);


  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

  sensor_msgs::JointState joint_to_send = sensor_msgs::JointState();
  ros::Rate loop_rate(10);

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();
  /* Create a RobotState and JointModelGroup to keep track of the current robot pose and planning group*/
  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));
  const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);

  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
  planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");

  boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
  planning_interface::PlannerManagerPtr planner_instance;
  std::string planner_plugin_name;


  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  if (!_nh.getParam("/move_group/planning_plugin", planner_plugin_name))
    ROS_FATAL_STREAM("Could not find planner plugin name");
  try
  {
    planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
        "moveit_core", "planning_interface::PlannerManager"));
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
  }
  try
  {
    planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
    if (!planner_instance->initialize(robot_model, _nh.getNamespace()))
      ROS_FATAL_STREAM("Could not initialize planner instance");
    ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
  }
  catch (pluginlib::PluginlibException& ex)
  {
    const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
    std::stringstream ss;
    for (const auto& cls : classes)
      ss << cls << " ";
    ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                                                        << "Available plugins: " << ss.str());
  }


  while (ros::ok())
  {
    joint_to_send = ik_computation.IK_compute(end_effector_pos);
    joint_pub.publish(joint_to_send);
    if (end_effector_pos.header.stamp.sec == 1) //swithc to change in the object to remove the chance for segmetation fault
    {
      //TODO
      /*
       * function to use generated joint states to make a trajectory and execute
       */
      ROS_INFO_STREAM("time to execute.");
      moveit::core::RobotState goal_state(robot_model);
      std::vector<double> joint_values;
      
      // for (size_t i = 0; i < joint_to_send.position.size(); i++)
      // {
      //   joint_values.push_back(joint_to_send.position[i]);
      // }
      // goal_state.setJointGroupPositions(joint_model_group, joint_values);
      // moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
      // std::cout << joint_goal << std::endl;
      // req.goal_constraints.clear();
      // req.group_name = PLANNING_GROUP;
      // req.goal_constraints.push_back(joint_goal);
      // planning_interface::PlanningContextPtr context =
      //     planner_instance->getPlanningContext(planning_scene, req, res.error_code_); //error here
      // /* Call the Planner */
      // context->solve(res);
      // /* Check that the planning was successful */
      // if (res.error_code_.val != res.error_code_.SUCCESS)
      // {
      //   ROS_ERROR("Could not compute plan successfully");
      //   return 0;
      // }
      
      moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
      std::vector<double> joint_group_positions;
      current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

      joint_group_positions = joint_to_send.position;

      bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      if (success)
      {
          move_group_interface.execute(my_plan);
      }
      cre
    }
    loop_rate.sleep();
  }
  

  return 0;
}
