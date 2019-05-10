//首先要包含API的头文件
#include <moveit/move_group_interface/move_group_interface.h>
#include<moveit_msgs/DisplayTrajectory.h>
#include<moveit_msgs/DisplayRobotState.h>
#include "hello.h"


#include <moveit/robot_model_loader/robot_model_loader.h>
#include<moveit/robot_model/robot_model.h>
#include<moveit/robot_state/robot_state.h>

int main(int argc, char **argv)
{
  print_Hello();
  ros::init(argc, argv, "moveit_random_demo", ros::init_options::AnonymousName);
  // 创建一个异步的自旋线程（spinning thread）
  ros::AsyncSpinner spinner(1);
  spinner.start();

//forward K
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
	ROS_INFO("zhe shi kai shi:Model frame: %s", kinematic_model->getModelFrame().c_str());

	robot_state::RobotStatePtr Kinematic_state(new robot_state::RobotState(kinematic_model));
	Kinematic_state->setToDefaultValues();
	const robot_state::JointModelGroup* joint_model_group=kinematic_model->getJointModelGroup("ca20n_arm");
	const std::vector<std::string>& joint_names=joint_model_group->getVariableNames();

std::vector<double> joint_values;
Kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
for (std::size_t i = 0; i < joint_names.size(); ++i)
{
  ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
}

//real FK

Kinematic_state->setToRandomPositions(joint_model_group);
const Eigen::Affine3d& end_effector_state = Kinematic_state->getGlobalLinkTransform("link_6");
ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");
  // 连接move_group节点中的机械臂实例组，这里的组名arm是我们之前在setup assistant中设置的
  moveit::planning_interface::MoveGroupInterface group("ca20n_arm");

  // 随机产生一个目标位置
  //group.setRandomTarget();
  // 开始运动规划，并且让机械臂移动到目标位置
moveit::planning_interface::MoveGroupInterface::Plan plan;
moveit::core::RobotStatePtr current_state = group.getCurrentState();
//bool success = (group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

std::vector<double> joint_group_position;
current_state->copyJointGroupPositions(joint_model_group, joint_group_position);
joint_group_position[0]=0.0;
joint_group_position[1]=0.0;
joint_group_position[2]=-1.5708;
joint_group_position[3]=0.0;
joint_group_position[4]=1.5708;
joint_group_position[5]=0.0;


group.setJointValueTarget(joint_group_position);
//success = (group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  group.plan(plan);
  group.execute(plan);

  geometry_msgs::Pose target_pose=group.getCurrentPose().pose;
  std::vector<geometry_msgs::Pose>waypoints;
  waypoints.push_back(target_pose);
  for(int i=1;i<20;i++)
  {
    target_pose.position.y+=0.05;
    target_pose.position.z-=0.05;
    waypoints.push_back(target_pose);
  }
  group.setMaxVelocityScalingFactor(0.1);
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
const double eef_step = 0.01;
double fraction = group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
plan.trajectory_=trajectory;
ROS_INFO("HERE");
group.plan(plan);
//ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
  ros::waitForShutdown();
}
