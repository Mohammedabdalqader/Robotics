#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal pickup_goal;
  move_base_msgs::MoveBaseGoal drop_off;

  // set up the frame parameters
  pickup_goal.target_pose.header.frame_id = "map";
  pickup_goal.target_pose.header.stamp = ros::Time::now();
  
  drop_off.target_pose.header.frame_id = "map";
  drop_off.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  pickup_goal.target_pose.pose.position.x = -4.0;
  pickup_goal.target_pose.pose.position.y = -0.75;
  pickup_goal.target_pose.pose.orientation.w = -1.5;
  
  drop_off.target_pose.pose.position.x = -3.7;
  drop_off.target_pose.pose.position.y = 3.0;
  drop_off.target_pose.pose.orientation.w = 1.5;

   // Send the first goal position and orientation for the robot to reach
  ROS_INFO("Sending pickup location");
  ac.sendGoal(pickup_goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The Robot reached its destination");
  else
    ROS_INFO("The Robot failed to reached its destination");
  
  ros::Duration(5.0).sleep() ; // Sleep for five second
    
  // Send the second goal position and orientation for the robot to reach
  ROS_INFO("Sending drop off location");
  ac.sendGoal(drop_off);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The Robot reached the drop off zone.");
  else
    ROS_INFO("The Robot failed to reached the drop off zone.");

  return 0;
}
