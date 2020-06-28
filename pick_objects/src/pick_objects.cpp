#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>

// Define desired goals for robot to reach
//float goal1 = {3.0, 4.0, 1.0};
//float goal2 = {0.0, 1.0, 1.0};
float goals[2][3] = { {-3.0, 2.0, -1.0}, {1.0, 0.5, 1.0} };
//float goals[2][3] = { {-1.0, -2.0, 1.0}, {-3.0, 2.0, -1.57} };

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  //float goals[2][3] = { {3.0, 4.0, 1.0}, {0.0, 1.0, 1.0} };

  for (int i=0; i<(sizeof(goals)/sizeof(goals[0])); i++) {
  //for (int i=0; i<2; i++) {

    // Define a position and orientation for the robot to reach
    //goal.target_pose.pose.position.x = 1.0;
    //goal.target_pose.pose.orientation.w = 1.0;
    goal.target_pose.pose.position.x = goals[i][0];
    goal.target_pose.pose.position.y = goals[i][1];
    //goal.target_pose.pose.orientation.w = goals[i][2];
    goal.target_pose.pose.orientation.w = 1.0;

    // Send the goal position and orientation for the robot to reach
    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    // Wait an infinite time for the results
    ac.waitForResult();

    // Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Hooray, the robot reached the goal");
      if (i == 0)
        ros::Duration(5.0).sleep();
    else
      ROS_INFO("The robot failed to reach the goal for some reason");
  }

  return 0;
}
