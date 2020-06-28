#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

float x_pos = 0.0;
float y_pos = 0.0;

void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  x_pos = msg->pose.pose.position.x;
  y_pos = msg->pose.pose.position.y;

  ROS_INFO("Seq: [%d]", msg->header.seq);
  ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
  ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Subscriber odometry_sub = n.subscribe("odom", 1000, odometryCallback);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  // Define the location of the pick-up and drop-off goals
  float goals[2][3] = { {-3.0, 2.0, -1.0}, {1.0, 0.5, 1.0} };

  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers";
    marker.id = 0;

    // Set the marker type to be the shape previously defined
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = goals[0][0];
    marker.pose.position.x = goals[0][1];
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    bool pickedUp = false;

    while (ros::ok())
    {
      float tolerance = 0.25;
      float x_distance;
      float y_distance;
      
      // Robot has not yet reached the pick-up zone
      if (pickedUp == false)
      {
        marker_pub.publish(marker);
        x_distance = fabs(x_pos-marker.pose.position.x);
        y_distance = fabs(y_pos-marker.pose.position.y);

        // Robot has reached the pick-up zone
        if ((x_distance<tolerance) && (y_distance<tolerance))
        {
          marker.action = visualization_msgs::Marker::DELETE;
          marker_pub.publish(marker);
          pickedUp = true;
        }
      }
      // Robot has not yet reached the drop-off zone
      else 
      {
        x_distance = fabs(x_pos-goals[1][0]);
        y_distance = fabs(y_pos-goals[1][1]);

        // Robot has reached the drop-off zone
        if ((x_distance<tolerance) && (y_distance<tolerance))
        {
          marker.pose.position.x = goals[1][0];
          marker.pose.position.y = goals[1][1];
          marker.pose.position.z = 0;

          marker.pose.orientation.x = 0.0;
          marker.pose.orientation.y = 0.0;
          marker.pose.orientation.z = 0.0;
          marker.pose.orientation.w = 1.0;
          //marker.pose.orientation.w = goals[1][2];

          marker.action = visualization_msgs::Marker::ADD;
          marker_pub.publish(marker);
        }
      }
    }

    r.sleep();
  }
}
