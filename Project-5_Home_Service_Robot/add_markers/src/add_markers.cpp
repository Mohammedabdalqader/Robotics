#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <math.h>


// Define pick_up and drop_off locations
float PICKUP_X = -4.0;
float PICKUP_Y = -0.75;
float PICKUP_W = -1.5;

float DROPOFF_X = -3.7;
float DROPOFF_Y = 3.0;
float DROPOFF_W = 1.5;


bool pick_up = false;
bool drop_off = false;
bool drop_off_finished =false;
float tolerance = 0.3;
// odometry Callback function
void odometry_callback(const nav_msgs::Odometry::ConstPtr& msg) 
{
	float robot_pose_x = msg->pose.pose.position.x;
	float robot_pose_y = msg->pose.pose.position.y;
	
	float pickup_loc;
	float dropoff_loc;
	
	if(!pick_up && !drop_off )
	{
       // Here we calculate the distance between the current position of the robots and the pick-up position
		pickup_loc = abs(robot_pose_x - PICKUP_X) + abs(robot_pose_y - PICKUP_Y);
		ROS_INFO("the robot is %0.3f meters away from pickup loaction ", pickup_loc);
		
		if(pickup_loc < tolerance)  
		{
			ROS_INFO("The Robot Arrived at the pick up zone");
			pick_up = true;
		}
	}
	if(pick_up && !drop_off)  // if the robot has already picked up the object, the second target is the "drop_off zone".

	{
       // Here we calculate the distance between the current position of the robots and the drop-off position
		dropoff_loc = abs(robot_pose_x - DROPOFF_X) + abs(robot_pose_y - DROPOFF_Y);
		ROS_INFO("the robot is %0.3f meters away from drop off loaction", dropoff_loc);
		
		if(dropoff_loc <= tolerance)
		{
			ROS_INFO("The Robot Arrived at the drop off zone");
			drop_off = true;
		}
	}
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Subscribe to odometry node
  ros::Subscriber odometry_sub = n.subscribe("/odom", 1000, odometry_callback);
  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;


  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "add_markers";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = shape;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;
    

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = PICKUP_X;
  marker.pose.position.y = PICKUP_Y;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = PICKUP_W;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration(5.0); 
  // Publish the marker
  
  	while (ros::ok())
  	{
		// Publish the marker
		while (marker_pub.getNumSubscribers() < 1)
		{
			if (!ros::ok())
			{
				return 0;
			}
			ROS_WARN_ONCE("Please create a subscriber to the marker");
			sleep(1);
		}
		
		if(pick_up && !drop_off)
		{
			marker.action = visualization_msgs::Marker::DELETE;
			ROS_INFO("Object was successfully pickerd up");
			ros::Duration(5.0).sleep();
		}
		
		if(drop_off && !drop_off_finished)
		{
            drop_off_finished = true;
			marker.pose.position.x = DROPOFF_X;
			marker.pose.position.y = DROPOFF_Y;
            marker.pose.orientation.w = DROPOFF_W;
			marker.action = visualization_msgs::Marker::ADD;
			ROS_INFO("Object was successfully dropped off");
		}
		
		marker_pub.publish(marker);
        ros::spinOnce();

		
	}
	
	return 0;

}