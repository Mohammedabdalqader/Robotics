#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    //Request a service and pass the velocities to it to drive the robot
  ball_chaser::DriveToTarget srv;
  srv.request.linear_x  = lin_x;
  srv.request.angular_z = ang_z;
  
      // Call the service and pass the requested velocities
    if (!client.call(srv))
        ROS_ERROR("Failed to call service");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    // Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera

    // |             Image            |
    // |   left   |   mid   | right   |
    // | 0-30%    | 31%-70% |71%-100% |

    int white_pixel = 255;
    bool target_found = false;
    float lin_x=0.0,ang_z = 0.0;
    // For pixel to be white all its three channels that is red, green and blue channels must have the intensities of 255.
    for (int i = 0; i < img.height * img.step; i ++) {
		if (img.data[i] == white_pixel  && img.data[i + 1] == white_pixel && img.data[i + 2] == white_pixel ) {          // The camera recognize the ball and now need to specify in which direction should the robot move
	    	target_found = true;
	    	int row = i % img.step;      

        	if (row <= img.step * 0.3 ) {
            	// Turn to the left
            	lin_x = 0.0;
            	ang_z = 0.5;
                
         	}  
      
         	else if ( img.step * 0.3 < row <= img.step * 0.7){
         		// Move Forward
           		lin_x = 0.5;
           		ang_z = 0.0;
           
         	}
         	else {
           		// Turn to the right
           		lin_x = 0.0;
           		ang_z = -0.5;
         	}
          
          break;
      
        }
    }
    if (target_found)
      ROS_INFO_STREAM("Target found");
    drive_robot(lin_x, ang_z);
}


int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
