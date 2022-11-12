#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // Request a service and pass the velocities to it to drive the robot
  ball_chaser::DriveToTarget srv;
  srv.request.linear_x = lin_x;
  srv.request.angular_z = ang_z;
  if (!client.call(srv))
        ROS_ERROR("Failed to call service command_robot");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
	//Loop through each pixel in the image and check if there's a bright one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
  	ROS_INFO("Searching");
    int white_pixel = 255;
  	bool found = false;
	int i = 0;
  while (i < img.height * img.step && !found) {
    ROS_INFO("Pixel value: %1.2f, pixel number %1.2f", (float)img.data[i], (float)i);
    if (img.data[i] == white_pixel)
    {
      found = true;
      int row = i % img.step;
      float column = row / (img.step / 3);

      if (column < 1) {
        ROS_INFO("turning left");
        drive_robot(0.1, 1);
      } else if (column > 2 || row == 0) {
        ROS_INFO("turning right");
        drive_robot(0.1, -1);
      } else {
        ROS_INFO("going forward");
        drive_robot(1, 0.0);
      }
    }
    i++;
  }
  if(!found){
  	drive_robot(0.0, 0.0);
    ROS_INFO("No black ball found");
  }

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