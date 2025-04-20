/**
 * @file cw2_solution_node.cpp
 * @brief Entry point for the CW2 solution node; initializes ROS, instantiates the cw2 class, 
 *        and enters the ROS spin loop.
 */

 #include "cw2_class.h"                    // cw2 class definition
 
 /**
  * @brief Main function: initializes ROS node and cw2 class, performs an initial move, and spins.
  * 
  * @param argc Argument count.
  * @param argv Argument vector.
  * @return int Exit status code.
  */
 int main(int argc, char **argv)
 {
   // Initialize ROS and name this node
   ros::init(argc, argv, "cw2_solution_node");
   ros::NodeHandle nh;
 
   // Instantiate the cw2 control class
   cw2 cw_class(nh);
 
   // Start asynchronous spinner for non-blocking MoveIt callbacks
   ros::AsyncSpinner spinner(1);
   spinner.start();
 
   // Move to a standby pose before processing service callbacks
   geometry_msgs::PointStamped standby = cw_class.make_point(0.4, 0.0, 0.5);
   cw_class.move_to_pose(standby, /* z_offset */ 0.0, /* reset_orientation */ true);
   ros::Duration(1.0).sleep();  // allow motion to complete
 
   // Enter the main ROS loop, processing callbacks at 10 Hz
   ros::Rate loop_hz(10);
   while (ros::ok()) {
     ros::spinOnce();
     loop_hz.sleep();
   }
 
   return 0;
 }
 
 