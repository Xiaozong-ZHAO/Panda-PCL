/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw2_team_<your_team_number> package */

#include <cw2_class.h> // change to your team name here!

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cw2_solution_node");
  ros::NodeHandle nh;

  // 实例化你的类
  cw2 cw_class(nh);

  // 启动异步 spinner（MoveIt 非阻塞必须）
  ros::AsyncSpinner spinner(1);
  spinner.start();

  /* ---------- 用 make_point 简化 ---------- */
  geometry_msgs::PointStamped standby = cw_class.make_point(0.4, 0.0, 0.5);
  cw_class.move_to_pose(standby, /*z_offset=*/0.0, /*reset_orientation=*/true);
  ros::Duration(1.0).sleep();

  /* ---------- 进入 ROS 主循环 ---------- */
  ros::Rate loop_hz(10);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_hz.sleep();
  }
}

