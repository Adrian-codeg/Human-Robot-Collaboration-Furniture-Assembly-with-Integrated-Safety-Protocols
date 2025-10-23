#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <vector>
#include <numeric>


bool stop = false;
bool slow_down = false;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  double angle_min = scan->angle_min;
  double angle_increment = scan->angle_increment;

  std::vector<float> right_ranges;
  std::vector<float> left_ranges;

  for (size_t i = 0; i < scan->ranges.size(); ++i)
  {
    double angle_rad = angle_min + i * angle_increment;
    double angle_deg = angle_rad * 180.0 / M_PI;

    float range = scan->ranges[i];
    if (std::isinf(range)) range = 0.0f;

    if (angle_deg > -20.0 && angle_deg < -1.0)
    {
      right_ranges.push_back(range);
    }

    if (angle_deg > 92.0 && angle_deg < 109.0)
    {
      left_ranges.push_back(range);
    }
  }

  // Compute mean for right
  if (!right_ranges.empty())
  {
    float right_mean = std::accumulate(right_ranges.begin(), right_ranges.end(), 0.0f) / right_ranges.size();
    if (right_mean > 1.0 && right_mean < 2.0)
    {
      ROS_INFO("Detecting a person on the right, mean range: %.2f, stop the robot!", right_mean);
      stop = true;
    }
    else
    {
      stop = false;
    }
  }

  // Compute mean for left
  if (!left_ranges.empty())
  {
    float left_mean = std::accumulate(left_ranges.begin(), left_ranges.end(), 0.0f) / left_ranges.size();
    if (left_mean >= 0.9 && left_mean < 2.0)
    {
      ROS_INFO("Detecting a person on the left, mean range: %.2f, slow down the robot!", left_mean);
      slow_down= true;
    }
    else
    {
      slow_down= false;
    }
  }
  if (stop)
  {
    ROS_INFO("Stopping the robot...");
  }
  else if (slow_down)
  {
    ROS_INFO("Slowing down the robot...");
  }
  else if (stop && slow_down)
  {
    ROS_INFO("Stopping and slowing down the robot...");
  }
  else
  {
    ROS_INFO("No obstacles detected, robot can proceed.");
  }
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "lidar_sector_listener");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/scan", 10, scanCallback);

  

  ros::spin();
  return 0;
}

// #include <ros/ros.h>
// #include <sensor_msgs/LaserScan.h>
// #include <cmath>

// void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
// {
//   double angle_min = scan->angle_min;         // Starting angle [rad]
//   double angle_increment = scan->angle_increment;

//   for (size_t i = 0; i < scan->ranges.size(); ++i)
//   {
//     double angle_rad = angle_min + i * angle_increment;
//     double angle_deg = angle_rad * 180.0 / M_PI;

//     // Filter angles between 90 and 110 degrees
//     if (angle_deg >= 80.0 && angle_deg <= 110.0)
//     {
//       ROS_INFO("Angle: %.2f°, Range: %.2f meters", angle_deg, scan->ranges[i]);
//     }
//   }
// }

// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "lidar_sector_listener_90_110");
//   ros::NodeHandle nh;

//   ros::Subscriber sub = nh.subscribe("/scan", 10, scanCallback);

//   ros::spin();
//   return 0;
// }
