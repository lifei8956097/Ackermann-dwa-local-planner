#ifndef __DWA_PLANNER_H
#define __DWA_PLANNER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <perception_msgs/Perception.h>
#include <actuator/actuator.h>
#include <actuator/cmd.h>
#include <planner/planner.h>
#include <planner/point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <atomic>
#include <Eigen/Dense>

class DWAPlanner {
 public:
  DWAPlanner(void);

  class State {
   public:
    State(double, double, double, double, double);

    double x;// robot position x
    double y;// robot posiiton y
    double yaw;// robot orientation yaw
    double velocity;// robot linear velocity
    double omega;// robot angular velocity
   private:
  };

  class Window {
   public:
    Window(void);
    Window(const double, const double, const double, const double);
    double min_velocity;
    double max_velocity;
    double min_omega;
    double max_omega;
   private:
  };
  void process(void);
  void local_path_callback(const planner::plannerConstPtr &);
  void laser_point_cloud_callback(const perception_msgs::PerceptionConstPtr &);
  void actuator_callback(const actuator::actuatorConstPtr &);
  void target_velocity_callback(const geometry_msgs::TwistConstPtr &);
  Window calc_dynamic_window(const double);
  float calc_to_goal_cost(const std::vector<State> &traj, const Eigen::Vector3d &goal);
  float calc_speed_cost(const std::vector<State> &traj, const float target_velocity);
  float calc_obstacle_cost(const std::vector<State> &traj, const std::vector<std::vector<float>> &);
  float calculateHeadingCost(const std::vector<State> &traj, const Eigen::Vector3d &goal);
  void motion(State &state, const double velocity, const double omega);
  std::vector<std::vector<float>> laser_point_cloud_to_obs();
  void visualize_trajectories(const std::vector<std::vector<State>> &, const double, const double, const double, const int, const ros::Publisher &);
  void visualize_trajectory(const std::vector<State> &, const double, const double, const double, const ros::Publisher &);
  std::vector<State> dwa_planning(Window, Eigen::Vector3d, std::vector<std::vector<float>>);
  nav_msgs::Path calculatePathYaw(nav_msgs::Path path_in);
 protected:
  double HZ;
  std::string ROBOT_FRAME;
  double TARGET_VELOCITY;
  double MAX_VELOCITY;
  double MIN_VELOCITY;
  double MAX_GAMMA;
  double MAX_ACCELERATION;
  double MAX_OMEGA;
  double MIN_GAMMA;
  double MAX_DIST;
  double VELOCITY_RESOLUTION;
  double OMEGA_RESOLUTION;
  double ANGLE_RESOLUTION;
  double PREDICT_TIME;
  double TO_GOAL_COST_GAIN;
  double SPEED_COST_GAIN;
  double OBSTACLE_COST_GAIN;
  double HEAD_COST_GAIN;
  double DT;
  double GOAL_THRESHOLD;
  double TURN_DIRECTION_THRESHOLD;
  double CAR_L;
  double CAR_W;
  double DETECT_OBSTACLE_DIS_THR;
  std::atomic<int> risk;
  nav_msgs::Path local_path;
  ros::NodeHandle nh;
  ros::NodeHandle local_nh;

  ros::Publisher akman_cmd_pub;
  ros::Publisher candidate_trajectories_pub;
  ros::Publisher selected_trajectory_pub;
  ros::Subscriber local_path_sub;
  ros::Subscriber local_map_sub;
  ros::Subscriber laser_point_cloud_sub;
  ros::Subscriber local_goal_sub;
  ros::Subscriber actuator_sub;
  ros::Subscriber target_velocity_sub;
//  tf::TransformListener listener;

  geometry_msgs::PoseStamped local_goal;
  perception_msgs::Perception laser_point_cloud;
  nav_msgs::OccupancyGrid local_map;
  double   current_omega;
  double current_velocity;
  double current_gamma;
  bool local_goal_subscribed;
  bool laser_point_cloud_updated;
  bool actuator_updated;
};

#endif //__DWA_PLANNER_H
