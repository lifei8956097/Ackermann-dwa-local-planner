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
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

class DWAPlanner {
 public:
  DWAPlanner(void);
  ~DWAPlanner();
  class State {
   public:
    State(double, double, double, double, double);

    double x;// robot position x
    double y;// robot posiiton y
    double yaw;// robot orientation yaw arc
    double velocity;// robot linear velocity
    double omega;// robot angular velocity arc/s
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

  /*
   * p1--------------p2
   * |  *         *  |
   * |    *     *    |
   * |       p       |
   * |     *    *    |
   * |  *          * |
   * p3--------------p4
   */
  class TrackedArea {
   public:
    TrackedArea(void);
    TrackedArea(const double, const double, const double, const double, const double, const double, const double);
    bool IsPtInArea(const double, const double);
    // 计算 |p1 p2| X |p1 p|
    inline double GetCross(const Eigen::Vector3d p1,
                           const Eigen::Vector3d p2,
                           const Eigen::Vector3d p) {
      return (p2[0] - p1[0]) * (p[1] - p1[1]) - (p[0] - p1[0]) * (p2[1] - p1[1]);
    }
    double min_x, max_x;
    double min_y, max_y;
    Eigen::Vector3d left_top; //p1
    Eigen::Vector3d right_top; // p2
    Eigen::Vector3d right_bottom; //p3
    Eigen::Vector3d left_bottom; //p4
    double offset_x;
    double offset_y;
    double yaw;
   private:
  };

  void process(void);
  void cmdThread();
  void local_path_callback(const planner::plannerConstPtr &);
  void laser_point_cloud_callback(const perception_msgs::PerceptionConstPtr &);
  void actuator_callback(const actuator::actuatorConstPtr &);
  Window calc_dynamic_window(const double);
  float calc_to_goal_cost(const std::vector<State> &traj, const Eigen::Vector3d &goal);
  float calc_speed_cost(const std::vector<State> &traj, const float target_velocity);
  float calc_obstacle_cost(const std::vector<State> &traj, const std::vector<std::vector<float>> &);
  float calculateHeadingCost(const std::vector<State> &traj, const Eigen::Vector3d &goal);
  float calculatecurvatureCost(const double, const double);
  void motion(State &state, const double velocity, const double omega);
  std::vector<std::vector<float>> laser_point_cloud_to_obs();
  // 当前的points精度为0.1m无法在过滤,该函数过滤车后的障碍物。
  std::set<std::vector<float> > laser_point_cloud_to_obs_with_filter();
  void visualize_trajectories(const std::vector<std::vector<State>> &, const double, const double, const double, const int, const ros::Publisher &);
  void visualize_trajectory(const std::vector<State> &, const double, const double, const double, const ros::Publisher &);
  void visualize_local_goal(const geometry_msgs::PoseStamped local_goal, const double, const double, const double, const ros::Publisher &);
  void visualize_car_mode(const TrackedArea &car_tracked_area, const double, const double, const double, const ros::Publisher &);
  void visualize_collision_points_warning(const std::set<std::vector<float>> &, const double, const double, const double, const ros::Publisher &);
  std::vector<State> dwa_planning(Window, Eigen::Vector3d, std::vector<std::vector<float>>);
  nav_msgs::Path calculatePathYaw(nav_msgs::Path path_in);
  bool is_enable_planner();
  // 8m以外不检测，检测到碰撞后，在碰撞点后5m设置local goal
  bool collision_detection(const planner::planner &, const std::set<std::vector<float>> &);
  /* 配合collision_detection 使用；
  ***设置local goal和id , path sample is 0.2m, offset = 0.5m,offset_index = 0.5 / 0.2;
  */
  void set_local_goal(const int collision_state_index, const double obs_x, const double obs_y, const int offset_index);
  void update_local_goal();
 protected:
  double HZ;
  std::string ROBOT_FRAME;
  double TARGET_VELOCITY;
  double MAX_VELOCITY;
  double MIN_VELOCITY;
  double MAX_GAMMA;
  double MAX_ACCELERATION;
  double MAX_OMEGA;
  double MIN_GAMMA; //arc
  double MAX_DIST;
  double VELOCITY_RESOLUTION;
  double OMEGA_RESOLUTION;
  double ANGLE_RESOLUTION;
  double PREDICT_TIME;
  double TO_GOAL_COST_GAIN;
  double SPEED_COST_GAIN;
  double OBSTACLE_COST_GAIN;
  double HEAD_COST_GAIN;
  double CURVATURE_COST_GAIN;
  double DT;
  double GOAL_THRESHOLD;
  double TURN_DIRECTION_THRESHOLD;
  double WHELL_AXLE_LENGTH;
  double CAR_WIDTH;
  double CAR_LENGTH;
  double CAR_FRONT_2_AKMAN_LENGTH;
  double CAR_BACK_2_AKMAN_LENGTH;
  double EXPANSION_DISTANCE_WIDTH;
  double EXPANSION_DISTANCE_LENGTH;
  double LIDAR_2_AKMAN_OFFSET_X;//lidar coord 2 akman coord
  std::atomic<int> risk;
  std::atomic<bool> cmd_updated;
  std::atomic<bool> enable_dwa_planner;
  ros::NodeHandle nh;
  ros::NodeHandle local_nh;

  ros::Publisher akman_cmd_pub;
  ros::Publisher local_path_pub;
  ros::Publisher candidate_trajectories_pub;
  ros::Publisher selected_trajectory_pub;
  ros::Publisher local_goal_pub;
  ros::Publisher car_traked_area_pub;
  ros::Publisher car_traked_area_warning_pub;
  ros::Publisher collision_points_warning_pub;
  ros::Subscriber local_path_sub;
  ros::Subscriber local_map_sub;
  ros::Subscriber laser_point_cloud_sub;
  ros::Subscriber local_goal_sub;
  ros::Subscriber actuator_sub;
  ros::Subscriber target_velocity_sub;
  //  tf::TransformListener listener;

  std::vector<std::vector<State>> candidate_trajectories;
  geometry_msgs::PoseStamped local_goal;
  actuator::cmd cmd_vel;
  int local_goal_id = -1;
  nav_msgs::Path local_path;
  planner::planner origin_local_path;// just for update goal position
  perception_msgs::Perception laser_point_cloud;
  nav_msgs::OccupancyGrid local_map;
  double   current_omega;// arc/s
  double current_velocity;//m/s
  double current_gamma_arc; //arc
  double MIN_TRUN_RADIUS;
  TrackedArea tracked_area;
  int  control_id = 1;
  bool local_goal_exist;
  bool local_path_updated;
  bool laser_point_cloud_updated;
  bool actuator_updated;
  boost::thread *cmd_thread_;
  boost::recursive_mutex cmd_mutex_;
  boost::recursive_mutex wake_up_mutex_;
  boost::condition_variable_any cmd_cond_;
};

#endif //__DWA_PLANNER_H
