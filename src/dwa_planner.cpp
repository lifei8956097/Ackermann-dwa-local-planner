#include "dwa_planner/dwa_planner.h"
#include <boost/thread.hpp>
#define DWA_PLANNER_VELOCITY_FACTOR 50
DWAPlanner::DWAPlanner(void)
  : local_nh("~"), local_goal_subscribed(false),
    laser_point_cloud_updated(false), actuator_updated(false), enable_dwa_planner(false) {
  local_nh.param("HZ", HZ, {20});
  local_nh.param("ROBOT_FRAME", ROBOT_FRAME, {"base_link"});
  local_nh.param("TARGET_VELOCITY", TARGET_VELOCITY, {0.8});
  local_nh.param("MAX_VELOCITY", MAX_VELOCITY, {1.0});
  local_nh.param("MIN_VELOCITY", MIN_VELOCITY, {0.0});
  local_nh.param("MAX_GAMMA", MAX_GAMMA, {0.52});
  local_nh.param("MIN_GAMMA", MIN_GAMMA, {-0.52});
  local_nh.param("MAX_OMEGA", MAX_OMEGA, {1.0});
  local_nh.param("MAX_ACCELERATION", MAX_ACCELERATION, {1.0});
  local_nh.param("MAX_DIST", MAX_DIST, {10.0}); // 当前轨迹生成距离策略为6m，大于6m触发不了dwa。
  local_nh.param("VELOCITY_RESOLUTION", VELOCITY_RESOLUTION, {0.1});
  local_nh.param("OMEGA_RESOLUTION", OMEGA_RESOLUTION, {0.1});
  local_nh.param("ANGLE_RESOLUTION", ANGLE_RESOLUTION, {0.2});
  local_nh.param("PREDICT_TIME", PREDICT_TIME, {3.0});
  local_nh.param("TO_GOAL_COST_GAIN", TO_GOAL_COST_GAIN, {1.0});
  local_nh.param("SPEED_COST_GAIN", SPEED_COST_GAIN, {1.0});
  local_nh.param("OBSTACLE_COST_GAIN", OBSTACLE_COST_GAIN, {1.0});
  local_nh.param("HEAD_COST_GAIN", HEAD_COST_GAIN, {1.0});
  local_nh.param("CURVATURE_COST_GAIN", CURVATURE_COST_GAIN, {1.0});
  local_nh.param("GOAL_THRESHOLD", GOAL_THRESHOLD, {0.3});
  local_nh.param("TURN_DIRECTION_THRESHOLD", TURN_DIRECTION_THRESHOLD, {1.0});
  local_nh.param("CAR_L", CAR_L, {0.8633246});
  local_nh.param("CAR_W", CAR_W, {1.036});
  local_nh.param("LIDAR_2_AKMAN_OFFSET_X", LIDAR_2_AKMAN_OFFSET_X, {0.5});
  DETECT_OBSTACLE_DIS_THR = CAR_W * 0.5 + 0.1;
  CAR_FRONT_TRACK_DIS = 2; //m
  DT = 1.0 / HZ;
  MAX_OMEGA = MAX_GAMMA / DT;
  risk = 0;
  cmd_updated = false;
  current_gamma_arc = 0;
  min_turn_radius = CAR_L / std::tan(MAX_GAMMA);
  tracked_area.min_x = - 0.7;
  tracked_area.max_x = CAR_FRONT_TRACK_DIS;
  tracked_area.min_y = -DETECT_OBSTACLE_DIS_THR;
  tracked_area.max_y = DETECT_OBSTACLE_DIS_THR;
  local_path.header.frame_id = "base_link";


  ROS_INFO("=== DWA Planner ===");
  ROS_INFO_STREAM("HZ: " << HZ);
  ROS_INFO_STREAM("DT: " << DT);
  ROS_INFO_STREAM("ROBOT_FRAME: " << ROBOT_FRAME);
  ROS_INFO_STREAM("TARGET_VELOCITY: " << TARGET_VELOCITY);
  ROS_INFO_STREAM("MAX_VELOCITY: " << MAX_VELOCITY);
  ROS_INFO_STREAM("MIN_VELOCITY: " << MIN_VELOCITY);
  ROS_INFO_STREAM("MAX_GAMMA: " << MAX_GAMMA / M_PI * 180);
  ROS_INFO_STREAM("MIN_GAMMA: " << MIN_GAMMA / M_PI * 180);
  ROS_INFO_STREAM("MAX_OMEGA: " << MAX_OMEGA);
  ROS_INFO_STREAM("MAX_ACCELERATION: " << MAX_ACCELERATION);
  ROS_INFO_STREAM("MAX_DIST: " << MAX_DIST);
  ROS_INFO_STREAM("VELOCITY_RESOLUTION: " << VELOCITY_RESOLUTION);
  ROS_INFO_STREAM("OMEGA_RESOLUTION: " << OMEGA_RESOLUTION);
  ROS_INFO_STREAM("ANGLE_RESOLUTION: " << ANGLE_RESOLUTION);
  ROS_INFO_STREAM("PREDICT_TIME: " << PREDICT_TIME);
  ROS_INFO_STREAM("TO_GOAL_COST_GAIN: " << TO_GOAL_COST_GAIN);
  ROS_INFO_STREAM("SPEED_COST_GAIN: " << SPEED_COST_GAIN);
  ROS_INFO_STREAM("OBSTACLE_COST_GAIN: " << OBSTACLE_COST_GAIN);
  ROS_INFO_STREAM("CURVATURE_COST_GAIN: " << CURVATURE_COST_GAIN);
  ROS_INFO_STREAM("GOAL_THRESHOLD: " << GOAL_THRESHOLD);
  ROS_INFO_STREAM("TURN_DIRECTION_THRESHOLD: " << TURN_DIRECTION_THRESHOLD);
  ROS_INFO_STREAM("CAR_L: " << CAR_L);
  ROS_INFO_STREAM("CAR_W:" << CAR_W);
  ROS_INFO_STREAM("LIDAR_2_AKMAN_OFFSET_X:" << LIDAR_2_AKMAN_OFFSET_X);
  akman_cmd_pub = nh.advertise<actuator::cmd>("/takeOver", 1);
  candidate_trajectories_pub = local_nh.advertise<visualization_msgs::MarkerArray>("candidate_trajectories", 1);
  selected_trajectory_pub = local_nh.advertise<visualization_msgs::Marker>("selected_trajectory", 1);
  local_goal_pub = local_nh.advertise<visualization_msgs::Marker>("dwa_local_goal", 1);
  car_traked_area_pub = local_nh.advertise<visualization_msgs::Marker>("car_tracked_area", 1);
  local_path_pub = local_nh.advertise<nav_msgs::Path>("dwa_local_path", 1, true);
  car_traked_area_warning_pub = local_nh.advertise<visualization_msgs::Marker>("car_tracked_area_warning", 1);
  collision_points_warning_pub = local_nh.advertise<visualization_msgs::Marker>("collision_points_warning", 1);
  local_path_sub = nh.subscribe("/planner", 1, &DWAPlanner::local_path_callback, this);
  laser_point_cloud_sub = nh.subscribe("/perception_info", 1, &DWAPlanner::laser_point_cloud_callback, this);
  actuator_sub = nh.subscribe("/actuator", 1, &DWAPlanner::actuator_callback, this);
//set up the planner's thread
  cmd_thread_ = new boost::thread(boost::bind(&DWAPlanner::cmdThread, this));
}

DWAPlanner::~DWAPlanner() {
  enable_dwa_planner = false;
  cmd_cond_.notify_all();
  cmd_thread_->interrupt();
  cmd_thread_->join();
  delete cmd_thread_;
}

DWAPlanner::State::State(double _x, double _y, double _yaw, double _velocity, double _omega)
  : x(_x), y(_y), yaw(_yaw), velocity(_velocity), omega(_omega) {
}

DWAPlanner::Window::Window(void)
  : min_velocity(0.0), max_velocity(0.0), min_omega(0.0), max_omega(0.0) {
}

DWAPlanner::Window::Window(const double min_v, const double max_v, const double min_omega, const double max_omega)
  : min_velocity(min_v), max_velocity(max_v), min_omega(min_omega), max_omega(max_omega) {
}

DWAPlanner::TrackedArea::TrackedArea(void)
  : offset_x(0.0), offset_y(0.0), min_x(0.0), max_x(0.0), min_y(0.0), max_y(0.0) {
}

DWAPlanner::TrackedArea::TrackedArea(const double off_x, const double off_y,
                                     const double min_xx, const double max_xx,
                                     const double min_yy, const double max_yy)
  : offset_x(off_x), offset_y(off_y), min_x(min_xx), max_x(max_xx), min_y(min_yy), max_y(max_yy) {
  min_x += offset_x;
  max_x += offset_x;
  min_y += offset_y;
  max_y += offset_y;
}

bool DWAPlanner::TrackedArea::IsPtInArea(const double x, const double y) {
  if (x >= min_x && x <= max_x && y >= min_y && y <= max_y) return true;
  return false;
}

void DWAPlanner::local_path_callback(const planner::plannerConstPtr &msg) {
  if (local_goal_id == -1) {
    local_path.poses.clear();
// 使用半径参数10m进行local goal的获取
    unsigned int i = msg->startPoint;
    for (; i < msg->points.size() ; i++) {
      geometry_msgs::Point P0;
      P0.x = msg->points[i].x;
      P0.y = msg->points[i].y;
      geometry_msgs::PoseStamped pose_tmp;
      if (sqrt(P0.x * P0.x  + P0.y * P0.y) <= 10) {
        pose_tmp.pose.position.x = P0.x + LIDAR_2_AKMAN_OFFSET_X;
        pose_tmp.pose.position.y = P0.y;
        local_path.poses.push_back(pose_tmp);
      } else {
        break;
      }
    }
    //TODO(lifei) not calculate the yaw
    if (!local_path.poses.empty()) {
//      local_path = calculatePathYaw(local_path);
      local_goal = local_path.poses.back();
      local_goal_id = msg->points[i - 1].id;
      local_goal_subscribed = true;
    }
  } else {
    for (unsigned int i = msg->startPoint; i < msg->points.size() ; i++) {
      if (msg->points[i].id == local_goal_id) {
        local_goal.pose.position.x = msg->points[i].x + LIDAR_2_AKMAN_OFFSET_X;
        local_goal.pose.position.y = msg->points[i].y;
        local_goal_subscribed = true;
        break;
      }
    }
//   ROS_INFO_STREAM("local_goal_id: " << local_goal_id);
  }
}

void DWAPlanner::laser_point_cloud_callback(const perception_msgs::PerceptionConstPtr &msg) {
  laser_point_cloud = *msg;
  laser_point_cloud_updated = true;
}

void DWAPlanner::actuator_callback(const actuator::actuatorConstPtr &msg) {
  if (msg->risk == 1 || risk == 0) {
    risk = msg->risk;
  }

  if (risk == 1) {
    current_gamma_arc = msg->steer / 180 * M_PI;
    current_velocity = msg->speed / 3.6; //m/s
    current_omega = current_velocity * std::tan(current_gamma_arc ) / CAR_L;
    actuator_updated = true;
  }
}

std::vector<DWAPlanner::State> DWAPlanner::dwa_planning(
  Window dynamic_window,
  Eigen::Vector3d goal,
  std::vector<std::vector<float>> obs_list) {
  float min_cost = 1e6;
  float min_obs_cost = min_cost;
  float min_goal_cost = min_cost;
  float min_speed_cost = min_cost;
//  float min_head_cost = min_cost;
  float min_curvature_cost = min_cost;
  std::vector<std::vector<State>> trajectories;
  std::vector<State> best_traj;
  // TODO(lifei) 角速度空间添加特定的采样角。
  std::set<float> omega_space;
  for (float i = -5; i < 5; i += 1) {
    double arc = i * 0.02;
    if (arc > dynamic_window.min_omega && arc < dynamic_window.max_omega)
      omega_space.insert(arc);
  }
  for (float omega = dynamic_window.min_omega; omega <= dynamic_window.max_omega; omega += OMEGA_RESOLUTION) {
    omega_space.insert(omega);
  }
  for (float v = dynamic_window.min_velocity; v <= dynamic_window.max_velocity; v += VELOCITY_RESOLUTION) {
    for (std::set<float>::iterator omega_it = omega_space.begin(); omega_it != omega_space.end(); ++omega_it) {
      State state(0.0, 0.0, 0.0, current_velocity, current_omega);
      std::vector<State> traj;
      for (float t = 0; t <= PREDICT_TIME; t += DT) {
        motion(state, v, *omega_it);
        // TODO(lifei) 选择角度在45内，距离在MAX_DIST内的路径，
        if (state.x != 0 && state.y != 0) {
          if ((fabs(state.yaw) > 0.52) ||
              std::sqrt(state.x * state.x + state.y * state.y) > MAX_DIST) break;
        }
        traj.push_back(state);
      }
//      if (traj.back().x < CAR_FRONT_TRACK_DIS + CAR_L) continue;
      if (tracked_area.IsPtInArea(traj.back().x, traj.back().y)) continue;
      trajectories.push_back(traj);

      float to_goal_cost = calc_to_goal_cost(traj, goal);
      float speed_cost = calc_speed_cost(traj, current_velocity);
//      float heading_cost = calculateHeadingCost(traj, goal);
      float curvature_cost = calculatecurvatureCost(v, *omega_it);
      float obstacle_cost = calc_obstacle_cost(traj, obs_list);
      float final_cost = TO_GOAL_COST_GAIN * to_goal_cost + SPEED_COST_GAIN * speed_cost +
                         OBSTACLE_COST_GAIN * obstacle_cost + CURVATURE_COST_GAIN * curvature_cost;
//          HEAD_COST_GAIN * heading_cost;
      if (min_cost >= final_cost) {
        min_goal_cost = TO_GOAL_COST_GAIN * to_goal_cost;
        min_obs_cost = OBSTACLE_COST_GAIN * obstacle_cost;
        min_speed_cost = SPEED_COST_GAIN * speed_cost;
        min_curvature_cost = CURVATURE_COST_GAIN * curvature_cost;
//        min_head_cost = HEAD_COST_GAIN * heading_cost;
        min_cost = final_cost;
        best_traj = traj;
      }
    }
  }
  ROS_INFO_STREAM("Cost: " << min_cost);
  ROS_INFO_STREAM("- Goal cost: " << min_goal_cost);
  ROS_INFO_STREAM("- Obs cost: " << min_obs_cost);
  ROS_INFO_STREAM("- Speed cost: " << min_speed_cost);
  ROS_INFO_STREAM("- curvature cost: " << min_curvature_cost);
  ROS_INFO_STREAM("num of trajectories: " << trajectories.size());
  candidate_trajectories = trajectories;
  if (min_cost == 1e6) {
    std::vector<State> traj;
    State state(0.0, 0.0, 0.0, 0, 0);
    traj.push_back(state);
    best_traj = traj;
    ROS_WARN_THROTTLE(1.0, "candidate_trajectories has no best way tracked");
  }
  return best_traj;
}

// yaw is arc
nav_msgs::Path DWAPlanner::calculatePathYaw(nav_msgs::Path path_in) {
  int length = path_in.poses.size();
  if (length <= 1) {
    if (length == 1)
      path_in.poses[0].pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    return path_in;
  }

  for (int i = length - 2; i < length - 1; ++i) {
    double dx = path_in.poses[i + 1].pose.position.x - path_in.poses[i].pose.position.x;
    double dy = path_in.poses[i + 1].pose.position.y - path_in.poses[i].pose.position.y;
    double theta = atan2(dy, dx);
    path_in.poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(theta);
  }

  path_in.poses.back().pose.orientation = path_in.poses[length - 2].pose.orientation;

  return path_in;
}

bool DWAPlanner::is_enable_planner() {
  if (!actuator_updated || !local_goal_subscribed || !laser_point_cloud_updated || risk != 1 ) return false;
  // Collision Detection
  std::set<std::vector<float>> obs_list;
  obs_list = laser_point_cloud_to_obs_with_filter();
  if (!collision_detection(local_path, obs_list)) return false;
  return true;
}

bool DWAPlanner:: collision_detection(const nav_msgs::Path &_local_path, const std::set<std::vector<float> > &_obs_list) {
  if (_local_path.poses.empty() || _obs_list.empty()) return false;
  int count = 0;
  for ( auto &iter : _local_path.poses) {
    if (++count < 12) continue;
    count = 0;
    TrackedArea path_tracked_area(iter.pose.position.x, iter.pose.position.y,
                                  tracked_area.min_x, tracked_area.max_x,
                                  tracked_area.min_y, tracked_area.max_y) ;
    for (const auto &obs : _obs_list) {
      if (obs.empty()) continue;
      if (path_tracked_area.IsPtInArea(obs[0], obs[1])) {
        local_path_pub.publish(_local_path);
        visualize_car_mode(path_tracked_area, 1, 1, 0, car_traked_area_warning_pub);
        visualize_collision_points_warning(_obs_list, 1, 1, 0, collision_points_warning_pub);
        return true;
      }
    }
  }
  return false;
}

void DWAPlanner::process(void) {
  ros::Rate loop_rate(HZ);
  while (ros::ok()) {
//    ROS_INFO("==========================================");
    double start = ros::Time::now().toSec();
    bool input_updated = false;
    if (laser_point_cloud_updated) {
      input_updated = true;
    }
    if (!enable_dwa_planner) {
      enable_dwa_planner = is_enable_planner();
      ros::spinOnce();
      loop_rate.sleep();
      ROS_INFO_STREAM_THROTTLE(1.0, "Not enable Planner");
      continue;
    }
    if (input_updated && local_goal_subscribed && actuator_updated) {
      Window dynamic_window = calc_dynamic_window(current_velocity);
      Eigen::Vector3d goal(local_goal.pose.position.x, local_goal.pose.position.y, tf::getYaw(local_goal.pose.orientation));
      ROS_INFO_STREAM_THROTTLE(5, "local goal: (" << goal[0] << "," << goal[1] << "," << goal[2] / M_PI * 180 << ")");
      boost::unique_lock<boost::recursive_mutex> lock(cmd_mutex_);
      cmd_vel.enable = true;
      cmd_vel.id = control_id;
//      if (goal.segment(0, 2).norm() > GOAL_THRESHOLD) {
      if (!tracked_area.IsPtInArea(goal[0], goal[1])) {
        std::vector<std::vector<float>> obs_list;
        obs_list = laser_point_cloud_to_obs();
        laser_point_cloud_updated = false;
        actuator_updated = false;

        std::vector<State> best_traj = dwa_planning(dynamic_window, goal, obs_list);
        cmd_vel.speed = best_traj[0].velocity * DWA_PLANNER_VELOCITY_FACTOR;
        cmd_vel.steer = std::atan(CAR_L * best_traj[0].omega /
                                  (best_traj[0].velocity + 0.001)) / M_PI * 180;
        visualize_trajectory(best_traj, 1, 0, 0, selected_trajectory_pub);
      } else {
        cmd_vel.speed = 0.0;
        cmd_vel.steer = 0.0;
        cmd_vel.enable = false;
        cmd_vel.id = control_id;
        local_goal_id = -1;
        risk = 0;
        enable_dwa_planner = false;
        local_goal_subscribed = false;
        local_path.poses.clear();
        current_omega = 0;
        current_velocity = 0;
        akman_cmd_pub.publish(cmd_vel);
      }
      cmd_updated = true;
      lock.unlock();
    } else {
      if (!local_goal_subscribed) {
        ROS_WARN_THROTTLE(1.0, "Local goal has not been updated");
      }
      if (!actuator_updated) {
        ROS_WARN_THROTTLE(1.0, "actuator has not been updated");
      }
      if (!laser_point_cloud_updated) {
        ROS_WARN_THROTTLE(1.0, "Laser point cloud has not been updated");
      }
    }
    // TODO(lifei) enable cmd thread
    cmd_cond_.notify_one();
    ros::spinOnce();
    loop_rate.sleep();
    ROS_INFO_STREAM_THROTTLE(1.0, "loop time: " << ros::Time::now().toSec() - start << "[s]");
  }
}

DWAPlanner::Window DWAPlanner::calc_dynamic_window(const double curr_v) {
  Window window(MIN_VELOCITY, MAX_VELOCITY, 0, 0);
  if (curr_v == 0) {
    window.min_omega = -MAX_OMEGA;
    window.max_omega = MAX_OMEGA;
  } else {
    double omega_min_limit = std::max(- MAX_OMEGA,
                                      curr_v * std::tan(MIN_GAMMA) / CAR_L);
    double omega_max_limit = std::min(MAX_OMEGA,
                                      curr_v * std::tan(MAX_GAMMA) / CAR_L);
    window.min_omega = std::min(omega_max_limit, omega_min_limit);
    window.max_omega = std::max(omega_max_limit, omega_max_limit);
  }
  ROS_INFO_STREAM_THROTTLE(5, "window velocity range[ " << window.min_velocity << ","
                           << window.max_velocity << "]");
  ROS_INFO_STREAM_THROTTLE(5, "window omega range[ " << window.min_omega << ","
                           << window.max_omega << "]");

  return window;
}

float DWAPlanner::calc_to_goal_cost(const std::vector<State> &traj, const Eigen::Vector3d &goal) {
  Eigen::Vector3d last_position(traj.back().x, traj.back().y, traj.back().yaw);
  if (last_position[0] > goal[0]) {
    return (last_position.segment(0, 2) - goal.segment(0, 2)).norm() + last_position[0] - goal[0] ;
  }
  return (last_position.segment(0, 2) - goal.segment(0, 2)).norm();
}

float DWAPlanner::calc_speed_cost(const std::vector<State> &traj, const float target_velocity) {
  float cost = fabs(target_velocity - fabs(traj[traj.size() - 1].velocity));
  return cost;
}

float DWAPlanner::calc_obstacle_cost(const std::vector<State> &traj, const std::vector<std::vector<float>> &obs_list) {
  float cost = 0.0;
  float min_dist = 1e3;
  for (const auto &state : traj) {
    for (const auto &obs : obs_list) {
      if (sqrt(state.x * state.x + state.y * state.y) < CAR_FRONT_TRACK_DIS + CAR_L) {
        TrackedArea path_tracked_area(state.x, state.y,
                                      tracked_area.min_x, tracked_area.max_x,
                                      tracked_area.min_y, tracked_area.max_y) ;
        if (path_tracked_area.IsPtInArea(obs[0], obs[1])) {
//          cost = 1e3;
//          if (state.x == state.y && state.x == 0) {
          cost = 1e6;
//          }
          return cost;
        }
      }
      float dist = sqrt((state.x - obs[0]) * (state.x - obs[0]) +
                        (state.y - obs[1]) * (state.y - obs[1]));

      min_dist = std::min(min_dist, dist);
    }
  }
  cost = 1.0 / min_dist;
  return cost;
}
//abandoned function
float DWAPlanner::calculateHeadingCost(const std::vector<State> &traj, const Eigen::Vector3d &goal) {
  Eigen::Vector3d last_position(traj.back().x, traj.back().y, traj.back().yaw);
  float dx = goal[0] - last_position[0];
  float dy = goal[1] - last_position[1];
  float angleError = atan2(dy, dx);
  float angleCost = angleError - last_position[2];
  return fabs(atan2(sin(angleCost), cos(angleCost)));
}
//TODO(lifei)改为曲率相似评价。符号相同，比较值，符号相反，符号为主
float DWAPlanner::calculatecurvatureCost(const double v, const double w) {
  double ret = 0.0;
  if (w * current_omega < 0) {
    ret = fabs(w / (v + 0.00001) * 2);
  } else {
    ret = fabs(w / (v + 0.00001) - current_omega / (current_velocity + 0.00001));
  }
  return ret;
}

void DWAPlanner::motion(State &state, const double velocity, const double omega) {
  double delta_yaw = omega * DT;
  state.velocity = velocity;
  state.omega = omega;
  if (omega == 0) {
    state.x += velocity * std::cos(state.yaw) * DT;
    state.y += velocity * std::sin(state.yaw) * DT;
    state.yaw += delta_yaw;
  } else {
    double r = velocity / omega;
    state.x += r * (-std::sin(state.yaw) + std::sin(state.yaw + delta_yaw));
    state.y += r * (std::cos(state.yaw) - std::cos(state.yaw + delta_yaw));
    state.yaw += delta_yaw;
  }
}

std::vector<std::vector<float>> DWAPlanner::laser_point_cloud_to_obs() {
  std::set<std::vector<float>> obs_list;
  std::vector<std::vector<float>> ret_obs_list;
  for (auto it : laser_point_cloud.info) {
    if (it.x == it.y && it.z == it.x && it.z == 0) continue;
    std::vector<float> obs_state = {(float)(it.x + LIDAR_2_AKMAN_OFFSET_X), float(it.y)};
    obs_list.insert(obs_state);
  }
  ret_obs_list.assign(obs_list.begin(), obs_list.end());
  return ret_obs_list;
}

std::set<std::vector<float> > DWAPlanner::laser_point_cloud_to_obs_with_filter() {
  std::set<std::vector<float>> obs_list;
  for (auto it : laser_point_cloud.info) {
    if (it.x < 0) continue;
    if (it.x == it.y && it.z == it.x && it.z == 0) continue;
    std::vector<float> obs_state = {(float)(it.x + LIDAR_2_AKMAN_OFFSET_X), float(it.y)};
    obs_list.insert(obs_state);
  }
  return obs_list;
}

void DWAPlanner::cmdThread() {
  ROS_DEBUG_NAMED("cmdThread", "Starting cmd thread...");
  ros::Rate loop_rate(HZ);
  ros::NodeHandle n;
  actuator::cmd temp_cmd;
  temp_cmd.enable = false;
  temp_cmd.id = control_id;
  temp_cmd.speed = 0;
  temp_cmd.steer = 0;
  while (n.ok()) {
    boost::unique_lock<boost::recursive_mutex> lock(wake_up_mutex_);
    while (!enable_dwa_planner) {
      if (cmd_updated) {
        boost::unique_lock<boost::recursive_mutex> lock(cmd_mutex_);
        temp_cmd = cmd_vel;
        lock.unlock();
        cmd_updated = false;
        akman_cmd_pub.publish(temp_cmd);
      }
      cmd_cond_.wait(lock);
    }

    if (cmd_updated) {
      boost::unique_lock<boost::recursive_mutex> lock(cmd_mutex_);
      temp_cmd = cmd_vel;
      lock.unlock();
      cmd_updated = false;
      visualize_local_goal(local_goal, 1, 1, 0, local_goal_pub);
      visualize_trajectories(candidate_trajectories, 0, 1, 0, 1000, candidate_trajectories_pub);
      visualize_car_mode(tracked_area, 1, 0, 0, car_traked_area_pub);
    }
    ROS_INFO_STREAM( "cmd_vel: (" << cmd_vel.speed / DWA_PLANNER_VELOCITY_FACTOR << "[m/s], " << cmd_vel.steer << "[degree])");
    akman_cmd_pub.publish(temp_cmd);
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void DWAPlanner::visualize_trajectories(const std::vector<std::vector<State>> &trajectories,
                                        const double r, const double g, const double b,
                                        const int trajectories_size, const ros::Publisher &pub) {
  visualization_msgs::MarkerArray v_trajectories;
  int count = 0;
  const int size = trajectories.size();
  for (; count < size; count++) {
    visualization_msgs::Marker v_trajectory;
    v_trajectory.header.frame_id = ROBOT_FRAME;
    v_trajectory.header.stamp = ros::Time::now();
    v_trajectory.color.r = r;
    v_trajectory.color.g = g;
    v_trajectory.color.b = b;
    v_trajectory.color.a = 0.8;
    v_trajectory.ns = pub.getTopic();
    v_trajectory.type = visualization_msgs::Marker::LINE_STRIP;
    v_trajectory.action = visualization_msgs::Marker::ADD;
    v_trajectory.lifetime = ros::Duration();
    v_trajectory.id = count;
    v_trajectory.scale.x = 0.02;
    geometry_msgs::Pose pose;
    pose.orientation.w = 1;
    v_trajectory.pose = pose;
    geometry_msgs::Point p;
    for (const auto &pose : trajectories[count]) {
      p.x = pose.x - LIDAR_2_AKMAN_OFFSET_X;
      p.y = pose.y;
      v_trajectory.points.push_back(p);
    }
    v_trajectories.markers.push_back(v_trajectory);
  }
  for (; count < trajectories_size;) {
    visualization_msgs::Marker v_trajectory;
    v_trajectory.header.frame_id = ROBOT_FRAME;
    v_trajectory.header.stamp = ros::Time::now();
    v_trajectory.ns = pub.getTopic();
    v_trajectory.type = visualization_msgs::Marker::LINE_STRIP;
    v_trajectory.action = visualization_msgs::Marker::DELETE;
    v_trajectory.lifetime = ros::Duration();
    v_trajectory.id = count;
    v_trajectories.markers.push_back(v_trajectory);
    count++;
  }
  pub.publish(v_trajectories);
}

void DWAPlanner::visualize_trajectory(const std::vector<State> &trajectory,
                                      const double r, const double g, const double b,
                                      const ros::Publisher &pub) {
  visualization_msgs::Marker v_trajectory;
  v_trajectory.header.frame_id = ROBOT_FRAME;
  v_trajectory.header.stamp = ros::Time::now();
  v_trajectory.color.r = r;
  v_trajectory.color.g = g;
  v_trajectory.color.b = b;
  v_trajectory.color.a = 0.8;
  v_trajectory.ns = pub.getTopic();
  v_trajectory.type = visualization_msgs::Marker::LINE_STRIP;
  v_trajectory.action = visualization_msgs::Marker::ADD;
  v_trajectory.lifetime = ros::Duration();
  v_trajectory.scale.x = 0.04;
  geometry_msgs::Pose pose;
  pose.orientation.w = 1;
  v_trajectory.pose = pose;
  geometry_msgs::Point p;
  for (const auto &pose : trajectory) {
    p.x = pose.x - LIDAR_2_AKMAN_OFFSET_X;
    p.y = pose.y;
    v_trajectory.points.push_back(p);
  }
  pub.publish(v_trajectory);
}

void DWAPlanner::visualize_local_goal(const geometry_msgs::PoseStamped local_goal,
                                      const double r, const double g, const double b,
                                      const ros::Publisher &pub) {
  visualization_msgs::Marker v_local_goal;
  v_local_goal.header.frame_id = ROBOT_FRAME;
  v_local_goal.header.stamp = ros::Time::now();
  v_local_goal.color.r = r;
  v_local_goal.color.g = g;
  v_local_goal.color.b = b;
  v_local_goal.color.a = 0.8;
  v_local_goal.ns = pub.getTopic();
  v_local_goal.type = visualization_msgs::Marker::POINTS;
  v_local_goal.action = visualization_msgs::Marker::ADD;
  v_local_goal.lifetime = ros::Duration();
  v_local_goal.scale.x = 0.5;
  v_local_goal.scale.y = 0.5;
  geometry_msgs::Pose pose;
  pose.orientation.w = 1;
  v_local_goal.pose = pose;
  geometry_msgs::Point p;
  p.x = local_goal.pose.position.x - LIDAR_2_AKMAN_OFFSET_X;
  p.y = local_goal.pose.position.y;
  v_local_goal.points.push_back(p);
  pub.publish(v_local_goal);
}

void DWAPlanner::visualize_car_mode(const DWAPlanner::TrackedArea &car_tracked_area,
                                    const double r, const double g, const double b,
                                    const ros::Publisher &pub) {
  visualization_msgs::Marker v_trajectory;
  v_trajectory.header.frame_id = ROBOT_FRAME;
  v_trajectory.header.stamp = ros::Time::now();
  v_trajectory.color.r = r;
  v_trajectory.color.g = g;
  v_trajectory.color.b = b;
  v_trajectory.color.a = 0.8;
  v_trajectory.ns = pub.getTopic();
  v_trajectory.type = visualization_msgs::Marker::LINE_STRIP;
  v_trajectory.action = visualization_msgs::Marker::ADD;
  v_trajectory.lifetime = ros::Duration();
  v_trajectory.scale.x = 0.1;
  geometry_msgs::Pose pose;
  pose.orientation.w = 1;
  v_trajectory.pose = pose;
  geometry_msgs::Point p_lt, p_rt, p_lb, p_rb;
  p_lt.x = car_tracked_area.max_x - LIDAR_2_AKMAN_OFFSET_X;
  p_lt.y = car_tracked_area.min_y;
  p_rt.x = car_tracked_area.max_x - LIDAR_2_AKMAN_OFFSET_X;
  p_rt.y = car_tracked_area.max_y;
  p_lb.x = car_tracked_area.min_x - LIDAR_2_AKMAN_OFFSET_X;
  p_lb.y = car_tracked_area.min_y;
  p_rb.x = car_tracked_area.min_x - LIDAR_2_AKMAN_OFFSET_X;
  p_rb.y = car_tracked_area.max_y;

  v_trajectory.points.push_back(p_lt);
  v_trajectory.points.push_back(p_rt);
  v_trajectory.points.push_back(p_rb);
  v_trajectory.points.push_back(p_lb);
  v_trajectory.points.push_back(p_lt);
  pub.publish(v_trajectory);
}

void DWAPlanner::visualize_collision_points_warning(const std::set<std::vector<float> > &collision_points,
                                                    const double r, const double g, const double b,
                                                    const ros::Publisher &pub) {
  visualization_msgs::Marker v_collision_points;
  v_collision_points.header.frame_id = ROBOT_FRAME;
  v_collision_points.header.stamp = ros::Time::now();
  v_collision_points.color.r = r;
  v_collision_points.color.g = g;
  v_collision_points.color.b = b;
  v_collision_points.color.a = 0.8;
  v_collision_points.ns = pub.getTopic();
  v_collision_points.type = visualization_msgs::Marker::POINTS;
  v_collision_points.action = visualization_msgs::Marker::ADD;
  v_collision_points.lifetime = ros::Duration();
  v_collision_points.scale.x = 0.1;
  v_collision_points.scale.y = 0.1;
  geometry_msgs::Pose pose;
  pose.orientation.w = 1;
  v_collision_points.pose = pose;
  geometry_msgs::Point p;
  for (const auto &pose : collision_points) {
    if (pose.empty()) continue;
    p.x = pose[0] - LIDAR_2_AKMAN_OFFSET_X;
    p.y = pose[1];
    v_collision_points.points.push_back(p);
  }
  pub.publish(v_collision_points);

}
