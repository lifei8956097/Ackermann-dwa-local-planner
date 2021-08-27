#include "dwa_planner/dwa_planner.h"
#include <boost/thread.hpp>
#define DWA_PLANNER_VELOCITY_FACTOR 50
DWAPlanner::DWAPlanner(void)
  : local_nh("~"), local_goal_subscribed(false), laser_point_cloud_updated(false), actuator_updated(false) {
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
  local_nh.param("GOAL_THRESHOLD", GOAL_THRESHOLD, {0.3});
  local_nh.param("TURN_DIRECTION_THRESHOLD", TURN_DIRECTION_THRESHOLD, {1.0});
  local_nh.param("CAR_L", CAR_L, {0.8633246});
  local_nh.param("CAR_W", CAR_W, {1.036});
  DETECT_OBSTACLE_DIS_THR = CAR_W * 0.5 + 0.1;
  CAR_FRONT_TRACK_DIS = 1; //m
  DT = 1.0 / HZ;
  MAX_OMEGA = MAX_GAMMA / DT;
  risk = 0;
  cmd_updated = false;
  current_gamma_arc = 0;
  tracked_area.min_x = - CAR_FRONT_TRACK_DIS;
  tracked_area.max_x = CAR_L + CAR_FRONT_TRACK_DIS;
  tracked_area.min_y = -CAR_W * 0.5;
  tracked_area.max_y = CAR_W * 0.5;

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
  ROS_INFO_STREAM("GOAL_THRESHOLD: " << GOAL_THRESHOLD);
  ROS_INFO_STREAM("TURN_DIRECTION_THRESHOLD: " << TURN_DIRECTION_THRESHOLD);
  ROS_INFO_STREAM("CAR_L: " << CAR_L);
  ROS_INFO_STREAM("CAR_W:" << CAR_W);
  akman_cmd_pub = nh.advertise<actuator::cmd>("/takeOver", 1);
  candidate_trajectories_pub = local_nh.advertise<visualization_msgs::MarkerArray>("candidate_trajectories", 1);
  selected_trajectory_pub = local_nh.advertise<visualization_msgs::Marker>("selected_trajectory", 1);
  local_goal_pub = local_nh.advertise<visualization_msgs::Marker>("dwa_local_goal", 1);

  local_path_sub = nh.subscribe("/planner", 1, &DWAPlanner::local_path_callback, this);
  laser_point_cloud_sub = nh.subscribe("/perception_info", 1, &DWAPlanner::laser_point_cloud_callback, this);
  actuator_sub = nh.subscribe("/actuator", 1, &DWAPlanner::actuator_callback, this);
  target_velocity_sub = nh.subscribe("/target_velocity", 1, &DWAPlanner::target_velocity_callback, this);
//set up the planner's thread
  cmd_thread_ = new boost::thread(boost::bind(&DWAPlanner::cmdThread, this));
}

DWAPlanner::~DWAPlanner() {
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
  : min_x(0.0), max_x(0.0), min_y(0.0), max_y(0.0) {
}

DWAPlanner::TrackedArea::TrackedArea(const double min_xx, const double max_xx, const double min_yy, const double max_yy)
  : min_x(min_xx), max_x(max_xx), min_y(min_yy), max_y(max_yy) {
}

bool DWAPlanner::TrackedArea::IsPtInArea(const double x, const double y) {
  if (x >= min_x && x <= max_x && y >= min_y && y <= max_y) return true;
  return false;
}

void DWAPlanner::local_path_callback(const planner::plannerConstPtr &msg) {
  local_path.poses.clear();
  if (local_goal_id == -1) {
// 使用半径参数10m进行local goal的获取
    unsigned int i = msg->startPoint;
    for (; i < msg->points.size() ; i++) {
      geometry_msgs::Point P0;
      P0.x = msg->points[i].x;
      P0.y = msg->points[i].y;
      geometry_msgs::PoseStamped pose_tmp;
      if (sqrt(P0.x * P0.x  + P0.y * P0.y) <= 10) {
        pose_tmp.pose.position.x = P0.x;
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
        geometry_msgs::PoseStamped pose_tmp;
        pose_tmp.pose.position.x = msg->points[i].x;
        pose_tmp.pose.position.y = msg->points[i].y;
        local_goal = pose_tmp;
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
//TODO(lifei)
void DWAPlanner::actuator_callback(const actuator::actuatorConstPtr &msg) {
  if ( msg->risk == 1) {
    risk = msg->risk;
  }
  if (risk == 1) {
    current_gamma_arc = msg->steer / 180 * M_PI;
    current_velocity = msg->speed / 3.6; //m/s
    current_omega = current_velocity * std::tan(current_gamma_arc ) / CAR_L;
    actuator_updated = true;
  }
}

void DWAPlanner::target_velocity_callback(const geometry_msgs::TwistConstPtr &msg) {
  TARGET_VELOCITY = msg->linear.x;
  ROS_INFO_STREAM("target velocity was updated to " << TARGET_VELOCITY << "[m/s]");
}

std::vector<DWAPlanner::State> DWAPlanner::dwa_planning(
  Window dynamic_window,
  Eigen::Vector3d goal,
  std::vector<std::vector<float>> obs_list) {
  float min_cost = 1e6;
  float min_obs_cost = min_cost;
  float min_goal_cost = min_cost;
  float min_speed_cost = min_cost;
  float min_head_cost = min_cost;
  std::vector<std::vector<State>> trajectories;
  std::vector<State> best_traj;


  for (float v = dynamic_window.min_velocity; v <= dynamic_window.max_velocity; v += VELOCITY_RESOLUTION) {
    for (float omega = dynamic_window.min_omega; omega <= dynamic_window.max_omega; omega += OMEGA_RESOLUTION) {
      State state(0.0, 0.0, 0.0, current_velocity, current_omega);
      std::vector<State> traj;
      for (float t = 0; t <= PREDICT_TIME; t += DT) {
        motion(state, v, omega);
        // TODO(lifei) 选择角度在30内，距离在MAX_DIST内的路径，
        if (state.x != 0 && state.y != 0) {
          if ((fabs(std::atan2(state.y, state.x)) > 0.52) ||
              std::sqrt(state.x * state.x + state.y * state.y) > MAX_DIST) break;
        }
        traj.push_back(state);
      }
      trajectories.push_back(traj);

      float to_goal_cost = calc_to_goal_cost(traj, goal);
      float speed_cost = calc_speed_cost(traj, current_velocity);
//      float heading_cost = calculateHeadingCost(traj, goal);
      float obstacle_cost = calc_obstacle_cost(traj, obs_list);
      float final_cost = TO_GOAL_COST_GAIN * to_goal_cost + SPEED_COST_GAIN * speed_cost +
                         OBSTACLE_COST_GAIN * obstacle_cost /*+ HEAD_COST_GAIN * heading_cost*/;
      if (min_cost >= final_cost) {
        min_goal_cost = TO_GOAL_COST_GAIN * to_goal_cost;
        min_obs_cost = OBSTACLE_COST_GAIN * obstacle_cost;
        min_speed_cost = SPEED_COST_GAIN * speed_cost;
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
//  ROS_INFO_STREAM("- Head cost: " << min_head_cost);
  ROS_INFO_STREAM("num of trajectories: " << trajectories.size());
  candidate_trajectories = trajectories;
//  visualize_trajectories(trajectories, 0, 1, 0, 1000, candidate_trajectories_pub);
  if (min_cost == 1e6) {
    std::vector<State> traj;
    State state(0.0, 0.0, 0.0, current_velocity, current_omega);
    traj.push_back(state);
    best_traj = traj;
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
  //TODO(lifei) only calculate the last one yaw
  for (int i = 0; i < length - 1; ++i) {
    double dx = path_in.poses[i + 1].pose.position.x - path_in.poses[i].pose.position.x;
    double dy = path_in.poses[i + 1].pose.position.y - path_in.poses[i].pose.position.y;
    double theta = atan2(dy, dx);
    path_in.poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(theta);
  }

  path_in.poses.back().pose.orientation = path_in.poses[length - 2].pose.orientation;

  return path_in;
}

void DWAPlanner::process(void) {
  ros::Rate loop_rate(HZ);
  while (ros::ok()) {
    ROS_INFO("==========================================");
    double start = ros::Time::now().toSec();
    bool input_updated = false;
    if (laser_point_cloud_updated) {
      input_updated = true;
    }
    if (input_updated && local_goal_subscribed && actuator_updated) {
      Window dynamic_window = calc_dynamic_window(current_velocity);
      Eigen::Vector3d goal(local_goal.pose.position.x, local_goal.pose.position.y, tf::getYaw(local_goal.pose.orientation));
      ROS_INFO_STREAM_THROTTLE(5, "local goal: (" << goal[0] << "," << goal[1] << "," << goal[2] / M_PI * 180 << ")");
      boost::unique_lock<boost::recursive_mutex> lock(cmd_mutex_);
      cmd_vel.enable = true;
      cmd_vel.id = control_id;
      if (goal.segment(0, 2).norm() > GOAL_THRESHOLD) {
        std::vector<std::vector<float>> obs_list;
        obs_list = laser_point_cloud_to_obs();
        laser_point_cloud_updated = false;

        std::vector<State> best_traj = dwa_planning(dynamic_window, goal, obs_list);
        cmd_vel.speed = best_traj[0].velocity * DWA_PLANNER_VELOCITY_FACTOR;
        cmd_vel.steer = std::atan(CAR_L * best_traj[0].omega /
                                  (best_traj[0].velocity + 0.001)) / M_PI * 180;
        visualize_trajectory(best_traj, 1, 0, 0, selected_trajectory_pub);
      } else {
        cmd_vel.speed = 0.0;
        if (fabs(goal[2]) > TURN_DIRECTION_THRESHOLD) {
          cmd_vel.steer = std::min(std::max((goal(2) - current_gamma_arc) / M_PI * 180, MIN_GAMMA), MAX_GAMMA);
        } else {
          cmd_vel.steer = 0.0;
          cmd_vel.enable = false;
          cmd_vel.id = control_id;
          local_goal_id = -1;
          risk = 0;
        }
      }
      ROS_INFO_STREAM_THROTTLE(5, "cmd_vel: (" << cmd_vel.speed / DWA_PLANNER_VELOCITY_FACTOR << "[m/s], " << cmd_vel.steer << "[degree])");
//      akman_cmd_pub.publish(cmd_vel);
      cmd_updated = true;
      actuator_updated = false;
      lock.unlock();
    } else {
      ROS_INFO_STREAM("waiting for data");
      if (risk == 1) {
        if (!local_goal_subscribed) {
          ROS_WARN_THROTTLE(1.0, "Local goal has not been updated");
        }
        if (!actuator_updated) {
          ROS_WARN_THROTTLE(1.0, "actuator has not been updated");
        }
        if (!laser_point_cloud_updated) {
          ROS_WARN_THROTTLE(1.0, "Laser point cloud has not been updated");
        }
      } else {
        ROS_INFO_STREAM("not enable the dwa planner;risk:" << risk);
      }
    }
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

//  window.min_velocity = std::max((curr_v - MAX_ACCELERATION * DT), MIN_VELOCITY);
//  window.max_velocity = std::min((curr_v + MAX_ACCELERATION * DT), MAX_VELOCITY);
  return window;
}

float DWAPlanner::calc_to_goal_cost(const std::vector<State> &traj, const Eigen::Vector3d &goal) {
  Eigen::Vector3d last_position(traj.back().x, traj.back().y, traj.back().yaw);
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
//      //1
//      float dist_end = sqrt((state.x - obs[0]) * (state.x - obs[0]) +
//                            (state.y - obs[1]) * (state.y - obs[1]));
//      if (dist_end <= DETECT_OBSTACLE_DIS_THR) {
//        cost = 1e6;
//        return cost;
//      }
//      //2
//      State state_front = state;
//      state_front.x += CAR_L;
//      float dist_front = sqrt((state_front.x - obs[0]) * (state_front.x - obs[0]) +
//                              (state_front.y - obs[1]) * (state_front.y - obs[1]));
//      if (dist_front <= DETECT_OBSTACLE_DIS_THR) {
//        cost = 1e6;
//        return cost;
//      }
      //3 track area rectangle
      if (tracked_area.IsPtInArea(obs[0], obs[1])) {
        cost = 1e6;
        return cost;
      }
      float dist = sqrt((state.x - obs[0]) * (state.x - obs[0]) +
                        (state.y - obs[1]) * (state.y - obs[1]));

      min_dist = std::min(min_dist, dist);
    }
  }
  cost = 1.0 / min_dist;
  return cost;
}

float DWAPlanner::calculateHeadingCost(const std::vector<State> &traj, const Eigen::Vector3d &goal) {
  Eigen::Vector3d last_position(traj.back().x, traj.back().y, traj.back().yaw);
  float dx = goal[0] - last_position[0];
  float dy = goal[1] - last_position[1];
  float angleError = atan2(dy, dx);
  float angleCost = angleError - last_position[2];
  return fabs(atan2(sin(angleCost), cos(angleCost)));
}

void DWAPlanner::motion(State &state, const double velocity, const double omega) {
  double delta_yaw = omega * DT;
  state.velocity = velocity;
  state.omega = omega;
  state.x += velocity * std::cos(state.yaw) * DT;
  state.y += velocity * std::sin(state.yaw) * DT;
  state.yaw += delta_yaw;

//  if (omega == 0) {
//    state.x += velocity * std::cos(state.yaw) * DT;
//    state.y += velocity * std::sin(state.yaw) * DT;
//    state.yaw += delta_yaw;
//  } else {
//    double r = velocity / omega;
//    state.x += r * (-std::sin(state.yaw) + std::sin(state.yaw + delta_yaw));
//    state.y += r * (std::cos(state.yaw) - std::cos(state.yaw + delta_yaw));
//    state.yaw += delta_yaw;
//  }
}

std::vector<std::vector<float>> DWAPlanner::laser_point_cloud_to_obs() {
  std::vector<std::vector<float>> obs_list;
  for (auto it : laser_point_cloud.info) {
    if (it.x == it.y && it.z == it.x && it.z == 0) continue;
    std::vector<float> obs_state = {it.x, it.y};
    obs_list.push_back(obs_state);
  }
  return obs_list;
}

void DWAPlanner::cmdThread() {
  ROS_DEBUG_NAMED("cmdThread", "Starting cmd thread...");
  ros::Rate loop_rate(HZ);
  ros::NodeHandle n;
  actuator::cmd temp_cmd;
  temp_cmd.enable = true;
  temp_cmd.id = control_id;
  temp_cmd.speed = 0;
  temp_cmd.steer = 0;
  while (n.ok()) {
    if (cmd_updated) {
      boost::unique_lock<boost::recursive_mutex> lock(cmd_mutex_);
      temp_cmd = cmd_vel;
      lock.unlock();
      cmd_updated = false;
      visualize_local_goal(local_goal, 1, 1, 0, local_goal_pub);
      visualize_trajectories(candidate_trajectories, 0, 1, 0, 1000, candidate_trajectories_pub);
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
      p.x = pose.x;
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
    p.x = pose.x;
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
  v_local_goal.scale.x = 0.3;
  v_local_goal.scale.y = 0.3;
  geometry_msgs::Pose pose;
  pose.orientation.w = 1;
  v_local_goal.pose = pose;
  geometry_msgs::Point p;
  p.x = local_goal.pose.position.x;
  p.y = local_goal.pose.position.y;
  v_local_goal.points.push_back(p);
  pub.publish(v_local_goal);
}
