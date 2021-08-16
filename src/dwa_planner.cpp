#include "dwa_planner/dwa_planner.h"

DWAPlanner::DWAPlanner(void)
  : local_nh("~"), local_goal_subscribed(false), laser_point_cloud_updated(false), actuator_updated(false) {
  local_nh.param("HZ", HZ, {20});
  local_nh.param("ROBOT_FRAME", ROBOT_FRAME, {"base_link"});
  local_nh.param("TARGET_VELOCITY", TARGET_VELOCITY, {1.388889});
  local_nh.param("MAX_VELOCITY", MAX_VELOCITY, {1.5});
  local_nh.param("MIN_VELOCITY", MIN_VELOCITY, {0.0});
  local_nh.param("MAX_GAMMA", MAX_GAMMA, {0.52});
  local_nh.param("MIN_GAMMA", MIN_GAMMA, {-0.52});
  local_nh.param("MAX_OMEGA", MAX_OMEGA, {1.0});
  local_nh.param("MAX_ACCELERATION", MAX_ACCELERATION, {1.0});
  local_nh.param("MAX_DIST", MAX_DIST, {10.0});
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
  DT = 1.0 / HZ;

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
  akman_cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  candidate_trajectories_pub = local_nh.advertise<visualization_msgs::MarkerArray>("candidate_trajectories", 1);
  selected_trajectory_pub = local_nh.advertise<visualization_msgs::Marker>("selected_trajectory", 1);

  local_path_sub = nh.subscribe("/planner", 1, &DWAPlanner::local_path_callback, this);
  laser_point_cloud_sub = nh.subscribe("/perception_info", 1, &DWAPlanner::laser_point_cloud_callback, this);
  actuator_sub = nh.subscribe("/actuator", 1, &DWAPlanner::actuator_callback, this);
  target_velocity_sub = nh.subscribe("/target_velocity", 1, &DWAPlanner::target_velocity_callback, this);
}

DWAPlanner::State::State(double _x, double _y, double _yaw, double _velocity, double _omega, double _gamma)
  : x(_x), y(_y), yaw(_yaw), velocity(_velocity), omega(_omega), gamma(_gamma) {
}

DWAPlanner::Window::Window(void)
  : min_velocity(0.0), max_velocity(0.0), min_omega(0.0), max_omega(0.0) {
}

DWAPlanner::Window::Window(const double min_v, const double max_v, const double min_omega, const double max_omega)
  : min_velocity(min_v), max_velocity(max_v), min_omega(min_omega), max_omega(max_omega) {
}

void DWAPlanner::local_path_callback(const planner::plannerConstPtr &msg) {
  local_path.poses.clear();

// 使用半径参数30m进行local goal的获取
  for (unsigned i = msg->startPoint; i < msg->points.size() ; i++) {
    geometry_msgs::Point P0;
    P0.x = msg->points[i].x;
    P0.y = msg->points[i].y;
    geometry_msgs::PoseStamped pose_tmp;
    if (sqrt(P0.x * P0.x  + P0.y * P0.y) <= 30) {
      pose_tmp.pose.position.x = P0.x;
      pose_tmp.pose.position.y = P0.y;
      local_path.poses.push_back(pose_tmp);
    } else {
      break;
    }
  }
  if (!local_path.poses.empty()) {
    local_path = calculatePathYaw(local_path);
    local_goal = local_path.poses.back();
    local_goal_subscribed = true;
  }
}

void DWAPlanner::laser_point_cloud_callback(const perception_msgs::PerceptionConstPtr &msg) {
  laser_point_cloud = *msg;
  laser_point_cloud_updated = true;
}
//TODO(lifei)
void DWAPlanner::actuator_callback(const actuator::actuatorConstPtr &msg) {
  double current_gamma = msg->steer;
  current_velocity = msg->speed / 3.6; //m/s
  current_omega = current_velocity * std::tan(current_gamma) / CAR_L;
  actuator_updated = true;
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
      State state(0.0, 0.0, 0.0, current_velocity, current_omega, 0);
      std::vector<State> traj;
      for (float t = 0; t <= PREDICT_TIME; t += DT) {
        motion(state, v, omega);
        traj.push_back(state);
      }
      trajectories.push_back(traj);

      float to_goal_cost = calc_to_goal_cost(traj, goal);
      float speed_cost = calc_speed_cost(traj, current_velocity);
      float heading_cost = calculateHeadingCost(traj, goal);
      float obstacle_cost = calc_obstacle_cost(traj, obs_list);
      float final_cost = TO_GOAL_COST_GAIN * to_goal_cost + SPEED_COST_GAIN * speed_cost +
                         OBSTACLE_COST_GAIN * obstacle_cost + HEAD_COST_GAIN * heading_cost;
      if (min_cost >= final_cost) {
        min_goal_cost = TO_GOAL_COST_GAIN * to_goal_cost;
        min_obs_cost = OBSTACLE_COST_GAIN * obstacle_cost;
        min_speed_cost = SPEED_COST_GAIN * speed_cost;
        min_head_cost = HEAD_COST_GAIN * heading_cost;
        min_cost = final_cost;
        best_traj = traj;
      }
    }
  }
  ROS_INFO_STREAM("Cost: " << min_cost);
  ROS_INFO_STREAM("- Goal cost: " << min_goal_cost);
  ROS_INFO_STREAM("- Obs cost: " << min_obs_cost);
  ROS_INFO_STREAM("- Speed cost: " << min_speed_cost);
  ROS_INFO_STREAM("- Head cost: " << min_head_cost);
  ROS_INFO_STREAM("num of trajectories: " << trajectories.size());
  visualize_trajectories(trajectories, 0, 1, 0, 1000, candidate_trajectories_pub);
  if (min_cost == 1e6) {
    std::vector<State> traj;
    State state(0.0, 0.0, 0.0, current_velocity, current_omega, 0);
    traj.push_back(state);
    best_traj = traj;
  }
  return best_traj;
}

nav_msgs::Path DWAPlanner::calculatePathYaw(nav_msgs::Path path_in) {
  int length = path_in.poses.size();
  if (length <= 1) {
    if (length == 1)
      path_in.poses[0].pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    return path_in;
  }

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
      ROS_INFO_STREAM("local goal: (" << goal[0] << "," << goal[1] << "," << goal[2] / M_PI * 180 << ")");

      geometry_msgs::Twist cmd_vel;
      if (goal.segment(0, 2).norm() > GOAL_THRESHOLD) {
        std::vector<std::vector<float>> obs_list;
        obs_list = laser_point_cloud_to_obs();
        laser_point_cloud_updated = false;

        std::vector<State> best_traj = dwa_planning(dynamic_window, goal, obs_list);
        //TODO(lifei) using omega to generating the gamma
        cmd_vel.linear.x = best_traj[0].velocity;
        cmd_vel.angular.z = best_traj[0].omega;
        visualize_trajectory(best_traj, 1, 0, 0, selected_trajectory_pub);
      } else {
        cmd_vel.linear.x = 0.0;
        if (fabs(goal[2]) > TURN_DIRECTION_THRESHOLD) {
          cmd_vel.angular.z = std::min(std::max(goal(2), -MAX_GAMMA), MAX_GAMMA);
        } else {
          cmd_vel.angular.z = 0.0;
        }
      }
      ROS_INFO_STREAM("cmd_vel: (" << cmd_vel.linear.x << "[m/s], " << cmd_vel.angular.z << "[rad/s])");
      akman_cmd_pub.publish(cmd_vel);

      actuator_updated = false;
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
    ros::spinOnce();
    loop_rate.sleep();
    ROS_INFO_STREAM("loop time: " << ros::Time::now().toSec() - start << "[s]");
  }
}
// TODO(lifei) use ackman model,angle_r velocity
DWAPlanner::Window DWAPlanner::calc_dynamic_window(const double curr_v) {
  double omega_min_limit = curr_v * std::tan(MIN_GAMMA) / CAR_L;
  double omega_max_limit = curr_v * std::tan(MAX_GAMMA) / CAR_L;
  Window window(MIN_VELOCITY, MAX_VELOCITY, omega_min_limit, omega_max_limit);

  window.min_omega = std::max((current_omega - MAX_OMEGA * DT), omega_min_limit);
  window.max_omega = std::min((current_omega + MAX_OMEGA * DT), omega_max_limit);

  window.min_velocity = std::max((curr_v - MAX_ACCELERATION * DT), MIN_VELOCITY);
  window.max_velocity = std::min((curr_v + MAX_ACCELERATION * DT), MAX_VELOCITY);
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
      float dist_end = sqrt((state.x - obs[0]) * (state.x - obs[0]) +
                            (state.y - obs[1]) * (state.y - obs[1]));
      if (dist_end <= DETECT_OBSTACLE_DIS_THR) {
        cost = 1e6;
        return cost;
      }
      State state_front = state;
      state_front.x += CAR_L;
      float dist_front = sqrt((state_front.x - obs[0]) * (state_front.x - obs[0]) +
                              (state_front.y - obs[1]) * (state_front.y - obs[1]));
      if (dist_front <= DETECT_OBSTACLE_DIS_THR) {
        cost = 1e6;
        return cost;
      }

      min_dist = std::min(min_dist, std::min(dist_front, dist_end));
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
  if (fabs(omega) == 0) {
    state.x += velocity * std::cos(state.yaw) * DT;
    state.y += velocity * std::sin(state.yaw) * DT;
    state.yaw += delta_yaw;
  } else {
    double r = velocity / omega;
    state.x += r * (std::sin(state.yaw) - std::sin(state.yaw + delta_yaw));
    state.y += -r * (std::cos(state.yaw) - std::cos(state.yaw + delta_yaw));
    state.yaw += delta_yaw;
  }
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

void DWAPlanner::visualize_trajectories(const std::vector<std::vector<State>> &trajectories, const double r, const double g, const double b, const int trajectories_size, const ros::Publisher &pub) {
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

void DWAPlanner::visualize_trajectory(const std::vector<State> &trajectory, const double r, const double g, const double b, const ros::Publisher &pub) {
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
  v_trajectory.scale.x = 0.05;
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
