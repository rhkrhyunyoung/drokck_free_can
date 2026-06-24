#include "MAIN.h"
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <algorithm>
#include <cmath>

#ifndef D2R
#define D2R (M_PI / 180.0)
#endif

// 🟢 enum을 MainActiveNode 내부가 아닌 독립 전역형태로 선언하여 컴파일 에러 원천 차단
enum class MyMissionState {
  TRACK_DRIVING,       
  TRAFFIC_STOP,        
  CARGO_WAIT,          
  MARKER_SLOWDOWN      
};

static MyMissionState current_mission_state_ = MyMissionState::TRACK_DRIVING;

// 전역 정적 변수 선언 (MAIN.h의 멤버변수 제약 우회)
static rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr local_imu_sub_;
static double local_current_yaw_rate_ = 0.0;
static bool local_is_cargo_stopped_ = false;
static bool local_is_slipping_ = false;
static double local_torque_multiplier_ = 1.0;
static rclcpp::Time local_slip_start_time_;
static rclcpp::Time local_cargo_stop_start_time_;

int main(int argc, char **argv) {
  printf("Init Simple Track Main with Objective Mission Manager ...\n");
  sleep(1);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MainActiveNode>());
  rclcpp::shutdown();
  std::cout << "rclcpp::shutdown done" << std::endl;
  return 0;
}

MainActiveNode::MainActiveNode() : Node("Main_Active_Track_1_4") {
  ThreadConfig thread_config = loadThreadConfig();
  CANConfig can_config = loadCANConfig();

  Publish_timer = this->create_wall_timer(
      10ms, std::bind(&MainActiveNode::publishMessage, this));
                    
  joystick_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy_vehicle", 10,
      std::bind(&MainActiveNode::joy_callback, this, std::placeholders::_1));

  command_mode = RPM_COMMAND;

  for (const auto& interface : can_config.interfaces) {
    if (interface.id < NUM_OF_CANABLE) {
      can_socks[interface.id] = CAN_INIT(interface.name);
    }
  }

  for (int i = 0; i < NUM_OF_CANABLE; i++) {
    for (int j = 0; j < NUM_OF_MOTOR; j++) {
      std::string topic_torque = "can_" + std::to_string(i) + "_motor_" + std::to_string(j) + "_torque";
      std::string topic_tau = "can_" + std::to_string(i) + "_motor_" + std::to_string(j) + "_tau";
      std::string topic_speed = "can_" + std::to_string(i) + "_motor_" + std::to_string(j) + "_speed";
      std::string topic_theta = "can_" + std::to_string(i) + "_motor_" + std::to_string(j) + "_theta";

      torque_publishers[i][j] = this->create_publisher<std_msgs::msg::Int16>(topic_torque, 10);
      tau_publishers[i][j] = this->create_publisher<std_msgs::msg::Int16>(topic_tau, 10);
      speed_publishers[i][j] = this->create_publisher<std_msgs::msg::Int16>(topic_speed, 10);
      theta_publishers[i][j] = this->create_publisher<std_msgs::msg::Float32>(topic_theta, 10);
    }
  }

  pthread_attr_t attr;
  pthread_attr_init(&attr);
  pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
  pthread_attr_setschedpolicy(&attr, thread_config.policy);

  struct sched_param param;
  param.sched_priority = thread_config.priority;
  pthread_attr_setschedparam(&attr, &param);

  if (pthread_create(&can_thread, &attr, &MainActiveNode::CAN_Thread_wrapper, this) != 0) {
    perror("Thread creation failed");
  }

  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(thread_config.core, &cpuset);
  if (pthread_setaffinity_np(can_thread, sizeof(cpu_set_t), &cpuset) != 0) {
    perror("Thread affinity set failed");
  }

  pthread_attr_destroy(&attr);
  printf("Thread creation and configuration complete.\n");

  nav_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&MainActiveNode::navCmdCallback, this, std::placeholders::_1));

  yolo_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/yolo_detected_object", 10, std::bind(&MainActiveNode::yoloCallback, this, std::placeholders::_1));
      
  yolo_xyz_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/yolo_object_xyz", 10, [this](const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
        this->obj_x_ = msg->vector.y;
        this->obj_z_ = msg->vector.x;
      });

  // 멤버 변수 대신 전역 정적 포인터를 활용해 IMU 바인딩 오류 우회
  local_imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", 10, [](const sensor_msgs::msg::Imu::SharedPtr msg) {
        local_current_yaw_rate_ = msg->angular_velocity.z;
      });

  track_width_ = 0.55;       
  sprocket_radius_ = 0.08;   
  gear_ratio_ = 36.0;        
  rmd_unit_scale_ = 600.0;   

  is_autonomous_mode = true; 
  last_vision_time_ = this->now(); 

  local_is_cargo_stopped_ = false;
  local_is_slipping_ = false;
  local_torque_multiplier_ = 1.0;
  local_current_yaw_rate_ = 0.0;
}

MainActiveNode::~MainActiveNode() {
  pthread_cancel(can_thread);
  pthread_join(can_thread, NULL);
  for (int i = 0; i < NUM_OF_CANABLE; i++) {
    close(can_socks[i]);
  }
}

void *MainActiveNode::CAN_Thread_wrapper(void *arg) {
  static_cast<MainActiveNode *>(arg)->CAN_Thread();
  return NULL;
}

void MainActiveNode::CAN_Thread() {
  struct can_frame frame;
  while (rclcpp::ok()) {
    READ_BUFFER();
    Check_State();
    ComputeTorque();

    for (int i = 0; i < NUM_OF_CANABLE; i++) {
      if (command_mode == RPM_COMMAND) {
        if (i == 0) {
          Rmd.RPM_CLOSED_LOOP_CONTROL(can_socks[i], 1, left_speed, frame);
          usleep(10);
          Rmd.RPM_CLOSED_LOOP_CONTROL(can_socks[i], 2, left_speed, frame);
          usleep(10);
        } else if (i == 3) {
          Rmd.RPM_CLOSED_LOOP_CONTROL(can_socks[i], 1, right_speed, frame);
          usleep(10);
          Rmd.RPM_CLOSED_LOOP_CONTROL(can_socks[i], 2, right_speed, frame);
          usleep(10);
        }
      }
    }
    usleep(1000);
  }
}

int MainActiveNode::CAN_INIT(const std::string &can_interface_name) {
  int s; struct sockaddr_can addr; struct ifreq ifr;
  if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) { perror("Socket"); return -1; }
  strcpy(ifr.ifr_name, can_interface_name.c_str()); ioctl(s, SIOCGIFINDEX, &ifr);
  memset(&addr, 0, sizeof(addr)); addr.can_family = AF_CAN; addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) { perror("Bind"); return -1; }
  return s;
}

void MainActiveNode::READ_BUFFER() {
  struct can_frame frame; int nbytes;
  for (int i = 0; i < NUM_OF_CANABLE; i++) {
    while ((nbytes = recv(can_socks[i], &frame, sizeof(struct can_frame), MSG_DONTWAIT)) > 0) {
      READ_TORQUE_SPEED_ANGLE_X4(can_socks[i], i, frame);
    }
  }
}

void MainActiveNode::READ_TORQUE_SPEED_ANGLE_X4(int s, int canable, struct can_frame frame) {
  (void)s; int16_t torque = 0; int16_t speed = 0; int16_t degree = 0; int32_t output_degree = 0;
  if (frame.data[0] == 0x9C) {
    torque = (frame.data[3] << 8) | frame.data[2]; speed = (frame.data[5] << 8) | frame.data[4]; degree = (frame.data[7] << 8) | frame.data[6]; output_degree = (int32_t)degree;
    if (frame.can_id == 0x241) {
      if (canable == 0) { motor_state[canable].torque[0] = torque; motor_state[canable].speed[0] = speed * D2R / 36; motor_state[canable].theta[0] = (output_degree) * D2R; }
      else if (canable == 3) { motor_state[canable].torque[0] = torque; motor_state[canable].speed[0] = speed * D2R / 36; motor_state[canable].theta[0] = (output_degree) * D2R; }
    } else if (frame.can_id == 0x242) {
      if (canable == 0) { motor_state[canable].torque[1] = torque; motor_state[canable].speed[1] = speed * D2R / 36; motor_state[canable].theta[1] = (output_degree) * D2R; }
      else if (canable == 3) { motor_state[canable].torque[1] = torque; motor_state[canable].speed[1] = speed * D2R / 36; motor_state[canable].theta[1] = (output_degree) * D2R; }
    }
  }
}

void MainActiveNode::publishMessage() {
  for (int i = 0; i < NUM_OF_CANABLE; i++) {
    for (int j = 0; j < NUM_OF_MOTOR; j++) {
      auto msg_torque = std::make_unique<std_msgs::msg::Int16>(); msg_torque->data = motor_state[i].torque[j]; torque_publishers[i][j]->publish(std::move(msg_torque));
      auto msg_tau = std::make_unique<std_msgs::msg::Int16>(); msg_tau->data = tau[i][j]; tau_publishers[i][j]->publish(std::move(msg_tau));
      auto msg_speed = std::make_unique<std_msgs::msg::Int16>(); msg_speed->data = motor_state[i].speed[j]; speed_publishers[i][j]->publish(std::move(msg_speed));
      auto msg_theta = std::make_unique<std_msgs::msg::Float32>(); msg_theta->data = motor_state[i].theta[j]; theta_publishers[i][j]->publish(std::move(msg_theta));
    }
  }
}

void MainActiveNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
  joystick_data_ = msg; last_joystick_time_= std::chrono::steady_clock::now();
  if (std::abs(msg->axes[1]) > 0.05 || std::abs(msg->axes[3]) > 0.05) {
    is_autonomous_mode = false; left_speed = msg->axes[1] * 3000; right_speed = msg->axes[3] * 3000;
  }
}

void MainActiveNode::Check_State() { (void)Rmd; }
void MainActiveNode::ComputeTorque() { (void)Rmd; }
void MainActiveNode::EMERGENCY_STOP() {}
float MainActiveNode::func_1_cos(float t, float init, float final, float T) { (void)t; (void)init; (void)final; (void)T; return 0; }
float MainActiveNode::dt_func_1_cos(float t, float init, float final, float T) { (void)t; (void)init; (void)final; (void)T; return 0; }
MainActiveNode::ThreadConfig MainActiveNode::loadThreadConfig() { return ThreadConfig{2, SCHED_FIFO, 80}; }
MainActiveNode::CANConfig MainActiveNode::loadCANConfig() { CANConfig config; config.interfaces = {{"can0", 0}, {"can3", 3}}; return config; }

void MainActiveNode::navCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  if (is_autonomous_mode) {
    nav_linear_x_ = msg->linear.x;
    nav_angular_z_ = msg->angular.z;
    decisionMakingTree();
  }
}

void MainActiveNode::yoloCallback(const std_msgs::msg::String::SharedPtr msg) {
  detected_class_ = msg->data;
  last_vision_time_ = this->now();
}

void MainActiveNode::decisionMakingTree() {
  double vision_dt = (this->now() - last_vision_time_).seconds();

  if (detected_class_ == "STOP" && vision_dt < 1.0) {
    current_mission_state_ = MyMissionState::TRAFFIC_STOP;
    executeStateBehavior();
    return; 
  }

  if (detected_class_ == "GO" && vision_dt < 1.0) {
    current_mission_state_ = MyMissionState::TRACK_DRIVING;
    executeStateBehavior();
    return;
  }

  if (detected_class_ == "cargo_zone") {
    current_mission_state_ = MyMissionState::CARGO_WAIT;
    executeStateBehavior();
    return;
  }

  if (detected_class_ == "vision_marker") {
    current_mission_state_ = MyMissionState::MARKER_SLOWDOWN;
    executeStateBehavior();
    return;
  }

  current_mission_state_ = MyMissionState::TRACK_DRIVING;
  executeStateBehavior();
}

void MainActiveNode::executeStateBehavior() {
  rclcpp::Time current_time = this->get_clock()->now();
  
  if (std::abs(nav_linear_x_) > 0.05 && std::abs(nav_angular_z_ - local_current_yaw_rate_) > 0.3) {
    if (!local_is_slipping_) {
      local_is_slipping_ = true;
      local_slip_start_time_ = current_time;
    } else {
      double slip_duration = (current_time - local_slip_start_time_).seconds();
      if (slip_duration >= 6.0) {
        local_torque_multiplier_ = 3.0; 
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "🚨 [STUCK] 궤도 고립 상태! 고토크 기어 3.0배 구동!");
      } else if (slip_duration >= 3.0) {
        local_torque_multiplier_ = 2.0; 
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "⚠️ [SLIP] 바퀴 헛돎 발생! 토크 2.0배 구동!");
      }
    }
  } else {
    local_is_slipping_ = false;
    local_torque_multiplier_ = 1.0;
  }

  if (current_mission_state_ == MyMissionState::TRAFFIC_STOP) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "미션 수행: [신호등 빨간불 감지 - 정지]");
    target_linear_x_ = 0.0;
    target_angular_z_ = 0.0;
    return;
  }

  switch (current_mission_state_) {
    case MyMissionState::CARGO_WAIT: 
      if (local_is_cargo_stopped_) {
        double stopped_duration = (current_time - local_cargo_stop_start_time_).seconds();
        if (stopped_duration < 20.0) { 
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
            "미션 수행: [보급상자 적재 대기] 제동 중... (%d / 20초)", (int)stopped_duration);
          target_linear_x_ = 0.0;
          target_angular_z_ = 0.0;
        } else {
          RCLCPP_INFO(this->get_logger(), "✨ [보급상자 완료] 20초 경과! 자율주행을 재개합니다.");
          local_is_cargo_stopped_ = false;
          target_linear_x_ = (nav_linear_x_ * 1.5) * local_torque_multiplier_; 
          target_angular_z_ = nav_angular_z_;
        }
      } else {
        local_is_cargo_stopped_ = true;
        local_cargo_stop_start_time_ = current_time;
        target_linear_x_ = 0.0;
        target_angular_z_ = 0.0;
      }
      break;

    case MyMissionState::MARKER_SLOWDOWN: 
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "미션 수행: [비전 마커 인식 - 안정적인 분석을 위한 서행 정밀 주행]");
      target_linear_x_ = (nav_linear_x_ * 0.8) * local_torque_multiplier_; 
      target_angular_z_ = nav_angular_z_;
      break;

    case MyMissionState::TRACK_DRIVING: 
    default:
      if (local_torque_multiplier_ > 1.0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "주행 보정: [험지 탈출 알고리즘 동작 중] 모터 출력 배율: %2.1fx", local_torque_multiplier_);
      }
      target_linear_x_ = nav_linear_x_ * local_torque_multiplier_;
      target_angular_z_ = nav_angular_z_;
      break;
  }

  double left_mps = target_linear_x_ - (target_angular_z_ * track_width_ / 2.0);
  double right_mps = target_linear_x_ + (target_angular_z_ * track_width_ / 2.0);

  double left_rpm = (left_mps / (2.0 * M_PI * sprocket_radius_)) * 60.0 * gear_ratio_;
  double right_rpm = (right_mps / (2.0 * M_PI * sprocket_radius_)) * 60.0 * gear_ratio_;

  int32_t left_cmd = static_cast<int32_t>(left_rpm * rmd_unit_scale_ / 60.0);
  int32_t right_cmd = static_cast<int32_t>(right_rpm * rmd_unit_scale_ / 60.0);

  if (is_autonomous_mode) {
    this->left_speed = left_cmd;
    this->right_speed = right_cmd;
  }
}
