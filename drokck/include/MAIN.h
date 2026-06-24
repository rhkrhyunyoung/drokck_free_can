#ifndef MAIN_ACTIVE_NODE_HPP
#define MAIN_ACTIVE_NODE_HPP

// CAN & Network Library
#include <errno.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

// ---signal & Flie Library--------
#include <signal.h>
#include <fstream>

// Thread & Time Library
#include <pthread.h>
#include <sched.h>
#include <chrono>
#include <time.h>
#include <mutex>

// C++ Basic Library
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <iostream>
#include <cmath>
#include <functional>
#include <memory>
#include <climits>
#include <cstring>
#include <string>
#include <array>

// ETC
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <fstream>

// ROS2 Libraries
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"              
#include "geometry_msgs/msg/vector3_stamped.hpp" 
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

// User Headers
#include "RMD_COMMAND.h"

#include <yaml-cpp/yaml.h>

#define PI 3.14159265359
#define PI2 6.28318530718
#define D2R PI / 180.

// Constants
#define TauMap 62.5
#define TauReadMap 62.061
#define ns_OneMilSec 1000000
#define ns_TwoMilSec 2000000
#define ns_FiveMilSec 5000000
#define ns_SixMilSec 6000000
#define ns_TenMilSec 10000000
#define ns_OneSec 1000000000
#define ns_TwoSec 2000000000
#define NUM_OF_CANABLE 4 
#define NUM_OF_MOTOR 2   
#define JOINT_NUM 4 

using namespace std::chrono_literals;
using std::placeholders::_1;

constexpr int NUM_MOTORS = 8; 
constexpr int QUEUE_SIZE = 20;

volatile bool main_thread_exit = false;
volatile bool thread_flag = true;
volatile bool time_check = false; 

int32_t left_rpm_command_;
int32_t right_rpm_command_;
float wheel_base = 0.35;  
float wheel_radius = 0.1; 
float linear_velocity;
float angular_velocity;
struct can_frame frame;
int cansock[NUM_OF_CANABLE];

class MainActiveNode : public rclcpp::Node
{
public:
    MainActiveNode();
    ~MainActiveNode();

    // 자율주행 미션용 상태 머신 구조
    enum class MissionState {
        EMERGENCY_STOP,          
        SUMMER_CARGO,            
        AUTUMN_MARKER,           
        WINTER_ICE,              
        ROBOT_DOG_BLIND,         
        ROBOT_DOG_FOLLOW,        
        DEFAULT_TRACK_DRIVING    
    };

    // ✨ main.cpp에서 호출하는 멀티스레드 래퍼 및 스레드 함수 추가
    static void *CAN_Thread_wrapper(void *arg);
    void CAN_Thread();

    // ✨ main.cpp에서 요구하는 모터 지령 속도 제어 변수 추가
    int32_t left_speed = 0;
    int32_t right_speed = 0;

private:
    struct Motor_state_t
    {
        float theta_org[2];
        float theta[2];
        float speed[2];
        float torque[2];
        float temp[2];
        int data_count[4] = {0, 0, 0, 0};
    };

    struct period_info
    {
        struct timespec next_period;
        struct timespec current_time_1;
        struct timespec current_time_2;
        long period_ns;
    };

    typedef struct Joint
    {
        float InitPos;
        float InitVel;
        float InitAcc;
        float RefPos;
        float RefVel;
        float RefAcc;
        float RefPrePos;
        float RefPreVel;
        float RefPreAcc;
        float RefTorque;
        float RefTargetPos;
        float RefTargetVel;
        float RefTargetAcc;
        float CurrentPos;
        float CurrentVel;
        float CurrentAcc;
        float CurrentTorque;
        bool IsMoving; 
        float runtime; 
    } JOINT;

    typedef enum
    {
        POSITION_COMMAND = 0,
        TORQUE_COMMAND,
        TEST,
        RPM_COMMAND
    } COMMAND_MODE;

    // Variables
    rclcpp::TimerBase::SharedPtr Publish_timer;
    
    // ✨ main.cpp 스레드 바인딩 에러 해결용 변수들 추가
    pthread_t main_thread;
    pthread_t can_thread; 
    int can_socks[NUM_OF_CANABLE]; 

    std::mutex _mtx;
    RMD_COMMAND Rmd;

    std::array<rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr, 4> flag_publishers;
    std::array<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr, 4> time_publishers;
    std::array<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr, JOINT_NUM> degree_publishers;
    
    // ✨ main.cpp에서 쓰고 있는 각도 퍼블리셔(theta_publishers) 매핑 추가
    std::array<std::array<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr, NUM_OF_MOTOR>, NUM_OF_CANABLE> theta_publishers;
    
    std::array<std::array<rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr, NUM_OF_MOTOR>, NUM_OF_CANABLE> tau_publishers;
    std::array<std::array<rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr, NUM_OF_MOTOR>, NUM_OF_CANABLE> torque_publishers;
    std::array<std::array<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr, NUM_OF_MOTOR>, NUM_OF_CANABLE> motor_degree_publishers;
    std::array<std::array<rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr, NUM_OF_MOTOR>, NUM_OF_CANABLE> temp_publishers;
    std::array<std::array<rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr, NUM_OF_MOTOR>, NUM_OF_CANABLE> speed_publishers;
    std::array<rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr, 12> refvel_publishers;
    std::array<rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr, 12> curvel_publishers;
    std::array<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr, 12> traj_publishers;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Joy>> joy_sub;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_subscriber_;

    int16_t torque_command_[4] = {0, 0, 0, 0};
    float torque_command_tmp[4] = {0, 0, 0, 0};
    int8_t temp = 0; 
    int16_t torque = 0;
    int16_t speed = 0;
    int16_t pulse = 0;
    float pulse_to_degree = 0.00001525878 * 10.;
    float pulse_to_degree_x4 = 0.00006103515 * 60.;
    float degree = 0.;
    float previous_degree[NUM_OF_CANABLE][NUM_OF_MOTOR];
    float output_degree = 0.;
    float Kp_FR, Kp_FL, Kp_BR, Kp_BL;
    float Kd_FR, Kd_FL, Kd_BR, Kd_BL;
    int angle_switch[NUM_OF_CANABLE][NUM_OF_MOTOR];
    int flag[NUM_OF_CANABLE][NUM_OF_MOTOR];
    float tau[NUM_OF_CANABLE][NUM_OF_MOTOR];
    int command;
    int err_count = 0;

    FILE *drokX_data = NULL;
    int PrintDataIndex = 0;
    float PrintDataTime = 0.0;

    float init_theta[NUM_OF_CANABLE][NUM_OF_MOTOR];
    float desired_theta[NUM_OF_CANABLE][NUM_OF_MOTOR];
    float traj_theta[NUM_OF_CANABLE][NUM_OF_MOTOR];
    float calc_Vel[NUM_OF_CANABLE][NUM_OF_MOTOR];

    float runtime = 0.;
    float thread_timer = 0.;
    float Timedata[4] = {0, 0, 0, 0};

    struct can_frame frame[4];
    Motor_state_t motor_state[4];
    Motor_state_t motor_states[4];
    COMMAND_MODE command_mode;
    JOINT *joint;
    sensor_msgs::msg::Joy::SharedPtr joystick_data_;

    std::chrono::time_point<std::chrono::steady_clock> last_joystick_time_;
    std::chrono::time_point<std::chrono::high_resolution_clock> before_thread_time; 

    struct ThreadConfig {
        int core;
        int policy;
        int priority;
    };

    struct CANConfig {
        struct Interface {
            std::string name;
            int id;
        };
        std::vector<Interface> interfaces;
    };

    ThreadConfig loadThreadConfig();
    CANConfig loadCANConfig();

    void publishMessage();
    int CAN_INIT(const std::string &can_interface_name);
    void READ_BUFFER();
    void READ_TORQUE_SPEED_ANGLE(int s, int canable, struct can_frame frame);
    void READ_TORQUE_SPEED_ANGLE_X4(int s, int canable, struct can_frame frame);

    void EMERGENCY_STOP();
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void Check_State();
    void ComputeTorque();

    float func_1_cos(float t, float init, float final, float T);
    float dt_func_1_cos(float t, float init, float final, float T);

    Motor_state_t Get_motor_state(int i) 
    {
        return motor_state[i]; 
    }

    void GetState();
    pthread_t Thread_init(void *(*thread_func)(void *), void *arg, int core, int policy, int priority);
    void *pthread_main();

    // 자율주행 제어 토픽 및 변수들
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr yolo_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr yolo_xyz_sub_;

    std::string detected_class_;
    rclcpp::Time last_vision_time_;
    
    double nav_linear_x_ = 0.0;
    double nav_angular_z_ = 0.0;
    double target_linear_x_ = 0.0;
    double target_angular_z_ = 0.0;
    double obj_x_ = 0.0;
    double obj_z_ = 0.0;

    double track_width_;
    double sprocket_radius_;
    double gear_ratio_;
    double rmd_unit_scale_;
    bool is_autonomous_mode;

    void navCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void yoloCallback(const std_msgs::msg::String::SharedPtr msg);
    void yoloXyzCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
    void decisionMakingTree();
    void executeStateBehavior();
};

#endif // MAIN_ACTIVE_NODE_HPP
