#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Pose2D.h"
#include "sensor_msgs/NavSatFix.h"
#include <vector>
#include <cmath>
#include <fstream>
#include <iostream>

// 매크로 정의
#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)/180.*M_PI)

// 차량 제어 상수
#define MAX_L_STEER -30 // 최대 좌측 조향각 (도)
#define MAX_R_STEER 30  // 최대 우측 조향각 (도)

// Waypoint 구조체
struct Waypoint {
    double x; // UTM Easting
    double y; // UTM Northing
};

class PurePursuitNode {
public:
    PurePursuitNode() {
        // ROS 파라미터 로드
        nh_.param<double>("wheelbase", wheelbase_, 1.0); // 차량의 축간거리 (m)
        nh_.param<double>("look_ahead_distance", look_ahead_distance_, 4.0); // 전방 주시 거리 (m)
        nh_.param<double>("target_velocity", target_velocity_, 30.0); // 목표 속도 (km/h 등, 단위 통일 필요)
        nh_.param<std::string>("waypoint_path", waypoint_path_, ""); // 웨이포인트 파일 경로

        // Subscriber & Publisher 설정
        front_gps_sub_ = nh_.subscribe("/gps/front/fix", 10, &PurePursuitNode::frontGpsCallback, this);
        rear_gps_sub_ = nh_.subscribe("/gps/rear/fix", 10, &PurePursuitNode::rearGpsCallback, this);

        steer_pub_ = nh_.advertise<std_msgs::Int16>("/car_control/steering_angle", 10);
        speed_pub_ = nh_.advertise<std_msgs::Float32>("/car_control/speed", 10);

        // Waypoint 로드
        loadWaypoints();
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber front_gps_sub_, rear_gps_sub_;
    ros::Publisher steer_pub_, speed_pub_;

    // GPS 데이터 수신 변수
    sensor_msgs::NavSatFix front_gps_fix_;
    sensor_msgs::NavSatFix rear_gps_fix_;
    bool front_gps_ok_ = false;
    bool rear_gps_ok_ = false;

    // 차량 상태 및 파라미터
    geometry_msgs::Pose2D current_pose_; // 현재 차량의 UTM x, y, heading(radian)
    std::vector<Waypoint> waypoints_;
    int current_waypoint_idx_ = 0;
    
    // Pure Pursuit 파라미터
    double wheelbase_;
    double look_ahead_distance_;
    double target_velocity_;
    std::string waypoint_path_;

    // WGS84 to UTM 변환 함수 (기존 코드에서 가져옴)
    void wgs2utm(double lat, double lon, int zone, double& east, double& north) {
        double lat_rad = lat * M_PI/180;
        double lon_rad = lon * M_PI/180;

        double phi = lat_rad;
        double lambda = lon_rad;
        double lambda0 = (zone * 6 - 183) * M_PI/180;
        double sm_a = 6378137;
        double sm_b = 6356752.31;

        double ep2 = (pow(sm_a, 2.0) - pow(sm_b, 2.0)) / pow(sm_b, 2.0);
        double nu2 = ep2*pow(cos(phi), 2.0);
        double N = pow(sm_a, 2.0) / (sm_b * sqrt(1 + nu2));
        double l = lambda - lambda0;
        double t = tan(phi);
        double t2 = t * t;

        double l3coef = 1.0 - t2 + nu2;
        double l5coef = 5.0 - 18.0 * t2 + (t2 * t2) + 14.0 * nu2 - 58.0 * t2 * nu2;
        double l7coef = 61.0 - 479.0 * t2 + 179.0 * (t2 * t2) - (t2 * t2 * t2);

        east = N * cos(phi) * l +
            (N / 6.0 * pow(cos(phi), 3.0) * l3coef * pow(l, 3.0)) +
            (N / 120.0 * pow(cos(phi), 5.0) * l5coef * pow(l, 5.0)) +
            (N / 5040.0 * pow(cos(phi), 7.0) * l7coef * pow(l, 7.0));
        
        east = east * 0.9996 + 500000;

        double n = (sm_a - sm_b) / (sm_a + sm_b);
        double alpha = ((sm_a + sm_b) / 2.0) * (1.0 + (pow(n, 2.0) / 4.0) + (pow(n,4.0) / 64.0));
        double beta = (-3.0 * n / 2.0) + (9.0 * pow(n, 3.0) / 16.0) + (-3.0 * pow(n, 5.0) / 32.0);
        double gamma = (15.0 * pow(n, 2.0) / 16.0) + (-15.0 * pow(n,4.0) / 32.0);
        double delta = (-35.0 * pow(n,3.0) / 48.0) + (105.0 * pow(n, 5.0) / 256.0);
        double epsilon = (315.0 * pow(n, 4.0) / 512.0);

        double ArcLengthMeridian = alpha * (phi + (beta * sin(2.0 * phi)) + (gamma * sin(4.0 * phi)) + (delta * sin(6.0  * phi)) + (epsilon * sin(8.0 * phi)));
        
        north = ArcLengthMeridian +
                (t / 2.0 * N * pow(cos(phi), 2.0) * pow(l, 2.0)) +
                (t / 24.0 * N * pow(cos(phi), 4.0) * (5.0 - t2 + 9 * nu2 + 4.0 * (nu2 * nu2)) * pow(l, 4.0)) +
                (t / 720.0 * N * pow(cos(phi), 6.0) * (61.0 - 58.0 * t2 + (t2 * t2) + 270.0 * nu2 - 330.0 * t2 * nu2) * pow(l, 6.0));
        
        north = north * 0.9996;
    }

    void loadWaypoints() {
        std::ifstream file(waypoint_path_);
        if (!file.is_open()) {
            ROS_ERROR("Cannot open waypoint file: %s", waypoint_path_.c_str());
            return;
        }

        std::string line;
        // 헤더 라인 스킵
        std::getline(file, line);

        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string value;
            Waypoint wp;
            
            std::getline(ss, value, ',');
            wp.x = std::stod(value);
            
            std::getline(ss, value, ',');
            wp.y = std::stod(value);

            // 속도 값은 현재 코드에서는 사용하지 않지만, 파싱은 가능
            // std::getline(ss, value, ','); 
            
            waypoints_.push_back(wp);
        }
        ROS_INFO("%ld waypoints loaded.", waypoints_.size());
        file.close();
    }

    void frontGpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        front_gps_fix_ = *msg;
        front_gps_ok_ = (msg->status.status >= sensor_msgs::NavSatStatus::STATUS_FIX);
        if(front_gps_ok_) processGpsData();
    }

    void rearGpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        rear_gps_fix_ = *msg;
        rear_gps_ok_ = (msg->status.status >= sensor_msgs::NavSatStatus::STATUS_FIX);
    }
    
    void processGpsData() {
        if (!front_gps_ok_ || !rear_gps_ok_ || waypoints_.empty()) {
            ROS_WARN_THROTTLE(1.0, "Waiting for valid GPS data or waypoints...");
            return;
        }

        // 1. 듀얼 GPS로 현재 위치 및 방위각 계산
        double front_utm_x, front_utm_y, rear_utm_x, rear_utm_y;
        int utm_zone = 52; // 대한민국 기준

        wgs2utm(front_gps_fix_.latitude, front_gps_fix_.longitude, utm_zone, front_utm_x, front_utm_y);
        wgs2utm(rear_gps_fix_.latitude, rear_gps_fix_.longitude, utm_zone, rear_utm_x, rear_utm_y);

        // 후방 GPS를 차량의 현재 위치로 설정
        current_pose_.x = rear_utm_x;
        current_pose_.y = rear_utm_y;

        // 방위각 계산
        current_pose_.theta = atan2(front_utm_y - rear_utm_y, front_utm_x - rear_utm_x);

        // 2. Pure Pursuit 알고리즘
        // 목표 지점(Goal Point) 탐색
        int goal_idx = findGoalPointIndex();
        if(goal_idx < 0) {
            ROS_INFO("Reached the final waypoint!");
            publishControl(0, 0); // 미션 완료 시 정지
            return;
        }
        Waypoint goal_point = waypoints_[goal_idx];

        // 목표 지점을 차량의 로컬 좌표계로 변환
        double dx = goal_point.x - current_pose_.x;
        double dy = goal_point.y - current_pose_.y;
        double local_x = dx * cos(current_pose_.theta) + dy * sin(current_pose_.theta);
        double local_y = -dx * sin(current_pose_.theta) + dy * cos(current_pose_.theta);

        // 조향각 계산
        double alpha = atan2(local_y, local_x);
        double steering_angle_rad = atan2(2.0 * wheelbase_ * sin(alpha), look_ahead_distance_);
        
        int steering_angle_deg = static_cast<int>(RAD2DEG(steering_angle_rad));

        // 조향각 제한
        if (steering_angle_deg > MAX_R_STEER) steering_angle_deg = MAX_R_STEER;
        if (steering_angle_deg < MAX_L_STEER) steering_angle_deg = MAX_L_STEER;

        // 3. 제어 명령 발행
        publishControl(target_velocity_, steering_angle_deg);
    }

    int findGoalPointIndex() {
        // 현재 위치에서 가장 가까운 웨이포인트를 찾음
        double min_dist = 1e9;
        for (int i = current_waypoint_idx_; i < waypoints_.size(); ++i) {
            double dx = waypoints_[i].x - current_pose_.x;
            double dy = waypoints_[i].y - current_pose_.y;
            double dist = sqrt(dx*dx + dy*dy);
            if (dist < min_dist) {
                min_dist = dist;
                current_waypoint_idx_ = i;
            }
        }
        
        // 가장 가까운 웨이포인트부터 전방 주시 거리만큼 떨어진 포인트를 탐색
        for (int i = current_waypoint_idx_; i < waypoints_.size(); ++i) {
            double dx = waypoints_[i].x - current_pose_.x;
            double dy = waypoints_[i].y - current_pose_.y;
            double dist = sqrt(dx*dx + dy*dy);

            if (dist >= look_ahead_distance_) {
                return i;
            }
        }

        return -1; // 경로 끝에 도달
    }

    void publishControl(float speed, int steer) {
        std_msgs::Float32 speed_msg;
        std_msgs::Int16 steer_msg;

        speed_msg.data = speed;
        steer_msg.data = steer;

        speed_pub_.publish(speed_msg);
        steer_pub_.publish(steer_msg);
        
        ROS_INFO("Current Pose: [x: %.2f, y: %.2f, th: %.1f deg] | Target Vel: %.1f, Steer: %d deg", 
                 current_pose_.x, current_pose_.y, RAD2DEG(current_pose_.theta), speed, steer);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "dual_gps_pure_pursuit_node");
    PurePursuitNode pp_node;
    ros::spin();
    return 0;
}