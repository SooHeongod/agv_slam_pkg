#include <ros/ros.h>
#include <nav_msgs/Odometry.h> // Odometry 메시지 타입
#include <geometry_msgs/Twist.h> // Twist 메시지 타입 (여기서는 사용하지 않지만, 모터 제어 노드에서 활용)
#include <tf/transform_broadcaster.h> // TF 브로드캐스터
#include <tf/tf.h> // 쿼터니언 변환을 위한 TF 유틸리티

#include <math.h> // 수학 함수 (sin, cos, M_PI 등)
#include <string> // 문자열 처리

// 로봇 및 엔코더 파라미터 정의
// ROS 파라미터 서버에서 로드하는 것이 일반적이지만, 예시를 위해 직접 정의
const double WHEEL_RADIUS = 0.033;       // 바퀴 반지름 (미터) - 터틀봇3 기준
const double WHEEL_SEPARATION = 0.287;   // 바퀴 간의 거리 (트랙 폭, 미터) - 터틀봇3 기준
const int ENCODER_PPR = 4096;            // 엔코더 펄스 수 (Pulses Per Revolution) - MD-200T 사양 확인 필요

// 오도메트리 상태 변수 (전역 변수 또는 클래스 멤버 변수로 관리)
double current_x = 0.0;        // 로봇의 현재 x 좌표 (미터)
double current_y = 0.0;        // 로봇의 현재 y 좌표 (미터)
double current_theta = 0.0;    // 로봇의 현재 yaw 각도 (라디안)

// 이전 엔코더 틱 값
long prev_left_ticks = 0;
long prev_right_ticks = 0;

// 이전 시간
ros::Time prev_time;

// ROS Publisher 및 TF Broadcaster
ros::Publisher odom_pub;
tf::TransformBroadcaster* odom_broadcaster; // 포인터로 선언하여 main 함수에서 초기화

// 가상의 엔코더 데이터 메시지 (실제 MD100 통신 시에는 해당 드라이버의 메시지 타입을 사용)
// 편의를 위해 간단한 구조체 정의
struct EncoderData {
    long left_encoder_value;
    long right_encoder_value;
};

// 가상의 엔코더 데이터 콜백 함수 (실제 MD100 드라이버 노드에서 이 데이터를 발행한다고 가정)
// 실제 구현에서는 MD100 드라이버와 시리얼 통신하여 엔코더 값을 읽어오는 로직이 필요합니다.
void encoderDataCallback(const EncoderData& msg) {
    // ROS_INFO("Received Encoder Data: L=%ld, R=%ld", msg.left_encoder_value, msg.right_encoder_value);

    // 현재 엔코더 틱 값
    long current_left_ticks = msg.left_encoder_value;
    long current_right_ticks = msg.right_encoder_value;

    // 현재 시간
    ros::Time current_time = ros::Time::now();
    double dt = (current_time - prev_time).toSec();

    // 시간 간격이 너무 작으면 계산 생략 (오류 방지)
    if (dt < 0.0001) {
        return;
    }

    // 1. 엔코더 틱 변화량 계산
    long delta_left_ticks = current_left_ticks - prev_left_ticks;
    long delta_right_ticks = current_right_ticks - prev_right_ticks;

    // 2. 각 바퀴의 이동 거리 계산
    double delta_d_left = (static_cast<double>(delta_left_ticks) / ENCODER_PPR) * (2 * M_PI * WHEEL_RADIUS);
    double delta_d_right = (static_cast<double>(delta_right_ticks) / ENCODER_PPR) * (2 * M_PI * WHEEL_RADIUS);

    // 3. 로봇의 선형 이동 거리 (delta_d) 및 회전 각도 (delta_theta) 계산
    double delta_d = (delta_d_left + delta_d_right) / 2.0;       // 로봇 중심의 이동 거리
    double delta_theta = (delta_d_right - delta_d_left) / WHEEL_SEPARATION; // 로봇의 회전 각도

    // 4. 로봇의 선속도 및 각속도 계산
    double linear_velocity = delta_d / dt;
    double angular_velocity = delta_theta / dt;

    // 5. 로봇의 자세(Pose) 업데이트
    // 곡선 운동 공식 적용 (delta_theta가 0에 가까울 때는 직선 운동 공식과 유사)
    // 0으로 나누는 것을 방지하기 위한 작은 임계값 설정
    if (fabs(delta_theta) < 1e-6) { // delta_theta가 거의 0이면 직선 운동으로 간주
        current_x += delta_d * cos(current_theta);
        current_y += delta_d * sin(current_theta);
        current_theta += delta_theta; // 0에 가까운 값
    } else {
        // 곡선 운동 공식
        double R_icc = delta_d / delta_theta; // 회전 중심까지의 거리 (R_icc)

        // 회전 중심 (ICC) 계산
        double icc_x = current_x - R_icc * sin(current_theta);
        double icc_y = current_y + R_icc * cos(current_theta);

        // 새로운 자세 계산 (회전 행렬 적용)
        current_x = icc_x + R_icc * sin(current_theta + delta_theta);
        current_y = icc_y - R_icc * cos(current_theta + delta_theta);
        current_theta = current_theta + delta_theta;
    }

    // 각도를 -pi ~ pi 범위로 정규화
    current_theta = atan2(sin(current_theta), cos(current_theta));

    // Odometry 메시지 생성 및 발행
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom"; // 오도메트리 프레임 ID
    odom.child_frame_id = "base_link"; // 로봇의 기본 프레임 ID

    // 위치 정보 설정
    odom.pose.pose.position.x = current_x;
    odom.pose.pose.position.y = current_y;
    odom.pose.pose.position.z = 0.0;
    // Yaw 각도를 쿼터니언으로 변환
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(current_theta);

    // 속도 정보 설정
    odom.twist.twist.linear.x = linear_velocity;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = angular_velocity;

    // 오도메트리 메시지 발행
    odom_pub.publish(odom);

    // TF 브로드캐스팅 (odom -> base_link)
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = current_x;
    odom_trans.transform.translation.y = current_y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(current_theta);

    // TF 브로드캐스팅
    odom_broadcaster->sendTransform(odom_trans);

    // 다음 계산을 위해 이전 값 업데이트
    prev_left_ticks = current_left_ticks;
    prev_right_ticks = current_right_ticks;
    prev_time = current_time;
}

int main(int argc, char** argv) {
    // ROS 노드 초기화
    ros::init(argc, argv, "md_robot_odometry_node");
    ros::NodeHandle nh;

    // ROS Publisher 초기화
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);

    // TF Broadcaster 초기화
    odom_broadcaster = new tf::TransformBroadcaster();

    // 이전 시간 초기화
    prev_time = ros::Time::now();

    // 가상의 엔코더 데이터 구독 (실제 MD100 드라이버 노드에서 발행하는 토픽을 구독)
    // 여기서는 'encoder_data'라는 가상의 토픽을 구독한다고 가정합니다
    // 이 예시에서는 EncoderData 구조체를 메시지 타입으로 사용했지만, 실제로는 std_msgs/Int64MultiArray 등을 사용할 수 있습니다.
    // ros::Subscriber encoder_sub = nh.subscribe("encoder_data", 10, encoderDataCallback);
    

    ros::Rate loop_rate(100); // 100Hz로 오도메트리 계산 및 발행 (MD100 데이터 수신 주기에 따라 조절)
    long sim_left_ticks = 0;
    long sim_right_ticks = 0;
    int counter = 0;

    while (ros::ok()) {
        // --- 가상의 엔코더 데이터 생성 (실제 MD100 드라이버에서 읽어온다고 가정) ---
        // 이 부분은 실제 MD100과의 시리얼 통신을 통해 엔코더 값을 읽어오는 코드로 대체되어야 합니다.
        // 예시를 위해 간단한 움직임을 시뮬레이션합니다.
        if (counter < 100) { // 직진
            sim_left_ticks += 10;
            sim_right_ticks += 10;
        } else if (counter < 200) { // 오른쪽 회전
            sim_left_ticks += 10;
            sim_right_ticks += 5;
        } else if (counter < 300) { // 왼쪽 회전
            sim_left_ticks += 5;
            sim_right_ticks += 10;
        } else { // 정지
            // sim_left_ticks = sim_left_ticks;
            // sim_right_ticks = sim_right_ticks;
        }
        counter++;
        if (counter > 400) counter = 0; // 반복 시뮬레이션

        EncoderData current_encoder_data;
        current_encoder_data.left_encoder_value = sim_left_ticks;
        current_encoder_data.right_encoder_value = sim_right_ticks;

        // 엔코더 데이터를 콜백 함수로 전달하여 오도메트리 계산 및 발행
        encoderDataCallback(current_encoder_data);
        // ----------------------------------------------------------------------

        ros::spinOnce(); // 콜백 함수 처리
        loop_rate.sleep(); // 주기 유지
    }

    // 메모리 해제
    delete odom_broadcaster;

    return 0;
}
