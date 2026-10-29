#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <limits>

class DWA_Navigator {
public:
    DWA_Navigator() : is_navigating_(false), goal_x_(0.0), goal_y_(0.0) {

        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

        goal_sub_  = nh_.subscribe("/move_base_simple/goal", 1,
                                   &DWA_Navigator::goalCallback, this);
        lidar_sub_ = nh_.subscribe("velodyne_points", 1,
                                   &DWA_Navigator::lidarCallback, this);

        // 속도 후보군
        radps_ = {0.0, 0.1, 0.3, 0.5, 0.7, 0.9, -0.1, -0.3, -0.5, -0.7};
        mps_   = {0.3, 0.31, 0.32, 0.33, 0.34, 0.35, 0.36, 0.4, 0.45, 0.5};
    }

    // =========================================================
    // TF에서 현재 위치 갱신
    // =========================================================
    bool updatePoseFromTF() {
        try {
            tf::StampedTransform transform;
            // map → base_link
            tf_listener_.lookupTransform("/map", "/base_link",
                                          ros::Time(0), transform);
            current_x_   = transform.getOrigin().x();
            current_y_   = transform.getOrigin().y();
            current_yaw_ = tf::getYaw(transform.getRotation());
            return true;
        }
        catch (tf::TransformException& ex) {
            // fallback: map → velodyne
            try {
                tf::StampedTransform transform;
                tf_listener_.lookupTransform("/map", "/velodyne",
                                              ros::Time(0), transform);
                current_x_   = transform.getOrigin().x();
                current_y_   = transform.getOrigin().y();
                current_yaw_ = tf::getYaw(transform.getRotation());
                return true;
            }
            catch (...) {
                ROS_WARN("TF 수신 실패 - 위치 갱신 불가");
                return false;
            }
        }
    }

    // =========================================================
    // 목표 좌표 콜백
    // =========================================================
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        goal_x_        = msg->pose.position.x;
        goal_y_        = msg->pose.position.y;
        is_navigating_ = true;
        ROS_INFO("목표 수신: X=%.2f, Y=%.2f", goal_x_, goal_y_);
    }

    // =========================================================
    // 라이다 콜백 — 메인 DWA 루프
    // =========================================================
    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        if (!is_navigating_) return;
        if (!updatePoseFromTF()) return;

        ROS_INFO("현재 위치: X=%.2f, Y=%.2f, Yaw=%.1f°",
                 current_x_, current_y_, current_yaw_ * 180.0 / M_PI);

        double dist = std::hypot(goal_x_ - current_x_, goal_y_ - current_y_);
        if (dist < 0.5) {
            ROS_INFO("목표 지점 도착!");
            stopRobot();
            is_navigating_ = false;
            return;
        }

        // PointCloud2 → PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty()) return;

        // DWA 연산
        Eigen::MatrixXd data_array  = preprocessing(cloud);
        Eigen::MatrixXd local_pos   = makeCombination();
        Eigen::MatrixXd candidates  = createPosCandidates(local_pos);

        int best_v_idx, best_w_idx;
        bool is_too_close;
        evaluateScores(candidates, local_pos, data_array,
                       best_v_idx, best_w_idx, is_too_close);

        if (is_too_close) {
            ROS_WARN("장애물 근접! 정지합니다.");
            stopRobot();
            return;
        }

        geometry_msgs::Twist vel;
        vel.linear.x  = mps_[best_v_idx];
        vel.angular.z = radps_[best_w_idx];
        cmd_pub_.publish(vel);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher  cmd_pub_;
    ros::Subscriber goal_sub_, lidar_sub_;
    tf::TransformListener tf_listener_;

    double current_x_ = 0.0, current_y_ = 0.0, current_yaw_ = 0.0;
    double goal_x_    = 0.0, goal_y_    = 0.0;
    bool   is_navigating_;

    std::vector<double> radps_, mps_;

    // =========================================================
    // 전처리: 로봇 몸통 + 높이 필터
    // =========================================================
    Eigen::MatrixXd preprocessing(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
        std::vector<Eigen::Vector3d> pts;
        for (auto& p : cloud->points) {
            if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z)) continue;
            // 높이 필터
            if (p.z <= -0.55 || p.z >= 0.5) continue;
            // x 필터
            if (p.x <= -0.45) continue;
            // 몸통 제거
            bool x_ok = (p.x > -0.45 || p.x < -0.47);
            bool y_ok = (std::abs(p.y) > 0.17);
            if (!x_ok || !y_ok) continue;
            pts.push_back({p.x, p.y, p.z});
        }
        Eigen::MatrixXd arr(pts.size(), 3);
        for (int i = 0; i < (int)pts.size(); i++)
            arr.row(i) = pts[i];
        return arr;
    }

    // =========================================================
    // 속도 후보 조합 생성 (100 x 2)
    // =========================================================
    Eigen::MatrixXd makeCombination() {
        int n_v = mps_.size();    // 10
        int n_w = radps_.size();  // 10 (0 포함)
        

        Eigen::MatrixXd local_pos(n_v * n_w, 2);
        int row = 0;
        for (int i = 0; i < n_v; i++) {
            double v = mps_[i];
            for (int j = 0; j < n_w; j++) {
                double w = radps_[j];
                double lx, ly;
                if (std::abs(w) < 1e-6) {
                    // 직선 운동
                    lx = v * 0.5;
                    ly = 0.0;
                } else {
                    double r = v / w;
                    double dt = 0.5;
                    lx = r * std::sin(w * dt);
                    ly = r * (1 - std::cos(w * dt));
                }
                local_pos(row, 0) = lx;
                local_pos(row, 1) = ly;
                row++;
            }
        }
        return local_pos;
    }

    // =========================================================
    // 로컬 → 글로벌 좌표 변환
    // =========================================================
    Eigen::MatrixXd createPosCandidates(const Eigen::MatrixXd& local_pos) {
        Eigen::Matrix2d R;
        R << std::cos(current_yaw_), -std::sin(current_yaw_),
             std::sin(current_yaw_),  std::cos(current_yaw_);

        Eigen::MatrixXd global_pos = local_pos * R.transpose();
        global_pos.col(0).array() += current_x_;
        global_pos.col(1).array() += current_y_;
        return global_pos;
    }

    // =========================================================
    // 점수 평가
    // =========================================================
    void evaluateScores(const Eigen::MatrixXd& candidates,
                        const Eigen::MatrixXd& local_pos,
                        const Eigen::MatrixXd& data_array,
                        int& best_v, int& best_w, bool& is_too_close) {
        int N = candidates.rows(); // 100

        Eigen::VectorXd remaining(N), obstacle(N), clearance(N), distance(N);

        // --- remaining scores ---
        for (int i = 0; i < N; i++) {
            double dx   = goal_x_ - candidates(i, 0);
            double dy   = goal_y_ - candidates(i, 1);
            double dist = std::max(std::hypot(dx, dy), 0.5);  // clip
            remaining(i) = 1.0 / dist;
        }
        remaining /= remaining.norm();

        // --- obstacle scores ---
        for (int i = 0; i < N; i++) {
            double min_d = std::numeric_limits<double>::max();
            for (int j = 0; j < (int)data_array.rows(); j++) {
                double dx = data_array(j, 0) - local_pos(i, 0);
                double dy = data_array(j, 1) - local_pos(i, 1);
                min_d = std::min(min_d, std::hypot(dx, dy));
            }
            distance(i) = min_d;
            obstacle(i) = (min_d <= 0.45) ? 0.0 : min_d;
        }
        double obs_norm = obstacle.norm();
        if (obs_norm > 1e-6) obstacle /= obs_norm;

        // --- clearance scores ---
        std::vector<int> clear_idx;
        for (int j = 0; j < (int)data_array.rows(); j++) {
            if (data_array(j, 0) > -0.1 && data_array(j, 0) < 1.0 &&
                std::abs(data_array(j, 1)) > 1.0)
                clear_idx.push_back(j);
        }
        for (int i = 0; i < N; i++) {
            double sum = 0.0;
            for (int j : clear_idx) {
                double dx = data_array(j, 0) - local_pos(i, 0);
                double dy = data_array(j, 1) - local_pos(i, 1);
                sum += std::hypot(dx, dy);
            }
            clearance(i) = clear_idx.empty() ? 0.0 : sum / clear_idx.size();
        }
        double clr_norm = clearance.norm();
        if (clr_norm > 1e-6) clearance /= clr_norm;

        // --- 최종 점수 ---
        Eigen::VectorXd scores = 1.6 * remaining + 1.2 * obstacle + 0.4 * clearance;

        int best_idx;
        scores.maxCoeff(&best_idx);
        best_v      = best_idx / 10;   // mps 인덱스
        best_w      = best_idx % 10;   // radps 인덱스
        is_too_close = (distance(best_idx) <= 0.45);
    }

    void stopRobot() {
        geometry_msgs::Twist vel;
        vel.linear.x = vel.angular.z = 0.0;
        cmd_pub_.publish(vel);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "scout_dwa_navigation");
    DWA_Navigator navigator;
    ros::spin();
    return 0;
}

