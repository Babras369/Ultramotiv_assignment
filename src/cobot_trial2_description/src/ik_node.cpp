#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <algorithm>
#include <chrono>

using std::placeholders::_1;
using namespace std;
using namespace Eigen;

class IKNode : public rclcpp::Node {
public:
    IKNode() : Node("ik_node") {
        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory_controller/joint_trajectory", 10);
        
        // Subscribe to target position topic with explicit QoS settings
        rclcpp::QoS qos_profile(10);
        qos_profile.reliability(rclcpp::ReliabilityPolicy::Reliable);
        qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
        
        target_subscriber_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/target_position", qos_profile, std::bind(&IKNode::target_callback, this, _1));
        
        // Timer for periodic IK solving (optional - can be triggered by target updates)
        timer_ = this->create_wall_timer(5s, std::bind(&IKNode::timer_callback, this));
        
        // Initialize target position
        target_position_ = Vector3d(0.5, 0.0, 0.4); // Default target
        
        RCLCPP_INFO(this->get_logger(), "IK Node initialized successfully!");
        RCLCPP_INFO(this->get_logger(), "Subscribed to: /target_position");
        RCLCPP_INFO(this->get_logger(), "Publishing to: /joint_trajectory_controller/joint_trajectory");
        RCLCPP_INFO(this->get_logger(), "Waiting for target positions...");
        
        // Add a small delay to ensure subscription is fully established
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

private:
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    Vector3d target_position_;
    bool new_target_received_ = false;

    double deg2rad(double deg) {
        return deg * M_PI / 180.0;
    }

    double rad2deg(double rad) {
        return rad * 180.0 / M_PI;
    }

    Matrix4d dh(double a, double alpha, double d, double theta) {
        Matrix4d T;
        T << cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta),
             sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta),
             0,           sin(alpha),            cos(alpha),             d,
             0,           0,                     0,                      1;
        return T;
    }

    Matrix4d forward_kinematics(const vector<double>& joint_angles) {
        // DH Parameters {a, alpha, d, theta}
        vector<vector<double>> dh_params = {
            {0.0,  M_PI/2, 0.3, joint_angles[0]},
            {0.4,  0.0,    0.0, joint_angles[1]},
            {0.3,  0.0,    0.0, joint_angles[2]},
            {0.0,  M_PI/2, 0.2, joint_angles[3]},
            {0.0,  0.0,    0.1, joint_angles[4]}
        };

        Matrix4d T = Matrix4d::Identity();
        for (auto &params : dh_params) {
            Matrix4d Ti = dh(params[0], params[1], params[2], params[3]);
            T = T * Ti;
        }
        return T;
    }

    MatrixXd compute_jacobian(const vector<double>& joint_angles) {
        MatrixXd J(3, 5); // 3D position jacobian for 5 joints
        double h = 1e-6; // Small perturbation for numerical differentiation
        
        Vector3d pos_original = forward_kinematics(joint_angles).block<3,1>(0,3);
        
        for (int i = 0; i < 5; i++) {
            vector<double> perturbed_angles = joint_angles;
            perturbed_angles[i] += h;
            
            Vector3d pos_perturbed = forward_kinematics(perturbed_angles).block<3,1>(0,3);
            Vector3d derivative = (pos_perturbed - pos_original) / h;
            
            J.col(i) = derivative;
        }
        
        return J;
    }

    vector<double> solve_ik_newton_raphson(const Vector3d& target, const vector<double>& initial_guess, 
                                          int max_iterations = 100, double tolerance = 1e-3) {
        vector<double> joint_angles = initial_guess;
        
        for (int iter = 0; iter < max_iterations; iter++) {
            // Compute current end-effector position
            Vector3d current_pos = forward_kinematics(joint_angles).block<3,1>(0,3);
            
            // Compute error
            Vector3d error = target - current_pos;
            double error_norm = error.norm();
            
            RCLCPP_DEBUG(this->get_logger(), "Iteration %d: Error norm = %.6f", iter, error_norm);
            
            if (error_norm < tolerance) {
                RCLCPP_INFO(this->get_logger(), "IK converged in %d iterations with error %.6f", 
                           iter, error_norm);
                return joint_angles;
            }
            
            // Compute Jacobian
            MatrixXd J = compute_jacobian(joint_angles);
            
            // Compute pseudo-inverse using SVD (more robust than direct inverse)
            JacobiSVD<MatrixXd> svd(J, ComputeThinU | ComputeThinV);
            MatrixXd J_pinv = svd.matrixV() * svd.singularValues().cwiseInverse().asDiagonal() * svd.matrixU().transpose();
            
            // Check for singularity
            if (svd.singularValues().minCoeff() < 1e-6) {
                RCLCPP_WARN(this->get_logger(), "Near singular configuration detected at iteration %d", iter);
            }
            
            // Update joint angles using damped least squares for stability
            double damping = 0.01;
            MatrixXd J_damped = J.transpose() * (J * J.transpose() + damping * MatrixXd::Identity(3, 3)).inverse();
            VectorXd delta_q = J_damped * error;
            
            // Apply joint limits and update
            for (int i = 0; i < 5; i++) {
                joint_angles[i] += delta_q(i);
                // Apply joint limits (adjust these based on your robot's specifications)
                joint_angles[i] = max(-M_PI, min(M_PI, joint_angles[i]));
            }
        }
        
        RCLCPP_WARN(this->get_logger(), "IK did not converge within %d iterations", max_iterations);
        return joint_angles;
    }

    void target_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
        target_position_ = Vector3d(msg->x, msg->y, msg->z);
        new_target_received_ = true;
        
        RCLCPP_INFO(this->get_logger(), "✓ Target position received: [%.3f, %.3f, %.3f]", 
                    target_position_(0), target_position_(1), target_position_(2));
        
        // Solve IK immediately when new target is received
        solve_and_publish_ik();
    }

    void timer_callback() {
        // Check subscription status
        size_t subscriber_count = this->count_publishers("/target_position");
        size_t subscription_count = target_subscriber_->get_publisher_count();
        
        RCLCPP_DEBUG(this->get_logger(), "Publishers on /target_position: %zu", subscriber_count);
        RCLCPP_DEBUG(this->get_logger(), "Connected publishers: %zu", subscription_count);
        
        // Periodic IK solving with default target or last received target
        if (!new_target_received_) {
            RCLCPP_INFO(this->get_logger(), "Using default target position: [%.3f, %.3f, %.3f]", 
                        target_position_(0), target_position_(1), target_position_(2));
            RCLCPP_INFO(this->get_logger(), "To send a new target: ros2 topic pub /target_position geometry_msgs/msg/Point \"{x: 0.5, y: 0.2, z: 0.4}\"");
        }
        solve_and_publish_ik();
        new_target_received_ = false;
    }

    void solve_and_publish_ik() {
        // Initial guess for joint angles (you can use current robot state if available)
        vector<double> initial_guess = {0.0, 0.0, 0.0, 0.0, 0.0};
        
        // Solve IK
        vector<double> solution = solve_ik_newton_raphson(target_position_, initial_guess);
        
        // Verify solution by computing forward kinematics
        Vector3d achieved_pos = forward_kinematics(solution).block<3,1>(0,3);
        Vector3d final_error = target_position_ - achieved_pos;
        double final_error_norm = final_error.norm();
        
        RCLCPP_INFO(this->get_logger(), "Target: [%.3f, %.3f, %.3f]", 
                    target_position_(0), target_position_(1), target_position_(2));
        RCLCPP_INFO(this->get_logger(), "Achieved: [%.3f, %.3f, %.3f]", 
                    achieved_pos(0), achieved_pos(1), achieved_pos(2));
        RCLCPP_INFO(this->get_logger(), "Final error: %.6f", final_error_norm);
        
        // Convert to degrees for logging
        RCLCPP_INFO(this->get_logger(), "Joint angles (deg): [%.2f, %.2f, %.2f, %.2f, %.2f]",
                    rad2deg(solution[0]), rad2deg(solution[1]), rad2deg(solution[2]), 
                    rad2deg(solution[3]), rad2deg(solution[4]));

        // Prepare and publish JointTrajectory message
        auto msg = trajectory_msgs::msg::JointTrajectory();
        msg.joint_names = {"link1_revolve", "link2_revolve", "base_revolve", "link3_revolve", "link4_revolve"};

        auto point = trajectory_msgs::msg::JointTrajectoryPoint();
        point.positions = solution;
        point.time_from_start = rclcpp::Duration::from_seconds(3.0);

        msg.points.push_back(point);
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "IK solution sent to joint trajectory controller.");
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IKNode>());
    rclcpp::shutdown();
    return 0;
}