#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;

class CmdVelSinePublisher : public rclcpp::Node
{
public:
    CmdVelSinePublisher()
        : Node("cmdvel_sine_publisher"),
          t_(0.0)
    {
        // Declare parameters
        this->declare_parameter("amplitude_linear", 0.5);
        this->declare_parameter("amplitude_angular", 0.5);
        this->declare_parameter("frequency", 0.2); // Hz

        // Get parameters
        amplitude_linear_  = this->get_parameter("amplitude_linear").as_double();
        amplitude_angular_ = this->get_parameter("amplitude_angular").as_double();
        frequency_         = this->get_parameter("frequency").as_double();

        // Publisher
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Timer @ 50 Hz
        timer_ = this->create_wall_timer(20ms, std::bind(&CmdVelSinePublisher::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), 
            "Sine cmd_vel publisher started: A_lin=%.2f, A_ang=%.2f, freq=%.2f Hz",
            amplitude_linear_, amplitude_angular_, frequency_);
    }

private:
    void timerCallback()
    {
        geometry_msgs::msg::Twist msg;

        // ω = 2πf
        double omega = 2.0 * M_PI * frequency_;

        msg.linear.x  = amplitude_linear_  * std::sin(omega * t_);
        msg.angular.z = amplitude_angular_ * std::sin(omega * t_);

        pub_->publish(msg);

        t_ += 0.02; // 时间步长（50Hz）

        // Debug print
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(),
            1000, // 每 1s 打印一次
            "cmd_vel sin: lx=%.3f, wz=%.3f", msg.linear.x, msg.angular.z);
    }

    // ROS2 元件
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 状态变量
    double t_;
    double amplitude_linear_;
    double amplitude_angular_;
    double frequency_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelSinePublisher>());
    rclcpp::shutdown();
    return 0;
}
