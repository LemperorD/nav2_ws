#include <chrono>
#include <cmath>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;

class CmdVelPublisher : public rclcpp::Node
{
public:
    CmdVelPublisher()
        : Node("cmdvel_publisher"), t_(0.0)
    {
        /* ---------------- 参数声明 ---------------- */
        this->declare_parameter<std::string>("mode", "sine");

        // constant 模式参数
        this->declare_parameter("constant_linear", 0.3);
        this->declare_parameter("constant_angular", 0.0);

        // sine 模式参数
        this->declare_parameter("amplitude_linear", 0.5);
        this->declare_parameter("amplitude_angular", 0.5);
        this->declare_parameter("frequency", 0.2);  // Hz

        /* ---------------- 参数读取 ---------------- */
        mode_ = this->get_parameter("mode").as_string();

        constant_linear_  = this->get_parameter("constant_linear").as_double();
        constant_angular_ = this->get_parameter("constant_angular").as_double();

        amplitude_linear_  = this->get_parameter("amplitude_linear").as_double();
        amplitude_angular_ = this->get_parameter("amplitude_angular").as_double();
        frequency_         = this->get_parameter("frequency").as_double();

        /* ---------------- Publisher ---------------- */
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        /* ---------------- Timer ---------------- */
        timer_ = this->create_wall_timer(
            20ms, std::bind(&CmdVelPublisher::timerCallback, this));

        RCLCPP_INFO(this->get_logger(),
            "cmd_vel publisher started. mode=%s", mode_.c_str());
    }

private:
    void timerCallback()
    {
        geometry_msgs::msg::Twist msg;

        if (mode_ == "constant")
        {
            msg.linear.x  = constant_linear_;
            msg.angular.z = constant_angular_;
        }
        else if (mode_ == "sine")
        {
            double omega = 2.0 * M_PI * frequency_;
            msg.linear.x  = amplitude_linear_  * std::sin(omega * t_);
            msg.angular.z = amplitude_angular_ * std::sin(omega * t_);
            t_ += 0.02;  // 50 Hz
        }
        else if (mode_ == "stop")
        {
            // 显式发 0，保证底盘停下来
            msg.linear.x  = 0.0;
            msg.angular.z = 0.0;
        }
        else
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Unknown mode: %s", mode_.c_str());
            return;
        }

        pub_->publish(msg);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(),
            1000,
            "mode=%s, lx=%.3f, wz=%.3f",
            mode_.c_str(), msg.linear.x, msg.angular.z);
    }

private:
    // ROS
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // mode
    std::string mode_;

    // constant
    double constant_linear_;
    double constant_angular_;

    // sine
    double amplitude_linear_;
    double amplitude_angular_;
    double frequency_;
    double t_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelPublisher>());
    rclcpp::shutdown();
    return 0;
}
