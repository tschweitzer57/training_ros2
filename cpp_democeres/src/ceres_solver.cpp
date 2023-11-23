// Demo using ceres helloworld to run in ros2
// CECILL-2.1 Licence

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "ceres/ceres.h"
#include "glog/logging.h"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a 
 * member function as a callback from the timer. */
 
 struct CostFunctor {
		template <typename T>
		bool operator()(const T* const x, T* residual) const {
		residual[0] = 10.0 - x[0];
		return true;
		}
	};

class CeresPublisher : public rclcpp::Node
{
public:
  CeresPublisher()
  : Node("ceres_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&CeresPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    double x = 0.5;
	const double initial_x = x;
	
	ceres::Problem problem;
	
	ceres::CostFunction* cost_function =
      new ceres::AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);
	problem.AddResidualBlock(cost_function, nullptr, &x);
	
	ceres::Solver::Options options;
	options.minimizer_progress_to_stdout = true;
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	std::cout << summary.BriefReport() << "\n";
	RCLCPP_INFO(this->get_logger(), "Report '%s'", summary.BriefReport().c_str());
	std::cout << "x : " << initial_x << " -> " << x << "\n";
	RCLCPP_INFO(this->get_logger(), "x : %f -> %f", x, initial_x);
	
	auto message = std_msgs::msg::String();
    message.data = "Hello, world !" + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  google::InitGoogleLogging(argv[0]);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CeresPublisher>());
  rclcpp::shutdown();
  return 0;
}