#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include <limits.h>
#include <unistd.h>

#include <vector>
#include <string>
#include <fstream>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include "rclcpp/rclcpp.hpp"
#include "ips_cam/ips_cam_node.hpp"

bool is_integer(const std::string & s)
{
  return !s.empty() &&
         std::find_if(s.begin(), s.end(), [](unsigned char c) {return !std::isdigit(c);}) ==
         s.end();
}

bool is_boolean(const std::string & s)
{
  return s == "true" || s == "false";
}

bool is_float(const std::string & s)
{
  std::istringstream iss(s);
  float f;
  iss >> std::noskipws >> f;    // Try to parse the string as a float
  return iss.eof() && !iss.fail();    // Ensure the entire string was parsed
}


// Helper function to load parameters from a YAML file into a vector of rclcpp::Parameter
std::vector<rclcpp::Parameter> load_parameters_from_yaml(
  const std::string & yaml_filename, const std::string & node_name)
{
  YAML::Node yaml_file = YAML::LoadFile(yaml_filename);
  std::vector<rclcpp::Parameter> parameters;

  if (yaml_file[node_name] && yaml_file[node_name]["ros__parameters"]) {
    auto params_node = yaml_file[node_name]["ros__parameters"];

    for (const auto & param : params_node) {
      std::string param_name = param.first.as<std::string>();

      if (param.second.IsScalar()) {
        std::string param_as_string = param.second.as<std::string>();

        // Check if it's an integer without using exceptions
        if (is_integer(param_as_string)) {
          parameters.emplace_back(param_name, param.second.as<int>());
        } else if (is_boolean(param_as_string)) {
          // Check if it's a boolean without using exceptions
          parameters.emplace_back(param_name, param.second.as<bool>());
        } else if (is_float(param_as_string)) {
          // Check if it's a floating point number
          parameters.emplace_back(param_name, param.second.as<double>());
        } else {
          // If no matching type, store as string or handle the type conversion failure
          parameters.emplace_back(param_name, param.second.as<std::string>());
        }
      }
    }
  }
  return parameters;
}

std::string getexepath()
{
  char result[PATH_MAX];
  ssize_t count = readlink("/proc/self/exe", result, PATH_MAX);
  return std::string(result, (count > 0) ? count : 0);
}

void dump(geometry_msgs::msg::Quaternion q)
{
  std::cout << "(" << q.w << "," << q.x << "," << q.y << "," << q.z << ")" << std::endl;
}

class MyNode : public rclcpp::Node
{
public:
  explicit MyNode(const rclcpp::NodeOptions & node_options)
  : Node("my_node", node_options)
  {
    this->declare_parameter<int>("param1", 0);
    this->declare_parameter<std::string>("param2", "Cheese");
    this->declare_parameter<bool>("param3", false);

    this->get_parameter("param1", param1);
    this->get_parameter("param2", param2);
    this->get_parameter("param3", param3);

    std::cout << param1 << std::endl;
    std::cout << param2 << std::endl;
    std::cout << param3 << std::endl;

    RCLCPP_INFO(this->get_logger(), "param1: %d", param1);
    RCLCPP_INFO(this->get_logger(), "param2: %s", param2.c_str());
    RCLCPP_INFO(this->get_logger(), "param3: %d", param3);
  }

public:
  int param1;
  std::string param2;
  bool param3;
};

TEST(test_node, test_params)
{
  rclcpp::init(0, nullptr);

  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  std::cout << getexepath() << std::endl;

  // Load parameters from the YAML file
  std::string yaml_file = "../../src/ips_cam/config/test_params.yaml";
  std::vector<rclcpp::Parameter> parameters = load_parameters_from_yaml(yaml_file, "/**");

  std::cout << parameters.size() << std::endl;

  std::cout << parameters[0].value_to_string() << std::endl;
  std::cout << parameters[1].value_to_string() << std::endl;
  std::cout << parameters[2].value_to_string() << std::endl;

  // Create NodeOptions and pass the loaded parameters
  rclcpp::NodeOptions options;
  options.parameter_overrides(parameters);    // Pass parameters to NodeOptions

  auto node = std::make_shared<MyNode>(options);

  executor->add_node(node);

  std::thread t(
    [&]() {
      ASSERT_NO_THROW(executor->spin(););
    }
  );

  std::this_thread::sleep_for(std::chrono::seconds(5));
  executor->cancel();
  t.join();    // Wait for thread completion
  // executor.spin_once();
  // executor.cancel();

  rclcpp::shutdown();
}

TEST(test_node, test_ips_node)
{
  rclcpp::init(0, nullptr);

  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  std::cout << getexepath() << std::endl;

  // Load parameters from the YAML file
  std::string yaml_file1 = "~/IndoorPositioningSystem/ips_config/node_params.yaml";
  std::string yaml_file = ips_cam::expand_and_check(yaml_file1);
  std::vector<rclcpp::Parameter> parameters = load_parameters_from_yaml(yaml_file, "/**");

  std::cout << parameters.size() << std::endl;

  // Create NodeOptions and pass the loaded parameters
  rclcpp::NodeOptions options;
  options.parameter_overrides(parameters);    // Pass parameters to NodeOptions

  auto node = std::make_shared<ips_cam::IpsCamNode>(options);

  executor->add_node(node);

  std::thread t(
    [&]() {
      ASSERT_NO_THROW(executor->spin(););
    }
  );

  std::this_thread::sleep_for(std::chrono::seconds(20));
  executor->cancel();
  t.join();    // Wait for thread completion
  // executor.spin_once();
  // executor.cancel();

  rclcpp::shutdown();
}

TEST(test_node, test_read_yaml)
{
  // Load parameters from the YAML file
  std::string yaml_file1 = "~/IndoorPositioningSystem/ips_config/node_params.yaml";
  std::string yaml_file = ips_cam::expand_and_check(yaml_file1);
  std::vector<rclcpp::Parameter> parameters = load_parameters_from_yaml(yaml_file, "/**");
  std::cout << parameters.size() << std::endl;
}

TEST(test_node, test_quat)
{
  ips_cam::TagPose tp;
  geometry_msgs::msg::Pose ros_pose;
  tp.x = 1.0;
  tp.y = 2.0;
  tp.z = 0.0;
  tp.theta = 0.5;
  ips_cam::from_tag_pose(tp, ros_pose);

  dump(ros_pose.orientation);
  std::cout << std::sin(0.5 / 2.0) << std::endl;
  std::cout << std::cos(0.5 / 2.0) << std::endl;
  std::cout << ros_pose.position.x << std::endl;
  std::cout << ros_pose.position.y << std::endl;

  ASSERT_EQ(ros_pose.position.x, tp.x);
  ASSERT_EQ(ros_pose.position.y, tp.y);
  ASSERT_EQ(ros_pose.orientation.w, std::cos(tp.theta / 2.0));
  ASSERT_EQ(ros_pose.orientation.z, std::sin(tp.theta / 2.0));
  ASSERT_EQ(ros_pose.orientation.x, 0);
  ASSERT_EQ(ros_pose.orientation.y, 0);
}
