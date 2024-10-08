#include <sys/stat.h>
#include <sys/types.h>

#include <filesystem>
#include <string>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <fstream>

#include "plansys2_msgs/msg/plan_item.hpp"
#include "plansys2_optic_plan_solver/optic_plan_solver.hpp"

namespace plansys2
{

OPTICPlanSolver::OPTICPlanSolver()
{
}

std::optional<std::filesystem::path>
OPTICPlanSolver::create_folders(const std::string & node_namespace)
{
  RCLCPP_INFO(lc_node_->get_logger(), "OPTICPlanSolver::create_folders");
  auto output_dir = lc_node_->get_parameter(output_dir_parameter_name_).value_to_string();

  // Allow usage of the HOME directory with the `~` character, returning if there is an error.
  const char * home_dir = std::getenv("HOME");
  if (output_dir[0] == '~' && home_dir) {
    output_dir.replace(0, 1, home_dir);
  } else if (!home_dir) {
    RCLCPP_ERROR(
      lc_node_->get_logger(), "Invalid use of the ~ character in the path: %s", output_dir.c_str()
    );
    return std::nullopt;
  }

  // Create the necessary folders, returning if there is an error.
  auto output_path = std::filesystem::path(output_dir);
  if (node_namespace != "") {
    for (auto p : std::filesystem::path(node_namespace) ) {
      if (p != std::filesystem::current_path().root_directory()) {
        output_path /= p;
      }
    }
    try {
      std::filesystem::create_directories(output_path);
    } catch (std::filesystem::filesystem_error & err) {
      RCLCPP_ERROR(lc_node_->get_logger(), "Error writing directories: %s", err.what());
      return std::nullopt;
    }
  }
  return output_path;
}

void OPTICPlanSolver::configure(
  rclcpp_lifecycle::LifecycleNode::SharedPtr lc_node,
  const std::string & plugin_name)
{
  RCLCPP_INFO(lc_node->get_logger(), "OPTICPlanSolver::configure");
  parameter_name_ = plugin_name + ".arguments";
  lc_node_ = lc_node;
  lc_node_->declare_parameter<std::string>(parameter_name_, "");
  RCLCPP_INFO(lc_node_->get_logger(), "OPTICPlanSolver::configure");
}

std::optional<plansys2_msgs::msg::Plan>
OPTICPlanSolver::getPlan(
  const std::string & domain, const std::string & problem,
  const std::string & node_namespace,
  const rclcpp::Duration solver_timeout)
{
  RCLCPP_INFO(lc_node_->get_logger(), "OPTICPlanSolver::getPlan");
  if (node_namespace != "") {
    std::filesystem::path tp = std::filesystem::temp_directory_path();
    for (auto p : std::filesystem::path(node_namespace) ) {
      if (p != std::filesystem::current_path().root_directory()) {
        tp /= p;
      }
    }
    std::filesystem::create_directories(tp);
  }

  plansys2_msgs::msg::Plan ret;
  std::ofstream domain_out("/tmp/" + node_namespace + "/domain.pddl");
  domain_out << domain;
  domain_out.close();

  std::ofstream problem_out("/tmp/" + node_namespace + "/problem.pddl");
  problem_out << problem;
  problem_out.close();
  std::stringstream timeout_s;
  timeout_s << solver_timeout.seconds();
  RCLCPP_INFO(lc_node_->get_logger(), "Sending system command");
  system(
    ("ros2 run optic_planner optic_planner " +
    lc_node_->get_parameter(parameter_name_).value_to_string() +
    " /tmp/" + node_namespace + "/domain.pddl /tmp/" + node_namespace + "/problem.pddl"
    " -x" + timeout_s.str() +
    " -y/tmp/" + node_namespace + "/plan").c_str());
  RCLCPP_INFO(lc_node_->get_logger(), "Waiting for plan");
  std::string line;
  std::ifstream plan_file("/tmp/" + node_namespace + "/plan");
  bool solution = false;

  if (plan_file.is_open()) {
    while (getline(plan_file, line)) {
      if (!solution) {
        if (line.find("Solution Found") != std::string::npos) {
          solution = true;
        }
      } else if (line.front() != ';') {
        plansys2_msgs::msg::PlanItem item;
        size_t colon_pos = line.find(":");
        size_t colon_par = line.find(")");
        size_t colon_bra = line.find("[");

        std::string time = line.substr(0, colon_pos);
        std::string action = line.substr(colon_pos + 2, colon_par - colon_pos - 1);
        std::string duration = line.substr(colon_bra + 1);
        duration.pop_back();
        std::cerr << "Time: " << time << " Action: " << action << " Duration: " << duration <<
          std::endl;

        item.time = std::stof(time);
        item.action = action;
        item.duration = std::stof(duration);

        ret.items.push_back(item);
      }
    }
    plan_file.close();
  }

  if (ret.items.empty()) {
    return {};
  } else {
    return ret;
  }
}

std::string
OPTICPlanSolver::check_domain(
  const std::string & domain,
  const std::string & node_namespace)
{
  if (node_namespace != "") {
    mkdir(("/tmp/" + node_namespace).c_str(), ACCESSPERMS);
  }

  std::ofstream domain_out("/tmp/" + node_namespace + "/check_domain.pddl");
  domain_out << domain;
  domain_out.close();

  std::ofstream problem_out("/tmp/" + node_namespace + "/check_problem.pddl");
  problem_out << "(define (problem void) (:domain plansys2))";
  problem_out.close();

  system(
    ("ros2 run optic_planner optic_planner /tmp/" + node_namespace + "/check_domain.pddl /tmp/" +
    node_namespace + "/check_problem.pddl -x10 -y/tmp/" + node_namespace + "/check.out").c_str());

  std::ifstream plan_file("/tmp/" + node_namespace + "/check.out");

  std::string result((std::istreambuf_iterator<char>(plan_file)),
    std::istreambuf_iterator<char>());

  return result;
}

}  // namespace plansys2

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(plansys2::OPTICPlanSolver, plansys2::PlanSolverBase);
