#include "rclcpp/rclcpp.hpp"
#include "robo_miner_task_solver/RoboMinerTaskSolver.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<RoboMinerTaskSolver> nodeTaskSolver = std::make_shared<RoboMinerTaskSolver>();
  nodeTaskSolver->init();

  nodeTaskSolver->run();

  rclcpp::shutdown();
  return 0;
}
