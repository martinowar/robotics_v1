#include "rclcpp/rclcpp.hpp"
#include "robo_miner_task_solver/RoboMinerTaskSolver.h"
//#include "robo_miner_common/defines/RoboMinerTopics.h"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

// TODO use the string from "robo_miner_common/defines/RoboMinerTopics.h"
constexpr auto QUERY_INITIAL_ROBOT_POSITION_SERVICE = "query_initial_robot_position";
constexpr auto ROBOT_MOVE_SERVICE = "move_robot";
constexpr auto FIELD_MAP_VALIDATE_SERVICE = "field_map_validate";

// TODO use from robo_miner_common
FieldPos::FieldPos(int32_t inputRow, int32_t inputCol) {
  row = inputRow;
  col = inputCol;
}

bool FieldPos::operator==(const FieldPos& other) const {
  return (row == other.row && col == other.col);
}

bool FieldPos::operator<(const FieldPos& other) const {
  if (row == other.row) {
    return col < other.col;
  }

  return row < other.row;
}

RoboMinerTaskSolver::RoboMinerTaskSolver() : Node("robo_miner_task_solver"), m_map_width(0), m_map_height(0), m_robot_pos_x(0), m_robot_pos_y(0)
{
}

void RoboMinerTaskSolver::init()
{
  client_query_initial_robot_position = create_client<QueryInitialRobotPosition>(QUERY_INITIAL_ROBOT_POSITION_SERVICE, rmw_qos_profile_services_default);
  client_robot_move = create_client<RobotMove>(ROBOT_MOVE_SERVICE, rmw_qos_profile_services_default);
  client_field_map_validate = create_client<FieldMapValidate>(FIELD_MAP_VALIDATE_SERVICE, rmw_qos_profile_services_default);
}

QueryInitialRobotPosition::Response RoboMinerTaskSolver::doQueryInitialRobotPosition()
{
  auto request = std::make_shared<QueryInitialRobotPosition::Request>();

  while (!client_query_initial_robot_position->wait_for_service(1s)) {
	if (!rclcpp::ok()) {
	  RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
	  exit(1);
	}
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client_query_initial_robot_position->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
	rclcpp::FutureReturnCode::SUCCESS)
  {
	  // print debug information
  } else {
	RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service %s", QUERY_INITIAL_ROBOT_POSITION_SERVICE);
  }

  return *result.get();
}

RobotMove::Response RoboMinerTaskSolver::doRobotMove(uint8_t robotMoveType)
{
  auto request = std::make_shared<RobotMove::Request>();
  request->robot_move_type.move_type = robotMoveType;

  while (!client_robot_move->wait_for_service(1s)) {
	if (!rclcpp::ok()) {
	  RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
	  exit(1);
	}
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client_robot_move->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
	rclcpp::FutureReturnCode::SUCCESS)
  {
	  // print debug information
  } else {
	RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service %s", ROBOT_MOVE_SERVICE);
  }

  return *result.get();
}

void RoboMinerTaskSolver::doFieldMapValidate(FieldData &data)
{
  auto request = std::make_shared<FieldMapValidate::Request>();
  request->field_map.rows = data.size() - 2;
  request->field_map.cols = data[0].size() - 2;
  std::vector<uint8_t> vData(request->field_map.rows * request->field_map.cols);

  for (uint32_t row = 0; row < request->field_map.rows; ++row)
  {
	  for (uint32_t col = 0; col < request->field_map.cols; ++col)
	  {
		  if (data[row+1][col+1] < 0)
		  {
			  vData[(row * request->field_map.cols) + col] = data[row + 1][col + 1] + CELL_PROCESSED_MARKER;
		  }
		  else
		  {
			  vData[(row * request->field_map.cols) + col] = data[row + 1][col + 1];
		  }
	  }
  }

  request->field_map.data = vData;

  while (!client_field_map_validate->wait_for_service(1s)) {
	if (!rclcpp::ok()) {
	  RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
	  exit(1);
	}
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client_field_map_validate->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
	rclcpp::FutureReturnCode::SUCCESS)
  {
	  // print debug information
  } else {
	RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service %s", FIELD_MAP_VALIDATE_SERVICE);
  }
}

bool RoboMinerTaskSolver::isValidMove_DFS(FieldData &data, const FieldPos &location)
{
  if (m_mapTopLeftPos.row > location.row) {
	return false;
  }

  if (m_mapBottomRightPos.row < location.row) {
	return false;
  }

  if (m_mapTopLeftPos.col > location.col) {
	return false;
  }

  if (m_mapBottomRightPos.col < location.col) {
	return false;
  }

  auto phyPos = getPhysicalPos(location);

  if (   (data[phyPos.row][phyPos.col] == TILE_OBSTACLE)
      || (data[phyPos.row][phyPos.col] == TILE_WALL)
	  || (data[phyPos.row][phyPos.col] <= 0) // the cell was already processed
  ) {
	return false;
  }

  return true;
}

FieldPos RoboMinerTaskSolver::getPhysicalPos(const FieldPos &logicalPos)
{
	FieldPos phyPos;

	phyPos.col = std::abs(m_mapTopLeftPos.col);
	if (logicalPos.col != 0)
	{
		phyPos.col += logicalPos.col;
	}

	phyPos.row = std::abs(m_mapTopLeftPos.row);
	if (logicalPos.row != 0)
	{
		phyPos.row += logicalPos.row;
	}

	return FieldPos(phyPos.row, phyPos.col);
}

void RoboMinerTaskSolver::setMapCell(FieldData &data, const FieldPos &cellPos, uint8_t tileValue)
{
	if (cellPos.row < m_mapTopLeftPos.row)
	{
		data.push_front(std::deque<char>(data[0].size(), 0));
		m_mapTopLeftPos.row--;
	}
	else if (cellPos.row > m_mapBottomRightPos.row)
	{
		data.push_back(std::deque<char>(data[0].size(), 0));
		m_mapBottomRightPos.row++;
	}

	if (cellPos.col < m_mapTopLeftPos.col)
	{
		for (uint32_t idx = 0; idx < data.size(); ++idx)
		{
			data[idx].push_front(0);
		}

		m_mapTopLeftPos.col--;
	}
	else if (cellPos.col > m_mapBottomRightPos.col)
	{
		for (uint32_t idx = 0; idx < data.size(); ++idx)
		{
			data[idx].push_back(0);
		}

		m_mapBottomRightPos.col++;
	}

	auto phyPos = getPhysicalPos(cellPos);

	if (data[phyPos.row][phyPos.col] != 0)
	{
		return;
	}

	data[phyPos.row][phyPos.col] = tileValue;
}

void RoboMinerTaskSolver::setMapCells(FieldData &data, FieldPos &robotPos, uint32_t robotDir, std::array<uint8_t,3> &tileArray)
{
	if (robotDir == RobotPositionResponse::DIRECTION_UP)
	{
		setMapCell(data, FieldPos(robotPos.row - 1, robotPos.col), tileArray[IDX_FORWARD]);
		setMapCell(data, FieldPos(robotPos.row, robotPos.col - 1), tileArray[IDX_LEFT]);
		setMapCell(data, FieldPos(robotPos.row, robotPos.col + 1), tileArray[IDX_RIGHT]);
	}
	else if (robotDir == RobotPositionResponse::DIRECTION_LEFT)
	{
		setMapCell(data, FieldPos(robotPos.row, robotPos.col - 1), tileArray[IDX_FORWARD]);
		setMapCell(data, FieldPos(robotPos.row + 1, robotPos.col), tileArray[IDX_LEFT]);
		setMapCell(data, FieldPos(robotPos.row - 1, robotPos.col), tileArray[IDX_RIGHT]);
	}
	else if (robotDir == RobotPositionResponse::DIRECTION_RIGHT)
	{
		setMapCell(data, FieldPos(robotPos.row, robotPos.col + 1), tileArray[IDX_FORWARD]);
		setMapCell(data, FieldPos(robotPos.row - 1, robotPos.col), tileArray[IDX_LEFT]);
		setMapCell(data, FieldPos(robotPos.row + 1, robotPos.col), tileArray[IDX_RIGHT]);
	}
	else if (robotDir == RobotPositionResponse::DIRECTION_DOWN)
	{
		setMapCell(data, FieldPos(robotPos.row + 1, robotPos.col), tileArray[IDX_FORWARD]);
		setMapCell(data, FieldPos(robotPos.row, robotPos.col + 1), tileArray[IDX_LEFT]);
		setMapCell(data, FieldPos(robotPos.row, robotPos.col - 1), tileArray[IDX_RIGHT]);
	}
}

void RoboMinerTaskSolver::changeRobotDir(uint8_t robotDir, uint8_t newDir)
{
	if (robotDir == newDir)
	{
		return;
	}

	if (    ((robotDir == RobotPositionResponse::DIRECTION_UP) && (newDir == RobotPositionResponse::DIRECTION_DOWN))
		 || ((robotDir == RobotPositionResponse::DIRECTION_DOWN) && (newDir == RobotPositionResponse::DIRECTION_UP))
		 || ((robotDir == RobotPositionResponse::DIRECTION_LEFT) && (newDir == RobotPositionResponse::DIRECTION_RIGHT))
		 || ((robotDir == RobotPositionResponse::DIRECTION_RIGHT) && (newDir == RobotPositionResponse::DIRECTION_LEFT))
	   )
	{
		doRobotMove(RobotMoveType::ROTATE_RIGHT);
		doRobotMove(RobotMoveType::ROTATE_RIGHT);
	}

	switch (newDir)
	{
	case RobotPositionResponse::DIRECTION_UP:
		if (robotDir == RobotPositionResponse::DIRECTION_LEFT)
		{
			doRobotMove(RobotMoveType::ROTATE_RIGHT);
		}
		else if (robotDir == RobotPositionResponse::DIRECTION_RIGHT)
		{
			doRobotMove(RobotMoveType::ROTATE_LEFT);
		}
		break;
	case RobotPositionResponse::DIRECTION_RIGHT:
		if (robotDir == RobotPositionResponse::DIRECTION_UP)
		{
			doRobotMove(RobotMoveType::ROTATE_RIGHT);
		}
		else if (robotDir == RobotPositionResponse::DIRECTION_DOWN)
		{
			doRobotMove(RobotMoveType::ROTATE_LEFT);
		}
		break;
	case RobotPositionResponse::DIRECTION_DOWN:
		if (robotDir == RobotPositionResponse::DIRECTION_LEFT)
		{
			doRobotMove(RobotMoveType::ROTATE_LEFT);
		}
		else if (robotDir == RobotPositionResponse::DIRECTION_RIGHT)
		{
			doRobotMove(RobotMoveType::ROTATE_RIGHT);
		}
		break;
	case RobotPositionResponse::DIRECTION_LEFT:
		if (robotDir == RobotPositionResponse::DIRECTION_DOWN)
		{
			doRobotMove(RobotMoveType::ROTATE_RIGHT);
		}
		else if (robotDir == RobotPositionResponse::DIRECTION_UP)
		{
			doRobotMove(RobotMoveType::ROTATE_LEFT);
		}
		break;
	}
}

void RoboMinerTaskSolver::moveToPrevPos(FieldPos &oldPos, const FieldPos &newPos, uint8_t &robotDir)
{
	uint8_t newRobotDir;

	if (oldPos.row == newPos.row)
	{
		if (oldPos.col < newPos.col)
		{
			newRobotDir = RobotPositionResponse::DIRECTION_RIGHT;
		}
		else
		{
			newRobotDir = RobotPositionResponse::DIRECTION_LEFT;
		}
	}
	else if (oldPos.row > newPos.row)
	{
		newRobotDir = RobotPositionResponse::DIRECTION_UP;
	}
	else
	{
		newRobotDir = RobotPositionResponse::DIRECTION_DOWN;
	}

	changeRobotDir(robotDir, newRobotDir);
	doRobotMove(RobotMoveType::FORWARD);
	oldPos = newPos;
	robotDir = newRobotDir;
}

void RoboMinerTaskSolver::dfsMapTraverse()
{
  std::stack<FieldPos> dataPath;
  FieldPos robotPos(0, 0);
  dataPath.push(robotPos);
  m_mapTopLeftPos = robotPos;
  m_mapBottomRightPos = robotPos;

  auto result_query = doQueryInitialRobotPosition();
  uint8_t robotDir = result_query.robot_position_response.robot_dir;
  FieldData data(1, std::deque<char>(1, 0));
  data[robotPos.row][robotPos.col] = result_query.robot_initial_tile - CELL_PROCESSED_MARKER;
  setMapCells(data, robotPos, robotDir, result_query.robot_position_response.surrounding_tiles);

  RobotMove::Response result_robot_move;

  while (!dataPath.empty())
  {
	const auto currLocation = dataPath.top();

	constexpr auto arrSize = 4;
	const std::array<FieldPos, arrSize> dirs = {
			FieldPos { currLocation.row - 1, currLocation.col},
			FieldPos { currLocation.row,     currLocation.col + 1 },
			FieldPos { currLocation.row + 1, currLocation.col },
		    FieldPos { currLocation.row,     currLocation.col - 1 } };

	bool tempVarFound = false;
	for (uint8_t idx = 0; idx < 4; ++idx)
	{
	  if (isValidMove_DFS(data, dirs[idx])) {
		tempVarFound = true;
		dataPath.push(dirs[idx]);
		changeRobotDir(robotDir, idx);
		result_robot_move = doRobotMove(RobotMoveType::FORWARD);
		robotDir = result_robot_move.robot_position_response.robot_dir;
		robotPos = dirs[idx];
		setMapCells(data, robotPos, robotDir, result_robot_move.robot_position_response.surrounding_tiles);
		auto phyPos = getPhysicalPos(robotPos);
		data[phyPos.row][phyPos.col] -= CELL_PROCESSED_MARKER;
		break;
	  }
	}

	if (!tempVarFound)
	{
		dataPath.pop();
		if (!dataPath.empty())
		{
			const auto prevPos = dataPath.top();
			moveToPrevPos(robotPos, prevPos, robotDir);
		}
	}
  }

  doFieldMapValidate(data);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<RoboMinerTaskSolver> node_task_solver = std::make_shared<RoboMinerTaskSolver>();
  node_task_solver->init();

//  node_task_solver->randomMouseMapTraverse();
  node_task_solver->dfsMapTraverse();

  rclcpp::shutdown();
  return 0;
}
