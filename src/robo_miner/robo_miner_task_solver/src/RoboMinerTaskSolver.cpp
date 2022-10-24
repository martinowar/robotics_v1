#include "rclcpp/rclcpp.hpp"
#include "robo_miner_task_solver/RoboMinerTaskSolver.h"
#include "robo_miner_task_solver/FloodFill.h"
//#include "robo_miner_common/defines/RoboMinerTopics.h"

#include <chrono>

using namespace std::chrono_literals;

// TODO use the string from "robo_miner_common/defines/RoboMinerTopics.h"
constexpr auto QUERY_INITIAL_ROBOT_POSITION_SERVICE = "query_initial_robot_position";
constexpr auto ROBOT_MOVE_SERVICE = "move_robot";
constexpr auto FIELD_MAP_VALIDATE_SERVICE = "field_map_validate";
constexpr auto LONGEST_SEQUENCE_VALIDATE_SERVICE = "longest_sequence_validate";
constexpr auto ACTIVATE_MINING_VALIDATE_SERVICE = "activate_mining_validate";

RoboMinerTaskSolver::RoboMinerTaskSolver() : Node("robo_miner_task_solver")
{
}

void RoboMinerTaskSolver::init()
{
  m_clientQueryInitialRobotPosition = create_client<QueryInitialRobotPosition>(QUERY_INITIAL_ROBOT_POSITION_SERVICE, rmw_qos_profile_services_default);
  m_clientRobotMove = create_client<RobotMove>(ROBOT_MOVE_SERVICE, rmw_qos_profile_services_default);
  m_clientFieldMapValidate = create_client<FieldMapValidate>(FIELD_MAP_VALIDATE_SERVICE, rmw_qos_profile_services_default);
  m_clientLongestSequenceValidate = create_client<LongestSequenceValidate>(LONGEST_SEQUENCE_VALIDATE_SERVICE, rmw_qos_profile_services_default);
  m_clientActivateMiningValidate = create_client<ActivateMiningValidate>(ACTIVATE_MINING_VALIDATE_SERVICE, rmw_qos_profile_services_default);
}

QueryInitialRobotPosition::Response RoboMinerTaskSolver::doQueryInitialRobotPosition()
{
  auto request = std::make_shared<QueryInitialRobotPosition::Request>();

  while (!m_clientQueryInitialRobotPosition->wait_for_service(1s)) {
	if (!rclcpp::ok()) {
	  RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
	  exit(1);
	}
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = m_clientQueryInitialRobotPosition->async_send_request(request);
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

RobotMove::Response RoboMinerTaskSolver::doRobotMove(const uint8_t robotMoveType)
{
  auto request = std::make_shared<RobotMove::Request>();
  request->robot_move_type.move_type = robotMoveType;

  while (!m_clientRobotMove->wait_for_service(1s)) {
	if (!rclcpp::ok()) {
	  RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
	  exit(1);
	}
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = m_clientRobotMove->async_send_request(request);
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
  // the 'data' contains the wall tiles; they should be skipped
  request->field_map.rows = data.size() - WALL_CELL - WALL_CELL;
  request->field_map.cols = data[0].size() - WALL_CELL - WALL_CELL;
  std::vector<uint8_t> vData(request->field_map.rows * request->field_map.cols);

  for (uint32_t row = 0; row < request->field_map.rows; ++row)
  {
	for (uint32_t col = 0; col < request->field_map.cols; ++col)
	{
	  if (data[row + WALL_CELL][col + WALL_CELL] < 0)
	  {
		// Remove the CELL_PROCESSED_MARKER offset
		data[row + WALL_CELL][col + WALL_CELL] += CELL_PROCESSED_MARKER;
	  }

	  vData[(row * request->field_map.cols) + col] = data[row + WALL_CELL][col + WALL_CELL];
	}
  }

  request->field_map.data = vData;

  while (!m_clientFieldMapValidate->wait_for_service(1s)) {
	if (!rclcpp::ok()) {
	  RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
	  exit(1);
	}
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = m_clientFieldMapValidate->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
	rclcpp::FutureReturnCode::SUCCESS)
  {
	  // print debug information
  } else {
	RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service %s", FIELD_MAP_VALIDATE_SERVICE);
  }
}

void RoboMinerTaskSolver::doLongestSequenceValidate(std::vector<FieldPos> &longestSequence)
{
  auto request = std::make_shared<LongestSequenceValidate::Request>();
  std::vector<FieldPoint> tmpSequence;

  for (auto &cellPos : longestSequence)
  {
	  FieldPoint tmpFieldPoint;
	  tmpFieldPoint.row = cellPos.row;
	  tmpFieldPoint.col = cellPos.col;
	  tmpSequence.push_back(tmpFieldPoint);
  }
  request->sequence_points = tmpSequence;

  while (!m_clientLongestSequenceValidate->wait_for_service(1s)) {
	if (!rclcpp::ok()) {
	  RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
	  exit(1);
	}
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = m_clientLongestSequenceValidate->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
	rclcpp::FutureReturnCode::SUCCESS)
  {
	  // print debug information
	  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "success (%d); error_reason = %s", result.get()->success, result.get()->error_reason.c_str());
  } else {
	RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service %s", LONGEST_SEQUENCE_VALIDATE_SERVICE);
  }
}

void RoboMinerTaskSolver::doActivateMiningValidate()
{
  auto request = std::make_shared<ActivateMiningValidate::Request>();

  while (!m_clientActivateMiningValidate->wait_for_service(1s)) {
	if (!rclcpp::ok()) {
	  RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
	  exit(1);
	}
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = m_clientActivateMiningValidate->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
	rclcpp::FutureReturnCode::SUCCESS)
  {
	  // print debug information
	  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "success (%d); error_reason = %s", result.get()->success, result.get()->error_reason.c_str());
  } else {
	RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service %s", ACTIVATE_MINING_VALIDATE_SERVICE);
  }
}

bool RoboMinerTaskSolver::isValidMove(const FieldData &data, const FieldPos &location) const
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

FieldPos RoboMinerTaskSolver::getPhysicalPos(const FieldPos &logicalPos) const
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

void RoboMinerTaskSolver::setMapCell( FieldData &data, const FieldPos &cellPos, const uint8_t tileValue)
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

void RoboMinerTaskSolver::setAdjacentCells(FieldData &data, const FieldPos &robotPos, const uint32_t robotDir, const std::array<uint8_t,3> &tileArray)
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

void RoboMinerTaskSolver::changeRobotDir(const uint8_t robotDir, const uint8_t newDir)
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

void cleanupMapData(FieldData &data)
{
	data.erase(data.begin());
	data.erase(data.end());

	for (auto &row : data)
	{
		row.erase(row.begin());
		row.erase(row.end());
	}
}

bool RoboMinerTaskSolver::isValidMoveWhenGoToCell(const FieldData &data, const FieldPos &location) const
{
  if (0 > location.row) {
	return false;
  }

  if (static_cast<int32_t>(data.size()) <= location.row) {
	return false;
  }

  if (0 > location.col) {
	return false;
  }

  if (!data[0].empty() &&
	  static_cast<int32_t>(data[0].size()) <= location.col) {
	return false;
  }

  if (   (data[location.row][location.col] == TILE_OBSTACLE)
      || (data[location.row][location.col] == TILE_WALL)
	  || (data[location.row][location.col] <= 0) // the cell was already processed
  ) {
	return false;
  }

  return true;
}

void RoboMinerTaskSolver::moveToCell(FieldData &data, FieldPos &robotPos, const FieldPos &cellPos, uint8_t &robotDir)
{
//  printf("moveToCell: robotPos (%d %d); cellPos (%d %d); robotDir (%d)\n", robotPos.row, robotPos.col, cellPos.row, cellPos.col, robotDir);
  std::stack<FieldPos> dataPath;
  dataPath.push(robotPos);

  RobotMove::Response result_robot_move;

  while (!((robotPos.row == cellPos.row) && (robotPos.col == cellPos.col)))
  {
//	printf("while BEGIN\n");
//	printf("moveToCell: robotPos (%d %d); cellPos (%d %d); robotDir (%d)\n", robotPos.row, robotPos.col, cellPos.row, cellPos.col, robotDir);
	const auto currLocation = dataPath.top();

	constexpr auto directionCount = 4;
	const std::array<FieldPos, directionCount> dirs = {
			FieldPos { currLocation.row - 1, currLocation.col},
			FieldPos { currLocation.row,     currLocation.col + 1 },
			FieldPos { currLocation.row + 1, currLocation.col },
			FieldPos { currLocation.row,     currLocation.col - 1 } };

	bool validMoveFound = false;
	for (uint8_t idx = 0; idx < directionCount; ++idx)
	{
	  if (isValidMoveWhenGoToCell(data, dirs[idx])) {
		validMoveFound = true;
		dataPath.push(dirs[idx]);
		changeRobotDir(robotDir, idx);
		result_robot_move = doRobotMove(RobotMoveType::FORWARD);
		robotDir = result_robot_move.robot_position_response.robot_dir;
		robotPos = dirs[idx];
		data[robotPos.row][robotPos.col] -= CELL_PROCESSED_MARKER;
		break;
	  }
	}

	if (!validMoveFound)
	{
	  dataPath.pop();
	  if (!dataPath.empty())
	  {
		const auto prevPos = dataPath.top();
		moveToPrevPos(robotPos, prevPos, robotDir);
	  }
	}

//	printf("moveToCell: robotPos (%d %d); cellPos (%d %d); robotDir (%d)\n", robotPos.row, robotPos.col, cellPos.row, cellPos.col, robotDir);
//	printf("while END\n");
  }
}

bool RoboMinerTaskSolver::isValidMoveWhileMining(const FieldData &data, const FieldPos &location, const int8_t crystalValue) const
{
  if (0 > location.row) {
	return false;
  }

  if (static_cast<int32_t>(data.size()) <= location.row) {
	return false;
  }

  if (0 > location.col) {
	return false;
  }

  if (!data[0].empty() &&
	  static_cast<int32_t>(data[0].size()) <= location.col) {
	return false;
  }

  if ((data[location.row][location.col] >= 0) &&
      (data[location.row][location.col] != crystalValue))
  {
	return false;
  }

  return true;
}

void RoboMinerTaskSolver::mineLongestSequence(FieldData &data, FieldPos &robotPos, uint8_t &robotDir)
{
  printf("moveToCell: robotPos (%d %d); robotDir (%d)\n", robotPos.row, robotPos.col, robotDir);

  auto crystalValue = data[robotPos.row][robotPos.col];
  std::stack<FieldPos> dataPath;
  dataPath.push(robotPos);

  RobotMove::Response result_robot_move;

  while (!dataPath.empty())
  {
//	printf("while BEGIN\n");
//	printf("moveToCell: robotPos (%d %d); cellPos (%d %d); robotDir (%d)\n", robotPos.row, robotPos.col, cellPos.row, cellPos.col, robotDir);
	const auto currLocation = dataPath.top();

	constexpr auto directionCount = 4;
	const std::array<FieldPos, directionCount> dirs = {
			FieldPos { currLocation.row - 1, currLocation.col},
			FieldPos { currLocation.row,     currLocation.col + 1 },
			FieldPos { currLocation.row + 1, currLocation.col },
			FieldPos { currLocation.row,     currLocation.col - 1 } };

	bool validMoveFound = false;
	for (uint8_t idx = 0; idx < directionCount; ++idx)
	{
	  if (isValidMoveWhileMining(data, dirs[idx], crystalValue)) {
		validMoveFound = true;
		dataPath.push(dirs[idx]);
		changeRobotDir(robotDir, idx);
		result_robot_move = doRobotMove(RobotMoveType::FORWARD);
		robotDir = result_robot_move.robot_position_response.robot_dir;
		robotPos = dirs[idx];
		data[robotPos.row][robotPos.col] -= CELL_PROCESSED_MARKER;
		break;
	  }
	}

	if (!validMoveFound)
	{
	  dataPath.pop();
	  if (!dataPath.empty())
	  {
		const auto prevPos = dataPath.top();
		moveToPrevPos(robotPos, prevPos, robotDir);
	  }
	}

//	printf("moveToCell: robotPos (%d %d); cellPos (%d %d); robotDir (%d)\n", robotPos.row, robotPos.col, cellPos.row, cellPos.col, robotDir);
//	printf("while END\n");
  }
}

void RoboMinerTaskSolver::mapTraverseAndValidate()
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
  setAdjacentCells(data, robotPos, robotDir, result_query.robot_position_response.surrounding_tiles);

  RobotMove::Response result_robot_move;

  while (!dataPath.empty())
  {
	const auto currLocation = dataPath.top();

	constexpr auto directionCount = 4;
	const std::array<FieldPos, directionCount> dirs = {
			FieldPos { currLocation.row - 1, currLocation.col},
			FieldPos { currLocation.row,     currLocation.col + 1 },
			FieldPos { currLocation.row + 1, currLocation.col },
		    FieldPos { currLocation.row,     currLocation.col - 1 } };

	bool validMoveFound = false;
	for (uint8_t idx = 0; idx < directionCount; ++idx)
	{
	  if (isValidMove(data, dirs[idx])) {
		validMoveFound = true;
		dataPath.push(dirs[idx]);
		changeRobotDir(robotDir, idx);
		result_robot_move = doRobotMove(RobotMoveType::FORWARD);
		robotDir = result_robot_move.robot_position_response.robot_dir;
		robotPos = dirs[idx];
		setAdjacentCells(data, robotPos, robotDir, result_robot_move.robot_position_response.surrounding_tiles);
		auto phyPos = getPhysicalPos(robotPos);
		data[phyPos.row][phyPos.col] -= CELL_PROCESSED_MARKER;
		break;
	  }
	}

	if (!validMoveFound)
	{
      dataPath.pop();
	  if (!dataPath.empty())
	  {
	    const auto prevPos = dataPath.top();
		moveToPrevPos(robotPos, prevPos, robotDir);
	  }
    }
  }

  // Convert to physical coordinates and decrease the rol/col due to the wall
  robotPos = getPhysicalPos(robotPos);
  robotPos.row--;
  robotPos.col--;

  doFieldMapValidate(data);

  cleanupMapData(data);

  auto longestCrystalSequence = FloodFill::findLongestCrystalSequence(data, std::vector<char>('#', 'X'));

  doLongestSequenceValidate(longestCrystalSequence);

  auto tmpData = data;
  moveToCell(tmpData, robotPos, longestCrystalSequence[0], robotDir);
  doActivateMiningValidate();

  mineLongestSequence(data, robotPos, robotDir);
}
