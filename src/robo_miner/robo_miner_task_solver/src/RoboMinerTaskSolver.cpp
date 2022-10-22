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
	  //printQueryInitialRobotPositionResponse(*result.get());
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
	  //printRobotMoveResponse(*result.get());
  } else {
	RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service %s", ROBOT_MOVE_SERVICE);
  }

  return *result.get();
}

void RoboMinerTaskSolver::doFieldMapValidate(FieldData &data)
{
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "doFieldMapValidate BEGIN");
	  auto request = std::make_shared<FieldMapValidate::Request>();
	  request->field_map.rows = data.size() - 2;
	  request->field_map.cols = data[0].size() - 2;
	  std::vector<uint8_t> data_plain(request->field_map.rows * request->field_map.cols);

	  for (uint32_t row = 0; row < request->field_map.rows; ++row)
	  {
		  for (uint32_t col = 0; col < request->field_map.cols; ++col)
		  {
			  if (data[row+1][col+1] < 0)
			  {
				  data_plain[(row * request->field_map.cols) + col] = data[row + 1][col + 1] + CELL_PROCESSED_MARKER;
			  }
			  else
			  {
				  data_plain[(row * request->field_map.cols) + col] = data[row + 1][col + 1];
			  }
			  printf("%c", data_plain[(row * request->field_map.cols) + col]);
		  }
		  printf("\n");
	  }

      request->field_map.data = data_plain;

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
		  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "success: %d", result.get()->success);
		  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "error_reason: %s", result.get()->error_reason.c_str());
	  } else {
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service %s", FIELD_MAP_VALIDATE_SERVICE);
	  }

	  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "doFieldMapValidate END");
}

void RoboMinerTaskSolver::printQueryInitialRobotPositionResponse(QueryInitialRobotPosition::Response &result)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "robot_initial_tile: %ld", result.robot_initial_tile);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "robot_dir: %ld", result.robot_position_response.robot_dir);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "surrounding_tiles: [left: %ld forward: %ld right: %ld]",
    		result.robot_position_response.surrounding_tiles[0],
			result.robot_position_response.surrounding_tiles[1],
			result.robot_position_response.surrounding_tiles[2]);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "success: %ld", result.robot_position_response.success);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "error_reason: %s", result.robot_position_response.error_reason.c_str());
}

void RoboMinerTaskSolver::printRobotMoveResponse(RobotMove::Response &result)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "robot_dir: %ld", result.robot_position_response.robot_dir);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "surrounding_tiles: [left: %ld forward: %ld right: %ld]",
    		result.robot_position_response.surrounding_tiles[0],
			result.robot_position_response.surrounding_tiles[1],
			result.robot_position_response.surrounding_tiles[2]);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "success: %ld", result.robot_position_response.success);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "error_reason: %s", result.robot_position_response.error_reason.c_str());
}
/*
bool RoboMinerTaskSolver::isValidMove(uint8_t tileForward)
{
	if (tileForward == TILE_OBSTACLE || tileForward == TILE_WALL)
	{
		return false;
	}

	return true;
}

uint32_t RoboMinerTaskSolver::getRandomDir()
{
	return (rand() % RobotMoveType::ROTATE_RIGHT);
}

void RoboMinerTaskSolver::randomMouseMapTraverse()
{
	auto result_query = doQueryInitialRobotPosition();
	auto tile_forward = result_query.robot_position_response.surrounding_tiles[IDX_FORWARD];
	uint32_t move_counter = 0;
	uint8_t reached_walls = 0;
	int32_t up_down_tracker = 0;
	int32_t left_right_tracker = 0;
	int32_t moves_up = 0;
	int32_t moves_down = 0;
	int32_t moves_left = 0;
	int32_t moves_right = 0;

	RobotMove::Response result_robot_move;

	while (reached_walls != 0x0f)
	{
		if (isValidMove_RandomMouse(tile_forward))
		{
			if (move_counter < 5)
			{
				result_robot_move = doRobotMove(RobotMoveType::FORWARD);

				switch (result_robot_move.robot_position_response.robot_dir)
				{
				case RobotPositionResponse::DIRECTION_UP:
					up_down_tracker--;
					if (up_down_tracker < moves_up)
					{
						moves_up = up_down_tracker;
					}
					break;
				case RobotPositionResponse::DIRECTION_DOWN:
					up_down_tracker++;
					if (up_down_tracker > moves_down)
					{
						moves_down = up_down_tracker;
					}
					break;
				case RobotPositionResponse::DIRECTION_LEFT:
					left_right_tracker--;
					if (left_right_tracker < moves_left)
					{
						moves_left = left_right_tracker;
					}
					break;
				case RobotPositionResponse::DIRECTION_RIGHT:
					left_right_tracker++;
					if (left_right_tracker > moves_right)
					{
						moves_right = left_right_tracker;
					}
					break;
				}
				tile_forward = result_robot_move.robot_position_response.surrounding_tiles[IDX_FORWARD];
				move_counter++;
			}
			else
			{
				auto new_dir = getRandomDir();
				if (new_dir == RobotMoveType::ROTATE_RIGHT)
				{
					result_robot_move = sendRobotMove(RobotMoveType::ROTATE_RIGHT);
				}
				else
				{
					result_robot_move = sendRobotMove(RobotMoveType::ROTATE_LEFT);
				}

				tile_forward = result_robot_move.robot_position_response.surrounding_tiles[IDX_FORWARD];
				move_counter = 0;
			}
		}
		else
		{
			if (tile_forward == TILE_WALL)
			{
				reached_walls |= (1 << result_robot_move.robot_position_response.robot_dir);
			}

			auto new_dir = getRandomDir();
			if (new_dir == RobotMoveType::FORWARD)
			{
				// Inverse the direction
				doRobotMove(RobotMoveType::ROTATE_RIGHT);
				result_robot_move = doRobotMove(RobotMoveType::ROTATE_RIGHT);
			}
			else
			{
				result_robot_move = doRobotMove(new_dir);
			}

			tile_forward = result_robot_move.robot_position_response.surrounding_tiles[IDX_FORWARD];
		}
	}

	m_map_width = std::abs(moves_left) + moves_right + 1;
	m_map_height = std::abs(moves_up) + moves_down + 1;

	m_robot_pos_x = std::abs(moves_left);

	if (left_right_tracker != 0)
	{
		m_robot_pos_x += left_right_tracker;
	}

	m_robot_pos_y = std::abs(moves_up);

	if (up_down_tracker != 0)
	{
		m_robot_pos_y += up_down_tracker;
	}

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "map_width: %d; map_height: %d; robot_pos_x: %d; robot_pos_y: %d", m_map_width, m_map_height, m_robot_pos_x, m_robot_pos_y);
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "left_right_tracker: %d; up_down_tracker: %d; robot_pos_x: %d; robot_pos_y: %d",
			left_right_tracker, up_down_tracker, m_robot_pos_x, m_robot_pos_y);
}
*/

bool RoboMinerTaskSolver::isValidMove_DFS(FieldData &data, const FieldPos &location)
{
  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "isValidMove_DFS: data[%d][%d] = %c (%d)", location.row, location.col, data[location.row][location.col], data[location.row][location.col]);

  if (0 >= location.row) {
	return false;
  }

  if (static_cast<int32_t>(data.size()) <= location.row) {
	return false;
  }

  if (0 >= location.col) {
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

void RoboMinerTaskSolver::setMapSell(char &cell, uint8_t tileValue)
{
	if (cell != 0)
	{
		return;
	}

	cell = tileValue;
}

void RoboMinerTaskSolver::setMapCells(FieldData &data, FieldPos &robotPos, uint32_t robotDir, std::array<uint8_t,3> &tileArray)
{
	// TODO set the neighbor cells' value (tile) according to the robot direction
	if (robotDir == RobotPositionResponse::DIRECTION_UP)
	{
		setMapSell(data[robotPos.row - 1][robotPos.col], tileArray[IDX_FORWARD]);
		setMapSell(data[robotPos.row][robotPos.col - 1], tileArray[IDX_LEFT]);
		setMapSell(data[robotPos.row][robotPos.col + 1], tileArray[IDX_RIGHT]);
	}
	else if (robotDir == RobotPositionResponse::DIRECTION_LEFT)
	{
		setMapSell(data[robotPos.row][robotPos.col - 1], tileArray[IDX_FORWARD]);
		setMapSell(data[robotPos.row + 1][robotPos.col], tileArray[IDX_LEFT]);
		setMapSell(data[robotPos.row - 1][robotPos.col], tileArray[IDX_RIGHT]);
	}
	else if (robotDir == RobotPositionResponse::DIRECTION_RIGHT)
	{
		setMapSell(data[robotPos.row][robotPos.col + 1], tileArray[IDX_FORWARD]);
		setMapSell(data[robotPos.row - 1][robotPos.col], tileArray[IDX_LEFT]);
		setMapSell(data[robotPos.row + 1][robotPos.col], tileArray[IDX_RIGHT]);
	}
	else if (robotDir == RobotPositionResponse::DIRECTION_DOWN)
	{
		setMapSell(data[robotPos.row + 1][robotPos.col], tileArray[IDX_FORWARD]);
		setMapSell(data[robotPos.row][robotPos.col + 1], tileArray[IDX_LEFT]);
		setMapSell(data[robotPos.row][robotPos.col - 1], tileArray[IDX_RIGHT]);
	}
}

void RoboMinerTaskSolver::changeRobotDir(uint8_t robotDir, uint8_t newDir)
{
//	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "changeRobotDir: robotDir (%d), newDir(%d)", robotDir, newDir);

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
//		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "changeRobotDir: REVERSE");
		doRobotMove(RobotMoveType::ROTATE_RIGHT);
		doRobotMove(RobotMoveType::ROTATE_RIGHT);
	}

	switch (newDir)
	{
	case RobotPositionResponse::DIRECTION_UP:
		if (robotDir == RobotPositionResponse::DIRECTION_LEFT)
		{
//			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "changeRobotDir: ROTATE_RIGHT");
			doRobotMove(RobotMoveType::ROTATE_RIGHT);
		}
		else if (robotDir == RobotPositionResponse::DIRECTION_RIGHT)
		{
//			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "changeRobotDir: ROTATE_LEFT");
			doRobotMove(RobotMoveType::ROTATE_LEFT);
		}
		break;
	case RobotPositionResponse::DIRECTION_RIGHT:
		if (robotDir == RobotPositionResponse::DIRECTION_UP)
		{
//			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "changeRobotDir: ROTATE_RIGHT");
			doRobotMove(RobotMoveType::ROTATE_RIGHT);
		}
		else if (robotDir == RobotPositionResponse::DIRECTION_DOWN)
		{
//			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "changeRobotDir: ROTATE_LEFT");
			doRobotMove(RobotMoveType::ROTATE_LEFT);
		}
		break;
	case RobotPositionResponse::DIRECTION_DOWN:
		if (robotDir == RobotPositionResponse::DIRECTION_LEFT)
		{
//			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "changeRobotDir: ROTATE_LEFT");
			doRobotMove(RobotMoveType::ROTATE_LEFT);
		}
		else if (robotDir == RobotPositionResponse::DIRECTION_RIGHT)
		{
//			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "changeRobotDir: ROTATE_RIGHT");
			doRobotMove(RobotMoveType::ROTATE_RIGHT);
		}
		break;
	case RobotPositionResponse::DIRECTION_LEFT:
		if (robotDir == RobotPositionResponse::DIRECTION_DOWN)
		{
//			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "changeRobotDir: ROTATE_RIGHT");
			doRobotMove(RobotMoveType::ROTATE_RIGHT);
		}
		else if (robotDir == RobotPositionResponse::DIRECTION_UP)
		{
//			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "changeRobotDir: ROTATE_LEFT");
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

//	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "moveToPrevPos: oldPos(%d,%d), newPos(%d,%d), newRobotDir (%d)", oldPos.row, oldPos.col, newPos.row, newPos.col, newRobotDir);
	changeRobotDir(robotDir, newRobotDir);
	doRobotMove(RobotMoveType::FORWARD);
	oldPos = newPos;
	robotDir = newRobotDir;
}

void RoboMinerTaskSolver::dfsMapTraverse()
{
  std::stack<FieldPos> dataPath;
//  FieldPos robotPos(6, 5);
//  FieldPos mapSize(8, 9); // including the walls
  FieldPos robotPos(2, 10);
  FieldPos mapSize(14, 16); // including the walls
  dataPath.push(robotPos);

  auto result_query = doQueryInitialRobotPosition();
  uint8_t robotDir = result_query.robot_position_response.robot_dir;
  FieldData data(mapSize.row, std::deque<char>(mapSize.col));
  data[robotPos.row][robotPos.col] = result_query.robot_initial_tile - CELL_PROCESSED_MARKER;
  setMapCells(data, robotPos, robotDir, result_query.robot_position_response.surrounding_tiles);

  RobotMove::Response result_robot_move;

  while (!dataPath.empty())
  {
//	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "while BEGIN");
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
//		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ValidMode idx = %d, cellTile(%d,%d) = %c (%d)", idx, dirs[idx].row, dirs[idx].col, data[dirs[idx].row][dirs[idx].col], data[dirs[idx].row][dirs[idx].col]);
		tempVarFound = true;
		dataPath.push(dirs[idx]);
		changeRobotDir(robotDir, idx);
		result_robot_move = doRobotMove(RobotMoveType::FORWARD);
		robotDir = result_robot_move.robot_position_response.robot_dir;
		robotPos = dirs[idx];
		setMapCells(data, robotPos, robotDir, result_robot_move.robot_position_response.surrounding_tiles);
		data[robotPos.row][robotPos.col] -= CELL_PROCESSED_MARKER;
		break;
	  }
	}

//	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "tempVarFound (%d), dataPath.size (%d)", tempVarFound, dataPath.size());
	if (!tempVarFound)
	{
		dataPath.pop();
		if (!dataPath.empty())
		{
			const auto prevPos = dataPath.top();
			moveToPrevPos(robotPos, prevPos, robotDir);
		}
	}
//	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "while END: dataPath.size: %d", dataPath.size());
  }

//  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "while AFTER");

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
