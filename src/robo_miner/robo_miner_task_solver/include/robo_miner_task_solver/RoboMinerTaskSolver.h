#ifndef ROBO_MINER_TASK_SOLVER_H_
#define ROBO_MINER_TASK_SOLVER_H_

#include "robo_miner_interfaces/srv/query_initial_robot_position.hpp"
#include "robo_miner_interfaces/srv/field_map_validate.hpp"
#include "robo_miner_interfaces/srv/robot_move.hpp"
#include "robo_miner_interfaces/msg/robot_move_type.hpp"
#include "robo_miner_interfaces/msg/robot_position_response.hpp"

using QueryInitialRobotPosition = robo_miner_interfaces::srv::QueryInitialRobotPosition;
using FieldMapValidate = robo_miner_interfaces::srv::FieldMapValidate;
using RobotMove = robo_miner_interfaces::srv::RobotMove;
using RobotMoveType = robo_miner_interfaces::msg::RobotMoveType;
using RobotPositionResponse = robo_miner_interfaces::msg::RobotPositionResponse;

//System headers
#include <cstdint>
#include <deque>
#include <stack>

struct FieldPos {
  FieldPos() = default;
  FieldPos(int32_t row, int32_t col);

  bool operator==(const FieldPos& other) const;
  bool operator<(const FieldPos& other) const;

  int32_t row { 0 };
  int32_t col { 0 };
};

class RoboMinerTaskSolver : public rclcpp::Node
{
public:
	RoboMinerTaskSolver();
	void init();
	void randomMouseMapTraverse();
	void dfsMapTraverse();

private:
	// TODO use FieldData from "robo_miner_common
	using FieldData = std::deque<std::deque<char>>;

	static constexpr char TILE_OBSTACLE = 'X';
	static constexpr char TILE_WALL = '#';
	static constexpr uint8_t IDX_LEFT = 0;
	static constexpr uint8_t IDX_FORWARD = 1;
	static constexpr uint8_t IDX_RIGHT = 2;
	static constexpr char CELL_PROCESSED_MARKER = 127; // decrease by 127 to mark the cell as processed

	QueryInitialRobotPosition::Response doQueryInitialRobotPosition();
	RobotMove::Response doRobotMove(uint8_t robotMoveType);
	void doFieldMapValidate(FieldData & data);
	void printQueryInitialRobotPositionResponse(QueryInitialRobotPosition::Response &result);
	void printRobotMoveResponse(RobotMove::Response &result);
	bool isValidMove_RandomMouse(uint8_t tileForward);
	bool isValidMove_DFS(FieldData &data, const FieldPos &location);
	void setMapCells(FieldData &data, FieldPos &robotPos, uint32_t robotDir, std::array<uint8_t,3> &tileArray);
	void setMapSell(char &cell, uint8_t tileValue);
	void changeRobotDir(uint8_t robotDir, uint8_t newDir);
	void moveToPrevPos(FieldPos &oldPos, const FieldPos &newPos, uint8_t &robotDir);
	uint32_t getRandomDir();

	rclcpp::Client<QueryInitialRobotPosition>::SharedPtr client_query_initial_robot_position;
	rclcpp::Client<RobotMove>::SharedPtr client_robot_move;
	rclcpp::Client<FieldMapValidate>::SharedPtr client_field_map_validate;
	uint32_t m_map_width;
	uint32_t m_map_height;
	int32_t m_robot_pos_x;
	int32_t m_robot_pos_y;
};

#endif /* ROBO_MINER_TASK_SOLVER_H_ */
