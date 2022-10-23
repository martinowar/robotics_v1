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

// TODO include FieldPos from robo_miner_commonv
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
	void mapTraverseAndValidate();

private:
	using FieldData = std::deque<std::deque<char>>;

	static constexpr char TILE_OBSTACLE = 'X';
	static constexpr char TILE_WALL = '#';
	static constexpr uint8_t IDX_LEFT = 0;
	static constexpr uint8_t IDX_FORWARD = 1;
	static constexpr uint8_t IDX_RIGHT = 2;
	static constexpr char CELL_PROCESSED_MARKER = 127; // decrease by 127 to mark the cell as processed

	QueryInitialRobotPosition::Response doQueryInitialRobotPosition();
	RobotMove::Response doRobotMove(const uint8_t robotMoveType);
	void doFieldMapValidate(const FieldData & data);
	bool isValidMove(const FieldData &data, const FieldPos &location);
	void setMapCells(FieldData &data, const FieldPos &robotPos, const uint32_t robotDir, const std::array<uint8_t,3> &tileArray);
	void setMapCell(FieldData &data, const FieldPos &cellPos, const uint8_t tileValue);
	void changeRobotDir(const uint8_t robotDir, const uint8_t newDir);
	FieldPos getPhysicalPos(const FieldPos &logicalPos);
	void moveToPrevPos(FieldPos &oldPos, const FieldPos &newPos, uint8_t &robotDir);

	rclcpp::Client<QueryInitialRobotPosition>::SharedPtr m_clientQueryInitialRobotPosition;
	rclcpp::Client<RobotMove>::SharedPtr m_clientRobotMove;
	rclcpp::Client<FieldMapValidate>::SharedPtr m_clientFieldMapValidate;
	FieldPos m_mapTopLeftPos;
	FieldPos m_mapBottomRightPos;
};

#endif /* ROBO_MINER_TASK_SOLVER_H_ */
