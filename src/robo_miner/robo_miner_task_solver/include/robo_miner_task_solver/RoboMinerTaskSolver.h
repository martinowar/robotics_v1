#ifndef ROBO_MINER_TASK_SOLVER_H_
#define ROBO_MINER_TASK_SOLVER_H_

#include "robo_miner_interfaces/srv/query_initial_robot_position.hpp"
#include "robo_miner_interfaces/srv/field_map_validate.hpp"
#include "robo_miner_interfaces/srv/longest_sequence_validate.hpp"
#include "robo_miner_interfaces/srv/activate_mining_validate.hpp"
#include "robo_miner_interfaces/srv/robot_move.hpp"
#include "robo_miner_interfaces/msg/robot_move_type.hpp"
#include "robo_miner_interfaces/msg/field_point.hpp"
#include "robo_miner_interfaces/msg/robot_position_response.hpp"
#include "robo_miner_task_solver/FieldPos.h"
#include "robo_miner_task_solver/FloodFill.h"

using QueryInitialRobotPosition = robo_miner_interfaces::srv::QueryInitialRobotPosition;
using FieldMapValidate = robo_miner_interfaces::srv::FieldMapValidate;
using LongestSequenceValidate = robo_miner_interfaces::srv::LongestSequenceValidate;
using ActivateMiningValidate = robo_miner_interfaces::srv::ActivateMiningValidate;
using RobotMove = robo_miner_interfaces::srv::RobotMove;
using RobotMoveType = robo_miner_interfaces::msg::RobotMoveType;
using RobotPositionResponse = robo_miner_interfaces::msg::RobotPositionResponse;
using FieldPoint = robo_miner_interfaces::msg::FieldPoint;

//System headers
#include <cstdint>
#include <deque>
#include <stack>

class RoboMinerTaskSolver : public rclcpp::Node
{
public:
	RoboMinerTaskSolver();
	void init();
	void mapTraverseAndValidate();

private:
	static constexpr char TILE_OBSTACLE = 'X';
	static constexpr char TILE_WALL = '#';
	static constexpr uint8_t IDX_LEFT = 0;
	static constexpr uint8_t IDX_FORWARD = 1;
	static constexpr uint8_t IDX_RIGHT = 2;
	static constexpr uint8_t WALL_CELL = 1;
	static constexpr char CELL_PROCESSED_MARKER = 127; // decrease by 127 to mark the cell as processed

	QueryInitialRobotPosition::Response doQueryInitialRobotPosition();
	RobotMove::Response doRobotMove(const uint8_t robotMoveType);
	void doFieldMapValidate(FieldData & data);
	void doLongestSequenceValidate(std::vector<FieldPos> &longestSequence);
	void doActivateMiningValidate();
	bool isValidMove(const FieldData &data, const FieldPos &location) const;
	bool isValidMoveWhenGoToCell(const FieldData &data, const FieldPos &location) const;
	bool isValidMoveWhileMining(const FieldData &data, const FieldPos &location, const int8_t crystalValue) const;
	void setAdjacentCells(FieldData &data, const FieldPos &robotPos, const uint32_t robotDir, const std::array<uint8_t,3> &tileArray);
	void setMapCell(FieldData &data, const FieldPos &cellPos, const uint8_t tileValue);
	void changeRobotDir(const uint8_t robotDir, const uint8_t newDir);
	FieldPos getPhysicalPos(const FieldPos &logicalPos) const;
	void moveToPrevPos(FieldPos &oldPos, const FieldPos &newPos, uint8_t &robotDir);
	void moveToCell(FieldData &data, FieldPos &robotPos, const FieldPos &cellPos, uint8_t &robotDir);
	void mineLongestSequence(FieldData &data, FieldPos &robotPos, uint8_t &robotDir);

	rclcpp::Client<QueryInitialRobotPosition>::SharedPtr m_clientQueryInitialRobotPosition;
	rclcpp::Client<RobotMove>::SharedPtr m_clientRobotMove;
	rclcpp::Client<FieldMapValidate>::SharedPtr m_clientFieldMapValidate;
	rclcpp::Client<LongestSequenceValidate>::SharedPtr m_clientLongestSequenceValidate;
	rclcpp::Client<ActivateMiningValidate>::SharedPtr m_clientActivateMiningValidate;
	FieldPos m_mapTopLeftPos;
	FieldPos m_mapBottomRightPos;
};

#endif /* ROBO_MINER_TASK_SOLVER_H_ */
