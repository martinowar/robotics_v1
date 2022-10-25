#include "robo_miner_task_solver/FieldPos.h"

// TODO include FieldPos from robo_miner_common
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
