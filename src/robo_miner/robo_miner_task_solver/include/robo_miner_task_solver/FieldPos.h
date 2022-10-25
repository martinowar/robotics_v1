#ifndef FIELD_POS_H_
#define FIELD_POS_H_

#include <cstdint>

// TODO include FieldPos from robo_miner_commonv
struct FieldPos {
  FieldPos() = default;
  FieldPos(int32_t row, int32_t col);

  bool operator==(const FieldPos& other) const;
  bool operator<(const FieldPos& other) const;

  int32_t row { 0 };
  int32_t col { 0 };
};

#endif // FIELD_POS_H_
