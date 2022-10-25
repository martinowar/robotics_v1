#ifndef ROBO_MINER_GUI_FLOODFILL_H_
#define ROBO_MINER_GUI_FLOODFILL_H_

//System headers
#include <cstdint>
#include <vector>
#include <deque>

//Other libraries headers
#include "robo_miner_task_solver/FieldPos.h"

//Own components headers

//Forward declarations


using FieldData = std::deque<std::deque<char>>;

class FloodFill {
public:
  FloodFill() = delete;

  static std::vector<FieldPos> findLongestCrystalSequence(
      const FieldData &data, const std::vector<char> &nonCrystalMarkers);

  static std::vector<FieldPos> findLocalCrystalSequence(
      const FieldData &data, const std::vector<char> &nonCrystalMarkers,
      const FieldPos &fieldPos);
};

#endif /* ROBO_MINER_GUI_FLOODFILL_H_ */
