#pragma once

#include <string>

#include "lidar_path_planning/reference_path.hpp"
#include "lidar_path_planning/types.hpp"

namespace lidar_path_planning {

class PlannerBackend {
 public:
  virtual ~PlannerBackend() = default;
  virtual std::string name() const = 0;
  virtual void reset() = 0;
  virtual PlannerResult plan(const ReferencePath& reference,
                             const PlannerInput& input) = 0;
};

}  // namespace lidar_path_planning
