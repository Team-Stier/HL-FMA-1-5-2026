#pragma once

#include <string>

#include "path_planner/reference_path.hpp"
#include "path_planner/types.hpp"

namespace path_planner {

class PlannerBackend {
 public:
  virtual ~PlannerBackend() = default;
  virtual std::string name() const = 0;
  virtual void reset() = 0;
  virtual PlannerResult plan(const ReferencePath& reference,
                             const PlannerInput& input) = 0;
};

}  // namespace path_planner
