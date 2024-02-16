// Copyright 2022 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MUJOCO_SRC_PLUGIN_ELASTICITY_PWL_BAR_H_
#define MUJOCO_SRC_PLUGIN_ELASTICITY_PWL_BAR_H_

#include <optional>
#include <vector>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>

namespace mujoco::plugin::elasticity {

class PWLBar {
public:
  // Creates a new PWLBar instance (allocated with `new`) or
  // returns null on failure.
  static std::optional<PWLBar> Create(const mjModel *m, mjData *d,
                                      int instance);
  PWLBar(PWLBar &&) = default;
  ~PWLBar() = default;

  void Compute(const mjModel *m, mjData *d, int instance);
  void Visualize(const mjModel *m, mjData *d, mjvScene *scn, int instance);

  static void RegisterPlugin();

  int jid;         // joint id
  int jnt_pos_adr; // joint address
  int jnt_dof_adr; // joint dof address
  mjtNum K1;       // pre-buckling stiffness
  mjtNum K2;       // post-buckling stiffness
  mjtNum rl;       // rest length
  mjtNum bd;       // buckling displacement
  mjtNum bf;       // buckling force
  mjtNum disp;     // displacement
  mjtNum frc;      // force

private:
  PWLBar(const mjModel *m, mjData *d, int instance);
};

} // namespace mujoco::plugin::elasticity

#endif // MUJOCO_SRC_PLUGIN_ELASTICITY_PWL_BAR_H_
