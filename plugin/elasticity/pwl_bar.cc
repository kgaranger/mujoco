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

#include <algorithm>
#include <cstddef>
#include <optional>
#include <sstream>

#include "pwl_bar.h"
#include <mujoco/mjplugin.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mujoco.h>

namespace mujoco::plugin::elasticity {
namespace {

// Jet color palette
void scalar2rgba(float rgba[4], mjtNum v, mjtNum rv, mjtNum vmin, mjtNum vmax) {
  v = v < vmin ? vmin : v;
  v = v > vmax ? vmax : v;

  if (v < rv) {
    mjtNum dv = (v - vmin) / (rv - vmin);
    rgba[0] = 0;
    rgba[1] = 0.1;
    rgba[2] = 0.5 + 05 * dv;
  } else {
    mjtNum dv = (v - rv) / (vmax - rv);
    rgba[0] = 0.5 + 0.5 * dv;
    rgba[1] = 0.1;
    rgba[2] = 0;
  }
}

// reads numeric attributes
bool CheckAttr(const char *name, const mjModel *m, int instance) {
  char *end;
  std::string value = mj_getPluginConfig(m, instance, name);
  value.erase(std::remove_if(value.begin(), value.end(), isspace), value.end());
  strtod(value.c_str(), &end);
  return end == value.data() + value.size();
}

} // namespace

// factory function
std::optional<PWLBar> PWLBar::Create(const mjModel *m, mjData *d,
                                     int instance) {
  if (CheckAttr("pre_buckling_stiffness", m, instance) &&
          CheckAttr("post_buckling_stiffness", m, instance),
      CheckAttr("buckling_displacement", m, instance)) {
    return PWLBar(m, d, instance);
  } else {
    mju_warning("Invalid parameter specification in pwl_bar plugin");
    return std::nullopt;
  }
}

// plugin constructor
PWLBar::PWLBar(const mjModel *m, mjData *d, int instance) {
  // parameters were validated by the factor function
  K1 = strtod(mj_getPluginConfig(m, instance, "pre_buckling_stiffness"),
              nullptr);
  K2 = strtod(mj_getPluginConfig(m, instance, "post_buckling_stiffness"),
              nullptr);
  rl = strtod(mj_getPluginConfig(m, instance, "rest_length"), nullptr);
  bd =
      strtod(mj_getPluginConfig(m, instance, "buckling_displacement"), nullptr);
  const char *joint_name = mj_getPluginConfig(m, instance, "joint_name");

  jid = mj_name2id(m, mjOBJ_JOINT, joint_name);
  if (jid < 0) {
    mju_error("Joint %s not found", joint_name);
  }

  if (m->jnt_type[jid] != mjJNT_SLIDE) {
    mju_error("Joint %s is not a slider", joint_name);
  }

  jnt_pos_adr = m->jnt_qposadr[jid];
  jnt_dof_adr = m->jnt_dofadr[jid];

  bf = bd * K1;
}

void PWLBar::Compute(const mjModel *m, mjData *d, int instance) {
  disp = rl - d->qpos[jnt_pos_adr];
  if (disp > bd) {
    frc = bf + K2 * (disp - bd);
  } else {
    frc = K1 * disp;
  }
  d->qfrc_passive[jnt_dof_adr] += frc;
}

void PWLBar::Visualize(const mjModel *m, mjData *d, mjvScene *scn,
                       int instance) {
  scalar2rgba(m->geom_rgba + 4 * m->body_geomadr[m->jnt_bodyid[jid]], disp, bd,
              0, 0.5 * rl);
}

void PWLBar::RegisterPlugin() {
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "mujoco.elasticity.pwl_bar";
  plugin.capabilityflags |= mjPLUGIN_PASSIVE;

  const char *attributes[] = {"pre_buckling_stiffness",
                              "post_buckling_stiffness", "rest_length",
                              "buckling_displacement", "joint_name"};
  plugin.nattribute = sizeof(attributes) / sizeof(attributes[0]);
  plugin.attributes = attributes;
  plugin.nstate = +[](const mjModel *m, int instance) { return 0; };

  plugin.init = +[](const mjModel *m, mjData *d, int instance) {
    auto elasticity_or_null = PWLBar::Create(m, d, instance);
    if (!elasticity_or_null.has_value()) {
      return -1;
    }
    d->plugin_data[instance] =
        reinterpret_cast<uintptr_t>(new PWLBar(std::move(*elasticity_or_null)));
    return 0;
  };
  plugin.destroy = +[](mjData *d, int instance) {
    delete reinterpret_cast<PWLBar *>(d->plugin_data[instance]);
    d->plugin_data[instance] = 0;
  };
  plugin.compute =
      +[](const mjModel *m, mjData *d, int instance, int capability_bit) {
        auto *elasticity = reinterpret_cast<PWLBar *>(d->plugin_data[instance]);
        elasticity->Compute(m, d, instance);
      };
  plugin.visualize = +[](const mjModel *m, mjData *d, const mjvOption *opt,
                         mjvScene *scn, int instance) {
    auto *elasticity = reinterpret_cast<PWLBar *>(d->plugin_data[instance]);
    elasticity->Visualize(m, d, scn, instance);
  };

  mjp_registerPlugin(&plugin);
}

} // namespace mujoco::plugin::elasticity
