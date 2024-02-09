// Copyright 2024 DeepMind Technologies Limited
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

#include <cstring>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>
#include "user/user_api.h"
#include "user/user_util.h"


// default body attributes
void mjm_defaultBody(mjmBody& body) {
  memset(&body, 0, sizeof(mjmBody));

  // set non-zero defaults
  body.fullinertia[0] = mjNAN;
  body.explicitinertial = 0;
  body.mocap = 0;
  body.quat[0] = 1;
  body.iquat[0] = 1;
  body.pos[0] = mjNAN;
  body.ipos[0] = mjNAN;
  body.alt.axisangle[0] = body.alt.xyaxes[0] = body.alt.zaxis[0] = body.alt.euler[0] = mjNAN;
  body.ialt.axisangle[0] = body.ialt.xyaxes[0] = body.ialt.zaxis[0] = body.ialt.euler[0] = mjNAN;
  body.plugin.active = false;
  body.plugin.instance = nullptr;
}


// default geom attributes
MJAPI void mjm_defaultGeom(mjmGeom& geom) {
  memset(&geom, 0, sizeof(mjmGeom));

  // set non-zero defaults
  geom.fromto[0] = mjNAN;
  geom.mass = mjNAN;
  geom.type = mjGEOM_SPHERE;
  geom.contype = 1;
  geom.conaffinity = 1;
  geom.condim = 3;
  geom.friction[0] = 1;
  geom.friction[1] = 0.005;
  geom.friction[2] = 0.0001;
  geom.solmix = 1.0;
  mj_defaultSolRefImp(geom.solref, geom.solimp);
  geom.density = 1000;             // water density (1000 kg / m^3)
  geom.fitscale = 1;
  geom.rgba[0] = geom.rgba[1] = geom.rgba[2] = 0.5f;
  geom.rgba[3] = 1.0f;
  geom.typeinertia = mjINERTIA_VOLUME;
  geom.quat[0] = 1;
  geom.alt.axisangle[0] = geom.alt.xyaxes[0] = geom.alt.zaxis[0] = geom.alt.euler[0] = mjNAN;
  geom.fluid_coefs[0] = 0.5;       // blunt drag coefficient
  geom.fluid_coefs[1] = 0.25;      // slender drag coefficient
  geom.fluid_coefs[2] = 1.5;       // angular drag coefficient
  geom.fluid_coefs[3] = 1.0;       // kutta lift coefficient
  geom.fluid_coefs[4] = 1.0;       // magnus lift coefficient
}



// default site attributes
void mjm_defaultSite(mjmSite& site) {
  memset(&site, 0, sizeof(mjmSite));

  // set non-zero defaults
  site.type = mjGEOM_SPHERE;
  site.quat[0] = 1;
  site.size[0] = site.size[1] = site.size[2] = 0.005;
  site.fromto[0] = mjNAN;
  site.alt.axisangle[0] = site.alt.xyaxes[0] = site.alt.zaxis[0] = site.alt.euler[0] = mjNAN;
  site.rgba[0] = site.rgba[1] = site.rgba[2] = 0.5f;
  site.rgba[3] = 1.0f;
}