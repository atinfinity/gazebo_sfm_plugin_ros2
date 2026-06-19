/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 * Copyright (C) 2026 atinfinity
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <gz/plugin/Register.hh>
#include <gz/common/Console.hh>

#include <gz/sim/Util.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/PoseCmd.hh>

#include "AttachModelPlugin.hh"

using namespace servicesim;
namespace components = gz::sim::components;

namespace
{
  /// \brief Effective world pose of an entity, preferring TrajectoryPose for
  /// actors driven by other plugins.
  gz::math::Pose3d EffectivePose(const gz::sim::Entity &_entity,
      gz::sim::EntityComponentManager &_ecm)
  {
    auto trajComp = _ecm.Component<components::TrajectoryPose>(_entity);
    if (trajComp)
      return trajComp->Data();
    return gz::sim::worldPose(_entity, _ecm);
  }
}  // namespace

/////////////////////////////////////////////////
AttachModelPlugin::AttachModelPlugin() = default;

/////////////////////////////////////////////////
void AttachModelPlugin::Configure(const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &/*_eventMgr*/)
{
  this->parentEntity = _entity;

  if (!_sdf->HasElement("link"))
  {
    gzerr << "AttachModelPlugin: no <link> sdf elements found." << std::endl;
    return;
  }

  auto linkElem = _sdf->FindElement("link");
  while (linkElem)
  {
    if (linkElem->HasElement("model"))
    {
      auto modelElem = linkElem->FindElement("model");
      while (modelElem)
      {
        auto modelName = modelElem->Get<std::string>("model_name");
        auto modelEntity = _ecm.EntityByComponents(components::Name(modelName));
        if (modelEntity == gz::sim::kNullEntity)
        {
          gzerr << "AttachModelPlugin: model [" << modelName
                << "] not found, make sure it is loaded before the parent."
                << std::endl;
        }
        else
        {
          gz::math::Pose3d pose;
          if (modelElem->HasElement("pose"))
            pose = modelElem->Get<gz::math::Pose3d>("pose");
          this->attachedModels.emplace_back(modelEntity, pose);
        }
        modelElem = modelElem->GetNextElement("model");
      }
    }
    linkElem = linkElem->GetNextElement("link");
  }
}

/////////////////////////////////////////////////
void AttachModelPlugin::PreUpdate(const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused || this->attachedModels.empty())
    return;

  gz::math::Pose3d parentPose = EffectivePose(this->parentEntity, _ecm);

  for (const auto &[modelEntity, offset] : this->attachedModels)
  {
    gz::math::Pose3d target = parentPose * offset;

    // Command the physics engine to teleport the model so that it stays a
    // physical (LiDAR-detectable) obstacle while following the actor.
    auto poseCmd = _ecm.Component<components::WorldPoseCmd>(modelEntity);
    if (!poseCmd)
      _ecm.CreateComponent(modelEntity, components::WorldPoseCmd(target));
    else
      *poseCmd = components::WorldPoseCmd(target);
  }
}

GZ_ADD_PLUGIN(
    servicesim::AttachModelPlugin,
    gz::sim::System,
    servicesim::AttachModelPlugin::ISystemConfigure,
    servicesim::AttachModelPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    servicesim::AttachModelPlugin,
    "servicesim::AttachModelPlugin")
