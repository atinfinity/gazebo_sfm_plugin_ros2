/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#ifndef SERVICESIM_ATTACHMODELPLUGIN_HH_
#define SERVICESIM_ATTACHMODELPLUGIN_HH_

#include <memory>
#include <vector>

#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/math/Pose3.hh>

namespace servicesim
{
  /// \brief A gz-sim System that makes other models in the world follow the
  /// pose of the actor (or model) this plugin is attached to. Used to drag a
  /// collision body along with a visual-only actor so it can be sensed by
  /// LiDAR.
  ///
  /// SDF description (kept compatible with the original plugin):
  /// <link>            More than one <link> can be defined.
  ///   <link_name>     Name of the link in this model (informational).
  ///   <model>         More than one <model> can be attached.
  ///     <pose>        Offset pose of the model in the parent frame.
  ///     <model_name>  Name of the other model in the world to drag along.
  class AttachModelPlugin
      : public gz::sim::System,
        public gz::sim::ISystemConfigure,
        public gz::sim::ISystemPreUpdate
  {
    /// \brief Constructor
    public: AttachModelPlugin();

    /// \brief Destructor
    public: ~AttachModelPlugin() override = default;

    // Documentation inherited
    public: void Configure(const gz::sim::Entity &_entity,
                const std::shared_ptr<const sdf::Element> &_sdf,
                gz::sim::EntityComponentManager &_ecm,
                gz::sim::EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;

    /// \brief Entity this plugin is attached to (the actor/model to follow).
    private: gz::sim::Entity parentEntity{gz::sim::kNullEntity};

    /// \brief Attached model entities and their pose offset in the parent
    /// frame.
    private: std::vector<std::pair<gz::sim::Entity, gz::math::Pose3d>>
        attachedModels;
  };
}  // namespace servicesim
#endif
