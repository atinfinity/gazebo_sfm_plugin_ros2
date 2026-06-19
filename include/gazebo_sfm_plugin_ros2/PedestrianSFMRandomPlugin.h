/***********************************************************************/
/**                                                                    */
/** PedestrianSFMRandomPlugin.h                                        */
/**                                                                    */
/** Copyright (c) 2021, Service Robotics Lab (SRL).                    */
/**                     http://robotics.upo.es                         */
/**                                                                    */
/** All rights reserved.                                               */
/**                                                                    */
/** Authors:                                                           */
/** Noé Pérez-Higueras (original maintainer)                           */
/** email: noeperez@upo.es                                             */
/**                                                                    */
/** Copyright (c) 2026, atinfinity                                     */
/**                                                                    */
/** All rights reserved.                                               */
/**                                                                    */
/** Authors:                                                           */
/** atinfinity (maintainer)                                            */
/** email: dandelion1124@gmail.com                                     */
/**                                                                    */
/** This software may be modified and distributed under the terms      */
/** of the BSD license. See the LICENSE file for details.              */
/**                                                                    */
/** http://www.opensource.org/licenses/BSD-3-Clause                    */
/**                                                                    */
/** Ported to Gazebo Harmonic (gz-sim8) / ROS 2 Jazzy.                 */
/**                                                                    */
/***********************************************************************/

#ifndef GAZEBO_SFM_PLUGIN_ROS2_PEDESTRIANSFMRANDOMPLUGIN_H_
#define GAZEBO_SFM_PLUGIN_ROS2_PEDESTRIANSFMRANDOMPLUGIN_H_

// C++
#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <algorithm>

// gz-sim
#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/math/Pose3.hh>
#include <sdf/Element.hh>

// Social Force Model
#include "lightsfm/sfm.hpp"

namespace gazebo_sfm_plugin_ros2
{
  /// \brief Drives a gz-sim actor along Social-Force-Model trajectories.
  ///
  /// This is the Gazebo Harmonic (gz-sim8) port of the original Gazebo Classic
  /// ModelPlugin. The actor's world pose is driven through the
  /// components::TrajectoryPose component and its walking animation through
  /// components::AnimationTime / components::AnimationName.
  class PedestrianSFMRandomPlugin
      : public gz::sim::System,
        public gz::sim::ISystemConfigure,
        public gz::sim::ISystemPreUpdate
  {
    /// \brief Constructor
    public: PedestrianSFMRandomPlugin();

    /// \brief Destructor
    public: ~PedestrianSFMRandomPlugin() override = default;

    // Documentation inherited
    public: void Configure(const gz::sim::Entity &_entity,
                const std::shared_ptr<const sdf::Element> &_sdf,
                gz::sim::EntityComponentManager &_ecm,
                gz::sim::EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;

    /// \brief Initialize SFM goals and animation (called from Configure).
    private: void Reset(gz::sim::EntityComponentManager &_ecm);

    /// \brief Helper function to detect the closest obstacles.
    private: void HandleObstacles(gz::sim::EntityComponentManager &_ecm,
                 const gz::math::Pose3d &_actorPose);

    /// \brief Helper function to detect the nearby pedestrians (other actors).
    private: void HandlePedestrians(gz::sim::EntityComponentManager &_ecm,
                 const gz::math::Pose3d &_actorPose, double _dt);

    /// \brief Entity of the controlled actor.
    private: gz::sim::Entity actorEntity{gz::sim::kNullEntity};

    /// \brief this actor as a SFM agent.
    private: sfm::Agent sfmActor;

    /// \brief names of the other models in my walking group.
    private: std::vector<std::string> groupNames;

    /// \brief vector of pedestrians detected.
    private: std::vector<sfm::Agent> otherActors;

    /// \brief Maximum distance to detect nearby pedestrians.
    private: double peopleDistance{5.0};

    /// \brief Pointer to the sdf element.
    private: std::shared_ptr<const sdf::Element> sdf;

    /// \brief Time scaling factor. Used to coordinate translational motion
    /// with the actor's walking animation.
    private: double animationFactor{1.0};

    /// \brief Name of the skeleton animation to play.
    private: std::string animationName{"walking"};

    /// \brief Time of the last update.
    private: std::chrono::steady_clock::duration lastUpdate{0};

    /// \brief List of models to ignore as obstacles.
    private: std::vector<std::string> ignoreModels;

    /// \brief Last known world poses of other actors, to estimate velocity.
    private: std::unordered_map<gz::sim::Entity, gz::math::Pose3d> lastPoses;
  };
}  // namespace gazebo_sfm_plugin_ros2
#endif
