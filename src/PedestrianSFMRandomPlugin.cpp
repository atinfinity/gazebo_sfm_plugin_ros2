/***********************************************************************/
/**                                                                    */
/** PedestrianSFMRandomPlugin.cpp                                       */
/**                                                                    */
/** Copyright (c) 2021, Service Robotics Lab (SRL).                    */
/**                     http://robotics.upo.es                         */
/**                                                                    */
/** All rights reserved.                                               */
/**                                                                    */
/** Authors:                                                           */
/** Noé Pérez-Higueras (maintainer)                                    */
/** email: noeperez@upo.es                                             */
/**                                                                    */
/** This software may be modified and distributed under the terms      */
/** of the BSD license. See the LICENSE file for details.              */
/**                                                                    */
/** http://www.opensource.org/licenses/BSD-3-Clause                    */
/**                                                                    */
/** Ported to Gazebo Harmonic (gz-sim8) / ROS 2 Jazzy.                 */
/**                                                                    */
/***********************************************************************/

#include <cmath>
#include <random>
#include <string>
#include <unordered_map>

#include <gz/plugin/Register.hh>
#include <gz/common/Console.hh>
#include <gz/math/Angle.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>

#include <gz/sim/Util.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Actor.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>

#include "gazebo_sfm_plugin_ros2/PedestrianSFMRandomPlugin.h"

using namespace gazebo_sfm_plugin_ros2;
namespace components = gz::sim::components;

namespace
{
  /// \brief Effective world pose of an entity. Actors driven by this and other
  /// SFM plugins carry their real pose in TrajectoryPose, so prefer it.
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
PedestrianSFMRandomPlugin::PedestrianSFMRandomPlugin() = default;

/////////////////////////////////////////////////
void PedestrianSFMRandomPlugin::Configure(const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &/*_eventMgr*/)
{
  this->actorEntity = _entity;
  this->sdf = _sdf;

  auto actorComp = _ecm.Component<components::Actor>(_entity);
  if (!actorComp)
  {
    gzerr << "PedestrianSFMRandomPlugin: entity [" << _entity
          << "] is not an actor." << std::endl;
    return;
  }

  this->sfmActor.id = static_cast<int>(_entity);

  // Initialize sfmActor pose from the actor's current world pose.
  gz::math::Pose3d pose = gz::sim::worldPose(_entity, _ecm);
  gz::math::Vector3d rpy = pose.Rot().Euler();
  this->sfmActor.position.set(pose.Pos().X(), pose.Pos().Y());
  this->sfmActor.yaw = utils::Angle::fromRadian(rpy.Z());
  this->sfmActor.velocity.set(0.0, 0.0);
  this->sfmActor.linearVelocity = 0.0;
  this->sfmActor.angularVelocity = 0.0;

  // Read in the maximum velocity of the pedestrian
  if (_sdf->HasElement("velocity"))
    this->sfmActor.desiredVelocity = _sdf->Get<double>("velocity");
  else
    this->sfmActor.desiredVelocity = 0.8;

  // Read in the various SFM force weights.
  if (_sdf->HasElement("goal_weight"))
    this->sfmActor.params.forceFactorDesired = _sdf->Get<double>("goal_weight");
  if (_sdf->HasElement("obstacle_weight"))
    this->sfmActor.params.forceFactorObstacle =
        _sdf->Get<double>("obstacle_weight");
  if (_sdf->HasElement("social_weight"))
    this->sfmActor.params.forceFactorSocial =
        _sdf->Get<double>("social_weight");
  if (_sdf->HasElement("group_gaze_weight"))
    this->sfmActor.params.forceFactorGroupGaze =
        _sdf->Get<double>("group_gaze_weight");
  if (_sdf->HasElement("group_coh_weight"))
    this->sfmActor.params.forceFactorGroupCoherence =
        _sdf->Get<double>("group_coh_weight");
  if (_sdf->HasElement("group_rep_weight"))
    this->sfmActor.params.forceFactorGroupRepulsion =
        _sdf->Get<double>("group_rep_weight");

  if (_sdf->HasElement("radius"))
    this->sfmActor.radius = _sdf->Get<double>("radius");

  // Name of the skeleton animation to play (defaults to the Fuel "Walking
  // actor" animation, which is named "walking").
  if (_sdf->HasElement("animation"))
    this->animationName = _sdf->Get<std::string>("animation");

  // Read in the animation factor (applied in PreUpdate).
  if (_sdf->HasElement("animation_factor"))
    this->animationFactor = _sdf->Get<double>("animation_factor");
  else
    this->animationFactor = 4.5;

  if (_sdf->HasElement("people_distance"))
    this->peopleDistance = _sdf->Get<double>("people_distance");
  else
    this->peopleDistance = 5.0;

  // Read in the pedestrians in your walking group
  if (_sdf->HasElement("group"))
  {
    this->sfmActor.groupId = this->sfmActor.id;
    auto modelElem = _sdf->FindElement("group")->FindElement("model");
    while (modelElem)
    {
      this->groupNames.push_back(modelElem->Get<std::string>());
      modelElem = modelElem->GetNextElement("model");
    }
  }
  else
  {
    this->sfmActor.groupId = -1;
  }

  // Read in the other obstacles to ignore
  if (_sdf->HasElement("ignore_obstacles"))
  {
    auto modelElem = _sdf->FindElement("ignore_obstacles")->FindElement("model");
    while (modelElem)
    {
      this->ignoreModels.push_back(modelElem->Get<std::string>());
      modelElem = modelElem->GetNextElement("model");
    }
  }

  // Add our own name to models we should ignore when avoiding obstacles.
  auto nameComp = _ecm.Component<components::Name>(_entity);
  if (nameComp)
    this->ignoreModels.push_back(nameComp->Data());

  this->Reset(_ecm);
}

/////////////////////////////////////////////////
void PedestrianSFMRandomPlugin::Reset(gz::sim::EntityComponentManager &_ecm)
{
  this->lastUpdate = std::chrono::steady_clock::duration{0};

  gz::math::Pose3d actorPose = gz::sim::worldPose(this->actorEntity, _ecm);

  // Read in the goals to reach
  if (this->sdf->HasElement("trajectory"))
  {
    auto cyclicElem =
        this->sdf->FindElement("trajectory")->FindElement("cyclic");
    if (cyclicElem)
      this->sfmActor.cyclicGoals = cyclicElem->Get<bool>();

    auto wpElem = this->sdf->FindElement("trajectory")->FindElement("waypoint");
    while (wpElem)
    {
      gz::math::Vector3d g = wpElem->Get<gz::math::Vector3d>();
      sfm::Goal goal;
      goal.center.set(g.X(), g.Y());
      goal.radius = 0.3;
      this->sfmActor.goals.push_back(goal);
      wpElem = wpElem->GetNextElement("waypoint");
    }
  }
  else if (this->sdf->HasElement("random_trajectory"))
  {
    auto rt = this->sdf->FindElement("random_trajectory");

    int seed;
    auto seedElem = rt->FindElement("seed");
    std::random_device rnd;
    if (seedElem)
      seed = seedElem->Get<int>() * static_cast<int>(this->actorEntity);
    else
      seed = rnd();
    std::mt19937 mt(seed);
    std::uniform_int_distribution<> rand100(0, 99);

    gz::math::Vector3d origin = rt->Get<gz::math::Vector3d>("origin");
    double rx_max = rt->Get<double>("rx");
    double ry_max = rt->Get<double>("ry");

    this->sfmActor.cyclicGoals = true;
    int steps = rt->Get<int>("steps");
    for (int i = 0; i <= steps; i++)
    {
      double rx = rand100(mt) / 100.0 * rx_max;
      double ry = rand100(mt) / 100.0 * ry_max;
      double t = rand100(mt) / 100.0 * 2.0 * M_PI;
      double x = origin.X() + rx * cos(t);
      double y = origin.Y() + ry * sin(t);

      sfm::Goal goal;
      goal.center.set(x, y);
      goal.radius = 0.3;
      this->sfmActor.goals.push_back(goal);
      if (i == 0)
      {
        this->sfmActor.position.set(x, y);
        actorPose.Pos().X(x);
        actorPose.Pos().Y(y);
      }
    }
  }

  // The rendered actor world pose is composed as (Pose * TrajectoryPose).
  // Keep the actor's vertical offset in the Pose component (the mesh root /
  // hips sit ~1 m above the feet) and drive only the 2D world motion through
  // TrajectoryPose, whose Z is therefore zeroed. Setting TrajectoryPose also
  // disables the model's SDF-scripted trajectory.
  auto poseComp = _ecm.Component<components::Pose>(this->actorEntity);
  if (poseComp)
  {
    auto basePose = poseComp->Data();
    basePose.Pos().X(0);
    basePose.Pos().Y(0);
    *poseComp = components::Pose(basePose);
  }

  actorPose.Pos().Z(0);

  auto trajPoseComp =
      _ecm.Component<components::TrajectoryPose>(this->actorEntity);
  if (!trajPoseComp)
    _ecm.CreateComponent(this->actorEntity,
        components::TrajectoryPose(actorPose));
  else
    *trajPoseComp = components::TrajectoryPose(actorPose);

  // Select the walking skeleton animation and enable plugin-driven timing.
  auto animNameComp =
      _ecm.Component<components::AnimationName>(this->actorEntity);
  if (!animNameComp)
    _ecm.CreateComponent(this->actorEntity,
        components::AnimationName(this->animationName));
  else
    *animNameComp = components::AnimationName(this->animationName);
  _ecm.SetChanged(this->actorEntity, components::AnimationName::typeId,
      gz::sim::ComponentState::OneTimeChange);

  auto animTimeComp =
      _ecm.Component<components::AnimationTime>(this->actorEntity);
  if (!animTimeComp)
    _ecm.CreateComponent(this->actorEntity, components::AnimationTime());
}

/////////////////////////////////////////////////
void PedestrianSFMRandomPlugin::HandleObstacles(
    gz::sim::EntityComponentManager &_ecm, const gz::math::Pose3d &_actorPose)
{
  double minDist = 10000.0;
  gz::math::Vector3d closestObs;
  this->sfmActor.obstacles1.clear();

  _ecm.Each<components::Model, components::Name>(
      [&](const gz::sim::Entity &_e, const components::Model *,
          const components::Name *_name) -> bool
      {
        if (std::find(this->ignoreModels.begin(), this->ignoreModels.end(),
                _name->Data()) != this->ignoreModels.end())
          return true;

        gz::math::Vector3d modelPos = gz::sim::worldPose(_e, _ecm).Pos();
        gz::math::Vector3d offset = modelPos - _actorPose.Pos();
        double modelDist = offset.Length();
        if (modelDist < minDist)
        {
          minDist = modelDist;
          closestObs = modelPos;
        }
        return true;
      });

  // Only report an obstacle if a real one was found; otherwise leave the list
  // empty so no phantom obstacle force is applied.
  if (minDist < 9999.0)
  {
    utils::Vector2d ob(closestObs.X(), closestObs.Y());
    this->sfmActor.obstacles1.push_back(ob);
  }
}

/////////////////////////////////////////////////
void PedestrianSFMRandomPlugin::HandlePedestrians(
    gz::sim::EntityComponentManager &_ecm, const gz::math::Pose3d &_actorPose,
    double _dt)
{
  this->otherActors.clear();

  _ecm.Each<components::Actor, components::Name>(
      [&](const gz::sim::Entity &_e, const components::Actor *,
          const components::Name *_name) -> bool
      {
        if (_e == this->actorEntity)
          return true;

        gz::math::Pose3d modelPose = EffectivePose(_e, _ecm);
        gz::math::Vector3d diff = modelPose.Pos() - _actorPose.Pos();
        if (diff.Length() >= this->peopleDistance)
          return true;

        sfm::Agent ped;
        ped.id = static_cast<int>(_e);
        ped.position.set(modelPose.Pos().X(), modelPose.Pos().Y());
        ped.yaw = utils::Angle::fromRadian(modelPose.Rot().Euler().Z());
        ped.radius = this->sfmActor.radius;

        // Actors are kinematic, so estimate velocity from pose deltas.
        gz::math::Vector3d vel(0, 0, 0);
        auto it = this->lastPoses.find(_e);
        if (it != this->lastPoses.end() && _dt > 1e-6)
          vel = (modelPose.Pos() - it->second.Pos()) / _dt;
        ped.velocity.set(vel.X(), vel.Y());
        ped.linearVelocity = vel.Length();
        ped.angularVelocity = 0.0;

        if (this->sfmActor.groupId != -1)
        {
          auto git = std::find(this->groupNames.begin(),
              this->groupNames.end(), _name->Data());
          ped.groupId = (git != this->groupNames.end())
              ? this->sfmActor.groupId : -1;
        }
        this->otherActors.push_back(ped);
        this->lastPoses[_e] = modelPose;
        return true;
      });
}

/////////////////////////////////////////////////
void PedestrianSFMRandomPlugin::PreUpdate(const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused || this->actorEntity == gz::sim::kNullEntity)
    return;

  std::chrono::duration<double> dtDuration = _info.simTime - this->lastUpdate;
  double dt = dtDuration.count();
  this->lastUpdate = _info.simTime;
  if (dt <= 0.0)
    return;

  auto trajPoseComp =
      _ecm.Component<components::TrajectoryPose>(this->actorEntity);
  if (!trajPoseComp)
    return;
  gz::math::Pose3d actorPose = trajPoseComp->Data();
  const double startZ = actorPose.Pos().Z();
  const gz::math::Vector3d startPos = actorPose.Pos();

  // Update closest obstacle and nearby pedestrians.
  HandleObstacles(_ecm, actorPose);
  HandlePedestrians(_ecm, actorPose, dt);

  // Compute Social Forces and update the SFM agent state.
  sfm::SFM.computeForces(this->sfmActor, this->otherActors);
  sfm::SFM.updatePosition(this->sfmActor, dt);

  // Smoothly rotate towards the SFM heading.
  double targetYaw = this->sfmActor.yaw.toRadian();
  gz::math::Angle current(actorPose.Rot().Euler().Z());
  double diff = gz::math::Angle(targetYaw - current.Radian()).Normalized().Radian();
  double yaw = targetYaw;
  if (std::fabs(diff) > GZ_DTOR(10))
    yaw = current.Radian() + diff * 0.005;

  actorPose.Pos().X(this->sfmActor.position.getX());
  actorPose.Pos().Y(this->sfmActor.position.getY());
  actorPose.Pos().Z(startZ);
  actorPose.Rot() = gz::math::Quaterniond(0, 0, yaw);

  double distanceTraveled = (actorPose.Pos() - startPos).Length();

  // Update actor world pose through the trajectory pose component.
  *trajPoseComp = components::TrajectoryPose(actorPose);
  _ecm.SetChanged(this->actorEntity, components::TrajectoryPose::typeId,
      gz::sim::ComponentState::OneTimeChange);

  // Advance the walking animation in proportion to the distance traveled.
  auto animTimeComp =
      _ecm.Component<components::AnimationTime>(this->actorEntity);
  if (animTimeComp)
  {
    auto animTime = animTimeComp->Data() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(
                distanceTraveled * this->animationFactor));
    *animTimeComp = components::AnimationTime(animTime);
    _ecm.SetChanged(this->actorEntity, components::AnimationTime::typeId,
        gz::sim::ComponentState::OneTimeChange);
  }
}

GZ_ADD_PLUGIN(
    gazebo_sfm_plugin_ros2::PedestrianSFMRandomPlugin,
    gz::sim::System,
    gazebo_sfm_plugin_ros2::PedestrianSFMRandomPlugin::ISystemConfigure,
    gazebo_sfm_plugin_ros2::PedestrianSFMRandomPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    gazebo_sfm_plugin_ros2::PedestrianSFMRandomPlugin,
    "gazebo_sfm_plugin_ros2::PedestrianSFMRandomPlugin")
