/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <string>

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include <pinocchio/multibody/joint/joint-composite.hpp>
#include <pinocchio/multibody/model.hpp>

#include "ocs2_mobile_manipulator/MobileManipulatorInterface.h"

#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/LoadStdVectorOfPair.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_core/soft_constraint/StateInputSoftBoxConstraint.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>
#include <ocs2_pinocchio_interface/urdf.h>
#include <ocs2_self_collision/SelfCollisionConstraint.h>
#include <ocs2_self_collision/SelfCollisionConstraintCppAd.h>

#include "ocs2_mobile_manipulator/ManipulatorModelInfo.h"
#include "ocs2_mobile_manipulator/MobileManipulatorPreComputation.h"
#include "ocs2_mobile_manipulator/constraint/EndEffectorConstraint.h"
#include "ocs2_mobile_manipulator/constraint/NoSlipConstraintCppAd.h"
#include "ocs2_mobile_manipulator/constraint/MobileManipulatorSelfCollisionConstraint.h"
#include "ocs2_mobile_manipulator/cost/QuadraticInputCost.h"
#include "ocs2_mobile_manipulator/dynamics/DefaultManipulatorDynamics.h"
#include "ocs2_mobile_manipulator/dynamics/WheelBasedMobileManipulatorV1Dynamics.h"
#include "ocs2_mobile_manipulator/dynamics/WheelBasedMobileManipulatorV2Dynamics.h"

// Boost
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

namespace ocs2 {
namespace mobile_manipulator {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MobileManipulatorInterface::MobileManipulatorInterface(const std::string& taskFile, const std::string& libraryFolder,
                                                       const std::string& urdfFile) {
  // check that task file exists
  boost::filesystem::path taskFilePath(taskFile);
  if (boost::filesystem::exists(taskFilePath)) {
    std::cerr << "[MobileManipulatorInterface] Loading task file: " << taskFilePath << std::endl;
  } else {
    throw std::invalid_argument("[MobileManipulatorInterface] Task file not found: " + taskFilePath.string());
  }
  // check that urdf file exists
  boost::filesystem::path urdfFilePath(urdfFile);
  if (boost::filesystem::exists(urdfFilePath)) {
    std::cerr << "[MobileManipulatorInterface] Loading Pinocchio model from: " << urdfFilePath << std::endl;
  } else {
    throw std::invalid_argument("[MobileManipulatorInterface] URDF file not found: " + urdfFilePath.string());
  }
  // create library folder if it does not exist
  boost::filesystem::path libraryFolderPath(libraryFolder);
  boost::filesystem::create_directories(libraryFolderPath);
  std::cerr << "[MobileManipulatorInterface] Generated library path: " << libraryFolderPath << std::endl;

  // read the task file
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  // resolve meta-information about the model
  // read manipulator type
  ManipulatorModelType modelType = mobile_manipulator::loadManipulatorType(taskFile, "model_information.manipulatorModelType");
  // read the joints to make fixed
  std::vector<std::string> removeJointNames;
  loadData::loadStdVector<std::string>(taskFile, "model_information.removeJoints", removeJointNames, false);
  // read the frame names
  std::string baseFrame, eeFrame;
  loadData::loadPtreeValue<std::string>(pt, baseFrame, "model_information.baseFrame", false);
  loadData::loadPtreeValue<std::string>(pt, eeFrame, "model_information.eeFrame", false);

  std::cerr << "\n #### Model Information:";
  std::cerr << "\n #### =============================================================================\n";
  std::cerr << "\n #### model_information.manipulatorModelType: " << static_cast<int>(modelType);
  std::cerr << "\n #### model_information.removeJoints: ";
  for (const auto& name : removeJointNames) {
    std::cerr << "\"" << name << "\" ";
  }
  std::cerr << "\n #### model_information.baseFrame: \"" << baseFrame << "\"";
  std::cerr << "\n #### model_information.eeFrame: \"" << eeFrame << "\"" << std::endl;
  std::cerr << " #### =============================================================================" << std::endl;

  // create pinocchio interface
  pinocchioInterfacePtr_.reset(new PinocchioInterface(createPinocchioInterface(urdfFile, modelType, removeJointNames)));
  std::cerr << *pinocchioInterfacePtr_;

  // ManipulatorModelInfo
  manipulatorModelInfo_ = mobile_manipulator::createManipulatorModelInfo(*pinocchioInterfacePtr_, modelType, baseFrame, eeFrame);

  bool usePreComputation = true;
  bool recompileLibraries = true;
  std::cerr << "\n #### Model Settings:";
  std::cerr << "\n #### =============================================================================\n";
  loadData::loadPtreeValue(pt, usePreComputation, "model_settings.usePreComputation", true);
  loadData::loadPtreeValue(pt, recompileLibraries, "model_settings.recompileLibraries", true);
  std::cerr << " #### =============================================================================\n";

  // Default initial state
  initialState_.setZero(manipulatorModelInfo_.stateDim);
  const int baseStateDim = manipulatorModelInfo_.stateDim - manipulatorModelInfo_.armDim;
  const int armStateDim = manipulatorModelInfo_.armDim;
  std::cout << "StateDim" << manipulatorModelInfo_.stateDim << std::endl;
  std::cout << "ArmDim" << manipulatorModelInfo_.armDim << std::endl;
  std::cout << "DOF" << manipulatorModelInfo_.dofNames.size() << std::endl;

  // arm base DOFs initial state
  if (baseStateDim > 0) {
    vector_t initialBaseState = vector_t::Zero(baseStateDim);
    loadData::loadEigenMatrix(taskFile, "initialState.base." + modelTypeEnumToString(modelType), initialBaseState);
    initialState_.head(baseStateDim) = initialBaseState;
  }

  // arm joints DOFs velocity limits
  vector_t initialArmState = vector_t::Zero(armStateDim);
  loadData::loadEigenMatrix(taskFile, "initialState.arm", initialArmState);
  initialState_.tail(armStateDim) = initialArmState;

  std::cerr << "Initial State:   " << initialState_.transpose() << std::endl;

  // DDP-MPC settings
  ddpSettings_ = ddp::loadSettings(taskFile, "ddp");
  mpcSettings_ = mpc::loadSettings(taskFile, "mpc");
  sqpSettings_ = sqp::loadSettings(taskFile, "sqp");
  slpSettings_ = slp::loadSettings(taskFile, "slp");

  // Reference Manager
  referenceManagerPtr_.reset(new ReferenceManager);

  /*
   * Optimal control problem
   */
  // Cost
  problem_.costPtr->add("inputCost", getQuadraticInputCost(taskFile));
  // Constraints
  // no slip constraint
  if (baseStateDim == 11) {
    std::cout << "No Slip constraint enforced!!" << std::endl;
    problem_.softConstraintPtr->add("noSlipCost", getQuadraticNoSlipCost(taskFile, "noSlipCost", libraryFolder, true));
  }
  // joint limits constraint
  problem_.softConstraintPtr->add("jointLimits", getJointLimitSoftConstraint(*pinocchioInterfacePtr_, taskFile));
  // end-effector state constraint
  problem_.stateSoftConstraintPtr->add("endEffector", getEndEffectorConstraint(*pinocchioInterfacePtr_, taskFile, "endEffector",
                                                                               usePreComputation, libraryFolder, recompileLibraries));
  problem_.finalSoftConstraintPtr->add("finalEndEffector", getEndEffectorConstraint(*pinocchioInterfacePtr_, taskFile, "finalEndEffector",
                                                                                    usePreComputation, libraryFolder, recompileLibraries));
  // self-collision avoidance constraint
  bool activateSelfCollision = true;
  loadData::loadPtreeValue(pt, activateSelfCollision, "selfCollision.activate", true);
  if (activateSelfCollision) {
    problem_.stateSoftConstraintPtr->add(
        "selfCollision", getSelfCollisionConstraint(*pinocchioInterfacePtr_, taskFile, urdfFile, "selfCollision", usePreComputation,
                                                    libraryFolder, recompileLibraries));
  }

  // Dynamics
  switch (manipulatorModelInfo_.manipulatorModelType) {
    case ManipulatorModelType::DefaultManipulator: {
      problem_.dynamicsPtr.reset(
          new DefaultManipulatorDynamics(manipulatorModelInfo_, "dynamics", libraryFolder, recompileLibraries, true));
      break;
    }
    case ManipulatorModelType::WheelBasedMobileManipulatorV1: {
      problem_.dynamicsPtr.reset(
          new WheelBasedMobileManipulatorV1Dynamics(manipulatorModelInfo_, "dynamics", libraryFolder, recompileLibraries, true));
      break;
    }
    case ManipulatorModelType::WheelBasedMobileManipulatorV2: {
      problem_.dynamicsPtr.reset(
          new WheelBasedMobileManipulatorV2Dynamics(manipulatorModelInfo_, "dynamics", libraryFolder, recompileLibraries, true));
      break;
    }
    default:
      throw std::invalid_argument("Invalid manipulator model type provided.");
  }

  /*
   * Pre-computation
   */
  if (usePreComputation) {
    problem_.preComputationPtr.reset(new MobileManipulatorPreComputation(*pinocchioInterfacePtr_, manipulatorModelInfo_));
  }

  // Rollout
  const auto rolloutSettings = rollout::loadSettings(taskFile, "rollout");
  rolloutPtr_.reset(new TimeTriggeredRollout(*problem_.dynamicsPtr, rolloutSettings));

  // Initialization
  initializerPtr_.reset(new DefaultInitializer(manipulatorModelInfo_.inputDim));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputCost> MobileManipulatorInterface::getQuadraticInputCost(const std::string& taskFile) {
  matrix_t R = matrix_t::Zero(manipulatorModelInfo_.inputDim, manipulatorModelInfo_.inputDim);
  const int baseInputDim = manipulatorModelInfo_.inputDim - manipulatorModelInfo_.armDim/2;
  const int armStateDim = manipulatorModelInfo_.armDim;
  const int stateDim = manipulatorModelInfo_.stateDim;

  // state costs
  matrix_t Q = matrix_t::Zero(stateDim, stateDim);
  loadData::loadEigenMatrix(taskFile, "Q", Q);

  std::cerr << "\n #### State Cost Settings: ";
  std::cerr << "\n #### =============================================================================\n";
  std::cerr << "Q:  \n" << Q << '\n';
  std::cerr << " #### =============================================================================\n";

  // arm base DOFs input costs
  if (baseInputDim > 0) {
    matrix_t R_base = matrix_t::Zero(baseInputDim, baseInputDim);
    loadData::loadEigenMatrix(taskFile, "inputCost.R.base." + modelTypeEnumToString(manipulatorModelInfo_.manipulatorModelType), R_base);
    R.topLeftCorner(baseInputDim, baseInputDim) = R_base;
  }

  // arm joints DOFs input costs
  matrix_t R_arm = matrix_t::Zero(armStateDim/2, armStateDim/2);
  loadData::loadEigenMatrix(taskFile, "inputCost.R.arm", R_arm);
  R.bottomRightCorner(armStateDim/2, armStateDim/2) = R_arm;

  std::cerr << "\n #### Input Cost Settings: ";
  std::cerr << "\n #### =============================================================================\n";
  std::cerr << "inputCost.R:  \n" << R << '\n';
  std::cerr << " #### =============================================================================\n";

  return std::make_unique<QuadraticInputCost>(std::move(Q), std::move(R), manipulatorModelInfo_.stateDim);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputCost> MobileManipulatorInterface::getQuadraticNoSlipCost(const std::string& taskFile, const std::string& prefix, 
                                                                                   const std::string& libraryFolder, bool recompileLibraries) {
  scalar_t muWheel = 1.0;
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::cerr << "\n #### No Slip: ";
  std::cerr << "\n #### =============================================================================\n";
  loadData::loadPtreeValue(pt, muWheel, "quadraticPenalty.noSlip", true);
  std::cerr << " #### =============================================================================\n";

  std::unique_ptr<StateInputConstraint> constraint;
  constraint.reset(
      new NoSlipConstraintCppAd(prefix, libraryFolder, recompileLibraries, manipulatorModelInfo_));
  std::vector<std::unique_ptr<PenaltyBase>> penaltyArray(8);
  std::generate_n(penaltyArray.begin(), 8, [&] { return std::unique_ptr<PenaltyBase>(new QuadraticPenalty(muWheel)); });
  return std::unique_ptr<StateInputCost>(new StateInputSoftConstraint(std::move(constraint), std::move(penaltyArray)));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateCost> MobileManipulatorInterface::getEndEffectorConstraint(const PinocchioInterface& pinocchioInterface,
                                                                                const std::string& taskFile, const std::string& prefix,
                                                                                bool usePreComputation, const std::string& libraryFolder,
                                                                                bool recompileLibraries) {
  scalar_t muPosition = 1.0;
  scalar_t muOrientation = 1.0;
  const std::string name = "WRIST_2";

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::cerr << "\n #### " << prefix << " Settings: ";
  std::cerr << "\n #### =============================================================================\n";
  loadData::loadPtreeValue(pt, muPosition, prefix + ".muPosition", true);
  loadData::loadPtreeValue(pt, muOrientation, prefix + ".muOrientation", true);
  std::cerr << " #### =============================================================================\n";

  if (referenceManagerPtr_ == nullptr) {
    throw std::runtime_error("[getEndEffectorConstraint] referenceManagerPtr_ should be set first!");
  }

  std::unique_ptr<StateConstraint> constraint;
  if (usePreComputation) {
    MobileManipulatorPinocchioMapping pinocchioMapping(manipulatorModelInfo_);
    PinocchioEndEffectorKinematics eeKinematics(pinocchioInterface, pinocchioMapping, {manipulatorModelInfo_.eeFrame});
    constraint.reset(new EndEffectorConstraint(eeKinematics, *referenceManagerPtr_));
  } else {
    MobileManipulatorPinocchioMappingCppAd pinocchioMappingCppAd(manipulatorModelInfo_);
    PinocchioEndEffectorKinematicsCppAd eeKinematics(pinocchioInterface, pinocchioMappingCppAd, {manipulatorModelInfo_.eeFrame},
                                                     manipulatorModelInfo_.stateDim, manipulatorModelInfo_.inputDim,
                                                     "end_effector_kinematics", libraryFolder, recompileLibraries, false);
    constraint.reset(new EndEffectorConstraint(eeKinematics, *referenceManagerPtr_));
  }

  std::vector<std::unique_ptr<PenaltyBase>> penaltyArray(6);
  std::generate_n(penaltyArray.begin(), 3, [&] { return std::make_unique<QuadraticPenalty>(muPosition); });
  std::generate_n(penaltyArray.begin() + 3, 3, [&] { return std::make_unique<QuadraticPenalty>(muOrientation); });

  return std::make_unique<StateSoftConstraint>(std::move(constraint), std::move(penaltyArray));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateCost> MobileManipulatorInterface::getSelfCollisionConstraint(const PinocchioInterface& pinocchioInterface,
                                                                                  const std::string& taskFile, const std::string& urdfFile,
                                                                                  const std::string& prefix, bool usePreComputation,
                                                                                  const std::string& libraryFolder,
                                                                                  bool recompileLibraries) {
  std::vector<std::pair<size_t, size_t>> collisionObjectPairs;
  std::vector<std::pair<std::string, std::string>> collisionLinkPairs;
  scalar_t mu = 1e-2;
  scalar_t delta = 1e-3;
  scalar_t minimumDistance = 0.0;

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::cerr << "\n #### SelfCollision Settings: ";
  std::cerr << "\n #### =============================================================================\n";
  loadData::loadPtreeValue(pt, mu, prefix + ".mu", true);
  loadData::loadPtreeValue(pt, delta, prefix + ".delta", true);
  loadData::loadPtreeValue(pt, minimumDistance, prefix + ".minimumDistance", true);
  loadData::loadStdVectorOfPair(taskFile, prefix + ".collisionObjectPairs", collisionObjectPairs, true);
  loadData::loadStdVectorOfPair(taskFile, prefix + ".collisionLinkPairs", collisionLinkPairs, true);
  std::cerr << " #### =============================================================================\n";

  PinocchioGeometryInterface geometryInterface(pinocchioInterface, collisionLinkPairs, collisionObjectPairs);

  const size_t numCollisionPairs = geometryInterface.getNumCollisionPairs();
  std::cerr << "SelfCollision: Testing for " << numCollisionPairs << " collision pairs\n";

  std::unique_ptr<StateConstraint> constraint;
  if (usePreComputation) {
    constraint = std::make_unique<MobileManipulatorSelfCollisionConstraint>(MobileManipulatorPinocchioMapping(manipulatorModelInfo_),
                                                                            std::move(geometryInterface), minimumDistance);
  } else {
    constraint = std::make_unique<SelfCollisionConstraintCppAd>(
        pinocchioInterface, MobileManipulatorPinocchioMapping(manipulatorModelInfo_), std::move(geometryInterface), minimumDistance,
        "self_collision", libraryFolder, recompileLibraries, false);
  }

  auto penalty = std::make_unique<RelaxedBarrierPenalty>(RelaxedBarrierPenalty::Config{mu, delta});

  return std::make_unique<StateSoftConstraint>(std::move(constraint), std::move(penalty));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputCost> MobileManipulatorInterface::getJointLimitSoftConstraint(const PinocchioInterface& pinocchioInterface,
                                                                                        const std::string& taskFile) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);

  bool activateJointPositionLimit = true;
  loadData::loadPtreeValue(pt, activateJointPositionLimit, "jointPositionLimits.activate", true);

  const int baseStateDim = manipulatorModelInfo_.stateDim - manipulatorModelInfo_.armDim;
  const int armStateDim = manipulatorModelInfo_.armDim;
  const int baseInputDim = manipulatorModelInfo_.inputDim - manipulatorModelInfo_.armDim/2;
  const int armInputDim = manipulatorModelInfo_.armDim/2;
  const auto& model = pinocchioInterface.getModel();

  // Load position and velocity limits
  std::vector<StateInputSoftBoxConstraint::BoxConstraint> stateLimits;
  if (activateJointPositionLimit) {
    scalar_t muLimits = 1e-2;
    scalar_t deltaLimits = 1e-3;

    // arm joint DOF limits from the parsed URDF
    const int arm_dof = armStateDim/2;
    int no_of_joints;
    if (baseStateDim > 0) {
      no_of_joints = arm_dof + (baseStateDim - 3);
    }
    else {
      no_of_joints = arm_dof;
    }
    const vector_t positionlowerBound = model.lowerPositionLimit.tail(no_of_joints);
    const vector_t positionupperBound = model.upperPositionLimit.tail(no_of_joints);
    const vector_t velocitylowerBound = -model.velocityLimit.tail(arm_dof);
    const vector_t velocityupperBound = model.velocityLimit.tail(arm_dof);

    vector_t lowbnd(no_of_joints + arm_dof);
    vector_t uppbnd(no_of_joints + arm_dof);
    lowbnd << positionlowerBound, velocitylowerBound;
    uppbnd << positionupperBound, velocityupperBound;
    const vector_t lowerBound = lowbnd;
    const vector_t upperBound = uppbnd;

    std::cerr << "\n #### JointPositionAndVelocityLimits Settings: ";
    std::cerr << "\n #### =============================================================================\n";
    std::cerr << " #### lowerBound: " << lowerBound.transpose() << '\n';
    std::cerr << " #### upperBound: " << upperBound.transpose() << '\n';
    loadData::loadPtreeValue(pt, muLimits, "jointPositionLimits.mu", true);
    loadData::loadPtreeValue(pt, deltaLimits, "jointPositionLimits.delta", true);
    std::cerr << " #### =============================================================================\n";

    stateLimits.reserve(no_of_joints + arm_dof);
    for (int i = 0; i < no_of_joints + arm_dof; ++i) {
      StateInputSoftBoxConstraint::BoxConstraint boxConstraint;
      if (baseStateDim > 0) {
        boxConstraint.index = 3 + i;
      }
      else {
        boxConstraint.index = i;
      }
      boxConstraint.lowerBound = lowerBound(i);
      boxConstraint.upperBound = upperBound(i);
      boxConstraint.penaltyPtr.reset(new RelaxedBarrierPenalty({muLimits, deltaLimits}));
      stateLimits.push_back(std::move(boxConstraint));
    }
  }

  // load acceleration limits
  std::vector<StateInputSoftBoxConstraint::BoxConstraint> inputLimits;
  {
    int effectiveBaseInputDim;
    if (baseInputDim == 11) {
      effectiveBaseInputDim = baseInputDim - 3;
    }
    else {
      effectiveBaseInputDim = baseInputDim;
    }

    vector_t lowerBound = vector_t::Zero(effectiveBaseInputDim + armInputDim);
    vector_t upperBound = vector_t::Zero(effectiveBaseInputDim + armInputDim);
    scalar_t muAccelerationLimits = 1e-2;
    scalar_t deltaAccelerationLimits = 1e-3;

    // Base DOFs velocity limits
    if (baseInputDim > 0) {
      vector_t lowerBoundBase = vector_t::Zero(effectiveBaseInputDim);
      vector_t upperBoundBase = vector_t::Zero(effectiveBaseInputDim);
      loadData::loadEigenMatrix(taskFile,
                                "jointAccelerationLimits.lowerBound.base." + modelTypeEnumToString(manipulatorModelInfo_.manipulatorModelType),
                                lowerBoundBase);
      loadData::loadEigenMatrix(taskFile,
                                "jointAccelerationLimits.upperBound.base." + modelTypeEnumToString(manipulatorModelInfo_.manipulatorModelType),
                                upperBoundBase);
      lowerBound.head(effectiveBaseInputDim) = lowerBoundBase;
      upperBound.head(effectiveBaseInputDim) = upperBoundBase;
    }

    // arm joint DOFs acceleration limits
    vector_t lowerBoundArm = vector_t::Zero(armInputDim);
    vector_t upperBoundArm = vector_t::Zero(armInputDim);
    loadData::loadEigenMatrix(taskFile, "jointAccelerationLimits.lowerBound.arm", lowerBoundArm);
    loadData::loadEigenMatrix(taskFile, "jointAccelerationLimits.upperBound.arm", upperBoundArm);
    lowerBound.tail(armInputDim) = lowerBoundArm;
    upperBound.tail(armInputDim) = upperBoundArm;

    std::cerr << "\n #### JointAccelerationLimits Settings: ";
    std::cerr << "\n #### =============================================================================\n";
    std::cerr << " #### 'lowerBound':  " << lowerBound.transpose() << std::endl;
    std::cerr << " #### 'upperBound':  " << upperBound.transpose() << std::endl;
    loadData::loadPtreeValue(pt, muAccelerationLimits, "jointAccelerationLimits.mu", true);
    loadData::loadPtreeValue(pt, deltaAccelerationLimits, "jointAccelerationLimits.delta", true);
    std::cerr << " #### =============================================================================\n";

    inputLimits.reserve(effectiveBaseInputDim + armInputDim);
    for (int i = 0; i < effectiveBaseInputDim + armInputDim; ++i) {
      StateInputSoftBoxConstraint::BoxConstraint boxConstraint;
      if (baseInputDim == 11) {
        boxConstraint.index = 3 + i;
      }
      else {
        boxConstraint.index = i;
      }
      boxConstraint.lowerBound = lowerBound(i);
      boxConstraint.upperBound = upperBound(i);
      boxConstraint.penaltyPtr.reset(new RelaxedBarrierPenalty({muAccelerationLimits, deltaAccelerationLimits}));
      inputLimits.push_back(std::move(boxConstraint));
    }
  }

  auto boxConstraints = std::make_unique<StateInputSoftBoxConstraint>(stateLimits, inputLimits);
  boxConstraints->initializeOffset(0.0, vector_t::Zero(manipulatorModelInfo_.stateDim), vector_t::Zero(manipulatorModelInfo_.inputDim));
  return boxConstraints;
}

}  // namespace mobile_manipulator
}  // namespace ocs2
