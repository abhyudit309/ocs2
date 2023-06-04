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

#include "ocs2_mobile_manipulator/FactoryFunctions.h"

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.
#include <pinocchio/multibody/joint/joint-composite.hpp>
#include <pinocchio/multibody/model.hpp>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_pinocchio_interface/urdf.h>

namespace ocs2 {
namespace mobile_manipulator {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioInterface createPinocchioInterface(const std::string& robotUrdfPath, const ManipulatorModelType& type) {
  switch (type) {
    case ManipulatorModelType::DefaultManipulator: {
      // return pinocchio interface
      return getPinocchioInterfaceFromUrdfFile(robotUrdfPath);
    }
    case ManipulatorModelType::WheelBasedMobileManipulatorV1: {
      // add XY-yaw joint for the wheel-base
      pinocchio::JointModelComposite jointComposite(3);
      jointComposite.addJoint(pinocchio::JointModelPX());
      jointComposite.addJoint(pinocchio::JointModelPY());
      jointComposite.addJoint(pinocchio::JointModelRZ());
      // return pinocchio interface
      return getPinocchioInterfaceFromUrdfFile(robotUrdfPath, jointComposite);
    }
    case ManipulatorModelType::WheelBasedMobileManipulatorV2: {
      // add XY-yaw joint for the wheel-base
      pinocchio::JointModelComposite jointComposite(3);
      jointComposite.addJoint(pinocchio::JointModelPX());
      jointComposite.addJoint(pinocchio::JointModelPY());
      jointComposite.addJoint(pinocchio::JointModelRZ());
      // return pinocchio interface
      return getPinocchioInterfaceFromUrdfFile(robotUrdfPath, jointComposite);
    }
    default:
      throw std::invalid_argument("Invalid manipulator model type provided.");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioInterface createPinocchioInterface(const std::string& robotUrdfPath, const ManipulatorModelType& type,
                                            const std::vector<std::string>& jointNames) {
  using joint_pair_t = std::pair<const std::string, std::shared_ptr<::urdf::Joint>>;

  // parse the URDF
  const auto urdfTree = ::urdf::parseURDFFile(robotUrdfPath);
  // remove extraneous joints from urdf
  ::urdf::ModelInterfaceSharedPtr newModel = std::make_shared<::urdf::ModelInterface>(*urdfTree);
  for (joint_pair_t& jointPair : newModel->joints_) {
    if (std::find(jointNames.begin(), jointNames.end(), jointPair.first) != jointNames.end()) {
      jointPair.second->type = urdf::Joint::FIXED;
    }
  }
  // resolve for the robot type
  switch (type) {
    case ManipulatorModelType::DefaultManipulator: {
      // return pinocchio interface
      return getPinocchioInterfaceFromUrdfModel(newModel);
    }
    case ManipulatorModelType::WheelBasedMobileManipulatorV1: {
      // add XY-yaw joint for the wheel-base
      pinocchio::JointModelComposite jointComposite(3);
      jointComposite.addJoint(pinocchio::JointModelPX());
      jointComposite.addJoint(pinocchio::JointModelPY());
      jointComposite.addJoint(pinocchio::JointModelRZ());
      // return pinocchio interface
      return getPinocchioInterfaceFromUrdfModel(newModel, jointComposite);
    }
    case ManipulatorModelType::WheelBasedMobileManipulatorV2: {
      // add XY-yaw joint for the wheel-base
      pinocchio::JointModelComposite jointComposite(3);
      jointComposite.addJoint(pinocchio::JointModelPX());
      jointComposite.addJoint(pinocchio::JointModelPY());
      jointComposite.addJoint(pinocchio::JointModelRZ());
      // return pinocchio interface
      return getPinocchioInterfaceFromUrdfModel(newModel, jointComposite);
    }
    default:
      throw std::invalid_argument("Invalid manipulator model type provided.");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ManipulatorModelInfo createManipulatorModelInfo(const PinocchioInterface& interface, const ManipulatorModelType& type,
                                                const std::string& baseFrame, const std::string& eeFrame) {
  const auto& model = interface.getModel();

  ManipulatorModelInfo info;
  info.manipulatorModelType = type;
  // resolve for actuated dof based on type of robot
  switch (type) {
    case ManipulatorModelType::DefaultManipulator: {
      // for default arm, the state dimension and input dimensions are same.
      info.stateDim = 2*model.nq;
      info.inputDim = model.nq;
      info.armDim = 2*model.nq;
      break;
    }
    case ManipulatorModelType::WheelBasedMobileManipulatorV1: {
      // for wheel-based, the input dimension is (v, omega, dq_j) while state dimension is (x, y, psi, q_j).
      info.stateDim = 2*(model.nq - 3) + 3;
      info.inputDim = (model.nq - 3) + 2;
      info.armDim = 2*(model.nq - 3);
      break;
    }
    case ManipulatorModelType::WheelBasedMobileManipulatorV2: {
      // for wheel-based, the input dimension is (v, omega, dq_j) while state dimension is (x, y, psi, q_j).
      info.stateDim = 2*(model.nq - 11) + 11;
      info.inputDim = (model.nq - 11) + 11;
      info.armDim = 2*(model.nq - 11);
      break;
    }
    default:
      throw std::invalid_argument("Invalid manipulator model type provided.");
      break;
  }
  // store frame names for using later.
  info.eeFrame = eeFrame;
  info.baseFrame = baseFrame;
  // get name of arm joints.
  const auto& jointNames = model.names;

  switch (type) {
    case ManipulatorModelType::DefaultManipulator: {
      info.dofNames = std::vector<std::string>(jointNames.end() - model.nq, jointNames.end());
      break;
    }
    case ManipulatorModelType::WheelBasedMobileManipulatorV1: {
      info.dofNames = std::vector<std::string>(jointNames.end() - (model.nq - 3), jointNames.end());
      break;
    }
    case ManipulatorModelType::WheelBasedMobileManipulatorV2: {
      info.dofNames = std::vector<std::string>(jointNames.end() - (model.nq - 3), jointNames.end());
      break;
    }
  }

  return info;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ManipulatorModelType loadManipulatorType(const std::string& configFilePath, const std::string& fieldName) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(configFilePath, pt);
  const size_t type = pt.template get<size_t>(fieldName);
  return static_cast<ManipulatorModelType>(type);
}

}  // namespace mobile_manipulator
}  // namespace ocs2