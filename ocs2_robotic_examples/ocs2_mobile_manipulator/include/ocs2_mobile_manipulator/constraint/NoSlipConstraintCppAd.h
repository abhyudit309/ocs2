#pragma once

#include <functional>
#include <string>
#include <vector>

#include <ocs2_core/Types.h>
#include <ocs2_core/constraint/StateInputConstraintCppAd.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>
#include "ocs2_mobile_manipulator/ManipulatorModelInfo.h"
#include "ocs2_mobile_manipulator/MobileManipulatorPinocchioMapping.h"

#include <ocs2_pinocchio_interface/pinocchio_forward_declaration.h>

namespace ocs2 {
namespace mobile_manipulator {

class NoSlipConstraintCppAd final: public StateInputConstraintCppAd {
  using ad_vector3 = Eigen::Matrix<ad_scalar_t, 3, 1>;
 public:
  NoSlipConstraintCppAd(const PinocchioInterface& pinocchioInterface, const PinocchioStateInputMapping<ad_scalar_t>& mapping,
                              const std::string& prefix, const std::string& libraryFolder, bool recompileLibraries, ManipulatorModelInfo manipulatorModelInfo);
  ~NoSlipConstraintCppAd() override = default;

  NoSlipConstraintCppAd* clone() const override;
  size_t getNumConstraints(scalar_t time) const override { return 8; }
  vector_t getParameters(scalar_t time) const { return vector_t(0); }
  ad_vector_t constraintFunction(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                                 const ad_vector_t& parameters) const override;
 protected:
  NoSlipConstraintCppAd(const NoSlipConstraintCppAd& rhs);

 private:
  const PinocchioInterface* pinocchioInterfacePtr_;
  std::unique_ptr<PinocchioStateInputMapping<ad_scalar_t>> mappingPtr_;
  ManipulatorModelInfo manipulatorModelInfo_;
};

}  // namespace swerve
}  // namespace ocs2