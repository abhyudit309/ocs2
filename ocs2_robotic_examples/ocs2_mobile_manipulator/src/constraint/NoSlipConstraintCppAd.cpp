#include "ocs2_mobile_manipulator/constraint/NoSlipConstraintCppAd.h"

namespace ocs2 {
namespace mobile_manipulator {

NoSlipConstraintCppAd::NoSlipConstraintCppAd(const std::string& prefix, const std::string& libraryFolder, bool recompileLibraries, ManipulatorModelInfo manipulatorModelInfo) 
  : StateInputConstraintCppAd(ConstraintOrder::Linear), manipulatorModelInfo_(manipulatorModelInfo) {
  initialize(manipulatorModelInfo_.stateDim, manipulatorModelInfo_.inputDim, 0, prefix, libraryFolder, recompileLibraries, true);
}

NoSlipConstraintCppAd::NoSlipConstraintCppAd(const NoSlipConstraintCppAd& rhs)
    : StateInputConstraintCppAd(rhs), manipulatorModelInfo_(rhs.manipulatorModelInfo_) {}

ad_vector_t NoSlipConstraintCppAd::constraintFunction(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                                                            const ad_vector_t& parameters) const {
  ad_vector_t constraint = Eigen::Matrix<ad_scalar_t, 8, 1>::Zero();
  // geometrical parameters:
  const double a = 0.05539;
  const double l = 0.13;
  const double d = 0.1325;
  const double wx = 0.005;
  const double wz = 0.017114;
  // extract parameters from state and input vector
  // from state:
  auto theta = state(2);
  auto p1 = state(3);
  auto p2 = state(4);
  auto q1 = state(5);
  auto q2 = state(6);
  auto r1 = state(7);
  auto r2 = state(8);
  auto s1 = state(9);
  auto s2 = state(10);
  // from input:
  auto x_dot = input(0);
  auto y_dot = input(1);
  auto theta_dot = input(2);
  auto p1_dot = input(3);
  auto p2_dot = input(4);
  auto q1_dot = input(5);
  auto q2_dot = input(6);
  auto r1_dot = input(7);
  auto r2_dot = input(8);
  auto s1_dot = input(9);
  auto s2_dot = input(10);

  // set up constraints:
  ad_scalar_t c1 = x_dot + l*theta_dot*cos(theta) - d*theta_dot*sin(theta) - a*p2_dot*sin(p1 + theta) - wx*(p1_dot + theta_dot)*sin(p1 + theta) + wz*(p1_dot + theta_dot)*cos(p1 + theta);
  ad_scalar_t c2 = y_dot + l*theta_dot*sin(theta) + d*theta_dot*cos(theta) + a*p2_dot*cos(p1 + theta) + wx*(p1_dot + theta_dot)*cos(p1 + theta) + wz*(p1_dot + theta_dot)*sin(p1 + theta);

  ad_scalar_t c3 = x_dot + l*theta_dot*cos(theta) + d*theta_dot*sin(theta) - a*q2_dot*sin(q1 + theta) + wx*(q1_dot + theta_dot)*sin(q1 + theta) + wz*(q1_dot + theta_dot)*cos(q1 + theta);
  ad_scalar_t c4 = y_dot + l*theta_dot*sin(theta) - d*theta_dot*cos(theta) + a*q2_dot*cos(q1 + theta) - wx*(q1_dot + theta_dot)*cos(q1 + theta) + wz*(q1_dot + theta_dot)*sin(q1 + theta);

  ad_scalar_t c5 = x_dot - l*theta_dot*cos(theta) - d*theta_dot*sin(theta) - a*r2_dot*sin(r1 + theta) - wx*(r1_dot + theta_dot)*sin(r1 + theta) - wz*(r1_dot + theta_dot)*cos(r1 + theta);
  ad_scalar_t c6 = y_dot - l*theta_dot*sin(theta) + d*theta_dot*cos(theta) + a*r2_dot*cos(r1 + theta) + wx*(r1_dot + theta_dot)*cos(r1 + theta) - wz*(r1_dot + theta_dot)*sin(r1 + theta);

  ad_scalar_t c7 = x_dot - l*theta_dot*cos(theta) + d*theta_dot*sin(theta) - a*s2_dot*sin(s1 + theta) + wx*(s1_dot + theta_dot)*sin(s1 + theta) - wz*(s1_dot + theta_dot)*cos(s1 + theta);
  ad_scalar_t c8 = y_dot - l*theta_dot*sin(theta) - d*theta_dot*cos(theta) + a*s2_dot*cos(s1 + theta) - wx*(s1_dot + theta_dot)*cos(s1 + theta) - wz*(s1_dot + theta_dot)*sin(s1 + theta);

  constraint << c1, c2, c3, c4, c5, c6, c7, c8;
  return constraint;
}

NoSlipConstraintCppAd* NoSlipConstraintCppAd::clone() const {
  return new NoSlipConstraintCppAd(*this);
}

}  // namespace mobile_manipulator
}  // namespace ocs2