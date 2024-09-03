#ifndef IMM_H
#define IMM_H

#include <vector>

#include "kf.h"

namespace apollo {
namespace seyond {
class Imm {
 public:
  Imm(const std::vector<KF>& models,
      const Eigen::MatrixXd& model_trans, const Eigen::MatrixXd& P_trans,
      const Eigen::VectorXd& U_prob);
  Eigen::VectorXd filt(const Eigen::VectorXd& Z);

 private:
  std::vector<KF> models_;
  Eigen::MatrixXd P_trans_;
  Eigen::VectorXd U_prob_;
  Eigen::MatrixXd model_trans_;

  int mode_cnt_;
};
}  // namespace seyond
}  // namespace apollo

#endif  // IMM_H
