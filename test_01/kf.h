#ifndef KF_H_
#define KF_H_

#include <Eigen/Dense>
#include <cmath>

#include "cyber/cyber.h"

namespace apollo {
namespace seyond {
class KF {
    public:
  KF(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd H, Eigen::MatrixXd Q,
     Eigen::MatrixXd R);
  KF(Eigen::MatrixXd A, Eigen::MatrixXd H);

  Eigen::VectorXd filt_(Eigen::VectorXd Z);

 private:
  void predict_();
  void update_(Eigen::VectorXd Z);

  public:
  Eigen::MatrixXd A_;
  Eigen::MatrixXd B_;
  Eigen::MatrixXd U_;
  Eigen::MatrixXd P_;
  Eigen::MatrixXd P_pre_;
  Eigen::MatrixXd H_;
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd R_;

  Eigen::VectorXd X_;
  Eigen::VectorXd X_pre_;
};
} // namespace seyond
}  // namespace apollo

#endif