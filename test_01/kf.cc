#include "kf.h"
namespace apollo {
namespace seyond {
KF::KF(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd H,
       Eigen::MatrixXd Q, Eigen::MatrixXd R) {
  A_ = A;
  B_ = B;
  H_ = H;
  Q_ = Q;
  R_ = R;
  U_ = Eigen::VectorXd::Zero(B.cols());
  X_ = Eigen::VectorXd::Zero(A.rows());
  X_pre_ = Eigen::VectorXd::Zero(A.rows());

  P_ = Eigen::MatrixXd::Zero(A.rows(), A.cols());
  P_pre_ = Eigen::MatrixXd::Zero(A.rows(), A.cols());
}

KF::KF(Eigen::MatrixXd A, Eigen::MatrixXd H) {
  A_ = A;
  H_ = H;
  B_ = Eigen::MatrixXd::Identity(A.rows(), A.cols());
  Q_ = Eigen::MatrixXd::Identity(A.rows(), A.cols());
  //   4*4
  R_ = Eigen::MatrixXd::Identity(H.rows(), H.rows());

  U_ = Eigen::VectorXd::Zero(B_.cols());
  X_ = Eigen::VectorXd::Zero(A.rows());
  X_pre_ = Eigen::VectorXd::Zero(A.rows());
  P_ = Eigen::MatrixXd::Zero(A.rows(), A.cols());
  P_pre_ = Eigen::MatrixXd::Zero(A.rows(), A.cols());
}

Eigen::VectorXd KF::filt_(Eigen::VectorXd Z) {
  predict_();
  update_(Z);
  return X_;
}

void KF::predict_() {
  X_ = A_ * X_ + B_ * U_;
  P_ = A_ * P_ * A_.transpose() + Q_;
}

void KF::update_(Eigen::VectorXd Z) {
  Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
  Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
  X_ = X_ + K * (Z - H_ * X_);
  P_ = (Eigen::MatrixXd::Identity(P_.rows(), P_.cols()) - K * H_) * P_;
}
}  // namespace seyond
}  // namespace apollo