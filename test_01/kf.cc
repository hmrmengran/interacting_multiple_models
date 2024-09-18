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
  // 2*2
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
  X_pre_ = A_ * X_ + B_ * U_;
  P_pre_ = A_ * P_ * A_.transpose() + Q_;
}

// void KF::update_(Eigen::VectorXd Z) {
//   // Regularization parameter to improve numerical stability
//   double epsilon = 1e-6;
//   Eigen::MatrixXd S = H_ * P_pre_ * H_.transpose() + R_;
//   S += epsilon * Eigen::MatrixXd::Identity(S.rows(), S.cols());
//   // Use Cholesky decomposition for the Kalman gain calculation
//   Eigen::LLT<Eigen::MatrixXd> lltOfS(S);
//   Eigen::MatrixXd K =
//       P_pre_ * H_.transpose() *
//       lltOfS.solve(Eigen::MatrixXd::Identity(S.rows(), S.cols()));
//   // Update state estimate
//   X_ = X_pre_ + K * (Z - H_ * X_pre_);
//   // Update covariance estimate
//   P_ = P_pre_ - K * H_ * P_pre_;
// }

void KF::update_(Eigen::VectorXd Z) {
  Eigen::MatrixXd S = H_ * P_pre_ * H_.transpose() + R_;
  Eigen::MatrixXd K = P_pre_ * H_.transpose() * S.inverse();
  X_ = X_pre_ + K * (Z - H_ * X_pre_);
  P_ = (Eigen::MatrixXd::Identity(P_pre_.rows(), P_pre_.cols()) - K * H_) * P_pre_;
}

}  // namespace seyond
}  // namespace apollo