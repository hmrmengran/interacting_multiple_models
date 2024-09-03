#include "imm.h"

namespace apollo {
namespace seyond {
Imm::Imm(const std::vector<KF>& models,
         const Eigen::MatrixXd& model_trans_, const Eigen::MatrixXd& P_trans_,
         const Eigen::VectorXd& U_prob_)
    : models_(models),
      P_trans_(P_trans_),
      U_prob_(U_prob_),
      model_trans_(model_trans_),
      mode_cnt_(models.size()) {}

Eigen::VectorXd Imm::filt(const Eigen::VectorXd& Z) {
  Eigen::VectorXd u = P_trans_.transpose() * U_prob_;
  Eigen::MatrixXd mu = Eigen::MatrixXd::Zero(P_trans_.rows(), P_trans_.cols());

  for (int i = 0; i < mode_cnt_; ++i) {
    for (int j = 0; j < mode_cnt_; ++j) {
      mu(i, j) = P_trans_(i, j) * U_prob_(i) / u(j);
    }
  }

  std::vector<Eigen::VectorXd> X_mix(mode_cnt_, Eigen::VectorXd::Zero(models_[0].X_.rows()));
  for (int j = 0; j < mode_cnt_; ++j) {
    for (int i = 0; i < mode_cnt_; ++i) {
      X_mix[j] += model_trans_.block(j * dim, i * dim, dim, dim) * models_[i].X_ *
                  mu(i, j);
    }
  }

  std::vector<Eigen::MatrixXd> P_mix(
      mode_cnt_, Eigen::MatrixXd::Zero(models_[0].P_.rows(), models_[0].P_.cols()));
  for (int j = 0; j < mode_cnt_; ++j) {
    for (int i = 0; i < mode_cnt_; ++i) {
      Eigen::MatrixXd P =
          models_[i].P_ +
          (models_[i].X_ - X_mix[i]) * (models_[i].X_ - X_mix[i]).transpose();
      P_mix[j] += mu(i, j) * model_trans_.block(j * dim, i * dim, dim, dim) * P *
                  model_trans_.block(j * dim, i * dim, dim, dim).transpose();
    }
  }

  for (int j = 0; j < mode_cnt_; ++j) {
    models_[j].X_ = X_mix[j];
    models_[j].P_ = P_mix[j];
    models_[j].filt_(Z);
  }

  for (int j = 0; j < mode_cnt_; ++j) {
    Eigen::VectorXd D = Z - models_[j].H_ * models_[j].X_pre_;
    Eigen::MatrixXd S =
        models_[j].H_ * models_[j].P_pre_ * models_[j].H_.transpose() + models_[j].R_;
    double Lambda = std::exp(-0.5 * D.transpose() * S.inverse() * D) /
                    std::sqrt((2 * M_PI * S).determinant());
    U_prob_(j) = Lambda * u(j);
  }
  U_prob_ /= U_prob_.sum();

  return U_prob_;
}
}  // namespace seyond
}  // namespace apollo
