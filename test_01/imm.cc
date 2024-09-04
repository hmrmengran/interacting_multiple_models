#include "imm.h"

namespace apollo {
namespace seyond {
Imm::Imm(const std::vector<std::shared_ptr<KF>>& models,
         const std::vector<std::vector<Eigen::MatrixXd>>& model_trans,
         const Eigen::MatrixXd& P_trans, const Eigen::VectorXd& U_prob)
    : models_(models),
      P_trans_(P_trans),
      U_prob_(U_prob),
      model_trans_(model_trans),
      mode_cnt_(models.size()) {}

Eigen::MatrixXd Imm::filt(const Eigen::MatrixXd& Z) {
  Eigen::MatrixXd u = P_trans_.transpose() * U_prob_;
  Eigen::MatrixXd mu = Eigen::MatrixXd::Zero(P_trans_.rows(), P_trans_.cols());

  for (int i = 0; i < mode_cnt_; ++i) {
    for (int j = 0; j < mode_cnt_; ++j) {
      mu(i, j) = P_trans_(i, j) * U_prob_(i, 0) / u(j, 0);
    }
  }

  std::vector<std::unique_ptr<Eigen::MatrixXd>> X_mix(mode_cnt_);

  for (int j = 0; j < mode_cnt_; ++j) {
    X_mix[j] = std::make_unique<Eigen::MatrixXd>(models_[j]->X_.rows(),
                                                 models_[j]->X_.cols());
    X_mix[j]->setZero();
    for (int i = 0; i < mode_cnt_; ++i) {
      Eigen::MatrixXd X_i = models_[i]->X_;
      *X_mix[j] += model_trans_[j][i] * X_i * mu(i, j);
    }
  }

  std::vector<std::unique_ptr<Eigen::MatrixXd>> P_mix(mode_cnt_);
  for (int j = 0; j < mode_cnt_; ++j) {
    P_mix[j] = std::make_unique<Eigen::MatrixXd>(models_[j]->P_.rows(),
                                                 models_[j]->P_.cols());
    P_mix[j]->setZero();
    for (int i = 0; i < mode_cnt_; ++i) {
      Eigen::MatrixXd P(models_[i]->P_.rows(), models_[i]->P_.cols());
      P.setZero();
      P = models_[i]->P_ + (models_[i]->X_ - *X_mix[j]) *
                               (models_[i]->X_ - *X_mix[j]).transpose();
      P_mix[j] +=
          mu(i, j) * (model_trans_[j][i] * P) * model_trans_[j][i].transpose();
    }
  }

  // Apply the filtering step
  for (int j = 0; j < mode_cnt_; ++j) {
    models_[j]->X_ = *X_mix[j];
    models_[j]->P_ = *P_mix[j];
    models_[j]->filt_(Z);
  }

  // Update probabilities
  Eigen::MatrixXd updated_U_prob(mode_cnt_, 1);
  for (int j = 0; j < mode_cnt_; ++j) {
    Eigen::MatrixXd D = Z - models_[j]->H_ * models_[j]->X_;
    Eigen::MatrixXd S =
        models_[j]->H_ * models_[j]->P_ * models_[j]->H_.transpose() +
        models_[j]->R_;

    Eigen::MatrixXd result_matrix = -0.5 * (D.transpose() * S.inverse() * D);
    double result = result_matrix.sum();
    double Lambda =
        std::pow((2 * M_PI * S).determinant(), -0.5) * std::exp(result);

    updated_U_prob(j, 0) = Lambda * u(j, 0);
  }

  updated_U_prob = updated_U_prob / updated_U_prob.sum();
  U_prob_ = updated_U_prob;

  return U_prob_;
}
}  // namespace seyond
}  // namespace apollo
