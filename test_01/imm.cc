#include "imm.h"

namespace apollo {
namespace seyond {
Imm::Imm(const std::vector<std::shared_ptr<KF>>& models,
         const std::vector<std::vector<std::shared_ptr<Eigen::MatrixXd>>>& model_trans,
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

  std::vector<std::shared_ptr<Eigen::MatrixXd>> X_mix(mode_cnt_);

  // std::cout << "X_mix start" << std::endl;
  for (int j = 0; j < mode_cnt_; ++j) {
    X_mix[j] = std::make_unique<Eigen::MatrixXd>(models_[j]->X_.rows(),
                                                 models_[j]->X_.cols());
    X_mix[j]->setZero();
    for (int i = 0; i < mode_cnt_; ++i) {
      Eigen::MatrixXd X_i = models_[i]->X_;
      *X_mix[j] += (*model_trans_[j][i]) * X_i * mu(i, j);
    }
  }
  // std::cout << "X_mix end" << std::endl;

  // std::cout << "P_mix start" << std::endl;
  std::vector<std::shared_ptr<Eigen::MatrixXd>> P_mix(mode_cnt_);
  for (int j = 0; j < mode_cnt_; ++j) {
    P_mix[j] = std::make_unique<Eigen::MatrixXd>(models_[j]->P_.rows(),
                                                 models_[j]->P_.cols());
    P_mix[j]->setZero();
    for (int i = 0; i < mode_cnt_; ++i) {
      //Explicitly convert VectorXd to MatrixXd
      Eigen::MatrixXd X_matrix = models_[i]->X_.matrix();
      Eigen::MatrixXd P = models_[i]->P_ + ((X_matrix - *X_mix[i])) * ((X_matrix - *X_mix[i])).transpose();
      *P_mix[j] += mu(i, j) * (*model_trans_[j][i] * P) * model_trans_[j][i]->transpose();
    }
  }
  // std::cout << "P_mix end" << std::endl;

  // Apply the filtering step
  for (int j = 0; j < mode_cnt_; ++j) {
    // std::cout << "Model: " << j << std::endl;
    models_[j]->X_ = *X_mix[j];
    models_[j]->P_ = *P_mix[j];
    models_[j]->filt_(Z);
  }

  // Update probabilities
  Eigen::MatrixXd updated_U_prob(mode_cnt_, 1);
  for (int j = 0; j < mode_cnt_; ++j) {
    Eigen::VectorXd D = Z - models_[j]->H_ * models_[j]->X_pre_;
    Eigen::MatrixXd S =
        models_[j]->H_ * models_[j]->P_pre_ * models_[j]->H_.transpose() +
        models_[j]->R_;

    // Ensure S is symmetric positive definite
    Eigen::MatrixXd S_inv = S.inverse();

    // Compute the exponent term
    auto exponent = -0.5 * D.transpose() * S_inv * D;
    // Extract scalar from 1x1 matrix
    double exponent_value = exponent(0, 0);

    // Compute the likelihood (Lambda)
    double det_S = S.determinant();
    double Lambda = std::pow(2 * M_PI, -Z.size() / 2.0) *
                    std::pow(det_S, -0.5) * std::exp(exponent_value);

    updated_U_prob(j, 0) = Lambda * u(j, 0);
  }

  updated_U_prob = updated_U_prob / updated_U_prob.sum();
  U_prob_ = updated_U_prob;

  return U_prob_;
}
}  // namespace seyond
}  // namespace apollo