#include "immkf.h"

Imm::Imm(const std::vector<KalmanFilter>& models,
         const Eigen::MatrixXd& model_trans, const Eigen::MatrixXd& P_trans,
         const Eigen::VectorXd& U_prob)
    : models(models),
      P_trans(P_trans),
      U_prob(U_prob),
      model_trans(model_trans),
      mode_cnt(models.size()),
      dim(models[0].A.rows()) {}

Eigen::VectorXd Imm::filt(const Eigen::VectorXd& Z) {
  Eigen::VectorXd u = P_trans.transpose() * U_prob;
  Eigen::MatrixXd mu = Eigen::MatrixXd::Zero(P_trans.rows(), P_trans.cols());

  for (int i = 0; i < mode_cnt; ++i) {
    for (int j = 0; j < mode_cnt; ++j) {
      mu(i, j) = P_trans(i, j) * U_prob(i) / u(j);
    }
  }

  std::vector<Eigen::VectorXd> X_mix(mode_cnt,
                                     Eigen::VectorXd::Zero(models[0].X.rows()));
  for (int j = 0; j < mode_cnt; ++j) {
    for (int i = 0; i < mode_cnt; ++i) {
      X_mix[j] += model_trans.block(j * dim, i * dim, dim, dim) * models[i].X *
                  mu(i, j);
    }
  }

  std::vector<Eigen::MatrixXd> P_mix(
      mode_cnt, Eigen::MatrixXd::Zero(models[0].P.rows(), models[0].P.cols()));
  for (int j = 0; j < mode_cnt; ++j) {
    for (int i = 0; i < mode_cnt; ++i) {
      Eigen::MatrixXd P =
          models[i].P +
          (models[i].X - X_mix[i]) * (models[i].X - X_mix[i]).transpose();
      P_mix[j] += mu(i, j) * model_trans.block(j * dim, i * dim, dim, dim) * P *
                  model_trans.block(j * dim, i * dim, dim, dim).transpose();
    }
  }

  for (int j = 0; j < mode_cnt; ++j) {
    models[j].X = X_mix[j];
    models[j].P = P_mix[j];
    models[j].filt(Z);
  }

  for (int j = 0; j < mode_cnt; ++j) {
    Eigen::VectorXd D = Z - models[j].H * models[j].X_pre;
    Eigen::MatrixXd S =
        models[j].H * models[j].P_pre * models[j].H.transpose() + models[j].R;
    double Lambda = std::exp(-0.5 * D.transpose() * S.inverse() * D) /
                    std::sqrt((2 * M_PI * S).determinant());
    U_prob(j) = Lambda * u(j);
  }
  U_prob /= U_prob.sum();

  return U_prob;
}
