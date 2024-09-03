#ifndef IMM_H
#define IMM_H

#include <vector>

#include "kalmanfilter.h"

class Imm {
 public:
  Imm(const std::vector<KalmanFilter>& models,
      const Eigen::MatrixXd& model_trans, const Eigen::MatrixXd& P_trans,
      const Eigen::VectorXd& U_prob);
  Eigen::VectorXd filt(const Eigen::VectorXd& Z);

 private:
  std::vector<KalmanFilter> models;
  Eigen::MatrixXd P_trans;
  Eigen::VectorXd U_prob;
  Eigen::MatrixXd model_trans;

  int mode_cnt;
  int dim;
};

#endif  // IMM_H
