#include "kalmanfilter.h"

void KFBase::predict(const double& stamp) {
  setCurrentTimeStamp(stamp);
  updatePrediction();
  this->P_ = this->F_ * this->P_ * this->F_.transpose() + this->Q_;
}

void KFBase::predict() {
  updatePrediction();
  this->P_ = this->F_ * this->P_ * this->F_.transpose() + this->Q_;
}

void KFBase::update(const Eigen::VectorXd& z) {
  updateMeasurement();
  Eigen::MatrixXd S = this->H_ * this->P_ * this->H_.transpose() + this->R_;
  Eigen::VectorXd v = z - this->z_;
  for (size_t i = 0; i < angle_mask_.size(); i++) {
    if (angle_mask_[i] == true) {
      v(i) = normalizeAngle(v(i));
    }
  }
  double det = S.determinant();
  this->S_ = S;
  S = S.inverse();
  this->likelihood_ =
      1.0 / sqrt(2 * M_PI * fabs(det)) * exp(-0.5 * v.transpose() * S * v);

  S = 0.5 * (S + S.transpose());
  Eigen::MatrixXd K = this->P_ * (this->H_.transpose() * S);

  this->x_ = this->x_ + K * v;
  Eigen::MatrixXd I;
  I.setIdentity(6, 6);
  Eigen::MatrixXd C = (I - K * this->H_);
  this->P_ = C * this->P_ * C.transpose() + K * R_ * K.transpose();
  this->P_ = this->P_ + 0.0001 * I;
}

void KFBase::updateOnce(const double& stamp, const Eigen::VectorXd* z) {
  if (z == nullptr) {
    predict(stamp);
  } else {
    predict(stamp);
    update(*z);
  }
}

void KFBase::updateOneStep(const Eigen::VectorXd* z) {
  if (z == nullptr) {
    predict();
  } else {
    predict();
    update(*z);
  }
}

CV::CV() {}

CV::~CV() {}

void CV::init(const double& stamp, const Eigen::VectorXd& x) {
  if (x.size() != 6) {
    AERROR << "[error] Dismatch between State and CV model.";
    exit(1);
  }

  this->current_time_stamp_ = stamp;
  this->P_.setIdentity(6, 6);
  this->R_.resize(3, 3);
  this->R_ << 0.25, 0, 0,  
              0, 0.25, 0, 
              0, 0, 0.25;
  this->x_ = x;
  this->F_.resize(6, 6);
  this->H_.resize(3, 6);
  this->H_ << 1, 0, 0, 0, 0, 0, 
              0, 1, 0, 0, 0, 0, 
              0, 0, 1, 0, 0, 0;
  this->angle_mask_ = {false, false, false, false};
  this->z_.resize(4);
}

void CV::updatePrediction() {
  Eigen::VectorXd xt;
  xt.resize(6);
  // double vx = x_(2);
  // double vy = x_(3);
  this->F_ << 1, 0, dt_, 0, 0, 0, 0, 1, 0, dt_, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
      1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1;
  this->x_ = this->F_ * this->x_;
  /*--------------------------------------------------------------------*\
  ** CALC Process Noice Q Matrix
  \*--------------------------------------------------------------------*/
  {
    // double delta_1 = dt_;
    double delta_2 = dt_ * dt_;
    // double delta_3 = dt_ * dt_ * dt_;
    Eigen::Matrix<double, 6, 2> G;
    G << 1 / 2.0 * delta_2, 0, 0, 1 / 2.0 * delta_2, dt_, 0, 0, dt_, 0, 0, 0, 0;
    Eigen::Matrix2d E;
    E << 400, 0, 0, 400;
    this->Q_ = G * E * G.transpose();
  }
}

void CV::updateMeasurement() {
  /*  观测量为 x, y, theta, v
      double vx = x_(2);
      double vy = x_(3);


      this->z_(0) = x_(0);
      this->z_(1) = x_(1);
      this->z_(2) = atan2(x_(3), x_(2));
      this->z_(3) = sqrt(x_(2) * x_(2) + x_(3) * x_(3));

      this->H_ << 1, 0,                       0,                      0, 0, 0,
                  0, 1,                       0,                      0, 0, 0,
                  0, 0,     -vy/(vx*vx + vy*vy),     vx/(vx*vx + vy*vy), 0, 0,
                  0, 0,  vx/sqrt(vx*vx + vy*vy), vy/sqrt(vx*vx + vy*vy), 0, 0;
   */

  this->z_(0) = x_(0);
  this->z_(1) = x_(1);
  this->z_(2) = x_(2);
  this->z_(3) = x_(3);
}


// CA::CA() {

// }

// CA::~CA() {

// }

// void CA::init(const double &stamp, const Eigen::VectorXd &x) {
//     if (x.size() != 6) {
//          std::cerr << "[error] Dismatch between State and CA model." << std::endl;
//          exit(1);
//     }
//     this->current_time_stamp_ = stamp;
//     this->P_.setIdentity(6, 6);
//     this->R_.resize(4, 4);
//     this->R_ << 0.25, 0, 0, 0,
//                 0, 0.25, 0, 0,
//                 0, 0, 5, 0,
//                 0, 0, 0, 5;
//     this->x_ = x; // x, y, theta, v
//     this->F_.resize(6, 6);
//     this->H_.resize(4, 6);      
//     this->H_ << 1, 0,  0, 0, 0, 0,
//                 0, 1,  0, 0, 0, 0,
//                 0, 0,  1, 0, 0, 0,
//                 0, 0,  0, 1, 0, 0;
//     this->angle_mask_ = {false, false, false, false};
//     this->z_.resize(4);
// }

// void CA::updatePrediction() {
//     Eigen::VectorXd xt;
//     xt.resize(6);
//     double vx = x_(2);
//     double vy = x_(3);
//     double ax = x_(4);
//     double ay = x_(5);
//     this->F_ <<     1,   0,   dt_,  0,  1 / 2.0 * dt_ * dt_,                     0,
//                     0,   1,   0,  dt_,                    0,   1 / 2.0 * dt_ * dt_,
//                     0,   0,   1,    0,                  dt_,                     0,
//                     0,   0,   0,    1,                    0,                   dt_,
//                     0,   0,   0,    0,                    1,                     0,
//                     0,   0,   0,    0,                    0,                     1;
//     this->x_ = this->F_ * this->x_;

//     ** CALC Process Noice Q Matrix

//   {
//       double delta_1 = dt_;
//       double delta_2 = dt_ * dt_;
//       double delta_3 = dt_ * dt_ * dt_; 
//       Eigen::Matrix<double, 6, 2> G;
//       G <<
//                   1 / 6.0 * delta_3,                  0,
//                                   0,  1 / 6.0 * delta_3, 
//                   1 / 2.0 * delta_2,                  0,
//                                   0,  1 / 2.0 * delta_2,
//                                 dt_,                  0,
//                                   0,                dt_;                    
//       Eigen::Matrix2d E;
//       E << 400, 0, 0, 400;
//       this->Q_ = G * E * G.transpose();
//   }
// }

// void CA::updateMeasurement() {
// /*  观测量为 x, y, theta, v
//     double vx = x_(2);
//     double vy = x_(3);

    
//     this->z_(0) = x_(0);
//     this->z_(1) = x_(1);
//     this->z_(2) = atan2(x_(3), x_(2));
//     this->z_(3) = sqrt(x_(2) * x_(2) + x_(3) * x_(3));

//     this->H_ << 1, 0,                       0,                      0, 0, 0,
//                 0, 1,                       0,                      0, 0, 0,
//                 0, 0,     -vy/(vx*vx + vy*vy),     vx/(vx*vx + vy*vy), 0, 0,
//                 0, 0,  vx/sqrt(vx*vx + vy*vy), vy/sqrt(vx*vx + vy*vy), 0, 0; */
//     // 观测量为 x, y, vx, vy            
//     this->z_(0) = x_(0);
//     this->z_(1) = x_(1);
//     this->z_(2) = x_(2);
//     this->z_(3) = x_(3);
// }