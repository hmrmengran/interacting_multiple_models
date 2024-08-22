#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <cmath>
#include <Eigen/Dense>
#include "cyber/cyber.h"

class KFBase {
 public:
  virtual void init(const double& stamp, const Eigen::VectorXd& x) {}
  virtual KFBase* clone() { return new KFBase(*this); }
  virtual void updatePrediction() {}
  virtual void updateMeasurement() {}

  void predict();
  void predict(const double& stamp);
  void update(const Eigen::VectorXd& z);
  void updateOnce(const double& stamp, const Eigen::VectorXd* z = nullptr);
  void updateOneStep(const Eigen::VectorXd* z);

  Eigen::VectorXd x() const { return this->x_; }
  Eigen::MatrixXd P() const { return this->P_; }
  Eigen::MatrixXd S() const { return this->S_; }

  void setCurrentTimeStamp(const double& stamp) {
    this->dt_ = stamp - this->current_time_stamp_;
    this->current_time_stamp_ = stamp;
    if (this->dt_ < 0) this->dt_ = 1e-4;
  }
  void setStateCoveriance(const Eigen::MatrixXd& P) { this->P_ = P; }
  void setState(const Eigen::VectorXd& x) { this->x_ = x; }
  double likelihood() const { return this->likelihood_; }

 protected:
  Eigen::MatrixXd F_;
  Eigen::MatrixXd H_;
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd R_;
  Eigen::MatrixXd J_;
  Eigen::MatrixXd P_;
  Eigen::MatrixXd S_;
  Eigen::VectorXd x_;
  Eigen::VectorXd z_;

  std::vector<bool> angle_mask_;
  double likelihood_;
  double dt_;
  double current_time_stamp_;

  static double normalizeAngle(const double raw_angle) {
    double angle = std::fmod(raw_angle, 2 * M_PI);
    if (angle > M_PI) {
      angle -= 2 * M_PI;
    } else if (angle <= -M_PI) {
      angle += 2 * M_PI;
    }

    return angle;
  }
};

class CV : public KFBase {
 private:
  void updatePrediction();
  void updateMeasurement();

 public:
  void init(const double& stamp, const Eigen::VectorXd& x);
  virtual CV* clone() { return new CV(*this); }
  CV();
  ~CV();
};

class CA : public KFBase {
 private:
  void updatePrediction();
  void updateMeasurement();

 public:
  void init(const double& stamp, const Eigen::VectorXd& x);
  virtual CA* clone() { return new CA(*this); }
  CA();
  ~CA();
};

class CT : public KFBase {
 private:
  void updatePrediction();
  void updateMeasurement();
  const double yaw_rate_;

 public:
  void init(const double& stamp, const Eigen::VectorXd& x);
  virtual CT* clone() { return new CT(*this); }
  CT(const double& yaw_rate);
  ~CT();
};

class CTRV : public KFBase {
 private:
 public:
  void init(const double& stamp, const Eigen::VectorXd& x);
  virtual CTRV* clone() { return new CTRV(*this); }
  CTRV();
  ~CTRV();
};

class CTRA : public KFBase {
 private:
  void updatePrediction();
  void updateMeasurement();

 public:
  void init(const double& stamp, const Eigen::VectorXd& x);
  virtual CTRA* clone() { return new CTRA(*this); }
  CTRA();
  ~CTRA();
};

// class CVModel : public KalmanFilter {
//  public:
//   CVModel() : KalmanFilter(Eigen::MatrixXd(4, 4), Eigen::MatrixXd(2, 4)) {
//     A << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1;
//     H << 1, 0, 0, 0, 0, 1, 0, 0;
//   }
// };

// class CAModel : public KalmanFilter {
//  public:
//   CAModel() : KalmanFilter(Eigen::MatrixXd(6, 6), Eigen::MatrixXd(2, 6)) {
//     A << 1, 0, 1, 0, 0.5, 0, 0, 1, 0, 1, 0, 0.5, 0, 0, 1, 0, 1, 0, 0, 0, 0,
//     1,
//         0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1;
//     H << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0;
//   }
// };

// class CTModel : public KalmanFilter {
//  public:
//   CTModel(double turn_rate)
//       : KalmanFilter(Eigen::MatrixXd(5, 5), Eigen::MatrixXd(2, 5)),
//         turn_rate(turn_rate) {
//     A << 1, 0, sin(turn_rate), 0, (1 - cos(turn_rate)) / turn_rate, 0, 1, 0,
//         sin(turn_rate), (cos(turn_rate) - 1) / turn_rate, 0, 0,
//         cos(turn_rate), -sin(turn_rate), 0, 0, 0, sin(turn_rate),
//         cos(turn_rate), 0, 0, 0, 0, 0, 1;
//     H << 1, 0, 0, 0, 0, 0, 1, 0, 0, 0;
//   }

//  private:
//   double turn_rate;
// };

#endif  // KALMANFILTER_H
