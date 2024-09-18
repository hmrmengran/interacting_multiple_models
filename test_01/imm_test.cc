#include <gtest/gtest.h>
#define private public
#include "cyber/cyber.h"
#include "cyber/common/file.h"
#include "modules/omnisense/interacting_multiple_models/proto/imm.pb.h"
#include "imm.h"
#include "kf.h"
namespace apollo {
namespace seyond {
#include <fstream>
#include <sstream>
#include <tuple>
using DataTuple = std::tuple<double, double, double, double>;

using namespace apollo::cyber::common;
using omnisense::imm::Conf;


std::vector<DataTuple> readCSV(const std::string &filename) {
  std::vector<DataTuple> data;
  std::ifstream file(filename);

  if (!file.is_open()) {
    std::cerr << "Error opening file: " << filename << std::endl;
    return data;
  }

  std::string line;
  // Read and ignore the header
  std::getline(file, line);

  while (std::getline(file, line)) {
    std::stringstream ss(line);
    std::string value;
    double x, vx, y, vy;

    // Read x, vx, y, vy in order
    std::getline(ss, value, ',');
    x = std::stod(value);

    std::getline(ss, value, ',');
    vx = std::stod(value);

    std::getline(ss, value, ',');
    y = std::stod(value);

    std::getline(ss, value, ',');
    vy = std::stod(value);

    data.emplace_back(x, vx, y, vy);
  }

  file.close();
  return data;
}

void writeCSV(const std::string &filename,
              const std::vector<Eigen::MatrixXd> &vec_z_filt,
              const std::vector<std::string> &headers) {
  std::ofstream file(filename);

  if (!file.is_open()) {
    std::cerr << "Error opening file: " << filename << std::endl;
    return;
  }

  // table header
  for (size_t i = 0; i < headers.size(); ++i) {
    file << headers[i];
    if (i != headers.size() - 1) {
      file << ",";
    }
  }
  file << "\n";

  // dump data
  for (const auto &row : vec_z_filt) {
    for (int i = 0; i < row.rows(); ++i) {
      file << row(i);
      if (i != row.rows() - 1) {
        file << ",";
      }
    }
    file << "\n";
  }

  file.close();
}

std::string config_file_path = "/apollo/modules/omnisense/interacting_multiple_models/conf/imm_test.pb.txt";
class KFTest : public ::testing::Test {
 protected:
  void SetUp() override {
    
    if (!GetProtoFromFile(config_file_path, &conf_)) {
      AERROR << "Parse conf file failed, " << conf_.DebugString();
    } else {
      AERROR << "Parse conf file success, " << conf_.DebugString();
    }

    // CV Model (Constant Velocity Model)
    A_cv_ = Eigen::MatrixXd(4, 4);
    H_cv_ = Eigen::MatrixXd(2, 4);
    A_cv_ << 1, dt, 0, 0, 
             0,  1, 0, 0, 
             0,  0, 1, dt, 
             0,  0, 0, 1;
    H_cv_ << 1, 0, 0, 0, 
             0, 0, 1, 0;
    cv_ = std::make_shared<KF>(A_cv_, H_cv_);

    // CA Model (Constant Acceleration Model)
    A_ca_ = Eigen::MatrixXd(6, 6);
    H_ca_ = Eigen::MatrixXd(2, 6);
    A_ca_ << 1, dt,  0.5 * dt * dt, 0,  0,             0, 
             0,  1,             dt, 0,  0,             0, 
             0,  0,              1, 0,  0,             0,  
             0,  0,              0, 1, dt, 0.5 * dt * dt, 
             0,  0,              0, 0,  1,            dt, 
             0,  0,              0, 0,  0,             1;
    H_ca_ << 1, 0, 0, 0, 0, 0, 
             0, 0, 0, 1, 0, 0;
    ca_ = std::make_shared<KF>(A_ca_, H_ca_);

    // CT Model (Constant Turn Model)
    A_ct_ = Eigen::MatrixXd(5, 5);
    H_ct_ = Eigen::MatrixXd(2, 5);
    A_ct_ << 1.0,         std::sin(theta) / dtheta, 0.0, -(1.0 - std::cos(theta)) / dtheta, 0.0, 
             0.0,                  std::cos(theta), 0.0,                  -std::sin(theta), 0.0, 
             0.0, (1.0 - std::cos(theta)) / dtheta, 1.0,          std::sin(theta) / dtheta, 0.0, 
             0.0,                  std::sin(theta), 0.0,                   std::cos(theta), 0.0, 
             0.0,                              0.0, 0.0,                               0.0, 1.0;
    H_ct_ << 1, 0, 0, 0, 0, 
             0, 0, 1, 0, 0;
    ct_ = std::make_shared<KF>(A_ct_, H_ct_);
  }

 private:
  double dt = 0.1;
  double dtheta = M_PI / 180 * 15;
  double theta = dtheta * dt;
  Eigen::MatrixXd A_cv_, H_cv_;
  Eigen::MatrixXd A_ca_, H_ca_;
  Eigen::MatrixXd A_ct_, H_ct_;

  std::shared_ptr<KF> cv_;
  std::shared_ptr<KF> ca_;
  std::shared_ptr<KF> ct_;
  Conf conf_;
};

// TEST_F(KFTest, TestCVInitialization) {
//   Eigen::VectorXd Z(2);
//   Z << 1, 1;
//   Eigen::VectorXd X = cv_->filt_(Z);
//   EXPECT_EQ(X.size(), 4);

//   EXPECT_DOUBLE_EQ(cv_->A_(0, 1), 0.1);
//   EXPECT_DOUBLE_EQ(cv_->A_(2, 3), 0.1);
// }

// TEST_F(KFTest, TestCAInitialization) {
//   Eigen::VectorXd Z(2);
//   Z << 1, 1;
//   Eigen::VectorXd X = ca_->filt_(Z);
//   EXPECT_EQ(X.size(), 6);

//   EXPECT_DOUBLE_EQ(ca_->A_(0, 1), 0.1);
//   EXPECT_DOUBLE_EQ(ca_->A_(0, 2), 0.5 * 0.1 * 0.1);
//   EXPECT_DOUBLE_EQ(ca_->A_(3, 4), 0.1);
//   EXPECT_DOUBLE_EQ(ca_->A_(3, 5), 0.5 * 0.1 * 0.1);
// }

// TEST_F(KFTest, TestCTInitialization) {
//   Eigen::VectorXd Z(2);
//   Z << 1, 1;
//   Eigen::VectorXd X = ct_->filt_(Z);
//   EXPECT_EQ(X.size(), 5);

//   EXPECT_DOUBLE_EQ(ct_->A_(0, 1), std::sin(theta) / dtheta);
//   EXPECT_DOUBLE_EQ(ct_->A_(3, 3), std::cos(theta));
// }

// TEST_F(KFTest, CVT_Test) {
//   Eigen::MatrixXd P_trans(2, 2);
//   P_trans << 0.98, 0.02, 0.02, 0.98;

//   Eigen::VectorXd U_prob(2);
//   U_prob << 0.5, 0.5;
//   std::vector<std::shared_ptr<KF>> models = {cv_, ct_};
//   Eigen::MatrixXd r(2, 1);
//   r << 10.0, 10.0;

//   // braodcast r to all models
//   for (auto &model : models) {
//     ACHECK(model->R_.rows() == r.rows());
//     Eigen::MatrixXd R = model->R_;
//     for (int i = 0; i < R.rows(); ++i) {
//       model->R_.row(i) = R.row(i) * r(i);
//     }
//   }

//   Eigen::MatrixXd T12(6, 4);
//   T12 << 1, 0, 0, 0, 
//         0, 1, 0, 0, 
//         0, 0, 0, 0, 
//         0, 0, 1, 0, 
//         0, 0, 0, 1, 
//         0, 0, 0, 0;

//   Eigen::MatrixXd T23(5, 6);
//   T23 << 1, 0, 0, 0, 0, 0, 
//         0, 1, 0, 0, 0, 0, 
//         0, 0, 0, 1, 0, 0, 
//         0, 0, 0, 0, 1, 0,
//         0, 0, 0, 0, 0, 0;

//   Eigen::MatrixXd T13 = T23 * T12;
//   std::vector<std::vector<std::shared_ptr<Eigen::MatrixXd>>> model_trans = {
//       {std::make_shared<Eigen::MatrixXd>(Eigen::MatrixXd::Identity(
//            models[0]->A_.rows(), models[0]->A_.cols())),
//        std::make_shared<Eigen::MatrixXd>(T13.transpose())},
//       {std::make_shared<Eigen::MatrixXd>(T13),
//        std::make_shared<Eigen::MatrixXd>(Eigen::MatrixXd::Identity(
//            models[1]->A_.rows(), models[1]->A_.cols()))}};
//   Imm imm(models, model_trans, P_trans, U_prob);
//   std::string filename =
//       "/apollo/modules/omnisense/interacting_multiple_models/py_implement/"
//       "z_noise.csv";
//   std::vector<DataTuple> data = readCSV(filename);
//   AERROR << "[CVAT_Test] data size: " << data.size();

//   imm.models_[0]->X_ << std::get<0>(data[0]), 0, std::get<2>(data[0]), 0;
//   imm.models_[1]->X_ << std::get<0>(data[0]), 0, std::get<2>(data[0]), 0, 0;
//   std::vector<Eigen::MatrixXd> vec_prob;
//   std::vector<Eigen::MatrixXd> vec_z_filt;
//   for (auto const &d : data) {
//     Eigen::VectorXd Z(2);
//     Z << std::get<0>(d), std::get<2>(d);
//     auto U_prob = imm.filt(Z);
//     for (int i = 0; i < U_prob.rows(); ++i) {
//       // AERROR << "U_prob[" << i << "]: " << U_prob(i, 0);
//     }
//     vec_prob.push_back(U_prob);

//     Eigen::VectorXd x = Eigen::VectorXd::Zero(imm.models_[0]->X_.rows());
//     for (size_t i = 0; i < imm.models_.size(); ++i) {
//       x += (*imm.model_trans_[0][i]) * imm.models_[i]->X_ * U_prob(i, 0);
//     }
//     vec_z_filt.push_back(x);
//   }
//   AINFO << "[CVT_Test] vec_prob size: " << vec_prob.size();
//   std::string filter_file =
//       "/apollo/modules/omnisense/interacting_multiple_models/"
//       "z_filt.csv";
//   std::string u_prob_file =
//       "/apollo/modules/omnisense/interacting_multiple_models/"
//       "u_prob.csv";
//   std::vector<std::string> filter_headers = {"x", "vx", "y", "vy"};
//   std::vector<std::string> headers = {"cv", "ct"};
//   writeCSV(filter_file, vec_z_filt, headers);
//   writeCSV(u_prob_file, vec_prob, headers);
// }

TEST_F(KFTest, CVAT_Test) {
  Eigen::MatrixXd P_trans(3, 3);
  P_trans << 0.98, 0.01, 0.01, 
             0.01, 0.98, 0.01, 
             0.01, 0.01, 0.98;

  Eigen::VectorXd U_prob(3);
  U_prob << 0.8, 
            0.1, 
            0.1;

  std::vector<std::shared_ptr<KF>> models = {cv_, ca_, ct_};
  Eigen::MatrixXd r(2, 1);
  r << 5.0, 
       5.0;

  // braodcast r to all models
  for (auto &model : models) {
    ACHECK(model->R_.rows() == r.rows());
    Eigen::MatrixXd R = model->R_;
    for (int i = 0; i < R.rows(); ++i) {
      model->R_.row(i) = R.row(i) * r(i);
    }
  }

  Eigen::MatrixXd T12(6, 4);
  T12 << 1, 0, 0, 0, 
         0, 1, 0, 0, 
         0, 0, 0, 0, 
         0, 0, 1, 0, 
         0, 0, 0, 1, 
         0, 0, 0, 0;

  Eigen::MatrixXd T23(5, 6);
  T23 << 1, 0, 0, 0, 0, 0, 
         0, 1, 0, 0, 0, 0, 
         0, 0, 0, 1, 0, 0, 
         0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 0;

  // [np.eye(models[0].A.shape[0]), T12.T, np.dot(T12.T, T23.T)],
  // [T12, np.eye(models[1].A.shape[0]), T23.T],
  // [np.dot(T23, T12), T23, np.eye(models[2].A.shape[0])]
  std::vector<std::vector<std::shared_ptr<Eigen::MatrixXd>>> model_trans;
  model_trans = {
      {std::make_shared<Eigen::MatrixXd>(Eigen::MatrixXd::Identity(
           models[0]->A_.rows(), models[0]->A_.cols())),
       std::make_shared<Eigen::MatrixXd>(T12.transpose()),
       std::make_shared<Eigen::MatrixXd>(T12.transpose() * T23.transpose())},
      {std::make_shared<Eigen::MatrixXd>(T12),
       std::make_shared<Eigen::MatrixXd>(Eigen::MatrixXd::Identity(
           models[1]->A_.rows(), models[1]->A_.cols())),
       std::make_shared<Eigen::MatrixXd>(T23.transpose())},
      {std::make_shared<Eigen::MatrixXd>(T23 * T12),
       std::make_shared<Eigen::MatrixXd>(T23),
       std::make_shared<Eigen::MatrixXd>(Eigen::MatrixXd::Identity(
           models[2]->A_.rows(), models[2]->A_.cols()))}};
  Imm imm(models, model_trans, P_trans, U_prob);

  std::string filename = conf_.noise_file();
  std::vector<DataTuple> data = readCSV(filename);
  AERROR << "[CVAT_Test] data size: " << data.size();

  imm.models_[0]->X_ << std::get<0>(data[0]), 0, std::get<2>(data[0]), 0;
  imm.models_[1]->X_ << std::get<0>(data[0]), 0, 0, std::get<2>(data[0]), 0, 0;
  imm.models_[2]->X_ << std::get<0>(data[0]), 0, std::get<2>(data[0]), 0, 0;
  std::vector<Eigen::MatrixXd> vec_prob;
  std::vector<Eigen::MatrixXd> vec_z_filt;
  for (auto const &d : data) {
    Eigen::VectorXd Z(2);
    Z << std::get<0>(d), std::get<2>(d);
    auto U_prob = imm.filt(Z);
    for (int i = 0; i < U_prob.rows(); ++i) {
      AERROR << "U_prob[" << i << "]: " << U_prob(i, 0);
    }
    vec_prob.push_back(U_prob);

    Eigen::VectorXd x = Eigen::VectorXd::Zero(imm.models_[0]->X_.rows());
    for (size_t i = 0; i < imm.models_.size(); ++i) {
      x += (*imm.model_trans_[0][i]) * imm.models_[i]->X_ * U_prob(i, 0);
    }
    vec_z_filt.push_back(x);
  }

  AINFO << "[CVAT_Test] vec_prob size: " << vec_prob.size();
  std::string filter_file = conf_.filter_file();
  std::string u_prob_file = conf_.u_prob_file();

  std::vector<std::string> filter_headers = {"x", "vx", "y", "vy"};
  std::vector<std::string> prob_headers = {"cv", "ca", "ct"};
  writeCSV(filter_file, vec_z_filt, filter_headers);
  writeCSV(u_prob_file, vec_prob, prob_headers);
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

}  // namespace seyond
}  // namespace apollo