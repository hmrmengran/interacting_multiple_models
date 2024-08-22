#include <gtest/gtest.h>

#include "immkf.h"
#include "kalmanfilter.h"

TEST(KalmanFilterTest, CVModel) {
    CVModel cv_model;
    Eigen::VectorXd Z(2);
    Z << 1, 1;
    cv_model.filt(Z);
    ASSERT_NEAR(cv_model.X(0), 1, 1e-5);
    ASSERT_NEAR(cv_model.X(1), 1, 1e-5);
}

TEST(KalmanFilterTest, CAModel) {
    CAModel ca_model;
    Eigen::VectorXd Z(2);
    Z << 1, 1;
    ca_model.filt(Z);
    ASSERT_NEAR(ca_model.X(0), 1, 1e-5);
    ASSERT_NEAR(ca_model.X(1), 1, 1e-5);
}

TEST(KalmanFilterTest, CTModel) {
    CTModel ct_model(0.1);
    Eigen::VectorXd Z(2);
    Z << 1, 1;
    ct_model.filt(Z);
    ASSERT_NEAR(ct_model.X(0), 1, 1e-5);
    ASSERT_NEAR(ct_model.X(1), 1, 1e-5);
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
