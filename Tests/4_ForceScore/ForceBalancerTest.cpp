#include <gtest/gtest.h>
#include <iostream>

#include "4_ForceScore/FingerForceBalancer.hpp"
#include "Eigen/src/Core/util/Constants.h"

TEST(FingerForceBalancerTest, BasicConstruction)
{
	Eigen::Vector3d InCom(0, 0, 0);
	TFingerForceBalancer<double, 2> Balancer(InCom);

	EXPECT_TRUE(Balancer.ExternalForce.isZero());

	Eigen::Matrix<double, 6, 1> InExtForce;
	InExtForce << 1, 2, 3, 4, 5, 6;

	Balancer.SetExternalForce(InExtForce);
	EXPECT_EQ(Balancer.ExternalForce, InExtForce);
}

TEST(FingerForceBalancerTest, MakeForceBalanced)
{
	// 1. Prepare the Center of Mass and External Force which balanced by the finger force.
	Eigen::Vector3d InCom(0, 0, 0);
	Eigen::Matrix<double, 6, 1> InExtForce;
	InExtForce << 0, 0, -10, 0, 0, 0;

	// 2. Construct the finger force balancer.
	TFingerForceBalancer<double, 2> Balancer(InCom, InExtForce);

	// 3. Prepare the finger(grasp) points.
	Eigen::Matrix<double, 2, 3, Eigen::RowMajor> InPoints;
	InPoints << 1, 0, -2, -1, 0, -2;

	// 4. Generate the G(grasping) matrix.
	Balancer.MakeGMat(InPoints);

	// 5. Generate the random initial finger force for balancing(Force is NOT required to be balanced).
	Eigen::Matrix<double, 6, 1> InOutInitForce;
	InOutInitForce.setRandom();

	// 6. Use [MakeForceBalanced] Make the force balanced.
	bool bGenSuccess = Balancer.MakeForceBalanced(InOutInitForce);
	EXPECT_TRUE(bGenSuccess);

	// 7. Check the result is correct (Can balanced with [InExtForce]).
	Eigen::Matrix<double, 6, 1> NetForce = Balancer.GMat * InOutInitForce - InExtForce;
	bool bIsNetForceZero                 = NetForce.isZero(1e-6);
	EXPECT_TRUE(bIsNetForceZero);
}

TEST(FingerForceBalancerTest, CalculateFingerForce)
{
	// The OLD way to generate a balanced finger force.
	Eigen::Vector3d InCom(0, 0, 0);
	Eigen::Matrix<double, 6, 1> InExtForce;
	InExtForce << 1, 0, 0, 0, 0, 0;

	TFingerForceBalancer<double, 2> Balancer(InCom, InExtForce);

	Eigen::Matrix<double, 2, 3> InPoints;
	InPoints << 0, 1, 0, 0, -1, 0;

	Balancer.MakeGMat(InPoints);

	Eigen::Matrix<double, 6, 1> InK;
	InK.setZero();

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
	auto OutResult = Balancer.CalculateFingerForeceByK(InK);
#pragma clang diagnostic pop
#pragma GCC diagnostic pop

	Eigen::Matrix<double, 6, 1> NetForce = Balancer.GMat * OutResult - InExtForce;
	EXPECT_TRUE(NetForce.isZero(1e-6));
}

TEST(FingerForceBalancerTest, SVDMatrixInvert)
{
	Eigen::Matrix<double, 6, 9> InTestMat;
	InTestMat.setRandom();

	auto OutInvMat = CalcSVD_MatInv(InTestMat);

	auto Identity = InTestMat * OutInvMat;
	EXPECT_TRUE((Identity).isIdentity(1e-7));
}

int main(int argc, char** argv)
{
	testing::InitGoogleTest(&argc, argv);

	return RUN_ALL_TESTS();
}