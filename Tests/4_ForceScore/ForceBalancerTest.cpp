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

TEST(FingerForceBalancerTest, GMatCalculation)
{
	Eigen::Matrix<double, 3, 1> InCoM;
	// Eigen::Matrix<double, 3, 1>::Zero();
	InCoM << 0.53020584583282471, -0.2218005508184433, 0.049539912492036819;
	Eigen::Matrix<double, 3, 3, Eigen::RowMajor> TestingPosition;
	// clang-format off
	TestingPosition << 	0.516943336, -0.238809869, 0.0564656109, 
						0.507865191, -0.220309898, 0.0582243688, 
						0.527730584, -0.221327618, 0.010671692;
	// clang-format on
	TFingerForceBalancer<double, 3> Balancer(InCoM);
	
	// Calculate the G matrix.
	Balancer.MakeGMat(TestingPosition);

	using _T = decltype(Balancer.GMat);
	using ExpMatType = Eigen::Matrix<_T::Scalar, _T::RowsAtCompileTime, _T::ColsAtCompileTime>;
	ExpMatType ExpectedGMat;
	// clang-format off
	const std::vector<double> ExpectedGMatData = {1, 0, 0, 0, 0.0069256983697414398, 0.017009317874908447, 0, 1, 0, -0.0069256983697414398, 0, -0.013262510299682617, 0, 0, 1, -0.017009317874908447, 0.013262510299682617, 0, 1, 0, 0, 0, 0.0086844563484191895, -0.0014906525611877441, 0, 1, 0, -0.0086844563484191895, 0, -0.022340655326843262, 0, 0, 1, 0.0014906525611877441, 0.022340655326843262, 0, 1, 0, 0, 0, -0.038868220522999763, -0.00047293305397033691, 0, 1, 0, 0.038868220522999763, 0, -0.0024752616882324219, 0, 0, 1, 0.00047293305397033691, 0.0024752616882324219, 0};
	// clang-format on
	for ( int i = 0; i < ExpectedGMat.size(); i++ )
	{
		ExpectedGMat(i) = ExpectedGMatData[i];
	}
	if ( !Balancer.GMat.isApprox(ExpectedGMat, 1e-6) )
	{
		std::cout << "GMat:\n" << Balancer.GMat << std::endl;
		std::cout << "ExpectedGMat:\n" << ExpectedGMat << std::endl;
	}
	EXPECT_TRUE(Balancer.GMat.isApprox(ExpectedGMat, 1e-6));

	using ExpInvMatType = Eigen::Matrix<_T::Scalar, _T::ColsAtCompileTime, _T::RowsAtCompileTime>;
	ExpInvMatType ExpectedGMatInv;
	// clang-format off
	const std::vector<double> ExpectedGMatInvData = { 0.14827624356793259, -0.09507888412268882, -0.084873794337833541, 0.63951560771652083, 0.12554805305322009, 0.13002238820255352, 0.21220814871554694, -0.03046916893053133, -0.045148593864719908, 0.61793615020199089, 0.63280700382529409, 0.26869084949833549, -0.55728444460984972, 0.10412911831867749, -0.23638020478365096, -0.06065170559214151, 0.26306387785602897, -0.032310644714684471, -0.096767243963134852, -0.038066204398149048, 0.29847274000715923, -0.14077640683959541, -0.057275921908455943, 0.27337414336955324, 0.23754365080273032, 0.095342126306604977, 0.42815311662328759, -12.004942741856976, -13.283368183215496, -11.320108176351052, 10.827919997648841, -3.9064019181356486, 7.9016614247476618, 1.1770227442081411, 17.189770101351158, 3.4184467516033932, 2.8803337043093284, -2.2495516238061719, -1.7263721595137564, 15.369416412494031, 2.9689537183776182, 7.8460134297483872, -18.249750116803355, -0.71940209457144455, -6.1196412702346299, 41.351411110465172, 15.480564764088406, 14.254494365663145, -37.291898802877633, -20.443806449285812, -13.796873716026457, -4.0595123075875463, 4.9632416851974011, -0.4576206496366898};
	// clang-format on
	for ( int i = 0; i < ExpectedGMatInv.size(); i++ )
	{
		ExpectedGMatInv(i) = ExpectedGMatInvData[i];
	}
	if ( !Balancer.GMatInv.isApprox(ExpectedGMatInv, 1e-6) )
	{
		std::cout << "ExpectedGMatInv:\n" << ExpectedGMatInv << std::endl;
		std::cout << "GMatInv:\n" << Balancer.GMatInv << std::endl;
	}
	EXPECT_TRUE(Balancer.GMatInv.isApprox(ExpectedGMatInv, 1e-6));
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