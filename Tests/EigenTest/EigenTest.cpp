#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/src/Core/util/Constants.h>
#include <cassert>
#include <iostream>

#include <Utilities/EigenHelper/Calculations.hpp>


#include <chrono>
#include <omp.h>

#define START_TIMER(NAME) auto NAME = std::chrono::high_resolution_clock::now()

#define STOP_TIMER(NAME)                                                                                               \
	auto NAME##_TIME_LENGTH = std::chrono::high_resolution_clock::now() - NAME;                                        \
	std::cout << "Timer [" << #NAME << "] use "                                                                        \
			  << std::chrono::duration_cast<std::chrono::microseconds>(NAME##_TIME_LENGTH).count() << " microseconds." \
			  << std::endl

#define __DEBUG_SLEEP_MS__(DURATION) std::this_thread::sleep_for(std::chrono::milliseconds(DURATION))



using namespace Eigen;
using namespace std;

void Test_VectorListDotProduct()
{
	constexpr int VEC_DIM		  = 5;
	constexpr int VEC_NUM		  = 10;
	constexpr int VEC_DYNAMIC_LEN = VEC_NUM == -1 ? 10 : VEC_NUM;

	std::cout << "Test_VectorListDotProduct() START" << std::endl;
	Matrix<float, VEC_NUM, VEC_DIM> VectorList			 = Matrix<float, VEC_NUM, VEC_DIM>::Random(VEC_DYNAMIC_LEN, VEC_DIM);
	Matrix<float, VEC_DIM, 1> DotVector					 = Matrix<float, VEC_DIM, 1>::Random(VEC_DIM, 1);

	Eigen::Matrix<float, VEC_NUM, 1>&& Result			 = EigenHelper::VectorListDotProduct(VectorList, DotVector);

	Eigen::Matrix<float, VEC_NUM, 1>&& ResultGroundTruth = Eigen::Matrix<float, VEC_NUM, 1>::Zero();
	for ( int i = 0; i < VectorList.rows(); i++ )
	{
		auto VecA			 = VectorList.row(i);
		ResultGroundTruth(i) = VecA.dot(DotVector);
	}

	const bool bTestResult = ResultGroundTruth.isApprox(Result, 1e-7);
	if ( !bTestResult )
	{
		std::cout << "Test_VectorListDotProduct() Test failed:" << std::endl;
		std::cout << "GroundTruth: \t" << ResultGroundTruth.transpose() << std::endl;
		std::cout << "Result: \t" << Result.transpose() << std::endl;
		assert(bTestResult);
	}
	std::cout << "Test_VectorListDotProduct() SUCCEED!" << std::endl;
}

void Test_X_Matrix()
{
	Matrix<double, -1, -1> DMat;
	DMat.resize(99, 33);
	DMat.setOnes();
	DMat *= 3;
	DMat.row(1).array() += 3;
	std::cout << DMat.row(1) << std::endl;
	std::cout << DMat.row(1).minCoeff() << std::endl;
	DMat.resize(NoChange_t::NoChange, 39);
	std::cout << DMat.row(1) << std::endl;
	// DMat.row(1).ar
}

void Test_OMP()
{
	Matrix<double, -1, -1> DMat;
	DMat.resize(150*149/2+1, 7);
	DMat.setOnes();

	Eigen::Matrix<double, 7, 1> Weight;
	Weight << 1, 2, 3, 4, 5, 6, 0;

	START_TIMER(Timer_OMP);
	// #pragma omp parallel for
	for ( int i = 0; i < DMat.rows(); i++ )
	{
		DMat.row(i)(6) = DMat.row(i) * Weight;
	}
	STOP_TIMER(Timer_OMP);

	START_TIMER(Timer_VEC);
	for ( int i = 0; i < DMat.cols(); i++ )
	{
		DMat.col(6).array() += DMat.col(i).array() * Weight(i);
	}
	STOP_TIMER(Timer_VEC);
	// std::cout << DMat << std::endl;

}

int Test_33() {
    // Method 1: Using dynamic vector
    Eigen::VectorXd col_vec(3);
    col_vec << 1, 2, 3;
    
    // Copy the column vector 4 times to create a matrix
    Eigen::MatrixXd result1 = col_vec.replicate(1, 4);
    
    // Method 2: Using fixed size vector
    Eigen::Vector3d fixed_vec;
    fixed_vec << 4, 5, 6;
    Eigen::Matrix3d result2;
    result2 = fixed_vec.replicate(1, 3);
    
    std::cout << "Original vector:\n" << col_vec << "\n\n";
    std::cout << "Replicated matrix (dynamic):\n" << result1 << "\n\n";
    std::cout << "Replicated matrix (fixed):\n" << result2 << "\n";
    
    return 0;
}

void Test_22()
{
	using MatType = Matrix<double, 2, 3, RowMajor>;
	using MappedType = Matrix<double, MatType::SizeAtCompileTime, 1>;
	MatType A;
	A << 1, 2, 3, 4, 5, 6;
	std::cout << "A: " << A << std::endl;

	Map<MappedType> MappedA(A.data());

	constexpr int EXTEND_ROW = 10;
	using ExtendMatType = Matrix<double, MatType::SizeAtCompileTime * EXTEND_ROW, 1>;
	using ExtendMatType2 = Matrix<double, MatType::SizeAtCompileTime, EXTEND_ROW>;
	ExtendMatType ExtendMat = MappedA.replicate(EXTEND_ROW, 1);
	ExtendMatType2 ExtendMat2 = MappedA.replicate(1, EXTEND_ROW);
	
	std::cout << "ROWS: " << ExtendMat.rows() << "; COLS: " << ExtendMat.cols() << std::endl;
	std::cout << ExtendMat << std::endl << std::endl << std::endl;
	
	std::cout << "ROWS: " << ExtendMat2.rows() << "; COLS: " << ExtendMat2.cols() << std::endl;
	std::cout << ExtendMat2 << std::endl;
}

int main(int argc, char* argv[])
{
	// Test_VectorListDotProduct();
	// Test_X_Matrix();
	// Test_OMP();
	Test_22();
	// Test_33();
	return 0;
}