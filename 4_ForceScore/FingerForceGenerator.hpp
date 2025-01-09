#ifndef A879F1EF_340C_44BA_908D_C2BC726C884F
#define A879F1EF_340C_44BA_908D_C2BC726C884F

#include "DataTypes/PointSetData.hpp"
#include "GlobalTypes.hpp"
#include "GlobalBaseTypes.hpp"
#include "4_ForceScore/FingerForceBalancer.hpp"
#include <cassert>
#include <cmath>
#include <cstddef>
#include <nlohmann/json.hpp>
#include <random>
#include <memory>


/**
 * @brief A class for generating finger forces that balance with the input [InKnifeForce (also called ExternalForce)].
 * Output these finger forces in a std::vector for further force score calculation
 *
 */
template<typename Scalar, int ForceCount = 2>
class TFingerForceGenerator : public std::enable_shared_from_this<TFingerForceGenerator<Scalar, ForceCount>>
{
protected:
	using json = nlohmann::json;

	using GeneratorPointSetData = TSearchSpaceElement<ForceCount>;
	using GeneratorPointSetDataPtr = std::shared_ptr<GeneratorPointSetData>;
	using GeneratorPointSetDataConstPtr = std::shared_ptr<const GeneratorPointSetData>;

public:
	using ForcePairType      = Eigen::Matrix<Scalar, ForceCount, 3, Eigen::RowMajor>;
	using ForcePairAllocType = Eigen::aligned_allocator<ForcePairType>;
	using ForcePairListType  = std::vector<ForcePairType, ForcePairAllocType>;
protected:
	using BalancerType       = TFingerForceBalancer<Scalar, ForceCount>;
	// using NormalType         = decltype(Types::CalcPointSetData::NormalPair);
	using NormalType         = Eigen::Matrix<Scalar, ForceCount, 3, Eigen::RowMajor>;


public:
	constexpr static int FORCE_COUNT = ForceCount;
	friend class FingerForceGeneratorTest;
	friend class FingerForceGeneratorTest_ParamSetTest_Test;
	friend class FingerForceGeneratorTest_GenerateRandomVectorTest_Test;

	TFingerForceGenerator() = delete;

	TFingerForceGenerator(Types::ForceTorqueType InKnifeForce,
		Types::Vec3 InCoM,
		GeneratorPointSetDataConstPtr InPointSetDataPtr,
		size_t InFingerForceGenCount,
		const json& InJson)
		: KnifeForce(InKnifeForce), CenterOfMass(InCoM), PointSetDataPtr(InPointSetDataPtr),
		  FingerForceGenCount(InFingerForceGenCount)
	{
		GeneratingRetryTimes = InJson["CalForceScore"]["ForceGenBasicParam"]["GeneratingRetryTimes"];
		AngleLimit           = InJson["CalForceScore"]["ForceGenBasicParam"]["AngleLimit"].get<double>() * M_PI / 180.0;

		ForcePairList.reserve(FingerForceGenCount);
		ForcePairList.assign(FingerForceGenCount, ForcePairType::Zero());

		assert(PointSetDataPtr != nullptr);
		FingerForceBalancer.MakeGMat(PointSetDataPtr->PositionPair);

		SetRandomGeneratorSeed(); // Set seed as a random seed.
	}

	// Set seed for random number generator
	void SetRandomGeneratorSeed(std::random_device::result_type InSeed = 0, bool bInUseRandomSeed = true)
	{
		if ( bInUseRandomSeed )
		{
			bUseFixedSeed = false;
			RandGenerator.seed(RandomDev());
		}
		else
		{
			bUseFixedSeed = true;
			RandGenerator.seed(InSeed);
		}
	}

	ForcePairListType& GetGeneratedFingerForceList() { return ForcePairList; }

	size_t GenerateFingerForceList()
	{
		size_t SuccessCount = 0;

		// #pragma omp parallel for reduction(+:SuccessCount)
		for ( size_t i = 0; i < FingerForceGenCount; i++ )
		{
			int RetryTimes = GeneratingRetryTimes;
			do
			{
				ForcePairType& ForcePair = ForcePairList[i];
				GenereateSingleForcePair(ForcePair);  // This function is implemented in the sub-class.

				Eigen::Map<typename BalancerType::ForceListType> FingerForceList(ForcePair.data());
				bool bGenSuccess = FingerForceBalancer.MakeForceBalanced(FingerForceList);

				// Check if the generated finger force is valid
				if ( !IsValidFingerForce(ForcePair, PointSetDataPtr->NormalPair) )
				{
					// If invalid, retry generating.
					continue;
				}
				if ( bGenSuccess )
				{
					// ONLY count the valid and balanced finger force.
					SuccessCount++;
					break;
				}
			}
			while ( RetryTimes-- );

			if ( RetryTimes == 0 )
			{
				// If the retry times is exhausted, set the force pair to infinity.
				ForcePairList[i].setConstant(Eigen::Infinity);
			}
		}

		return SuccessCount;
	}

	// Getters
	size_t GetFingerForceGenCount() const { return FingerForceGenCount; }
	int GetGeneratingRetryTimes() const { return GeneratingRetryTimes; }

protected:
	virtual bool IsValidFingerForce(const ForcePairType& InFingerForce, const NormalType& InPointSetNormal) const
	{
		// Both [InFingerForce] and [InPointSetNormal} are (n, 3) matrix where n is the number of fingers, each row is a
		// finger force or normal vector. Check if the force vector and the normal vector is within the angle limit.
		for ( int i = 0; i < FORCE_COUNT; i++ )
		{
			const auto& InForce = InFingerForce.row(i);
			const auto& Normal  = InPointSetNormal.row(i);

			const double ForceLength = InForce.norm();


			// Skip zero force vectors
			if ( ForceLength < 1e-6 )
			{
				continue;
			}

			// Check if the force vector is within the specified range
			if (ForceLength < ForceMin || ForceLength > ForceMax)
			{
				return false;
			}

			// Calculate angle between force and normal
			const auto ForceDir = InForce / ForceLength;
			const double Angle = std::acos(ForceDir.dot(Normal));

			// Check if angle exceeds limit
			if ( Angle > AngleLimit )
			{
				return false;
			}
		}
		return true;
	}

	virtual void GenereateSingleForcePair(ForcePairType& InOutForcePair) = 0;

protected:
	Types::ForceTorqueType KnifeForce;
	Types::Vec3 CenterOfMass;
	GeneratorPointSetDataConstPtr PointSetDataPtr;
	ForcePairListType ForcePairList;
	size_t FingerForceGenCount;
	int GeneratingRetryTimes;

	double AngleLimit;
	bool bUseFixedSeed = false;
	/// The length of the generated force vector is limited to [ForceMin, ForceMax] default is [0, INFINITY]
	/// you can override this value in the sub-class.
	double ForceMin = 0;
	/// The length of the generated force vector is limited to [ForceMin, ForceMax] default is [0, INFINITY], 
	/// you can override this value in the sub-class.
	double ForceMax = INFINITY;

	// Random seed generator
	std::random_device RandomDev;
	// Random number generator
	std::mt19937 RandGenerator;

	BalancerType FingerForceBalancer{ CenterOfMass, KnifeForce };
};
template<typename Scalar = double, int ForceCount = 2>
class TFingerForceGeneratorWithinCone : public TFingerForceGenerator<Scalar, ForceCount>
{
	DEF_MATRIX_TYPES_ALIASES(Scalar);
public:
	using Super = TFingerForceGenerator<Scalar, ForceCount>;

	friend class FingerForceGeneratorTest;
	friend class FingerForceGeneratorTest_ParamSetTest_Test;
	friend class FingerForceGeneratorTest_GenerateRandomVectorTest_Test;

	TFingerForceGeneratorWithinCone() = delete;

	TFingerForceGeneratorWithinCone(Types::ForceTorqueType InKnifeForce,
		Types::Vec3 InCoM,
		typename Super::GeneratorPointSetDataConstPtr InPointSetDataPtr,
		size_t InFingerForceGenCount,
		const typename Super::json& InJson)
		: Super(InKnifeForce, InCoM, InPointSetDataPtr, InFingerForceGenCount, InJson)
	{
		// Do nothing
		const typename Super::json& GeneratorJsonObj = InJson["CalForceScore"]["FingerForceGeneratorWithinCone_Param"];
		const double ForceRangeMin   = GeneratorJsonObj["ForceRangeMin"];
		const double ForceRangeMax   = GeneratorJsonObj["ForceRangeMax"];
		const double RandomHalfAngle = (GeneratorJsonObj["RandomHalfAngle"].template get<double>()) * M_PI / 180.0;
		this->AngleLimit = std::min(this->AngleLimit, RandomHalfAngle);

		this->ForceMin = ForceRangeMin;
		this->ForceMax = ForceRangeMax;

		this->LengthDist  = std::uniform_real_distribution<>{ ForceRangeMin, ForceRangeMax };
		this->AngleDist   = std::uniform_real_distribution<>{ 0, RandomHalfAngle };
		this->AzimuthDist = std::uniform_real_distribution<>{ 0, 2 * M_PI };

		// for (int i = 0; i < Types::CalcPointSetData::FORCE_COUNT; i++) {
		// 	const auto NormalVec = this->PointSetDataPtr->NormalPair.row(i);
		// 	CachedQuaternions[i] = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), NormalVec.normalized());
		// }
		CachedRotationMatrices.assign(ForceCount, Eigen::Matrix<Scalar, 3, 3>::Identity());
		for ( int i = 0; i < ForceCount; i++ )
		{
			auto& NormalVec      = this->PointSetDataPtr->NormalPair.row(i);
			Vec3 zAxis     = Vec3::UnitZ();
			Vec3 axis      = zAxis.cross(NormalVec.normalized());
			double angle              = acos(zAxis.dot(NormalVec));
			CachedRotationMatrices[i] = Eigen::AngleAxis<Scalar>(angle, axis).toRotationMatrix();
		}
	}

protected:
	virtual void GenereateSingleForcePair(typename Super::ForcePairType& InOutForcePair) override
	{
		for ( int i = 0; i < Types::CalcPointSetData::FORCE_COUNT; i++ )
		{
			const auto NormalVec = this->PointSetDataPtr->NormalPair.row(i);

			// Generate random length
			const double Length = this->LengthDist(this->RandGenerator);

			// Generate random direction within cone
			const double Theta = this->AngleDist(this->RandGenerator);
			const double Phi   = this->AzimuthDist(this->RandGenerator);

			// Convert spherical coordinates to Cartesian coordinates
			// Generate a random direction within a cone that direction is (0, 0, 1)
			// Types::Vec3 RandomDir;
			// RandomDir.x() = sin(Theta) * cos(Phi);
			// RandomDir.y() = sin(Theta) * sin(Phi);
			// RandomDir.z() = cos(Theta);

			InOutForcePair.row(i) << sin(Theta) * cos(Phi), sin(Theta) * sin(Phi), cos(Theta);
			// InOutForcePair.row(i).x() = sin(Theta) * cos(Phi);
			// InOutForcePair.row(i).y() = sin(Theta) * sin(Phi);
			// InOutForcePair.row(i).z() = cos(Theta);

			// Rotate RandomDir to align with InNormalVec
			// Rotate above vector (cone direction of generating this vector) to align with InNormalVec
			// Eigen::Quaterniond Q = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), NormalVec.normalized());
			// const Eigen::Vector3d RotatedDir = CachedRotationMatrices[i] * RandomDir;

			// InOutForcePair.row(i)

			// Scale by length
			// InOutForcePair.row(i) = Length * (CachedRotationMatrices[i] * RandomDir);
			const Vec3 RotatedDir = CachedRotationMatrices[i] * InOutForcePair.row(i).transpose();
			InOutForcePair.row(i) = Length * RotatedDir;
			// InOutForcePair.row(i) = Length * (CachedRotationMatrices[i] * InOutForcePair.row(i).transpose());
			// InOutForcePair.row(i) = Length * (InOutForcePair.row(i) * (CachedRotationMatrices[i].transpose()));

			// const Eigen::Vector3d rotatedVector = InOutForcePair.row(i) * CachedRotationMatrices[i].transpose();
			// const Eigen::Vector3d rotatedVector = InOutForcePair.row(i) * CachedRotationMatrices[i];
			// InOutForcePair.row(i) = Length * rotatedVector;
		}
	}

	// 生成 n 个圆锥范围内的随机向量，存储在 n x 3 的矩阵中
	Eigen::MatrixXd random_vectors_in_cone(const Eigen::Vector3d& axis, double theta, double R, int n)
	{
		// 随机数生成器
		static std::random_device rd;
		static std::mt19937 gen(rd());
		static std::uniform_real_distribution<> dis(0.0, 1.0);

		// 1. 将轴线向量归一化
		Eigen::Vector3d d = axis.normalized();

		// 2. 生成随机极角 phi，范围 [0, theta]
		Eigen::VectorXd phi = (Eigen::VectorXd::NullaryExpr(n,
			[&]()
			{
				return std::acos(1 - dis(gen) * (1 - std::cos(theta)));
			}));

		// 3. 生成随机方位角 psi，范围 [0, 2pi]
		Eigen::VectorXd psi = (Eigen::VectorXd::NullaryExpr(n,
			[&]()
			{
				return 2 * M_PI * dis(gen);
			}));

		// 4. 生成随机长度 r，范围 [0, R]
		Eigen::VectorXd r = (Eigen::VectorXd::NullaryExpr(n,
			[&]()
			{
				return R * dis(gen);
			}));

		// 5. 将球坐标转换为笛卡尔坐标
		Eigen::MatrixXd vectors(n, 3);
		vectors.col(0) = r.array() * phi.array().sin() * psi.array().cos(); // x
		vectors.col(1) = r.array() * phi.array().sin() * psi.array().sin(); // y
		vectors.col(2) = r.array() * phi.array().cos();                     // z

		// 6. 构造一个与轴线对齐的旋转矩阵
		Eigen::Vector3d z_axis(0, 0, 1);
		if ( !d.isApprox(z_axis) && !d.isApprox(-z_axis) )
		{
			// 计算旋转矩阵
			Eigen::Vector3d rotation_axis = z_axis.cross(d);
			rotation_axis.normalize();
			double rotation_angle = std::acos(z_axis.dot(d));

			// 使用 Rodrigues 旋转公式
			Eigen::Matrix3d K;
			K << 0, -rotation_axis.z(), rotation_axis.y(), rotation_axis.z(), 0, -rotation_axis.x(), -rotation_axis.y(),
				rotation_axis.x(), 0;
			Eigen::Matrix3d rotation_matrix =
				Eigen::Matrix3d::Identity() + std::sin(rotation_angle) * K + (1 - std::cos(rotation_angle)) * K * K;

			// 对所有向量应用旋转
			vectors = (rotation_matrix * vectors.transpose()).transpose();
		}
		else if ( d.isApprox(-z_axis) )
		{
			// 如果轴线是负 z 轴，只需翻转 z 坐标
			vectors.col(2) *= -1;
		}

		return vectors;
	}

protected:
	std::uniform_real_distribution<> LengthDist;
	std::uniform_real_distribution<> AngleDist;
	std::uniform_real_distribution<> AzimuthDist;
	// std::vector<Eigen::Quaterniond> CachedQuaternions;
	std::vector<Eigen::Matrix<Scalar, 3, 3>, Eigen::aligned_allocator<Eigen::Matrix<Scalar, 3, 3>>> CachedRotationMatrices;
};

template<typename Scalar = double, int ForceCount = 2>
class TFingerForceGeneratorByRandomK : public TFingerForceGenerator<Scalar, ForceCount>
{
public:
	using Super = TFingerForceGenerator<Scalar, ForceCount>;

	friend class FingerForceGeneratorTest;
	friend class FingerForceGeneratorTest_ParamSetTest_Test;
	friend class FingerForceGeneratorTest_GenerateRandomVectorTest_Test;

	TFingerForceGeneratorByRandomK() = delete;

	TFingerForceGeneratorByRandomK(Types::ForceTorqueType InKnifeForce,
		Types::Vec3 InCoM,
		typename Super::GeneratorPointSetDataConstPtr InPointSetDataPtr,
		size_t InFingerForceGenCount,
		const typename Super::json& InJson)
		: TFingerForceGenerator<Scalar, ForceCount>(InKnifeForce, InCoM, InPointSetDataPtr, InFingerForceGenCount, InJson)
	{
		const typename Super::json& GeneratorJsonObj = InJson["CalForceScore"]["FingerForceGeneratorByRandomK_Param"];
		const double ForceRangeMin   = GeneratorJsonObj["ForceRangeMin"];
		const double ForceRangeMax   = GeneratorJsonObj["ForceRangeMax"];

		this->ForceMin = ForceRangeMin;
		this->ForceMax = ForceRangeMax;

		this->LengthDist = std::uniform_real_distribution<>{ ForceRangeMin, ForceRangeMax };
	}

	virtual void GenereateSingleForcePair(typename Super::ForcePairType& InOutForcePair) override
	{
		typename Super::BalancerType::ForceListType RandForce = typename Super::BalancerType::ForceListType::Zero();
		if ( this->bUseFixedSeed )
		{
			// Insert fixed seed random number into [InOutForcePair]
			for ( int i = 0; i < InOutForcePair.size(); i++ )
			{
				RandForce(i) = this->LengthDist(this->RandGenerator);
			}
		}
		else
		{
			RandForce = typename Super::BalancerType::ForceListType::Random();
		}

		// Normalize the random force vector with each row.
		// And rescale the force vector with a length.
		for ( int i = 0; i < InOutForcePair.rows(); i++ )
		{
			auto ForceVec = RandForce.block(i * 3, 0, (i + 1) * 3, 1);
			ForceVec.normalized();
			ForceVec *= this->LengthDist(this->RandGenerator);
		}

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
		typename Super::BalancerType::ForceListType Force = this->FingerForceBalancer.CalculateFingerForeceByK(RandForce);
#pragma GCC diagnostic pop
#pragma clang diagnostic pop

		InOutForcePair = Eigen::Map<typename Super::ForcePairType>(Force.data());
	}

protected:
	std::uniform_real_distribution<> LengthDist;
};

#endif /* A879F1EF_340C_44BA_908D_C2BC726C884F */
