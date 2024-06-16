#ifndef AF667CED_ABEF_4438_ACDC_BAB21EF30796
#define AF667CED_ABEF_4438_ACDC_BAB21EF30796

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h> // For type: [pcl::PointIndices]
#include <pcl/PolygonMesh.h>

namespace PCL_Helper
{

/**
 * @brief Define the prototype for checking if a member in a class.
 *
 */
#define DEFINE_HAS_MEMBER_CHECK_CLASS(MEMBER_NAME)                                             \
	template<typename T, typename = int>                                                       \
	struct ClassHasMember_##MEMBER_NAME : std::false_type                                      \
	{                                                                                          \
	};                                                                                         \
	template<typename T>                                                                       \
	struct ClassHasMember_##MEMBER_NAME<T, decltype((void)T::MEMBER_NAME, 0)> : std::true_type \
	{                                                                                          \
	}

#define CLASS_HAS_MEMBER(CHECKED_TYPE, MEMBER_NAME) ClassHasMember_##MEMBER_NAME<CHECKED_TYPE>::value

DEFINE_HAS_MEMBER_CHECK_CLASS(x);
DEFINE_HAS_MEMBER_CHECK_CLASS(y);
DEFINE_HAS_MEMBER_CHECK_CLASS(z);
DEFINE_HAS_MEMBER_CHECK_CLASS(r);
DEFINE_HAS_MEMBER_CHECK_CLASS(g);
DEFINE_HAS_MEMBER_CHECK_CLASS(b);
DEFINE_HAS_MEMBER_CHECK_CLASS(normal_x);
DEFINE_HAS_MEMBER_CHECK_CLASS(normal_y);
DEFINE_HAS_MEMBER_CHECK_CLASS(normal_z);
DEFINE_HAS_MEMBER_CHECK_CLASS(curvature);

template<typename CheckedType>
constexpr bool HAS_z_FIELD()
{
	return CLASS_HAS_MEMBER(CheckedType, z);
}

template<typename CheckedType>
constexpr bool HAS_RGB_FIELD()
{
	return CLASS_HAS_MEMBER(CheckedType, r) && CLASS_HAS_MEMBER(CheckedType, g) && CLASS_HAS_MEMBER(CheckedType, b);
}

template<typename CheckedType>
constexpr bool HAS_NORMAL_FIELD()
{
	return CLASS_HAS_MEMBER(CheckedType, normal_x) && CLASS_HAS_MEMBER(CheckedType, normal_y) && CLASS_HAS_MEMBER(CheckedType, normal_z)
		   && CLASS_HAS_MEMBER(CheckedType, curvature);
}

// #define HAS_RGB_FIELD(TYPENAME) ClassHasMember_r<TYPENAME>::value
#define HAS_RGB_FIELD(TYPENAME) ClassHasMember_r<TYPENAME>::value&& ClassHasMember_g<TYPENAME>::value&& ClassHasMember_b<TYPENAME>::value
#define HAS_NORMAL_FIELD(TYPENAME) \
	ClassHasMember_normal_x<TYPENAME>::value&& ClassHasMember_normal_y<TYPENAME>::value&& ClassHasMember_normal_z<TYPENAME>::value

} // namespace PCL_Helper

#endif /* AF667CED_ABEF_4438_ACDC_BAB21EF30796 */
