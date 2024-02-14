#ifndef _OPPT_POSE_HPP_
#define _OPPT_POSE_HPP_
#include "oppt/opptCore/typedefs.hpp"
#include "oppt/opptCore/MathUtils.hpp"

namespace oppt {
namespace geometric {
class Pose {
public:
	_MOVE(Pose)
	Pose();

	Pose(const Vector3f &position_, const Quaternionf &orientation_);

	Pose (const FloatType &x, const FloatType &y, const FloatType &z, const FloatType &oX, const FloatType &oY, const FloatType &oZ);

	Pose(const Pose &other);

#ifdef GZ_SUPPORT
	Pose (const oppt::GZPose &gzPose);

	GZPose toGZPose() const;
#endif

	Pose (const Vector3f &position_, const Vector3f &orientation_);

	Pose &operator=(const Pose &other);

	Pose operator+(const Pose &other) const;

	Pose operator-(const Pose &other) const;

	bool operator==(const Pose &other) const;

	bool operator!=(const Pose &other) const;

	VectorFloat toStdVecEuler() const;

	friend std::ostream& operator<< (std::ostream& out, const Pose& pose) {
		out << pose.position.x() << ", " << pose.position.y() << ", " << pose.position.z() << ", ";
		out << pose.orientation.w() << ", " << pose.orientation.x() << ", " << pose.orientation.y() << ", " << pose.orientation.z();
		return out;
	}

	Vector3f position;
	Quaternionf orientation;

private:
	const FloatType epsilon_ = 1e-8;

	bool equal_(const FloatType &val1, const FloatType &val2) const;
};

/** @brief A std::shared_ptr to oppt::geometric::Pose*/
typedef std::shared_ptr<geometric::Pose> PoseSharedPtr;
/** @brief A std::unique_ptr to oppt::geometric::Pose*/
typedef std::unique_ptr<geometric::Pose> PoseUniquePtr;
}
}
#endif
