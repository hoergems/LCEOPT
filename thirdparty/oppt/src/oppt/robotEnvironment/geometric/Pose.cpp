#include "oppt/opptCore/geometric/Pose.hpp"

namespace oppt {
namespace geometric {
Pose::Pose():
	position(0.0, 0.0, 0.0),
	orientation(1.0, 0.0, 0.0, 0.0) {
}

Pose::Pose(const Vector3f &position_, const Quaternionf &orientation_):
	position(position_),
	orientation(orientation_) {

}

Pose::Pose(const FloatType &x, const FloatType &y, const FloatType &z, const FloatType &oX, const FloatType &oY, const FloatType &oZ):
	position(x, y, z),
	orientation(oppt::math::eulerAnglesToQuaternion(oX, oY, oZ)) {

}

Pose::Pose(const Pose &other):
	position(other.position),
	orientation(other.orientation) {
}

Pose::Pose(const Vector3f &position_, const Vector3f &orientation_):
	position(position_),
	orientation(oppt::math::eulerAnglesToQuaternion(orientation_.x(),
	            orientation_.y(),
	            orientation_.z())) {
}

#ifdef GZ_SUPPORT
Pose::Pose(const oppt::GZPose &gzPose):
#ifdef GZ_GT_7
	position(gzPose.Pos().X(), gzPose.Pos().Y(), gzPose.Pos().Z()),
	orientation(gzPose.Rot().W(), gzPose.Rot().X(), gzPose.Rot().Y(), gzPose.Rot().Z()) {
#else
	position(gzPose.pos.x, gzPose.pos.y, gzPose.pos.z),
	orientation(gzPose.rot.w, gzPose.rot.x, gzPose.rot.y, gzPose.rot.z) {
#endif
}

GZPose Pose::toGZPose() const {
#ifdef GZ_GT_7
	return GZPose(position.x(), position.y(), position.z(), orientation.w(), orientation.x(), orientation.y(), orientation.z());
#else
	return GZPose(GZVector3(position.x(), position.y(), position.z()), GZQuaternion(orientation.w(), orientation.x(), orientation.y(), orientation.z()));
#endif
}
#endif

Pose &Pose::operator=(const Pose &other) {
	if (&other == this) {
		return *this;
	}

	position = other.position;
	orientation = other.orientation;

	return *this;
}

Pose Pose::operator+(const Pose &other) const {
#ifdef GZ_SUPPORT
	return Pose(toGZPose() + other.toGZPose());
#else
	Pose newPose;
	Quaternionf tmp(0.0, position.x(), position.y(), position.z());
	tmp = other.orientation * (tmp * other.orientation.inverse());
	newPose.position.x() = other.position.x() + tmp.x();
	newPose.position.y() = other.position.y() + tmp.y();
	newPose.position.z() = other.position.z() + tmp.z();
	newPose.orientation = other.orientation * orientation;
	return newPose;
#endif
}

Pose Pose::operator-(const Pose &other) const {
#ifdef GZ_SUPPORT
	return Pose(toGZPose() - other.toGZPose());
#else
	Pose newPose;
	Quaternionf tmp(0.0,
	                position.x() - other.position.x(),
	                position.y() - other.position.y(),
	                position.z() - other.position.z());
	tmp = other.orientation.inverse() * (tmp * other.orientation);
	newPose.position.x() = tmp.x();
	newPose.position.y() = tmp.y();
	newPose.position.z() = tmp.z();
	newPose.orientation = (other.orientation.inverse() * orientation);
	newPose.orientation.normalize();
	return newPose;
#endif
}

bool Pose::operator==(const Pose &other) const {
	return equal_(position.x(), other.position.x()) &&
	       equal_(position.y(), other.position.y()) &&
	       equal_(position.z(), other.position.z()) &&
	       equal_(orientation.w(), other.orientation.w()) &&
	       equal_(orientation.x(), other.orientation.x()) &&
	       equal_(orientation.y(), other.orientation.y()) &&
	       equal_(orientation.z(), other.orientation.z());
}

bool Pose::operator!=(const Pose &other) const {
	return !equal_(position.x(), other.position.x()) ||
	       !equal_(position.y(), other.position.y()) ||
	       !equal_(position.z(), other.position.z()) ||
	       !equal_(orientation.w(), other.orientation.w()) ||
	       !equal_(orientation.x(), other.orientation.x()) ||
	       !equal_(orientation.y(), other.orientation.y()) ||
	       !equal_(orientation.z(), other.orientation.z());
}

VectorFloat Pose::toStdVecEuler() const {
	Vector3f euler = oppt::math::quaternionToEulerAngles(orientation);
	VectorFloat vec({position.x(), position.y(), position.z(), euler.x(), euler.y(), euler.z()});
	return vec;
}

bool Pose::equal_(const FloatType &val1, const FloatType &val2) const {
	return std::fabs(val1 - val2) <= epsilon_;
}

}
}