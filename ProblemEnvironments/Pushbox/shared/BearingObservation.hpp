#ifndef _BEARING_OBSERVATION_HPP_
#define _BEARING_OBSERVATION_HPP_
#include <oppt/robotHeaders/Observation.hpp>

namespace oppt {
class BearingObservation: public DiscreteVectorObservation {
public:
	BearingObservation(const VectorInt &theBearings, const bool &pushed):
		DiscreteVectorObservation(VectorFloat()),
		theBearings_(theBearings),
		pushed_(pushed) {

	}

	_NO_COPY_BUT_MOVE(BearingObservation)

	virtual bool equals(const Observation& otherObservation) const override {
		VectorInt otherBearings = static_cast<const BearingObservation *>(&otherObservation)->getBearings();
		bool otherPushed = static_cast<const BearingObservation *>(&otherObservation)->getPushed();
		for (size_t i = 0; i != theBearings_.size(); ++i) {
			if (theBearings_[i] != otherBearings[i])
				return false;
		}

		if (otherPushed != pushed_)
			return false;
		return true;
	}

	virtual FloatType distanceTo(const Observation& otherObservation) const {
		if (equals(otherObservation))
			return 0.0;
		return std::numeric_limits<FloatType>::infinity();
	}

	virtual void print(std::ostream& os) const override {
		os << "bearings: ";
		for (size_t i = 0; i != theBearings_.size(); ++i) {
			os << theBearings_[i] << " ";
		}
		os << ", pushed: " << pushed_;
	}

	VectorInt getBearings() const {
		return theBearings_;
	}

	bool getPushed() const {
		return pushed_;
	}

private:
	VectorInt theBearings_;
	bool pushed_ = false;
};

class PushboxPositionObservation: public DiscreteVectorObservation {
public:
	PushboxPositionObservation(const VectorFloat &observation, const bool &pushed):
		DiscreteVectorObservation(observation),
		pushed_(pushed) {

	}

	_NO_COPY_BUT_MOVE(PushboxPositionObservation)

	virtual ~PushboxPositionObservation() = default;

	bool getPushed() const {
		return pushed_;
	}

	virtual bool equals(const Observation& otherObservation) const override {
		bool otherPushed = static_cast<const BearingObservation *>(&otherObservation)->getPushed();
		if (otherPushed != pushed_)
			return false;
		return DiscreteVectorObservation::equals(otherObservation);
	}

	virtual FloatType distanceTo(const Observation& otherObservation) const {
		FloatType distance = 0.0;
		if (static_cast<const PushboxPositionObservation *>(&otherObservation)->getPushed() != pushed_) {
			distance = std::numeric_limits<FloatType>::infinity();
		} else {
			distance = math::euclideanDistance(static_cast<const PushboxPositionObservation *>(&otherObservation)->asVector(), observationVec_);
		}

		return distance;
	}

	/**virtual void print(std::ostream& os) const override {
		os << "bearings: ";
		for (size_t i = 0; i != theBearings_.size(); ++i) {
			os << theBearings_[i] << " ";
		}
		os << ", pushed: " << pushed_;
	}*/

private:
	bool pushed_ = false;

};

class ContinuousBearingObservation: public Observation {
public:
	ContinuousBearingObservation(const FloatType &bearing, const bool &pushed):
		Observation(),
		bearing_(bearing),
		pushed_(pushed) {

	}

	virtual ~ContinuousBearingObservation() = default;

	_NO_COPY_BUT_MOVE(ContinuousBearingObservation)

	const FloatType &bearing() const {
		return bearing_;
	}

	const bool &pushed() const {
		return pushed_;
	}

	virtual void serialize(std::ostream& os, const std::string& prefix = "") const override {
		if (prefix.empty() == false)
			os << prefix << ": ";
		print(os);
		os << " END ";
	}

	virtual void print(std::ostream& os) const override {
		os << "bearing: " << bearing_ << ", pushed: " << pushed_;
	}

	virtual oppt::ObservationUniquePtr copy() const override {
		return ObservationUniquePtr(new ContinuousBearingObservation(bearing_, pushed_));
	}

	virtual bool equals(const Observation& otherObservation) const override {
		if (static_cast<const ContinuousBearingObservation &>(otherObservation).pushed() != pushed_)
			return false;

		if (std::fabs(static_cast<const ContinuousBearingObservation &>(otherObservation).bearing() - bearing_) < 1e-18)
			return true;
		return false;
	}

	virtual std::size_t hash() const override {
		return 0;
	}

	virtual FloatType distanceTo(const Observation& otherObservation) const override {
		if (static_cast<const ContinuousBearingObservation &>(otherObservation).pushed() != pushed_)
			return std::numeric_limits<FloatType>::infinity();
		return std::fabs(static_cast<const ContinuousBearingObservation &>(otherObservation).bearing() - bearing_);
	}

private:
	FloatType bearing_;
	bool pushed_;

};

class DistObservation: public DiscreteVectorObservation {
public:
	DistObservation(const int dist, const bool &pushed):
		DiscreteVectorObservation(VectorFloat({})),
		dist_(dist),
		pushed_(pushed) {
		if (pushed_) {
			setBinNumber(dist+11);
		} else {
			setBinNumber(dist);
		}
	}

	virtual ~DistObservation() = default;

	bool pushed() const {
		return pushed_;
	}

	int dist() const {
		return dist_;
	}

	virtual bool equals(const Observation& otherObservation) const override {
		auto otherObs = otherObservation.as<DistObservation>();
		if (otherObs->pushed() != pushed_)
			return false;
		if (otherObs->dist() != dist_)
			return false;
		return true;
	}

	virtual void print(std::ostream& os) const override {
		os << "dist: " << dist_ << ", pushed: " << pushed_; 
	}

private:
	bool pushed_;

	int dist_;

};

}

#endif