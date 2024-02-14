#pragma once
#include <oppt/robotHeaders/Observation.hpp>

namespace oppt {

class TagObservation: public DiscreteVectorObservation {
public:
	TagObservation(const bool &visible):
		DiscreteVectorObservation(VectorFloat({})),
		visible_(visible) {
		if (visible_) {
			setBinNumber(1);
		} else {
			setBinNumber(0);
		}
	}

	virtual ~TagObservation() = default;

	bool visible() const {
		return visible_;
	}

	virtual void serialize(std::ostream& os, const std::string& prefix = "") const override {
		if (prefix != "") {
			os << prefix << ": ";
		}

		os << visible_ << " END ";
	}

	virtual void print(std::ostream& os) const override {
		os << "visible: " << visible_;
	}

	virtual oppt::ObservationUniquePtr copy() const override {
		ObservationUniquePtr copied(new TagObservation(visible_));
		return copied;
	}

	virtual bool equals(const Observation& otherObservation) const override {
		return otherObservation.as<TagObservation>()->visible() == visible_;
	}

	virtual std::size_t hash() const override {
		return 0;
	}

	virtual FloatType distanceTo(const Observation& otherObservation) const override {
		return otherObservation.as<TagObservation>()->visible() == visible_ ? 0.0 : std::numeric_limits<FloatType>::infinity();
		//otherObservation.as<TagObservation>()->visible() == visible_ ? return 0.0 : return std::numeric_limits<FloatType>::infinity();
	}

private:
	bool visible_;

};

}