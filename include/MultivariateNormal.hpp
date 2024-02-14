/**
 * Copyright 2017
 *
 * This file is part of On-line POMDP Planning Toolkit (OPPT).
 * OPPT is free software: you can redistribute it and/or modify it under the terms of the
 * GNU General Public License published by the Free Software Foundation,
 * either version 2 of the License, or (at your option) any later version.
 *
 * OPPT is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with OPPT.
 * If not, see http://www.gnu.org/licenses/.
 */
#pragma once
#include <oppt/opptCore/core.hpp>

using namespace Eigen;

namespace oppt
{

/**
 * Mutivariate normal distribution
 */
template<typename Scalar>
class MultivariateNormal: public Distribution<Scalar>
{
public:
    MultivariateNormal(RandomEnginePtr& randomEngine):
        Distribution<FloatType>(randomEngine) {
        randN_.setRandomEngine(this->randomEngine_);
    }

    // Copy constructor
    MultivariateNormal(const MultivariateNormal& other):
        Distribution<Scalar>(other.randomEngine_),
        mean_(other.mean_),
        covariance_(other.covariance_),
        randN_(other.randN_) {

    }

    // Assignment operator
    MultivariateNormal& operator=(const MultivariateNormal& other) {
        if (this != &other) {
            randN_ = other.randN_;
            mean_ = other.mean_;
            covariance_ = other.covariance_;
        }

        return *this;
    }

    /**
     * @brief Set the mean of the distribution
     */
    void setMean(const Eigen::Matrix< Scalar, Dynamic, 1>& mean) {
        mean_ = mean;
    }

    /**
     * @brief Set the covariance matrix of the distribution
     */
    void setCovariance(const Eigen::Matrix<Scalar, Dynamic, Dynamic>& covariance) {
        numRows_ = covariance.rows();
        transform_ = Eigen::LLT<Eigen::Matrix<Scalar, Dynamic, Dynamic>>(covariance).matrixL();
        //Eigen::LLT<Eigen::Matrix<Scalar, Dynamic, Dynamic>> cholSolver(covariance);
        /**if (cholSolver.info() == Eigen::Success) {            
            //transform_ = cholSolver.matrixL();
        } else {
            ERROR("Cholesky decomposition not successful");
            SelfAdjointEigenSolver<Matrix<Scalar, Dynamic, Dynamic> > eigenSolver(covariance);
            transform_ = eigenSolver.eigenvectors() * eigenSolver.eigenvalues().cwiseMax(0).cwiseSqrt().asDiagonal();
        }*/
    }

    virtual FloatType pdf(const std::vector<Scalar>& position) const override {
        return 0.0;
    }

    virtual const Eigen::Matrix < Scalar, Dynamic, -1 > sample(const unsigned int& numSamples) const override {
        return (transform_ * Matrix < Scalar, Dynamic, -1 >::NullaryExpr(numRows_, numSamples, randN_)).colwise() + mean_;
    }

    /**
     * @brief Get the covariance matrix
     */
    Eigen::Matrix<Scalar, Dynamic, Dynamic> getCovarianceMatrix() const {
        return covariance_;
    }

    /**
     * @brief Get the mean
     */
    Eigen::Matrix< Scalar, Dynamic, 1> getMean() const {
        mean_;
    }

    virtual unsigned int getNumDimensions() const override {
        return mean_.rows();
    }

    virtual void setRandomEngine(RandomEnginePtr &randomEngine) override {
        this->randomEngine_ = randomEngine;
        randN_.setRandomEngine(this->randomEngine_);
    }

private:
    struct ScalarDistOperator {
    public:
        mutable std::normal_distribution<Scalar> norm;

        mutable std::uniform_real_distribution<FloatType> real_distr;

        ScalarDistOperator() {

        }

        template<typename Index>
        inline const Scalar operator()(Index, Index = 0) const {
            return norm(*(randomEngine_.get()));
        }

        inline FloatType sampleUniform() {
            FloatType sample = real_distr(*(randomEngine_.get()));
            return sample;
        }

        inline void setRandomEngine(RandomEnginePtr randomEngine) {
            randomEngine_ = randomEngine;
        }

    private:
        RandomEnginePtr randomEngine_;

    };

    ScalarDistOperator randN_;

private:
    Eigen::Matrix< Scalar, Dynamic, 1> mean_;

    Eigen::Matrix<Scalar, Dynamic, Dynamic> covariance_;

    Eigen::Matrix<Scalar, Dynamic, Dynamic> covarianceInverse_;

    Scalar denominator_;

    Matrix<Scalar, Dynamic, Dynamic> transform_;

    size_t numRows_ = 0;

};

}