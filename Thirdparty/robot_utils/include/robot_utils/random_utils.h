/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/

#pragma once

#include <cmath>
#include <cassert>
#include <cstring>
#include <Eigen/Dense>
#include <iostream>
#include <random>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Cholesky>
#include <random>

namespace robot_utils {
	
	/**
	 * A templated random sample function (w/o replacement). Based on MATLAB implementation of
	 * randsample()
	 * @tparam T A number type
	 * @tparam URBG A UniformRandomBitGenerator type
	 * @param input A input vector containing the whole population
	 * @param num_samples Number of samples we want
	 * @param g
	 * @return
	 */
	template<class T, class URBG>
	std::vector<T> randomSample(std::vector<T> input, size_t num_samples, URBG &&g) {
		
		std::vector<T> output;
		output.reserve(num_samples);
		if (4 * num_samples > input.size()) {
			// if the sample is a sizeable fraction of the whole population,
			// just randomly shuffle the entire population and return the
			// first num_samples
			std::shuffle(input.begin(), input.end(), g);
			for (size_t i = 0; i < num_samples; ++i) {
				output.push_back(input[i]);
			}
		} else {
			// if the sample is small, repeatedly sample with replacement until num_samples
			// unique values
			std::unordered_set<size_t> sample_indices;
			std::uniform_int_distribution<> dis(0, input.size());
			while (sample_indices.size() < num_samples) {
				sample_indices.insert(dis(std::forward<URBG>(g)));
			}
			for (auto &&i: sample_indices) {
				output.push_back(input[i]);
			}
		}
		return output;
	}

    template<typename T>
    Eigen::Matrix<T, 3, 1> sampleMultivariateNormal(const Eigen::Matrix<T, 3, 1> &mean,
                                                    const Eigen::Matrix<T, 3, 3> &cov) {
		// use Cholesky decomposition to compute the square root of the covariance matrix
        Eigen::LLT<Eigen::Matrix<T, 3, 3>> chol(cov);
        Eigen::Matrix<T, 3, 3> L = chol.matrixL();

		// use normal distribution to generate random numbers
        std::random_device rd;
        std::mt19937 gen(rd());
        std::normal_distribution<T> distribution(0.0, 1.0);

		// generate random vector
        int n = mean.size();
        Eigen::Matrix<T, 3, 1> x(n);
        for (int i = 0; i < n; ++i) {
            x(i) = distribution(gen);
        }

		// z = mean + L * x
        Eigen::Matrix<T, 3, 1> z = mean + L * x;
        return z;
    }

    template<typename T>
    Eigen::Matrix<T, 3, 3> randomRotation(const T &angle_bound) {
        Eigen::Matrix<T, 3, 1> axis = Eigen::Matrix<T, 3, 1>::Random();
        axis.normalize();
        // random [-1, 1]
        T angle = angle_bound * Eigen::internal::random<T>() / 180.0 * M_PI;
        return Eigen::AngleAxis<T>(angle, axis).toRotationMatrix();
    }

    template<typename T>
    Eigen::Matrix<T, 3, 1> randomTranslation(const T &translation_bound) {
        // random [-1, 1] for each element
        return Eigen::Matrix<T, 3, 1>::Random() * translation_bound;
    }

    template<typename T>
    Eigen::Matrix<T, 4, 4> randomTransformation(const T &angle_bound, const T &translation_bound) {
        Eigen::Matrix<T, 4, 4> ans = Eigen::Matrix<T, 4, 4>::Identity();
        ans.template block<3, 3>(0, 0) = randomRotation(angle_bound);
        ans.template block<3, 1>(0, 3) = randomTranslation(translation_bound);
        return ans;
    }

    template<typename T>
    inline T random_normal(T std) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::normal_distribution<T> dis(0, std);
        T number = dis(gen);
        return number;
    }

    template<typename T>
    inline T random_uniform(T min, T max) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<T> dis(min, max);
        T number = dis(gen);
        return number;
    }
}
