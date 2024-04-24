#ifndef ROBOT_UTILS_EIGEN_TYPES_H
#define ROBOT_UTILS_EIGEN_TYPES_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

/// alias for eigen
using Vec2i = Eigen::Vector2i;
using Vec3i = Eigen::Vector3i;

using Vec2d = Eigen::Vector2d;
using Vec2f = Eigen::Vector2f;
using Vec3d = Eigen::Vector3d;
using Vec3f = Eigen::Vector3f;
using Vec5d = Eigen::Matrix<double, 5, 1>;
using Vec5f = Eigen::Matrix<float, 5, 1>;
using Vec6d = Eigen::Matrix<double, 6, 1>;
using Vec6f = Eigen::Matrix<float, 6, 1>;
using Vec15d = Eigen::Matrix<double, 15, 15>;

using Mat1d = Eigen::Matrix<double, 1, 1>;
using Mat3d = Eigen::Matrix3d;
using Mat3f = Eigen::Matrix3f;
using Mat4d = Eigen::Matrix4d;
using Mat4f = Eigen::Matrix4f;
using Mat5d = Eigen::Matrix<double, 5, 5>;
using Mat5f = Eigen::Matrix<float, 5, 5>;
using Mat6d = Eigen::Matrix<double, 6, 6>;
using Mat6f = Eigen::Matrix<float, 6, 6>;
using Mat15d = Eigen::Matrix<double, 15, 15>;

using Quatd = Eigen::Quaterniond;
using Quatf = Eigen::Quaternionf;

namespace robot_utils {

/// less of vector
    template<int N>
    struct less_vec {
        inline bool operator()(const Eigen::Matrix<int, N, 1> &v1, const Eigen::Matrix<int, N, 1> &v2) const;
    };

/// hash of vector
    template<int N>
    struct hash_vec {
        inline size_t operator()(const Eigen::Matrix<int, N, 1> &v) const;
    };

/// implementation
    template<>
    inline bool less_vec<2>::operator()(const Eigen::Matrix<int, 2, 1> &v1, const Eigen::Matrix<int, 2, 1> &v2) const {
        return v1[0] < v2[0] || (v1[0] == v2[0] && v1[1] < v2[1]);
    }

    template<>
    inline bool less_vec<3>::operator()(const Eigen::Matrix<int, 3, 1> &v1, const Eigen::Matrix<int, 3, 1> &v2) const {
        return v1[0] < v2[0] ||
               (v1[0] == v2[0] && v1[1] < v2[1]) && (v1[0] == v2[0] && v1[1] == v2[1] && v1[2] < v2[2]);
    }

/// vec 2 hash
/// @see Optimized Spatial Hashing for Collision Detection of Deformable Objects, Matthias Teschner et. al., VMV 2003
    template<>
    inline size_t hash_vec<2>::operator()(const Eigen::Matrix<int, 2, 1> &v) const {
        return size_t(((v[0]) * 73856093) ^ ((v[1]) * 471943)) % 10000000;
    }

/// vec 3 hash
    template<>
    inline size_t hash_vec<3>::operator()(const Eigen::Matrix<int, 3, 1> &v) const {
        return size_t(((v[0]) * 73856093) ^ ((v[1]) * 471943) ^ ((v[2]) * 83492791)) % 10000000;
    }


    struct Vec3dHash {
        std::size_t operator()(const std::tuple<int, int, int> &vec3) const {
            return size_t(
                    ((std::get<0>(vec3)) * 73856093) ^ ((std::get<1>(vec3)) * 471943) ^ ((std::get<2>(vec3)) * 83492791)) % 10000000;
        }
    };

    constexpr auto less_vec2i = [](const Vec2i &v1, const Vec2i &v2) {
        return v1[0] < v2[0] || (v1[0] == v2[0] && v1[1] < v2[1]);
    };
	
	/**
	 * Remove one row from matrix.
	 * Credit to: https://stackoverflow.com/questions/13290395
	 * @param matrix an Eigen::Matrix.
	 * @param rowToRemove index of row to remove. If >= matrix.rows(), no operation will be taken
	 */
	template<class T, int R, int C>
	void removeRow(Eigen::Matrix<T, R, C> &matrix, unsigned int rowToRemove) {
		if (rowToRemove >= matrix.rows()) {
			return;
		}
		unsigned int numRows = matrix.rows() - 1;
		unsigned int numCols = matrix.cols();
		
		if (rowToRemove < numRows) {
			matrix.block(rowToRemove, 0, numRows - rowToRemove, numCols) =
					matrix.bottomRows(numRows - rowToRemove);
		}
		
		matrix.conservativeResize(numRows, numCols);
	}
	
	/**
	 * Remove one column from matrix.
	 * Credit to: https://stackoverflow.com/questions/13290395
	 * @param matrix
	 * @param colToRemove index of col to remove. If >= matrix.cols(), no operation will be taken
	 */
	template<class T, int R, int C>
	void removeColumn(Eigen::Matrix<T, R, C> &matrix, unsigned int colToRemove) {
		if (colToRemove >= matrix.cols()) {
			return;
		}
		unsigned int numRows = matrix.rows();
		unsigned int numCols = matrix.cols() - 1;
		
		if (colToRemove < numCols) {
			matrix.block(0, colToRemove, numRows, numCols - colToRemove) =
					matrix.rightCols(numCols - colToRemove);
		}
		
		matrix.conservativeResize(numRows, numCols);
	}
	
	/**
	 * Helper function to calculate the diameter of a row vector of points
	 * @param X
	 * @return the diameter of the set of points given
	 */
	template<class T, int D>
	float calculateDiameter(const Eigen::Matrix<T, D, Eigen::Dynamic> &X) {
		Eigen::Matrix<T, D, 1> cog = X.rowwise().sum() / X.cols();
		Eigen::Matrix<T, D, Eigen::Dynamic> P = X.colwise() - cog;
		Eigen::Matrix<T, 1, Eigen::Dynamic> temp = P.array().square().colwise().sum();
		return 2 * std::sqrt(temp.maxCoeff());
	}
	
	/**
	 * Use an boolean Eigen matrix to mask a vector
	 * @param mask a 1-by-N boolean Eigen matrix
	 * @param elements vector to be masked
	 * @return
	 */
	template<class T>
	inline std::vector<T> maskVector(Eigen::Matrix<bool, 1, Eigen::Dynamic> mask,
									 const std::vector<T> &elements) {
		assert(mask.cols() == elements.size());
		std::vector<T> result;
		for (size_t i = 0; i < mask.cols(); ++i) {
			if (mask(i)) {
				result.emplace_back(elements[i]);
			}
		}
		return result;
	}
	
	/**
	 * Get indices of non-zero elements in an Eigen row vector
	 * @param mask a 1-by-N boolean Eigen matrix
	 * @return A vector containing indices of the true elements in the row vector
	 */
	template<class T>
	inline std::vector<int> findNonzero(const Eigen::Matrix<T, 1, Eigen::Dynamic> &mask) {
		std::vector<int> result;
		for (size_t i = 0; i < mask.cols(); ++i) {
			if (mask(i)) {
				result.push_back(i);
			}
		}
		return result;
	}

}  // namespace faster_lio

#endif
