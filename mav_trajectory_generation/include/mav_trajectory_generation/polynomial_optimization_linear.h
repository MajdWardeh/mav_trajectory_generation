/*
 * Copyright (c) 2016, Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright (c) 2016, Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright (c) 2016, Helen Oleynikova, ASL, ETH Zurich, Switzerland
 * Copyright (c) 2016, Rik Bähnemann, ASL, ETH Zurich, Switzerland
 * Copyright (c) 2016, Marija Popovic, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef MAV_TRAJECTORY_GENERATION_POLYNOMIAL_OPTIMIZATION_LINEAR_H_
#define MAV_TRAJECTORY_GENERATION_POLYNOMIAL_OPTIMIZATION_LINEAR_H_

#include <Eigen/Sparse>
#include <Eigen/StdVector>
#include <glog/logging.h>
#include <tuple>

#include "mav_trajectory_generation/extremum.h"
#include "mav_trajectory_generation/motion_defines.h"
#include "mav_trajectory_generation/polynomial.h"
#include "mav_trajectory_generation/segment.h"
#include "mav_trajectory_generation/trajectory.h"
#include "mav_trajectory_generation/vertex.h"

namespace mav_trajectory_generation {

/**
 * \brief Implements the unconstrained optimization of paths consisting of
 * polynomial segments as described in [1]
 *
 * [1]: Polynomial Trajectory Planning for Aggressive Quadrotor Flight in Dense
 * Indoor Environments.
 *      Charles Richter, Adam Bry, and Nicholas Roy. In ISRR 2013
 *
 * \tparam _N Number of coefficients of the underlying polynomials.
 * Polynomial coefficients are stored with increasing powers, i.e. \f$c_0 +
 * c_1*t ... c_{N-1} * t^{N-1}\f$
 */
template <int _N = 10>
class PolynomialOptimization {
  static_assert(_N % 2 == 0, "The number of coefficients has to be even.");

 public:
  enum { N = _N };
  static constexpr int kHighestDerivativeToOptimize = N / 2 - 1;
  typedef Eigen::Matrix<double, N, N> SquareMatrix;
  typedef std::vector<SquareMatrix, Eigen::aligned_allocator<SquareMatrix> >
      SquareMatrixVector;

  /**
   * \brief Sets up the optimization problem for the specified dimension.
   */
  PolynomialOptimization(size_t dimension);

  /**
   * \brief Sets up the optimization problem from a vector of Vertex objects and
   * a vector of times between the vertices.
   *
   * \param[in] vertices Vector containing the vertices defining the support
   * points and constraints of the path.
   * \param[in] segment_times Vector containing the time between two vertices.
   * Thus, its size is size(vertices) - 1.
   * \param[in] derivative_to_optimize Specifies the derivative of which the
   * cost is optimized.
   */
  bool setupFromVertices(
      const Vertex::Vector& vertices, const std::vector<double>& segment_times,
      int derivative_to_optimize = kHighestDerivativeToOptimize);

  /**
   * \brief Sets up the optimization problem from a vector of positions and a
   * vector of times between the via points.
   * The optimized derivative is set to the maximum possible based on the given
   * N (N/2-1).
   * \param[in] positions Vector containing the start positions, intermediate
   * positions and the final position.
   * \param[in] times Vector containing the time between two positions. Thus,
   * its size is size(positions) - 1.
   */
  bool setupFromPositons(const std::vector<double>& positions,
                         const std::vector<double>& times);

  /**
   * \brief Wrapper that inverts the mapping matrix (A in [1]) to take advantage
   * of its structure.
   * \param[in] A matrix
   * \param[out] Ai inverse of the A matrix
   */
  static void invertMappingMatrix(const SquareMatrix& mapping_matrix,
                                  SquareMatrix* inverse_mapping_matrix);

  static void setupMappingMatrix(double segment_time, SquareMatrix* A);

  /**
   * \brief Computes the cost in the derivative that was specified during
   * setupFromVertices().
   *
   * The cost is computed as: \f$\sum{0.5 c^T Q c}\f$,
   * where c are the coefficients and Q is the cost matrix of each segment.
   */
  double computeCost() const;

  /**
   * \brief Updates the segment times. The number of times has to be equal to
   * the number of
   *        vertices that was initially passed during the problem setup.
   *
   * This recomputes all cost- and inverse mapping block-matrices and is meant
   * to be called
   * during non-linear optimization procedures.
   */
  void updateSegmentTimes(const std::vector<double>& segment_times);

  /**
   * \brief Solves the linear optimization problem according to [1].
   *
   * The solver is re-used for every dimension, which means:
   *  - segment times are equal for each dimension.
   *  - each dimension has the same type/set of constraints. Their values can of
   * course differ.
   */
  bool solveLinear();

  /**
   * \brief Returns the trajectory created by the optimization.
   *
   * Only valid after solveLinear() is called. This is the preferred external
   * interface for getting information back out of the solver.
   */
  void getTrajectory(Trajectory* trajectory) const {
    CHECK_NOTNULL(trajectory);
    trajectory->setSegments(segments_);
  }

  /**
   * \brief Computes the candidates for the maximum magnitude of a single
   * segment in the specified derivative.
   *
   * In the 1D case, it simply returns the roots of the derivative of the
   * segment-polynomial.
   * For higher dimensions, e.g. 3D, we need to find the extrema of \f$
   * \sqrt{x(t)^2 + y(t)^2 + z(t)^2} \f$,
   * where x, y, z are polynomials describing the position or the derivative,
   * specified by Derivative.
   * Taking the derivative yields \f$ 2 x \dot{x} + 2 y \dot{y} + 2 z \dot{z}
   * \f$, which needs to be zero
   * at the extrema. The multiplication of two polynomials is a convolution of
   * their coefficients.
   * Re-ordering by their powers and addition yields a polynomial, for which we
   * can compute the roots.
   *
   * \tparam Derivative Derivative of position, in which to find the maxima.
   * \param[in] segment Segment to find the maximum.
   * \param[in] t_start Only maxima >= t_start are returned. Usually set to 0
   * \param[in] t_stop Only maxima <= t_stop are returned. Usually set to
   * segment time.
   * \param[out] candidates Vector containing the candidate times for a maximum.
   * Returns whether the computation succeeded -- false means no candidates
   * were found by Jenkins-Traub.
   */
  template <int Derivative>
  static bool computeSegmentMaximumMagnitudeCandidates(
      const Segment& segment, double t_start, double t_stop,
      std::vector<double>* candidates);

  /**
   * \brief Computes the candidates for the maximum magnitude of a single
   * segment in the specified derivative.
   *        Computed by sampling and rather meant for debugging / testing.
   *
   * \tparam Derivative Derivative of position, in which to find the maxima.
   * \param[in] segment Segment to find the maximum.
   * \param[in] t_start Start time of sampling. Usually set to 0
   * \param[in] t_stop End time of sampling. Usually set to segment time.
   * \param[in] sampling_interval Time between two sampling points.
   * \param[out] candidates Vector containing the candidate times for a maximum.
   */
  template <int Derivative>
  static void computeSegmentMaximumMagnitudeCandidatesBySampling(
      const Segment& segment, double t_start, double t_stop,
      double sampling_interval, std::vector<double>* candidates);

  /**
   * \brief Computes the global maximum of the magnitude of the path in the
   * specified derivative.
   *
   * This uses \sa computeSegmentMaximumMagnitudeCandidates to compute the
   * candidates for each segment.
   *
   * \tparam Derivative Derivative of position, in which to find the maxima.
   * \param[out] candidates Vector containing the candidate times for the global
   * maximum, i.e. all local maxima.
   *                        Optional, can be set to nullptr if not needed.
   * \return The global maximum of the path.
   */
  template <int Derivative>
  Extremum computeMaximumOfMagnitude(std::vector<Extremum>* candidates) const;

  // Only for internal use -- always use getTrajectory() instead if you can!
  void getSegments(Segment::Vector* segments) const {
    CHECK_NOTNULL(segments);
    *segments = segments_;
  }

  void getSegmentTimes(std::vector<double>* segment_times) const {
    CHECK(segment_times != nullptr);
    *segment_times = segment_times_;
  }

  void getFreeConstraints(
      std::vector<Eigen::VectorXd>* free_constraints) const {
    CHECK(free_constraints != nullptr);
    *free_constraints = free_constraints_compact_;
  }

  void setFreeConstraints(const std::vector<Eigen::VectorXd>& free_constraints);

  void getFixedConstraints(
      std::vector<Eigen::VectorXd>* fixed_constraints) const {
    CHECK(fixed_constraints != nullptr);
    *fixed_constraints = fixed_constraints_compact_;
  }

  /**
   * \brief Computes the Jacobian of the integral over the squared derivative
   * \param[out] C Jacobian matrix to write into. If C is dynamic, the correct
   * size has to be set.
   * \param[in] t time of evaluation
   * \param[in] derivative used to compute the cost
   */
  static void computeQuadraticCostJacobian(int derivative, double t,
                                           SquareMatrix* cost_jacobian);

  size_t getDimension() const { return dimension_; }
  size_t getNumberSegments() const { return n_segments_; }
  size_t getNumberAllConstraints() const { return n_all_constraints_; }
  size_t getNumberFixedConstraints() const { return n_fixed_constraints_; }
  size_t getNumberFreeConstraints() const { return n_free_constraints_; }

  // Accessor functions for internal matrices.
  void getAInverse(Eigen::MatrixXd* A_inv) const;
  void getM(Eigen::MatrixXd* M) const;
  void getR(Eigen::MatrixXd* R) const;

  void printReorderingMatrix(std::ostream& stream) const;

 private:
  /** Constructs the sparse R (cost) matrix. **/
  void constructR(Eigen::SparseMatrix<double>* R) const;

  /**
   * \brief Sets up the matrix (C in [1]) that reorders constraints for the
   * optimization problem.
   *
   * This matrix is the same for each dimension, i.e. each dimension must have
   * the same fixed and free parameters.
   */
  void setupConstraintReorderingMatrix();

  /**
   * \brief Updates the segments stored internally from the set of compact fixed
   * and free constraints.
   */
  void updateSegmentsFromCompactConstraints();

  /**
   * \brief Matrix consisting of entries with value 1 to reorder free and fixed
   * constraints.
   *        Corresponds to \f$C\f$ in [1]
   */
  Eigen::SparseMatrix<double> constraint_reordering_;

  // Original vertices containing the constraints.
  Vertex::Vector vertices_;

  // The actual segments containing the solution.
  Segment::Vector segments_;

  /**
   * \brief Vector that stores an inverted mapping matrix for each segment.
   * Corresponds to \f$ A^{-1} \f$ in [1].
   */
  SquareMatrixVector inverse_mapping_matrices_;

  /**
   * \brief Vector that stores the cost matrix for each segment. Corresponds to
   * \f$ Q \f$ in [1].
   */
  SquareMatrixVector cost_matrices_;

  /**
   * \brief Contains the compact form of fixed constraints for each dimension.
   * Corresponds to \f$ d_f \f$ in [1].
   */
  std::vector<Eigen::VectorXd> fixed_constraints_compact_;

  /**
   * \brief Contains the compact form of free constraints to optimize for each
   * dimension.
   *        Corresponds to \f$ d_p \f$ in [1].
   */
  std::vector<Eigen::VectorXd> free_constraints_compact_;

  std::vector<double> segment_times_;

  /**
   * \brief This is the number of polynomials e.g 3 for a 3D path.
   */
  size_t dimension_;

  int derivative_to_optimize_;
  size_t n_vertices_;
  size_t n_segments_;

  size_t n_all_constraints_;
  size_t n_fixed_constraints_;
  size_t n_free_constraints_;
};

// Constraint class that aggregates all constraints from incoming Vertices.
struct Constraint {
  inline bool operator<(const Constraint& rhs) const {
    if (vertex_idx < rhs.vertex_idx) return true;
    if (rhs.vertex_idx < vertex_idx) return false;

    if (constraint_idx < rhs.constraint_idx) return true;
    if (rhs.constraint_idx < constraint_idx) return false;
    return false;
  }

  inline bool operator==(const Constraint& rhs) const {
    return vertex_idx == rhs.vertex_idx && constraint_idx == rhs.constraint_idx;
  }

  size_t vertex_idx;
  size_t constraint_idx;
  Vertex::ConstraintValue value;
};

}  // namespace mav_trajectory_generation

#include "mav_trajectory_generation/impl/polynomial_optimization_linear_impl.h"

#endif  // MAV_TRAJECTORY_GENERATION_POLYNOMIAL_OPTIMIZATION_LINEAR_H_
