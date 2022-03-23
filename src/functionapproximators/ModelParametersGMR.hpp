/**
 * @file   ModelParametersGMR.hpp
 * @brief  ModelParametersGMR class header file.
 * @author Freek Stulp, Thibaut Munzer
 *
 * This file is part of DmpBbo, a set of libraries and programs for the 
 * black-box optimization of dynamical movement primitives.
 * Copyright (C) 2014 Freek Stulp, ENSTA-ParisTech
 * 
 * DmpBbo is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 * 
 * DmpBbo is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with DmpBbo.  If not, see <http://www.gnu.org/licenses/>. 
 */
 
#ifndef MODELPARAMETERSGMR_H
#define MODELPARAMETERSGMR_H

#include "functionapproximators/ModelParameters.hpp"

#include "dmpbbo_io/EigenBoostSerialization.hpp"

#include <iosfwd>
#include <vector>

#include <boost/serialization/access.hpp>
#include <boost/serialization/nvp.hpp>

namespace DmpBbo {

/** \brief Model parameters for the GMR function approximator
 * \ingroup FunctionApproximators
 * \ingroup GMR
 */
class ModelParametersGMR : public ModelParameters
{
  friend class FunctionApproximatorGMR;
  
public:
  
  /** Constructor for the model parameters of the GMR function approximator.
   */
  ModelParametersGMR(std::vector<double> priors,
    std::vector<Eigen::VectorXd> means, 
    std::vector<Eigen::MatrixXd> covars, int n_output_dims=1);

  /** Constructor for the model parameters of the GMR function approximator.
   */
  ModelParametersGMR(std::vector<double> priors, std::vector<Eigen::VectorXd> mu_xs,
    std::vector<Eigen::VectorXd> mu_ys, std::vector<Eigen::MatrixXd> sigma_xs,
    std::vector<Eigen::MatrixXd> sigma_ys, std::vector<Eigen::MatrixXd> sigma_x_ys);

  /** Constructor for the model parameters of the GMR function approximator (Used by the incremental training).
   */
  ModelParametersGMR(int n_observations,
    std::vector<double> priors,
    std::vector<Eigen::VectorXd> means,
    std::vector<Eigen::MatrixXd> covars, int n_output_dims=1);

  /** Constructor for the model parameters of the GMR function approximator (Used by the incremental training).
   */
  ModelParametersGMR(int n_observations, std::vector<double> priors, std::vector<Eigen::VectorXd> mu_xs,
    std::vector<Eigen::VectorXd> mu_ys, std::vector<Eigen::MatrixXd> sigma_xs,
    std::vector<Eigen::MatrixXd> sigma_ys, std::vector<Eigen::MatrixXd> sigma_x_ys);

  /** 
   * Get the number of Gaussians in the GMM.
   * \return The number of Gaussians in the GMM.
   */ 
  inline unsigned int getNumberOfGaussians(void) const 
  {
    return priors_.size();
  }
  
  inline virtual int getExpectedOutputDim(void) const 
  {
	  assert(means_y_.size()>0); // This is also checked in the constructor
    return means_y_[0].size();
  }
  
	inline virtual int getExpectedInputDim(void) const
	{
	  assert(means_x_.size()>0); // This is also checked in the constructor
	  return means_x_[0].size();
	};

	
	std::string toString(void) const;

  ModelParameters* clone(void) const;

  void getSelectableParameters(std::set<std::string>& selected_values_labels) const;
  void getParameterVector(Eigen::VectorXd& values, bool normalized=false) const {};
  void setParameterVector(const Eigen::VectorXd& values, bool normalized=false) {};
  
  /** Save a Gaussian mixture model to a directory; useful for debugging.
   * \param[in] directory Directory to save to
   * \param[in] centers Centers of the Gaussians
   * \param[in] covars Covariance matrices of the Gaussians
   * \param[in] overwrite Whether to overwrite existing files or not.
   * \param[in] iter Iteration number when running Expectation-Maximization. Allows the GMM to be stored with a different filename (in the same directory) at each iteration.
   * \return true if successful, false otherwise
   */
  static bool saveGMM(std::string directory, const std::vector<Eigen::VectorXd>& centers, const std::vector<Eigen::MatrixXd>& covars, bool overwrite=false, int iter=-1);
  
  /** Save a Gaussian mixture model to a directory; useful for debugging.
   * \param[in] directory Directory to save to
   * \param[in] overwrite Whether to overwrite existing files or not.
   * \return true if successful, false otherwise
   */
  bool saveGMM(std::string directory, bool overwrite=false) const;

  /** This function represents the Gaussian Mixture Model as one big matrix.
   *
   * \param[out] gmm_as_matrix The Gaussian Mixture Model as one big matrix
   *
   * In combination with ModelParametersGMR::saveGMMToMatrix() and ModelParametersGMR::loadGMMFromMatrix(), this hacky function allowed for easier exchange with some Matlab code we wrote.
   *
   * This matrix contains the following rows (example for two Gaussians, with dimensionality 3)
   * 
\verbatim
 0. meta_data  (n_gaussians and n_output_dims)
 1. meta_data  (n_observations)
__________________________
 2. prior1 and E1
 3. mean1      (size: 3)
 4. covar1     (row1 of covar matrix)
 5. covar1     (row2 of covar matrix)
 6. covar1     (row3 of covar matrix)
 __________________________
 7. prior2 and E2
 8. mean2      (size: 3)
 9. covar2     (row1 of covar matrix)
 10. covar2     (row2 of covar matrix)
 11. covar2     (row3 of covar matrix)
\endverbatim
   *
   * Thus, the number of rows in the Matrix is 2 (for meta-data) + n_gaussians * (1+1+n_dims)
   *
   *
   */
  void toMatrix(Eigen::MatrixXd& gmm_as_matrix) const;
  
  
  /** Initialize a GMM from a matrix.
   * \see toMatrix() for the format of the file
   *  \param[in] gmm_matrix The GMM, represented as a matrix.
   *  \return The model parameters of the GMM.
   */
  static ModelParametersGMR* fromMatrix(const Eigen::MatrixXd& gmm_matrix);
  
  /** Save the GMM as a matrix in an ASCII file.
   * \see toMatrix() for the format of the file
   *  \param[in] filename The name of the file to save the GMM to.
   *  \param[in] overwrite Whether to overwrite the file if it already exists.
   *  \return true if saving was successful, false otherwise.
   */
  bool saveGMMToMatrix(std::string filename, bool overwrite=false) const; 
  
  /** Load the GMM from a matrix in an ASCII file.
   * \see toMatrix() for the format of the file
   *  \param[in] filename The name of the file to load the GMM from.
   *  \return The model parameters of the GMM.
   */
  static ModelParametersGMR* loadGMMFromMatrix(std::string filename);

  
  UnifiedModel* toUnifiedModel(void) const;

protected:
  void setParameterVectorAll(const Eigen::VectorXd& values);
  
private:
  std::vector<double> priors_;

  std::vector<Eigen::VectorXd> means_x_;
  std::vector<Eigen::VectorXd> means_y_;

  std::vector<Eigen::MatrixXd> covars_x_;
  std::vector<Eigen::MatrixXd> covars_y_;
  std::vector<Eigen::MatrixXd> covars_y_x_;

  /** Number of observations used to create the model. (Used by the incremental learning) */
  int n_observations_;

  void updateCachedMembers(void);
  
  /** This is covars_x_.inverse(). Since we used it often, we cache it here. */
  std::vector<Eigen::MatrixXd> covars_x_inv_;
  
  /** This is the scale factor in a multivariate normal distribution
   * 1/sqrt((2*pi)^k*|Sigma|). Since we used it often, we cache it here. */
  std::vector<double> mvgd_scale_;

  /**
   * Default constructor.
   * \remarks This default constuctor is required for boost::serialization to work. Since this
   * constructor should not be called by other classes, it is private (boost::serialization is a
   * friend)
   */
  ModelParametersGMR(void) {};
  
  /** Give boost serialization access to private members. */  
  friend class boost::serialization::access;
  
  /** Serialize class data members to boost archive. 
   * \param[in] ar Boost archive
   * \param[in] version Version of the class
   * See http://www.boost.org/doc/libs/1_55_0/libs/serialization/doc/tutorial.html#simplecase
   */
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(ModelParameters);
    ar & BOOST_SERIALIZATION_NVP(priors_);
    ar & BOOST_SERIALIZATION_NVP(means_x_);
    ar & BOOST_SERIALIZATION_NVP(means_y_);
    ar & BOOST_SERIALIZATION_NVP(covars_x_);
    ar & BOOST_SERIALIZATION_NVP(covars_y_);
    ar & BOOST_SERIALIZATION_NVP(covars_y_x_);
    ar & BOOST_SERIALIZATION_NVP(covars_x_inv_);
    ar & BOOST_SERIALIZATION_NVP(mvgd_scale_);
  }
    

};

}

#endif        //  #ifndef MODELPARAMETERSGMR_H

