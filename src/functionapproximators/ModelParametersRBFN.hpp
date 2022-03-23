/**
 * @file   ModelParametersRBFN.hpp
 * @brief  ModelParametersRBFN class header file.
 * @author Freek Stulp
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
 
#ifndef MODELPARAMETERSRBFN_H
#define MODELPARAMETERSRBFN_H

#include "functionapproximators/ModelParameters.hpp"

#include "dmpbbo_io/EigenBoostSerialization.hpp"

#include <iosfwd>
#include <vector>

#include <eigen3/Eigen/Core>

#include <boost/serialization/access.hpp>
#include <boost/serialization/nvp.hpp>

namespace DmpBbo {

  // Forward declaration
class UnifiedModel;


/** \brief Model parameters for the Radial Basis Function Network (RBFN) function approximator
 * \ingroup FunctionApproximators
 * \ingroup RBFN
 */
class ModelParametersRBFN : public ModelParameters
{
  friend class FunctionApproximatorRBFN;
  
public:
  /** Constructor for the model parameters of the LWPR function approximator.
   *  \param[in] centers Centers of the basis functions
   *  \param[in] widths  Widths of the basis functions. 
   *  \param[in] weights Weight of each basis function
   */
  ModelParametersRBFN(const Eigen::MatrixXd& centers, const Eigen::MatrixXd& widths, const Eigen::MatrixXd& weights);
  
  std::string toString(void) const;
  
	ModelParameters* clone(void) const;
	
  int getExpectedInputDim(void) const  {
    return centers_.cols();
  };
  
  /** Get the number of basis functions in this model.
   * \return The number of basis functions.
   */
  inline unsigned int getNumberOfBasisFunctions() const
  {
    return centers_.rows();
  }
  
  /** Get the kernel activations for given inputs
   * \param[in] inputs The input data (size: n_samples X n_dims)
   * \param[out] kernel_activations The kernel activations, computed for each of the samples in the input data (size: n_samples X n_basis_functions)
   */
  void kernelActivations(const Eigen::Ref<const Eigen::MatrixXd>& inputs, Eigen::MatrixXd& kernel_activations) const;
  
  void getSelectableParameters(std::set<std::string>& labels) const;
  void getParameterVector(Eigen::VectorXd& values, bool normalized) const;
  void setParameterVector(const Eigen::VectorXd& values, bool normalized);
  void setParameterVectorModifierPrivate(std::string modifier, bool new_value);
  
  /** Return the weights of the basis functions.
   * \return weights of the basis functions.
   */
  const Eigen::VectorXd& weights(void) const { return weights_; }  
  
  /** Return the weights of the basis functions.
   * \param[out] weights of the basis functions.
   */
  inline void weights(Eigen::VectorXd& weights) const { weights=weights_; }  
  
private:
  Eigen::MatrixXd centers_; // n_centers X n_dims
  Eigen::MatrixXd widths_;  // n_centers X n_dims
  Eigen::VectorXd weights_; // n_centers X 1
    
public:
	/** Turn caching for the function kernelActivations() on or off.
	 * Turning this on should lead to substantial improvements in execution time if the centers and
	 * widths of the kernels do not change often AND you call normalizedKernelActivations with the
	 * same inputs over and over again.
	 * \param[in] caching Whether to turn caching on or off
	 * \remarks In the constructor, caching is set to true, so by default it is on.
	 */
	inline void set_caching(bool caching)
	{
	  caching_ = caching;
	  if (!caching_) clearCache();
	}
	
  UnifiedModel* toUnifiedModel(void) const;
  
private:
  
  mutable Eigen::MatrixXd inputs_cached_;
  mutable Eigen::MatrixXd kernel_activations_cached_;
  bool caching_;
  inline void clearCache(void) 
  {
    inputs_cached_.resize(0,0);
    kernel_activations_cached_.resize(0,0);
  }
  
  /**
   * Default constructor.
   * \remarks This default constuctor is required for boost::serialization to work. Since this
   * constructor should not be called by other classes, it is private (boost::serialization is a
   * friend)
   */
  ModelParametersRBFN(void) {};

  
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
    ar & BOOST_SERIALIZATION_NVP(centers_);
    ar & BOOST_SERIALIZATION_NVP(widths_);
    ar & BOOST_SERIALIZATION_NVP(weights_);
    ar & BOOST_SERIALIZATION_NVP(caching_);
  }

};

}

#endif        //  #ifndef MODELPARAMETERSRBFN_H

