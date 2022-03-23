/**
 * @file   ModelParametersLWPR.hpp
 * @brief  ModelParametersLWPR class header file.
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
 

#ifndef MODELPARAMETERSLWPR_H
#define MODELPARAMETERSLWPR_H

#include <iosfwd>

#include "functionapproximators/ModelParameters.hpp"

#include "dmpbbo_io/EigenBoostSerialization.hpp"

#include <boost/serialization/access.hpp>
#include <boost/serialization/nvp.hpp>

// Forward declarations
class LWPR_Object;

namespace DmpBbo {

class UnifiedModel; // Required for conversion to UnifiedModel

/** \brief Model parameters for the Locally Weighted Projection Regression (LWPR) function approximator
 * \ingroup FunctionApproximators
 * \ingroup LWPR
 */
class ModelParametersLWPR : public ModelParameters
{
  friend class FunctionApproximatorLWPR;
  
public:
  /** Initializing constructor.
   *
   *  Initialize the LWPR model parameters with an LWPR object from the external library.
   *
   *  \param[in] lwpr_object LWPR object from the external library
   */
	ModelParametersLWPR(LWPR_Object* lwpr_object);
	
	~ModelParametersLWPR(void);

  std::string toString(void) const;

  ModelParameters* clone(void) const;

  int getExpectedInputDim(void) const;
  
  void getSelectableParameters(std::set<std::string>& selected_values_labels) const;
  void getParameterVector(Eigen::VectorXd& values, bool normalized=false) const {};
  void setParameterVector(const Eigen::VectorXd& values, bool normalized=false) {};
  
  /** 
   * Convert these LWPR model parameters to unified model parameters.
   * \return Unified model parameter representation
   * \remarks Currently only works if input and output dimensionality are 1
   * \todo Convert for input dim >1
   */
  UnifiedModel* toUnifiedModel(void) const;

private:
  LWPR_Object* lwpr_object_;
  
  void countLengths(void);
  int n_centers_;
  int n_widths_;
  int n_slopes_;
  int n_offsets_;

  /**
   * Default constructor.
   * \remarks This default constuctor is required for boost::serialization to work. Since this
   * constructor should not be called by other classes, it is private (boost::serialization is a
   * friend)
   */
  ModelParametersLWPR(void) {};

  /** Give boost serialization access to private members. */  
  friend class boost::serialization::access;
  
  /** Serialize class data members to boost archive. 
   * \todo Implement this function
   * \param[in] ar Boost archive
   * \param[in] version Version of the class
   * See http://www.boost.org/doc/libs/1_55_0/libs/serialization/doc/tutorial.html#simplecase
   */
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    // serialize base class information
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(ModelParameters);
    
    std::cerr << "ERROR: Don't know how to serialize ModelParametersLWPR yet..." << std::endl;
  
    /*
    ar & BOOST_SERIALIZATION_NVP(centers_);
    ar & BOOST_SERIALIZATION_NVP(widths_);
    ar & BOOST_SERIALIZATION_NVP(slopes_);
    ar & BOOST_SERIALIZATION_NVP(offsets_);
    ar & BOOST_SERIALIZATION_NVP(asymmetric_kernels_);
    ar & BOOST_SERIALIZATION_NVP(lines_pivot_at_max_activation_);
    ar & BOOST_SERIALIZATION_NVP(slopes_as_angles_);
    ar & BOOST_SERIALIZATION_NVP(all_values_vector_size_);
    ar & BOOST_SERIALIZATION_NVP(caching_);
    */
  }

};

}


#endif        //  #ifndef MODELPARAMETERSLWPR_H

