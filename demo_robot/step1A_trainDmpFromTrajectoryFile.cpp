/**
 * \file trainDmpTrainFromTrajectoryFile.cpp
 * \author Freek Stulp
 * \brief  Demonstrates how to train a Dmp with a trajectory in a txt file.
 *
 * \ingroup Demos
 * \ingroup Dmps // zzz
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

#include "dmp/Dmp.hpp"
#include "dmp/Trajectory.hpp"
#include "dmp/serialization.hpp"

#include "dynamicalsystems/DynamicalSystem.hpp"
#include "dynamicalsystems/ExponentialSystem.hpp"
#include "dynamicalsystems/SigmoidSystem.hpp"
#include "dynamicalsystems/TimeSystem.hpp"
#include "dynamicalsystems/SpringDamperSystem.hpp"

#include "functionapproximators/FunctionApproximatorRBFN.hpp"
#include "functionapproximators/MetaParametersRBFN.hpp"
#include "functionapproximators/ModelParametersRBFN.hpp"
#include "functionapproximators/FunctionApproximatorLWR.hpp"
#include "functionapproximators/MetaParametersLWR.hpp"
#include "functionapproximators/ModelParametersLWR.hpp"

#include "dmpbbo_io/EigenFileIO.hpp"

#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>

#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>

using namespace std;
using namespace Eigen;
using namespace DmpBbo;

void help(char* binary_name)
{
  cout << "Usage:   " << binary_name << " <input trajectory (txt)> <output dmp (xml)> [output policy parameters.txt] [output training directory] [n_basis_functions]" << endl;
  cout << "Example: " << binary_name << " trajectory.txt results/dmp.xml results/policy_parameters.txt results/train/ 10" << endl;
  cout << "Default for n_basis_functions: 5 " << endl;
}

/** Main function
 * \param[in] n_args Number of arguments
 * \param[in] args Arguments themselves
 * \return Success of exection. 0 if successful.
 */
int main(int n_args, char** args)
{
  
  string input_trajectory_file;
  string output_dmp_file;
  string output_train_directory("");
  string output_parameters_file("");
  int n_basis_functions = 5;
  
  if (n_args<3)
  {
    help(args[0]);
    return -1;
  }
  
  if (string(args[1]).compare("--help")==0)
  {
    help(args[0]);
    return 0;
  }

  input_trajectory_file = string(args[1]);
  output_dmp_file = string(args[2]);
  if (n_args>3)
    output_parameters_file = string(args[3]);
  if (n_args>4)
    output_train_directory = string(args[4]);
  if (n_args>5)
    n_basis_functions = atoi(args[5]);
    
  cout << "C++    | Executing "; 
  for (int ii=0; ii<n_args; ii++) cout << " " << args[ii]; 
  cout << endl;
  
  cout << "C++    |     Reading trajectory from file: " << input_trajectory_file << endl;
  int n_gains = 7;
  Trajectory trajectory = Trajectory::readFromFile(input_trajectory_file, n_gains);
  if (trajectory.length()==0)
  {
    cerr << "ERROR: The file " << input_trajectory_file << " could not be found. Aborting." << endl << endl;
    help(args[0]);
    return -1;
  }

  //double tau = trajectory.duration();
  //int n_time_steps = trajectory.length();
  VectorXd ts = trajectory.ts(); // Time steps
  int n_dims = trajectory.dim();

  
  // Initialize some meta parameters for training RBFN function approximator
  int input_dim = 1;
  double intersection = 0.7;
  MetaParametersRBFN* meta_parameters_rbfn = new MetaParametersRBFN(input_dim,n_basis_functions,intersection);      
  FunctionApproximatorRBFN* fa_rbfn = new FunctionApproximatorRBFN(meta_parameters_rbfn);  
  
  // Clone the function approximator for each dimension of the DMP
  vector<FunctionApproximator*> function_approximators(n_dims);    
  for (int dd=0; dd<n_dims; dd++)
    function_approximators[dd] = fa_rbfn->clone();
  
  // Initialize the DMP
  Dmp* dmp = new Dmp(n_dims, function_approximators, Dmp::KULVICIUS_2012_JOINING);
  // Set which parameters to optimize
  set<string> parameters_to_optimize;
  parameters_to_optimize.insert("weights"); // Optimize trajectory
  // Initialize variable for dmp training with or without gains
  bool overwrite = true;
  Eigen::VectorXd parameter_vector;
  std::ofstream ofs(output_dmp_file);
  boost::archive::xml_oarchive oa(ofs);
  bool with_gains = true;
  if (with_gains) {
    MetaParametersLWR* meta_parameters_lwr = new MetaParametersLWR(input_dim, 4,intersection);      
    FunctionApproximatorLWR* fa_lwr = new FunctionApproximatorLWR(meta_parameters_lwr); 

    // Clone the function approximator for each extra dimension of the DMP
    vector<FunctionApproximator*> function_approximators_gains(n_gains);      
    for (int dd=0; dd<n_gains; dd++)
      function_approximators_gains[dd] = fa_lwr->clone();  
    
    DmpWithGainSchedules* dmp_gains = new DmpWithGainSchedules(dmp,function_approximators_gains);

    cout << "C++    |     Training Dmp with gains (n_basis_functions= 4)" << endl;
    overwrite = true;
    dmp_gains->train(trajectory,output_train_directory,overwrite);

    // Add gain parameters
    parameters_to_optimize.insert("offsets_gains");
    parameters_to_optimize.insert("slopes_gains");
    // Set the parameters to optimize
    dmp_gains->setSelectedParameters(parameters_to_optimize);
    
    cout << "C++    |     Writing trained Dmp with Impedance Gains to XML file: " << output_dmp_file << endl;
    oa << boost::serialization::make_nvp("dmp",dmp_gains);
    ofs.close();
    
    // Save the initial parameter vector to file
    dmp_gains->getParameterVector(parameter_vector);
    delete dmp_gains;
    delete meta_parameters_lwr;
    delete fa_lwr;
  }
  else {
    cout << "C++    |     Training Dmp... (n_basis_functions=" << n_basis_functions << ")" << endl;
    overwrite = true;
    dmp->train(trajectory,output_train_directory,overwrite);
    dmp->setSelectedParameters(parameters_to_optimize);
  
    cout << "C++    |     Writing trained Dmp to XML file: " << output_dmp_file << endl;
    oa << boost::serialization::make_nvp("dmp",dmp);
    ofs.close();

    dmp->getParameterVector(parameter_vector);
    delete dmp;
  }
  overwrite = true;
  cout << "C++    |     Writing initial parameter vector to file : " << output_parameters_file << endl;
  saveMatrix(output_parameters_file,parameter_vector,overwrite);
  
  delete meta_parameters_rbfn;
  delete fa_rbfn;
    
  cout << "C++    |     Finished Training" << endl;
  return 0;
}
