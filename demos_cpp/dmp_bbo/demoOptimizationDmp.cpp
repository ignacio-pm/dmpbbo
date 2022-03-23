/**
 * \file demoOptimizationDmp.cpp
 * \author Freek Stulp
 * \brief  Demonstrates how to run an evolution strategy to optimize a Dmp.
 *
 * \ingroup Demos
 * \ingroup DMP_BBO
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

#include <string>
#include <set>
#include <eigen3/Eigen/Core>

#include "dmp_bbo/tasks/TaskViapoint.hpp"
#include "dmp_bbo/TaskSolverDmp.hpp"
#include "dmp_bbo/runOptimizationTask.hpp"

#include "dmp/Dmp.hpp"
#include "functionapproximators/ModelParametersRBFN.hpp"
#include "functionapproximators/FunctionApproximatorRBFN.hpp"

#include "bbo/DistributionGaussian.hpp"
#include "bbo/Updater.hpp"
#include "bbo/updaters/UpdaterCovarDecay.hpp"

#include "dmpbbo_io/EigenFileIO.hpp"

using namespace std;
using namespace Eigen;
using namespace DmpBbo;

/** Main function
 * \param[in] n_args Number of arguments
 * \param[in] args Arguments themselves
 * \return Success of exection. 0 if successful.
 * \todo To focus on the demo code, it would be nice to have the processing of arguments in a separate function.
 */
int main(int n_args, char* args[])
{
  int n_dims = 1;
  string directory;
  if (n_args!=3)
  {
      cout << "Usage: " << args[0] << " n_dims directory" << endl;
      return -1;
  }
  else
  {
    n_dims = atoi(args[1]);
    directory = string(args[2]);
  }

  if (!boost::filesystem::exists(directory))
  {
    cerr << "Directory '" << directory << "' does not exist." << endl;
    cerr << "HINT: The preferred way to run this demo is by calling ";
    cerr << "python3 " << args[0] <<"Wrapper.py, "; 
    cerr << "rather than this binary." << endl; 
    cerr << "Abort." << endl;
    return -1;
  }
  
  // Make the task
  //VectorXd viapoint = VectorXd::Constant(n_dims,2.0);
  //viapoint[0] = 2.5;
  //double viapoint_time = 0.5;
  //if (n_dims==2)
  //  viapoint_time = -1;
  //TaskViapoint* task = new TaskViapoint(viapoint,viapoint_time);
  
  TaskViapoint task = TaskViapoint::readFromFile(directory+"viapoint_task.txt");
  
  // Some DMP parameters
  double tau = 1;
  VectorXd y_init = VectorXd::Constant(n_dims,1.0);
  VectorXd y_attr = VectorXd::Constant(n_dims,3.0);
  
  // Make the initial function approximators (RBFN with zero slopes)
  int n_basis_functions = 8;
  VectorXd centers = VectorXd::LinSpaced(n_basis_functions,0.0,1.0);
  VectorXd widths  = VectorXd::Constant(n_basis_functions,0.2);
  VectorXd weights = VectorXd::Zero(n_basis_functions);
  ModelParametersRBFN* model_parameters = new ModelParametersRBFN(centers,widths,weights);
  vector<FunctionApproximator*> function_approximators(n_dims);
  for (int i_dim=0; i_dim<n_dims; i_dim++)
    function_approximators[i_dim] = new FunctionApproximatorRBFN(model_parameters);
  
  Dmp* dmp = new Dmp(tau, y_init, y_attr, function_approximators, Dmp::KULVICIUS_2012_JOINING);

  // Make the task solver
  set<string> parameters_to_optimize;
  parameters_to_optimize.insert("weights");
  double dt=0.01;
  double integrate_dmp_beyond_tau_factor=1.2;
  bool use_normalized_parameter=true;
  TaskSolverDmp* task_solver = new TaskSolverDmp(dmp,parameters_to_optimize,
                                       dt,integrate_dmp_beyond_tau_factor,use_normalized_parameter);
  // task_solver->set_perturbation(1.0); // Add perturbations
  
  // Make the initial distribution
  VectorXd mean_init;
  dmp->getParameterVector(mean_init);
  
  MatrixXd covar_init = 1000.0*MatrixXd::Identity(mean_init.size(),mean_init.size());

  DistributionGaussian* distribution = new DistributionGaussian(mean_init,covar_init);

  // Make the parameter updater
  double eliteness = 10;
  double covar_decay_factor = 0.8;
  string weighting_method("PI-BB");
  Updater* updater = new UpdaterCovarDecay(eliteness, covar_decay_factor, weighting_method);
  
  
  
  // Run the optimization
  int n_updates = 40;
  int n_samples_per_update = 15;
  bool overwrite = true;
  runOptimizationTask(&task, task_solver, distribution, updater, n_updates, n_samples_per_update,directory,overwrite);
  
}