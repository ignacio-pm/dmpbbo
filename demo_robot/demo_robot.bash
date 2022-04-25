#!/bin/bash

################################
# STEP 1: Train the DMP with a trajectory

# # Train a dmp with a trajectory, and save it to a file
# rosrun franka_tool_handover step1A_trainDmpFromTrajectoryFile trajectory.txt results/dmp.xml results/policy_parameters.txt results/train/ 15

# # Plot the results of training the dmp
# # python3 step1B_trainDmpFromTrajectoryFilePlot.py results/train/


# ################################
# # STEP 2: Define and save the task

# python3 step2_defineTask.py results/taskPickTool.p

# ## Try rollout

# rosrun franka_tool_handover rollout results/dmp.xml results/trajectory.txt results/executed_policy_parameters.txt
# rosrun franka_tool_handover returnToInitial results/trajectory.txt

# get the plots of the experiment 3

# echo "bash   | Calling rosrun rollout results/policy_parameters.txt results/dmp.xml" &
# rosrun franka_tool_handover rollout results/experiment_3/final/dmp.xml results/experiment_3/final/trajectory.txt results/experiment_3/final/policy_parameters.txt results/experiment_3/final/dmp.xml &
# echo "bash   | Calling rosrun create_cost_vars.py results/cost_vars.txt" &
# rosrun franka_tool_handover create_cost_vars.py results/experiment_3/final/cost_vars.txt &
# # rosrun franka_tool_handover handPub open &
# # echo "bash   | Calling rosrun handPub open" &
# wait

# sleep 0.5

# echo "bash   | Calling rosrun returnToInitial results/trajectory.txt"
# rosrun franka_tool_handover returnToInitial results/experiment_3/final/trajectory.txt
# rosrun franka_tool_handover plot_trajectories.py cost_vars results/experiment_3/final

# sleep 6
# echo "bash   | Calling rosrun franka_tool_handover handPub close"
# rosrun franka_tool_handover handPub close

# ################################
# # STEP 3: Tune the exploration noise

# # Generate some samples to tune the exploration, run the dmp, and plot the results
# # The penultimate parameters is the magnitude of the exploration. Here we try three values

# python3 step3A_tuneExploration.py results/policy_parameters.txt results/distribution_initial_covar.txt results/tune_exploration_5.0/ 5.0 10
# ./step3B_performExplorationRollouts.bash results/dmp.xml results/tune_exploration_5.0/
# python3 step3C_tuneExplorationPlot.py results/tune_exploration_5.0/ results/taskPickTool.p

# python3 step3A_tuneExploration.py results/policy_parameters.txt results/distribution_initial_covar.txt results/tune_exploration_7.0/ 7.0 10
# ./step3B_performExplorationRollouts.bash results/dmp.xml results/tune_exploration_7.0/
# python3 step3C_tuneExplorationPlot.py results/tune_exploration_7.0/ results/taskPickTool.p

# python3 step3A_tuneExploration.py results/policy_parameters.txt results/distribution_initial_covar.txt results/tune_exploration_10.0/ 10.0 10
# ./step3B_performExplorationRollouts.bash results/dmp.xml results/tune_exploration_10.0/
# python3 step3C_tuneExplorationPlot.py results/tune_exploration_10.0/ results/taskPickTool.p

################################
# STEP 4: Run the optimization
 
for i in {0..1}
do
  python3 step4A_oneOptimizationUpdate.py results/experiment_p/
  ./step4B_performRollouts.bash results/experiment_p/  
done

# Plot intermediate results (after 10 updates)
# python3 step4C_plotOptimization.py results/experiment_p/

# for i in {0..4}
# do
#   python3 step4A_oneOptimizationUpdate.py results/
#   ./step4B_performRollouts.bash results/  
# done

# # Plot results (after 8 updates)
# python3 step4C_plotOptimization.py results/

for i in {0..9}
do
  python3 step4A_oneOptimizationUpdate.py results/experiment_giver/
  python3 step4A_oneOptimizationUpdate.py results/experiment_rec/
  ./step4B_performRollouts_2Robots.bash results/experiment_giver/ results/experiment_rec/ 
done

# Plot intermediate results (after 10 updates)
# python3 step4C_plotOptimization.py results/experiment_p/
