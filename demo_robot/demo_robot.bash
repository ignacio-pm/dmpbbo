#!/bin/bash

################################
# STEP 1: Train the DMP with a trajectory

# # Train a dmp with a trajectory, and save it to a file
# rosrun franka_tool_handover step1A_trainDmpFromTrajectoryFile trajectory.txt results/dmp.xml results/policy_parameters.txt results/train/ 15
# rosrun franka_tool_handover step1A_trainDmpFromTrajectoryFile results/experiment_giver/trajectory.txt results/experiment_giver/dmp.xml results/experiment_giver/policy_parameters.txt results/experiment_giver/train/ 15

# # Plot the results of training the dmp
# # python3 step1B_trainDmpFromTrajectoryFilePlot.py results/experiment_giver/train/


# ################################
# # STEP 2: Define and save the task

# python3 step2_defineTask.py results/experiment_giver/TaskHandoverTool.p

# ## Try rollout

# rosrun franka_tool_handover rollout results/experiment_giver/dmp.xml results/experiment_giver/try_rollout/trajectory.txt
# rosrun franka_tool_handover returnToInitial results/experiment_giver/trajectory.txt

# get the plots of the experiment 3

# echo "bash   | Calling rosrun rollout results/experiment_giver/dmp.xml results/experiment_giver/try_rollout/trajectory.txt" &
# rosrun franka_tool_handover rollout results/experiment_giver/dmp.xml results/experiment_giver/try_rollout/trajectory.txt &
# echo "bash   | Calling rosrun create_cost_vars.py results/cost_vars.txt" &
# rosrun franka_tool_handover create_cost_vars.py results/experiment_giver/try_rollout/cost_vars.txt &
# # rosrun franka_tool_handover handPub open &
# # echo "bash   | Calling rosrun handPub open" &
# wait

# sleep 0.5

# echo "bash   | Calling rosrun returnToInitial results/experiment_giver/trajectory.txt"
# rosrun franka_tool_handover returnToInitial results/experiment_giver/try_rollout/trajectory.txt
# rosrun franka_tool_handover plot_trajectories.py cost_vars results/experiment_giver/try_rollout

# sleep 6
# echo "bash   | Calling rosrun franka_tool_handover handPub close"
# rosrun franka_tool_handover handPub close

# ################################
# # STEP 3: Tune the exploration noise

# # Generate some samples to tune the exploration, run the dmp, and plot the results
# # The penultimate parameters is the magnitude of the exploration. Here we try three values

# python3 step3A_tuneExploration.py results/experiment_giver/policy_parameters.txt results/experiment_giver/distribution_initial_covar.txt results/experiment_giver/tune_exploration_3.0/ 3.0 10
# ./step3B_performExplorationRollouts.bash results/experiment_giver/dmp.xml results/experiment_giver/tune_exploration_3.0/
# python3 step3C_tuneExplorationPlot.py results/experiment_giver/tune_exploration_3.0/ results/experiment_giver/TaskHandoverTool.p

# python3 step3A_tuneExploration.py results/experiment_giver/policy_parameters.txt results/experiment_giver/distribution_initial_covar.txt results/experiment_giver/tune_exploration_5.0/ 5.0 4
# ./step3B_performExplorationRollouts.bash results/experiment_giver/dmp.xml results/experiment_giver/tune_exploration_5.0/
# python3 step3C_tuneExplorationPlot.py results/experiment_giver/tune_exploration_5.0/ results/experiment_giver/TaskHandoverTool.p

# python3 step3A_tuneExploration.py results/experiment_giver/policy_parameters.txt results/experiment_giver/distribution_initial_covar.txt results/experiment_giver/tune_exploration_7.0/ 7.0 4
# ./step3B_performExplorationRollouts.bash results/experiment_giver/dmp.xml results/experiment_giver/tune_exploration_7.0/
# python3 step3C_tuneExplorationPlot.py results/experiment_giver/tune_exploration_7.0/ results/experiment_giver/TaskHandoverTool.p

# python3 step3A_tuneExploration.py results/experiment_giver/policy_parameters.txt results/experiment_giver/distribution_initial_covar.txt results/experiment_giver/tune_exploration_10.0/ 10.0 10
# ./step3B_performExplorationRollouts.bash results/experiment_giver/dmp.xml results/experiment_giver/tune_exploration_10.0/
# python3 step3C_tuneExplorationPlot.py results/experiment_giver/tune_exploration_10.0/ results/experiment_giver/TaskHandoverTool.p

################################
# STEP 4: Run the optimization
 
for i in {0..1}
do
  python3 step4A_oneOptimizationUpdate.py results/experiment_giver/exploration_7/
  ./step4B_performRollouts.bash results/experiment_giver/exploration_7/
done

# Plot intermediate results (after 10 updates)
python3 step4C_plotOptimization.py results/experiment_giver/exploration_7/

# for i in {0..4}
# do
#   python3 step4A_oneOptimizationUpdate.py results/experiment_giver/
#   ./step4B_performRollouts.bash results/experiment_giver/  
# done

# # Plot results (after 8 updates)
# python3 step4C_plotOptimization.py results/experiment_giver/