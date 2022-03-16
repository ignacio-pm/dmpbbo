if [ "$#" -ne 2 ]; then
    echo "Usage: $0 <input dmp> <directory with rollouts>"
    exit -1
fi

INPUT_DMP=$1
DIRECTORY=$2

source /home/panda/Documents/Ignacio/catkin_ws/devel_isolated/setup.bash

# Get the rollout directory within the update directory
ROLLOUT_DIRS=`find ${DIRECTORY} -type d -name 'rollout*' | sort`
echo "bash   | Rollout directories are ${ROLLOUT_DIRS}"

sleep 2

# Call execute dmp for each rollout
for CUR_DIR in $ROLLOUT_DIRS
do
  echo "bash   | Calling rosrun rollout $INPUT_DMP $CUR_DIR/policy_parameters.txt $CUR_DIR/dmp.xml" &
  rosrun franka_tool_handover rollout $INPUT_DMP $CUR_DIR/trajectory.txt $CUR_DIR/policy_parameters.txt $CUR_DIR/dmp.xml &
  echo "bash   | Calling rosrun create_cost_vars.py $CUR_DIR/cost_vars.txt" &
  rosrun franka_tool_handover create_cost_vars.py $CUR_DIR/cost_vars.txt &
  # rosrun franka_tool_handover handPub open &
  # echo "bash   | Calling rosrun handPub open" &
  wait

  sleep 0.5

  echo "bash   | Calling rosrun returnToInitial $CUR_DIR/trajectory.txt"
  rosrun franka_tool_handover returnToInitial $CUR_DIR/trajectory.txt
  rosrun franka_tool_handover handPub close

  sleep 3

done