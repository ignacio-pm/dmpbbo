if [ "$#" -ne 2 ]; then
    echo "Usage: $0 <giver directory> <receiver directory>"
    echo "Example: $0 results/experimetnt_giver results/experiment_receiver"
    exit -1
fi

DIRECTORY_GIVER=$1
DIRECTORY_RECEIVER=$1

# Get the most recent update directory
# `tail -n 1` because we only need the last (most recent) one
UPDATE_DIR_GIVER=`find ${DIRECTORY_GIVER} -type d -name 'update*' | sort | tail -n 1`
echo "bash   | Update directory is ${UPDATE_DIR_GIVER}"

echo "bash   | Calling ./robotPerformRollouts.bash $DIRECTORY_GIVER/dmp.xml ${UPDATE_DIR_GIVER} for giver"

# Same for receiver
UPDATE_DIR_RECEIVER=`find ${DIRECTORY_RECEIVER} -type d -name 'update*' | sort | tail -n 1`
echo "bash   | Update directory is ${UPDATE_DIR_RECEIVER}"

echo "bash   | Calling ./robotPerformRollouts.bash $DIRECTORY_RECEIVER/dmp.xml ${UPDATE_DIR_RECEIVER} for giver"

./robotPerformRollouts.bash $DIRECTORY_GIVER/dmp.xml ${UPDATE_DIR_GIVER} &&
./robotPerformRollouts.bash $DIRECTORY_RECEIVER/dmp.xml ${UPDATE_DIR_RECEIVER}