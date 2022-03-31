#!/usr/bin/env python

import numpy as np
import sys
import os

from TaskPickTool import *

if __name__ == '__main__':
    if len(sys.argv) == 2: 
        task_object = TaskPickTool([0.3, 0.3, 0.2, 0.2])
        directory = sys.argv[1]
        print(len([i for i, j, k in os.walk(directory)])-1)
        for l in range(len([i for i, j, k in os.walk(directory)])-1):
            file = directory + 'rollout0' + str(l+1).zfill(2) + '/cost_vars.txt'
            cost_vars = np.loadtxt(file)
            samples = np.zeros((len(cost_vars)))
            costs = task_object.evaluateRollout(cost_vars, samples)
            print("Costs Rollout " + str(l+1) + ": " + str(costs))
    else:
        print("Usage: evaluate_cost_vars.py + rollout_directory")

