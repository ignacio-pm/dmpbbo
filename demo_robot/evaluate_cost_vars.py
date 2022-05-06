#!/usr/bin/env python

import numpy as np
import sys
import os

from TaskHandoverTool import *

def get_costs(file):
    cost_vars = np.loadtxt(file)
    samples = np.zeros((len(cost_vars)))
    costs = task_object.evaluateRollout(cost_vars, samples)
    return costs

if __name__ == '__main__':
    if len(sys.argv) == 2: 
        task_object = TaskHandoverTool([0.4, 0.2, 0.2, 0.2])
        directory = sys.argv[1]
        if len([i for i, j, k in os.walk(directory)]) > 600:
            for l in range(len([i for i, j, k in os.walk(directory)])-1):
                file = directory + 'rollout0' + str(l+1).zfill(2) + '/cost_vars.txt'
                costs = get_costs(file)
                print("Costs Rollout " + str(l+1) + ": " + str(costs))
        else: 
            file = directory + '/cost_vars.txt'
            costs = get_costs(file)
            print("Costs: " + str(costs))
            
    else:
        print("Usage: evaluate_cost_vars.py + rollout_directory or directory with cost_vars.txt")
