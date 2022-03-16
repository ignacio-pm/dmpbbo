from cProfile import label
import os
import sys
import numpy as np
import matplotlib.pyplot as plt

lib_path = os.path.abspath('../python')
sys.path.append(lib_path)

from dmp_bbo.Task import Task

class TaskPickTool(Task):
    
    def __init__(self, weigths=[0.6, 0.2, 0.1, 0.1]):
        self.weights_ = weigths
        self.n_dims = 7
        self.n_dims_wrench = 6
    
    def costLabels(self):
        return ['completion','time','forces','acceleration']

    def evaluateRollout(self,cost_vars,sample):
        n_time_steps = cost_vars.shape[0] - 1
        max_time = 10
        max_wrench = 6
        max_acc = 2000
        
        ts = cost_vars[:-1,0]
        wrenches = cost_vars[:-1,1:1+self.n_dims_wrench] 
        accelerations = cost_vars[:-1,1+self.n_dims_wrench:1+self.n_dims_wrench+self.n_dims]
        not_completion = cost_vars[-1,0]
        time = n_time_steps
        
        sum_acc = np.sum(np.square(accelerations)) / n_time_steps
        sum_wrenches = np.sum(np.square(wrenches)) / n_time_steps
            
        costs = np.zeros(1+4)
        costs[1] = self.weights_[0] * not_completion
        costs[2] = self.weights_[1] * time / max_time
        costs[3] = self.weights_[2] * sum_wrenches / max_wrench
        costs[4] = self.weights_[3] * sum_acc / max_acc
        costs[0] = np.sum(costs[1:])
        return costs
        
    def plotRollout(self,cost_vars,ax):
        """Simple script to plot y of DMP trajectory"""
        t = cost_vars[:-1,0]
        wrenches = cost_vars[:-1,1:1+self.n_dims_wrench] 
        accelerations = cost_vars[:-1,1+self.n_dims_wrench:1+self.n_dims_wrench+self.n_dims]
        sum_wrenches = np.sum(np.square(wrenches), axis=1) / np.sum(np.square(wrenches[0]))
        sum_acc = np.sum(np.square(accelerations), axis=1) / np.sum(np.square(accelerations[0]))
        print(np.sum(np.square(wrenches[0])))
        print(np.sum(np.square(accelerations[0])))
    
        
        line_handles = ax.plot(t, sum_wrenches,linewidth=0.5, label="Wrenches")
        line_handles_acc = ax.plot(t, sum_acc, color='red', label="Torques")

        line_handles.extend(line_handles_acc)

        ax.axis('equal')
        ax.set_xlabel('Time')
        ax.set_ylabel('Variation')
        ax.set_xlim([0.0, 5.0])
        ax.set_ylim([-1.0, 4.0])
        ax.legend()
            
        return line_handles
