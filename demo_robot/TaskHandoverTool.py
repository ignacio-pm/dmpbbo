from cProfile import label
import os
import sys
import numpy as np
import matplotlib.pyplot as plt

lib_path = os.path.abspath('../python')
sys.path.append(lib_path)

from dmp_bbo.Task import Task

class TaskHandoverTool(Task):
    
    def __init__(self, weigths=[0.6, 0.2, 0.1, 0.1]):
        self.weights_ = weigths
        self.n_dims = 7
        self.n_dims_wrench = 6
    
    def costLabels(self):
        return ['completion','time','forces','efforts']

    def evaluateRollout(self,cost_vars,sample):
        n_time_steps = cost_vars.shape[0] - 1
        # This parameters are task dependent
        max_time = 6.0
        max_wrench = 70
        max_acc = 2000
        
        ts = cost_vars[:-1,0]
        wrenches = cost_vars[:-1,1:1+self.n_dims_wrench] 
        accelerations = cost_vars[:-1,1+self.n_dims_wrench:1+self.n_dims_wrench+self.n_dims]
        handover_duration = cost_vars[-1,0]
        if handover_duration < max_time and handover_duration > 3:
            time = handover_duration
            not_completion = 0
        else: 
            time = max_time
            not_completion = 1
        
        sum_acc = np.sum(np.square(accelerations)) / n_time_steps
        sum_wrenches = np.sum(np.square(wrenches)) / n_time_steps
            
        costs = np.zeros(1+4)
        costs[1] = self.weights_[0] * min(not_completion, 1)
        costs[2] = self.weights_[1] * min(time / max_time, 1)
        costs[3] = self.weights_[2] * min(sum_wrenches / max_wrench, 1)
        costs[4] = self.weights_[3] * min(sum_acc / max_acc, 1)
        costs[0] = np.sum(costs[1:])

        # Error detected during the movement
        if handover_duration < 0:
            costs[0] = 1.0

        return costs
        
    def plotRollout(self,cost_vars,ax):
        """Simple script to plot y of DMP trajectory"""
        t = cost_vars[:-1,0]
        wrenches = cost_vars[:-1,1:1+self.n_dims_wrench] 
        accelerations = cost_vars[:-1,1+self.n_dims_wrench:1+self.n_dims_wrench+self.n_dims]
        sum_wrenches = np.sum(np.square(wrenches), axis=1) / np.mean(np.square(wrenches))
        sum_acc = np.mean(np.square(accelerations), axis=1)
    
        # line_handles = ax.plot(t, sum_wrenches,linewidth=0.5, label="Wrenches")
        colors = plt.cm.Blues(np.linspace(0, 1, 30))
        if(np.amin(sum_acc) > 50):
            line_handles_acc = ax.plot(t, sum_acc, linewidth=0.5, color=colors[5+len(ax.lines)])

            # line_handles.extend(line_handles_acc)

            # ax.axis('equal')
            ax.set_xlabel('Time')
            ax.set_ylabel('Nm')
            ax.set_ylim([0.0, 600.0])
            ax.set_xlim([0.0, 13.6])
            ax.set_title("Evolution of the mean joint efforts during the optimization")
            # ax.legend()
                
            return line_handles_acc
