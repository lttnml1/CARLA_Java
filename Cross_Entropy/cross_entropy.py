#!/usr/bin/env python

from normal_distrib import NormalDistrib
import numpy as np

class CrossEntropy(object):
    def __init__(self, N, rho, gamma, distributions):
        self.N = N
        self.rho = rho
        self.gamma = gamma
        self.distributions = distributions
        self.n = len(distributions)
    
    def draw_random_samples(self):
        y=np.empty((self.N,self.n))
        for i in range(self.n):
            y[:,i] = self.distributions[i].draw_samples(self.N)
        return y
    
    def calculate_elite(self, y, scores):
        print("Original scores/values are:")
        for i in range(len(y)):
            print(f"{i}:\t{y[i,0]}\t{scores[i,0]}")
        sorted_scores = np.sort(scores[:,0])
        sorted_score_indices = np.argsort(scores[:,0])
        print(f"Sorted scores are: {sorted_scores}, indices are: {sorted_score_indices}")
        gamma_index = round(self.rho * self.N)
        print(f"The gamma element should be at {gamma_index}, which means it's {sorted_scores[gamma_index]}")
        print(f"Elite set has indicies: {sorted_score_indices[0:gamma_index+1]}")
        print(f"This means the elite set is made up of: {y[sorted_score_indices[0:gamma_index+1]]}")