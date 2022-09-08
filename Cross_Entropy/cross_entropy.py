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
    
    