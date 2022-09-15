#!/usr/bin/env python

from numpy.random import default_rng
import numpy as np

class NormalDistrib(object):
    def __init__(self, mu, sigma):
        self.rng = default_rng()
        self.mu = mu
        self.sigma = sigma
    
    def draw_samples(self, N):
        return self.rng.normal(self.mu, self.sigma, N)
    
    def update_params(self, elites):
        self.mu = np.mean(elites)
        self.sigma = np.std(elites)
    
    def print_params(self):
        print(f"Mu:{self.mu},Sigma:{self.sigma}")