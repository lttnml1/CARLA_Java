#!/usr/bin/env python

from numpy.random import default_rng

class NormalDistrib(object):
    def __init__(self, mu, sigma):
        self.rng = default_rng()
        self.mu = mu
        self.sigma = sigma
    
    def draw_samples(self, N):
        return self.rng.normal(self.mu, self.sigma, N)