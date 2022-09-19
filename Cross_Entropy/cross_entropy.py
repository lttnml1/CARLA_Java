#!/usr/bin/env python

from normal_distrib import NormalDistrib
import numpy as np
import time
from carla_functions import CarlaScenario

class CrossEntropy(object):
    def __init__(self, N, rho, gamma, distributions):
        self.N = N
        self.rho = rho
        self.gamma = gamma
        self.distributions = distributions
        self.n = len(distributions)
    
    def draw_random_samples(self, num_samples=None):
        if num_samples is None:
            num_samples = self.N
        y=np.empty((num_samples,self.n))
        for i in range(self.n):
            y[:,i] = self.distributions[i].draw_samples(num_samples)
        return y
    
    def calculate_elite(self, y, scores):
        distribution_elites = []
        #print("Original scores/values are:")
        #for i in range(len(y)):
            #print(f"{i}:\t{y[i,:]}\t{scores[i,0]}")
        sorted_scores = np.sort(scores[:,0])
        sorted_score_indices = np.argsort(scores[:,0])
        #print(f"Sorted scores are: {sorted_scores}, indices are: {sorted_score_indices}")
        gamma_index = round(self.rho * self.N)
        #print(f"The gamma element should be at {gamma_index}, which means it's {sorted_scores[gamma_index]}")
        gamma_element = sorted_scores[gamma_index]
        #print(f"Elite set has indicies: {sorted_score_indices[0:gamma_index+1]}")
        elite_set = sorted_score_indices[0:gamma_index+1]
        for i in range(self.n):
            #print(f"**For distribution: {i} **")
            #print(f"This means the elite set is made up of: {y[sorted_score_indices[0:gamma_index+1],i]}")
            distribution_elite = y[sorted_score_indices[0:gamma_index+1],i]
            distribution_elites.append(distribution_elite)
        return gamma_element, distribution_elites
    
    def update_parameters(self, elites):
        for i in range(self.n):
            self.distributions[i].update_params(elites[i])
    
    def print_distribution_parameters(self):
        for i in range(self.n):
            self.distributions[i].print_params()
    
    def execute_ce(self, args):
        gamma = 100
        round = 0
        while(gamma > self.gamma):
            print(f"*****Beginning Round {round}*****")
            round_start_time = time.time()
            y = self.draw_random_samples()
            scores = np.empty(y.shape)
            for i in range(np.shape(y)[0]):
                cs = CarlaScenario()
                ret = cs.execute_scenario(args, y[i,:],"search")
                scores[i,0] = ret[0]
                print(f"Completed\t{i+1}/{np.shape(y)[0]}")
                if ret[1]<0:
                    print("CE Loop cancelled by user!")
                    return
            
            gamma, elites = self.calculate_elite(y, scores)
            self.update_parameters(elites)
            
            self.print_distribution_parameters()
            print(f"Gamma:{gamma}")
            print(f"\n*****Round: {round} took {time.time()-round_start_time}*****")
            round += 1
    
    def demonstrate_and_label(self, args, num_scenarios):
        args.no_render = False
        #draw samples from the final distribution
        y = self.draw_random_samples(num_scenarios)

        #play the scenarios for the user
        for i in range(np.shape(y)[0]):
            cs = CarlaScenario()
            ret = cs.execute_scenario(args, y[i,:],"label")
            #ask the user if they want to save it as a '1'?
            print(f"Completed\t{i+1}/{np.shape(y)[0]}, Score was: {ret[0]}")
            ans = input("Do you want to save this scenario? y/n: ")
            if(ans == 'y'): 
                print("Saving")
                cs.write_features()
            else: print("Discarding")

        #if so, extract features and save to file

        

