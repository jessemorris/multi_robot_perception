#!/usr/bin/env python3

import rospy
import rospkg
import numpy as np
import yaml
import random
import math
import cv2

from scipy.optimize import lsq_linear



def load_results(results_file):
    """[Loads results from yaml file and returns the target vector (results collected from the lidar)
    and the observed results (results collected from monodepth2).]

    Args:
        results_file ([str]): [Path to yaml file]

    Returns:
        [tuple[np.array]]: [observed vector, target vector]
    """
    with open(results_file, "r") as file:
        results = yaml.safe_load(file)

    target_vector = []
    observed_vector = []

    for key, value in results.items():
        rospy.loginfo("Loading data for image {}".format(key))
        data_arrays = value["data"]
        for data in data_arrays:
            lidar_dist = data["lidar_depth"]
            mono_dist = data["mono_depth"]

            target_vector.append(lidar_dist)
            observed_vector.append(mono_dist)

        rospy.loginfo("Loaded ({}, {}) many data points".format(len(observed_vector), len(target_vector)))

    target_vector = np.array(target_vector)

  
    observed_vector = np.array(observed_vector)
    #reshape becuase lsq_linear wants A, shape (m,n) and B, shape (m,)
    # here n = 1 and m is length of data input
    observed_vector = observed_vector.reshape(observed_vector.size, 1)

    return observed_vector, target_vector

def sample_solution(observed, target, sol_x, no_samples=-1, verbose=False):
    """[Tests a random set of points in observed and target using the result sol_x (obtained 
    from the lsq soltion).
    
    The equation used for the error is:
        0.5 * || observed_i * sol_x - target_i || ** 2 
    
    ]

    Args:
        observed ([list]): [List of depth datapoints obtained from mono depth inference]
        target ([list]): [List of depth datapoints obtained from the lidar]
        sol_x ([float]): [the solution from the optimizer]
        no_samples ([int]): [how many data points to sample. If -1 use all the data points] Defaults to -1
        verbose ([bool]): [To print out the error and data points during processing] Defaults to False

    Returns:
        [float]: [The average error]
    """
    zipped_results = list(zip(observed, target))
    
    if no_samples == -1:
        no_to_sample = len(observed) -1
    else:
        no_to_sample = min(no_samples, len(observed) - 1) #make sure we stick within bounds of array

    rospy.loginfo("Sampling {} datapoints".format(no_to_sample))
    test_data = random.sample(zipped_results, no_to_sample)

    assert(len(test_data) == no_to_sample)
    total_error = 0

    for mono_data, lidar_data in test_data:
        error = 0.5 * abs(mono_data * sol_x - lidar_data) ** 2
        total_error += error

        if verbose is True:
            print("Mono depth: {}\nLidar depth: {}\nScaled Mono depth: {}\nError: {}\n\n".format(mono_data, 
                        lidar_data, mono_data * sol_x, error))

    average_error = total_error/no_to_sample

    return average_error


if __name__ == '__main__':
    rospy.init_node("lidar_monodepth2_results_solver")
    rospack = rospkg.RosPack()
    package_path = rospack.get_path("lidar_monodepth2")
    results_file = package_path + "/results/depth_mod.yaml"
    rospy.loginfo("Loading results from file: {}".format(results_file))

    #lsq linear params
    lower_bounds = 0
    upper_bounds = 65536 #2^16
    lsmr_tol = 'auto' #result tolerance
    verbose=1

    data_limit = 20000 #for testing


    observed_vector, target_vector = load_results(results_file)
    print(observed_vector.shape)
    print(target_vector.shape)

    target_vector = target_vector[:data_limit]
    observed_vector = observed_vector[:data_limit]

    res = lsq_linear(observed_vector, target_vector, bounds=(lower_bounds, upper_bounds),
                            lsmr_tol=lsmr_tol, verbose=verbose)

    optimal_x = float(res.x[0])
    sample_size = 1000

    average_error = sample_solution(observed_vector, target_vector, optimal_x, sample_size)
    print("Average error using samples size {} and optimal _x {}: {}".format(sample_size, optimal_x, average_error))
    print("For divisor optimal_x is {}".format(1/optimal_x))

    #lets use x = 0.00041480843832029246




