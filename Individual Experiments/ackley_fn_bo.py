# -*- coding: utf-8 -*-
"""
Created on Wed May 11 16:31:44 2018

@author: sboosi
"""

import GPy
import GPyOpt
import numpy as np
from time import time
import sys
import math
from pypot.creatures import PoppyHumanoid
from pypot.primitive.move import MoveRecorder, MovePlayer
poppy = PoppyHumanoid()
# Global variables
best = 0
count = 0
logDicx = dict()
logDicy = dict()
num_of_iter = 10


def speed_calc(diff):
    '''
   This function is used to dynamically calculate the speed of the robot
   from the difference of prior angle and the posterior angle.
   Input : Difference (angle)

   Output: Speed of the robot in seconds

    '''
    speed = float(diff / 20)
    return speed


def search_indication():
    '''
    This function depicts the slot machines that the robot has searched and its indicated movements.
    The function also does error handling of the positions.
    '''
    present = poppy.r_shoulder_y.present_position
    movement = 10
    poppy.r_shoulder_y.goto_position(present + movement, 1, wait=0.01)
    error = abs(poppy.r_shoulder_y.present_position - present) - movement
    print('The error is {}'.format(error))
    present = poppy.r_shoulder_y.present_position
    poppy.r_shoulder_x.goto_position(poppy.r_shoulder_x.present_position - 5, 1, wait=0.01)
    poppy.r_shoulder_y.goto_position(present - (movement + error), 1, wait=0.01)
    error = abs(poppy.r_shoulder_y.present_position - present) - movement - error
    print('The erroe after the movement is {}'.format(error))
    if error > 2 or error < -2:
        print('Error of {} exists during the slot indication with a margin for the iteration {}'.format(error, count))
        poppy.r_shoulder_y.goto_position(poppy.r_shoulder_y.present_position + 2 * error, 1, wait=0.01)
    poppy.r_shoulder_x.goto_position(poppy.r_shoulder_x.present_position + 5, 1, wait=0.01)


def poppy_robot(next_position_x, next_position_y, position_difference_x,
                position_difference_y, final_movement=False):
    '''
    This method can be used when your black-box function has two parameters that are used to measure the function.

    If you want to use the number of parameters you can modify the function by adding the position for the third
        parameter and soon
    :return: The places that the robot is exploring and exploiting in a two dimensional space.
    '''
    speed_to_move = speed_calc(position_difference_y)
    poppy.r_shoulder_y.goto_position(next_position_y, speed_to_move, wait=0.01)
    print('The difference along the y-axis is {} and the speed is {}'.format(position_difference_y, speed_to_move))
    speed_to_move = speed_calc(position_difference_x)
    poppy.abs_z.goto_position(next_position_x, speed_to_move, wait=0.01)
    print('The difference along the x-axis is {} and the speed is {}'.format(position_difference_x, speed_to_move))

    # Maintaining the robots horizontal position
    if poppy.abs_z.present_position >= 40:
        poppy.abs_x.goto_position(-3, 0.5, wait=0.01)
    elif poppy.abs_z.present_position <= -40:
        poppy.abs_x.goto_position(3, 0.5, wait=0.01)
    else:
        poppy.abs_x.goto_position(0, 0.5, wait=0.01)

    if final_movement is False:
        # Exploration stage indicated with single motion
        search_indication()

    else:
        # Optimized stage indicated with double motion
        search_indication()
        search_indication()

def get_results_final():
    # Final optimal position of the poppy after EXPLOITATION and EXPLORATION
    position_x, position_y = myBopt.x_opt
    print('The best x value is {}'.format(position_x))
    print('The best y value is {}'.format(position_y))
    position_x = position_x * 180 - 90
    position_y = -(position_y * 70) - 110
    di_temp = logDicx.get(num_of_iter + 4)
    prev_position_x, prev_position_y = di_temp[0][0], di_temp[0][1]
    prev_position_x = prev_position_x * 180 - 90
    prev_position_y = -(prev_position_y * 70) - 110
    if position_x > prev_position_x:
        difference_x = position_x - prev_position_x
    else:
        difference_x = abs(position_x - prev_position_x)

    if position_y > prev_position_y:
        difference_y = position_y - prev_position_y
    else:
        difference_y = abs(position_y - prev_position_y)

    flag = True
    poppy_robot(next_position_x=position_x, next_position_y=position_y,
                position_difference_x=difference_x,
                position_difference_y=difference_y,
                final_movement=flag)

    # Display of the optimal point and it's reward value
    print('The optimal point of function is x -> {}'.format(myBopt.x_opt))
    print('The optimal value of function is y -> {}'.format(abs(myBopt.fx_opt)))
    # Plotting of the iterations and the consecutiveness between the X's
    myBopt.plot_acquisition()
    myBopt.plot_convergence()


def get_results_mid(x, y, diff_x, diff_y, result_mid):
    # Final optimal position of the poppy after EXPLOITATION and EXPLORATION
    flag = True
    poppy_robot(next_position_x=x, next_position_y=y,
                position_difference_x=diff_x,
                position_difference_y=diff_y,
                final_movement=flag)
    # Display of the optimal point and it's reward value
    print('The optimal point of function is x -> {} {}'.format(x, y))
    print('The optimal value of function is y -> {}'.format(abs(result_mid)))

def ackley_fn(x):
    global count
    if count is 0:
        prev_x_temp = 0.5
        prev_y_temp = 0
        prev_x_temp = prev_x_temp * 180 - 90
        prev_y_temp = -110 + prev_y_temp
    else:
        prev_ind = logDicx[count - 1]
        prev_x_temp, prev_y_temp = prev_ind[0][0], prev_ind[0][1]
        prev_x_temp = prev_x_temp * 180 - 90
        prev_y_temp = -(prev_y_temp * 70) - 110

    x_temp = x[0, 0] * 180 - 90
    y_temp = -(x[0, 1] * 70) - 110
    # prev_position_x = x[0, 0]
    # prev_position_y = x[0, 1]
    position_difference_x = abs(x_temp - prev_x_temp)
    position_difference_y = abs(y_temp - prev_y_temp)
    final_flag = False
    poppy_robot(next_position_x=x_temp, next_position_y=y_temp,
                position_difference_x=position_difference_x,
                position_difference_y=position_difference_y,
                final_movement=final_flag)

    result = (0.26 * (math.pow(x[0, 0], 2) + math.pow(x[0, 1], 2))) - (0.48 * x[0, 0] * x[0, 1])
    logDicx[count] = x
    logDicy[count] = result
    print('\n Best X: ', logDicx[min(logDicy, key=logDicy.get)])
    print('\n Best Y: ', logDicy[min(logDicy, key=logDicy.get)])
    # String the list of explored parameters to the BO function
    global best
    if best < abs(result):
        best = abs(result)
        print('The best outcome reward  till now is {}'.format(best))
    if best > (0.95*4.675):
        get_results_mid(x_temp, y_temp, position_difference_x, position_difference_y, result)
        sys.exit('I found the maximum')
    count += 1
    # List of all the positions the robot and explored and exploited
    print('The dictionary values are', logDicx)
    return result

'''
The following code is a single-objective declaration and the following functions are defined:
kernel Function (KF): Radial Basis Function
Acquisition Function (AF): Expected Improvement 
Black-Box Function (BBF): f(x) which is a Probabilistic Density Function

Inputs : Inputs are KF, AF, BBF, Length Scale to the kernel,
 			and the number of iteration (Stopping criteria)


Output : 1)The possible uncertain x value(x_value) and their corresponding
			function values (y_value)
		 2) We also get the optimal x value and its reward value in the given iterations.	
'''
t0 = time()
bounds = [
    {'name': 'x0', 'type': 'continuous', 'domain': (-5, 5)},
    {'name': 'x1', 'type': 'continuous', 'domain': (-5, 5)}]
# Declaration of the kernel function
ker2 = GPy.kern.RBF(input_dim=2, lengthscale=0.1, ARD=False)
myBopt = GPyOpt.methods.BayesianOptimization(f=ackley_fn, domain=bounds,
                                             kernel=ker2,
                                             model_type='GP',
                                             acquisition_type='EI')

myBopt.run_optimization(max_iter=num_of_iter)
t1 = time()
total = t1-t0
print('The total time is : {}'.format(total))

get_results_final()
