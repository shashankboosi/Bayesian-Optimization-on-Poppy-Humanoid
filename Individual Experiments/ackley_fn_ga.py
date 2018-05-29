
#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Find the global maximum on ackley function
'''

from math import sin, cos, pi, pow, exp
import math
from gaft import GAEngine
from gaft.components import BinaryIndividual
from gaft.components import Population
from gaft.operators import TournamentSelection
from gaft.operators import UniformCrossover
from gaft.operators import FlipBitBigMutation
from time import time
# Built-in best fitness analysis.
from gaft.analysis.fitness_store import FitnessStore
from gaft.analysis.console_output import ConsoleOutput
from pypot.creatures import PoppyHumanoid
from pypot.primitive.move import MoveRecorder, MovePlayer
poppy = PoppyHumanoid()
best = 0
count = 0
x_values = []
y_values = []
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

def ackley_fn(x, y):

    global count
    if count is 0:
        prev_x_temp = 0.5
        prev_y_temp = 0
        prev_x_temp = prev_x_temp * 180 - 90
        prev_y_temp = -110 + prev_y_temp
    else:
        prev_x_temp = x_values[count-1]
        prev_y_temp = y_values[count-1]
        prev_x_temp = prev_x_temp * 180 - 90
        prev_y_temp = -(prev_y_temp * 70) - 110
    x_temp = x * 180 - 90
    y_temp = -(y*70) - 110
    x_values.insert(count, x)
    y_values.insert(count, y)
    position_difference_x = abs(x_temp - prev_x_temp)
    position_difference_y = abs(y_temp - prev_y_temp)
    final_flag = False
    poppy_mov = poppy.PoppyBO(next_position_x=x_temp, next_position_y=y_temp,
                              position_difference_x=position_difference_x,
                              position_difference_y=position_difference_y,
                              final_movement=final_flag)
    poppy_mov.multi_dimensional_poppy_robot()
    a = -20 * math.exp(-0.2 * (math.sqrt(0.5 * (math.pow(x, 2) + math.pow(y, 2)))))
    b = -math.exp(0.5 * (math.cos(2 * math.pi * x) + math.cos(2 * math.pi * y)))
    c = math.exp(1)
    d = 20
    result = a + b + c + d
    global best
    if best < abs(result):
        best = abs(result)
        print('The best outcome reward  till now is {}'.format(best))
    count += 1
    return result

search_domain = (-5, 5)
indv_template = BinaryIndividual(ranges=[search_domain, search_domain], eps=0.001)
# Define population
population = Population(indv_template=indv_template, size=50).init()

# Create genetic operators.
#selection = RouletteWheelSelection()
selection = TournamentSelection()
crossover = UniformCrossover(pc=0.8, pe=0.5)
mutation = FlipBitBigMutation(pm=0.1, pbm=0.55, alpha=0.6)

# Create genetic algorithm engine.
# Here we pass all built-in analysis to engine constructor.
engine = GAEngine(population=population, selection=selection,
                  crossover=crossover, mutation=mutation,
                  analysis=[ConsoleOutput, FitnessStore])

# Define fitness function.
@engine.fitness_register
def fitness(indv):
    x, y = indv.solution
    return ackley_fn(x, y)

if '__main__' == __name__:
    engine.run(ng=10)
