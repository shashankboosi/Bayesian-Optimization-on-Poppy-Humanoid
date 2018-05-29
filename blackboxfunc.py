import poppy
from scipy import stats
import numpy as np
import math
import bo
class BlackBox:
    # Black Box function for the robot
    def __init__(self, search_domain, is_two_dimensional=False):
        self.is_two_dimensional = is_two_dimensional
        self.MAX = 0.95
        self.best = 0
        self.count = 0
        self.search_domain = search_domain
        self.poppy_mov = poppy.PoppyBO(count=self.count)
        if self.is_two_dimensional is True:
            self.logDicx = dict()
            self.logDicy = dict()
        else:
            self.prev_position_x = 0


    def poppy_angle_settings(self):
        if self.search_domain is (0, 1):
            if self.is_two_dimensional is True:
                if self.count is 0:
                    prev_x_temp = 0.5
                    prev_y_temp = 0
                    prev_x_temp = prev_x_temp * 180 - 90
                    prev_y_temp = -110 + prev_y_temp
                else:
                    prev_ind = self.logDicx[self.count - 1]
                    prev_x_temp, prev_y_temp = prev_ind[0][0], prev_ind[0][1]
                    prev_x_temp = prev_x_temp * 180 - 90
                    prev_y_temp = -(prev_y_temp * 70) - 110
                return prev_x_temp, prev_y_temp
            else:
                if self.count is 0:
                    prev_x_temp = 0.5
                    prev_x_temp = prev_x_temp * 180 - 90
                else:
                    prev_x_temp = self.prev_position_x
                    prev_x_temp = prev_x_temp * 180 - 90
                return prev_x_temp
        else:
            search_temp = self.search_domain[1]
            if self.is_two_dimensional is True:
                temp_x = (180/(search_temp*2))
                temp_y = (70/(search_temp*2))
                if self.count is 0:
                    prev_x_temp = 0.5
                    prev_y_temp = 0
                    prev_x_temp = prev_x_temp * 180 - 90
                    prev_y_temp = -110 + prev_y_temp
                else:
                    prev_ind = self.logDicx[self.count - 1]
                    prev_x_temp, prev_y_temp = prev_ind[0][0], prev_ind[0][1]
                    prev_x_temp = (prev_x_temp+search_temp) * temp_x - 90
                    prev_y_temp = -((prev_y_temp+search_temp) * temp_y) - 110
                return prev_x_temp, prev_y_temp
            else:
                temp_x = (180 / (search_temp * 2))
                if self.count is 0:
                    prev_x_temp = 0.5
                    prev_x_temp = prev_x_temp * 180 - 90
                else:
                    prev_x_temp = self.prev_position_x
                    prev_x_temp = (prev_x_temp+search_temp) * temp_x - 90
                return prev_x_temp

    def init_values(self, x, y):
        prev_x_temp, prev_y_temp = self.poppy_angle_settings()
        search_temp = self.search_domain[1]
        if self.search_domain is (0,1):
            x_temp = x * 180 - 90
            y_temp = -(y * 70) - 110

        else:
            temp_x = (180 / (search_temp * 2))
            temp_y = (70 / (search_temp * 2))
            x_temp = (x + search_temp) * temp_x - 90
            y_temp = -((y + search_temp) * temp_y) - 110
        return x_temp, y_temp, prev_x_temp, prev_y_temp

    def probability_density_fn(self, x):
        '''
        result = -(1/math.sqrt(2*math.pi*var))*(math.exp(-((x-mean)**2) / (2*var)))
        '''
        mean = 0.3
        var = 0.2
        prev_x_temp = self.poppy_angle_settings()
        if self.search_domain is (0,1):
            x_temp = x * 180 - 90
        else:
            search_temp = self.search_domain[1]
            temp_x = (180/(search_temp*2))
            x_temp = (prev_x_temp + search_temp) * temp_x - 90
        self.prev_position_x = x
        position_difference = abs(x_temp - prev_x_temp)
        final_flag = False
        poppy_mov = poppy.PoppyBO(next_position_x=x_temp, position_difference_x=position_difference,
                                  final_movement=final_flag)
        poppy_mov.single_dimensional_poppy_robot()

        # Calculation of f(X) function
        result = -stats.norm(mean, var).pdf(x)

        #String the list of explored parameters to the BO function
        bo.Bayesian_optimization_main.explored_parameters_fn(x=x, result=result)
        if self.best < abs(result):
            best = abs(result)
            print('The best outcome reward is {}'.format(best))
        self.count += 1
        return result

    def ackley_fn(self, x):

        x_temp, y_temp, prev_x_temp, prev_y_temp = self.init_values(x, y)
        position_difference_x = abs(x_temp - prev_x_temp)
        position_difference_y = abs(y_temp - prev_y_temp)
        final_flag = False
        poppy_mov = poppy.PoppyBO(next_position_x=x_temp, next_position_y=y_temp,
                                  position_difference_x=position_difference_x,
                                  position_difference_y=position_difference_y,
                                  final_movement=final_flag)
        poppy_mov.multi_dimensional_poppy_robot()

        a = -20*np.exp(-0.2*(np.sqrt(0.5*(math.pow(x[0, 0], 2) + math.pow(x[0, 1], 2)))))
        b = -np.exp(0.5*(math.cos(2*np.pi*x[0, 0]) + math.cos(2*np.pi*x[0, 1])))
        c = np.exp(1)
        d = 20
        result = -(a+b+c+d)
        self.logDicx[self.count] = x
        self.logDicy[self.count] = result
        print('\n Best X: ', self.logDicx[min(self.logDicy, key=self.logDicy.get)])
        print('\n Best Y: ', self.logDicy[min(self.logDicy, key=self.logDicy.get)])
        #String the list of explored parameters to the BO function
        bo.Bayesian_optimization_main.explored_parameters_fn(x=self.logDicx, result=self.logDicy)
        if self.best < abs(result):
            best = abs(result)
            print('The best outcome reward  till now is {}'.format(best))
        self.count += 1
        return result

    def eesom_fn(self, x):

        x_temp, y_temp, prev_x_temp, prev_y_temp = self.init_values(x, y)
        position_difference_x = abs(x_temp - prev_x_temp)
        position_difference_y = abs(y_temp - prev_y_temp)
        final_flag = False
        poppy_mov = poppy.PoppyBO(next_position_x=x_temp, next_position_y=y_temp,
                                  position_difference_x=position_difference_x,
                                  position_difference_y=position_difference_y,
                                  final_movement=final_flag)
        poppy_mov.multi_dimensional_poppy_robot()

        a = math.cos(x[0, 0])
        b = math.cos(x[0, 1])
        c = np.exp(-(math.pow(x[0, 0] - np.pi, 2) + math.pow(x[0, 1] - np.pi, 2)))
        result = a * b * c
        self.logDicx[self.count] = x
        self.logDicy[self.count] = result
        print('\n Best X: ', self.logDicx[min(self.logDicy, key=self.logDicy.get)])
        print('\n Best Y: ', self.logDicy[min(self.logDicy, key=self.logDicy.get)])
        #String the list of explored parameters to the BO function
        bo.Bayesian_optimization_main.explored_parameters_fn(x=self.logDicx, result=self.logDicy)
        if self.best < abs(result):
            best = abs(result)
            print('The best outcome reward  till now is {}'.format(best))
        self.count += 1
        return result

    def schaffer_N_2(self, x):

        x_temp, y_temp, prev_x_temp, prev_y_temp = self.init_values(x, y)
        position_difference_x = abs(x_temp - prev_x_temp)
        position_difference_y = abs(y_temp - prev_y_temp)
        final_flag = False
        poppy_mov = poppy.PoppyBO(next_position_x=x_temp, next_position_y=y_temp,
                                  position_difference_x=position_difference_x,
                                  position_difference_y=position_difference_y,
                                  final_movement=final_flag)
        poppy_mov.multi_dimensional_poppy_robot()
        a = (math.pow(math.sin(math.pow(x[0,0], 2) - math.pow(x[0,1], 2)), 2) - 0.5) / (
            math.pow((1 + 0.001 * (math.pow(x[0, 0], 2) + math.pow(x[0, 1], 2))), 2))
        result = -1 * (0.5 + a)
        self.logDicx[self.count] = x
        self.logDicy[self.count] = result
        print('\n Best X: ', self.logDicx[min(self.logDicy, key=self.logDicy.get)])
        print('\n Best Y: ', self.logDicy[min(self.logDicy, key=self.logDicy.get)])
        #String the list of explored parameters to the BO function
        bo.Bayesian_optimization_main.explored_parameters_fn(x=self.logDicx, result=self.logDicy)
        if self.best < abs(result):
            best = abs(result)
            print('The best outcome reward  till now is {}'.format(best))
        self.count += 1
        return result

    def schaffer_N_4(self, x):

        prev_x_temp, prev_y_temp = self.poppy_angle_settings()
        x_temp = x[0, 0] * 180 - 90
        y_temp = -(x[0, 1]*70) - 110
        position_difference_x = abs(x_temp - prev_x_temp)
        position_difference_y = abs(y_temp - prev_y_temp)
        final_flag = False
        poppy_mov = poppy.PoppyBO(next_position_x=x_temp, next_position_y=y_temp,
                                  position_difference_x=position_difference_x,
                                  position_difference_y=position_difference_y,
                                  final_movement=final_flag)
        poppy_mov.multi_dimensional_poppy_robot()
        a = (math.pow(math.cos(math.sin(abs(math.pow(x[0, 0], 2) - math.pow(x[0, 1], 2)))), 2) - 0.5) / (
            math.pow((1 + 0.001 * (math.pow(x[0, 0], 2) + math.pow(x[0, 1], 2))), 2))
        result = -1 * (0.5 + a)
        self.logDicx[self.count] = x
        self.logDicy[self.count] = result
        print('\n Best X: ', self.logDicx[min(self.logDicy, key=self.logDicy.get)])
        print('\n Best Y: ', self.logDicy[min(self.logDicy, key=self.logDicy.get)])
        #String the list of explored parameters to the BO function
        bo.Bayesian_optimization_main.explored_parameters_fn(x=self.logDicx, result=self.logDicy)
        if self.best < abs(result):
            best = abs(result)
            print('The best outcome reward  till now is {}'.format(best))
        self.count += 1
        return result


    def sinc_fn(self, x):
        prev_x_temp, prev_y_temp = self.poppy_angle_settings()
        if self.search_domain is (0,1):
            x_temp = x * 180 - 90
        else:
            search_temp = self.search_domain[1]
            temp_x = (180/(search_temp*2))
            x_temp = (prev_x_temp + search_temp) * temp_x - 90
        self.prev_position_x = x
        position_difference_x = abs(x_temp - prev_x_temp)
        final_flag = False
        poppy_mov = poppy.PoppyBO(next_position_x=x_temp,position_difference_x=position_difference_x,
                    final_movement=final_flag)
        poppy_mov.single_dimensional_poppy_robot()
        result = -np.sinc(x)
        bo.Bayesian_optimization_main.explored_parameters_fn(x=x, result=result)
        if self.best < abs(result):
            self.best = abs(result)
            print('The best outcome reward  till now is {}'.format(self.best))
        self.count += 1
        return result




