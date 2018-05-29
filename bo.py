# -*- coding: utf-8 -*-
"""
Created on Wed Apr 11 16:31:44 2018

@author: sboosi
"""

import GPy
import GPyOpt
import poppy
import blackboxfunc

class Bayesian_optimization_main:
    def __init__(self, num_iter, search_domain, is_two_dimensional=False):

        self.is_two_dimensional = is_two_dimensional
        self.num_iter = num_iter
        self.fn = blackboxfunc.BlackBox(self.search_domain, self.is_two_dimensional)
        self.search_domain = search_domain
        if self.is_two_dimensional is True:
            self.x_dict_values = dict()
            self.y_dict_values = dict()
            self.bounds = [{'name': 'x0', 'type': 'continuous', 'domain': self.search_domain},
                           {'name': 'x1', 'type': 'continuous', 'domain': self.search_domain}]
            self.ker = GPy.kern.RBF(input_dim=2, lengthscale=0.1, ARD=False)
            self.myBopt = GPyOpt.methods.BayesianOptimization(f=self.fn.booth_fn,
                                                          domain=self.bounds,
                                                          kernel=self.ker,
                                                          model_type='GP',
                                                          acquisition_type='EI')

        else:
            self.x_values = []
            self.y_values = []
            self.bounds = [
                        {'name': 'x0', 'type': 'continuous', 'domain': self.search_domain}]
            # Declaration of the kernel function
            self.ker2 = GPy.kern.RBF(input_dim=1, lengthscale=0.1, ARD=False)
            self.myBopt = GPyOpt.methods.BayesianOptimization(f=self.fn.probability_density_fn,
                                                          domain=self.bounds,
                                                          kernel=self.ker2,
                                                          model_type='GP',
                                                          initial_design_numdata=1,
                                                          acquisition_type='EI')

        self.myBopt.run_optimization(max_iter=num_iter)
        self.get_results()

    def explored_parameters_fn(self, x, result):
        if self.is_two_dimensional is True:
            self.x_values = x
            self.y_values = result
        else:
            self.x_values.append(x)
            self.y_values.append(result)



    def get_results(self):
        '''
        Displays the final results and plot related to the experiments.
        '''
        if self.is_two_dimensional is False:
            # List of all the positions the robot and explored and exploited
            self.x_values = [i * 180 - 90 for i in self.x_values]
            print('The x observations are', self.x_values)
            print('The y observations are', self.y_values)

            # Final optimal position of the poppy after EXPLOITATION and EXPLORATION
            position = self.myBopt.x_opt * 180 - 90
            prev_position = self.x_values[-1] * 180 - 90
            if position > prev_position:
                difference = position - prev_position
            else:
                difference = abs(position - prev_position)
            final_flag = True
            poppy_mov = poppy.PoppyBO(next_position_x=position, position_difference_x=difference,
                                      final_movement=final_flag)
            poppy_mov.single_dimensional_poppy_robot()

            # Display of the optimal point and it's reward value
            print('The optimal point of function is x -> {}'.format(self.myBopt.x_opt))
            print('The optimal value of function is y -> {}'.format(abs(self.myBopt.fx_opt)))

            # Plotting of the iterations and the consecutiveness between the X's
            self.myBopt.plot_acquisition()
            self.myBopt.plot_convergence()
        else:
            # Final optimal position of the poppy after EXPLOITATION and EXPLORATION
            position_x, position_y = self.myBopt.x_opt
            print('The best x value is {}'.format(position_x))
            print('The best y value is {}'.format(position_y))
            position_x = position_x * 180 - 90
            position_y = -(position_y * 70) - 110
            di_temp = self.x_dict_values.get(self.num_iter + 4)
            prev_position_x, prev_position_y = di_temp[0][0], di_temp[0][1]
            print('The last x value is {}'.format(prev_position_x))
            print('The last y value is {}'.format(prev_position_y))
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
            poppy_mov = poppy.PoppyBO(next_position_x=position_x, next_position_y=position_y,
                        position_difference_x=difference_x,
                        position_difference_y=difference_y,
                        final_movement=flag)
            poppy_mov.multi_dimensional_poppy_robot()
            # Display of the optimal point and it's reward value
            print('The optimal point of function is x -> {}'.format(self.myBopt.x_opt))
            print('The optimal value of function is y -> {}'.format(abs(self.myBopt.fx_opt)))
            # Plotting of the iterations and the consecutiveness between the X's
            self.myBopt.plot_acquisition()
            self.myBopt.plot_convergence()
