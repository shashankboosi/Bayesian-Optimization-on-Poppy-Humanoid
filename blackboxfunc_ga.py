import poppy
import math
import bo

class BlackBox:
    # Black Box function for the robot
    def __init__(self, search_domain, is_two_dimensional=False):
        self.is_two_dimensional = is_two_dimensional
        self.search_domain = search_domain
        self.best = 0
        self.count = 0
        self.poppy_mov = poppy.PoppyBO(count=self.count)
        if self.is_two_dimensional is True:
            self.prev_position_x = 0
            self.prev_position_y = 0
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
                    prev_x_temp = self.prev_position_x
                    prev_y_temp = self.prev_position_y
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
                    prev_x_temp = self.prev_position_x
                    prev_y_temp = self.prev_position_y
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

    def binary_fn(self, x, y):
        '''
        The black_box function is a binary function of form y*sin(2*pi*x) + x*cos(2*pi*y)
        '''
        x_temp, y_temp, prev_x_temp, prev_y_temp = self.init_values(x, y)
        self.prev_position_x = x
        self.prev_position_y = y
        position_difference_x = abs(x_temp - prev_x_temp)
        position_difference_y = abs(y_temp - prev_y_temp)
        final_flag = False
        poppy_mov = poppy.PoppyBO(next_position_x=x_temp, next_position_y=y_temp,
                                  position_difference_x=position_difference_x,
                                  position_difference_y=position_difference_y,
                                  final_movement=final_flag)
        poppy_mov.multi_dimensional_poppy_robot()

        # Calculation of f(X) function
        result = y*math.sin(2*math.pi*x) + x*math.cos(2*math.pi*y)

        if self.best < abs(result):
            best = abs(result)
            print('The best outcome reward is {}'.format(best))
        self.count += 1
        return result

    def ackley_fn(self, x, y):

        x_temp, y_temp, prev_x_temp, prev_y_temp = self.init_values(x, y)
        self.prev_position_x = x
        self.prev_position_y = y
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
        if self.best < abs(result):
            best = abs(result)
            print('The best outcome reward  till now is {}'.format(best))
        self.count += 1
        return result

    def eesom_fn(self, x, y):

        x_temp, y_temp, prev_x_temp, prev_y_temp = self.init_values(x, y)
        self.prev_position_x = x
        self.prev_position_y = y
        position_difference_x = abs(x_temp - prev_x_temp)
        position_difference_y = abs(y_temp - prev_y_temp)
        final_flag = False
        poppy_mov = poppy.PoppyBO(next_position_x=x_temp, next_position_y=y_temp,
                                  position_difference_x=position_difference_x,
                                  position_difference_y=position_difference_y,
                                  final_movement=final_flag)
        poppy_mov.multi_dimensional_poppy_robot()

        a = math.cos(x)
        b = math.cos(y)
        c = math.exp(-(math.pow(x - math.pi, 2) + math.pow(y - math.pi, 2)))
        result = -(a * b * c)
        if self.best < abs(result):
            best = abs(result)
            print('The best outcome reward  till now is {}'.format(best))
        self.count += 1
        return result

    def schaffer_N_2(self, x, y):

        x_temp, y_temp, prev_x_temp, prev_y_temp = self.init_values(x, y)
        self.prev_position_x = x
        self.prev_position_y = y
        position_difference_x = abs(x_temp - prev_x_temp)
        position_difference_y = abs(y_temp - prev_y_temp)
        final_flag = False
        poppy_mov = poppy.PoppyBO(next_position_x=x_temp, next_position_y=y_temp,
                                  position_difference_x=position_difference_x,
                                  position_difference_y=position_difference_y,
                                  final_movement=final_flag)
        poppy_mov.multi_dimensional_poppy_robot()
        a = (math.pow(math.sin(math.pow(x, 2) - math.pow(y, 2)), 2) - 0.5) / (
            math.pow((1 + 0.001 * (math.pow(x, 2) + math.pow(y, 2))), 2))
        result = 0.5 + a
        if self.best < abs(result):
            best = abs(result)
            print('The best outcome reward  till now is {}'.format(best))
        self.count += 1
        return result

    def schaffer_N_4(self, x, y):

        x_temp, y_temp, prev_x_temp, prev_y_temp = self.init_values(x, y)
        self.prev_position_x = x
        self.prev_position_y = y
        position_difference_x = abs(x_temp - prev_x_temp)
        position_difference_y = abs(y_temp - prev_y_temp)
        final_flag = False
        poppy_mov = poppy.PoppyBO(next_position_x=x_temp, next_position_y=y_temp,
                                  position_difference_x=position_difference_x,
                                  position_difference_y=position_difference_y,
                                  final_movement=final_flag)
        poppy_mov.multi_dimensional_poppy_robot()
        a = (math.pow(math.cos(math.sin(abs(math.pow(x, 2) - math.pow(y, 2)))), 2) - 0.5) / (
            math.pow((1 + 0.001 * (math.pow(x, 2) + math.pow(y, 2))), 2))
        result = 0.5 + a
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
        poppy_mov = poppy.PoppyBO(next_position_x=x_temp, position_difference_x=position_difference_x,
                                  final_movement=final_flag)
        poppy_mov.single_dimensional_poppy_robot()
        if x is 0:
            result = 1
        else:
            result = (math.sin(x)/x)
        if self.best < abs(result):
            self.best = abs(result)
            print('The best outcome reward  till now is {}'.format(self.best))
        self.count += 1
        return result




