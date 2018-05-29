import speed
from pypot.creatures import PoppyHumanoid


class PoppyBO:
    def __init__(self, next_position_x=0, next_position_y=0, position_difference_x=0,
                 position_difference_y=0, is_two_dimensional=False, final_movement=False, count=0):

        self.count = count
        self.poppy = PoppyHumanoid()
        self.is_two_dimensional=is_two_dimensional
        if self.is_two_dimensional is True:
            if self.count is 0:
                self.speed_obj = speed.Speed()
                self.initialize_robot()
            else:
                self.position_difference_x = position_difference_x
                self.position_difference_y = position_difference_y
                self.final_movement = final_movement
                self.next_position_y = next_position_x
                self.next_position_y = next_position_y
                self.speed_obj = speed.Speed()
        else:
            if self.count is 0:
                self.initialize_robot()
            else:
                self.position_difference_x = position_difference_x
                self.final_movement = final_movement
                self.next_position_x = next_position_x
                self.speed_obj = speed.Speed()


    def initialize_robot(self):
        '''
        This function initializes robot to move to its base position
        '''
        if self.is_two_dimensional is False:
            i = self.poppy.abs_z.present_position
            abs_z_initializer = 0
            self.position_difference_x = abs(i - abs_z_initializer)
            speed_to_move = self.speed_obj.dynamic_speed_measure(self.position_difference_x)
            self.poppy.abs_z.goto_position(abs_z_initializer, speed_to_move, wait=0.01)
        else:
            i = self.poppy.abs_z.present_position
            j = self.poppy.l_shoulder_y.present_position
            abs_z_initializer = 0
            l_shoulder_y_initializer = -110
            self.position_difference_y = abs(j - l_shoulder_y_initializer)
            self.position_difference_x = abs(i - abs_z_initializer)
            speed_to_move = self.speed_obj.dynamic_speed_measure(self.position_difference_y)
            self.poppy.l_shoulder_y.goto_position(abs_z_initializer, speed_to_move, wait=0.01)
            speed_to_move = self.speed_obj.dynamic_speed_measure(self.position_difference_x)
            self.poppy.abs_z.goto_position(abs_z_initializer, speed_to_move, wait=0.01)


    def search_indication(self):
        '''
        This function depicts the slot machines that the robot has searched and its indicated movements.
        The function also does error handling of the positions.
        '''
        present = self.poppy.r_shoulder_y.present_position
        movement = 10
        self.poppy.r_shoulder_y.goto_position(present + movement, 1, wait=0.01)
        error = abs(self.poppy.r_shoulder_y.present_position - present) - movement
        print(error)
        present = self.poppy.r_shoulder_y.present_position
        self.poppy.r_shoulder_x.goto_position(self.poppy.r_shoulder_x.present_position - 5, 1, wait=0.01)
        self.poppy.r_shoulder_y.goto_position(present - (movement + error), 1, wait=0.01)
        error = abs(self.poppy.r_shoulder_y.present_position - present) - movement - error
        print(error)
        if error > 2 or error < -2:
            print('Error of {} exists during the slot indication with a margin'.format(error))
            self.poppy.r_shoulder_y.goto_position(self.poppy.r_shoulder_y.present_position + 2 * error, 1, wait=0.01)
        self.poppy.r_shoulder_x.goto_position(self.poppy.r_shoulder_x.present_position + 5, 1, wait=0.01)

    def single_dimensional_poppy_robot(self):
        '''
        This method can be used when your black-box function is single_dimensional which only has a single parameter.

        :return: The places that the robot is exploring and exploiting in a one dimensional space.
        '''
        speed_to_move = self.speed_obj.dynamic_speed_measure()
        print('The difference is {} and the speed is {}'.format(self.position_difference_x, speed_to_move))
        self.poppy.abs_z.goto_position(self.next_position_x, speed_to_move, wait=0.01)

        # Maintaining the robots horizontal position
        if self.poppy.abs_z.present_position >= 40:
            self.poppy.abs_x.goto_position(-3, 0.5, wait=0.01)
        elif self.poppy.abs_z.present_position <= -40:
            self.poppy.abs_x.goto_position(3, 0.5, wait=0.01)
        else:
            self.poppy.abs_x.goto_position(0, 0.5, wait=0.01)

        if self.final_movement is False:
            # Exploration stage indicated with single motion
            self.poppy.r_shoulder_y.goto_position(self.poppy.r_shoulder_y.present_position + 10, 1, wait=0.01)
            self.poppy.r_shoulder_y.goto_position(self.poppy.r_shoulder_y.present_position - 10, 1, wait=0.01)
        else:
            # Optimized stage indicated with double motion
            self.poppy.r_shoulder_y.goto_position(self.poppy.r_shoulder_y.present_position + 10, 1, wait=0.01)
            self.poppy.r_shoulder_y.goto_position(self.poppy.r_shoulder_y.present_position - 10, 1, wait=0.01)
            self.poppy.r_shoulder_y.goto_position(self.poppy.r_shoulder_y.present_position + 10, 1, wait=0.01)
            self.poppy.r_shoulder_y.goto_position(self.poppy.r_shoulder_y.present_position - 10, 1, wait=0.01)
        self.poppy.r_shoulder_y.goto_position(self.poppy.l_shoulder_y.present_position, 0.5, wait=0.01)


    def multi_dimensional_poppy_robot(self):
        '''
        This method can be used when your black-box function has two parameters that are used to measure the function.

        If you want to use the number of parameters you can modify the function by adding the position for the third
            parameter and soon
        :return: The places that the robot is exploring and exploiting in a two dimensional space.
        '''
        speed_to_move = self.speed_obj.dynamic_speed_measure(self.position_difference_y)
        self.poppy.l_shoulder_y.goto_position(self.next_position_y, speed_to_move, wait=0.01)
        print('The difference along the y-axis is {} and the speed is {}'.format(self.position_difference_y, speed_to_move))
        speed_to_move = self.speed_obj.dynamic_speed_measure(self.position_difference_x)
        self.poppy.abs_z.goto_position(self.next_position_x, speed_to_move, wait=0.01)
        print('The difference along the x-axis is {} and the speed is {}'.format(self.position_difference_x, speed_to_move))

        # Maintaining the robots horizontal position
        if self.poppy.abs_z.present_position >= 40:
            self.poppy.abs_x.goto_position(-3, 0.5, wait=0.01)
        elif self.poppy.abs_z.present_position <= -40:
            self.poppy.abs_x.goto_position(3, 0.5, wait=0.01)
        else:
            self.poppy.abs_x.goto_position(0, 0.5, wait=0.01)

        if self.final_movement is False:
            # Exploration stage indicated with single motion
            self.search_indication()

        else:
            # Optimized stage indicated with double motion
            self.search_indication()
            self.search_indication()

