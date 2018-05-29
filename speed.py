class Speed:

    def speed_measure(position_difference=0):
        '''
        This method give a speed for every 30 degrees which is not
        optimal to use on a real robot but can try it on a simulated software
        for output evaluation.

        :return:  Speed with which it moves
        '''
        if position_difference > 90:
            if position_difference > 150:
                speed = 6
            elif position_difference > 120 and position_difference <= 150:
                speed = 5
            else:
                speed = 4
        elif position_difference <= 90:
            if position_difference > 60:
                speed = 3
            elif position_difference > 30 and position_difference <= 60:
                speed = 2
            else:
                speed = 1
        return speed

    def dynamic_speed_measure(position_difference):
        '''

        Speed calculation: For every 20 degrees it takes the optlimal time taken is one second

        So for x degrees, it takes x/20 seconds to move.

        input: The difference of the angle from previous position to the present position
        :return: Speed with which the robot should move in that iteration

        '''

        speed = float(position_difference/20)
        return speed