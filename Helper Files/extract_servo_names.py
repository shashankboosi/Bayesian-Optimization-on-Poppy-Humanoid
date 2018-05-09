from pypot.creatures import PoppyHumanoid
poppy = PoppyHumanoid()
temp = ' '
poppy_servo_name = []
for i in poppy.motors:
    for j in i.name:
        con = j
        con = con.encode('ascii','ignore')
        if(con == 'x' or con == 'y' or con == 'z'):
            temp = temp + con
            poppy_servo_name.append(temp)
            temp = ' '
        else:
            temp = temp + con
print(poppy_servo_name)
print(len(poppy_servo_name))
