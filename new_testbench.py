from MotorWrapper import *

M = MotorWrapper()

for i in range(8):
    d = input("Give a direction: ")
    v = int(input("Give a speed [-4200, 4200] "))
    if d == "forward":
        M.move_forward(v)
    elif d == "backward":
        M.move_backward(v)
    elif d == "left":
        M.move_left(v)
    elif d == "right":
        M.move_right(v)
    elif d == "up":
        M.move_up(v)
    elif d == "down":
        M.move_down(v)
    elif d == "turn_up":
        M.turn_up(v)
    elif d == "turn_down":
        M.turn_down(v)
    elif d == "turn_left":
        M.turn_left(v)
    elif d == "turn_right":
        M.turn_right(v)
    elif d == "roll_left":
        M.roll_left(v)
    elif d == "roll_right":
        M.roll_right(v)
    elif d == "stop":
        M.stop()
    else:
        print("Invalid direction! Please enter a valid command.")

    M.send_command()