def get_func():
    return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
def send_func(vec):
    print("Received vector:", vec)

    return 42, "Processed successfully"

import sys
sys.path.append('/usr/lib/python3.8/site-packages')


import UTTG_interface as ui
A = ui.ArmServoModeInterface()
print(A.init("/workspaces/pure_linux/UTTG/realman/servo.yml", get_func, send_func))
while True:
    A.servoToPoint([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])