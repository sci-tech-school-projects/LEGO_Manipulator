ZOOM_25 = 0.25
ZOOM_50 = 0.5
NETWORK = {'W': 192, 'H': 192, }
CHS = ['ch00', 'ch01', 'ch02', 'ch03', 'ch04', 'ch05']
ARM_STATES = ['APPROACHING', 'TARGETING', 'GRIP-CLOSED', 'LIFTING_UP', 'REACHING', 'GRIP-OPENED', 'LEAVING']
SERVO = {'STR': {'MOVE_PITCH': str(4), 'STEP_SEC': str(0.01), 'STEP_SEC_SLOW': str(0.5)},
         'I_F': {'MOVE_PITCH': 4, 'STEP_SEC': 0.01, 'STEP_SEC_SLOW': 0.5}}
INIT_POS = [65.0, 10.0, -50.0]
GRIP_DEGS = {
    'ch00': {'open': 110, 'close': 91},
    'ch06': {'open': 73, 'close': 90}
}
