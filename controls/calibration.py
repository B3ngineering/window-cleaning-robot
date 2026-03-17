import serial, time

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
time.sleep(0.1)

# 1 bottom right, 2 top left, 3 bottom left, 4 top right
# don't do 4 yet until short is fixed

def move_motors(commands):
    # commands: [motor_id, dir, steps_per_sec, pulses]
    packet = bytes([len(commands)])
    for m in commands:
        speed_enc = m[2] // 10
        packet += bytes([
            m[0],               # motor_id
            m[1],               # direction
            speed_enc >> 8,     # speed high
            speed_enc & 0xFF,   # speed low
            m[3] >> 8,          # steps high
            m[3] & 0xFF,        # steps low
        ])
    print(packet)
    return packet


# Motor 1 moves CW not CCW when commanded to do so [2, 1, 1, 0, 160, 6, 64, 3, 1, 0, 200, 18, 192]

# BL = 3, 0 = wind, 1 = unwind
# TL = 2, 0 = wind, 1 = unwind
# BR = 1, 0 = unwind, 1 = wind
# TR = 4, 0 = unwind, 1 wind
cmd = move_motors([
# [1,1,300,0],
# [2,1,300,0],
# [3,1,300,0],
[4,1,300,1600]
# []
])


print(list(cmd))
ser.write(cmd)
while True:
    line = ser.readline()
    if line:
        print(line)

ser.close()

