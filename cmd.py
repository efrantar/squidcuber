# File implementing the actual direct commands that are sent to and then executed
# by the Mindstorm bricks.

import ev3
import struct

# Read the tacho count of a motor
def cmd_tacho(port, var):
    return b''.join([
        ev3.opInput_Device,
        ev3.GET_RAW,
        ev3.LCX(0),
        ev3.port_motor_input(port),
        ev3.GVX(var)
    ])

# Rotate motors for given degrees
def cmd_rotate(ports, deg):
    return b''.join([
        ev3.opOutput_Step_Power,
        ev3.LCX(0),
        ev3.LCX(ports),
        ev3.LCX(100 if deg > 0 else -100),
        ev3.LCX(0),
        ev3.LCX(abs(deg)),
        ev3.LCX(0),
        ev3.LCX(1)
    ])

# Wait for any potentially ongoing rotations to complete.
def cmd_ready(ports):
    return b''.join([
        ev3.opOutput_Ready,
        ev3.LCX(0),
        ev3.LCX(ports)
    ])

# Compute target tacho count for waiting
def cmd_waitdeg_target(deg, waitport, waitdeg, tarvar):
    return cmd_tacho(waitport, tarvar) + b''.join([
        ev3.opAdd32,
        ev3.GVX(tarvar),
        ev3.LCX(waitdeg if deg > 0 else -waitdeg),
        ev3.GVX(tarvar)
    ])

# Wait until target tacho count is reached by a rotating motor
def cmd_waitdeg_wait(deg, waitport, tarvar, waitvar):
    return cmd_tacho(waitport, waitvar) + b''.join([
        ev3.opJr_Lt32 if deg > 0 else ev3.opJr_Gt32,
        ev3.GVX(waitvar),
        ev3.GVX(tarvar),
        ev3.LCX(-9)
    ])

# Return some individual port of a port-bitmask
def some_port(ports):
    return 1 << ((ports & -ports).bit_length() - 1)

# Peform a single face move
def rotate(brick, ports, deg, waitdeg):
    waitport = some_port(ports)
    cmd = cmd_ready(ports)
    cmd += cmd_waitdeg_target(deg, waitport, waitdeg, 0)
    cmd += cmd_rotate(ports, deg)
    cmd += cmd_waitdeg_wait(deg, waitport, 0, 4)
    brick.send_direct_cmd(cmd, global_mem=8)

# Perform an axial move where both sides are rotated by the same abs-degrees
def rotate1(brick, ports1, ports2, deg1, deg2, waitdeg):
    waitport = some_port(ports2)
    cmd = cmd_ready(ports1 + ports2)
    cmd += cmd_waitdeg_target(deg2, waitport, waitdeg, 0)
    cmd += cmd_rotate(ports1, deg1)
    cmd += cmd_rotate(ports2, deg2)
    cmd += cmd_waitdeg_wait(deg2, waitport, 0, 4)
    brick.send_direct_cmd(cmd, global_mem=8)

# Perform an axial move where one side is a half-turn and the other a quarter-turn.
# In this case we want to start the latter turn a little later so that they both
# end jointly and are thus automatically aligned by the next move.
def rotate2(brick, ports1, ports2, deg1, deg2, waitdeg1, waitdeg2):
    waitport = some_port(ports1)
    cmd = cmd_ready(ports1 + ports2)
    cmd += cmd_waitdeg_target(deg1, waitport, waitdeg1, 0)
    cmd += cmd_waitdeg_target(deg1, waitport, waitdeg2, 4)
    cmd += cmd_rotate(ports1, deg1)
    cmd += cmd_waitdeg_wait(deg1, waitport, 0, 8)
    cmd += cmd_rotate(ports2, deg2)
    cmd += cmd_waitdeg_wait(deg1, waitport, 4, 8)
    brick.send_direct_cmd(cmd, global_mem=12)

# Check if a button is pressed
def is_pressed(brick, port):
    cmd = b''.join([
        ev3.opInput_Read,
        ev3.LCX(0),
        ev3.LCX(port),
        ev3.LCX(16),
        ev3.LCX(0),
        ev3.GVX(0) 
    ])
    return struct.unpack('<b', brick.send_direct_cmd(cmd, global_mem=1)[5:])[0] > 0

