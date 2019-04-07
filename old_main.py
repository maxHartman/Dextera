import sys
import readline

from arm import Arm

dextera = Arm()
# WRITE A MAP OF ALL THE COMMANDS?

# dextera.parse_text('rotate out 300 degrees')
# time.sleep(10)
# dextera.parse_text('stop')
# time.sleep(10)
# dextera.parse_text('start moving down')

while True:
    command = input('command: ')
    dextera.parse_text(command)

# when program is stopped, the motor does not stop
