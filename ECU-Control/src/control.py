import can
from piracer.vehicles import PiRacerStandard


bus = can.interface.Bus(channel='can0', bustype='socketcan')
piracer = PiRacerStandard()

try:
    while True:
        message = bus.recv()

        data = message.data[1] + message.data[2] * 0.01
        if message.data[0] == 1:
            data *= -1

        if message.arbitration_id == 0:
            piracer.set_steering_percent(data * -0.9)
        if message.arbitration_id == 1:
            piracer.set_throttle_percent(data * 0.9)

except KeyboardInterrupt:
    bus.shutdown()
