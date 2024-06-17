import time
import pygame
import can

pygame.init()

pygame.display.set_mode((300, 200))
pygame.display.set_caption("Controller (w, a, s, d)")

bus = can.interface.Bus(channel='can0', bustype='socketcan')

steering_data = None
throttle_data = None
pre_steering_data = None
pre_throttle_data = None

running = True

try:
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        keys = pygame.key.get_pressed()

        if keys[pygame.K_a] and keys[pygame.K_d]:
            steering_data = 0
        elif keys[pygame.K_a]:
            steering_data = -1
        elif keys[pygame.K_d]:
            steering_data = 1
        else:
            steering_data = 0

        if keys[pygame.K_w] and keys[pygame.K_s]:
            throttle_data = 0
        elif keys[pygame.K_w]:
            throttle_data = 1
        elif keys[pygame.K_s]:
            throttle_data = -1
        else:
            throttle_data = 0

        if pre_steering_data is None or pre_steering_data != steering_data:
            pre_steering_data = steering_data

            if steering_data < 0:
                msg = can.Message(arbitration_id=0, data=[1, bool(steering_data), 0, 0])
            else:
                msg = can.Message(arbitration_id=0, data=[0, bool(steering_data), 0, 0])

            bus.send(msg)

        if pre_throttle_data is None or pre_throttle_data != throttle_data:
            pre_throttle_data = throttle_data

            if throttle_data < 0:
                msg = can.Message(arbitration_id=1, data=[1, bool(throttle_data), 0, 0])
            else:
                msg = can.Message(arbitration_id=1, data=[0, bool(throttle_data), 0, 0])

            bus.send(msg)

        time.sleep(0.1)

except KeyboardInterrupt:
    bus.shutdown()
    pygame.quit()
