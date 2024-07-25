import machine
import time
import struct

def set_velocity(address, i2c, value):
    packet = struct.pack('@f', value)
    i2c.writeto_mem(address, 0, packet)

def get_control(address, i2c):
        packet = i2c.readfrom_mem(address, 4, 12)
        return struct.unpack('<iq', packet)

i2c = machine.I2C(0, scl=machine.Pin(21), sda=machine.Pin(20), freq=100000)

motors = [0x20, 0x21, 0x22, 0x23]

for motor in motors:

    
    print(f"Testing motor {motor}")
    for target in [-300, -100, 100, 300]:
        _, last_position = get_control(0x20, i2c)

        start_position = last_position
        avg_throttle = 0
        count = 0
        start = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), start) < 10000:
            set_velocity(0x20, i2c, target)
            throttle, position = get_control(0x20, i2c)
            avg_throttle += throttle
            count += 1
            #print(f"{target}, {throttle}, {position}, {(position - last_position)*20}")
            last_position = position
            time.sleep_ms(50)


        print(f"\t{target}: {(last_position - start_position)/10}, {avg_throttle/count}")

        time.sleep_ms(1000)
