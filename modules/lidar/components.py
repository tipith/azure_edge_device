#import numpy as np
import time
import threading
import smbus
import RPi.GPIO as GPIO
import pigpio
from collections import deque

DIR = 15
CLK = 14
EN = 18
PWM_PIN = 12

class Lidar(threading.Thread):
    ADDR = 0x62

    def __init__(self):
        self.bus = smbus.SMBus(1)
        self.deq = deque(maxlen=10)
        super(Lidar, self).__init__()

    def _measure(self):
        self.bus.write_byte_data(Lidar.ADDR,0,4)
        self.bus.write_byte(Lidar.ADDR, 1)
        while self.bus.read_byte(Lidar.ADDR) & 1 != 0:
            continue
        self.bus.write_byte(Lidar.ADDR, 0xf)
        d = self.bus.read_byte(Lidar.ADDR) << 8
        self.bus.write_byte(Lidar.ADDR, 0x10)
        d |= self.bus.read_byte(Lidar.ADDR)
        return d

    def measure(self):
        while True:  # i2c wires are flaky, make retries
            try:
                return self._measure()
            except IOError:
                pass

    def run(self):
        while True:
            self.deq.append(self.measure())

    def distance(self):
        return sum(deq) / len(deq)


class StepperParams:

    def __init__(self, deg_per_step, micro_steps):
        self._degs = deg_per_step
        self.micro_steps = micro_steps
        print(self.deg_per_step)

    @property
    def deg_per_step(self):
        return self._degs / self.micro_steps


def angle_diff(from_angle, to_angle):
    a = from_angle - to_angle
    a -= 360 if a > 180 else 0
    a += 360 if a < -180 else 0
    return a


class Stepper:

    def __init__(self, dir, clk, en, params):
        GPIO.setmode(GPIO.BCM)
        self.dir_pin = dir
        self.clk_pin = clk
        self.en_pin = en
        self.params = params
        self.cum_angle = 0
        GPIO.setup(self.dir_pin, GPIO.OUT)
        GPIO.setup(self.clk_pin, GPIO.OUT)
        GPIO.setup(self.en_pin, GPIO.OUT)
        self.period = 0.0002

    def __del__(self):
       self.disable()
       GPIO.cleanup()

    def enable(self):
       GPIO.output(self.en_pin, GPIO.HIGH)

    def disable(self):
       GPIO.output(self.en_pin, GPIO.LOW)

    def step(self, n):
        for x in range(n):
            GPIO.output(self.clk_pin, GPIO.HIGH)
            time.sleep(self.period / 2)
            GPIO.output(self.clk_pin, GPIO.LOW)
            time.sleep(self.period / 2)
            self.cum_angle += self.params.deg_per_step * (-1 if self.get_dir() == 'cw' else 1)

    @property
    def angle(self):
       a = self.cum_angle % 360
       a = 360 - a if a < 0 else a
       a = a if a < 180 else -(360 - a)
       return a

    def within_stepper_accuracy(self, angle):
        err = abs(self.angle - angle)
        print('pos {:.1f}, target {:.1f}, err {:.1f} goal_err {:.1f} {}'.format(
              self.angle, angle, err, self.params.deg_per_step, err <= self.params.deg_per_step))
        return err <= 2*self.params.deg_per_step

    def to_angle(self, angle):
        dist = angle_diff(self.angle, angle)
        steps = int(abs(dist) / self.params.deg_per_step)
        dir = 'ccw' if dist < 0 else 'cw'
        self.set_dir(dir)
        #print('{:.1f} -> {:.1f}: {:.1f} deg to traverse, {} {} steps'.format(self.angle, angle, dist, dir, steps))
        self.step(steps)
        #while not self.within_stepper_accuracy(angle):
        #    self.step(1)

    def get_dir(self):
        return 'cw' if GPIO.input(self.dir_pin) else 'ccw'

    def set_dir(self, cw_or_ccw='cw'):
        state = GPIO.HIGH if cw_or_ccw == 'cw' else GPIO.LOW
        GPIO.output(self.dir_pin, state)
        time.sleep(self.period / 2)

    def toggle_dir(self):
        new_dir = 'ccw' if self.get_dir() == 'cw' else 'cw'
        self.set_dir(new_dir)


class Servo:
    """
    Angle range 0 .. 180 deg where 90 deg is the nominal angle.
    """
    FREQ = 100
    PULSE_NOM = 0.00152
    PULSE_DIFF = 0.001
    OFFSET = 5
    ANGLE_NOM = 90

    def __init__(self, pwm_pin):
        self.pwm_pin = pwm_pin
        self.period = 1.0 / Servo.FREQ
        self.pi = pigpio.pi()
        self._set_dc(self._calc_dc(angle=Servo.ANGLE_NOM))
        self.angle = Servo.ANGLE_NOM

    def _calc_delay(self, new_angle):
        delay = abs(self.angle - new_angle) / 90.0  # 90 deg movement equals 1.0 sec
        #print('delay', delay)
        return delay

    def _set_dc(self, dc):
        self.pi.hardware_PWM(self.pwm_pin, Servo.FREQ, int(dc * 1e6))

    def _calc_dc(self, angle):
        angle = self.clamp(angle, 0, 180) + Servo.OFFSET - Servo.ANGLE_NOM
        angle *= -1
        pulse = {'min': Servo.PULSE_NOM - Servo.PULSE_DIFF, 'max': Servo.PULSE_NOM + Servo.PULSE_DIFF, 'nom': Servo.PULSE_NOM}
        nominal_dc = pulse['nom'] / self.period
        scale = (pulse['max'] - pulse['min']) / self.period / 180.0
        dc = scale*angle + nominal_dc
        #print('{:.1f} deg -> {:.2f} ({:.3f} / {:.3f} ms)'.format(
        #      angle, dc, 1e3 * dc * self.period, 1e3 * self.period))
        dc = self.clamp(dc, 0.0, 1.0)
        return dc

    def clamp(self, n, minn, maxn):
        return max(min(maxn, n), minn)

    def to_angle(self, angle):
        self._set_dc(self._calc_dc(angle=angle))
        time.sleep(self._calc_delay(angle))
        self.angle = angle


def run(servo, stepper, lidar):
    while True:
        cmd = raw_input('y<angle> / p<angle> / m: ')
        if cmd:
            if cmd[0] == 'y':
                stepper.to_angle(int(cmd[1:]))
            if cmd[0] == 'p':
                servo.to_angle(int(cmd[1:]))
            if cmd[0] == 'm':
                print(lidar.measure())


if __name__ == '__main__':
    stepper = Stepper(DIR, CLK, EN, StepperParams(1.8, 8))
    stepper.enable()
    lidar = Lidar()
    servo = Servo(PWM_PIN)
    try:
        run(servo, stepper, lidar)
    except KeyboardInterrupt:
        stepper.disable()
    del stepper
