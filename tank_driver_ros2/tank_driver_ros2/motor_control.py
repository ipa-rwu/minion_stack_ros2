#!/usr/bin/env python

import RPi.GPIO as GPIO

_FREQUENCY = 20
_MAX_PWM = 255
_MIN_PWM = 0
_LOW = 0
_HIGH = 1

def _clip(value, minimum, maximum):
    """Ensure value is between minimum and maximum."""

    if value < minimum:
        return minimum
    elif value > maximum:
        return maximum
    return value

class MotorGPIO:
    def __init__(self, forward_pin, backward_pin, pwm_pin):
        self._forward_pin = forward_pin
        self._backward_pin = backward_pin

        GPIO.setmode(GPIO.BCM)

        GPIO.setup(forward_pin, GPIO.OUT)
        GPIO.setup(backward_pin, GPIO.OUT)
        GPIO.setup(pwm_pin, GPIO.OUT)

        self._pwm = GPIO.PWM(pwm_pin, _FREQUENCY)

    # 0<speed_percent<100
    def move(self, speed_percent):
        speed = _clip(abs(speed_percent), 0, 100)

        # Positive speeds move wheels forward,
        # negative speeds move wheels backward
        if speed_percent < 0:
            GPIO.output(self._backward_pin, _HIGH)
            GPIO.output(self._forward_pin, _LOW)
            self._pwm.start(speed)
        else:
            GPIO.output(self._forward_pin, _HIGH)
            GPIO.output(self._backward_pin, _LOW)
            self._pwm.start(speed)
    
    def stop_motor(self):
        GPIO.output(self._forward_pin, _LOW)
        GPIO.output(self._backward_pin, _LOW)
        self._pwm.start(0)