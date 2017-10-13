#!/usr/bin/env python

if __name__ == "__main__":

   import time
   import pigpio
   import gpio_pin

   GPIO_PIN    = 7
   RUN_TIME    = 120.0
   SAMPLE_TIME = 2.0

   device = pigpio.pi()
   pin = gpio_pin.gpio_pin(device, GPIO_PIN)

   start = time.time()
   while (time.time() - start) < RUN_TIME:

      time.sleep(SAMPLE_TIME)

      print("Time delta={}".format(pin._delta))
      print("Event count={}".format(pin._eventcount))

   # exit cleanly by turning off the pin activities and stopping the device
   pin.cancel()
   device.stop()
