#!/usr/bin/env python
# -*- coding: utf-8 -*-
# vim: ai ts=4 sts=4 et sw=4 nu

'''
Gauge
=====

The :class:`Gauge` widget is a widget for displaying gauge.

.. note::

Source svg file provided for customing.

'''

__all__ = ('KAYAK ERGO',)

__title__ = 'garden.gauge'
__version__ = '0.2'
__author__ = 'julien@hautefeuille.eu'

import kivy
kivy.require('1.6.0')
from kivy.config import Config
from kivy.app import App
from kivy.clock import Clock
from kivy.properties import NumericProperty
from kivy.properties import StringProperty
from kivy.properties import BoundedNumericProperty
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.widget import Widget
from kivy.uix.scatter import Scatter
from kivy.uix.image import Image
from kivy.uix.label import Label
from os.path import join, dirname, abspath

import pigpio
from gpio_pin import gpio_pin
from datetime import datetime, timedelta
from math import degrees, radians, cos, sin, asin, sqrt, pi
from tcx import tcx_preamble, tcx_trackpoint, tcx_postamble

import os
os.environ['KIVY_WINDOW'] = 'egl_rpi'

class Gauge(Widget):
    '''
    Gauge class

    '''

    unit        = NumericProperty(225/10) # 1 needle tick = 180+45 degrees divided by range
    hrpm        = BoundedNumericProperty(0, min = 0, max = 10, errorvalue = 10)
    speed       = BoundedNumericProperty(0, min = 0, max = 15, errorvalue = 15)
    distance    = BoundedNumericProperty(0, min = 0, max = 50000, errorvalue = 0)
    elapsed     = StringProperty(str(datetime.now()))
    path        = dirname(abspath(__file__))
    file_gauge  = StringProperty(join(path, "lm_tacho_dial_768.png"))
    file_needle = StringProperty(join(path, "lm_tacho_needle_768.png"))
    size_gauge  = BoundedNumericProperty(128, min = 128, max = 768, errorvalue = 128)
    size_text   = NumericProperty(24)

    def __init__(self, **kwargs):
        super(Gauge, self).__init__(**kwargs)

        self._gauge = Scatter(
            size=(self.size_gauge, self.size_gauge),
            do_rotation=False,
            do_scale=False,
            do_translation=False
        )

        _img_gauge = Image(
            source=self.file_gauge,
            size=(self.size_gauge, self.size_gauge)
        )

        self._needle = Scatter(
            size=(self.size_gauge, self.size_gauge),
            do_rotation=False,
            do_scale=False,
            do_translation=False
        )

        _img_needle = Image(
            source=self.file_needle,
            size=(self.size_gauge, self.size_gauge)
        )

        self._speed    = Label(font_size=self.size_text*3, markup=True)
        self._distance = Label(font_size=self.size_text*1.5, markup=True)
        self._elapsed  = Label(font_size=self.size_text*1.5, markup=True)

        self._gauge.add_widget(_img_gauge)
        self._needle.add_widget(_img_needle)

        self.add_widget(self._gauge)
        self.add_widget(self._needle)
        self.add_widget(self._speed)
        self.add_widget(self._distance)
        self.add_widget(self._elapsed)

        self.bind(pos=self._update)
        self.bind(size=self._update)
        self.bind(hrpm=self._turn)
        self.bind(speed=self._turn)
        self.bind(distance=self._turn)
        self.bind(elapsed=self._turn)

    def _update(self, *args):
        '''
        Update gauge and needle positions after sizing or positioning.

        '''
        self._gauge.pos         = self.pos
        self._needle.pos        = (self.x, self.y)
        self._needle.center     = self._gauge.center
        self._speed.center_x    = self._gauge.center_x
        self._speed.center_y    = self._gauge.center_y - (self.size_gauge / 7)
        self._distance.center_x = self._gauge.center_x
        self._distance.center_y = self._gauge.center_y - (self.size_gauge / 4.4)
        self._elapsed.center_x  = self._gauge.center_x
        self._elapsed.center_y  = self._gauge.center_y - (self.size_gauge / 3.4)

    def _turn(self, *args):
        '''
        Turn needle, 1 degree = 1 unit, 0 degree point start on 50 value.

        '''
        self._needle.center_x = self._gauge.center_x
        self._needle.center_y = self._gauge.center_y
        self._needle.rotation = -self.hrpm * self.unit
        self._speed.text      = "[color=5599ff][b]{0:.1f}[/b][/color]".format(self.speed)
        self._distance.text   = "[color=99ff55][b]{0:.0f}m[/b][/color]".format(self.distance)
        self._elapsed.text    = "[b]{}[/b]".format(self.elapsed)


if __name__ == '__main__':

    o = {'lat':  53.8100160, 'lon':   -1.9596520} # oxenhope
    o = {'lat':  21.2765000, 'lon': -157.8460000} # waikiki
    o = {'lat': -34.2452585, 'lon':   18.6372443} # capetown

    R = 6371  # earth radius in km
    r = 1.00  # radius used in xy geometry

    # ------------------------------------------------------------------------------
    # haversine returns the distance between two points {latitude, longitude}
    def haversine(p, q):

        dlat = radians(q['lat'] - p['lat'])
        dlon = radians(q['lon'] - p['lon'])
        lat1 = radians(p['lat'])
        lat2 = radians(q['lat'])

        a = sin(dlat/2)**2 + cos(lat1)*cos(lat2)*sin(dlon/2)**2
        c = 2*asin(sqrt(a))

        return c * R * 1000 # return value in metres

    # ------------------------------------------------------------------------------
    # geometry in the xy plane - here are some nice curves
    def circle(t):
        x = r*cos(t)
        y = r*sin(t)
        return (x, y)

    # lemniscate of bernoulli
    def bernoulli(t):
        x = r*sqrt(2)*cos(t)/(sin(t)*sin(t)+1)
        y = r*sqrt(2)*cos(t)*sin(t)/(sin(t)*sin(t)+1)
        return (x, y)

    # lemniscate of gerono
    def gerono(t):
        x = r*cos(t)
        y = r*cos(t)*sin(t)
        return (x, y)

    def geometry(t):
        #x, y = circle(t)
        x, y = bernoulli(t)
        #x, y = gerono(t)
        return (x, y)

    # ------------------------------------------------------------------------------
    # xy to geographical coordinates
    def geographical(p):
        x, y = p
        # world coordinates R in km -> x,y in km, {lat, lon} in degrees
        lat = o['lat'] + degrees(y/R)
        lon = o['lon'] + degrees(x/(R*cos(radians(o['lat']))))
        return {'lat': lat, 'lon': lon}

    # ------------------------------------------------------------------------------

    TRACKRES = 500
    dtheta   = 2*pi/TRACKRES # assuming curves have 2*pi periodicity
    theta    = 0
    track    = []
    dist     = 0
    q        = {}

    for step in range(TRACKRES):
        # calculate xy coordinates and map to geographical.
        # R in km and geometry x,y in km {lat, lon} in degrees
        p = geographical(geometry(theta))

        if theta > 0: # calculate distance to last point
            dist += haversine(p, q)

        track.append({'lat': p['lat'], 'lon': p['lon'], 'dist': dist})
        theta += dtheta
        q = p

    # total lap closes the loop back to the first point from the last
    lap_distance = dist + haversine(q, geographical(geometry(0)))

    # -------------------------------------------------------------------------

    GPIO_PIN = 7
    device   = pigpio.pi()
    pin      = gpio_pin(device, GPIO_PIN)

    time_start = datetime.now()
    timestamps = []

    trackptr         = 0
    lap_count        = 0

    class GaugeApp(App):

        def build(self):
            box = BoxLayout(orientation='horizontal', padding=5)
            self.gauge = Gauge(hrpm=0, speed=0, distance=0,  size_gauge=768, size_text=25)

            box.add_widget(self.gauge)
            Clock.schedule_interval(lambda *t: self.gauge_update(), 0.10)
            return box

        def gauge_update(self):
            global pin, time_start, track, timestamps, lap_distance, lap_count, trackptr

            time_now = datetime.now()

            time_delta = time_now - time_start
            hour, remr = divmod(time_delta.seconds, 60*60)
            mins, secs = divmod(remr, 60)
            self.gauge.elapsed  = "{:02d}:{:02d}:{:02d}".format(hour, mins, secs)

            pin_delta      = pin._delta
            pin_eventcount = pin._eventcount

            if pin_eventcount > 0:
                # the GPIO pin timer clock is 1 MHz <=> 1 us period
                # count hundreds of rpm, i.e. hrpm = 60*1E6/(100*delta)
                hrpm = 600000 / pin_delta
                # using 750 rpm = 11 kph as a model, kph = rpm * 11/750
                # then kph = 60*1E6/delta * 11/750 = 880000/delta
                kph  = 880000 / pin_delta        # 11 kph = 750 rpm
                # using 60 mins * 750 rpm = 11 km, 1 rev = 11E3/(60*750) metres
                # 1 rev = 11000/(60*750) = 11/45 = 0.244.. m
                dist = pin_eventcount * 0.2444444444

                self.gauge.hrpm     = hrpm
                self.gauge.speed    = kph
                self.gauge.distance = dist

                if dist > (track[trackptr]['dist'] + lap_count*lap_distance):
                    timestamps.append({'time': time_now.strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3], 'speed': kph, 'dist': dist})
                    # let the trackpointer roll over at the end of each lap
                    # but update the lap count too
                    lap_count, trackptr = divmod(len(timestamps), TRACKRES)

            else:
                self.gauge.hrpm     = 0
                self.gauge.speed    = 0
                self.gauge.distance = 0

    GaugeApp().run()

    # grab the final value from the pin
    total_revs = pin._eventcount

    # exit cleanly by turning off the pin activities and stopping the device
    pin.cancel()
    device.stop()

    # -------------------------------------------------------------------------
    # the app has run, now generate the activity file in tcx format

    max_speed = 0
    for x in timestamps:
        if x['speed'] > max_speed:
            max_speed = x['speed']

    total_distance = timestamps[-1]['dist']                            # taken from the last timestamp
    time_start_str = time_start.strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3]
    time_elapsed   = datetime.now() - time_start
    calories       = 1000*time_elapsed.seconds/3600                    # crude calc: 1000 calories/hour
    elevation      = 0
    average_speed  = total_distance / time_elapsed.seconds

    activity = open('activity_{}.tcx'.format(time_start.strftime("%Y%m%d%H%M")), 'w')
    activity.write(tcx_preamble.format(time_start_str, time_start_str, time_elapsed.seconds, total_distance, max_speed, calories))

    for tp in range(len(timestamps)): # loop over all trackpoints reached
        activity.write(tcx_trackpoint.format(timestamps[tp]['time'], track[tp%TRACKRES]['lat'], track[tp%TRACKRES]['lon'], elevation, timestamps[tp]['dist'], timestamps[tp]['speed']))

    activity.write(tcx_postamble.format(average_speed))
    activity.close()

    print("Total distance: {}".format(total_distance))
    hour, remr = divmod(time_elapsed.seconds, 60*60)
    mins, secs = divmod(remr, 60)
    print("Total time: {:02d}:{:02d}:{:02d}".format(hour, mins, secs))
    print("Average speed: {}".format(average_speed))
    print("Total revs: {}".format(total_revs))
    print("Lap length: {}".format(lap_distance))
    print("Total laps: {}".format(total_distance/lap_distance))
    print("File: {}".format('activity_{}.tcx'.format(time_start.strftime("%Y%m%d%H%M"))))
