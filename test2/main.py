#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile, Image

import os
import time
from sys import stderr

def debug(str1):
    print(str1, end=' ', file=stderr)
# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

# Create your objects here.
ev3 = EV3Brick()

# Write your program here.
def erstes():

    ev3.speaker.beep()

    m_r = Motor(Port.C, Direction.COUNTERCLOCKWISE)
    m_l = Motor(Port.B, Direction.COUNTERCLOCKWISE)

    db = DriveBase(m_l, m_r, wheel_diameter=30, axle_track=140)

    db.straight(distance=50)
    db.turn(90)
    db.straight(distance=-50)



    # play some sound and get angry
    #im = Image('./Angry.bmp')
    im = ImageFile.ANGRY
    ev3.screen.load_image(im)
    ev3.speaker.play_file(SoundFile.CAT_PURR)

    # drive up to a distance of 100 mm
    db.drive(speed=10, turn_rate=0) # start driving
    while abs(db.distance()) < 100:
        time.sleep(0.1)  # wait 100 msec before querying distance again
    db.stop()

#erstes()
    
def drehe_bis_dizzy():
    """
    become dizzy by rotating based on color sensor input
    """
    farbsensor = ColorSensor(Port.S3)
    ev3.screen.load_image(ImageFile.DIZZY)
    while (farbsensor.color() != Color.GREEN):
        time.sleep(0.1)
    db.drive(speed=0, turn_rate=5)
    while (farbsensor.color() != Color.RED):
        time.sleep(0.1)
    db.stop()

# drehe_bis_dizzy()

def fahre_bis_ir():
    """
    drive until ir distance drops below 50%
    """
    ev3.screen.load_image(ImageFile.PINCHED_LEFT)

    infrarot = InfraredSensor(Port.S4)
    db.drive(speed=20, turn_rate=0)
    while (infrarot.distance() > 50):
        time.sleep(0.1)
    db.stop()
    db.turn(180)
    db.straight(150)

# fahre_bis_ir()

def aua():
    """
    i do not like being touched : auaaaa....
    """
    beruehrung = TouchSensor(Port.S1)

    ev3.screen.load_image(ImageFile.NEUTRAL)
    while (beruehrung.pressed() == False):
        time.sleep(0.1)
    ev3.screen.load_image(ImageFile.KNOCKED_OUT)
    ev3.speaker.play_file(SoundFile.BACKING_ALERT)

# aua()

def aua_mit_auszeit(auszeit):
    """
    you can make me angry by touching for as long as a duration of auszeit
    """
    beruehrung = TouchSensor(Port.S1)

    tic = time.time()   # start time

    while (time.time()-tic) < auszeit:
        if beruehrung.pressed():
            ev3.screen.load_image(ImageFile.KNOCKED_OUT)
            ev3.speaker.play_file(SoundFile.BACKING_ALERT)
        else:
            ev3.screen.load_image(ImageFile.NEUTRAL)
        time.sleep(0.1)

# aua_mit_auszeit(auszeit=20)

 
class SensorKopf:
    """
    this class implement the sensor head with a motor that has no encoder
    """
    def __init__(self, arg_correction=None):
        self.dict_head = {'ir': 0, 'touch': -110, 'color': 90}
        self.datei = 'winkel.txt'
        self.angle_ist = 0
        if arg_correction is not None:
            headmotor=Motor(Port.A, Direction.COUNTERCLOCKWISE)
            if type(arg_correction) == str:
                angle = -self.dict_head[arg_correction]
            else:
                angle = arg_correction
            headmotor.run_target(speed=20, target_angle=angle)
        else:
            if self.datei in os.listdir():  # gibt es die Datei?
                debug('lese')
                self._lese_winkel()

        debug('init '+ str(self.angle_ist))

    def _schreibe_winkel(self):
        debug('schdreibe '+str(self.angle_ist))
        with open(self.datei, 'w') as f:
            f.write(str(self.angle_ist))

    def _lese_winkel(self):
        debug('lese '+str(self.angle_ist))
        with open(self.datei, 'r') as f:
            self.angle_ist = int(f.read())

    def kalibriere(self):
        headmotor=Motor(Port.A, Direction.COUNTERCLOCKWISE)
        farbsensor = ColorSensor(Port.S3)
        headmotor.run_until_stalled(speed=10, duty_limit=50) 
        debug('winkel='+str(headmotor.angle()))
        headmotor.run_target(speed=10,target_angle=-120,wait=False)

        while (farbsensor.reflection()<10): # & (headmotor.speed() != 0):
            debug('farbwert='+str(farbsensor.reflection()))
            time.sleep(0.1)

        headmotor.stop()
        headmotor.run_angle(speed=10,rotation_angle=15)
        debug(str(farbsensor.reflection()))

        # winkel auf 0
        headmotor.reset_angle(0)
        self.angle_ist=0
        self._schreibe_winkel()
        # winkel in datei schreiben

    def waehle(self, str_sensor):
        """
        choice of sensor
        """
        headmotor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
        angle_soll = self.dict_head[str_sensor]
        angle_diff = angle_soll-self.angle_ist
        debug('angle '+ str(angle_diff))
        headmotor.run_target(speed=20, target_angle=angle_diff) 
        self.angle_ist = angle_soll
        self._schreibe_winkel()

#sk = SensorKopf(arg_correction='color')
#sk = SensorKopf(arg_correction=70)
sk = SensorKopf()
# sk.kalibriere()
#sensor_wahl('touch')
sk.waehle('color')  # SensorKopf.waehle(sk, 'color')
sk.waehle('ir')
sk.waehle('touch')
time.sleep(2)
sk.waehle('color')
time.sleep(2)
sk.waehle('touch')
time.sleep(2)
sk.waehle('ir')
      
