#!/usr/bin/python
## run on raspberry pi
## 1. gopigo encoder read 
## 2. encoder counts => radian => angular velocity
## 3. publish lwheel_angular_vel_enc to controller + odometry nodes
import rospy
import roslib

import math 
import numpy

# Messages
from std_msgs.msg import Float32

# Query GoPiGo robot for left and right wheel encoders.
# Publish the estimated left and right angular wheel velocities
class WheelEncoderPublisher:
  def __init__(self):
    rospy.init_node('gopigo_state_updater')
    
    # Read in tangential velocity targets
    self.lwheel_angular_vel_motor_sub = rospy.Subscriber('lwheel_angular_vel_motor', Float32, self.lwheel_angular_vel_motor_callback)
    self.rwheel_angular_vel_motor_sub = rospy.Subscriber('rwheel_angular_vel_motor', Float32, self.rwheel_angular_vel_motor_callback)
    self.lwheel_angular_vel_control_pub = rospy.Subscriber('lwheel_angular_vel_control', Float32, self.lwheel_angular_vel_control_callback)
    self.rwheel_angular_vel_control_pub = rospy.Subscriber('rwheel_angular_vel_control', Float32, self.rwheel_angular_vel_control_callback)
    self.lwheel_angular_vel_enc_pub = rospy.Publisher('lwheel_angular_vel_enc', Float32, queue_size=10)
    self.rwheel_angular_vel_enc_pub = rospy.Publisher('rwheel_angular_vel_enc', Float32, queue_size=10)

    self.rate = rospy.get_param('~rate', 10)
    self.err_tick_incr = rospy.get_param('~err_tick_incr',20) # Filter out clearly erroneous encoder readings
    self.time_prev_update = rospy.Time.now();
    self.gopigo_on = rospy.get_param('~gopigo_on',True)
    if self.gopigo_on:
      import gopigo   
      ## TODO
      ## ENCODER READINGS
      self.lwheel_encs = [gopigo.enc_read(1)]*5
      self.rwheel_encs = [gopigo.enc_read(0)]*5
    self.R = rospy.get_param('~robot_wheel_radius', .03)

    # Need a little hack to incorporate direction wheels are spinning
    self.lwheel_dir = 1;
    self.rwheel_dir = 1;
    self.rwheel_angular_vel_control = 0;
    self.lwheel_angular_vel_control = 0;

  # Really bad hack to get motor spin direction
  def lwheel_angular_vel_motor_callback(self,msg):
    if msg.data >= 0: self.lwheel_dir = 1
    else: self.lwheel_dir = -1
  # Really bad hack to get motor spin direction
  def rwheel_angular_vel_motor_callback(self,msg):
    if msg.data >= 0: self.rwheel_dir = 1
    else: self.rwheel_dir = -1
  def lwheel_angular_vel_control_callback(self,msg):
    self.lwheel_angular_vel_control = msg.data
  def rwheel_angular_vel_control_callback(self,msg):
    self.rwheel_angular_vel_control = msg.data

  ## distance to radian given radius of circle
  def enc_2_rads(self,enc_cms):
    prop_revolution = (enc_cms) / (2.0*math.pi*self.R)
    rads =  prop_revolution * (2*math.pi)
    return rads

  def update(self):
    if self.gopigo_on: # Running on actual robot
      import gopigo
      lwheel_enc = self.lwheel_dir * gopigo.enc_read(1) * .01 # cm's moved
      rwheel_enc = self.rwheel_dir * gopigo.enc_read(0) * .01 # cm's moved

      self.lwheel_encs = self.lwheel_encs[1:] + [lwheel_enc]
      self.rwheel_encs = self.rwheel_encs[1:] + [rwheel_enc]

      # History of past three encoder reading
      time_curr_update = rospy.Time.now()
      dt = (time_curr_update - self.time_prev_update).to_sec()

      # Compute angular velocity in rad/s from encoder count
      ## encoder counts => radian => angular velocity
      lwheel_enc_delta = abs(self.lwheel_encs[-1]) - abs(self.lwheel_encs[-2])
      rwheel_enc_delta = abs(self.rwheel_encs[-1]) - abs(self.rwheel_encs[-2])
      lwheel_angular_vel_enc = self.enc_2_rads(lwheel_enc_delta) / dt
      rwheel_angular_vel_enc = self.enc_2_rads(rwheel_enc_delta) / dt

      # Adjust sign
      if self.lwheel_encs[-1] < 0: lwheel_angular_vel_enc = -lwheel_angular_vel_enc
      if self.rwheel_encs[-1] < 0: rwheel_angular_vel_enc = -rwheel_angular_vel_enc
      self.lwheel_angular_vel_enc_pub.publish(lwheel_angular_vel_enc)
      self.rwheel_angular_vel_enc_pub.publish(rwheel_angular_vel_enc)

      self.time_prev_update = time_curr_update

    else: # Running in simulation -- blindly copy from target assuming perfect execution
      self.lwheel_angular_vel_enc_pub.publish(self.lwheel_angular_vel_control)
      self.rwheel_angular_vel_enc_pub.publish(self.rwheel_angular_vel_control)
      

  def spin(self):
    rospy.loginfo("Start gopigo_state_updater")
    rate = rospy.Rate(self.rate)
    rospy.on_shutdown(self.shutdown)

    while not rospy.is_shutdown():
      self.update();
      rate.sleep()
    rospy.spin()

  def shutdown(self):
    rospy.loginfo("Stop gopigo_state_updater")
    # Stop message
    self.lwheel_angular_vel_enc_pub.publish(0)
    self.rwheel_angular_vel_enc_pub.publish(0)
    rospy.sleep(1)

def main():
  encoder_publisher = WheelEncoderPublisher();
  encoder_publisher.spin()

if __name__ == '__main__':
  main(); 


