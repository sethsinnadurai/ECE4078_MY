from simple_pid import PID
from picamera2 import Picamera2
from flask import Flask, Response, request, jsonify
from gpiozero import Robot, Motor, DigitalInputDevice, Servo
import io
import time
import threading
import pigpio

app = Flask(__name__)


class Encoder(object):
    def __init__(self, pin):
        self._value = 0
        self._last_time = time.time()
        self.debounce_time = 0.0025  # 3ms debounce time
        self.encoder = DigitalInputDevice(pin)
        self.encoder.when_activated = self._increment
        self.encoder.when_deactivated = self._increment
    
    def reset(self):
        self._value = 0
    
    def _increment(self):
        current_time = time.time()
        if current_time - self._last_time > self.debounce_time:
            self._value += 1
            self._last_time = current_time
        
    @property
    def value(self):
        return self._value
        

# main function to control the robot wheels
def move_robot():
    global use_pid, left_speed, right_speed
    flag_new_pid_cycle = True
    flag_new_straight_cycle = True
    flag_new_turning_cycle = True
    while True:
        ### if not using pid, just move the wheels as commanded
        if not use_pid:
            pibot.value = (left_speed, right_speed)
            if (motion == 'stop'):
                left_encoder.reset()
                right_encoder.reset()          
        #Helosss
    
        ### with pid, left wheel is set as reference, and right wheel will try to match the encoder counter of left wheel
        ### pid only runs when robot moves forward or backward. Turning does not use pid
        else:
            if (motion == 'stop') or (motion == 'turning'):
                pibot.value = (left_speed, right_speed) 
                if (motion == 'turning') and flag_new_turning_cycle:
                    left_encoder.reset()
                    right_encoder.reset()
                    flag_new_turning_cycle = False
                if (motion == 'stop'):
                    flag_new_turning_cycle = True

                    
                    
                # if (motion =='stop'):
                #     left_encoder.reset()
                #     right_encoder.reset()
                flag_new_pid_cycle = True 
                flag_new_straight_cycle = True         
            elif(motion == 'forward') or (motion == 'backward'):
                flag_new_turning_cycle = True
                if flag_new_straight_cycle:
                    left_encoder.reset()
                    right_encoder.reset()
                    flag_new_straight_cycle = False
                left_speed, right_speed = abs(left_speed), abs(right_speed)
                if flag_new_pid_cycle:
                    pid_right = PID(kp, ki, kd, setpoint=left_encoder.value, output_limits=(0,1), starting_output=right_speed)
                    flag_new_pid_cycle = False
                pid_right.setpoint = left_encoder.value
                right_speed = pid_right(right_encoder.value)
                if motion == 'forward': pibot.value = (left_speed, right_speed)
                else: pibot.value = (-left_speed, -right_speed)
                # print('Value', left_encoder.value, right_encoder.value)
                # print('Speed', left_speed, right_speed)
        time.sleep(0.005)
    
    
# Receive confirmation whether to use pid or not to control the wheels (forward & backward)
@app.route('/pid')
def set_pid():
    global use_pid, kp, ki, kd
    use_pid = int(request.args.get('use_pid'))
    if use_pid:
        kp, ki, kd = float(request.args.get('kp')), float(request.args.get('ki')), float(request.args.get('kd'))
        return "Using PID"
    else:
        return "Not using PID"
    
# Receive a request to capture and send a snapshot of the picamera
@app.route('/image')
def capture_image():
    stream = io.BytesIO()
    picam2.capture_file(stream, format='jpeg')
    stream.seek(0)
    return Response(stream, mimetype='image/jpeg')
    

 # Receive command to move the pibot
@app.route('/move')
def move():
    global left_speed, right_speed, motion
    left_speed, right_speed = float(request.args.get('left_speed')), float(request.args.get('right_speed'))
    
    if (left_speed == 0 and right_speed == 0):
        motion = 'stop'
    elif (left_speed != right_speed ):
        motion = 'turning'
    elif (left_speed > 0 and right_speed > 0):
        motion = 'forward'
    elif (left_speed < 0 and right_speed < 0):
        motion = 'backward'
    return motion
    
    # if 'time' in request.args:

#***********************************#
# Receive a request for encoder values
@app.route('/encoders')
def get_encoders():
    return jsonify({
        "left_encoder": left_encoder.value,
        "right_encoder": right_encoder.value,
        "motion": motion
    })
#***********************************#
# # Function to set the servo angle
# def set_servo_angle(angle):
#     # servo_value = angle / 90  # Convert degrees to range -1 to 1
#     # servo.value = max(min(servo_value, 1), -1)
#     servo.value = angle
    
# @app.route('/servo')
# def move_servo():
#     angle = float(request.args.get('angle'))  # Get the angle from the client
#     set_servo_angle(angle)
#     return f"Servo set to {angle} degrees"
# Function to set servo angle using pulse width
# def set_servo_angle(angle):
#     # Convert angle to pulse width (500 to 2500 for 0 to 180 degrees)
#     pulsewidth = int(angle)
#     pi.set_servo_pulsewidth(servo_pin, pulsewidth)

# # Flask route to move the servo
# @app.route('/servo')
# def move_servo():
#     angle = float(request.args.get('angle'))  # Get the angle from the client
#     set_servo_angle(angle)
#     return f"Servo set to {angle} degrees"


# Constants
in1 = 17 # may have to change this
in2 = 27 # may have to change this
ena = 18
in3 = 23 # may have to change this
in4 = 24 # may have to change this
enb = 25
enc_a = 26
enc_b = 16
# servo_pin = 12

# Initialize robot and encoders
pibot = Robot(right=Motor(forward=in1, backward=in2, enable=ena), left=Motor(forward=in3, backward=in4, enable=enb))
# Initialise Servo
# servo = Servo(servo_pin)

# Initialize pigpio for servo control
# pi = pigpio.pi()

# Set PWM frequency for the servo (50 Hz for most servos)
# pi.set_PWM_frequency(servo_pin, 50)

# Initialize PID controllers for wheels
left_encoder = Encoder(enc_a)
right_encoder = Encoder(enc_b)
use_pid = 0
kp = 0
ki = 0
kd = 0
left_speed, right_speed = 0, 0
motion = ''

# Initialize the PiCamera
picam2 = Picamera2()
config = picam2.create_preview_configuration(lores={"size": (640,480)})
picam2.configure(config)
picam2.start()
time.sleep(2)

# Initialize flask
def run_flask():
    app.run(host='0.0.0.0', port=5000)
flask_thread = threading.Thread(target=run_flask)
flask_thread.daemon = True
flask_thread.start()

try:
    while True:
        move_robot()
except KeyboardInterrupt:
    pibot.stop()
    picam2.stop()
    # pi.set_servo_pulsewidth(servo_pin, 0)
    print("Program interrupted by user.")