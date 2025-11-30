#!/usr/bin/env python3

"""
FlexSEA ActPackPlus Current Control Demo
"""
from statistics import mean
from time import sleep, time
import matplotlib.pyplot as plt
import RPi.GPIO as GPIO 

from flexsea import fxUtils as fxu
from flexsea import fxEnums as fxe
from flexsea import flexsea as flex
import math;
import numpy as np

def bilateral_entrainment_experiment(fxs, baud_rate, time_step=1/100):
	# GPIO input 
	GPIO.setwarnings(False)
	GPIO.setmode(GPIO.BOARD)
	GPIO.setup(7, GPIO.IN)

	times = []
	i = 0
	torque_0 = []
	torque_1 = []
	mot_ang_0 = []
	mot_ang_1 = []
	mot_vel_0 = []
	mocap_start = 0
    
	#setting up the motors
	ports = ['/dev/ttyACM0', '/dev/ttyACM1']
	dev_id_0 = fxs.open(ports[0], baud_rate, log_level=6)
	dev_id_1 = fxs.open(ports[1], baud_rate, log_level=6)
	sleep(0.1)
	fxs.start_streaming(dev_id_0, 100, log_en=False)
	sleep(0.1)
	fxs.start_streaming(dev_id_1, 100, log_en=False)
	sleep(0.1)
	
	act_pack_0=fxs.read_device(dev_id_0)
	act_pack_1=fxs.read_device(dev_id_1)
	sleep(1)
	fxs.set_gains(dev_id_0, 40, 400, 0, 0, 0, 96)
	fxs.set_gains(dev_id_1, 40, 400, 0, 0, 0, 96)
	sleep(0.1)
	fxs.send_motor_command(dev_id_0, fxe.FX_CURRENT, 0.0)
	fxs.send_motor_command(dev_id_1, fxe.FX_CURRENT, 0.0)
	
	on_time = 30
	pulse_on_time = 0.2 #in seconds
	period = 0.98-0.025#in seconds
	off_time = 200*period+30
    
	sleep(2)
	print("sampling at "+str(1.0/time_step)+"\n")
	print("Press cntrl-c to stop\n")
	input("Press enter/any key to start")
	sleep(2)
	file1 = open("sample.txt", "a+")
	#file1 = truncate(0)
	file1.write("time mot_ang_0 mot_ang_1 mot_vel_0 torque_0 torque_1\n") #  motor angle data is encoder data 
	file1.flush()
	while True:
		if (GPIO.input(7) == GPIO.LOW) and (mocap_start == 0):
			continue
		elif (GPIO.input(7) == GPIO.HIGH) or (mocap_start == 1): 
			if mocap_start == 0:
				print('Mocap started recording. Experiment began.')
				mocap_start = 1
				start_time=time()                
			try:
				loop_start = time()
				curr_time=time()-start_time
				times.append(curr_time)

				if curr_time >off_time+30: #Time needs to be changed
					print('Time to stop baseline trial. Stop mocap.')
					print('Mocap is over. Saving to a file.')
					mocap_start = 0
					file1.flush()
					file1.close()
					print("Over")
					break

				act_pack_0=fxs.read_device(dev_id_0)
				act_pack_1=fxs.read_device(dev_id_1)
				motor_angle_0=act_pack_0.mot_ang
				motor_angle_1=act_pack_1.mot_ang
				mot_vel_rad_0 = act_pack_0.mot_vel*((math.pi)/180)*(1/6)

				#Apply torque
				if (curr_time >= on_time) and (curr_time <= off_time):
					t = (curr_time - on_time) - int((curr_time - on_time)/period)*period
					if t < pulse_on_time: 
						motor_torque_0 = 8000/6
						motor_torque_1 = 8000/6
					else: 
						motor_torque_0 = 0
						motor_torque_1 = 0
				else:
					motor_torque_0 = 0
					motor_torque_1 = 0
				torque_0.append(motor_torque_0)
				torque_1.append(motor_torque_1)
				mot_ang_0.append(motor_angle_0)
				mot_ang_1.append(motor_angle_1)
				mot_vel_0.append(mot_vel_rad_0)
    

				#for safety purposes
				if motor_torque_0 > 5e3:
					motor_torque_0 = 5e3
				if motor_torque_0 < -5e3:
					motor_torque_0 = -5e3
				if motor_torque_1 > 5e3:
					motor_torque_1 = 5e3
				if motor_torque_1 < -5e3:
					motor_torque_1 = -5e3

				#motor_torque=-damping*motor_velocity_radians
				current_0=(motor_torque_0)/(0.146)
				current_1=(motor_torque_1)/(0.146)
                
				fxs.send_motor_command(dev_id_0, fxe.FX_CURRENT, current_0)
				fxs.send_motor_command(dev_id_1, fxe.FX_CURRENT, current_1)
                
                  
				loop_end = time()
				if time_step - (loop_end - loop_start) < 0:
					print((loop_end - loop_start))
				else:
					sleep(time_step - (loop_end - loop_start))
				file1.write(str(times[i]) + " " + str(mot_ang_0[i]) + " " + str(mot_ang_1[i]) + " " + str(mot_vel_0[i]) + " " + str(torque_0[i]) + " " + str(torque_1[i]) + "\n")
				i = i+1
				
			except KeyboardInterrupt:
				fxs.send_motor_command(dev_id_0, fxe.FX_CURRENT, 0.0)
				fxs.send_motor_command(dev_id_1, fxe.FX_CURRENT, 0.0)
				sleep(0.2)
				print("Test Interrupted, plotting and writing data\n")
				break
				return False

	# When we exit we want the motor to be off
	fxs.send_motor_command(dev_id_0, fxe.FX_CURRENT, 0.0)
	fxs.send_motor_command(dev_id_1, fxe.FX_CURRENT, 0.0)
	sleep(0.5)
	print("test stopped\n")
	fxs.send_motor_command(dev_id_0, fxe.FX_CURRENT, 0.0)
	fxs.send_motor_command(dev_id_1, fxe.FX_CURRENT, 0.0)
	fxs.set_gains(dev_id_0, 0, 0, 0, 0, 0, 0)
	fxs.set_gains(dev_id_1, 0, 0, 0, 0, 0, 0)
	fxs.send_motor_command(dev_id_0, fxe.FX_NONE, 0)
	fxs.send_motor_command(dev_id_1, fxe.FX_NONE, 0)
	fxs.close(dev_id_0)
	fxs.close(dev_id_1)

	
	return True


def main():
	"""
	Standalone current control execution
	"""
	# pylint: disable=import-outside-toplevel
	import argparse

	parser = argparse.ArgumentParser(description=__doc__)
	#parser.add_argument(
	#	"ports", metavar="Ports", type=str, nargs=1, help="Your device serial ports."
	#)
	parser.add_argument(
		"-b",
		"--baud",
		metavar="B",
		dest="baud_rate",
		type=int,
		default=230400,
		help="Serial communication baud rate.",
	)
	args = parser.parse_args()
	
	#experiment_t_delta=input("Please input time between samples (gonna change this to Hz?)")
	bilateral_entrainment_experiment(flex.FlexSEA(), args.baud_rate)


if __name__ == "__main__":
	main()

