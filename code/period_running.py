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

def period(fxs, baud_rate, time_step=1/100):
	times = []
	i = 0
	mocap_start = 0
	stop= 0


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
	off_time = 60*2
	on_time = 0
	sleep(2)
	print("sampling at "+str(1.0/time_step)+"\n")
	print("Press cntrl-c to stop\n")
	input("Press enter/any key to start")
	sleep(2)
	start_time = time()
	while True:  
		curr_time = time()-start_time
		if curr_time > off_time:
			vel_filtered_0 = np.array([mot_vel_0[0]])
			start_count = 0
			stride_count = np.array(0)
			current_stride = 0
			#moving average window 50 for walking, but 20 for running
			for i in range(1, len(mot_vel_0)):
				if i <= 11:
					vel_filtered_0 = np.append(vel_filtered_0 , np.mean(mot_vel_0[0:i]))
				elif len(mot_vel_0) - i <=  11:
					vel_filtered_0 = np.append(vel_filtered_0, np.mean(mot_vel_0[i-10:len(mot_vel_0)]))
				else:
					vel_filtered_0 = np.append(vel_filtered_0, np.mean(mot_vel_0[i-10:i+11]))

			for i in range(1, len(mot_vel_0)):
				if vel_filtered_0[i] > 0.2:
					start_count = 1
				if start_count == 1 and vel_filtered_0[i-1]  < 0 and vel_filtered_0[i]  > 0:
					current_stride = current_stride + 1
				stride_count = np.append(stride_count, current_stride)
			
			print("Total number of strides ", stride_count[-1])
			for i in (stride_count[-1]-61, stride_count[-1]-1):
				ind = np.where(stride_count == i)[0]
				if i == stride_count[-1]-61:
					stride_time = np.array(times[ind[-1]] - times[ind[0]])
				else:
					stride_time = np.append(stride_time, times[ind[-1]] - times[ind[0]])
			print("Average stride time is ", np.mean(stride_time))
			file1 = open("S01_period.txt", "a+")
			#file1 = truncate(0)
			file1.write("time mot_vel_0 vel_filtered_0 stride_count\n")
			file1.flush()
			for index in range(len(times)):
				file1.write(str(times[index]) + " " + str(mot_vel_0[index]) + " " + str(vel_filtered_0[index])+ " " + str(stride_count[index])+ "\n")
			file1.flush()
			file1.close()
			break
		elif curr_time <= off_time:
			try:
				loop_start = time()
				times.append(curr_time)
				
				act_pack_0=fxs.read_device(dev_id_0)
				act_pack_1=fxs.read_device(dev_id_1)

				if i == 0:
					mot_vel_rad_0 = act_pack_0.mot_vel*((math.pi)/180)*(1/6)
					mot_vel_0 = np.array(mot_vel_rad_0)
				else:
					mot_vel_rad_0 = act_pack_0.mot_vel*((math.pi)/180)*(1/6)
					mot_vel_0 = np.append(mot_vel_0, mot_vel_rad_0)
                  
				loop_end = time()
				if time_step - (loop_end - loop_start) < 0:
					print((loop_end - loop_start))
				else:
					sleep(time_step - (loop_end - loop_start))
				fxs.send_motor_command(dev_id_0, fxe.FX_CURRENT, 0.0)
				fxs.send_motor_command(dev_id_1, fxe.FX_CURRENT, 0.0)
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
	print("test stopped, plotting\n")
	fxs.send_motor_command(dev_id_0, fxe.FX_CURRENT, 0.0)
	fxs.send_motor_command(dev_id_1, fxe.FX_CURRENT, 0.0)
	fxs.set_gains(dev_id_0, 0, 0, 0, 0, 0, 0)
	fxs.set_gains(dev_id_1, 0, 0, 0, 0, 0, 0)
	fxs.send_motor_command(dev_id_0, fxe.FX_NONE, 0)
	fxs.send_motor_command(dev_id_1, fxe.FX_NONE, 0)
	fxs.close(dev_id_0)
	fxs.close(dev_id_1)

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
	period(flex.FlexSEA(), args.baud_rate)


if __name__ == "__main__":
	main()


