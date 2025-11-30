#!/usr/bin/env python3

"""
FlexSEA ActPackPlus Current Control Demo
"""
from statistics import mean
from time import sleep, time
import matplotlib.pyplot as plt

from flexsea import fxUtils as fxu
from flexsea import fxEnums as fxe
from flexsea import flexsea as flex
import math;
import numpy as np

def bilateral_spring_experiment(fxs, baud_rate, exp_time = 5*60, time_step=1/70):
	ports = ['/dev/ttyACM0', '/dev/ttyACM1']
	dev_id_0 = fxs.open(ports[0], baud_rate, log_level=4)
	dev_id_1 = fxs.open(ports[1], baud_rate, log_level=4)
	sleep(0.1)
	fxs.start_streaming(dev_id_0, 100, log_en=True)
	sleep(0.1)
	fxs.start_streaming(dev_id_1, 100, log_en=True)
	sleep(0.1) 
	
	act_pack_0=fxs.read_device(dev_id_0)
	act_pack_1=fxs.read_device(dev_id_1)
	sleep(1)
	fxs.set_gains(dev_id_0, 40, 400, 0, 0, 0, 128)
	fxs.set_gains(dev_id_1, 40, 400, 0, 0, 0, 128)
	sleep(0.1)
	fxs.send_motor_command(dev_id_0, fxe.FX_CURRENT, 0.0)
	fxs.send_motor_command(dev_id_1, fxe.FX_CURRENT, 0.0)


	num_time_steps=int(exp_time/time_step)

	sleep(2)
	print("sampling at "+str(1.0/time_step)+" for "+str(exp_time)+"seconds\n")
	print("Press cntrl-c to stop\n")
	input("Press enter/any key to start")
	sleep(2)
	start_time=time()
	zero_setpoint_0 = 0
	zero_setpoint_1 = 0
	for i in range(num_time_steps):
		if i < 11:
			if i>5:
				zero_0 = act_pack_0.mot_ang
				zero_1 = act_pack_1.mot_ang
				print(i)
				print(str(zero_0) + " " + str(zero_1))
				zero_setpoint_0 = round((zero_setpoint_0*(i-6) + zero_0)/(i-5))
				zero_setpoint_1 = round((zero_setpoint_1*(i-6) + zero_1)/(i-5))
				print(str(zero_setpoint_0) + " " + str(zero_setpoint_1))
			if i == 10:
				print("Test started\n")
		try:
			loop_start = time()
			curr_time=time()-start_time
			
			act_pack_0=fxs.read_device(dev_id_0)
			act_pack_1=fxs.read_device(dev_id_1)
			mot_ang_rad_0=(zero_setpoint_0-act_pack_0.mot_ang)*(1/16383)*2*math.pi*(1/6)
			mot_ang_rad_1=(zero_setpoint_1-act_pack_1.mot_ang)*(1/16383)*2*math.pi*(1/6)

			if curr_time < 5:
				stiffness = curr_time/5*5000/3
			motor_torque_0=stiffness*(mot_ang_rad_0 + mot_ang_rad_1)
			motor_torque_1=stiffness*(mot_ang_rad_0 + mot_ang_rad_1)

			#for safety purposes
			#if motor_torque_0 > 5e3:
			#	motor_torque_0 = 5e3
			#if motor_torque_0 < -5e3:
			#	motor_torque_0 = -5e3
			#if motor_torque_1 > 5e3:
			#	motor_torque_1 = 5e3
			#if motor_torque_1 < -5e3:
			#	motor_torque_1 = -5e3

			current_0=(motor_torque_0)/(0.146)
			current_1=(motor_torque_1)/(0.146)
			fxs.send_motor_command(dev_id_0, fxe.FX_CURRENT, current_0)
			fxs.send_motor_command(dev_id_1, fxe.FX_CURRENT, current_1)

			loop_end = time()
			if time_step - (loop_end - loop_start) < 0:
				print((loop_end - loop_start))
			else:
				sleep(time_step - (loop_end - loop_start))
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
	bilateral_spring_experiment(flex.FlexSEA(), args.baud_rate)


if __name__ == "__main__":
	main()


