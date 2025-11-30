=#!/usr/bin/env python3

"""
FlexSEA ActPackPlus Current Control Demo
"""
from statistics import mean
from time import sleep, time
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline

from flexsea import fxUtils as fxu
from flexsea import fxEnums as fxe
from flexsea import flexsea as flex
import math;
import numpy as np

def bilateral_flexion(fxs, baud_rate, exp_time = 20*60, time_step=1/100):
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
	
	on_time = 0
	off_time = 20*60
	flexion_start_0 = 0
	flexion_start_time_0 = 0
	flexion_start_1 = 0
	flexion_start_time_1 = 0

	moving_avg_window=20
	num_time_steps=int(exp_time/time_step)
	positions=np.zeros((num_time_steps,2))
	torque=np.zeros((num_time_steps,2))
	times=[]
	setpoints=np.zeros((num_time_steps,2))
	currents=np.zeros((num_time_steps,2))
	motor_curr=np.zeros((num_time_steps,2))
	bat_cur=np.zeros((num_time_steps,2))
	ang_rad=np.zeros((num_time_steps,2))
	ang_rad_avg=np.zeros((num_time_steps,2))
	velocities_rad_avg=np.zeros((num_time_steps,2))
	velocities_rad=np.zeros((num_time_steps,2))

	sleep(1)
	print("sampling at "+str(1.0/time_step)+" for "+str(exp_time)+"seconds\n")
	print("Press cntrl-c to stop\n")
	cs = CubicSpline([0,0.2,0.4], [0,6/6*1000,0]) 

	input("Press enter/any key to read the standing zeropoint position")
	sleep(1)
	
	zero_setpoint_0 = 0
	zero_setpoint_1 = 0
	start_time=time()
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
				input("Press enter/any key to start")
				print("Test started\n")
				
		try:
			loop_start = time()
			curr_time=time()-start_time

			act_pack_0=fxs.read_device(dev_id_0)
			act_pack_1=fxs.read_device(dev_id_1)
			motor_angle_0=act_pack_0.mot_ang
			motor_angle_1=act_pack_1.mot_ang
			delta_motor_angle_0=zero_setpoint_0-motor_angle_0
			delta_motor_angle_1=zero_setpoint_1-motor_angle_1
			mot_ang_rad_0=delta_motor_angle_0*(1/16383)*2*math.pi*(1/6)
			mot_ang_rad_1=delta_motor_angle_1*(1/16383)*2*math.pi*(1/6)
			motor_velocity_radians_0=act_pack_0.mot_vel*((math.pi)/180)*(1/6)
			motor_velocity_radians_1=act_pack_1.mot_vel*((math.pi)/180)*(1/6)
			
			#print(motor_angle_0, motor_angle_1)
			velocities_rad[i] = [motor_velocity_radians_0, motor_velocity_radians_1]
			if i<=moving_avg_window:                            
				velocities_rad_avg[i, 0] = np.mean(velocities_rad[:(i+1),0])
				velocities_rad_avg[i, 1] = np.mean(velocities_rad[:(i+1),1])
			elif i>moving_avg_window:    
				velocities_rad_avg[i, 0] = velocities_rad_avg[i-1, 0]+(1/moving_avg_window)*(velocities_rad[i-1, 0]-velocities_rad[i-moving_avg_window-1, 0])
				velocities_rad_avg[i, 1] = velocities_rad_avg[i-1, 1]+(1/moving_avg_window)*(velocities_rad[i-1, 1]-velocities_rad[i-moving_avg_window-1, 1])
			
			#motor_0_high = (zero_setpoint_0 + 6000) - 16383*int((zero_setpoint_0 + 6000) / 16383)
			#motor_0_low = 16383*int((zero_setpoint_0 - 2000) / 16383) + (zero_setpoint_0 - 2000)
			#motor_1_low = 16383*int((zero_setpoint_1 - 6000) / 16383) + (zero_setpoint_1 - 6000)
			#motor_1_high = (zero_setpoint_1 + 2000) - 16383*int((zero_setpoint_1 + 2000) / 16383)
			
			#Check if flexion has started for the right side
			if velocities_rad_avg[i, 0] > 0 and velocities_rad_avg[i-1, 0] < 0 and flexion_start_0 == 0 and motor_angle_0 < zero_setpoint_0 + 8000 and motor_torque_1 == 0:
				#if  motor_angle_0 < motor_0_high or motor_angle_0 > motor_0_low:
				flexion_start_time_0 = curr_time
				flexion_start_0 = 1
			
			if (flexion_start_0) == 1 and (curr_time - flexion_start_time_0 <= 0.4):
				motor_torque_0 =  cs(curr_time - flexion_start_time_0)               
			elif (flexion_start_0) == 1 and (curr_time - flexion_start_time_0 > 0.4):
				motor_torque_0 = 0
				flexion_start_0=0
				flexion_start_time_0 = 0
			elif flexion_start_0 == 0:
				motor_torque_0 = 0



			#Check if flexion has started for the left side
			if velocities_rad_avg[i, 1] < 0 and velocities_rad_avg[i-1, 1] > 0 and flexion_start_1 == 0 and motor_angle_1 > zero_setpoint_1 - 8000 and motor_torque_0 == 0:
				#if motor_angle_1 > motor_1_low or motor_angle_1 < motor_1_high:
				flexion_start_time_1 = curr_time
				flexion_start_1 = 1

			if (flexion_start_1) == 1 and (curr_time - flexion_start_time_1 <= 0.4):
				motor_torque_1 = -cs(curr_time - flexion_start_time_1)               
			elif (flexion_start_1) == 1 and (curr_time - flexion_start_time_1 > 0.4):
				motor_torque_1 = 0
				flexion_start_1=0
				flexion_start_time_1 = 0
			elif flexion_start_1 == 0:
				motor_torque_1 = 0


			torque[i] = [motor_torque_0, motor_torque_1]

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
			

              
			bat_cur[i] = [act_pack_0.batt_curr, act_pack_1.batt_curr]
			#print(bat_cur[i])
			
			currents[i]=[current_0, current_1]
			motor_curr[i]= [act_pack_0.mot_cur, act_pack_1.mot_cur]
			times.append(curr_time)
			positions[i] = [motor_angle_0, motor_angle_1]
			setpoints[i] = [zero_setpoint_0, zero_setpoint_0]
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
	
	#Save everything into a text file
	file1=open("S10_session4.txt","a+")
	#file1.truncate(0)
	file1.write("time position_0 position_1 command_current_0 command_current_1 motor_current_0 motor_current_1\n")
	file1.flush()
	for index in range(len(times)):
		file1.write(str(times[index])+" "+ str(positions[index, 0]) + " "+ str(positions[index, 1]) + " " +str(currents[index, 0])+" "+str(currents[index, 1])+" "+str(motor_curr[index, 0])+" "+str(motor_curr[index, 1])+"\n" )
	file1.flush()
	file1.close()
	print('File saved')
	
	#plotting
	plt.figure()
	plt.plot(times, positions[:,0], color="r",label='position 0')
	plt.plot(times,zero_setpoint_0*np.ones((len(positions[:,0]), 1)), color="b",label='zero_setpoint_0')
	plt.xlabel("time (s)")
	plt.legend(loc="upper right")
	
	plt.figure()
	plt.plot(times, positions[:,1], color="r",label='position 1')
	plt.plot(times,zero_setpoint_1*np.ones((len(positions[:,1]), 1)), color="b",label='zero_setpoint_1')
	plt.xlabel("time (s)")
	plt.legend(loc="upper right")

	
	plt.figure()
	plt.plot(times, torque[:,0], color="r",label="torque 0")
	plt.xlabel("time (s)")
	plt.legend(loc="upper right")
	
	plt.figure()
	plt.plot(times, torque[:,1], color="r",label="torque 1")
	plt.xlabel("time (s)")
	plt.legend(loc="upper right")

	plt.figure()
	plt.plot(times, currents[:,0], color="b",label="commanded current 0")
	plt.xlabel("time (s)")
	plt.legend(loc="upper right")
	
	plt.figure()
	plt.plot(times, currents[:,1], color="b",label="commanded current 1")
	plt.xlabel("time (s)")
	plt.legend(loc="upper right")
	
	plt.figure()
	plt.plot(times, motor_curr[:,0], color="b",label="current 0")
	plt.xlabel("time (s)")
	plt.legend(loc="upper right")
	
	plt.figure()
	plt.plot(times, motor_curr[:,1], color="b",label="current 1")
	plt.xlabel("time (s)")
	plt.legend(loc="upper right")
	plt.show()
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
	bilateral_flexion(flex.FlexSEA(), args.baud_rate)


if __name__ == "__main__":
	main()



