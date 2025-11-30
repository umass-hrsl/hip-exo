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

def bilateral_impedance(fxs, baud_rate, exp_time=30, time_step=1/100):
	ports = ['/dev/ttyACM0', '/dev/ttyACM1']
	dev_id_0 = fxs.open(ports[0], baud_rate, log_level=6)
	dev_id_1 = fxs.open(ports[1], baud_rate, log_level=6)

	fxs.start_streaming(dev_id_0, 100, log_en=False)
	sleep(0.1)
	fxs.start_streaming(dev_id_1, 100, log_en=False)
	sleep(0.1)
	
	act_pack_0=fxs.read_device(dev_id_0)
	act_pack_1=fxs.read_device(dev_id_1)
	
	
	print("Setting controller to current...")
	# Gains are, in order: kp, ki, kd, K, B & ff
	# 30 600 30 0 0 128
	#fxs.set_gains(dev_id_0, 30, 500, 30, 0, 0, 96)
	#fxs.set_gains(dev_id_1, 30, 500, 30, 0, 0, 96)
	fxs.set_gains(dev_id_0, 40, 400, 10, 0, 0, 96)
	fxs.set_gains(dev_id_1, 40, 400, 10, 0, 0, 96)
	sleep(0.5)


	start_time=time()
	
	stiffness_0=500 #mNm/rad
	stiffness_1=500
	damping_0=0 #(mNm*s)/rad
	damping_1=0

	fxs.send_motor_command(dev_id_0, fxe.FX_CURRENT, 0.0)
	fxs.send_motor_command(dev_id_1, fxe.FX_CURRENT, 0.0)
	
	moving_avg_window=170
	num_time_steps=int(exp_time/time_step)
	#the array that holds average velocities as recorded each step (holds filtered velocity values)
	velocities_rad_averages=np.zeros((num_time_steps,2))
	temp_velocity_average=np.zeros(2)
	positions=np.zeros((num_time_steps,2))
	torque=np.zeros((num_time_steps,2))
	times=[]
	setpoints=np.zeros((num_time_steps,2))
	currents=np.zeros((num_time_steps,2))
	motor_curr=np.zeros((num_time_steps,2))
	radians1=np.zeros((num_time_steps,2))
	velocities_deg=np.zeros((num_time_steps,2))
	velocities_rad=np.zeros((num_time_steps,2))
	stride_count = []
	current_stride = 0.0
	last_step_velocity=0.0
	
	sleep(2)
	zero_setpoint_0=act_pack_0.mot_ang
	zero_setpoint_1=act_pack_1.mot_ang
	
	print("sampling at "+str(1.0/time_step)+"for "+str(exp_time)+"seconds\n")
	print("Press cntrl-c to stop\n")
	input("Press enter/any key to start")
	print("Test started\n")
	
	for i in range(num_time_steps):
			
		try:
			sleep(time_step)  # every loop is taking longer, I think that is creating a lagging response
			#T=K*theta
			#delta_motor_angle=setpoint-motor_angle
			#positive displacement: motor angle>setpoint
			#positive displacement: negative delta motor angle, negative motor torque, restoring spring torque
			act_pack_0=fxs.read_device(dev_id_0)
			act_pack_1=fxs.read_device(dev_id_1)
			motor_angle_0=act_pack_0.mot_ang
			motor_angle_1=act_pack_1.mot_ang
			#(motor_angle_1)
			motor_velocity_deg_0=act_pack_0.mot_vel #in degrees: also filtered with a C filter for speed? might need to calculate from motor position
			motor_velocity_deg_1=act_pack_1.mot_vel
			motor_velocity_radians_0=motor_velocity_deg_0*((math.pi)/180)*(1/6)
			motor_velocity_radians_1=motor_velocity_deg_1*((math.pi)/180)*(1/6)

			#filter velocity here
			velocities_deg[i] = [motor_velocity_deg_0, motor_velocity_deg_1]
			velocities_rad[i] = [motor_velocity_radians_0, motor_velocity_radians_1]
			if i<=moving_avg_window:
				temp_velocity_average[0]=np.mean(velocities_rad[:(i+1),0])
				temp_velocity_average[1]=np.mean(velocities_rad[:(i+1),1])
				velocities_rad_averages[i, 0] = temp_velocity_average[0]
				velocities_rad_averages[i, 1] = temp_velocity_average[1]
			elif i>moving_avg_window:
				temp_velocity_average[0]=temp_velocity_average[0]+(1/moving_avg_window)*(velocities_rad[i-1, 0]-velocities_rad[i-moving_avg_window-1, 0])
				temp_velocity_average[1]=temp_velocity_average[1]+(1/moving_avg_window)*(velocities_rad[i-1, 1]-velocities_rad[i-moving_avg_window-1, 1])
				velocities_rad_averages[i, 0] = temp_velocity_average[0]
				velocities_rad_averages[i, 1] = temp_velocity_average[1]
				
			if i<=500:
				temp_velocity_step=mean(velocities_rad[:(i+1),0])
				
			elif i>500:
				temp_velocity_step=temp_velocity_step+(1/500)*(velocities_rad[i-1, 0]-velocities_rad[i-500-1, 0])
				
			delta_motor_angle_0=zero_setpoint_0-motor_angle_0
			delta_motor_angle_1=zero_setpoint_1-motor_angle_1
			mot_ang_rad_0=delta_motor_angle_0*(1/16383)*2*math.pi*(1/6)
			mot_ang_rad_1=delta_motor_angle_1*(1/16383)*2*math.pi*(1/6)
			radians1[i] = [mot_ang_rad_0, mot_ang_rad_1]

			#motor_torque=stiffness*mot_ang_rad
			motor_torque_0=stiffness_0*mot_ang_rad_0-damping_0*temp_velocity_average[0]
			motor_torque_1=stiffness_1*mot_ang_rad_1-damping_1*temp_velocity_average[1]
			
			#Dead zone
			#if (abs(mot_ang_rad_0) < (5/180*3.14)):
			#	motor_torque_0 = 0
			#if (abs(mot_ang_rad_1) < (5/180*3.14)):
			#	motor_torque_1 = 0
			
			#for safety purposes
			if motor_torque_0 > 1e4:
				motor_torque_0 = 1e4
			if motor_torque_0 < -1e4:
				motor_torque_0 = -1e4
			if motor_torque_1 > 1e4:
				motor_torque_1 = 1e4
			if motor_torque_1 < -1e4:
				motor_torque_1 = -1e4

			#motor_torque=-damping*motor_velocity_radians
			current_0=(motor_torque_0)/(0.146)
			current_1=(motor_torque_1)/(0.146)
			currents[i]=[current_0, current_1]
			motor_curr[i]= [act_pack_0.mot_cur, act_pack_1.mot_cur]
			
			fxs.send_motor_command(dev_id_0, fxe.FX_CURRENT, current_0)
			fxs.send_motor_command(dev_id_1, fxe.FX_CURRENT, current_1)
			
			curr_time=time()-start_time
			#Stride count
			if ((i%(0.1/time_step)) == 0): 
				if (last_step_velocity)>0 and temp_velocity_step<0 :
					#record stride
					last_step_velocity=temp_velocity_step
					current_stride = current_stride+1.0
					#print(current_stride)
					stride_count.append(current_stride)
				else:
					last_step_velocity=temp_velocity_step
					stride_count.append(current_stride)
			else:
				stride_count.append(current_stride)
              
			torque[i] = [motor_torque_0, motor_torque_1]
			times.append(curr_time)
			positions[i] = [motor_angle_0, motor_angle_1]
			setpoints[i] = [zero_setpoint_0, zero_setpoint_0]
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
	
	#plotting
	plt.figure()
	plt.plot(times, positions[:,0], color="r",label="position 0")
	plt.xlabel("time (s)")
	plt.legend(loc="upper right")
	
	plt.figure()
	plt.plot(times, positions[:,1], color="r",label="position 1")
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
	plt.plot(times, motor_curr[:,0], color="r",label="motor current 0")
	plt.xlabel("time (s)")
	plt.legend(loc="upper right")

	plt.figure()
	plt.plot(times, motor_curr[:,1], color="r",label="motor current 1")
	plt.xlabel("time (s)")
	plt.legend(loc="upper right")
	
	plt.figure()
	plt.plot(times,velocities_rad[:,0],color="b",label="velocity, radians 0")
	plt.xlabel("time (s)")
	plt.legend(loc="upper right")
	
	plt.figure()
	plt.plot(times,velocities_rad[:,1],color="b",label="velocity, radians 1")
	plt.xlabel("time (s)")
	plt.legend(loc="upper right")
	
	plt.figure()
	plt.plot(times,velocities_rad_averages[:, 0],color="m",label="velocity, radians, filtered 0")
	plt.xlabel("time (s)")
	plt.legend(loc="upper right")
	
	plt.figure()
	plt.plot(times,velocities_rad_averages[:, 1],color="m",label="velocity, radians, filtered 1")
	plt.xlabel("time (s)")
	plt.legend(loc="upper right")
	
	plt.figure()
	plt.plot(times,stride_count,color="r",label="stride count")
	plt.xlabel("time (s)")
	plt.legend(loc="upper right")
	plt.show()
	
	file1=open("exodatatext.txt","a+")
	file1.truncate(0)
	file1.write("time position_rad_0 position_rad_0 command_current_0 command_current_1 motor_current_0 motor_current_1 velocity_rad_0 velocity_rad_1 velocity_filtered_0 velocity_filtered_1 torque_0 torque_1 stride_count\n")
	file1.flush()
	for index in range(len(times)):
		file1.write(str(times[index])+" "+str(radians1[index, 0])+" "+str(radians1[index, 1])+" "+str(currents[index, 0])+" "+str(currents[index, 1])+" "+str(motor_curr[index, 0])+" "+str(motor_curr[index, 1])+" "+str(velocities_rad[index, 0])+" "+str(velocities_rad[index, 1])+" "+str(velocities_rad_averages[index, 0])+" "+str(velocities_rad_averages[index, 1])+" "+str(torque[index, 0])+" "+str(torque[index, 1])+" "+str(stride_count[index])+"\n" )
	file1.flush()
	file1.close()
	#with open("exodatatest.txt","w+") as file1:
		#file1.write("time position_rad_0 position_rad_0 command_current_0 command_current_1 motor_current_0 motor_current_1 velocity_rad_0 velocity_rad_1 velocity_filtered_0 velocity_filtered_1 torque_0 torque_1 stride_count\n")
		#for index in range(len(times)):
			#file1.write(str(times[index])+" "+str(radians1[index, 0])+" "+str(radians1[index, 1])+" "+str(currents[index, 0])+" "+str(currents[index, 1])+" "+str(motor_curr[index, 0])+" "+str(motor_curr[index, 1])+" "+str(velocities_rad[index, 0])+" "+str(velocities_rad[index, 1])+" "+str(velocities_rad_averages[index, 0])+" "+str(velocities_rad_averages[index, 1])+" "+str(torque[index, 0])+" "+str(torque[index, 1])+" "+str(stride_count[index])+"\n" )
		#file1.close()


	return True


def main():
	"""
	Standalone current control execution
	"""
	# pylint: disable=import-outside-toplevel
	import argparse

	parser = argparse.ArgumentParser(description=__doc__)
	#parser.add_argument(
	#"ports", metavar="Ports", type=str, nargs=1, help="Your device serial ports."
	#)
	parser.add_argument(
		"-b",
		"--baud",
        help="Serial communication baud rate.",
	)
	args = parser.parse_args()
	
	bilateral_impedance(flex.FlexSEA(), args.baud_rate)


if __name__ == "__main__":
	main()
