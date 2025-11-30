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

def unilateral_impedance(fxs, baud_rate, exp_time=300, time_step=1/500):
	"""
	demo current control
	"""
	port = '/dev/ttyACM0'
	dev_id = fxs.open(port, baud_rate, log_level=6)
	fxs.start_streaming(dev_id, 500, log_en=False)
	app_type = fxs.get_app_type(dev_id)

	print("Setting controller to current...")
	# Gains are, in order: kp, ki, kd, K, B & ff
	# 30 600 30 0 0 128
	#fxs.set_gains(dev_id, 30, 500, 30, 0, 0, 96)
	fxs.set_gains(dev_id, 40, 400, 0, 0, 0, 128)
	sleep(0.5)

	init_data=fxs.read_device(dev_id)
	zero_setpoint=init_data.mot_ang
	start_time=time()
	stiffness=0
	damping=0 #(mNm*s)/rad
	#setting it to 1000 made it oscillate very fast
	#when we set the stiffness to 1000 from 2000, and changed current from current=K*theta*0.146 to current=k*theta/0.146
	#we changed the current by a factor of (1000/2000)*(1/(0.146^2)), which means our current was effectively 26.45 times what we were previously sending
	
	current=0.0
	fxs.send_motor_command(dev_id, fxe.FX_CURRENT, current)
	
	moving_avg_window=170
	
	#the array that holds average velocities as recorded each step (holds filtered velocity values)
	velocities_rad_averages=[]
	temp_velocity_average=0
	
	num_time_steps=int(exp_time/time_step)
	positions=[]
	torque=[]
	times=[]
	setpoints=[]
	currents=[]
	motor_curr=[]
	mot_ang=[]
	velocities_rad=[]
	stride_count = []
	current_stride = 0.0
	last_step_velocity=0.0
	print("sampling at "+str(1.0/time_step)+"for "+str(exp_time)+"seconds\n")
	print("Press cntrl-c to stop\n")
	input("Press enter/any key to start")
	print("Test started\n")
	for i in range(num_time_steps):
		try:
			sleep(time_step)
	
			# get the return vals for read_device from Python->flexsea->flexsea->dev_spec->ExoState.py
			#reading device, getting device state object
			act_pack=fxs.read_device(dev_id)
			motor_angle=act_pack.mot_ang
			motor_velocity_radians=act_pack.mot_vel*((math.pi)/180)*(1/6) # act_pack.mot_vel in deg
			velocities_rad.append(motor_velocity_radians)
			
			if i<=moving_avg_window:
				temp_velocity_average=mean(velocities_rad)
				velocities_rad_averages.append(temp_velocity_average)
			elif i>moving_avg_window:
				temp_velocity_average=temp_velocity_average+(1/moving_avg_window)*(velocities_rad[i-1]-velocities_rad[i-moving_avg_window-1])
				velocities_rad_averages.append(temp_velocity_average)
			
			if i<=500:
				temp_velocity_step=mean(velocities_rad)
				
			elif i>500:
				temp_velocity_step=temp_velocity_step+(1/500)*(velocities_rad[i-1]-velocities_rad[i-500-1])
				
			#print(zero_setpoint)
			delta_motor_angle=zero_setpoint-motor_angle
			mot_ang_rad=delta_motor_angle*(1/16383)*2*math.pi*(1/6)
			mot_ang.append(mot_ang_rad)           

			current=0
			motor_torque=stiffness*mot_ang_rad-damping*temp_velocity_average
			current=(motor_torque)/(0.146)
			currents.append(current)
			motor_curr.append(act_pack.mot_cur)

			curr_time=time()-start_time
			#Stride count
			if ((i%(0.1/time_step)) == 0): 
				if (last_step_velocity)>0 and temp_velocity_step<0 :
					#record stride
					last_step_velocity=temp_velocity_step
					current_stride = current_stride+1.0
					print(current_stride)
					stride_count.append(current_stride)
				else:
					last_step_velocity=temp_velocity_step
					stride_count.append(current_stride)
			else:
				stride_count.append(current_stride)

			torque.append(motor_torque)
			times.append(curr_time)
			positions.append(motor_angle)
			setpoints.append(zero_setpoint)
			fxs.send_motor_command(dev_id, fxe.FX_CURRENT, current)
		except KeyboardInterrupt:
			
			fxs.send_motor_command(dev_id, fxe.FX_CURRENT, 0.0)
			sleep(0.2)
			print("Test Interrupted, plotting and writing data\n")
			break
			return False

	# When we exit we want the motor to be off
	
	fxs.send_motor_command(dev_id, fxe.FX_CURRENT, 0.0)
	#fxs.send_motor_command(dev_id, fxe.FX_NONE, 0)
	sleep(0.5)
	print("test stopped, plotting\n")
	fxs.send_motor_command(dev_id, fxe.FX_CURRENT, 0.0)
	fxs.send_motor_command(dev_id, fxe.FX_NONE, 0)
	fxs.close(dev_id)

	#plotting
	plt.figure()
	plt.plot(times, positions, color="b",label="position")
	plt.plot(times,setpoints,color="g",label="setpoint")
	plt.xlabel("time (s)")
	plt.legend(loc="upper right")

	plt.figure()
	plt.plot(times, torque, color="r",label="torque")
	plt.xlabel("time (s)")
	plt.legend(loc="upper right")

	plt.figure()
	plt.plot(times, currents, color="b",label="commanded current")
	plt.xlabel("time (s)")
	plt.legend(loc="upper right")

	plt.figure()
	plt.plot(times, motor_curr, color="r",label="motor current")
	plt.xlabel("time (s)")
	plt.legend(loc="upper right")

	plt.figure()
	plt.plot(times,velocities_rad,color="b",label="velocity, radians")
	plt.xlabel("time (s)")
	plt.legend(loc="upper right")
	
	plt.figure()
	plt.plot(times,velocities_rad_averages,color="m",label="velocity, radians, filtered")
	plt.xlabel("time (s)")
	plt.legend(loc="upper right")

	plt.figure()
	plt.plot(times,radians1,color="r",label="motor angle radians")
	plt.xlabel("time (s)")
	plt.legend(loc="upper right")
	
	plt.figure()
	plt.plot(times,stride_count,color="r",label="stride count")
	plt.xlabel("time (s)")
	plt.legend(loc="upper right")
	plt.show()
	
	file1=open("exodatatext.txt","a+")
	file1.truncate(0)
	file1.write("time command_current motor_current velocity_rad velocity_deg position_click position_rad torque filtered_velocity\n")
	file1.flush()
	for index in range(len(times)):
		file1.write(str(times[index])+" "+str(currents[index])+" "+str(motor_curr[index])+" "+str(velocities_rad[index])+" "+str(velocities_deg[index])+" "+str(positions[index])+" "+str(radians1[index])+" "+str(torque[index])+str(velocities_rad_averages[index])+"\n" )
	file1.flush()
	file1.close()
# 	with open("exodatatest.txt","w+") as file1:
# 		file1.write("time command_current motor_current velocity_rad velocity_deg position_click position_rad torque\n")
# 		for index in range(len(times)):
# 			file1.write(str(times[index])+" "+str(currents[index])+" "+str(motor_curr[index])+" "+str(velocities_rad[index])+" "+str(velocities_deg[index])+" "+str(positions[index])+" "+str(radians1[index])+" "+str(torque[index]) )
# 		file1.close()


	return True


def main():
	"""
	Standalone current control execution
	"""
	# pylint: disable=import-outside-toplevel
	import argparse
	port = '/dev/ttyACM0'
    
	parser = argparse.ArgumentParser(description=__doc__)
	#parser.add_argument(
	#"port", metavar="Port", type=str, nargs=1, help="Your device serial port."
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
	unilateral_impedance(flex.FlexSEA(), args.baud_rate)


if __name__ == "__main__":
	main()

