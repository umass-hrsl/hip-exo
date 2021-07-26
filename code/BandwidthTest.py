#!/usr/bin/env python3

from math import pi, sin, log, exp
from time import sleep, time
import numpy as np
from scipy.signal import chirp
import matplotlib.pyplot as plt
import matplotlib
from flexsea import fxUtils as fxu
from flexsea import fxEnums as fxe
from flexsea import fxPlotting as fxp
from flexsea import flexsea as flex

# Plot in a browser:
matplotlib.use("WebAgg")
if fxu.is_pi():
    matplotlib.rcParams.update({"webagg.address": "0.0.0.0"})
else
    cmd_freq = 100
    print(f"Capping the command frequency in Windows to {cmd_freq}")

def Bandwidth_Test(
    fxs,
    ports,
    baud_rate,
    controller_type=fxe.HSS_POSITION,
    signal_type=Signal.sine,
    cmd_freq=500,
    signal_amplitude=500,
    number_of_loops=15,
    signal_freq=5,
    cycle_delay=0.1,
    request_jitter=False,
    jitter=20,
):
    debug_logging_level = 6  # 6 is least verbose, 0 is most verbose
    data_log = False  # Data log logs device data

    delay_time = 1.0 / (float(cmd_freq))
    print(delay_time)

    # Open the device and start streaming
    dev_id0 = fxs.open(ports[0], baud_rate, debug_logging_level)
    fxs.start_streaming(dev_id0, cmd_freq, data_log)
    print("Connected to device 0 with ID", dev_id0)

    # Get initial position:
    print("Reading initial position...")
    # Give the device time to consume the startStreaming command and start streaming
    sleep(0.1)
    data = fxs.read_device(dev_id0)
    initial_pos_0 = data.mot_ang
    initial_pos_1 = 0

    start_freq = 1,
    end_freq = 10,
    T = 10,
    n = 1000
    t = np.linspace(0, T, n, endpoint=False)
    samples = chirp(t, start_freq, T, end_freq, method='logarithmic')

    requests = []
    measurements = []
    times = []
    cycle_stop_times = []
    dev_write_command_times = []
    dev_read_command_times = []

    print("setting up chirp test")
    # Gains are, in order: kp, ki, kd, K, B & ff
    fxs.set_gains(dev_id0, 300, 50, 0, 0, 0, 0)

    i = 0
    start_time = time()
    for rep in range(number_of_loops):
        elapsed_time = time() - start_time
        fxu.print_loop_count_and_time(rep, number_of_loops, elapsed_time)

        dev0_read_time_before = time()
        data0 = fxs.read_device(dev_id0)
        dev0_read_time_after = time()

        dev0_write_time_before = time()
        fxs.send_motor_command(dev_id0, fxe.FX_POSITION, sample + initial_pos_0)
        dev0_write_time_after = time()
        measurements0.append(data0.mot_ang - initial_pos_0)

        dev0_read_command_times.append(dev0_read_time_after - dev0_read_time_before)
        dev0_write_command_times.append(dev0_write_time_after - dev0_write_time_before)
        times.append(time() - start_time)
        requests.append(sample)
        i = i + 1

        cycle_stop_times.append(time() - start_time)
        fxs.send_motor_command(dev_id0, fxe.FX_NONE, 0)
        sleep(0.1)

    elapsed_time = time() - start_time
    actual_period = cycle_stop_times[0]
    actual_frequency = 1 / actual_period
    cmd_freq = i / elapsed_time

    figure_counter = 1  # First time, functions will increment
    figure_counter = fxp.plot_setpoint_vs_desired(
        dev_id0,
        figure_counter,
        controller_type,
        actual_frequency,
        signal_amplitude,
        signal_type_str,
        cmd_freq,
        times,
        requests,
        measurements0,
        cycle_stop_times,
    )
    figure_counter = fxp.plot_exp_stats(
        dev_id0, figure_counter, dev0_write_command_times, dev0_read_command_times
    )

    fxu.print_plot_exit()
    plt.show()
    fxs.close_all()
    fxs.close_all()



