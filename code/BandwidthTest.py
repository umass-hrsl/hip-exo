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
    number_of_loops=4,
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
    y = chirp(t, start_freq, T, end_freq, method='logarithmic')








