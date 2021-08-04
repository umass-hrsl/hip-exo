#!/usr/bin/env python3

"""
FlexSEA Two Position Control Demo
"""
from time import sleep, time
from scipy.signal import chirp
import matplotlib
import matplotlib.pyplot as plt
from flexsea import fxUtils as fxu
from flexsea import fxEnums as fxe
from flexsea import flexsea as flex

matplotlib.use("WebAgg")
if fxu.is_pi():
    matplotlib.rcParams.update({"webagg.address": "0.0.0.0"})


def bandwidth_test(
    fxs,
    port,
    baud_rate,
    exp_time=13,
    time_step=0.1,
    transition_time=1.5,
    resolution=100,
):
    """
    Send two positions commands to test the positionc controller
    """
    # Open device
    dev_id = fxs.open(port, baud_rate, 0)
    fxs.start_streaming(dev_id, resolution, log_en=True)
    sleep(0.1)

    # Setting initial angle and angle waypoints
    act_pack_state = fxs.read_device(dev_id)
    initial_angle = act_pack_state.mot_ang

    # Setting angle waypoints
    f0 = 1
    f1 = 5
    t1 = 10
    t = np.linspace(0, 10, 7000)
    position = chirp(t, f0, f1, t1, method='logarithmic')

    # Setting loop duration and transition rate
    # num_time_steps = int(exp_time / time_step)
    # transition_steps = int(transition_time / time_step)

    # Setting gains (dev_id, kp, ki, kd, K, B, ff)
    fxs.set_gains(dev_id, 50, 0, 0, 0, 0, 0)
    # kp=50 and ki=0 is a very soft controller, perfect for bench top experiments

    # fxs.send_motor_command(dev_id, fxe.FX_POSITION, position)

    # Matplotlib - initialize lists
    requests = []
    measurements = []
    times = []

    start_time = time()
    while t <= 10:
        fxs.send_motor_command(dev_id, fxe.FX_POSITION, position)
        act_pack_state = fxs.read_device(dev_id)
        measured_pos = act_pack_state.mot_ang
        times.append(time() - start_time)
        requests.append(position)
        measurements.append(measured_pos)

    # Disable the controller, send 0 PWM
    fxs.send_motor_command(dev_id, fxe.FX_VOLTAGE, 0)
    sleep(0.1)

    # Plot before exit:
    plt.title("Chirp Test")
    plt.plot(times, requests, color="b", label="Desired position")
    plt.plot(times, measurements, color="r", label="Measured position")
    plt.xlabel("Time (s)")
    plt.ylabel("Encoder position")
    plt.legend(loc="upper right")
    fxu.print_plot_exit()
    plt.show()

    # Close device and do device cleanup

    return fxs.close(dev_id)


def main():
    """
	Standalone two position control execution
	"""
    # pylint: disable=import-outside-toplevel
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "port", metavar="Port", type=str, nargs=1, help="Your device serial port."
    )
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
    bandwidth_test(flex.FlexSEA(), args.port[0], args.baud_rate)


if __name__ == "__main__":
    main()
