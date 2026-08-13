import serial
import time
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque


# ============================================================
# UART
# ============================================================

UART_PORT = 'COM7'
BAUD_RATE = 921600


# ============================================================
# GRAPH
# ============================================================

MAX_PLOT_POINTS = 100

y_plot_data = deque(
    [0] * MAX_PLOT_POINTS,
    maxlen=MAX_PLOT_POINTS
)


# ============================================================
# DATA LOG
# ============================================================

all_timestamps = []
all_voltages = []


# ============================================================
# FIGURE
# ============================================================

fig, ax = plt.subplots()

line, = ax.plot(y_plot_data)

ax.set_ylim(0, 5000)

ax.set_xlim(0, MAX_PLOT_POINTS - 1)

ax.set_title("Real-Time ADC Vpp")

ax.set_ylabel("Voltage (mVpp)")

ax.set_xlabel("Samples")


# ============================================================
# SERIAL
# ============================================================

try:

    ser = serial.Serial(
        UART_PORT,
        BAUD_RATE,
        timeout=0.02
    )

    ser.reset_input_buffer()

    print(f"Connected to {UART_PORT}.")
    print("Real-time Vpp graph is running.")
    print("Close the graph window to stop and save data.")

except serial.SerialException as e:

    print(f"Error opening serial port: {e}")
    raise SystemExit


# ============================================================
# INIT
# ============================================================

def init():

    line.set_ydata(y_plot_data)

    return line,


# ============================================================
# UPDATE
# ============================================================

def update(frame):

    # --------------------------------------------------------
    # Read everything currently waiting in UART buffer.
    #
    # This prevents old measurements from accumulating and
    # appearing on the graph with unnecessary delay.
    # --------------------------------------------------------

    reads = 0

    while ser.in_waiting > 0 and reads < 500:

        try:

            raw_line = ser.readline().decode(
                'ascii',
                errors='ignore'
            ).strip()


            if raw_line.isdigit():

                voltage = int(raw_line)


                # ------------------------------------------------
                # Valid ADC Vpp range
                # ------------------------------------------------

                if 0 <= voltage <= 5000:

                    y_plot_data.append(voltage)

                    all_timestamps.append(
                        time.time()
                    )

                    all_voltages.append(
                        voltage
                    )

        except Exception:

            pass


        reads += 1


    # --------------------------------------------------------
    # Update graph
    # --------------------------------------------------------

    line.set_ydata(y_plot_data)

    return line,


# ============================================================
# ANIMATION
#
# 20 ms = 50 GUI updates / second
# ============================================================

ani = animation.FuncAnimation(
    fig,
    update,
    init_func=init,
    interval=20,
    blit=False,
    cache_frame_data=False
)


# ============================================================
# RUN
# ============================================================

try:

    plt.show()

except KeyboardInterrupt:

    pass

except Exception as e:

    print(f"Plot closed with error: {e}")


# ============================================================
# CLEANUP
# ============================================================

finally:

    if 'ser' in locals() and ser.is_open:

        ser.close()

        print("\nSerial port closed.")


    file_time = time.strftime(
        "%Y%m%d_%H%M%S"
    )


    filename = (
        rf"C:\Users\User\Desktop\excel"
        rf"\fpga_voltage_log_{file_time}.xlsx"
    )


    if len(all_voltages) > 0:

        df = pd.DataFrame({

            "Timestamp":
                pd.to_datetime(
                    all_timestamps,
                    unit='s'
                ).strftime(
                    '%H:%M:%S.%f'
                ),

            "Voltage (mVpp)":
                all_voltages

        })


        try:

            df.to_excel(
                filename,
                index=False
            )

            print(
                f"Successfully saved "
                f"{len(all_voltages)} readings to:"
            )

            print(filename)

        except Exception as e:

            print(
                f"Failed to save Excel file: {e}"
            )

    else:

        print(
            "No valid data received. "
            "Excel file was not created."
        )