import serial, numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial.tools.list_ports
import scipy.ndimage   # added for interpolation

# ports = list(serial.tools.list_ports.comports())
# if not ports:
#     print("No serial ports found.")
#     exit()

# print("Available serial ports:")
# for idx, port in enumerate(ports):
#     print(f"{idx}: {port.device} - {port.description}")

# port_index = int(input("Select a port index: "))
# selected_port = ports[port_index].device
# ser = serial.Serial(selected_port, 115200, timeout=1)

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

fig, ax = plt.subplots()
data = np.zeros((12, 16))
im = ax.imshow(data, vmin=25, vmax=40, cmap='turbo')
plt.colorbar(im)

def update(frame):
    raw = ser.readline().decode('ascii', errors='ignore').strip()
    raw = raw.replace('\\n', '')
    parts = raw.rstrip(',').split(',')

    if len(parts) == 192:
        temps = np.array([float(p) for p in parts]).reshape((12, 16))

        # interpolate the 12x16 grid by a factor of 3 for better resolution
        temps = scipy.ndimage.zoom(temps, 3, order=3)

        im.set_data(temps)

        # dynamically adjust color scale based on detected temps
        #im.set_clim(temps.min(), temps.max())

    return (im,)

ani = animation.FuncAnimation(fig, update, interval=10, blit=True, save_count = 1)
plt.show()
