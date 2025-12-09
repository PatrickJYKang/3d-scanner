import serial
import matplotlib.pyplot as plt
from math import pi, sin, cos


# Need to figure out port! Easiest way for now is via Arduino IDE.
# Just make sure you don't let the Arduino IDE connect & hog the connection...
# And make sure the baudrate matches what the Arduino code sets.

port = ''
baudrate = 9600

ser = serial.Serial(port, baudrate, timeout=None)


# Let's assume we get data from the Arduino that loops like ##,%%%\n
#   where the ## is the angle of rotation and the %%% is distance measured
# We will store the received data in two lists, which we initialize here
angles = list()
dists  = list()
x_coords = list()
y_coords = list()

# Let's also initialize our plotting code
plt.ion()
plot_fig, plot_axes = plt.subplots()
plot_axes.axis('equal')


# Read lines until we receive the word 'start'
while True:
    data_str = ser.readline().decode()
    if 'start' in data_str:
        break


while True:                           # If we only wanted to read 50 lines, instead do: for i in range(50):
    data_bytes = ser.readline()       # Read from serial until newline character (\n)
    data_str = data_bytes.decode()    # Convert from 'bytes' object to a string
    data_str = data_str.strip()       # Strip off whitespace (e.g. newlines)

    # If this line contains 'done', exit the loop
    if 'done' in data_str:
        break

    # Otherwise (if we haven't received 'done'), we assume this line has numbers
    # This line of code on the string '25,6.7' will give the list of strings ['25', '6.7']
    numbers = data_str.split(',')

    # Convert this list of strings into actual numbers, so we can do math on them
    # TODO: Use a try/except statement to handle errors if we don't receive exactly what we expected
    angle = float( numbers[0] )
    dist  = float( numbers[1] )

    # FIX ME: Convert from angle/distance to X & Y coordinates!
    # You can use pi, sin(), and cos() since they got imported above.
    # ... just note that the trig functions expect radian inputs!
    # Note that you could theoretically do this in the Arduino script.
    # It doesn't really matter where you do this math, as long as it happens somewhere!
    x = 0
    y = 0

    print(f'Angle: {angle}, Dist: {dist}, X: {x}, Y: {y}')

    # Append this new data to our lists
    angles.append( angle )
    dists.append( dist )

    x_coords.append( x )
    y_coords.append( y )

    # Clear the plot, make scatter plot of our updated data, and how the plot
    plt.cla()
    plot_axes.scatter(x_coords, y_coords, c='k')
    plt.show()
    plt.pause(0.01)

print('Done!')
