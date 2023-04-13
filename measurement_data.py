import serial
import math

# Modify to the correct UART port for your PC - this program uses COM4
PORT = 'COM4'
s = serial.Serial(PORT, 115200) # 115200 is the baud rate

# Reset input and output buffers 
s.reset_output_buffer()
s.reset_input_buffer()

# Open file to write measurements into 
with open("pointdata.xyz", "w") as f:
    steps = 0
    x = 0 # Initial x-displacement (mm)
    STEP_INCREMENT = 350 # X-displacement per step (mm)
    ROTATION = 32 # Modify to number of steps for next measurement taken
    TOTAL_STEPS = 512 # One rotation is 512 steps for 28BYJ-48 motor
    num_increments = int(input("Enter number of increments in measurement: ")) # How many displacement increments the program runs for 
    count = 0
    while count < num_increments:
        # Format from UART - decode into string and remove special output characters
        data = s.readline().decode("utf-8")[0:-2]
        if data.isdigit():
            # Compute angle and coordinates
            angle = (steps/TOTAL_STEPS)*2*math.pi 
            r = int(data)
            y = r*math.cos(angle) 
            z = r*math.sin(angle) 
            # Write to console and xyz file
            print(y)
            print(z)
            f.write(f'{x} {y} {z}\n')
            steps += ROTATION # For next measurement 
        if steps == 512:
            # Reset number of steps after a full rotation is completed and increment x and count
            steps = 0
            x += STEP_INCREMENT
            count += 1
        print(data)
