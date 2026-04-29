import matplotlib.pyplot as plt
import numpy as np
import serial
import os

# FORCE Python to look in the correct folder
try:
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
except:
    pass 

try:
    # Creating serial ID
    ser = serial.Serial('COM6', 115200)
    ser.reset_input_buffer() # Clear any old junk
    x_points = []
    y_points = []

    # Replace the file reading logic with Serial reading logic
    print("Waiting for Arduino data...")
    while True:
        # Read the line once and store it
        raw_line = ser.readline().decode('utf-8', errors='ignore').strip()
        
        # Check if we should stop
        if raw_line == "END_DUMP":
            print("Received END_DUMP signal.")
            break
            
        # If it's a coordinate, process it
        if "," in raw_line:
            try:
                parts = raw_line.split(",")
                x_val = float(parts[0])
                y_val = float(parts[1])
                x_points.append(x_val)
                y_points.append(y_val)
                print(f"Received: {x_val}, {y_val}")
            except (ValueError, IndexError):
                continue
            
    # Save data with quicker array type
    x_arr = np.array(x_points)
    y_arr = np.array(y_points)

    # Plot graph
    plt.figure(figsize=(10, 8))
    plt.plot(x_arr, y_arr)
    plt.title("Arduino Sensor Map")
    plt.grid(True)
    
    # Save and open image
    image_path = 'my_map_output.png'
    plt.savefig(image_path, dpi=300)
    print("Map saved successfully!")
    plt.show()
    os.startfile(image_path)

except Exception as e:
    print(f"\nCRITICAL ERROR: {e}")