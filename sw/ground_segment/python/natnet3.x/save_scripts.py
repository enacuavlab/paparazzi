import csv
import re
import subprocess
import time
from ivy.ivy import IvyServer

def parse_mocap_data(data):
    """
    Parses motion capture data to extract rigid body ID, position, and orientation.
    
    Args:
    - data (str): Raw motion capture data as a string.

    Returns:
    - list of dicts: Each dict contains the ID, Position, and Orientation of a rigid body.
    """
    rigid_body_data = []

    # Regular expression to match the relevant sections in the data
    rigid_body_pattern = re.compile(r"ID\s+:\s+(\d+)\s+Position\s+:\s+\[([-\d.]+),\s+([-\d.]+),\s+([-\d.]+)\]\s+Orientation\s+:\s+\[([-\d.]+),\s+([-\d.]+),\s+([-\d.]+),\s+([-\d.]+)\]")

    # Find all rigid body matches in the data
    matches = rigid_body_pattern.findall(data)

    # Iterate over all matches and format the data
    for match in matches:
        body_id = match[0]
        position = [float(match[1]), float(match[2]), float(match[3])]
        orientation = [float(match[4]), float(match[5]), float(match[6]), float(match[7])]
        
        rigid_body_data.append({
            'ID': body_id,
            'Position': position,
            'Orientation': orientation
        })
    
    return rigid_body_data


def save_rigid_body_data_to_csv(data, output_file="rigid_body_data.csv"):
    """
    Saves the rigid body data (ID, Position, and Orientation) to a CSV file.

    Args:
    - data (list of dicts): List of rigid body data dictionaries.
    - output_file (str): Path to the output CSV file (default is "rigid_body_data.csv").
    """
    with open(output_file, "a", newline="") as f:
        writer = csv.writer(f)

        # If the file is empty, write headers
        if f.tell() == 0:
            writer.writerow(["ID", "x", "y", "z", "qx", "qy", "qz", "qw"])

        # Write data for each rigid body
        for entry in data:
            writer.writerow([entry['ID'], *entry['Position'], *entry['Orientation']])

    print(f"Rigid body data saved to {output_file}")


def process_mocap_data(data):
    """
    Processes motion capture data, extracts the rigid body information, and saves it to a CSV file.
    
    Args:
    - data (str): Raw motion capture data as a string.
    """
    rigid_body_data = parse_mocap_data(data)
    save_rigid_body_data_to_csv(rigid_body_data)


def run_natnet_to_ivy():
    """
    Runs the external command './natnet2ivy.py' with the specified arguments using subprocess.
    This should only run once to establish the connection.
    """
    command = [
        "./natnet2ivy.py", "--up_axis", "z_up", "-le", "near", "--ac_nose", "far",
        "--x_side", "right", "-s", "192.168.1.240", "-ac", "117", "2", "-v", "-s", "192.168.1.240"
    ]
    
    try:
        print("Running the natnet2ivy command to establish the connection...")

        # Run the command to establish the connection to Ivy Bus (this only runs once)
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        stdout, stderr = process.communicate()

        # Check if the command executed successfully
        if process.returncode == 0:
            print("Connection to Ivy Bus established successfully.")
        else:
            print(f"Error executing command: {stderr.decode('utf-8')}")
            return

    except Exception as e:
        print(f"An error occurred while running the command: {e}")


class MotionCaptureRecorder(IvyServer):
    def __init__(self, output_file="rigid_body_data.csv"):
        IvyServer.__init__(self, "MotionCaptureRecorder")
        self.output_file = output_file

        # Connect to the Ivy Bus
        self.start("192.168.1.240")  # Update with your actual Ivy Bus IP & port
        self.bind_msg(self.on_mocap_data, "MOCAP_DATA (.*)")

        # Initialize CSV file
        with open(self.output_file, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["Timestamp", "ID", "x", "y", "z", "qx", "qy", "qz", "qw"])

    def on_mocap_data(self, sender, msg):
        """Callback function to handle incoming motion capture data."""
        try:
            # Process incoming message (motion capture data)
            process_mocap_data(msg)

        except Exception as e:
            print("Error processing mocap data:", e)


if __name__ == "__main__":
    # First, run the natnet2ivy command to establish the connection to the Ivy Bus
    run_natnet_to_ivy()

    # Start the MotionCaptureRecorder to receive and process data continuously
    recorder = MotionCaptureRecorder()

    try:
        while True:
            time.sleep(1)  # Sleep for 1 second to keep the script running
    except KeyboardInterrupt:
        print("Stopping motion capture recorder. Pressed Ctrl+C.")
