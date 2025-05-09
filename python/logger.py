# logger.py
#
# GH Labs, NAATOS V2 serial logger
# Written by: Mike Deeds, GH Labs, January 2025
#   (With help from ChatGPT)
#
# Required libraries
#    pip install serial pyserial

import sys
import os
import datetime
import serial
import serial.tools.list_ports as port_list

VERSION = "v1.0"

class Logger:
    """Class to read and print contents of a file line by line."""
    
    def __init__(self, file_path):
        """Initialize with file path."""
        self.file_path = file_path
        self.logfile_open = False
        
        if "COM" in file_path:           
            self.isSerial = True
            self.serial_init(file_path)
        elif "/dev" in file_path:           
            self.isSerial = True
            self.serial_init(file_path)
        else:
            self.isSerial = False

    def run(self):
        if self.isSerial:
            self.serial_read_lines()
        else:
            self.read_file()
        
    def open_logfile(self, suffix):
        if self.logfile_open:
            self.close_logfile()

        logs_dir = os.path.join(os.getcwd(), "logs")
        os.makedirs(logs_dir, exist_ok=True)

        current_time = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_file_name = f"logs\\{current_time}_{suffix}.log"
        print(f"\tOpening log file: {self.log_file_name}")
        self.log = open(self.log_file_name, "a")
        self.logfile_open = True
        self.log.write(f"NAATOS V2 {self.log_file_name} on {self.file_path}\n")
        
    def close_logfile(self):
        print(f"\tClosing log file: {self.log_file_name}")
        self.log.close()
        self.logfile_open = False        
        
    def process_line(self, line):
        print(line, end='')  # Print each line without adding extra newlines

        # open a new log file each time the MK board powers up
        keyword = "HW:"
        if keyword in line:
            boardnum = line.split(keyword, 1)[1].strip()
            print(f"\tFound: {boardnum}")
            self.open_logfile(boardnum)
            
        if self.logfile_open:
            self.log.write(line.rstrip('\n'))
            self.log.flush()
            
        # close the log file at the end of the amplification sequence
        keyword = "valve_ramp_time"
        if keyword in line:
            boardnum = line.split(keyword, 1)[1].strip()
            print(f"\tFound: {keyword}")
            self.close_logfile()
            
    def serial_init(self, port, baudrate=115200, timeout=1):
        """Initialize serial connection."""
        try:
            self.ser = serial.Serial(port, baudrate, timeout=timeout)
            print(f"Connected to {port} at {baudrate} baud.")
            self.open_logfile("start")     #start a log immediately
        except serial.SerialException as e:
            print(f"Error: Unable to open port {port} - {e}")
            sys.exit(1)

    def serial_read_lines(self):
        """Read and print incoming serial data line by line."""
        try:
            while True:
                line = self.ser.readline().decode('utf-8', errors='ignore')
                self.process_line(line)
                
        except KeyboardInterrupt:
            print("\nStopping serial reader.")
        finally:
            self.ser.close()
            print("Serial port closed.")                    

    def read_file(self):
        """Read the file and print each line."""
        try:
            with open(self.file_path, 'r') as file:
                for line in file:
                    self.process_line(line)
                    
        except FileNotFoundError:
            print(f"Error: The file '{self.file_path}' was not found.")
        except Exception as e:
            print(f"Error: {e}")


def main():
    """Main function to check command-line arguments and run the FileReader."""
    print(f"NAATOS v2 Logger version: {VERSION}")

    if len(sys.argv) != 2:
        print("\tUsage: python -m logger <filename>")
        print("\tThis program will open and record a new log file each time a NAATOS board powers up.")
        print("\tlog files are saved to the logs/ subdirectory.")

        ports = list(port_list.comports())
    
        if not ports:
            print("No serial ports found.")
            return
        else:
            print("Available Serial Ports:")
            for port in ports:
                print(f"- {port.device} ({port.description})")
            return

    file_path = sys.argv[1]
    logger = Logger(file_path)
    logger.run()

if __name__ == "__main__":
    main()
