import serial
import time

class ServoController:
    def __init__(self, arduino_port='/dev/ttyACM0', baud_rate=115200):
        """
        Initializes the ServoController object.
        :param arduino_port: Serial arduino_port (e.g., '/dev/ttyACM0')
        :param baud_rate: Baud rate for the serial communication
        """
        self.arduino_port = arduino_port
        self.baud_rate = baud_rate
        self.ser = None

    def connect(self):
        """
        Establishes the connection to the Arduino via the serial arduino_port.
        """
        try:
            self.ser = serial.Serial(self.arduino_port, self.baud_rate, timeout=1)
            time.sleep(2)  # Allow Arduino time to reset
        except serial.SerialException as e:
            raise RuntimeError(f"Failed to connect to {self.arduino_port}: {e}")

    def disconnect(self):
        """
        Closes the serial port connection.
        """
        if self.ser and self.ser.isOpen():
            self.ser.close()

    def send_servo_command(self, commands):
        """
        Sends servo commands to the Arduino and reads the response.
        :param commands: Servo commands as a string (e.g., '1 500, 6 500')
        :return: Response from the Arduino
        """
        if not self.ser or not self.ser.isOpen():
            raise RuntimeError("Serial port is not open")
        
        self.ser.write((commands + '\n').encode())
        time.sleep(0.1)
        response = self.ser.readline().decode(errors='ignore').strip()
        return response

    def send_commands(self, commands):
        """
        Takes servo commands as arguments and sends them to the Arduino.
        :param commands: List of tuples [(servo_id, position), ...]
        :return: Response from the Arduino
        """
        formatted_commands = ', '.join(f"{servo_id} {position}" for servo_id, position in commands)
        return self.send_servo_command(formatted_commands)



# Example usage when running this script directly
if __name__ == "__main__":
    controller = ServoController(arduino_port='/dev/ttyACM0', baud_rate=115200)
    controller.connect()
    #try:
    #    controller.connect()
    #    # Example: Sending commands directly as arguments
    #    commands = [(1, 350), (6, 400), (9, 500)]  # Servo ID and position
    #    response = controller.send_commands(commands)
    #    print("Response from Arduino:", response)
    #finally:
    #    controller.disconnect()

