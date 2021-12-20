import myo
import serial
import sys

port = serial.Serial(port='COM8', baudrate=115200)      #Initialization of Serial port


class Listener(myo.DeviceListener):
    """
    Device listen class override for myo.DeviceListener
    Eventhandler for differnent inputs from the Myo Armband
    Handles events such as when a pose is recieved
    """
    i = 0                                               # Used to recognize shutdown sequence
    shutdown = [0, 0, 0]
    j = 1                                               # Indicates currently selected motor

    def on_paired(self, event):
        """
        Event method to perform tasks when the Myo Armband is paried
        Prints hello and the Myo Armband device name and vibrates the myo
        :param event: Event object
        :return: nothing
        """
        print("Hello, {}!".format(event.device_name))
        event.device.vibrate(myo.VibrationType.short)

    def on_unpaired(self, event):
        """
        Event method to perform tasks when the Myo is unpaired
        :param event: The input event object
        :return: False, as the Myo is un_paired, auto-shutdown of program
        """
        return False                                    # Stop the hub

    def on_pose(self, event):
        """
        Event method called when a new pose is received from the Myo Armband
        Handles the input and converts it to data which is then sent to the
        Arduino Mega
        :param event: Event Object
        :return: Nothing
        """
        pose = event.pose
        if not self.shutdown == [2, 2, 1]:              # Checks which new pose is registered unless the
            if pose == myo.Pose.fist:                   # last three poses are: Fingersspread, Fingersspread, Fist
                sys.stdout.write("\033[K")
                sys.stdout.write("\033[F")
                print("Fist")
                port.write(b'1')                        # Sends a byte with a number representing the pose
                self.shutdown[0] = self.shutdown[1]
                self.shutdown[1] = self.shutdown[2]     # "Cycles" the shutdown sequence and inserts the newest pose
                self.shutdown[2] = 1
                print(self.shutdown)                    # Prints the current input sequence

            if pose == myo.Pose.fingers_spread:
                sys.stdout.write("\033[K")
                sys.stdout.write("\033[F")
                print("Fingersspread")
                port.write(b'2')
                self.shutdown[0] = self.shutdown[1]
                self.shutdown[1] = self.shutdown[2]
                self.shutdown[2] = 2
                print(self.shutdown)
                self.j += 1
                if self.j == 4:
                    self.j = 1
                print('Motor:', self.j)

            if pose == myo.Pose.wave_in:
                sys.stdout.write("\033[K")
                sys.stdout.write("\033[F")
                print("Wave in")
                port.write(b'3')
                self.shutdown[0] = self.shutdown[1]
                self.shutdown[1] = self.shutdown[2]
                self.shutdown[2] = 3
                print(self.shutdown)

            if pose == myo.Pose.wave_out:
                sys.stdout.write("\033[K")
                sys.stdout.write("\033[F")
                print("Wave out")
                port.write(b'4')
                self.shutdown[0] = self.shutdown[1]
                self.shutdown[1] = self.shutdown[2]
                self.shutdown[2] = 4
                print(self.shutdown)

            if pose == myo.Pose.double_tap:
                sys.stdout.write("\033[K")
                sys.stdout.write("\033[F")
                print("Double tap")
                port.write(b'5')
                self.shutdown[0] = self.shutdown[1]
                self.shutdown[1] = self.shutdown[2]
                self.shutdown[2] = 5
                print(self.shutdown)

            if pose == myo.Pose.rest:
                sys.stdout.write("\033[K")
                sys.stdout.write("\033[F")
                print("------------")
                port.write(b'9')
        elif self.shutdown == [2, 2, 1]:
            port.write(b'6')                            # '6' represents the shutdown sequence
            self.shutdown = [0, 0, 0]
            print(self.shutdown)
            print('Shutdown')
            self.j = 1
            print('Motor:', self.j)

    def on_locked(self, event):
        """
        Event method called when the Myo Armband is locked
        Prints a message to the user when the Armband is locked
        :param event: Event Object
        :return: Nothing
        """
        print("Device is now locked")

    def on_unlocked(self, event):
        """
        Event method called when the Myo Armband is unlocked
        Prints a message to the user when the Armband is unlocked
        :param event: Event Object
        :return: Nothing
        """
        print("Device is now locked")

    def on_arm_synced(self, event):
        """
        Event method called when the Myo Armband is synced
        Prints a message to the user when the Armband is synced
        :param event: Event object
        :return: Nothing
        """
        print("Myo armband synced")

    def on_arm_unsynced(self, event):
        """
        Event method called when the Myo Armband is unsynced
        Prints a message to the user when the Armband is unsynced
        :param event: Event Object
        :return: Nothing
        """
        print("Myo armband out of sync, please re-sync the device")

    def on_warmup_completed(self, event):
        """
        Event method called when the warm-up of the Myo armband is compledted
        Vibrates the myo armband when warmup has completed
        :param event: Event Object
        :return: Nothing
        """
        event.device.vibrate(myo.VibrationType.long)


if __name__ == '__main__':     # Main function
    """
    Main function
    Looping until either an error occurs or the program is shutdown
    This "main" function opens the Serial port for transmitting, initializes
    the myo armband using the SDK provided from Thalmic Labs, creates and 
    instantiates the hub for connectivity between the Myo Armband and the computer
    Instantiates a lister object for event handling and check for feedback from the Myo
    armband.
    """
    if not port.isOpen():
        port.open()                 # Open the port if it isn't already
    myo.init(sdk_path='./myo-sdk-win-0.9.0/')
    hub = myo.Hub()
    listener = Listener()
    while hub.run(listener.on_event, 500): #Listen to myo armband for events
        pass
