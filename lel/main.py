import myo
import serial
import sys

port = serial.Serial(port='COM8', baudrate=115200)      # Set COMPort and baudrate used for Arduino code


class Listener(myo.DeviceListener):
    i = 0                                               # Used to recognize shutdown sequence
    shutdown = [0, 0, 0]
    j = 1                                               # Indicates currently selected motor

    def on_paired(self, event):                         # Indicates that Myo is ready to go
        print("Hello, {}!".format(event.device_name))
        event.device.vibrate(myo.VibrationType.short)

    def on_unpaired(self, event):
        return False                                    # Stop the hub

    def on_pose(self, event):                           # Called when a new pose is registered
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
                print(self.shutdown)

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
        print("Device is now locked")

    def on_unlocked(self, event):
        print("Device is now locked")

    def on_arm_synced(self, event):
        print("Myo armband synced")

    def on_arm_unsynced(self, event):
        print("Myo armband out of sync, please re-sync the device")

    def on_warmup_completed(self, event):
        event.device.vibrate(myo.VibrationType.long)


if __name__ == '__main__': # Main function
    if not port.isOpen():
        port.open()
    myo.init(sdk_path='./myo-sdk-win-0.9.0/')
    hub = myo.Hub()
    listener = Listener()
    while hub.run(listener.on_event, 500):
        pass