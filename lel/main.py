import myo
import serial
import sys

port = serial.Serial(port='COM8', baudrate=115200)


class Listener(myo.DeviceListener):
    def on_paired(self, event):
        print("Hello, {}!".format(event.device_name))
        event.device.vibrate(myo.VibrationType.short)

    def on_unpaired(self, event):
        return False  # Stop the hub

    def on_pose(self, event):
        pose = event.pose
        if pose == myo.Pose.fist:
            sys.stdout.write("\033[K")
            sys.stdout.write("\033[F")
            print("Fist")
            if not port.isOpen():
                port.open()
            port.write(b'1')
            port.close()

        if pose == myo.Pose.fingers_spread:
            sys.stdout.write("\033[K")
            sys.stdout.write("\033[F")
            print("Fingersspread")
            if not port.isOpen():
                port.open()
            port.write(b'2')
            port.close()

        if pose == myo.Pose.wave_in:
            sys.stdout.write("\033[K")
            sys.stdout.write("\033[F")
            print("Wave in")
            if not port.isOpen():
                port.open()
            port.write(b'3')
            port.close()

        if pose == myo.Pose.wave_out:
            sys.stdout.write("\033[K")
            sys.stdout.write("\033[F")
            print("Wave out")
            if not port.isOpen():
                port.open()
            port.write(b'4')
            port.close()

        if pose == myo.Pose.double_tap:
            sys.stdout.write("\033[K")
            sys.stdout.write("\033[F")
            print("Double tap")
            if not port.isOpen():
                port.open()
            port.write(b'5')
            port.close()

        if pose == myo.Pose.rest:
            sys.stdout.write("\033[K")
            sys.stdout.write("\033[F")
            print("Rest")
            if not port.isOpen():
                port.open()
            port.write(b'6')
            port.close()

        # else:
        #     sys.stdout.write("\033[K")
        #     sys.stdout.write("\033[F")
        #     print("Unknown pose")

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

    def on_battery_level(self, event):
        batt = event.battery_level


if __name__ == '__main__':
    myo.init(sdk_path='./myo-sdk-win-0.9.0/')
    hub = myo.Hub()
    listener = Listener()
    while hub.run(listener.on_event, 500):
        pass