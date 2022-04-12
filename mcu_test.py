from utils import *
from mcu_interface.interface import UARTMCUInterface, MCUInterface
from mcu_interface.commands import *
import time
import glob
import os

def main():
    if os.getuid() != 0:
        input("not running as root. continue?")

    print(f"available serial ports: {glob.glob('/dev/tty[A-Za-z]*')}")
    port = input("enter serial port: ")
    mcu = UARTMCUInterface(port)

    mcu.start()

    print("communications successfully started")

    cmd_echo(mcu, "echotest")
    time.sleep(0.1)
    if mcu.data.other.strip() == "echotest":
        print("yay, a connection could be established!")
    elif mcu.data.other:
        print(f"something was received, but it was not correct: {mcu.data.other}")
    else:
        print("something went wrong...")

    mcu.stop()

    print("communications successfully stopped")

    # TODO: make this useful

if __name__ == "__main__":
    main()
