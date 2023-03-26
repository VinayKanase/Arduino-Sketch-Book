import serial
import pyautogui
import time
arduino = serial.Serial(port='COM4', baudrate=9600, timeout=.1)

while True:
    command_string = str(arduino.readline())
    command_string = command_string[2:][:-5]
    print(command_string)
    if command_string == "Play/Pause":
        pyautogui.press(['space'])
    elif command_string == "Volume Up":
        pyautogui.hotkey('ctrl', 'up')
    elif command_string == "Volume Down":
        pyautogui.hotkey('ctrl', 'down')
    command_string = ""
