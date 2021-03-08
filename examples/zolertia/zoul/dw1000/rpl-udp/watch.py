#!/usr/bin/python

import serial
import datetime
import os
import sys



def get_time() -> str:
    time = datetime.datetime.now()
    return str(time)

def save(content: str, filename: str, savedir: str = "export/", subfolder: str = get_time()):
    if not os.path.isdir(savedir): 
        os.mkdir(savedir)
    savefolder = os.path.join(savedir, subfolder)
    if not os.path.isdir(savefolder) : 
        os.mkdir(savefolder)
    with open(os.path.join(savefolder, filename.split("/")[-1]), "a") as f:
        f.write(content)



if __name__ == "__main__":

    
    port = '/dev/ttyUSB0'
    start_time = get_time()
    
    # datetime.datetime.strptime("2021-03-08 10:01:47.391636", '%Y-%m-%d %H:%M:%S.%f')

    if len(sys.argv) > 1:
        port = sys.argv[1]
    
    ser = serial.Serial(
        port=port,
        baudrate=115200
    )
    
    while True:    
        out = ""
        while ser.inWaiting() > 0:
            out += ser.read(1).decode("ASCII")
            if out[-1] == '\n':
                out += f"{get_time()} "
        
        if out != "" : 
            print(out, end="")
            save(out, port, subfolder=start_time)
