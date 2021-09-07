from datetime import datetime
import minimalmodbus
import math
import time

# sample period (in seconds)
Ts = 60

# initialize data averaging variables
N = 0
p = 0
T = 0

# open serial device
instrument = minimalmodbus.Instrument('/dev/ttyUSB0',1)
instrument.serial.baudrate = 9600

# data collection (inifinite) loop
while True:

  # get the current time, time of current sample, and time to stop sampling
  t = time.time()
  t_center = (math.floor(t / Ts) + 1) * Ts
  t_stop = t_center + 0.5 * Ts

  # loop over one sample period
  while (t < t_stop):

    # get pressure and temperature and add to running sum
    p += instrument.read_float(2,3,2)
    T += instrument.read_float(8,3,2)

    # increment loop counter and update current time
    N += 1
    t = time.time()

  # compute average pressure and temperature
  p = 1E5*p/N
  T = T/N

  # create filename
  YYYY = datetime.fromtimestamp(t_center).strftime('%Y')
  JJJ = datetime.fromtimestamp(t_center).strftime('%j')
  fn = '/home/sdewolf/Data/%s.%s.txt'%(YYYY,JJJ)

  # open file, create and write output string to file, and close file
  out_file = open(fn,'a')
  out_string = '%s,%0.6f,%0.6f\n'%(datetime.fromtimestamp(t_center).strftime('%Y-%m-%dT%H:%M:%S.%f'),p,T)
  out_file.write(out_string)
  out_file.close()

  # display output to command line
  dt_string = datetime.fromtimestamp(t_center).strftime('%Y:%j:%H:%M:%S.%f')
  print('%s  N = %i  p = %0.6f  T = %0.6f'%(dt_string,N,p,T))

  # reset loop variables
  N = 0
  p = 0
  T = 0
