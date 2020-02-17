import csv
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
from scipy import interpolate
from tqdm import tqdm
import time
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
matplotlib.rcParams['figure.figsize'] = [12, 5]



#data importer class
#expects certain input format I should look that up

class hardware_safety():
    def __init__(self):
        
        self.inches_per_step = 0.00078
        self.run_time = None
        
        self.init_gpio()
        
    def init_gpio(self):
        GPIO.setmode(GPIO.BCM)
        self.control_pins = [17, 27]
        for pin in self.control_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, 0)
        self.input_pins = [19, 26]
        for pin in self.input_pins:
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            
    def zero(self):
        while GPIO.input(26):
            self.step(-1)
            time.sleep(0.0005)
        
        for i in range(int(7131/2)):
            self.step(1)
            time.sleep(0.0005)
    
    def step(self, direction):
        seq1 = [
            [1,0],
            [0,0],
        ]
        seq2 = [
            [1,1],
            [0,1],
        ]
        
        if direction == 1:
            for halfstep in range(2):
                GPIO.output(self.control_pins, seq1[halfstep])
        elif direction == -1: #-1
            for halfstep in range(2):
                GPIO.output(self.control_pins, seq2[halfstep])
                
    def get_limit_switch_states(self):
        #returns right switch, left switch states
        return GPIO.input(26), GPIO.input(19)
    
    def load_trajectory(self, earthquakeFile = 'earthquake.csv', percentEarthquake=1, dt = 0.001, amplitude_scaling = 0.5):
        self.earthquake = earthquake_reader(earthquakeFile, percentEarthquake = percentEarthquake, dt = dt, step_size = 2*np.pi/180/25, amplitude_scaling = amplitude_scaling)
        self.earthquake.fit_spline()
        self.earthquake.interpolate_earthquake(time_stretch = 50)
        
        #earthquake.sine(0.2)
        self.earthquake.calculate_steps()

        
    def run_trajectory(self, limit_switch_safety = True):

        start = time.time()
        dt = self.earthquake.dt/30
        A = self.earthquake.relativeSteps
        a = np.zeros(len(A))

        for i in range(len(A)):
            
            if limit_switch_safety:
                if not GPIO.input(19) or not GPIO.input(26):
                    print('Hit a limit switch, should not have')
                    break
                
            start = time.time()
            val = A[i]
            self.step(val)
            while(time.time()-start<dt):
                wait = 1
            a[i] = (((time.time()-start)-dt)**2)**0.5
            
        self.run_time = time.time() - start

class earthquake_reader():
    def __init__(self, csv_file_name, percentEarthquake, dt, step_size, amplitude_scaling):
        
        self.dt = dt
        self.step_size = step_size
        self.amplitude_scaling = amplitude_scaling
        
        #load file
        self.earthquake_data = []
        with open(csv_file_name, 'r') as csvfile:
            spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')
            for row in spamreader:
                self.earthquake_data.append(row)
                
        #strip amplitude data
        amplitude = [value[1] for value in self.earthquake_data[9:]]
        
        self.amplitude = np.array(amplitude).astype('float')
        self.amplitude = self.amplitude[:int(percentEarthquake*self.amplitude.shape[0])]
        self.time = np.arange(0,self.dt*self.amplitude.shape[0],self.dt)
        
        self.dt_interpolated = None
        self.amplitude_interpolated = None
        self.time_interpolated = None
        
    def sine(self, frequency):
        amplitude = np.max(np.abs(self.amplitude_interpolated))
        for i in range(self.amplitude_interpolated.shape[0]):
            self.amplitude_interpolated[i] = np.sin(i*frequency*2*np.pi*self.dt)*amplitude
        
    def fit_spline(self):
        self.f = interpolate.interp1d(self.time, self.amplitude) #, kind='cubic')

    def interpolate_earthquake(self, time_stretch):
        self.dt_interpolated = self.dt / time_stretch
        self.time_interpolated = np.arange(0, self.time[-1], self.dt_interpolated)
        self.amplitude_interpolated = self.f(self.time_interpolated)
        self.time_interpolated *= time_stretch
        
    def calculate_steps(self):
        self.amplitude_interpolated_scaled = self.amplitude_interpolated * self.amplitude_scaling
        self.da = self.amplitude_interpolated_scaled[1:]-self.amplitude_interpolated_scaled[:-1]
        
        if np.abs(np.max(self.da)) > 1:
            print('Scaling constant or control frequency is too low')
            
            
        self.relativeSteps = np.zeros(self.amplitude_interpolated_scaled.shape[0])
        relativeStepsSum = 0
        for i in tqdm(range(self.amplitude_interpolated_scaled.shape[0])):
            if i > 0:
                relativeStepsSum += self.relativeSteps[i-1]
            if self.amplitude_interpolated_scaled[i] - relativeStepsSum >= 1:
                self.relativeSteps[i] = 1
            elif self.amplitude_interpolated_scaled[i] - relativeStepsSum <= -1:
                self.relativeSteps[i] = -1
