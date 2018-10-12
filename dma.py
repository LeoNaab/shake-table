import pigpio
from time import sleep
from collections import deque
import numpy as np

cw = (
  (1, 0),
  (0, 0)
)




class StepperMotor:

  def __init__(self, pi, pin1, pin2, sequence = cw, delayAfterStep = 0.0001):
    if not isinstance(pi, pigpio.pi):
      raise TypeError("Is not pigpio.pi instance.")
    pi.set_mode(pin1, pigpio.OUTPUT)
    pi.set_mode(pin2, pigpio.OUTPUT)
    self.pin1 = pin1
    self.pin2 = pin2
    self.wave_size = 1000
    self.pi = pi
    self.delayAfterStep = delayAfterStep
    self.deque = deque(sequence)



  def doStepAndDelay(self, step):
    self.pi.write(self.pin1, step[0])
    self.pi.write(self.pin2, step[1])
    sleep(self.delayAfterStep)

  def generateWave_us(self, us):
    self.pi.wave_clear()
    self.wave = []
    for i in range(1000):
      self.wave.append(pigpio.pulse(1<<self.pin1 | 1<<self.pin2, 0, us))
      self.wave.append(pigpio.pulse(1<<self.pin2, 1<<self.pin1, us))
    for i in range(1000):
      self.wave.append(pigpio.pulse(1<<self.pin1, 1<<self.pin2, us))
      self.wave.append(pigpio.pulse(0, 1<<self.pin1 | 1<<self.pin2, us))

  def waveStep(self, wave_input):

    self.generateWave_us(100)
    self.pi.wave_add_generic(self.wave)    
    wave_id = pi.wave_create()
    
    cbs = pi.wave_send_repeat(wave_id)
    sleep(2)
    self.pi.wave_tx_stop()

    self.generateWave_us(25)
    self.pi.wave_add_generic(self.wave)    
    wave_id = pi.wave_create()
    
    cbs = pi.wave_send_repeat(wave_id)
    sleep(2)
    self.pi.wave_tx_stop()


  def waveChain(self, wave_input):

    self.generateWave_us(20)
    self.pi.wave_add_generic(self.wave)
    wave_id = pi.wave_create()

    chain = []
    for i in range(50):
      chain += [wave_id]
      
    #for i in range(10):
    #  chain += [255, 0, wid[i], 255, 1, x, y]
    pi.wave_chain(chain) # Transmit chain.

    while pi.wave_tx_busy(): # While transmitting.
      sleep(0.1)

    # delete all waves
    for i in range(l):
      pi.wave_delete(wid[i])

  def chunks(self, n):
    l = self.wave
    n = max(1, n)
    return (l[i:i+n] for i in range(0, len(l), n))

  def waveChainInput(self, wave_input, us):
    pi.wave_clear()   
    self.wave = []
    for step in wave_input:
      if step == 1: #forward
        self.wave.append(pigpio.pulse(1<<self.pin1 | 1<<self.pin2, 0, us))
        self.wave.append(pigpio.pulse(1<<self.pin2, 1<<self.pin1, us))
      elif step == -1:
        self.wave.append(pigpio.pulse(1<<self.pin1, 1<<self.pin2, us))
        self.wave.append(pigpio.pulse(0, 1<<self.pin1 | 1<<self.pin2, us))
      elif step == 0:
        self.wave.append(pigpio.pulse(0, 1<<self.pin1 | 1<<self.pin2, us))
        self.wave.append(pigpio.pulse(0, 1<<self.pin1 | 1<<self.pin2, us))

    if len(self.wave)%self.wave_size != 0:
      for i in range(self.wave_size - len(self.wave)%self.wave_size):
        self.wave.append(pigpio.pulse(0, 1<<self.pin1 | 1<<self.pin2, us))

    #self.waves = self.chunks(len(self.wave) / 2000)

    print('Number of waves: {}'.format(len(self.wave)))

    self.wid = []
    for i in range(int(len(self.wave) / self.wave_size)):
      pi.wave_add_generic(self.wave[i*self.wave_size:(i+1)*self.wave_size])
      wave_id = pi.wave_create()
      self.wid.append(wave_id)

    self.chain = []
    for i in range(50):
      for wave_id in self.wid:
        self.chain += [wave_id]
    #pi.wave_add_generic(self.wave)
    #wave_id = pi.wave_create()

    #chain = []
    #for i in range(50):
      #chain += [wave_id]
      
    #for i in range(10):
    #  chain += [255, 0, wid[i], 255, 1, x, y]
    pi.wave_chain(self.chain) # Transmit chain.

    while pi.wave_tx_busy(): # While transmitting.
      sleep(0.1)

    # delete all waves
    for i in range(l):
      pi.wave_delete(wid[i])
      
  def generateStepArray(self, trajectory, stepSize):
    stepTrajectory = np.array(trajectory) / stepSize #calculates trajectory in number of steps
    relativeSteps = np.zeros(len(trajectory))
    if (np.abs(stepTrajectory[:-1] - stepTrajectory[1:]) > 1).any():
        error = np.abs(stepTrajectory[:-1] - stepTrajectory[1:]).max()
        print("error, trajectory is not fine enough, error mag is {}".format(error))
        return error, -1
    else:
        #np.round(trajectory[1:] - trajectory[:-1]) this won't work...
        for i in range(len(stepTrajectory)):
            if stepTrajectory[i] - np.sum(relativeSteps) >= 1:
                relativeSteps[i] = 1
            elif stepTrajectory[i] - np.sum(relativeSteps) <= -1:
                relativeSteps[i] = -1
    return relativeSteps, 1

  def runLongWave(self, trajectory, us):
    self.pi.wave_clear()   
    self.wave = []
    for step in trajectory:
      if step == 1: #forward
        self.wave.append(pigpio.pulse(1<<self.pin1 | 1<<self.pin2, 0, us))
        self.wave.append(pigpio.pulse(1<<self.pin2, 1<<self.pin1, us))
      elif step == -1:
        self.wave.append(pigpio.pulse(1<<self.pin1, 1<<self.pin2, us))
        self.wave.append(pigpio.pulse(0, 1<<self.pin1 | 1<<self.pin2, us))
      elif step == 0:
        self.wave.append(pigpio.pulse(0, 1<<self.pin1 | 1<<self.pin2, us))
        self.wave.append(pigpio.pulse(0, 1<<self.pin1 | 1<<self.pin2, us))

    if len(self.wave)%self.wave_size != 0:
      for i in range(self.wave_size - len(self.wave)%self.wave_size):
        self.wave.append(pigpio.pulse(0, 1<<self.pin1 | 1<<self.pin2, us))

    old_id = -1
    wave_pos = 0
    while wave_pos <= len(self.wave) - self.wave_size:
      slice = self.wave[wave_pos:wave_pos + self.wave_size]
      wave_pos += self.wave_size

      
      self.pi.wave_add_generic(slice)
      new_id = self.pi.wave_create()
      self.pi.wave_send_using_mode(new_id, pigpio.WAVE_MODE_ONE_SHOT_SYNC)

      while self.pi.wave_tx_at() != new_id:
        sleep(0.01)

      old_id = new_id
      
      if old_id >= 0:
        self.pi.wave_delete(old_id)

      #old_id = new_id
      
    

if __name__ == '__main__':
  pi = pigpio.pi()
  motor = StepperMotor(pi, 2, 3)

  #motor.waveStep([1])
  #sub_trajectory = [1]*10000 + [0]*10000 + [1]*10000
  #trajectory = []
  #for i in range(10):
  #  trajectory += sub_trajectory


  trajectory = list(np.ones(20000).astype(int))
  #trajectory = list(np.random.randint(0,2,20000).astype(int))
  
  #motor.waveChainInput(trajectory, 100)
  motor.runLongWave(trajectory, 25)
  #for i in range(5000):
  #  motor.doStepAndDelay(cw[0])
  #  motor.doStepAndDelay(cw[1])
  
  pi.stop()
