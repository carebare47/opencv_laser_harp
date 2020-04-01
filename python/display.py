import numpy as np
import sounddevice as sd
from scipy.signal import chirp, sweep_poly
from scipy.io.wavfile import write

fs = 44100















phase = 90 * 2 * np.pi / 360
#pase = 0

base=440

intervals = {
    "octave": (2.0/1.0),
    "fifth":  (3.0/2.0),
    "fourth": (5.0/4.0)
}


def draw_line(start_x, start_y, end_x, end_y):
    


def gen_stereo(left_frequency, right_frequency, duration):
    wave_A = (np.sin(2 * np.pi * np.arange(fs * duration) * left_frequency / fs + phase)).astype(np.float32)
    wave_B = (np.sin(2 * np.pi * np.arange(fs * duration) * right_frequency / fs)).astype(np.float32)
    print wave_A.size
    print wave_B.size
    stereo_wave = np.column_stack((wave_A, wave_B))
    return stereo_wave

def gen_mono(frequency, duration, phase=0):
    wave_A = (np.sin(2 * np.pi * np.arange(fs * duration) * frequency / fs + phase)).astype(np.float32)
    return wave_A    

def play_stereo_tone(left_frequency, right_frequency, duration):
    sd.play(gen_stereo(left_frequency, right_frequency, duration), fs)
    sd.wait()

def gen_sweep(start, end, duration):
    t = np.linspace(0, duration, fs*duration)
    w = chirp(t, f0=start, f1=end, t1=duration, method='linear')
    return w
    
def play_two_waves(wave_A, wave_B):
    stereo_wave = np.column_stack((wave_A, wave_B))
    sd.play(stereo_wave, fs)
    sd.wait()
    
def gliss(start, end, hold_duration, gliss_duration):
    arr = gen_mono(start, hold_duration)
    arr = np.append(arr, gen_sweep(start, end, gliss_duration))
    arr = np.append(arr, gen_mono(end, hold_duration))
    return arr
    
def get_interval(interval, base_frequency):
    if interval not in intervals:
        print "\"", interval, "\" does not exist"
        return base_frequency
    else:
        return base_frequency*intervals[interval]


    
wave_1 = gliss(440, base*octave, 1.0, 1.0)
wave_2 = gen_mono(440, 3.0, phase)

wave_1 = np.append(wave_1, gliss(base*octave, base*fifth, 1.0, 1.0))
wave_2 = np.append(wave_2, gen_mono(440, 3.0))

wave_1 = np.append(wave_1, gliss(base*fifth, base*fourth, 1.0, 1.0))
wave_2 = np.append(wave_2, gen_mono(440, 3.0))

wave_1 = np.append(wave_1, gliss(base*fourth, 440, 1.0, 1.0))
wave_2 = np.append(wave_2, gen_mono(440, 3.0, phase))



play_two_waves(wave_1, wave_2)
#stereo_wave = np.column_stack((wave_1, wave_2))
#write("stereo_tone.wav", fs, stereo_wave)

