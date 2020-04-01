import numpy as np
import sounddevice as sd
from scipy.signal import chirp, sweep_poly

fs = 44100

phase = 90 * 2 * np.pi / 360
#pase = 0

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

wave_1 = gen_mono(400, 5.0)
wave_2 = gen_mono(400, 5.0, phase)


#wave_1 = np.append(wave_1, gen_sweep(800, 400, 1.0))
#wave_2 = np.append(wave_2, wave_2)

#wave_1 = np.append(wave_1, gen_sweep(400, 800, 1.0))
#wave_2 = np.append(wave_2, gen_mono(400, 1.0, phase))

#wave_1 = np.append(wave_1, gen_sweep(800, 400, 1.0))
#wave_2 = np.append(wave_2, gen_mono(400, 1.0, phase))



play_two_waves(wave_1, wave_2)


