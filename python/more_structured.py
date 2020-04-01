import numpy as np
import sounddevice as sd
from scipy.signal import chirp, sweep_poly
from scipy.io.wavfile import write
import time

fs = 44100

phase = 90 * 2 * np.pi / 360
#pase = 0

base=440

intervals = {
    "octave": (2.0/1.0),
    "fifth":  (3.0/2.0),
    "fourth": (5.0/4.0)
}

is_playing=False

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

def play_two_waves(wave_A, wave_B, block=True, loop=False):
    stereo_wave = np.column_stack((wave_A, wave_B))
    sd.play(stereo_wave, fs, blocking=block, loop=loop)
    #sd.wait()

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


last_left=0
last_right=0

def compare_two_notes(left_frequency, right_frequency):
    blocking=False
    duration = 0.6
    global last_left, last_right
    if last_left == left_frequency:
        left_wave = gen_mono(left_frequency, duration, phase)
    else:
        left_wave = gen_sweep(last_left, left_frequency, 0.5)
        left_wave = np.append(left_wave, gen_mono(left_frequency, (duration-0.5), phase))
    
    if last_right == right_frequency:
        right_wave = gen_mono(right_frequency, duration)
    else:
        right_wave = gen_sweep(last_right, right_frequency, 0.5)        
        right_wave = np.append(right_wave, gen_mono(right_frequency, (duration-0.5)))
    
    play_two_waves(left_wave, right_wave, block=True, loop=False)
    
    right_wave = gen_mono(right_frequency, duration)
    left_wave = gen_mono(left_frequency, duration, phase)
    play_two_waves(left_wave, right_wave, block=False, loop=True)
    last_left=left_frequency
    last_right=right_frequency
    return


base_frequency = 440
fifth = get_interval("fifth", base_frequency)
fourth = get_interval("fourth", base_frequency)
octave = get_interval("octave", base_frequency)

while True:

    compare_two_notes(440, 440)

    time.sleep(3)

    compare_two_notes(fifth, 440)

    time.sleep(3)

    compare_two_notes(fourth, 440)

    time.sleep(3)

    compare_two_notes(440, 880)

    time.sleep(3)

exit()

#fifth begins rotating
wave_1 = gen_mono(880, 5.0)
wave_2 = gen_sweep(get_interval("fifth", 880), (get_interval("fifth", 880) - 5), 5.0)

#quickly sweep to close to unity
wave_1 = np.append(wave_1, gen_mono(880, 1.0))
wave_2 = np.append(wave_2, gen_sweep((fifth - 5), 880, 1.0))

#slowly sweep the rest of the way 
wave_1 = np.append(wave_1, gen_mono(880, 5.0))
wave_2 = np.append(wave_2, gen_sweep(875, 880, 5.0))

#one second circle
wave_1 = np.append(wave_1, gen_mono(880, 1.0))
wave_2 = np.append(wave_2, gen_mono(880, 1.0, phase))

wave_1 = np.append(wave_1, gen_mono(880, 5.0))
wave_2 = np.append(wave_2, gen_sweep(880, fourth-5, 5.0))

wave_1 = np.append(wave_1, gen_mono(880, 1.0))
wave_2 = np.append(wave_2, gen_sweep(fourth-5, fourth, 1.0))

wave_1 = np.append(wave_1, gen_mono(880, 5.0))
wave_2 = np.append(wave_2, gen_mono(fourth, 5.0))

wave_1 = np.append(wave_1, gen_mono(880, 1.0))
wave_2 = np.append(wave_2, gen_sweep(fourth, fourth+5, 1.0))

wave_1 = np.append(wave_1, gen_mono(880, 1.0))
wave_2 = np.append(wave_2, gen_sweep(fourth+5, fifth-5, 1.0))

wave_1 = np.append(wave_1, gen_mono(880, 2.0))
wave_2 = np.append(wave_2, gen_sweep(fifth-5, fifth, 2.0))


wave_1 = np.append(wave_1, gen_mono(880, 5.0))
wave_2 = np.append(wave_2, gen_mono(fifth, 5.0))


play_two_waves(wave_1, wave_2)
#stereo_wave = np.column_stack((wave_1, wave_2))
#write("stereo_tone.wav", fs, stereo_wave)

