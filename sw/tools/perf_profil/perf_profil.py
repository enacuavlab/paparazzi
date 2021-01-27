#!/usr/bin/python3

'''
Tools to analyse perf_profil module output
Provides performance profiling information on the AP functions
'''

import sys
import numpy as np
import matplotlib.pyplot as plt

sysclk = 216 # MHz

if len(sys.argv) == 2:
    data_file = sys.argv[1]
else:
    print("Please provide data file")
    exit(1)

time_vector = {
        "init_start":       (np.zeros(0), np.zeros(0), None),
        "init_end":         (np.zeros(0), np.zeros(0), None),
        "periodic_start":   (np.zeros(0), np.zeros(0), None),
        "main":             (np.zeros(0), np.zeros(0), None),
        "ap_gen":           (np.zeros(0), np.zeros(0), None),
        "ap_static":        (np.zeros(0), np.zeros(0), None),
        "sensors":          (np.zeros(0), np.zeros(0), None),
        "attitude":         (np.zeros(0), np.zeros(0), None),
        "modules_sync":     (np.zeros(0), np.zeros(0), None),
        "modules":          (np.zeros(0), np.zeros(0), None),
        "radio":            (np.zeros(0), np.zeros(0), None),
        "failsafe":         (np.zeros(0), np.zeros(0), None),
        "monitor":          (np.zeros(0), np.zeros(0), None),
        "electrical":       (np.zeros(0), np.zeros(0), None),
        "telemetry":        (np.zeros(0), np.zeros(0), None),
        "baro":             (np.zeros(0), np.zeros(0), None),
        "periodic_end":     (np.zeros(0), np.zeros(0), None),
        "event_start":      (np.zeros(0), np.zeros(0), None),
        "event_end":        (np.zeros(0), np.zeros(0), None)
        }
last_time = None
last_data = None

with open(data_file) as f:
    for line in f:
        data = line.split()
        if len(data) == 3 and data[0] == 'PPT':
            if last_data is None:
                last_data = data
                continue
            try:
                name = last_data[1]
                time = int(last_data[2])
                new_time = int(data[2])

                if not (name in ['init_start', 'init_end', 'periodic_start', 'periodic_end', 'event_start', 'event_end']):
                    (freq, duty, previous) = time_vector[name]
                    if previous is None:
                        time_vector[name] = (freq, duty, time)
                    else:
                        df = time - previous
                        dt = new_time - time
                        #print(name, dt, df, time, new_time, previous)
                        if df < 0:
                            df = df + 2**32/sysclk
                        time_vector[name] = (
                                np.append(freq, df),
                                np.append(duty, dt),
                                time)
                else:
                    pass

                last_data = data
                #if data[1] == 'main':
                #    time_vector[data[1]] = np.append(time_vector[data[1]], [int(data[2])])
                #    #print(data[1], data[2], time_vector[data[1]])
            except KeyboardInterrupt:
                exit(1)
            except:
                print("invalid time or name at line", line)
        else:
            print("invalid data at line:", line)

# print results
for key in time_vector:
    (p,d,t) = time_vector[key]
    if t is not None:
        print("{:<20}({})\t| period: {:.2f} \t({:.3f}) (f={:.3f})\t| duty: {:.3f} ({:.3f})".format(key, len(p), np.mean(p), np.std(p), 1e6/np.mean(p), np.mean(d), np.std(d)))

    if False:
        i = np.arange(0, len(d))
        plt.figure()
        plt.plot(i, p)
        plt.xlabel('sample')
        plt.ylabel('usec')
        plt.title('{} period'.format(key))
        plt.figure()
        plt.plot(i, d)
        plt.xlabel('sample')
        plt.ylabel('usec')
        plt.title('{} duty'.format(key))
        plt.show()

