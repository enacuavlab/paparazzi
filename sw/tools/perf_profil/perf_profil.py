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

event_vector = None

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

                if not (name in ['periodic_start', 'periodic_end', 'event_start', 'event_end', 'event']):
                    (period, duty, previous) = time_vector[name]
                    if previous is None:
                        time_vector[name] = (period, duty, time)
                    else:
                        p = time - previous
                        dt = new_time - time
                        if p < 0:
                            p = p + 2**32/sysclk
                        time_vector[name] = (
                                np.append(period, p),
                                np.append(duty, dt),
                                time)
                elif not (name in ['periodic_start', 'periodic_end', 'event']):
                    split = name.split('_', 1)
                    base = split[0]
                    tail = split[1]
                    if tail == "end":
                        (period, duty, previous) = time_vector[name]
                        (_, _, start) = time_vector["{}_start".format(base)]
                        #print(name, base, tail, previous, start)
                        if previous is None:
                            time_vector[name] = (period, duty, time)
                        elif start is not None:
                            p = time - previous
                            dt = new_time - start
                            #print(name, dt, df, time, new_time, previous)
                            if p < 0:
                                p = p + 2**32/sysclk
                            time_vector[name] = (
                                    np.append(period, p),
                                    np.append(duty, dt),
                                    time)
                    else: # tail = start
                        time_vector[name] = (np.zeros(0), np.zeros(0), time)
                else:
                    pass

                last_data = data
                #if data[1] == 'main':
                #    time_vector[data[1]] = np.append(time_vector[data[1]], [int(data[2])])
                #    #print(data[1], data[2], time_vector[data[1]])
            except KeyboardInterrupt:
                print("stop loop by hand")
                break
            except:
                print("invalid time or name at line", line)
        elif len(data) == 7 and data[0] == 'PPTE':
            try:
                name = data[1]
                if name == 'event':
                    nb_sample = int(data[2])
                    nb_over = int(data[3])
                    dt = float(data[4])
                    dt_min = float(data[5])
                    dt_max = float(data[6])
                    if event_vector is None:
                        event_vector = (np.array(nb_sample), np.array(nb_over), np.array(dt / float(nb_sample)), np.array(dt_min), np.array(dt_max))
                    else:
                        event_vector = (
                                np.append(event_vector[0], nb_sample),
                                np.append(event_vector[1], nb_over),
                                np.append(event_vector[2], dt / float(nb_sample)),
                                np.append(event_vector[3], dt_min),
                                np.append(event_vector[4], dt_max)
                                )
            except KeyboardInterrupt:
                print("stop loop by hand")
                break
            except:
                print("invalid event at line", line)
        else:
            print("invalid data at line:", line)

# print results
print("name                (samples)\t| period (us) \t[std     ] | freq (Hz)\t| duty (us) \t[std     ] [min     ] [max     ] [nb over ]")
for key in time_vector:
    if not (key in ['periodic_start', 'periodic_end', 'event_start']):
        (p,d,t) = time_vector[key]
        if t is not None and len(p) > 1:
            print("{:<20}({})\t| {:.2f} \t[{:<8.3f}] | {:<8.3f}\t| {:.3f} \t[{:<8.3f}] [{:<8.3f}] [{:<8.3f}] [{:<8d}]".format(key, len(p),
                np.mean(p), np.std(p), 1e6/np.mean(p),
                np.mean(d), np.std(d), np.min(d), np.max(d),
                (d > np.mean(p)).sum()))

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

if event_vector is not None:
    print("{:<20}({}*{})\t| {:.2f} \t[{:<8.3f}] | {:<8.3f}\t| {} \t[{:<8}] [{:<8.3f}] [{:<8.3f}] [{:<8d}]".format('event',
        len(event_vector[0]), event_vector[0][0],
        np.mean(1000*event_vector[2]), np.std(1000*event_vector[2]), 1e6/np.mean(1000*event_vector[2]),
        'N/A\t', 'N/A', np.min(event_vector[3]), np.max(event_vector[4]),
        np.sum(event_vector[1])))


