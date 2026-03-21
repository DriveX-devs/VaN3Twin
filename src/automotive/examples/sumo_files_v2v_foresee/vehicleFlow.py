#!/usr/bin/env python

#example of usage:  ./vehicleFlow.py cars.rou.xml -m 5 -r 0.7 -s 13.89 -t 20
import os
import sys
import random
import datetime
import shutil
from optparse import OptionParser

def get_options():
    optParser = OptionParser()
    optParser.add_option(
        "-r", "--rate", type="float", default=0.1, help="creation rate")
    optParser.add_option(
        "-m", "--minGap", type="float", default=2.0, help="vehicle min gap, negative numbers denote the center of a uniform distribution [x-0.2, x+0.2]")
    optParser.add_option(
        "-s", "--maxSpeed", type="float", default=40, help="vehicle max speed, negative numbers denote the center of a uniform distribution [x-3, x+3]")
    optParser.add_option(
        "-d", "--speedDev", type="float", default=0, help="speed dev (see SUMO spec)")
    optParser.add_option(
        "-f", "--speedFactor", type="float", default=0, help="speed factor (see SUMO spec)")
    optParser.add_option(
        "-b", "--begin", type="int", default=0, help="begin time")
    optParser.add_option(
        "-e", "--end", type="int", default=1000, help="end time")
    optParser.add_option("-i", "--index", type="int",
                         default=1, help="starting index for naming vehicles")
    optParser.add_option(
        "-n", "--name", default="veh", help="base name for vehicles")
    optParser.add_option(
        "-t", "--total", default=0, help="Total number of vehicles to be created. If 0, begin and end are used to create the vehicles")
    
    (options, args) = optParser.parse_args()
    
    options.output = args[0]
    return options

def line_prepender(filename, line):
    with open(filename, 'r+') as f:
        content = f.read()
        f.seek(0, 0)
        f.write(line.rstrip('\r\n') + '\n' + content)

def write_init(f,options):
    f.write('<routes>\n')
    f.write('  <route id="0" edges="ne_to_nw"/>\n')
    f.write('  <route id="1" edges="nw_to_ne"/>\n')
 
    f.write('\n')
    f.write('  <vType carFollowModel="IDM" tau="0.8" lcDuration="1.5" accel="3" decel="3" emergencyDecel="7" minGap="2.0" id="Car0" maxSpeed="40" lcKeepRight="0" lcContRight="0" lcSpeedGain="0" lcCooperative="-1" lcStrategic="-1"/>\n')
    f.write('  <vType carFollowModel="IDM" tau="1.0" lcDuration="1.5" accel="3" decel="3" emergencyDecel="7" minGap="2.0" id="Truck" maxSpeed="40" lcKeepRight="0" lcContRight="0" lcSpeedGain="0" lcCooperative="-1" lcStrategic="-1"/>\n')
    f.write('\n')

def write_veh(f, name, car_type, index, depart, route_var, departLane, desired_speed):
    # Random route
    #in this way we can have circular rotation among the street but random choice of the path
    f.write('      <vehicle id="%s%s" type="%s" depart="%s" route="%s" departLane="%s" arrivalLane="%s">\n' %
        (name, index, car_type, depart, route_var, departLane, departLane))
    f.write('    </vehicle>\n')

def main():
    avg_speed = 33.3  # m/s
    deviation = 0.2
    options = get_options()
    i = 0
    departLane = 0
    with open(options.output, 'w') as f:
        f.write('<!-- generated on %s by "%s" -->\n' %
                (datetime.datetime.now(), os.path.basename(" ".join(sys.argv))))
        write_init(f, options)
        index = options.index
        # Time instant in which the last car is scheduled to depart
        last_depart = options.begin*10.0/10.0
        if options.total == 0:
            while last_depart < options.end:
                desired_speed = random.uniform(avg_speed * (1 - deviation), avg_speed * (1 + deviation))
                intertime = 1.5
                route_var = random.choice([0, 1])
                write_veh(f, options.name, random.choice(["Car0", "Truck"]), index, last_depart+intertime, route_var, departLane, desired_speed)
                last_depart += intertime
                index += 1
                departLane += 1
                if (departLane > 2):
                    departLane = 0
        else:
            while i < int(options.total):
                desired_speed = random.uniform(avg_speed * (1 - deviation), avg_speed * (1 + deviation))
                intertime = 1.5
                route_var = random.choice([0, 1])
                write_veh(f, options.name, random.choice(["Car0", "Truck"]), index, last_depart+intertime, route_var, departLane, desired_speed)
                last_depart += intertime
                index += 1
                i += 1
                departLane += 1
                if (departLane > 2):
                    departLane = 0
        f.write('</routes>')
    index = int(index)-1
    line = "<!-- number of vehicles:%d -->\n" % index
    line_prepender(options.output, line)

if __name__ == "__main__":
    main()
