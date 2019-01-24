#!/usr/bin/env python

# Current GPS Coordinates
current_lat = 22.3118631
current_lon = 39.0952322
current_alt = 5.2

# Current local ENU coordinates
current_local_x = 0.0
current_local_y = 0.0
current_local_z = 0.0

worker1_found_flag = 0

current_state = 'Takeoff'
current_signal = 'Running'

#---------Data Log Variables----------#
#######################################
datalog_counter = 60

if datalog_counter > 50:
	hs = open("DataLog.txt","a")
	s = "Current Latitude\n"
	s = s + str(current_lat)
	s = s + "\nCurrent Longitude\n"
	s = s + str(current_lon)
	s = s + "\nCurrent Altitude\n"
	s = s + str(current_alt)
	s = s + "\nCurrent State\n"
	s = s + current_state
	s = s + "\nCurrent Signal\n"
	s = s + current_signal
	s = s + "\n\n"
	hs.write(s)
	hs.close()
	datalog_counter = 0
else:
	datalog_counter = datalog_counter + 1

			# NOTES: We will need to figure out how to capture an image once the drone correctly identifies a worker or tag. I'm not
			# sure how to do this right now

			# NOTES: Will also
