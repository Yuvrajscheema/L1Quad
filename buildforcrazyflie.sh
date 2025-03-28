cd ardupilot
./waf distclean
./waf configure --board crazyflie2
./waf copter -j8 # j8 works fore machines with at least 8 cores, change to the number of cores you have