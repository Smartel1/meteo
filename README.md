Just another meteostation.

Measures wind speed, azimuth of wind, temperature, pressure and voltage
and sends this metrics to the server via GPRS (needs sim card).
After sending the data meteostation goes into deep sleep mode and wakes up in 5 minutes to repeat the cycle.


This code is designed and tested on the hardware: 
1. lilygo t-call 1.4 sim800
2. hmc5883l magnetometer (to measure wind azimuth)
3. a3144 hall sensor to measure wind speed (rotation speed of anemometer)
4. bmp280 to measure temperature and air pressure

In order to get precise values meteostation should be calibrated.
You need to get coefficients for compass by calling calibrate_compass() (read comments in code)
and find correct coefficient to decode anemometer rotation speed to wind speed (use another anemometer as standard) 