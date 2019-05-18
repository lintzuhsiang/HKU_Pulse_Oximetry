# HKU_Pulse_Oximetry

We use Red LED and Infrared Red LED as light source and a Photoresistor to measure the values.
Use Butterworth filter to get better analog signals and sampling frequency is 80Hz to get enough accuracy.
Code Optimization:
  * Using charging and discharging time of Photoresistor to calculate and display on TFT screen to achieve 80Hz sampling rate.
  * Instead erasing and writing every vertical line at a time, erasing and writing the previous pixel.
  * Separate calculation and display section, calculate nine times and display one time.

Calculate SpO2 and Heart Rate and display the Red LED wave and Infrared Red LED wave onto TFT screen.
Save data into SD card for later use.
