# HKU_Pulse_Oximetry
Introduction
============
I Built a portable pulse oximetry by Arduino to monitor the heart rate and SpO2, and achieve 80 Hz refresh rate in TFT screen. 

1. Use photoresistor to collect absorption rate from red and infrared red LED two light source. 
2. Use Butterworth filter to get better analog signals, filter out frequency larger than 120 Hz, and sampling frequency is 80Hz to get enough accuracy.
3. Code Optimization:
  * Using charging and discharging time of Photoresistor to calculate and display on TFT screen to achieve 80Hz sampling rate.
  * When draw red and ingrared red LED pathon TFT screen, instead erasing and writing every vertical line at a time, erasing and writing the previous pixel.
  * Separate calculation and display section, calculate nine times and display one time.

4. Calculate SpO2 and Heart Rate and display the Red LED wave and Infrared Red LED wave onto TFT screen.
5. Save data into SD card for later use.




I also use a high speed camera to collect pictures of different algae to build an auto classification model 
  1.	Adjust pumping speed to get clear and stable video stream
  2.	Use Matlab to crop potential algae image from video stream
