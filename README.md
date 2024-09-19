![Solar Tracker image](/img/solar_tracker_main.JPG "Solar Tracker")

# Solar tracker with ARDUINO or ESP32 and soltrack-2.2 library

An embedded system that uses an Arduino or ESP32 microcontroller to accurately track the position of the sun and adjust the angle of a solar panel accordingly. By monitoring the sun's movement throughout the day, the Solar Tracker ensures that the solar panel is always in the optimal position for maximum energy production. The system uses precise calculations to determine the sun's position and employs relays to adjust panel angles for optimal performance.

>Features will differ depending on the microcontroller used. The ESP32 version has a wifi connection permitting remote monitoring through MQTT. RTC will automatically adjust the time and date through the internet. The Arduino version will have a manual adjustment of the time and date.

# Libraries

Thanks to the [soltrack-2.2 library](https://github.com/MarcvdSluys/SolTrack). This library is used to calculate the sun's position based on the date, time, and location. The library provides the azimuth and elevation angles of the sun, which are used to adjust the solar panel's position.