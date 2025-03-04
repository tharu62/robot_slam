# LDS01RR LIDAR DATA STRUCTURE

Data is sent through UART transmission at 115200 baud rate.
The data is composed of hexadecimal values (unsigned char).

Lidar data are sent in 2 modes:
* Calibration initial mode
* Data Mode

## Calibration Mode:

the data sent is structured as:

...

## Data Mode

The data has structure:

| HEADER | Angle | Rpm | Data 1 | Data 2 | Data 3 | Data 4 | Checksum |
| ------ | ----- | --- | ------ | ------ | ------ | ------ | -------- |
| 1 byte | 2 byte | 2 byte | 4 byte | 4 byte | 4 byte | 4 byte | 2 byte |
| constant 0xfa | from 0Xa0 to 0xfa | float | Data Structure | Data Structure | Data Structure | Data Structure | int |

## Data Strucure

the data is composed of a distance in mm and a signal strength

| Data |
| distance | signal strength |