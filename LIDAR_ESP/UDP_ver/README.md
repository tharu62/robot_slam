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

|   | **HEADER**    | **ANGLE[Grad]**   | **RPM** | **DATA 1**     | **DATA 2**     | **DATA 2**     | **DATA 4**     | **CHECKSUM** |   |
|---|---------------|-------------------|---------|----------------|----------------|----------------|----------------|--------------|---|
|   | 1 byte        | 2 byte            | 2 byte  | 4 byte         | 4 byte         | 4 byte         | 4 byte         | 2 byte       |   |
|   | constant **0xfa** | from **0xa0** to **0xfa** | float   | Data Structure | Data Structure | Data Structure | Data Structure | int          |   |

### HEADER

The header is a constat value always sent at the start of a scan message

### ANGLE

The angle is in a range from 0 to 89 degrees (90 tot). Every scan message is composed of 4 scanned angles: 
taken **i** as the angle given by the message the actual angles measured are: {4***i**, 4***i**+1, 4***i**+2, 4***i**+3}

### RPM

The Lidar returns the rpm measured as a float value: 

in c/c++:

float(packet[2] | ((packet[3]<<8))) / 64.f;

### Data Strucure

The data is composed of a distance in mm and a signal strength

**Data:**

| first 2 bytes | last 2 bytes |

| distance | signal strength |