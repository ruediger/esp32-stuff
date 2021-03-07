# WiFi Hello World: Echo

This is based on the station wifi example from esp-idf.


## How to use example

### Configure the project

```
idf.py menuconfig
```

* Set WiFi SSID and WiFi Password and Maximum retry under Example Configuration Options.
* For client usage: Set port, addr, message
* For server usage: Set listening port

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

### Echo Client

A simple way to run an echo client on another machine on the WiFi is using netcat:

```
nc -l -p PORT
```

### Echo Server

```
nc ADDR PORT
```
