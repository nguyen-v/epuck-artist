
# epuck-artist
<img src="./misc/banner.png"/>

An edge processing, 4-color equipped wall plotter. 

Embedded Systems and Robotics semester project at [EPFL](https://www.epfl.ch/) using the e-puck 2 robot from [GCtronic](https://www.gctronic.com/).

## Features
- Reproduction of any subject (100 x 90) in 4 different colors
- Semi-automatic calibration
- Interactive starting position configuration
## Requirements
### Python 3.x
#### External libraries
  - serial
  - PIL
  - numpy

### VSCode with PlatformIO IDE
#### Libraries
  - NeoSWSerial
  - ServoTimer2
  - Stepper

### Eclipse IDE
#### Libraries
  - [e-puck2_main_processor](https://github.com/e-puck2/e-puck2_main-processor) ([wiki](https://www.gctronic.com/doc/index.php?title=e-puck2_robot_side_development))

### Hardware
| Peripheral                  | Model                                                                                                    |
|-----------------------------|----------------------------------------------------------------------------------------------------------|
| Robot                       | [e-puck2 (GCtronic)](https://www.gctronic.com/e-puck2.php)                                               |
| Board                       | [Arduino Nano ATMega328](https://store.arduino.cc/arduino-nano)                                          | 
| Bluetooth module            | [Velleman HC-05](https://www.velleman.eu/products/view/?id=435518)                                       | 
| Servo motor                 | [ST55MG](https://amewi.com/AMX-Racing-Micro-Digital-Servo-ST55MG)                                        |
| Stepper motor and driver    | [Gear Stepper Motor Driver Pack](https://www.seeedstudio.com/Gear-Stepper-Motor-Driver-Pack-p-3200.html) |


## Demos:
### Live demo
<a href="http://www.youtube.com/watch?feature=player_embedded&v=znKsJ0n5lfQ
" target="_blank"><img src="https://i.imgur.com/wOXOsNd.png" 
alt="IMAGE ALT TEXT HERE" /></a>

### Timelapses
#### Colorful concentric circles
<a href="http://www.youtube.com/watch?feature=player_embedded&v=ZIJO9M60r-s
" target="_blank"><img src="https://i.imgur.com/PPv8ymH.jpg" 
alt="IMAGE ALT TEXT HERE" /></a>

#### Cubes
<a href="http://www.youtube.com/watch?feature=player_embedded&v=i3Xt1w0KOP0
" target="_blank"><img src="https://i.imgur.com/jQVq5HA.jpg" 
alt="IMAGE ALT TEXT HERE" /></a>

#### GCtronic logo
<a href="http://www.youtube.com/watch?feature=player_embedded&v=iaR-Tps1SmQ
" target="_blank"><img src="https://i.imgur.com/MM49DA3.jpg" 
alt="IMAGE ALT TEXT HERE" /></a>
