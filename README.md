# Introduction
This repository will contain the sensor related information for the training of the
WHS robotics electrical team.  A place for presentations and general info relating
to sensor use in FRC robots.

# Sensor Types
- Switch
  - Pressing the switch can indicate a physical condition
  - Uses:
    - Object detection: Game object touches switch when it is in a mechanism
    - Mechanism position: The mechanism presses on the switch when it is in the starting position - helpful during queuing and if the robot experiences a brownout during the match, which is why this is often referred to as a _limit switch_ (the mechanism is at the safe limit of travel)
  - Examples:
    - [REV Touch Sensor](https://www.revrobotics.com/rev-31-1425/?searchid=4661895)
    - [Limit Switch Long Straight Hinge Lever](https://www.amazon.com/dp/B09SWCJ8FF)
    - [Limit Switch Hinge Lever](https://www.amazon.com/dp/B0BDDLWL27)
    - [Limit Switch Hinge Roller Lever Arm](https://www.amazon.com/dp/B09SWBGPZK)
  - Connection Notes:
    - The switches have 3 connections:
      - C = Common - connected to the GND pin of a DIO
      - NC = Normally connected - typically connected to the signal pin of a DIO
      - NO = Normally open - optionally connected to the signal pin of a DIO in lieu of NC
      - If the switch isn't pressed, NC connects to C
      - If the switch is pressed, NO connects to C      
      - Typically, we connect NC and C so that the DIO input voltage is GND (0 V) and the switch reports False (GND) when not pressed, True when pressed
      - Alternatively, we connect NO and C so that the DIO input voltage is pulled up and the switch reports True when not pressed, False when pressed
    - [0.187" Female Quick Disconnect Spade Terminals](https://www.amazon.com/dp/B01N0M6PP1) makes for an easy and reliable connection to the switch's terminals

- Hall Effect
  - Sensor used to detect magnetic field presence / strength
    - Magnetic field may be an electromagnet or a permanent magnet
  - Works with Axial or Diametrical magnets
    - Alially magnetized: The poles of the magnet is on the flat ends
    - Diametrically magnetized: The poles of the magnet is on the curved sides
  - Uses:
    - Motor rotation: Hall effect sensors in NEO, NEO 550, or Vortex motors may be used to specify motor shaft movement (incremental detection)
    - Mechanism position: An axial magnet on a moving component may be sensed by a Hall effect sensor on an un-moving component (digital)
      - Examples:
        - [REV Magnetic Limit Switch](https://www.revrobotics.com/rev-31-1462/)
        - [HAMLIN 55100-3H-02-A HALL EFFECT MAGNETIC SENSOR](https://www.amazon.com/dp/B00MMYH5NC)
        - [ScoutMakes DRV5032 Digital Magnetic Hall Effect Sensor breakout](https://www.adafruit.com/product/6051)
    - Shaft speed or angle: A diametrical magnet is embedded in the end of a shaft and sensor is placed on unmoving support near the end of the moving shaft
      - Examples:
        - [SRX Mag Encoder](https://store.ctr-electronics.com/products/srx-mag-encoder)
        - [Adafruit AS5600 Magnetic Angle Sensor](https://www.adafruit.com/product/6357)
        - [AS5600 Magnetic Encoder](https://www.amazon.com/WWZMDiB-Measurement-Module%EF%BC%8CMainly-Information-Progressive/dp/B0CM3C8KFT)
    - Motor current: We don't use this on the robot, but our DMMs may have clamp on Hall effect sensors that allow us to measure current being provided to motors - this has been used in the past to ensure safe use of motors

- Force sensitive
  - Resistors whose resistance changes based on a force applied to the force-sensitive resistor
    - Requires additional electrical circuit (PCB) to interface to a RoboRIO
    - i.e. A resistor alone does not make a sensor
    - A minimum of a resistor divider that feeds into an analog input
    - But the resistor divider output may be buffered by an opamp and used in an analog input
    - Or the resistor divider output may be fed to an analog comparator, converting it to a digital output and used in a digital input (the comparator tuned to a specific resistor divider voltage representing a specific applied force)
  - Uses:
    - Object detection: Can detect squeezing of a game object, but not a brush past
  - Examples:
    - [Disk](https://www.amazon.com/Pressure-Sensitivity-Sensitive-Industrial-Measurement/dp/B0CZ6L5NMM)
    - [Square](https://www.amazon.com/RP-S40-ST-Pressure-20g-10kg-Accuracy-Intelligent/dp/B07FCLV5BJ)
    - [Strip](https://www.amazon.com/Pressure-ZD10-100-Resistance-Type-Resistor-Sensitive/dp/B07MHTWR1C)

- Rotational
  - Typicaly this sensor links to a drive shaft
  - There are two types: Absolute & Incremental
  - Absolute:
    - Absolute provides the absolute angle of sensor shaft
    - Absolute generally has a limit to the rotational range
    - Uses:
      - Angle sensing: Swerve pod steering
      - Angle sensing: Arm angle detection
    - Examples:
      - Potentiometer connected in voltage divider configuration
      - [USDigital MA3](https://www.usdigital.com/products/encoders/absolute/shaft/ma3/)

  - Incremental:
    - Incremental provides an indication of a small change in shaft angle (delta theta)
    - Incremental requires hardware (in RoboRIO) to accumulate changes to determine the absolute angle
    - Time between incremental indications provides instantaneous rotational velocity
    - Uses:
      - Shaft speed: A flywheel shooter's rotational speed
      - Wheel speed: Rotational speed of swerve pod wheel
    - Examples:
      - [Grayhill 63R series](https://www.grayhill.com/wp-content/uploads/media/63r-datasheet.pdf)
      - [USDigital S4T](https://www.usdigital.com/products/encoders/incremental/shaft/s4t/)

- Distance
  - Ultra Sonic
    - Sound waves are emitted, bounce off object, and the reflection is detected
    - Distance is based on speed of sound in air and time between emission and reflection
    - Examples:
      - [Maxbotix LV-MaxSonar-EZ series](https://maxbotix.com/collections/lv-maxsonar-ez-products)
  - Infrared
    - Examples:
      - [Sharp](https://global.sharp/products/device/lineup/selection/opto/haca/diagram.html)
  - Time of Flight
    - Examples:
      - [VL53L0X (30 mm to 1000 mm)](https://www.adafruit.com/product/3317)
      - [VL53L4CD (1 mm to 1300 mm)](https://www.adafruit.com/product/5396)
      - [VL53L1X (30 mm to 4000 mm)](https://www.adafruit.com/product/3967)
      - [VL53L4CX (1 mm to 6000 mm)](https://www.adafruit.com/product/5425)
      - [TFmini-S (0.1 m to 12 m)](https://www.amazon.com/Benewake-TFmini-S-Single-Point-Raspberry-Interface/dp/B08D1XVRV5)
  - Lidar
    - Light is emitted, bounces off object, and the reflection is detected
    - Distance is based on speed of light and time between emission and reflection
    - The light source and frequency is the general difference between infrared, ToF, and Lidar sensors
    - Examples:
      - [Garmin Lidar Lite v3](https://cdn.sparkfun.com/assets/f/e/6/3/7/PM-14032.pdf)
      - [Garmin Lidar Lite v4](https://www.sparkfun.com/garmin-lidar-lite-v4-led-distance-measurement-sensor-qwiic.html)
  - Uses:
    - Distance to external object (field) or internal object (game object)
    - Distance to field object may be used for aligning before placement (e.g. reef in 2025)
    - Distance to internal object can be used for adjusting delivery when intake is wide (e.g. 2023)

- Presence
  - Game object detection without knowing distance to the object
  - Beam breaks use an emitter and detector on opposite sides of some mechanism to detect an object between
    - The distance may be limited by the emitter's power or detector's sensitivity
    - Accuracy in positioning the emitter/detector pair is critical
    - Examples:
      - [Adafruit IR Beam Break - 5 mm LEDs](https://www.adafruit.com/product/2168)
      - [Adafruit IR Beam Break - 3 mm LEDs](https://www.adafruit.com/product/2167)
  - Reflectance sensors also use an emitter and detector pair, but are in the same sensor enclosure
    - Light from emitter bounces off object and returns to detector (the light reflects rather than breaks)
    - Examples:
        - [E18-D80NK (Short Style)](https://www.amazon.com/XOIIIQND-avoidance-E18-D80NK-reflection-photoelectric/dp/B0D83MR3QG)
        - Note this provides a digital interface with non-standard color wires (black=signal, brown=5V, blue=GND)

- Color
  - Detects the color of light reflecting off an object
  - Can be used to determine a game object's color
  - Light source may want to to be RGB so that reflected color of specific source color (a quick sequence)
  - Examples:
    - [REV Color Sensor V3](https://www.revrobotics.com/rev-31-1557/?searchid=4661894)
    - [Adafruit AS7341 breakout](https://www.adafruit.com/product/4698)
    - [Adafruit OPT4048 breakout](https://www.adafruit.com/product/6335)

- Vision
  - Object detection
    - See game object, use vision to drive to object and intake
  - Field position by detecting April tags
    - Supplements robot position detection with vision (i.e. IMU + drive encoders + vision)
  - Examples:
    - [Limelight 4 camera](https://limelightvision.io/products/limelight-4)
      - With optional [Hailo-8 module](https://limelightvision.io/products/hailo-8-upgrade-kit-for-limelight-4) neural processor for AI that provides 26 TOPS 
    - [OpenMV N6](https://openmv.io/collections/all-products/products/openmv-n6) a low-cost Python programmable camera with 120 fps global shutter, a Cortex-M55 CPU, and neural processing unit
    - [Arducam ToF Camera](https://www.amazon.com/Arducam-Rolling-Shutter-Raspberry-Solution/dp/B0BRB12W7Y)provides a 3D camera used with a Raspberry Pi or Nvidia Jetson

- Accelerometer
  - Senses the force applied, including gravity
  - Can be 1, 2, or 3 dimensional (e.g. x, (x,y), or (x,y,z))
  - Uses:
    - Speed and distance: Integration of acceleration (v) or double integration (x) of the force acting on the robot (but removing gravity)
    - Angle sensing: Use the force of gravity to determine accelerometer's orientation and therefore the angle of an arm
  - Examples:
    - [CTRE Pigeon 2.0 IMU](https://store.ctr-electronics.com/products/pigeon-2?_pos=1&_sid=432d1aba2&_ss=r) as part of the IMU function
    - [NavX2 MXP IMU](https://andymark.com/products/navx2-mxp-robotics-navigation-sensor) as part of the IMU function
    - [Analog Devices ADIS 16470 IMU](https://www.analog.com/media/en/technical-documentation/data-sheets/ADIS16470.pdf) acquired from FIRST Choice in an FRC-specific module that plugs into the RoboRIO's SPI port
    - [Adafruit ADXL345 - Triple-Axis Accelerometer breakout](https://www.adafruit.com/product/1231)
    - [Adafruit ADXL375 - High G Accelerometer (+-200g) breakout](https://www.adafruit.com/product/5374)

- Gyroscope
  - Senses rotational movement
  - Reports rotational speed
  - Can be 1, 2, or 3 dimensional
  - Uses:
    - Angle sensing: Integration of rotational speed (theta)
  - Examples:
    - Refer to the Pigeon 2.0, NavX2 IMUs above
    - [Adafruit TDK InvenSense ICM-20948 9-DoF IMU breakout](https://www.adafruit.com/product/4554)
    - [Adafruit ISM330DHCX - 6 DoF IMU breakout](https://www.adafruit.com/product/4502)

# Some Sensor Tricks
- Four-phase / Quadrature encoding
  - A four-phase encoded sensor provides two signals (a/b) that provide four states
    - States are: 0/0, 0/1, 1/0. and 1/1
  - Using these states with Gray Code
    - Gray code is a binary numeral system in which two successive values differ in only one bit, also known as a unit distance code
    - i.e. Signals a/b only changes a or b from one state to another (i.e. 0/0 can change to 0/1 or 1/0 but not 1/1 which would be a change of two bits)
  - Gray code sequences
    - 0/0, 0/1, 1/1, 1/0, 0/0 is the up count sequence
    - 0/0, 1/0, 1/1, 0/1, 0/0 is the down count sequence
  - Uses:
    - Used for incremental encoder counting (up/down) in motors (hall effect sensors)
    - Beam Break Counter: A pair of beam breaks spaced so that an object passing through it generates a four-phase sequence
  - Tip:
    - Using CircuitPython rotaryio module IncrementalEncoder class provides easy counting

# Sensor Fusion
- Some times we want to combine sensor functions
  - e.g. Using beam breaks so that we can both count game objects passing through a robot mechanism, but also trigger a light sensor when at the mid-point between beam breaks when color sensing would be most accurate
