## Sneedville_Milli-microcontroller  

  Program for the Ruggeduino in the system-monitoring design originally developed for the Cedar Grove Baptist Church clean water
  project in Sneedville. The code and other relevant files will be updated and maintained as part of a senior design project for
  the 2020 spring semester at the following GitHub repository:  
  <br>
  https://github.com/jjgiesey/Sneedville_Milli-microcontroller  
  <br>
  The generic version of the project that lacks the freeze protection features that are specialized to the Sneedville
  implementation is maintained in a different GitHub repository:  
  <br>
  https://github.com/Milligan-Engineering/Milli-microcontroller 
  <br>
  Should the project require maintenance/changes/improvements/additions in the future, this repository should serve as a
  reference for other contributors.  
  <br>

*********************************************************************************************************************************
###### Ruggeduino - modifications & testing
*********************************************************************************************************************************

  Arduino Software IDE can be used to verify, compile, and upload the code to an Arduino or Ruggeduino. Find it here:  
  <br>
  https://www.arduino.cc/en/Main/Software  
  <br>
  A Ruggeduino is essentially an Arduino that has been ruggedized to tolerate temperatures, voltages, and currents well above
  what would destroy a normal Arduino. Ruggeduinos are much better suited for industrial-like projects with continuous long-term
  operation and harsh environmental conditions compared to normal Arduinos, which are intended for lightweight home and
  laboratory projects. When used with the Arduino Software IDE, Ruggeduinos are designed to look and act like their normal
  Arduino counterparts, so changes to an already-implemented project can be tested and tweaked on a normal Arduino. Code for the
  Ruggeduino-SE ST used in this project is directly compatible with the Arduino UNO boards used in the labs for Computer
  Engineering II at Milligan. One caveat--a USB driver will need to be installed on a computer the first time a Ruggeduino-SE is
  connected to it, which can be obtained here:  
  <br>
  https://www.silabs.com/products/development-tools/software/usb-to-uart-bridge-vcp-drivers  
  <br>

*********************************************************************************************************************************
###### Repository structure
*********************************************************************************************************************************

  - All code is in the C programming language and is contained within the single file 'SneedvilleMilli-microcontroller.ino',
    located in thesame directory as this README  
  - 'electrical_box_layout.png' is the electrical box layout and can be updated with the CAD file 'electrical_box_layout.dwg'  
  - 'electrical-overview.png' is a schematic that gives an overview of all electrical connections for the Cedar Grove project  
  - The 'electrical-overview_orcad_project' folder contains an OrCAD Capture project that can be used to update the schematic  
  - 'UV-C_sensor_circuit.png' identifies the pin numbers and shows the surface-mount component layout for the Adafruit 1212  
  - 'UV-C_sensor_circuit_schematic.png' shows the external connections and suface-mount connections for the Adafruit 1212  
  <br>

*********************************************************************************************************************************
###### Parts referenced in the electrical overview schematic
*********************************************************************************************************************************

  - Ruggeduino-SE ST microcontroller
      - https://www.rugged-circuits.com/microcontroller-boards/ruggeduino-se-special-edition-mpdes
  - LPMNSD DZ47-63 C10 circuit breaker
      - http://www.lpmnsd.com/en/Product/product_show.asp?did=1556
  - Mean Well MDR-20-24 power supply [+24 VDC]
      - https://www.meanwell-web.com/en-gb/ac-dc-industrial-din-rail-power-supply-output-mdr--20--24
  - Mean Well MDR-20-5 power supply [-5 VDC]
      - https://www.meanwell-web.com/en-gb/ac-dc-industrial-din-rail-power-supply-output-5vdc-mdr--20--5
  - PurTest PT-12 ultraviolet water disinfection system
      - https://purtest.com/wp-content/uploads/sites/7409/2018/05/PurTest-PT-Series_Service-Manual-November-2-2016.pdf
  - BACOENG 2W-025-08 solenoid valve
      - https://bacoeng.com/collections/solenoid-valve/products/ss-brass-dc12v-24v-ac110v-normally-closed-solenoid-valve
  - CZH-Labs MD-D1304T/3-1 three-channel relay module
      - https://czh-labs.com/czh-labs-din-rail-mount-3-channel-12-amp-solid-state-relay-ssr-module-in-432vdc-out-100240vac-p1245.html
  - Adafruit 1212 surface-mount breakout board
      - https://www.adafruit.com/product/1212
  - De'Longhi TRN0812T electric oil filled radiator
      - https://www.delonghi.com/en-us/products/comfort/portable-heating/radiant-heaters/bambino-trn0812t-0105086103
  - Seeed 114991178 pressure transmitter
      - https://www.seeedstudio.com/Water-Pressure-Sensor-G1-4-1-2MPa-p-2887.html
  - Onset HOBO UX120-006M analog data logger
      - https://www.onsetcomp.com/products/data-loggers/ux120-006m
  - Onset CABLE-ADAP5 voltage input cable
      - https://www.onsetcomp.com/products/sensors/cable-dcvolt
  - Analog Devices TMP36GT9Z temperature sensor
      - https://www.analog.com/en/products/tmp36.html
  - Dwyer 629HLP differential pressure transmitter
      - https://www.dwyer-inst.com/Product/Pressure/DifferentialPressure/Transmitters/Series629HLP
  - Dwyer WPT-A-C-04 water meter
      - https://www.dwyer-inst.com/Product/Flow/WaterMeters/SeriesWPT
  - Onset HOBO UX120-017 pulse data logger
      - https://www.onsetcomp.com/products/data-loggers/ux120-017