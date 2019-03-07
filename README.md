# timelapse-ESP32-CAM

please subscribe to my youtube channel to support and to recieve more updates.
   https://www.youtube.com/user/v12345vtm  please subscribe if this code helped you
   https://www.paypal.me/v12345vtm/3 if you the video/code helped you , buy me a coffee :)
   this arduino sketch uses :
   -websockets to communicate to the webserver.
   -Jpg pictures can be stored into the SDcard on the board.
   -the webserver uses JSON format to parse the esp32-cam variables
   -toggle the small led on the pcb via webinterface ( socket)
   TO program this code to your ESP32-CAM use these settings :
  ESP32 Wrovermodule
  Huge app (3Mb no Ota)
  Qio , 80Mhz , 921600 .
  these lib need to be installed on your pc ( arduino ide )
  https://dl.espressif.com/dl/package_esp32_index.json
  and install ESP32 lib !!
  JSON version : ArduinoJson by benoit blanchon  v 5.13.4
SD card format  : FAT32 , up to 4GB
   this code is beta , and tested on ESP32-CAM with  OV2640 (sold with board) and is not tested with OV7670 cameras .
   ESP32-CAM board uses  the SD card to the following pins:
   SD Card | ESP32    |esp32-cam
      D2       -          -
      D3       SS         gpio13
      CMD      MOSI       gpio15
      VSS      GND        gnd
      VDD      3.3V       3.3v
      CLK      SCK        gpio14
      VSS      GND        gnd
      D0       MISO       gpio2
      D1       -          gpio4 + LED flash also  :(
  FLASHLED                gpio4
  led1                    gpio33 (mini smd ledje below ESP32-controler)
      SD card socket : pin 9 is SD ( = CARD DETECTION , is a card inserted ? )
 
  https://www.youtube.com/user/v12345vtm  please subscribe if this code helped you
  this code  = https://github.com/v12345vtm/timelapse-ESP32-CAM
 
