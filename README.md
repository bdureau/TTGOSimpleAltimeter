# TTGOSimpleAltimeter
Simple altimeter that will do what a Joly logic or Estes altimeter does
using the [LILYGO T-QT Pro](https://www.lilygo.cc/products/t-qt-pro?srsltid=AfmBOopF1SZ4JgGggyEN98mcQudv0hQCX8_p8glDDWhrxQgzO5AKA-nX) board

The altimeter looks like the following

<img src="/photos/ttgo-t-qt-simple-altimeter.jpg" width="35%">

# Building the code

You will need to download the Arduino ide from the [Arduino web site](https://www.arduino.cc/). 
Make sure that you install ESP32 support
The project depend on the following libraries
  - TFT
  - TFT_eSPI
  - Button2
  - uiwidgets
  
Compile with the following options:

<img src="/photos/LILYGo-T-QT.png" width="35%">

Prior to compiling go to the TFT_eSPI and open up the file User_Setup_Select.h

Comment out the following line
```
//#include <User_Setup.h> 
```
and uncomment the following line
```
#include <User_Setups/Setup211_LilyGo_T_QT_Pro_S3.h>
```

You will need to have the ESP32 board support version 2.0.14, anything higher than that may not work !!!

<img src="/photos/Esp32 board.png" width="55%">

# Contributing

If you want to contribute to the project just fork my project or send me some code. 

Report any issue or bug that you have

Suggestions and enhancement are welcome

The code is free for you to download and you do not need to buy anything from me. However it cost money to try out new boards, you need to buy them and fly them so if you want to financially help me you can donate via paypal

| Paypal | 
| ------ |
| [![](https://www.paypalobjects.com/en_US/i/btn/btn_donateCC_LG.gif)](https://www.paypal.com/paypalme/bearaltimeter) | 

# Disclaimer

I am not responsible for any damage that could happen. The code is provided as it is
