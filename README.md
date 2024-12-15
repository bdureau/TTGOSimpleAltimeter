# TTGOSimpleAltimeter
Simple altimeter that will do what a Joly logic or Estes altimeter does
using the [LILYGO T-QT Pro](https://www.lilygo.cc/products/t-qt-pro?srsltid=AfmBOopF1SZ4JgGggyEN98mcQudv0hQCX8_p8glDDWhrxQgzO5AKA-nX) board

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

//#include <User_Setup.h> 

and uncomment the following line

#include <User_Setups/Setup211_LilyGo_T_QT_Pro_S3.h>
