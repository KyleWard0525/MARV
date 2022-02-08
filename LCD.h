#ifndef LCD_H
#define LCD_H
/**
 * This is an interface for the 1602A LCD Display with a PCF8754T
 * I2C backpack module
 * 
 * kward
 */
#include "LiquidCrystal_I2C.h"


class LCD {

  private:
    const uint8_t nRows = 2;                      //  Number of rows in LCD screen
    const uint8_t nCols = 16;                     //  Number of columns in LCD screen
    const uint8_t deviceAddr = 0x27;              //  Device address
    const uint8_t screenPadding = 1;              //  Number of columns of padding on either side
    
  public:
    LiquidCrystal_I2C* screen;                     //  I2C Interface for communicating with LCD I2C module

    LCD(LiquidCrystal_I2C* i2cModule)
    {
      // Create i2c module objects     
      screen = i2cModule;
      screen->init();
      screen->noBlink();
    }


    //  Turn on LCD and set to default state
    void on()
    {
      // Turn on the backlight, display, and cursor
      screen->backlight();
      screen->display();
      screen->cursor();

      // Reset to initial state
      resetScreen();
    }

    //  Turn off LCD display
    void off()
    {
      // Turn off the backlight, display, and cursor
      screen->noBacklight();
      screen->noDisplay();
      screen->noCursor();
      
      
      // Reset to initial state
      resetScreen();
    }

    //  Reset screen to default state
    void resetScreen()
    {
      // Clear screen and move cursor back to 0,0
      screen->clear();
      screen->home();
    }

    
};




#endif  //  END LCD_H
