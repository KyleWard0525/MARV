#ifndef LCD_H
#define LCD_H
/**
 * This is an interface for the 1602A LCD Display with a PCF8754T
 * I2C backpack module
 * 
 * kward
 */
#include <LiquidCrystal_I2C.h>


class LCD {

  private:
    const uint8_t nRows = 2;                      //  Number of rows in LCD screen
    const uint8_t nCols = 16;                     //  Number of columns in LCD screen
    const uint8_t deviceAddr = 0x27;              //  Device address
    
    
  public:
    const uint8_t screenPadding = 1;              //  Columns of padding on both sides
    LiquidCrystal_I2C* screen;                    //  I2C Interface for communicating with LCD I2C module

    LCD(LiquidCrystal_I2C* lcdI2C_module)
    {
      //LiquidCrystal_I2C lcdI2C_module(deviceAddr,nCols,nRows);
      //lcdI2C_module.init();
      
      screen = lcdI2C_module;
      
      // Create i2c module objects     
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
      screen->noBlink();
      
      // Reset to initial state
      resetScreen();
    }

    //  Reset screen to default state
    void resetScreen()
    {
      // Clear screen and move cursor back to screenPadding,0
      screen->clear();

      // Set cursor after padding
      screen->setCursor(screenPadding,0);
    }

    //  Show message on screen for a specified time (-1 for infinite)
    void showMessage(String message, int dur, int cursorX, int cursorY)
    {
      on();
      screen->blink();
      screen->setCursor(cursorX, cursorY);
      screen->print(message.c_str());

      if(dur == -1)
      {
        return;
      }
      else {
        delay(dur);
        off();
      }
    }

    int getPadding()
    {
      return screenPadding;
    }
    
};




#endif  //  END LCD_H
