#ifndef BUMPERS_H
#define BUMPERS_H

/**
 * Header/src file for operations and actions using the connected
 * bump sensors
 * 
 * kward
 */

class Bumpers {
  private:
    int collisionFreq;
  
  public:
    
    //  Main constructor
    Bumpers() 
    {
      // Set status led
      setStatusLed(1);
      
      // Set buzzer frequency for collision alert
      collisionFreq = 240;
    }

    //  Check if one of the bumpers has been hit
    bool checkForCollision()
    {
      // Bumper impact
      if(digitalRead(BP_SW_PIN_0) == LOW || digitalRead(BP_SW_PIN_1) == LOW || digitalRead(BP_SW_PIN_2) == LOW ||
        digitalRead(BP_SW_PIN_3) == LOW || digitalRead(BP_SW_PIN_4) == LOW || digitalRead(BP_SW_PIN_5) == LOW) {
      
         return 1;
        }
      return 0;
    }

    /**
    * Overload function for checkForCollision() that returns the bumper collided with
    */
    

    //  Trigger bump alert using LED and buzzer
    void alert(int buzzer)
    {
      // Set status led
      setStatusLed(0);
      
      // Play beep
      beep(collisionFreq, buzzer);
    }

    //  Set LED status. 1 = OK, 0 = Collision
    void setStatusLed(int stat)
    {
      if(stat)
      {
        // Set bumper status LED to green 
        digitalWrite(GREEN_LED, 1);
        digitalWrite(RED_LED, 0);
      }
      else {
        // Set status led to red
        digitalWrite(GREEN_LED, 0);
        digitalWrite(RED_LED, 1);
      }
    }
};


#endif
