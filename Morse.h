/**
 * An attempt at creating a morse code library for MARV to communicate with
 * 
 * kward
 */

class Morse {
  private:
    int buzzer;                     //  Buzzer pin
    int beepFreq;                   //  Beep frequency 
    int dotDuration;                //  Dot beep duration
    int dashDuration;               //  Dash beep duration
    int letterSpaceDuration;        //  Duration of time between letters

  public:

    //  Main constructor
    Morse(int buzzerPin)
    {
      buzzer = buzzerPin;
      beepFreq = 470;
      dotDuration = 250;
      dashDuration = 350;
      letterSpaceDuration = 350;
    }

    //  Play dot sound
    void dot()
    {
      // Loop for dot duration
      for(int i = 0; i < dotDuration; i++)
      {
        beep(beepFreq, buzzer);
        delayMicroseconds(100);
      }

      delay(50);
    }

    //  Play dash sound
    void dash()
    {
      // Loop for dot duration
      for(int i = 0; i < dashDuration; i++)
      {
        beep(beepFreq, buzzer);
        delayMicroseconds(100);
      }
      delay(50);
    }

    // Play 'a' sound
    void a()
    {
      dot();
      dash();
      delay(letterSpaceDuration);
    }

    // Play 'b' sound
    void b()
    {
      dash();
      dot();dot();dot();
      delay(letterSpaceDuration);
    }

    // Play 'c' sound
    void c()
    {
      dash();
      dot();
      dash();
      dot();
      delay(letterSpaceDuration);
    }

};
