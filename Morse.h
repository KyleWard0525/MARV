#ifndef MORSE_H
#define MORSE_H

/**
 * An attempt at creating a morse code library for MARV to communicate with
 * 
 * kward
 */

class Morse {
  private:
    int buzzer;                     //  Buzzer pin
    int commLed;                    //  Visual communication LED pin
    int beepFreq;                   //  Beep frequency 
    int dotDuration;                //  Dot beep duration
    int dashDuration;               //  Dash beep duration
    int letterSpaceDuration;        //  Duration of time between letters

  public:

    //  Main constructor
    Morse(int buzzerPin, int ledPin)
    {
      // Set pins
      buzzer = buzzerPin;
      commLed = ledPin;

      // Set sound variables
      beepFreq = 300;
      dotDuration = 250;
      dashDuration = 350;
      letterSpaceDuration = 350;
    }

    //  Play dot sound
    void dot()
    {
      digitalWrite(commLed, HIGH);
      // Loop for dot duration
      for(int i = 0; i < dotDuration; i++)
      {
        beep(beepFreq, buzzer);
        delayMicroseconds(50);
      }
      digitalWrite(commLed, LOW);
      delay(50);
      
    }

    //  Play dash sound
    void dash()
    {
      digitalWrite(commLed, HIGH);
      // Loop for dot duration
      for(int i = 0; i < dashDuration; i++)
      {
        beep(beepFreq, buzzer);
        delayMicroseconds(50);
      }
      digitalWrite(commLed, LOW);
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

    // Play 'd' sound
    void d()
    {
      dash();
      dot();
      dot();
      delay(letterSpaceDuration);
    }

    // Play 'e' sound
    void e()
    {
      dot();
      delay(letterSpaceDuration);
    }

    // Play 'f' sound
    void f()
    {
      dot();
      dot();
      dash();
      dot();
      delay(letterSpaceDuration);
    }

    // Play 'g' sound
    void g()
    {
      dash();
      dash();
      dot();
      delay(letterSpaceDuration);
    }

    // Play 'h' sound
    void h()
    {
      dot();
      dot();
      dot();
      dot();
      delay(letterSpaceDuration);
    }

    // Play 'i' sound
    void i()
    {
      dot();
      dot();
      delay(letterSpaceDuration);
    }    

    // Play 'j' sound
    void j()
    {
      dot();
      dash();dash();dash();
      delay(letterSpaceDuration);
    }

    // Play 'k' sound
    void k()
    {
      dash();
      dot();
      dash();
      delay(letterSpaceDuration);
    }

    // Play 'l' sound
    void l()
    {
      dot();
      dash();
      dot();
      dot();
      delay(letterSpaceDuration);
    }

    // Play 'm' sound
    void m()
    {
      dash();
      dash();
      delay(letterSpaceDuration);
    }

    // Play 'n' sound
    void n()
    {
      dash();
      dot();
      delay(letterSpaceDuration);
    }

    // Play 'o' sound
    void o()
    {
      dash();
      dash();
      dash();
      delay(letterSpaceDuration);
    }

    // Play 'p' sound
    void p()
    {
      dot();
      dash();
      dash();
      dot();
      delay(letterSpaceDuration);
    }

    // Play 'q' sound
    void q()
    {
      dash();
      dash();
      dot();
      dash();
      delay(letterSpaceDuration);
    }

    // Play 'r' sound
    void r()
    {
      dot();
      dash();
      dot();
      delay(letterSpaceDuration);
    }

    // Play 's' sound
    void s()
    {
      dot();
      dot();
      dot();
      delay(letterSpaceDuration);
    }

    // Play 't' sound
    void t()
    {
      dash();
      delay(letterSpaceDuration);
    }

    // Play 'u' sound
    void u()
    {
      dot();
      dot();
      dash();
      delay(letterSpaceDuration);
    }

    // Play 'v' sound
    void v()
    {
      dot();
      dot();
      dot();
      dash();
      delay(letterSpaceDuration);
    }

    // Play 'w' sound
    void w()
    {
      dot();
      dash();
      dash();
      delay(letterSpaceDuration);
    }

    // Play 'x' sound
    void x()
    {
      dash();
      dot();
      dot();
      dash();
      delay(letterSpaceDuration);
    }

    // Play 'y' sound
    void y()
    {
      dash();
      dot();
      dash();
      dash();
      delay(letterSpaceDuration);
    }

    // Play 'z' sound
    void z()
    {
      dash();
      dash();
      dot();
      dot();
      delay(letterSpaceDuration);
    }
};


#endif      //  End MORSE_H
