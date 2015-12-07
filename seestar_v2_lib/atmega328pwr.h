// acknowledgement to JeeLibs Ports 2009-02-13 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#ifndef ATMEGA328PWR_h
#define ATMEGA328PWR_h

/// @file
/// Ports library definitions.

/// @file
/// atmega328pwr library definitions.

#if ARDUINO >= 100
#include <Arduino.h> // Arduino 1.0
#else
#include <WProgram.h> // Arduino 0022
#endif

#include <stdint.h>
#include <avr/pgmspace.h>
//#include <util/delay.h>

// tweak this to switch ATtiny84 etc to new Arduino 1.0+ conventions
// see http://arduino.cc/forum/index.php/topic,51984.msg371307.html#msg371307
// and http://forum.jeelabs.net/node/1567
#if ARDUINO >= 100
#define WRITE_RESULT size_t
#else
#define WRITE_RESULT void
#endif


/// The millisecond timer can be used for timeouts up to 60000 milliseconds.
/// Setting the timeout to zero disables the timer.
///
/// * for periodic use, poll the timer object with "if (timer.poll(123)) ..."
/// * for one-shot use, call "timer.set(123)" and poll as "if (timer.poll())"

class MilliTimer {
    word next;
    byte armed;
public:
    MilliTimer () : armed (0) {}

    /// poll until the timer fires
    /// @param ms Periodic repeat rate of the time, omit for a one-shot timer.
    byte poll(word ms =0);
    /// Return the number of milliseconds before the timer will fire
    word remaining() const;
    /// Returns true if the timer is not armed
    byte idle() const { return !armed; }
    /// set the one-shot timeout value
    /// @param ms Timeout value. Timer stops once the timer has fired.
    void set(word ms);
};

/// Low-power utility code using the Watchdog Timer (WDT). Requires a WDT
/// interrupt handler, e.g. EMPTY_INTERRUPT(WDT_vect);
class SleepDog {
public:
    /// start the watchdog timer (or disable it if mode < 0)
    /// @param mode Enable watchdog trigger after "16 << mode" milliseconds
    ///             (mode 0..9), or disable it (mode < 0).
    /// @note If you use this function, you MUST included a definition of a WDT
    /// interrupt handler in your code. The simplest is to include this line:
    ///
    ///     ISR(WDT_vect) { SleepDog::watchdogEvent(); }
    ///
    /// This will get called when the watchdog fires.
    static void watchdogInterrupts (char mode);

    /// enter low-power mode, wake up with watchdog, INT0/1, or pin-change
    static void powerDown ();

    /// Spend some time in low-power mode, the timing is only approximate.
    /// @param msecs Number of milliseconds to sleep, in range 0..65535.
    /// @returns 1 if all went normally, or 0 if some other interrupt occurred
    /// @note If you use this function, you MUST included a definition of a WDT
    /// interrupt handler in your code. The simplest is to include this line:
    ///
    ///     ISR(WDT_vect) { SleepDog::watchdogEvent(); }
    ///
    /// This will get called when the watchdog fires.
    static byte delay_msec (word msecs);

    /// This must be called from your watchdog interrupt code.
    static void watchdogEvent();
};



#endif // #ifndef ATMEGA328PWR_h