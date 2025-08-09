#pragma once
#include "main.h"

extern dace::Drive chassis;
extern dace::Odometry trackers;
// Controller
// The master controller object is used to receive input from the user via joysticks and buttons.
extern pros::Controller master;

/*
Util Header File
*/

namespace dace{

    void odomStandards();  // links odometry to chassis and sets exit conditions
    void pidStandards();   // sets exit conditions without odometry

    class util{

        public:
            
        private:

    };

    constexpr std::uint8_t adiLetterToNum(char p) {
        return static_cast<std::uint8_t>(p - 'A' + 1);
    }

    // Single-acting pneumatic piston
    class Piston {
    public:
      explicit Piston(std::uint8_t port, bool default_state = false)
      : out(port), state_(default_state) { out.set_value(state_); }

      explicit Piston(char port, bool default_state = false)
      : out(adiLetterToNum(port)), state_(default_state) { out.set_value(state_); }

      void set(bool s)        { state_ = s; out.set_value(state_); }
      void toggle()           { set(!state_); }
      bool get() const        { return state_; }

      // For use with controller new-press booleans
      void button_toggle(bool pressed) { if (pressed) toggle(); }

    private:
      pros::ADIDigitalOut out;
      bool state_;
    };

}//namespace dace