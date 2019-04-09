#ifndef ROTARY_ENCODER
#define ROTARY_ENCODER
#include <Arduino.h> 

/**
 * You can track the knobs movements and detect clicks of the knob as well.
 * For it to work, you need to call \p update() frequently.  The easiest way to
 * do this is to call it at the top of your \p loop() function and not use \p
 * delay() in your code.
 */
//template<int id>
class RotaryEncoder {
	int pinA, pinB, currentPos, maxPos, pinButton;
	uint8_t enc_prev_pos;
	uint8_t enc_flags;

public:

	/** \brief Constructor 
	 *
	 * The parameter identifies the pin the encoder is connected to.
	 *
	 * If you use the sketch that came with your robot, you won't need call this.
	 */
	RotaryEncoder(int pin_a, int pin_b ) : pinA(pin_a), pinB(pin_b)
		{}

	/**
	 * \brief Setup the breakout.
	 *
	 * Call this function once in your setup() function. 
	 */
	void setup()
		{
			pinMode(pinA, INPUT_PULLUP);
			pinMode(pinB, INPUT_PULLUP);
			currentPos = 0;
			enc_prev_pos = 0;
			enc_flags = 0;

			// get an initial reading on the encoder pins
			if (digitalRead(pinA) == LOW) {
				enc_prev_pos |= (1 << 0);
			}
			if (digitalRead(pinB) == LOW) {
				enc_prev_pos |= (1 << 1);
			}
		}
	
	/**
	 * \brief Update the encoder
	 *
	 * This function 
	 * must be called within the program's main update loop. 
	 * Otherwise, the RotaryEncoder's internal values won't update. 
	 * Additionally, frequent calls to delay may affect this function's ability
	 * to update responsively.
	 */	
	void update()
		{
			int8_t enc_action = 0; // 1 or -1 if moved, sign is direction
			// http://www.arduino.cc/en/Reference/PortManipulation
			uint8_t enc_cur_pos = 0;
			// read in the encoder state first
			if (digitalRead(pinA)) {
				enc_cur_pos |= (1 << 0);
			}
			if (digitalRead(pinB)) {
				enc_cur_pos |= (1 << 1);
			}
 
			// if any rotation at all
			if (enc_cur_pos != enc_prev_pos)
			{
				if (enc_prev_pos == 0x00)
				{
					// this is the first edge
					if (enc_cur_pos == 0x01) {
						enc_flags |= (1 << 0);
					}
					else if (enc_cur_pos == 0x02) {
						enc_flags |= (1 << 1);
					}
				}
 
				if (enc_cur_pos == 0x03)
				{
					// this is when the encoder is in the middle of a "step"
					enc_flags |= (1 << 4);
				}
				else if (enc_cur_pos == 0x00)
				{
					// this is the final edge
					if (enc_prev_pos == 0x02) {
						enc_flags |= (1 << 2);
					}
					else if (enc_prev_pos == 0x01) {
						enc_flags |= (1 << 3);
					}
 
					// check the first and last edge
					// or maybe one edge is missing, if missing then require the middle state
					// this will reject bounces and false movements
					if (bit_is_set(enc_flags, 0) && (bit_is_set(enc_flags, 2) || bit_is_set(enc_flags, 4))) {
						enc_action = 1;
					}
					else if (bit_is_set(enc_flags, 2) && (bit_is_set(enc_flags, 0) || bit_is_set(enc_flags, 4))) {
						enc_action = 1;
					}
					else if (bit_is_set(enc_flags, 1) && (bit_is_set(enc_flags, 3) || bit_is_set(enc_flags, 4))) {
						enc_action = -1;
					}
					else if (bit_is_set(enc_flags, 3) && (bit_is_set(enc_flags, 1) || bit_is_set(enc_flags, 4))) {
						enc_action = -1;
					}
 
					enc_flags = 0; // reset for next time
				}
			}
 
			enc_prev_pos = enc_cur_pos;
 
			currentPos += enc_action;
		}
	

	/**
	 * \brief Get the current value
	 *
	 * Returns the current position of the rotary encoder.  It will be between 0 and the value set by \p setMaxPos() (default = 24).
	 */
	int getCurrentPos() {
		return currentPos;
	}

	/**
	 * \brief Set the current value
	 *
	 */
	int setCurrentPos(int v) {
		currentPos = v;
	}

};

#endif
