/*
 * FUNCTIONS.h
 *
 *  Created on: Nov 27, 2024
 *      Author: dhanv
 */

#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_


int c_RNG(uint8_t *dest, unsigned size) {
	// Use the least-significant bits from the ADC for an unconnected pin (or connected to a source of
	// random noise). This can take a long time to generate random data if the result of analogRead(0)
	// doesn't change very frequently.
	while (size) {
		uint8_t val = 0;
		for (unsigned i = 0; i < 8; ++i) {
			int init = rand();
			int count = 0;
			while (rand() == init) {
				srand(HAL_GetTick());
				++count;
			}

			if (count == 0) {
				val = (val << 1) | (init & 0x01);
			} else {
				val = (val << 1) | (count & 0x01);
			}
		}
		*dest = val;
		++dest;
		--size;
	}
	// NOTE: it would be a good idea to hash the resulting random data using SHA-256 or similar.
	return 1;
}






#endif /* FUNCTIONS_H_ */
