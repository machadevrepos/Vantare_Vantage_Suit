/*
 * RGB_LED.h
 *
 *  Created on: Nov 26, 2024
 *      Author: dhanv
 */

#ifndef RGB_LED_H_
#define RGB_LED_H_

class RGB_LED {

		bool invert = 0;
		DIG_PIN LR;
		DIG_PIN LG;
		DIG_PIN LB;
		public:

		RGB_LED(DIG_PIN LR, DIG_PIN LG, DIG_PIN LB, bool invert = 0) :
				invert(invert), LR(LR), LG(LG), LB(LB) {
//			SET_PWRPIN(PWRPIN_GPI_Ox, PWRPIN_GPIO_PI_Nx);
			SET(0, 0, 0);
		}
		RGB_LED(GPIO_TypeDef *LR_GPIOx, uint16_t LR_GPIO_PINx, GPIO_TypeDef *LG_GPIOx, uint16_t LG_GPIO_PINx, GPIO_TypeDef *LB_GPIOx, uint16_t LB_GPIO_PINx, bool invert_d = 0,bool invert = 0) :
				invert(invert), LR(LR_GPIOx, LR_GPIO_PINx, invert_d), LG(LG_GPIOx, LG_GPIO_PINx, invert_d), LB(LB_GPIOx, LB_GPIO_PINx, invert_d) {
			SET(0, 0, 0);
		}

		void Toggle(bool red, bool green, bool blue, uint32_t del = 0) {

			green ? LG.Toggle(del) : LG.SET(0);
			red ? LR.Toggle(del) : LR.SET(0);
			blue ? LB.Toggle(del) : LB.SET(0);
//			if (red)
//				LR.Toggle(del);
//			else
//				LR.SET(0);
//			if (blue)
//				LB.Toggle(del);
//			else
//				LB.SET(0);
			if (del > 0)
				delay_ms(del);

		}
		void SET(bool red, bool green, bool blue, uint32_t del = 0) {
			if (invert) {
				red = !red;
				green = !green;
				blue = !blue;
			}
//			debug.Print("R:" + d_t_s(red) + " G:" + d_t_s(green) + " B:" + d_t_s(blue) + "\r\n");
			LR.SET(red);
			LG.SET(green);
			LB.SET(blue);
			if (del > 0)
				delay_ms(del);

		}
		void OFF() {
			SET(0, 0, 0);
		}
		void ON() {
			SET(1, 1, 1);
		}

		void PULSE(bool red, bool green, bool blue, uint32_t del1 = 0, uint32_t del2 = 0) {
			if (invert) {
				red = !red;
				green = !green;
				blue = !blue;
			}
			SET(red, green, blue, del1);
			SET(0, 0, 0, del2);
		}

		void TEST() {
			for (int i = 0; i < 8; i++) {
				SET((i >> 0) & 0x1, (i >> 1) & 0x1, (i >> 2) & 0x1, 5 * us_s);
			}

		}

};

#endif /* RGB_LED_H_ */
