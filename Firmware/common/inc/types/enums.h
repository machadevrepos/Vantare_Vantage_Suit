/*
 * ENUMS.h
 *
 *  Created on: Aug 12, 2023
 *      Author: dhanveer.singh
 */

#ifndef ENUMS_H_
#define ENUMS_H_

enum LOOP_CONT {
	$BREAK, $REPEAT, $CONTINUE
};
/**
 * @enum NEOWAY_RETURN
 * @brief To tell about response recieved from neoway
 */
enum NEOWAY_RETURN {
	$N_RESET,
	$N_OK,  //!<for successful communication
	$EXPECTED_RESPONSE,  //!<for getting expected response
	$GOT_RESPONSE,  //!<for getting Response
	$ex_resp_NULL,  //!<for null expected response
	$NO_RESPONSE,  //!<for getting no response
	$ERROR_RESPONSE,  //!<for getting error in response.
	$N_ERROR,  //!<for error in communication
};

/**
 * @enum SENSOR_RETURN
 * @brief To tell about status of Values returned from SENSORs
 */
enum SENSOR_RETURN {
	$OK,  //!<Designates Successful Communication
	$ERROR,  //!<Designates some sort of error in communication
	$SENSOR_ERROR,  //!<Designates some sort of error in Sensor
	$PARA_SIZE_0,  //!<Designates no parameter returned by sensor
	$PARA_SIZE_OUT,  //!<Designates parameter size out
	$OUT_OF_THRESHOLD  //!<Designates out of threshold status of parameter
};

#endif /* ENUMS_H_ */
