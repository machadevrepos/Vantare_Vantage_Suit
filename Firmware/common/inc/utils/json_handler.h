/*
 * FUNCTIONS.h
 *
 *  Created on: Nov 27, 2024
 *      Author: dhanv
 */

#ifndef JSONHANDLER_H_
#define JSONHANDLER_H_

#include <INCLUDER.h>

//#define SLS_HUB
//#define SLS_RELAY
//#define SLS_DALI

cJSON *config_doc = NULL;

#define config_uid config_doc["UID"]

#if defined(SLS_HUB)
#define config_spokes config_doc["spokes"]
#endif

#if defined(SLS_RELAY)  || defined (SLS_DALI)


#define config_hub config_doc["hub"]
#endif

LOOP_CONT JSON_EXTRACTER(String &json_string, cJSON **json_ret) {
	uint32_t json_start_index = json_string.find('{');
	uint32_t json_end_index = json_string.find('}');

	if (json_start_index != not_found && json_end_index != not_found) {
		json_string = json_string.substr(json_start_index, json_end_index - json_start_index + 1);
		if (*json_ret) {
			cJSON_Delete(*json_ret);
		}
		*json_ret = cJSON_Parse(json_string.c_str());
		if (*json_ret == NULL) {
			return $BREAK;
		}
		return $CONTINUE;
	} else {
		return $BREAK;
	}
}

#endif /* FUNCTIONS_H_ */
