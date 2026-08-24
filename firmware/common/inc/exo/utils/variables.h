/**
 * @file VARIABLES.h
 * @brief for a general parent VARIABLES class definition which will be used be used by base classes including ANALOG,RS485,RAIN_GAUGE etc. for different.
 *
 *	dependencies :	DynamicArray.h
 *
 *  Created on: May 8, 2023
 *      Author: dhanveer.singh
 */

#ifndef INC_VARIABLES_H_
#define INC_VARIABLES_H_


/**
 * total_menu_index.a static variable which stores total no of variables representing total no of sensors connected to stm32
 */

static uint8_t total_menu_index = 0;
/**
 * for definition of VARIABLES class used to define an object representing **a sensor** collected.
 */
class VARIABLES {

		String VAR_NAME = "";  //!<  A String value representing this VARIABLES object's name.
		double VALUE = 1;  //!<gives the value of variable or if sensor is connected or not
		const uint8_t index = ++total_menu_index;  //!< A const uint8 value representing index no assigned to this VARIABLES object.
		bool add_to_json = 0;  //!< A bool value representing if this VARIABLES object is added to json or not.

	public:

		/**
		 *@brief A dynamic array for parameters stored by each variable.
		 *@details a Dynamic Array - (Object) with pointer pointing to a dynamic array of parameter(structure).Parameter-Dynamic-array-object consisting of pointer pointing to a dynamic array of parameter (structure).Initially Pointing to null
		 */
		DynamicArray<Parameter> parameter;
		/**
		 *@brief Constructor for VARIABLES object with certain input parameters.
		 *@detail To expand variables_pointer -dynamic array size by one and passing value as VARIABLES* Setting.
		 *@param [in] variables_pointer To add a new pointer pointing to VARIABLES object in variables_pointer object(defined in main function) by passing it's value by reference to VARIABLES Function.
		 *@param [in] VAR_NAME a String to assign this VARIABLES object variable name.
		 *@param [in] add_to_json a bool to designate if this  VARIABLES object should be added to json or not.
		 */
		VARIABLES(DynamicArray<VARIABLES*> *variables_pointer, String VAR_NAME, bool add_to_json = 0) {
			if (variables_pointer != NULL) {
				variables_pointer->push_back(this);
			}
			this->VAR_NAME = VAR_NAME;
			this->add_to_json = add_to_json;
		}
		/**
		 *@fn uint32_t ADD_PARA(String para_name)
		 *@brief To add parameter to this VARIABLE's object and assigning it name.
		 *@details Corresponding this VARIABLES's parameter dynamic array is expanded by one and new Parameter name is assigned to it.
		 *@param [in] para_name To add a parameter name to new Parameter structure which has to be added to parameter dynamic array.
		 *@return parameter.size() Returns size of updated parameter Dynamic array of corresponding this VARIABLES Object.
		 */
		uint32_t ADD_PARA(String para_name) {
			Parameter temp;
			temp.name = para_name;
			parameter.push_back(temp);
			return parameter.size();
		}
		/**
		 *@fn SENSOR_RETURN SET_PARA_VALUE(uint16_tpara_index, double value)
		 *@brief To set a value to a particular parameter of this VARIABLES object as per input parameter index no and String value.
		 *@details If(index is within allowable range) then pointer corresponding to input index no is returned and then assigned a String value by typecasting using doubletoString func.
		 *@param [in] para_index a uint indicating dynamic array-parameter's -index no whose Parameter structure's value is to be defined.
		 *@param [in] value a double value assigning  to the parameter.
		 *@return SENSOR_RETURN if(input index is within allowable range) return $OK enum else return $para_size_out enum.
		 */
		SENSOR_RETURN SET_PARA_VALUE(uint16_t para_index, double value, int decimals = 4) {
			if (para_index < parameter.size()) {
				parameter.at(para_index)->value_double = value;
//			parameter.at(para_index)->value_string = "";
				parameter.at(para_index)->value_string = d_t_s(value, decimals);
				return $OK;
			} else {
				return $PARA_SIZE_OUT;
			}
		}
		/**
		 *@fn SENSOR_RETURN SET_PARA_VALUE(uint para_index, String value)
		 *@brief To set a value to a particular parameter of this VARIABLES object as per input parameter index no and String value.
		 *@details If(index is within allowable range) then pointer corresponding to input index no is returned and then assigned a String value.
		 *@param [in] para_index a uint indicating dynamic array-parameter's -index no whose Parameter structure's value is to be defined.
		 *@param [in] value a String value assigning  to the parameter.
		 *@return SENSOR_RETURN if(input index is within allowable range) return $OK enum else return $para_size_out enum.
		 */
		SENSOR_RETURN SET_PARA_VALUE(uint16_t index, String value) {
			if (index < parameter.size()) {
				parameter.at(index)->value_string = value;
				parameter.at(index)->value_double = s_t_d(value);

				return $OK;
			} else {
				return $PARA_SIZE_OUT;
			}
		}
		/**
		 * @fn Parameter* GET_PARAMETER(uint8_t para_index).
		 * @brief To get a pointer pointing to parameter structure corresponding to input para_index of this VARIABLES object.
		 * @param [in] para_index uint8 dynamic parameter's array index no whose pointer is to be returned.
		 * @return a Pointer Pointing to Parameter structure.
		 */
		Parameter* GET_PARAMETER(uint8_t para_index) {
			return parameter.at(para_index);
		}
		/**
		 * @fn String GET_VALUE(uint8_t para_index).
		 * @brief To get value stored in a parameter structure corresponding to input para_index of this VARIABLES object.
		 * @param [in] para_index uint8 indicating dynamic array-parameter's -index no whose Parameter structure's value is to be returned.
		 * @return String para_index's parameter's value.
		 */
		double GET_VALUE_DOUBLE(uint8_t para_index) {
			return parameter.at(para_index)->value_double;
		}
		/**
		 * @fn String GET_VALUE(uint8_t para_index).
		 * @brief To get value stored in a parameter structure corresponding to input para_index of this VARIABLES object.
		 * @param [in] para_index uint8 indicating dynamic array-parameter's -index no whose Parameter structure's value is to be returned.
		 * @return String para_index's parameter's value.
		 */
		String GET_VALUE(uint8_t para_index) {
			return parameter.at(para_index)->value_string;
		}
		/**
		 * @fn String GET_NAME(uint8_t para_index).
		 * @brief To get name of parameter stored in a parameter structure corresponding to input para_index of this VARIABLES object.
		 * @param [in] para_index uint8 indicating dynamic array-parameter's -index no whose Parameter structure's name is to be returned.
		 * @return String return Parameter name of input index no.
		 */
		String GET_NAME(uint8_t para_index) {
			return parameter.at(para_index)->name;
		}
		/**
		 * @fn uint32_t GET_PARA_SIZE().
		 * @brief To get no of parameter stored in this VARIABLES object.
		 * @return parameter-size no of parameters stored in this VARIABLES object.
		 */
		uint32_t GET_PARA_SIZE() {
			return parameter.size();
		}
		/**
		 * @fn int GET_index() const.
		 * @brief To get index no of this VARIABLES object.
		 * @return index no corresponding to this VARIABLES object.
		 */
		int GET_index() const {
			return index;
		}
		/**
		 * @fn const String& GET_VAR_NAME() const.
		 * @brief To get variables no of this VARIABLES object.
		 * @return a constant String designating variable name corresponding to this VARIABLES object.
		 */
		const String& GET_VAR_NAME() const {
			return VAR_NAME;
		}
		/**
		 * @fn bool IS_add_to_json() const.
		 * @brief To get to know if this VARIABLES object is added to json or not.
		 * @return a bool value designating add_to_jason value corresponding to this VARIABLES object.
		 */
		bool IS_add_to_json() const {
			return add_to_json;
		}
		/**
		 * @fn void SET_add_to_json(bool add_to_json = 0).
		 * @brief To set add this VARIABLES object to json.
		 * @param [in] add_to_json a bool value indicating if this VARIABLES object is to be added to json or not.By default initialized to 0.
		 */
		void SET_add_to_json(bool add_to_json = 0) {
			this->add_to_json = add_to_json;
		}
		/**
		 * @fn double GET_VALUE() const.
		 * @brief To get this VARIABLEs object's connected value.
		 * @return VALUE a double value.
		 */
		double GET_VALUE() const {
			return VALUE;
		}
		/**
		 * @fn void SET_VALUE(double VALUE = 0).
		 * @brief To set this VARIABLEs object's connected value.
		 * @param [in] VALUE a double value to set value of this VARIABLES object's VALUE.
		 */
		void SET_VALUE(double VALUE = 0) {
			this->VALUE = VALUE;
		}
		double* GET_VALUE_PTR() {
			return &VALUE;
		}
};

#endif /* INC_VARIABLES_H_ */
