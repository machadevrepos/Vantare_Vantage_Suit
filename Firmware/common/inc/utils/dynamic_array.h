/**
 * @file :DynamicArray.h
 * @brief: A generic DynamicArray class Definition.
 *  	Created on: May 4, 2023
 *      Author: dhanveer.singh
 *      Dynamic array template based
 */

#ifndef DYNAMICARRAY_H_
#define DYNAMICARRAY_H_

#include <main.h>
/**
 * A generic type Dynamic Array.
 * ```
 * Tda is used as a generic type in dynamic array class definition which may be then replaced by a particular datatype during class contructor call.
 * ```
 */
template<typename Tda>
class DynamicArray {
		/**
		 * Generic Pointer for Dynamic Array.
		 * A Generic Pointer pointing to Null initially
		 */
		Tda *ptr = NULL;
		/**
		 * 32 bit-int size of Dynamic Array.
		 * Holds size of dynamic array to be created
		 */
		uint32_t u_size;

	public:
//	allocate pointer to new memory location , set capacity to size+1 == 1 at start
		/**
		 *@brief Constructor for Dynamic Array without any input parameter
		 *@details assigns **u_size =0** and updates **ptr** pointer to a new dynamic memory location of `size = u_size+1`
		 */
		DynamicArray() {
			u_size = 0;
			ptr = new Tda[u_size + 1];
		}
// allocate pointer to new array with starting size given
		/**
		 *@brief Constructor for Dynamic Array with u_size input parameter
		 *@details Updates Size of dynamic array also Updates Generic Pointer pointing to new dynamic array
		 *@param [in] u_size Size of Dynamic Array to be created.
		 */
		DynamicArray(uint32_t u_size) {
			this->u_size = u_size;
			ptr = new Tda[u_size + 1];
		}
		/**
		 *@brief Destructor for Dynamic Array
		 */
		~DynamicArray() {
//		u_size = 0;
		}
		//	return size of array
		/**
		 *@fn size()
		 *@brief To Return Size of this Dynamic Array
		 *@return 32bit Unsized integer present in u_size
		 */
		void clean() {
			//  make size equals to size of list
			u_size = 0;
			// make temp Darray with size + 1
			Tda *temp_ptr = new Tda[u_size + 1];
			// delete old array
			delete[] ptr;
			// allocate pointer to new array
			ptr = temp_ptr;
		}
//	return size of array
		/**
		 *@fn size()
		 *@brief To Return Size of this Dynamic Array
		 *@return 32bit Unsized integer present in u_size
		 */
		uint32_t size() {
			return u_size;
		}
// push back new element at the end,
		/**
		 * @fn void push_back(Tda value)
		 *@brief To create one extra memory location in dynamic array and assign it with input: value.
		 *@details Calling expand_1() function and Updating last index's value in updated Dynamic Array
		 *@param[in]  value  Generic Data type value to be added to expanded dynamic array.
		 */
		void push_back(Tda value) {
			// create new space
			expand_1();
			//add element at last
			ptr[u_size - 1] = value;
		}
// push back new element at the end,
		/**
		 * @fn void push_back(Tda value)
		 *@brief To create one extra memory location in dynamic array and assign it with input: value.
		 *@details Calling expand_1() function and Updating last index's value in updated Dynamic Array
		 *@param[in]  value  Generic Data type value to be added to expanded dynamic array.
		 */

		Tda pop_back() {
		    if (u_size == 0) {
		        return 0;
		    }

		    // Get the last element
		    Tda popped_element = ptr[u_size - 1];

		    // Contract the array
		    contract_1();

		    return popped_element;
		}


// expand size by one
		/**
		 *@fn void expand_1()
		 *@brief To increment Size of Existing Dynamic Array by 1
		 *@details Updates *u_size* of Existing Dynamic Array by 1 and Updates *ptr* to a New Expanded Dynamic Array
		 */
		void contract_1() {
		    // Check if the array can be contracted (avoiding negative size)
		    if (u_size <= 1) {
		        return; // Can't contract further
		    }

		    // Create a temporary array of decreased size
		    Tda *temp_ptr = new Tda[--u_size - 1];

		    // Copy old values to the new array
		    for (uint32_t iterator = 0; iterator < u_size; iterator++) {
		        temp_ptr[iterator] = ptr[iterator];
		    }

		    // Delete the old array
		    delete[] ptr;

		    // Assign the pointer to the new array
		    ptr = temp_ptr;
		}


// expand size by one
		/**
		 *@fn void expand_1()
		 *@brief To increment Size of Existing Dynamic Array by 1
		 *@details Updates *u_size* of Existing Dynamic Array by 1 and Updates *ptr* to a New Expanded Dynamic Array
		 */
		void expand_1() {
			// make a temporary array of increased size
			Tda *temp_ptr = new Tda[++u_size + 1];
			// assign all old values to new array
			for (uint32_t iterator = 0; iterator < u_size - 1; iterator++) {
				*(temp_ptr + iterator) = *(ptr + iterator);
			}
			// delete old array
			delete[] ptr;
			// allocate pointer to new array
			ptr = temp_ptr;
		}
// access element at given index, if index not available, returns pointer to last index
		/**
		 * @fn Tda* at(uint32_t index)
		 *@brief To return Generic Pointer pointing to a generic type value stored in corresponding input : index no.
		 *@details if(demanded index no. matches with allowable indices) then Return Generic Pointer Pointing to Corresponding Index else Return Generic Pointer Pointing to Last Index.
		 *@param[in]  index  32 bit INDEX No of this Dynamic Array  whose Generic Pointer is to be Returned .
		 *@return ptr+index a generic Pointer of type Tda ,Pointing to input index no corresponding to this  dynamic array.
		 */
		Tda* at(uint32_t index) {
			if (index < u_size) {
				return ptr + index;
			} else {
				return ptr + u_size - 1;
			}
		}
		/**
		 * @fn void swap(uint32_t at_1, uint32_t at_2)
		 *@brief To swap values corresponding to 2 input indices of this Dynamic Array
		 *@details Elements corresponding to input Index numbers are being swapped  with each other.
		 *@param[in]  at_1,at_2  Input indices whose values are to be swapped.
		 */
		void swap(uint32_t at_1, uint32_t at_2) {
			if (at_1 < u_size && at_2 < u_size) {

				Tda *ptr_at_1 = ptr + at_1;
				Tda *ptr_at_2 = ptr + at_2;
				Tda ptr_temp = *(ptr + at_1);

				*ptr_at_1 = *ptr_at_2;
				*ptr_at_2 = ptr_temp;

			}

		}
// returns 1 if size = 0
		/**
		 * @fn bool is_empty()
		 *@brief To check if this Dynamic Array is empty or not
		 *@details Checks for value of u_size of this Dynamic array.
		 *@return true if u_size == 0 else false
		 */
		bool is_empty() {
			return u_size == 0;
		}
// assign list to the array, deletes old Darray
		template<typename Tp>
		/**
		 * @fn void assign(initializer_list<Tp> list)
		 *@brief To assign a list of generic type <Tp> to this dynamic array of type <Tda> & Size may be equal or may be not equal.
		 *@details Updates Size of Existing Dynamic array to size of input list also Updates Pointer to a new Dynamic array of updated size with typecasted contents being copied from list
		 *@param[in]  list  A generic type List whose contents are to be copied to existing Dynamic Array.
		 */
		void assign(std::initializer_list<Tp> list) {
			//  make size equals to size of list
			u_size = list.size();
			// make temp Darray with size + 1
			Tda *temp_ptr = new Tda[u_size + 1];
			// pointer to first element of list for iteration
			Tda *list_ptr = (Tda*) list.begin();
			// assign all elements of list to Darray
			for (uint32_t iterator = 0; iterator < u_size; iterator++) {
				*(temp_ptr + iterator) = (Tda) *(list_ptr + iterator);
			}
			// delete old array
			delete[] ptr;
			// allocate pointer to new array
			ptr = temp_ptr;
		}
// assign array to dynamic array
		template<typename Tp>
		/**
		 *@fn void assign(Tp arr[], uint16_t *len)
		 *@brief To assign an array of generic type <Tp> to this dynamic array of type <Tda> & Size may be equal or may not be equal.
		 *@details Updates Size of Existing Dynamic array to size of input array also Updates Pointer to a new Dynamic array of updated size with typecasted contents being copied from input array
		 *@param[in]  arr[] An Array of generic Type <Tp> whose contents are to be copied to this Dynamic Array of generic type <Tda>.
		 *@param[in]  len A Pointer pointing to 16 bit unsigned integer indicating size of input arr[].

		 */
		void assign(Tp arr[], uint16_t *len) {
			//  make size equals to size of array given
			u_size = *len;
			// make temp Darray with size + 1
			Tda *temp_ptr = new Tda[u_size + 1];
			// assign all elements of list to Darray
			for (uint32_t iterator = 0; iterator < u_size; iterator++) {
				*(temp_ptr + iterator) = (Tda) *(arr + iterator);
			}
			// delete old array
			delete[] ptr;
			// allocate pointer to new array
			ptr = temp_ptr;
		}
// equals operator
		/**
		 * @fn DynamicArray<Tda> operator=(DynamicArray<Tda> &obj)
		 *@brief To overload equalto(=) operate , to assign a dynamic array object to this dynamic array such that both having same generic type <Tda> & Size may be equal or may not be.
		 *@details Updates Size of Existing Dynamic array to size of input dynamic array object also Updates Pointer to a new Dynamic array of updated size with contents being copied from input object.
		 *@param[in]  obj  A Dynamic Array object whose contents are to be copied to this Dynamic Array.
		 */
		DynamicArray<Tda> operator=(DynamicArray<Tda> &obj) {
			// make temp Darray with object size
			Tda *temp_ptr = new Tda[obj.u_size];
			// assign all elements of list to Darray
			for (int iterator = 0; iterator < u_size - 1; iterator++) {
				*(temp_ptr + iterator) = *(ptr + iterator);
			}
			// delete old array
			delete[] ptr;
			// allocate pointer to new array
			ptr = temp_ptr;

		}
// return pointer to first element
		/**@fn first()
		 *@brief To return a generic Pointer pointing to First index of this dynamic array
		 *@return ptr A generic Pointer of type Tda.
		 */
		Tda* first() {
			return ptr;
		}

};

#endif /* DYNAMICARRAY_H_ */
