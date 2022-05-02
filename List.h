#ifndef LIST_H
#define LIST_H
/**
 * An implementation of a generic list data structure
 * BECAUSE ENERGIA DOESN'T SUPPORT LISTS OR VECTORS IN C++ FOR SOME 
 * GOD FORSAKEN REASON?!?!?!?!?!?!!!!!!
 * 
 * kward
 */

#include "Energia.h"
#include "Utils.h"
#include <unistd.h>

// Generic list 
template<typename T>
class List {
  
  private:
    T* arr;               //  Internal data array
    uint32_t sz;          //  List size (number of elements it can store)
    uint32_t elems;       //  List length (number of elements actually in the list)
    uint32_t currIdx;     //  Current index at in the list

    // Resize internal data array by 2x
    void resize()
    {
      uint32_t startSize = sz;            //  Get size of data
      uint32_t newSize = startSize * 2;   //  Compute new array size

      // Create new, larger array
      T* newArr = new T[newSize];

      // Iterate through data and store in newData
      for(int i = 0; i < startSize; i++)
      {
        // Transfer data from old list to new list
        newArr[i] = arr[i];
      }

      // Set internal data array to newly sized array
      arr = newArr;

      // Update capacity
      sz = newSize;

    }
    

  public:

    /** 
     *  Main constructor
     *  
     *  @param nElements - Number of elements to initially allocate for list
     */
    List(uint32_t nElements)
    {
      // Dynamically allocate a nElements size array of type T
      arr = new T[nElements];

      // Set internal variables
      sz = nElements;
      elems = 0;
      currIdx = -1;
    }


    /**
     * Main destructor
     */
    ~List()
    {
      // Free dynamically allocated memory
      free(arr);
    }

    /**
     * Push a value to the end of the list
     */
    void push(T value)
    {
      // Check if there is enough space in the internal list to add elements
      if(currIdx + 1 < sz)
      {
        // Store data and increment current index
        currIdx++;
        arr[currIdx] = value;
        elems++;
      }
      else {
        // Not enough space. List needs to be resized
        resize();

        // Append data to array and increment index
        currIdx++;
        arr[currIdx] = value;
        elems++;
      }
    }
  
    /** 
     *  Return the array value at the corresponding index
     *  
     *  @param idx - index of element in list
     */
    T get(int idx)
    {
      // Validate index
      if(!range_check(idx, 0, sz-1))
      {
        Serial.println("\n\nERROR in List.get(): index " + String(idx) + " out of bounds for list of size " + String(sz));
        Serial.println("<END>");
        exit(0);
      }
      // Return imu_vals
      return arr[idx];
    }

    /** 
     *  Set value of internal array at index idx
     *  
     *  @param idx - index of element in list
     */
//    void set(uint32_t idx, T val)
//    {
//      // Validate index
//      if(!range_check(idx, 0, sz-1))
//      {
//        Serial.println("\n\nERROR in List.at(): index " + String(idx) + " out of bounds for list of size " + String(sz));
//        exit(0);
//      }
//
//      // Set value
//      arr[idx] = val;
//
//      // Check if the index being set had been set before
//      if(idx >= elems)
//      {
//        // Increment number of elements in the list
//        elems++;
//      }
//    }

    /**
     * Remove the value at a given index in the list
     */
//    void rmv(uint32_t idx)
//    {
//      // Validate index
//      if(!range_check(idx, 0, sz-1))
//      {
//        Serial.println("\n\nERROR in List.at(): index " + String(idx) + " out of bounds for list of size " + String(sz));
//        exit(0);
//      }
//
//      // Ensure there is an element stored at index
//      if(idx > elems)
//      {
//        return;
//      }
//
//      // Check for special case where idx is 0 (i.e. the array's base address)
//      if(idx == 0)
//      {
//        arr = &arr[idx+1];  // Move the head of the list forward 1 element
//        elems--;
//        return;
//      }
//
//      // Remove element from list by replacing it with 0s
//      arr[idx] &= 0;
//
//      // Decrement number of elements in list
//      elems--;
//    }

    /**
     * Return the number of elements that can be stored
     * in the list
     */
    uint32_t capacity()
    {
      return sz;
    }

   /**  
    *   Return number of elements currently stored in the 
    *   list
    */
    uint32_t len()
    {
      return elems;
    }
};










#endif  //  End LIST_H
