//
// Created by root on 16/09/16.
//

#ifndef ARRAY_H
#define ARRAY_H

#include <cstdint>

#define ARRAY_FULL 1
#define ARRAY_IS_NOT_FULL 0

template <typename T, uint16_t qsize>
class Array
{
private:
    T array[qsize];
    T buf;

public:
    uint16_t start_index = 0;
    uint16_t end_index;
    uint16_t current_index;
    uint8_t full;

    Array();
    ~Array();

    uint16_t size();
    uint16_t capacity();
    uint8_t empty();
    void clear();

    T * data();
    T & operator[](uint16_t index);

    uint8_t push(T const&);
    // uint8_t push_front(T const&);

    T pop(uint16_t index);
    //T pop_back();


    T & front();
    T & back();

    class iterator
    {
    private:
        Array<T, qsize> *array;
        uint16_t *end_index;
    protected:

    public:
        iterator(Array *, uint16_t index);
        ~iterator();

        void operator++();

        T * operator->();
        T & operator*();
        uint8_t operator!=(iterator it);
        uint8_t operator==(iterator it);
    };

    Array<T, qsize>::iterator begin();
    Array<T, qsize>::iterator end();
};

///////////////////////// ARRAY::ITERATOR CONSTRUCTORS ///////////////////////////////
template <typename T, uint16_t qsize>
Array<T, qsize>::iterator::iterator(Array<T, qsize> *array, uint16_t index)
{
    this->array = array;
    this->array->current_index = index;
    end_index = &(this->array->end_index);
}

template <typename T, uint16_t qsize>
Array<T, qsize>::iterator::~iterator() {}

template <typename T, uint16_t qsize>
typename Array<T, qsize>::iterator Array<T, qsize>::begin()
{
    return iterator(this, 0);
}

template <typename T, uint16_t qsize>
typename Array<T, qsize>::iterator Array<T, qsize>::end()
{
    return iterator(this, end_index-1);
}

///////////////////////// ITERATOR OPERATORS ///////////////////////////////
template <typename T, uint16_t qsize>
void Array<T, qsize>::iterator::operator++()
{
    if (array->empty()) return;
    if (this->array->current_index+1 <= *end_index)
        this->array->current_index++;


}

template <typename T, uint16_t qsize>
uint8_t Array<T, qsize>::iterator::operator!=(iterator it)
{
    return it.array->current_index != this->array->current_index;
}

template <typename T, uint16_t qsize>
T * Array<T, qsize>::iterator::operator->()
{
    return &(this->array->array[array->current_index]);
}

template <typename T, uint16_t qsize>
T & Array<T, qsize>::iterator::operator*()
{
    return this->array->array[array->current_index];
}

template <typename T, uint16_t qsize>
uint8_t Array<T, qsize>::iterator::operator==(iterator it)
{
    return it.array->current_index == this->array->current_index;
}

///////////////////////// ARRAY CONSTRUCTORS AND METHODS ///////////////////////////////


template <typename T, uint16_t qsize>
Array<T, qsize>::Array()
{
    full = ARRAY_IS_NOT_FULL;
    start_index = end_index = 0;
}

template <typename T, uint16_t qsize>
Array<T, qsize>::~Array() {}

template <typename T, uint16_t qsize>
uint16_t Array<T, qsize>::size()
{
    return end_index - start_index;
}

template <typename T, uint16_t qsize>
uint16_t Array<T, qsize>::capacity()
{
    return qsize;
}

template <typename T, uint16_t qsize>
uint8_t Array<T, qsize>::empty()
{
    return end_index == 0;
}

template <typename T, uint16_t qsize>
void Array<T, qsize>::clear()
{
    full = ARRAY_IS_NOT_FULL;
    start_index = end_index = 0;
}

template <typename T, uint16_t qsize>
T * Array<T, qsize>::data()
{
    return array;
}

template <typename T, uint16_t qsize>
T & Array<T, qsize>::operator[](uint16_t index) {
        if (index < end_index) {
            return array[index];
        }
}

template <typename T, uint16_t qsize>
uint8_t Array<T, qsize>::push(T const &item)
{
    // If storage has no space return.
    if (full) return 0;

    array[end_index++] = item;
    // Check for full state.
    if (end_index >= qsize)
    {
        full = ARRAY_FULL;
    }

    return 1;
}

template <typename T, uint16_t qsize>
T Array<T, qsize>::pop(uint16_t index)
{
    if (!this->empty()) {
        if (end_index > index) {
            if (end_index == 1) {
                end_index--;
                return array[index];
            } else {
                buf = array[index];     // push(42); array[0] = 42, end_index = 1, capacity = 1;
                end_index--;
                array[index] = array[end_index];
                if(current_index == end_index){
                    current_index--;
                }
                return buf;
            }

        }

    } else {
        end_index = 0;
    };


    if (start_index >= qsize)
    {
        start_index = 0;
    }

    if (full) full = ARRAY_IS_NOT_FULL;
}


#endif //ARRAY_H
