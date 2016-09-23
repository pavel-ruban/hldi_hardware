//
// Created by root on 16/09/16.
//
#pragma once
#include <scheduler/include/event.h>
#include <cstdint>
#include <stm32f10x_conf.h>
#include "config.h"
#pragma pack(1)

#define ARRAY_FULL 1
#define ARRAY_IS_NOT_FULL 0

template <typename T, uint16_t qsize>
class Array
{
private:
    T array[qsize];
    T buf;
    uint16_t obj_count;
public:
    uint16_t start_index = 0;
    uint16_t end_index;
    uint8_t full;
    Array();
    ~Array();
    uint16_t size();
    uint16_t capacity();
    uint8_t empty();
    void clear();
    T * data();
    T & operator[](uint16_t index);
    uint8_t push(T item);
    T pop(uint16_t index);
    T & front();
    T & back();
    class iterator
    {
    private:
        Array<T, qsize> *array;
    protected:

    public:
        iterator(Array *, uint16_t index);
        ~iterator();
        uint16_t current_index;

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
Array<T, qsize>::iterator::iterator(Array<T, qsize> *array, uint16_t index) {
    this->array = array;
    this->current_index = index;
}

template <typename T, uint16_t qsize>
Array<T, qsize>::iterator::~iterator() {

}

template <typename T, uint16_t qsize>
typename Array<T, qsize>::iterator Array<T, qsize>::begin()
{
    return iterator(this, 0);
}

template <typename T, uint16_t qsize>
typename Array<T, qsize>::iterator Array<T, qsize>::end()
{
    uint32_t bufindex = 0;
    for (uint32_t i  = 0; i < qsize; i++) {
        if(array[i].deleted == 0)
            bufindex = i;
    }
    end_index = bufindex + 1;
    return iterator(this, bufindex + 1);
}

///////////////////////// ITERATOR OPERATORS ///////////////////////////////
template <typename T, uint16_t qsize>
void Array<T, qsize>::iterator::operator++()
{
    if (array->empty()) return;
    if (array->end_index <= current_index) {
        current_index = array->end_index;
        return;
    }
    for (uint16_t i = current_index + 1;i < array->end_index; i++) {
        if (array->array[i].deleted != 1) {
            current_index = i;
            return;
        }
    }
    current_index = array->end_index;
    return;
}

template <typename T, uint16_t qsize>
uint8_t Array<T, qsize>::iterator::operator!=(iterator it)
{
    if (it.current_index < current_index) {

    }
    return it.current_index != current_index;
}

template <typename T, uint16_t qsize>
T * Array<T, qsize>::iterator::operator->()
{
    return &(this->array->array[current_index]);
}

template <typename T, uint16_t qsize>
T & Array<T, qsize>::iterator::operator*()
{
    return this->array->array[current_index];
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
    obj_count = 0;
    full = ARRAY_IS_NOT_FULL;
    start_index = end_index = 0;
    for (uint32_t i = 0; i < qsize; i++) {
        array[i].deleted = 1;
    }
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
    return end_index;
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
    for (uint32_t i = 0;i < qsize;i++ ) {
        array[i].deleted = 1;
    }
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
uint8_t Array<T, qsize>::push(T item)
{
    // If storage has no space return.
    if (full) return 0;

    for (uint32_t i = 0; i < qsize; i++) {
        if (array[i].deleted == 1) {
            array[i] = item;
            array[i].deleted = 0;
            obj_count++;
            if (i >= end_index)
                end_index++;
            return 1;
        }
    }
    if (obj_count >= qsize)
    {
        full = ARRAY_FULL;
    }

    return 1;
}

template <typename T, uint16_t qsize>
T Array<T, qsize>::pop(uint16_t index)
{
    if (!this->empty()) {
        if (array[index].deleted) {
                while(1) {

                }
        }
        T buf = array[index];
        array[index].deleted = 1;
        obj_count--;
        if (obj_count < qsize)
        {
            full = ARRAY_IS_NOT_FULL;
        }
        if (end_index == index + 1) {  //Если удаление элемента произошло перед конечным элементом
            for (uint16_t i = end_index; i > 0; i--) {
                if (array[i].deleted == 0) {
                    end_index = i + 1;
                    return buf;
                }
            }
            end_index = 0;
            return buf;
        }
        return buf;

    } else {
        end_index = 0;
        assert_param(0);
    };
}
