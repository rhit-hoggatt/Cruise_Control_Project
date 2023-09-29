#include "CursedDoubleLinkedListInterface.hpp"
#include "LinkedList.h"
#include <iostream>
#include "AList.h"

template <typename T>
LinkedList<T> curList;

template <typename T>
AList<T>::AList(){
    curList = LinkedList<T>();
};

template <typename T>
bool AList<T>::add(T value){
    curList.add(value);
    return true;
};

template <typename T>
T AList<T>::get(int index){
    return curList.get(index);
};

template <typename T>
bool AList<T>::remove(int index){
    curList.remove(index);
    return true;
};

template <typename T>
bool AList<T>::removeLast(){
    curList.remove(curList.size - 1);
    return true;
};

template <typename T>
bool AList<T>::removeAll(){
    curList.clear();
    return true;
};