#include "CursedDoubleLinkedListInterface.hpp"
#include "LinkedList.h"
// #include <iostream>
#include "AList.h"

// template <typename T>
LinkedList<int> curList;

// template <typename T>
AList::AList(){
    curList = LinkedList<int>();
};

// template <typename T>
bool AList::add(int value){
    curList.add(value);
    return true;
};

// template <typename T>
int AList::get(int index){
    return curList.get(index);
};

// template <typename T>
bool AList::remove(int index){
    curList.remove(index);
    return true;
};

// template <typename T>
bool AList::removeLast(){
    curList.remove(curList.size() - 1);
    return true;
};

// template <typename T>
bool AList::removeAll(){
    curList.clear();
    return true;
};