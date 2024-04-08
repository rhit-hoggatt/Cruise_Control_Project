//AList.h

#ifndef ALIST_H
#define ALIST_H

// #include <Arduino.h>

// template <typename T>
class AList {
    private:
        int size;
        
    public:
        AList();
        bool add(int value);
        int get(int index);
        bool remove(int index);
        bool removeLast();
        bool removeAll();
};

#endif