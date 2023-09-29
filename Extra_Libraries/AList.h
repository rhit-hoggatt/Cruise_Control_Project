//AList.h

#ifndef ALIST_H
#define ALIST_H

// #include <Arduino.h>

template <typename T>
class AList {
    private:
        int size;
        
    public:
        AList(T type);
        bool add(int value);
        T get(int index);
        bool remove(int index);
        bool removeLast();
        bool removeAll();
};

#endif