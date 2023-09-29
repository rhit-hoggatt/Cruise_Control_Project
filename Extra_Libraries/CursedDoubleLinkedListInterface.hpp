#ifndef CURSED_DOUBLE_LINKED_LIST_INTERFACE_HPP
#define CURSED_DOUBLE_LINKED_LIST_INTERFACE_HPP
// CursedDoubleLinkedListInterface.hpp
// Library is what it name is - it's a cursed linked list interface for C++
// but I made it for Arduino just to mess with more people heads.
//
// It works great if you want to create objects and have them accesible from anywhere
// Good for stack allocation as well as heap or static allocation
//
// Its supposed to be used as intefrace and templated.
// It will return pointer to the T object on iteration so it's useful to remove boilerplate code
// Enjoy and please note this is MIT licensed so I am not responsible - especially for damage while reading it
//
// This library is **evil**, you have been warned

// Derive it and pass your class type into T
template <typename T>
class CDLLI {
    // We only need _next but _prev is useful for handling it faster
    // and accesing nearby nodes faster
    CDLLI<T>* _next;
    CDLLI<T>* _prev;
 public:
    // head and tail are static, this way you can use it easily with T.head for iteration
    static CDLLI<T>* head;
    static CDLLI<T>* tail;
    
    // Initialization - on construction object is automatically added to the linked list
    explicit CDLLI(bool toFront = false) {
        if(toFront) {
            _prev = nullptr;
            if(!tail) tail = this;
            if(head) head->_prev = this;
            _next = head;
            head = this;
        } else {
            _prev = tail;
            if(!head) head = this;
            if(tail) tail->_next = this;
            tail = this;
            _next = nullptr;
        }
    }
    
    virtual ~CDLLI() {
        if(_prev) _prev->_next = _next;
        if(_next) _next->_prev = _prev;
        if(tail == this) tail = _prev;
        if(head == this) head = _next;
    }
    
    // NOTE: use object.T::next() if object is inherited by different lists
    T* next(bool wrap = false) {
        if(!_next && wrap) return (T*) head;
        return ((T*) _next);
    }
    T* previous(bool wrap = false) {
        if(!_prev && wrap) return (T*) tail;
        return ((T*) _prev);
    }

    static T& getHead() {
        return *(T*)head;
    }

    static T& getTail() {
        return *(T*)tail;
    }
    
    class Iterator {
        CDLLI<T>* current;

     public:
        explicit Iterator(CDLLI<T>* CDLLI) : current(CDLLI) {}

        Iterator& operator++() {
            current = current->_next;
            return *this;
        }

        bool operator!=(const Iterator& other) const {
            return current != other.current;
        }

        T& operator*(){
            return *((T*) current);
        }
    };

    Iterator begin() {
        return Iterator(head);
    }

    Iterator end() {
        return Iterator(nullptr);
    }

    // custom swap method by @creator from OLC Discord
    void swap(CDLLI<T>& other) {
        head = (head == this ? &other :
                       (head == &other ? this : head));
        tail = (tail == this ? &other :
                       (tail == &other ? this : tail));

        auto swap = _prev;
        _prev = other._prev;
        other._prev = swap;

        swap = _next;
        _next = other._next;
        other._next = swap;

        if(_prev) _prev->_next = this;
        if(_next) _next->_prev = this;
        if(other._prev) other._prev->_next = &other;
        if(other._next) other._next->_prev = &other;
    }
};

template <typename T>
CDLLI<T>* CDLLI<T>::head = nullptr;
template <typename T>
CDLLI<T>* CDLLI<T>::tail = nullptr;

#endif