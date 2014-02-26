/*
 * Macros.h
 *
 *  Created on: 22/lug/2012
 *      Author: Mladen Mazuran
 */

#ifndef MACROS_H_
#define MACROS_H_

template <typename T>
struct ForeachContainer {
    ForeachContainer(T &t) : b(1), i(t.begin()), e(t.end()) {}
    int b; typename T::iterator i, e;
};

template <typename T>
struct ForeachContainer<const T> {
    ForeachContainer(const T &t) : b(1), i(t.begin()), e(t.end()) {}
    int b; typename T::const_iterator i, e;
};

/*
    Foreach emulation macro, probably works only with gcc / clang.
    Requires the iterable object to be in-scope. If the iterable object
    goes out of scope at the end of the line code (i.e. a return by value)
    expect bad stuff to happen
*/
#define foreach(decl, var) \
    for(ForeachContainer<__typeof__(var)> __f_container(var); \
        __f_container.b && __f_container.i != __f_container.e; \
        ++__f_container.i, __f_container.b--) \
        for(decl = *__f_container.i;; __extension__({ __f_container.b++; break; }))


#endif /* MACROS_H_ */
