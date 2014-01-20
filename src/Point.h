#pragma once
#include <iostream>
#include <cmath>  
#include <cstddef>
#include <cstdint>
#include <type_traits>

template <std::size_T N, typename T>

class Point
{
public:
    Point()
    {
        std::fill_n(mData, N, T());   
    }

    explicit Point(const T& pX): mData[0](pX)
    {
        // or some variant (enable_if also works)
        static_assert(N == 1, "X constructor only usable in 1D");
    }


    explicit Point(const T& pX, const T& pY):
    mData[0](pX),
    mData[1](pY)
    {
        static_assert(N == 2, "XY constructor only usable in 2D");
    }

    explicit Point(const T& pX, const T& pY, const T& pZ):
    mData[0](pX),
    mData[1](pY),
    mData[2](pZ)
    {
        static_assert(N == 3, "XYZ constructor only usable in 3D");
    }
    
    /**
    // incomplete, left as exercise for reader. :P
    template <std::size_T N, typename T>
    static T getDist(const Point<N, T>& pFirst, const Point<N, T>& pSecond)
    {
        // generic, compiler will unroll loops
        T dist;
        for(uint i=0; i<N;++i)
            dist += pFirst[i]-pSecond[i]; 
        return sqrt(dist);
    }
    
    template <std::size_T N, typename T>
    std::string to_string() const{
        std::string str;
        for(uint i=0; i<N;++i)
            str += mData[i];
            if(i!=N-1)
                str += " "; 
        return str;
    }
       
    **/
private:
    T mData[N];
    
};


