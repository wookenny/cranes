#include <iostream>
#include <cmath>  
#include <cstddef>
#include <type_traits>
#include <cassert>

template <std::size_t N, typename T>

class Point
{
public:
    Point()
    {
        std::fill_n(mData, N, T{});   
    }

    Point(const T& pX): mData{{pX}}
    {
        // or some variant (enable_if also works)
        static_assert(N == 1, "X constructor only usable in 1D");
    }


    Point(const T& pX, const T& pY): mData{{pX,pY}}
    {
        static_assert(N == 2, "XY constructor only usable in 2D");
    }

    Point(const T& pX, const T& pY, const T& pZ): mData{{pX,pY,pZ}}
    {
        static_assert(N == 3, "XYZ constructor only usable in 3D");
    }
    
    Point(std::initializer_list<T> c) {
        //static_assert(c.size() == N, "Wrong number of dimensions");
        assert(c.size() == N);
        std::copy(c.begin(), c.end(), mData);
    }
    
    // incomplete, left as exercise for reader. :P
    static double getDist(const Point<N, T>& pFirst, const Point<N, T>& pSecond)
    {
        // generic, compiler will unroll loops
        double dist;
        for(uint i=0; i<N;++i)
            dist += pFirst.mData[i]-pSecond.mData[i]; 
        return sqrt(dist);
    }
    

    std::string to_string() const{
        std::string str;
        for(uint i=0; i<N;++i){
            str += std::to_string(mData[i]);
            if(i!=N-1)
                str += " "; 
        }        
        return str;
    }
     
    const T& x() const{
        static_assert(N >= 1, "X coordinate not usable");
        return mData[0];
    }    
    
    const T& y() const{
        static_assert(N >= 1, "X coordinate not usable");
        return mData[0];
    }   
    
    const T& z() const{
        static_assert(N >= 1, "X coordinate not usable");
        return mData[0];
    }   
    
private:
    T mData[N];
    
};

int main(){
    Point<3,int> p;
    p = {1,2,3};
    Point<3,int> p1;
    p1 = {1,2,4};
    std::cout<< "'"<< p.to_string() <<"'"<<std::endl;
    std::cout<< "'"<< Point<3,int>::getDist(p1,p) <<"'"<<std::endl;
    
}
