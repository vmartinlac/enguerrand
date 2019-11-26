
#pragma once

#include <iostream>
#include <array>
#include <vector>

template<typename T, size_t D>
class Tensor
{
public:

    Tensor()
    {
        std::fill(myStrides.begin(), myStrides.end(), 0);
        std::fill(myDimensions.begin(), myDimensions.end(), 0);
    }

    T& operator()(std::initializer_list<size_t> multi_index)
    {
        if(multi_index.size() != D)
        {
            std::cerr << "Internal error!" << std::endl;
            exit(1);
        }

        size_t index = 0;
        size_t k = 0;

        for(size_t i : multi_index)
        {
            index += i*myStrides[k];
            k++;
        }

        return myData[index];
    }

    void resize(std::initializer_list<size_t> dims)
    {
        if(dims.size() != D)
        {
            std::cerr << "Internal error!" << std::endl;
            exit(1);
        }

        size_t count = 1;
        size_t k = 0;

        for(size_t d : dims)
        {
            myStrides[k] = count;
            count *= d;
            myDimensions[k] = d;
            k++;
        }

        myData.resize(count);
    }

    void swap(Tensor<T,D>& other)
    {
        std::swap(myDimensions, other.myDimensions);
        std::swap(myStrides, other.myStrides);
        myData.swap(other.myData);
    }

    size_t size(size_t i)
    {
        return myDimensions[i];
    }

protected:

    std::array<size_t,D> myDimensions;
    std::array<size_t,D> myStrides;
    std::vector<T> myData;
};


