
#pragma once

#include <exception>
#include <iostream>
#include <array>
#include <vector>

template<typename T, size_t D>
class Tensor
{
public:

    using iterator = typename std::vector<T>::iterator;

public:

    iterator begin()
    {
        return myData.begin();
    }

    iterator end()
    {
        return myData.end();
    }

    Tensor()
    {
        std::fill(myStrides.begin(), myStrides.end(), 0);
        std::fill(myDimensions.begin(), myDimensions.end(), 0);
    }

    const T& operator()(std::initializer_list<size_t> multi_index) const
    {
        return myData[computeIndex(multi_index)];
    }

    T& operator()(std::initializer_list<size_t> multi_index)
    {
        return myData[computeIndex(multi_index)];
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

    size_t computeIndex(std::initializer_list<size_t> multi_index) const
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
            if( i < 0 || myDimensions[k] <= i )
            {
                throw std::runtime_error("Incorrect multidimensional index!");
            }

            index += i*myStrides[k];
            k++;
        }

        return index;
    }


protected:

    std::array<size_t,D> myDimensions;
    std::array<size_t,D> myStrides;
    std::vector<T> myData;
};

