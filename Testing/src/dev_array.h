/*

  Based mostly on code from https://www.quantstart.com/articles/dev_array_A_Useful_Array_Class_for_CUDA
  I only added a couple extra things
*/

#ifndef _DEV_ARRAY_H_
#define _DEV_ARRAY_H_

#include <stdexcept>
#include <algorithm>
#include <vector>
#include <cuda_runtime.h>
#include <thrust/device_vector.h>
#include <thrust/remove.h>
#include <thrust/unique.h>
#include <thrust/binary_search.h>
#include <thrust/sort.h>

template <class T>
class dev_array
{
    // public functions
  public:
    explicit dev_array()
    : start_(0),
      end_(0)
    {}

    // constructor
    explicit dev_array(size_t size)
    {
      allocate(size);
    }
    // destructor
    ~dev_array()
    {
      free();
    }

    // resize the vector
    void resize(size_t size)
    {
      free();
      allocate(size);
    }

    // get the size of the array
    size_t getSize() const
    {
      return end_ - start_;
    }

    // get data
    const T* getData() const
    {
      return start_;
    }

    T* getData()
    {
      return start_;
    }

    // set
    void set(const T* src, size_t size)
    {
      size_t min = std::min(size, getSize());
      cudaError_t result = cudaMemcpy(start_, src, min * sizeof(T), cudaMemcpyHostToDevice);
      if (result != cudaSuccess)
      {
        throw std::runtime_error("failed to copy to device memory");
      }
    }
    // get
    void get(T* dest, size_t size)
    {
      size_t min = std::min(size, getSize());
      cudaError_t result = cudaMemcpy(dest, start_, min * sizeof(T), cudaMemcpyDeviceToHost);
      if (result != cudaSuccess)
      {
        throw std::runtime_error("failed to copy to host memory");
      }
      cudaDeviceSynchronize();
    }
    static void setGPU(int devID)
    {
      cudaError_t result =cudaSetDevice(devID);
      if (result != cudaSuccess)
      {
        throw std::runtime_error("failed to set device");
      }
    }
    static void removeRepeatedIdx(std::vector<int> indices_in, std::vector<int> indices_out)
    {
      thrust::device_vector<int> nn_points=indices_in;
      thrust::sort(nn_points.begin(),nn_points.end());
      nn_points.erase(thrust::unique(nn_points.begin(),nn_points.end()), nn_points.end());
      thrust::copy(nn_points.begin(),nn_points.end(),indices_out.begin());
    }

    // private functions
  private:
    // allocate memory on the device
    void allocate(size_t size)
    {
      cudaError_t result = cudaMalloc((void**)&start_, size * sizeof(T));
      if (result != cudaSuccess)
      {
        start_ = end_ = 0;
        throw std::runtime_error("failed to allocate device memory");
      }
      end_ = start_ + size;
    }

    // free memory on the device
    void free()
    {
      if (start_ != 0)
      {
        cudaFree(start_);
        start_ = end_ = 0;
      }
    }

    T* start_;
    T* end_;
};

#endif
