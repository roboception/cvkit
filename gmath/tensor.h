/*
 * This file is part of the Computer Vision Toolkit (cvkit).
 *
 * Author: Heiko Hirschmueller
 *
 * Copyright (c) 2026 Roboception GmbH
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef GMATH_TENSOR_H
#define GMATH_TENSOR_H

#include "dvector.h"
#include "dmatrix.h"

#include <initializer_list>

#include <iostream>

#include <gutil/exception.h>

#include <ios>
#include <stdexcept>
#include <sstream>
#include <vector>
#include <exception>
#include <limits>
#include <functional>

namespace gmath
{

/*
 * Helper class for Tensor::slice() method.
 *
 * This defines a range with step size.
 */

class TRange
{
  public:

    size_t start;
    size_t stop;
    size_t step;

    TRange() : start(0), stop(0), step(1) { }
    TRange(size_t _start, size_t _stop=std::numeric_limits<size_t>::max(), size_t _step=1) :
      start(_start), stop(_stop), step(_step) { }
};

/*
 * Helper class for Tensor::slice() method.
 *
 * This means all elements of the corresponding dimension.
 */

class TAll : public TRange
{
  public:

    TAll() : TRange(0) { }
};

/*
 * Helper class for Tensor::slice() method.
 *
 * This means exactly one element of the corresponding dimension.
 */

class TValue : public TRange
{
  public:

    TValue(size_t v) : TRange(v, v) { }
};

/**
 * An object of this class addresses one element of a tensor. It is initialized
 * with the shape of the tensor to support increment functionality. The index
 * can be calculated into an element position by providing a step array.
 */

class TIndex
{
  private:

    std::vector<size_t> shape_;
    std::vector<size_t> index_;

  public:

    explicit TIndex(const std::vector<size_t> &shape) : shape_(shape), index_(shape.size(), 0)
    {
      for (size_t i=0; i<shape_.size(); i++)
      {
        if (shape_[i] == 0)
        {
          throw std::invalid_argument("TIndex: No shape dimension must be 0");
        }
      }
    }

    /**
     * Set the size of one dimension of the shape. Negative dimensions count
     * from the back.
     */

    void setShape(int dim, size_t size)
    {
      if (size == 0)
      {
        throw std::invalid_argument("TIndex::setShape: No shape dimension must be 0");
      }

      if (dim < 0) dim=static_cast<int>(shape_.size())+dim;

      shape_[dim]=size;
    }

    const std::vector<size_t> &getShape() const { return shape_; }
    const std::vector<size_t> &getIndex() const { return index_; }

    /**
     * Reset index to zero.
     */

    void reset()
    {
      for (size_t i=0; i<index_.size(); i++) index_[i]=0;
    }

    /**
     * Sets this index to one after the last element according to the shape.
     */

    void setEnd()
    {
      if (index_.size() > 0) index_[0]=shape_[0];
      for (size_t i=1; i<index_.size(); i++) index_[i]=0;
    }

    /**
     * Sets the index of a given dimension. Negative dimensions count from the
     * back.
     */

    void set(int dim, size_t index)
    {
      if (dim < 0) dim=static_cast<int>(shape_.size())+dim;
      index_[dim]=index;
    }

    /**
     * Set the index to the given values. If the given list is shorter than
     * this index, then only the last indices are set. All higher dimensions
     * are unchanged.
     */

    void set(std::initializer_list<size_t> index)
    {
      if (index.size() > index_.size())
      {
        throw std::invalid_argument("TIndex::set: Given index is too big");
      }

      if (index.size() > 0)
      {
        size_t i=index.size();
        size_t k=index_.size();

        do
        {
          index_[--k]=index.begin()[--i];
        }
        while (i > 0 && k > 0);
      }
    }

    /**
     * Set the index to the given values. If the given list is shorter than
     * this index, then only the last indices are set. All higher dimensions
     * are unchanged.
     */

    void set(const std::vector<size_t> &index)
    {
      if (index.size() > index_.size())
      {
        throw std::invalid_argument("TIndex::set: Rank of given index too big");
      }

      if (index.size() > 0)
      {
        size_t i=index.size();
        size_t k=index_.size();

        do
        {
          index_[--k]=index[--i];
        }
        while (i > 0 && k > 0);
      }
    }

    /**
     * Increments a certain dimension by one. Negative dimensions count from
     * the back. Incrementation stops one element after the last one according
     * to the shape.
     *
     * The return value is true as long as the index is valid, i.e. has not
     * reached the end.
     */

    bool increment(int dim=-1)
    {
      if (index_.size() == 0)
      {
        return false;
      }

      if (index_[0] < shape_[0])
      {
        if (dim < 0) dim=static_cast<int>(shape_.size())+dim;

        if (dim >= 0)
        {
          while (dim >= 0)
          {
            index_[dim]++;
            if (index_[dim] >= shape_[dim] && dim > 0)
            {
              index_[dim]=0;
              dim--;
            }
            else
            {
              break;
            }
          }
        }
        else
        {
          // this happens if a negative dimension is converted to one before
          // the first real one, which means that we jump right to the end

          setEnd();
        }
      }

      return index_[0] < shape_[0];
    }

    bool operator<(const TIndex &a) const
    {
      if (index_.size() != a.index_.size())
      {
        throw std::invalid_argument("TIndex::operator<: Can only be applied to indices of same rank");
      }

      for (size_t i=0; i<index_.size(); i++)
      {
        if (index_[i] < a.index_[i]) { return true; } else if (index_[i] > a.index_[i]) { return false; }
      }

      return false;
    }

    bool operator<=(const TIndex &a) const
    {
      if (index_.size() != a.index_.size())
      {
        throw std::invalid_argument("TIndex::operator<=: Can only be applied to indices of same rank");
      }

      for (size_t i=0; i<index_.size(); i++)
      {
        if (index_[i] < a.index_[i]) { return true; } else if (index_[i] > a.index_[i]) { return false; }
      }

      return true;
    }

    bool operator>(const TIndex &a) const
    {
      if (index_.size() != a.index_.size())
      {
        throw std::invalid_argument("TIndex::operator>: Can only be applied to indices of same rank");
      }

      for (size_t i=0; i<index_.size(); i++)
      {
        if (index_[i] > a.index_[i]) { return true; } else if (index_[i] < a.index_[i]) { return false; }
      }

      return false;
    }

    bool operator>=(const TIndex &a) const
    {
      if (index_.size() != a.index_.size())
      {
        throw std::invalid_argument("TIndex::operator>=: Can only be applied to indices of same rank");
      }

      for (size_t i=0; i<index_.size(); i++)
      {
        if (index_[i] > a.index_[i]) { return true; } else if (index_[i] < a.index_[i]) { return false; }
      }

      return true;
    }

    bool operator==(const TIndex &a) const
    {
      if (index_.size() != a.index_.size())
      {
        throw std::invalid_argument("TIndex::operator==: Can only be applied to indices of same rank");
      }

      for (size_t i=0; i<index_.size(); i++)
      {
        if (index_[i] != a.index_[i]) return false;
      }

      return true;
    }

    size_t position(const std::vector<size_t> &step) const
    {
      size_t ret=0;

      for (size_t i=0; i<index_.size(); i++) ret+=index_[i]*step[i];

      return ret;
    }
};

template<class Ch, class Tr>
std::basic_ostream<Ch, Tr> &operator<<(std::basic_ostream<Ch, Tr> &out, const std::vector<size_t> &a)
{
  // write shape

  out << "{";

  for (size_t i=0; i<a.size(); i++)
  {
    out << a[i];
    if (i+1 < a.size()) out << ' ';
  }

  out << "}";

  return out;
}

/**
 * Tries to equalize the given shapes and corrects the steps accordingly.
 * Missing leading dimensions will be set to 1. Unequal dimensions will be
 * made equal by broadcasting if the lower one is 1. If successfull, the
 * shape arrays will be equal and can be used to initialize the target tensor.
 * The step arrays will be adapted so that they can be used with an index of
 * the target tensor to calculate the element position of the corresponding
 * source tensors with TIndex::position().
 */

void broadcast(std::vector<size_t> &shape1, std::vector<size_t> &step1,
  std::vector<size_t> &shape2, std::vector<size_t> &step2);

/**
 * This template defines a tensor.
 */

template<class T> class Tensor
{
  private:

    size_t rank_;
    std::vector<size_t> shape_;
    std::vector<size_t> step_;
    std::vector<T> value_;

    bool sameShape(const std::vector<size_t> &shape) const
    {
      if (rank_ != shape.size()) return false;

      for (size_t i=0; i<rank_; i++)
      {
        if (shape_[i] != shape[i]) return false;
      }

      return true;
    }

    bool equalContiguous(const Tensor<T> &t) const
    {
      if (rank_ != t.rank_) return false;

      for (size_t i=0; i<rank_; i++)
      {
        if (shape_[i] != t.shape_[i] || step_[i] != t.step_[i]) return false;
      }

      return true;
    }

  public:

    // ========== construction and reinitialization ==========

    /**
     * Allocate rank 0 tensor, i.e. scalar.
     */

    explicit Tensor(T u=0)
    {
      rank_=0;
      value_.resize(1);
      value_[0]=u;
    }

    /**
     * Allocate rank 1 tensor, i.e. vector.
     */

    Tensor(const DVector<T> &u)
    {
      rank_=1;
      shape_.resize(rank_);
      step_.resize(rank_);

      shape_[0]=static_cast<size_t>(u.size());
      step_[0]=1;

      value_.resize(shape_[0]);
      for (int i=0; i<u.size(); i++)
      {
        value_[i]=u[i];
      }
    }

    /**
     * Allocate rank 2 tensor, i.e. matrix.
     */

    Tensor(const DMatrix<T> &u)
    {
      rank_=2;
      shape_.resize(rank_);
      step_.resize(rank_);

      shape_[0]=static_cast<size_t>(u.rows());
      shape_[1]=static_cast<size_t>(u.cols());

      step_[0]=shape_[1];
      step_[1]=1;

      value_.resize(shape_[0]*shape_[1]);

      size_t j=0;
      for (int k=0; k<u.rows(); k++)
      {
        for (int i=0; i<u.cols(); i++)
        {
          value_[j++]=u(k, i);
        }
      }
    }

    Tensor(const Tensor<T> &a)
    {
      rank_=a.rank_;
      shape_=a.shape_;
      step_=a.step_;
      value_=a.value_;
    }

    Tensor(Tensor<T> &&a) noexcept : rank_(a.rank_), shape_(std::move(a.shape_)),
      step_(std::move(a.step_)), value_(std::move(a.value_))
    {
      a.rank_=0;
    }

    /**
     * Allocation of tensor with given dimensions. All elements will be set to 0.
     */

    explicit Tensor(std::initializer_list<size_t> shape)
    {
      init(shape);
    }

    explicit Tensor(const std::vector<size_t> &shape)
    {
      init(shape);
    }

    /**
     * Reinitialize the tensor with given dimensions. All elements will be set
     * to 0.
     */

    void init(size_t rank, const size_t *shape)
    {
      if (rank > 0)
      {
        rank_=rank;
        shape_.resize(rank_);
        step_.resize(rank_);

        for (size_t i=0; i<rank_; i++)
        {
          if (shape[i] == 0)
          {
            throw std::invalid_argument("Tensor::init(): Shape must not have a null element");
          }

          shape_[i]=shape[i];
        }

        step_[rank_-1]=1;
        for (size_t i=rank_-1; i>0; i--)
        {
          step_[i-1]=shape_[i]*step_[i];
        }

        value_.resize(shape_[0]*step_[0]);
        for (size_t i=0; i<value_.size(); i++)
        {
          value_[i]=0;
        }
      }
      else
      {
        rank_=0;
        shape_.clear();
        step_.clear();
        value_.resize(1);
        value_[0]=0;
      }
    }

    void init(const std::vector<size_t> &shape)
    {
      init(shape.size(), shape.data());
    }

    void init(std::initializer_list<size_t> shape)
    {
      init(shape.size(), shape.begin());
    }

    // ========== internal memory layout ==========

    /**
     * Check if current order is contiguous
     */

    bool isContiguous() const
    {
      bool contiguous=true;
      for (size_t i=1; i<rank_; i++)
      {
        if (step_[i-1] < step_[i])
        {
          contiguous=false;
          break;
        }
      }

      return contiguous;
    }

    /**
     * Reorder values to make it contiguous. This operation is expensive in
     * terms of runtime and memory, unless the tensor already is contiguous.
     */

    void makeContiguous()
    {
      if (!isContiguous())
      {
        std::vector<T> tmp(value_.size());

        size_t shape_k=shape_.back();
        size_t step_k=step_.back();

        size_t ti=0;
        std::vector<size_t> index(rank_);

        while (ti < value_.size())
        {
          // compute position of source element excluding last dimension

          size_t si=0;
          for (size_t i=0; i<rank_-1; i++)
          {
            si+=index[i]*step_[i];
          }

          // copy all elements of last dimension

          for (size_t k=0; k<shape_k; k++)
          {
            tmp[ti++]=value_[si];
            si+=step_k;
          }

          // increment previous dimension

          for (size_t i=rank_-1; i>0; i--)
          {
            index[i-1]++;
            if (index[i-1] >= shape_[i-1])
            {
              index[i-1]=0;
            }
            else
            {
              break;
            }
          }
        }

        value_=std::move(tmp);

        // recompute steps

        step_[rank_-1]=1;
        for (size_t i=rank_-1; i>0; i--)
        {
          step_[i-1]=shape_[i]*step_[i];
        }
      }
    }

    // ========== accessing size and shape ==========

    /**
     * Return rank of tensor.
     */

    size_t rank() const { return rank_; }

    /**
     * Total number of elements.
     */

    size_t size() const { return value_.size(); }

    /**
     * Returns the size of dimension i.
     */

    size_t shape(int i) const
    {
      if (i < 0)
      {
        return shape_[static_cast<int>(rank_)+i];
      }

      return shape_[i];
    }

    /**
     * Return the sizes of all dimensions.
     */

    const std::vector<size_t> &shape() const { return shape_; }

    /**
     * Return the internal steps for all dimensions.
     */

    const std::vector<size_t> &step() const { return step_; }

    // ========== modifying shape and transpose indices ==========

    /**
     * Eliminates all dimensions with size 1. This is a light weight operation
     * that just changes the shape.
     */

    Tensor &reduceRank()
    {
      // compute new rank

      size_t rank=0;
      for (size_t i=0; i<rank_; i++)
      {
        if (shape_[i] > 1)
        {
          rank++;
        }
      }

      // eliminate all dimensions with size 1

      if (rank < rank_)
      {
        std::vector<size_t> shape(rank);
        std::vector<size_t> step(rank);

        size_t k=0;
        for (size_t i=0; i<rank_; i++)
        {
          if (shape_[i] > 1)
          {
            shape[k]=shape_[i];
            step[k]=step_[i];
            k++;
          }
        }

        rank_=rank;
        shape_=std::move(shape);
        step_=std::move(step);
      }

      return *this;
    }

    /**
     * Reshape this tensor in place, with the same number of values, but with
     * potentially a different number of dimensions or elements in each
     * dimension. The size of one dimension may be negative and in this case,
     * it will be computed. The given shape must have the same number of
     * elements. Otherwise, an exception is thrown.
     */

    void setReshaped(std::initializer_list<int> shape)
    {
      // compute number of elements of new shape

      size_t n=1;
      int k=-1;
      for (size_t i=0; i<shape.size(); i++)
      {
        int s=shape.begin()[i];
        if (s > 0)
        {
          n*=static_cast<size_t>(s);
        }
        else if (s == 0)
        {
          throw std::invalid_argument("Tensor::setReshaped(): Shape must not have a null element");
        }
        else
        {
          if (k < 0)
          {
            k=static_cast<int>(i);
          }
          else
          {
            throw std::invalid_argument("Tensor::setReshaped(): Only one element can be negative");
          }
        }
      }

      // compute size of missing dimension, if any

      int n_k=0;
      if (k >= 0)
      {
        n_k=value_.size()/n;
        n*=n_k;
      }

      // ensure that new shape has the same number of elements as the old one

      if (n != value_.size())
      {
        throw std::invalid_argument("Tensor::setReshaped(): "+std::to_string(value_.size())
           +" elements expected in new shape, but "+std::to_string(n)+" defined");
      }

      // ensure that values are contiguous

      makeContiguous();

      // set new shape

      if (shape.size() > 0)
      {
        rank_=shape.size();
        shape_.resize(rank_);
        step_.resize(rank_);

        for (size_t i=0; i<rank_; i++)
        {
          if (static_cast<int>(i) != k)
          {
            shape_[i]=shape.begin()[i];
          }
          else
          {
            shape_[i]=n_k;
          }
        }

        // recompute steps

        step_[rank_-1]=1;
        for (size_t i=rank_-1; i>0; i--)
        {
          step_[i-1]=shape_[i]*step_[i];
        }
      }
    }

    /**
     * Creates a new tensor with the same number of values, but with
     * potentially a different number of dimensions or elements in each
     * dimension. The size of one dimension may be negative and in this case,
     * it will be computed. The given shape must have the same number of
     * elements. Otherwise, an exception is thrown.
     */

    Tensor<T> reshape(std::initializer_list<int> shape) const
    {
      Tensor<T> ret(*this);
      ret.setReshaped(shape);
      return ret;
    }

    /**
     * Transpose this tensor in place.
     *
     * This operation defines a new order of indices and is a light weight
     * operation. The index list must contain all indices exactly once, i.e.
     * if the rank is n, indices 0 to n-1 must be given in a list of size n.
     *
     * Indices can also be negative, which means that they are counted from
     * back, i.e. -1 is the last index.
     *
     * If the given list is shorter than the rank, then the leading elements
     * are filled with their own indices, i.e. they do not change.
     */

    void setTransposed(std::initializer_list<int> index_list)
    {
      std::vector<int> index(index_list);

      while (index.size() < rank_)
      {
        index.insert(index.begin(), static_cast<int>(rank_-index.size()-1));
      }

      if (index.size() != rank_)
      {
        throw std::invalid_argument("Tensor::setTransposed(): "+std::to_string(rank_)
          +" elements expected, but "+std::to_string(index.size())+" given");
      }

      if (rank_ > 0)
      {
        std::vector<size_t> ntmp(rank_, 0);
        std::vector<size_t> stmp(rank_, 0);

        // check if all indices are unique and not out of range

        for (size_t i=0; i<rank_; i++)
        {
          int k=index[i];
          if (k >= 0)
          {
            if (static_cast<size_t>(k) >= rank_)
            {
              throw std::invalid_argument("Tensor::setTransposed(): Index < "+std::to_string(rank_)
                +" expected, but "+std::to_string(k)+" given");
            }
          }
          else
          {
            k=static_cast<int>(rank_)+k;
            if (k < 0)
            {
              throw std::invalid_argument("Tensor::setTransposed(): Index >= "+std::to_string(-rank_)
                +" expected, but "+std::to_string(index[i])+" given");
            }
          }

          stmp[k]=1;
        }

        for (size_t i=0; i<rank_; i++)
        {
          if (stmp[i] == 0)
          {
            throw std::invalid_argument("Tensor::setTransposed(): Double use of some given indices");
          }

          ntmp[i]=shape_[i];
          stmp[i]=step_[i];
        }

        // transpose as requested

        for (size_t i=0; i<rank_; i++)
        {
          int k=index[i];
          if (k < 0)
          {
            k=static_cast<int>(rank_)+k;
          }

          shape_[i]=ntmp[k];
          step_[i]=stmp[k];
        }
      }
    }

    /**
     * Create of copy of this tensor, which is transposed.
     *
     * This operation defines a new order of indices of the returned tensor.
     * The index list must contain all indices exactly once, i.e. if the rank
     * is n, indices 0 to n-1 must be given in a list of size n.
     *
     * Indices can also be negative, which means that they are counted from
     * back, i.e. -1 is the last index.
     *
     * If the given list is shorter than the rank, then the leading elements
     * are filled with their own indices, i.e. they do not change.
     */

    Tensor<T> transpose(std::initializer_list<int> index_list) const
    {
      Tensor<T> ret(*this);
      ret.setTransposed(index_list);
      return ret;
    }

    // ========== copy part ==========

    /**
     * Extraction of a part from this tensor. The resulting tensor has the same
     * rank, unless stop < start for one of the given dimensions in which case
     * a rank 0 vector with value 0 is returned. The reduceRank() operation can
     * be used afterwards to eliminate all dimensions with size 1.
     */

    Tensor<T> slice(std::initializer_list<TRange> _range) const
    {
      if (_range.size() != rank_)
      {
        throw std::invalid_argument("Tensor::slice(): "+std::to_string(rank_)
          +" elements expected, but "+std::to_string(_range.size())+" given");
      }

      Tensor<T> ret;

      if (rank_ > 0)
      {
        // limit out of bounds maximum

        std::vector<TRange> range(rank_);

        for (size_t i=0; i<rank_; i++)
        {
          range[i]=_range.begin()[i];

          if (range[i].stop >= shape_[i]) range[i].stop=shape_[i]-1;

          if (range[i].stop < range[i].start)
          {
            range.clear();
            break;
          }
        }

        if (range.size() > 0)
        {
          // compute new shape and initialize new tensor with it

          std::vector<size_t> shape(rank_);
          for (size_t i=0; i<rank_; i++)
          {
            shape[i]=(range[i].stop-range[i].start+range[i].step)/range[i].step;
          }

          ret.init(shape.size(), shape.data());

          // copy data

          size_t r_start_k=range[rank_-1].start;
          size_t r_stop_k=range[rank_-1].stop;
          size_t r_step_k=range[rank_-1].step;
          size_t step_k=step_[rank_-1];

          size_t ti=0;
          std::vector<size_t> index(rank_);

          for (size_t i=0; i<rank_; i++)
          {
            index[i]=range[i].start;
          }

          while (ti < ret.value_.size())
          {
            // compute position of source element excluding last dimension

            size_t si=0;
            for (size_t i=0; i<rank_-1; i++)
            {
              si+=index[i]*step_[i];
            }

            // copy all elements of last dimension

            for (size_t k=r_start_k; k<=r_stop_k; k+=r_step_k)
            {
              ret.value_[ti++]=value_[si];
              si+=step_k*r_step_k;
            }

            // increment previous dimension

            for (size_t i=rank_-1; i>0; i--)
            {
              index[i-1]+=range[i-1].step;
              if (index[i-1] > range[i-1].stop)
              {
                index[i-1]=range[i-1].start;
              }
              else
              {
                break;
              }
            }
          }
        }
      }

      return ret;
    }

    // ========== copy and move operators ==========

    Tensor<T> &operator=(T u)
    {
      rank_=0;
      shape_.clear();
      step_.clear();

      value_.resize(1);
      value_[0]=u;

      return *this;
    }

    Tensor<T> &operator=(const DVector<T> &u)
    {
      rank_=1;
      shape_.resize(rank_);
      step_.resize(rank_);

      shape_[0]=static_cast<size_t>(u.size());
      step_[0]=1;

      value_.resize(u.size());
      for (int i=0; i<u.size(); i++)
      {
        value_[i]=u[i];
      }

      return *this;
    }

    Tensor<T> &operator=(const DMatrix<T> &u)
    {
      rank_=2;
      shape_.resize(rank_);
      step_.resize(rank_);

      shape_[0]=static_cast<size_t>(u.rows());
      shape_[1]=static_cast<size_t>(u.cols());

      step_[0]=shape_[1];
      step_[1]=1;

      value_.resize(shape_[0]*shape_[1]);

      size_t j=0;
      for (int k=0; k<u.rows(); k++)
      {
        for (int i=0; i<u.cols(); i++)
        {
          value_[j++]=u(k, i);
        }
      }

      return *this;
    }

    Tensor<T> &operator=(const Tensor &t)
    {
      rank_=t.rank_;
      shape_=t.shape_;
      step_=t.step_;
      value_=t.value_;

      return *this;
    }

    Tensor<T> &operator=(Tensor &&t) noexcept
    {
      rank_=t.rank_;
      shape_=std::move(t.shape_);
      step_=std::move(t.step_);
      value_=std::move(t.value_);

      return *this;
    }

    // ========== casting into vector and matrix ==========

    operator DVector<T>() const
    {
      if (rank_ != 1)
      {
        throw std::invalid_argument("Tensor: Cannot cast tensor of rank "
          +std::to_string(rank_)+" to vector.");
      }

      DVector<T> v(static_cast<int>(shape_[0]));

      for (size_t i=0; i<shape_[0]; i++)
      {
        v[i]=value_[i];
      }

      return v;
    }

    operator DMatrix<T>() const
    {
      if (rank_ != 2)
      {
        throw std::invalid_argument("Tensor: Cannot cast tensor of rank "
          +std::to_string(rank_)+" to matrix.");
      }

      DMatrix<T> M(static_cast<int>(shape_[0]), static_cast<int>(shape_[1]));

      size_t j=0;
      for (size_t k=0; k<shape_[0]; k++)
      {
        for (size_t i=0; i<shape_[1]; i++)
        {
          M(k, i)=value_[j++];
        }
      }

      return M;
    }

    // ========== comparison ==========

    bool operator==(const Tensor<T> &a) const
    {
      if (!sameShape(a.shape())) return false;

      if (equalContiguous(a))
      {
        for (size_t i=0; i<value_.size(); i++) if (value_[i] != a.value_[i]) return false;
      }
      else
      {
        TIndex index(shape_);

        do
        {
          if (at(index.position(step())) != a.at(index.position(a.step()))) return false;
        }
        while (index.increment());
      }

      return true;
    }

    bool operator!=(const Tensor<T> &a) const
    {
      return !(*this == a);
    }

    // ========== element access ==========

    /**
     * Returns an element at a certain position. The position can be calculated
     * from shape and step using a TIndex object.
     */

    T &at(size_t position)
    {
      return value_[position];
    }

    const T &at(size_t position) const
    {
      return value_[position];
    }

    /**
     * Accessing one element of the tensor. The given list must have the same
     * size as the rank of this tensor. And the coordinates must not be out
     * of bounds.
     */

    T &operator()(std::initializer_list<size_t> index)
    {
#ifndef NDEBUG
      // this is only checked when compiled as debug to save time for accessing
      // individual elements

      if (index.size() != rank_)
      {
        throw std::invalid_argument("Tensor::operator(): "+std::to_string(rank_)
          +" elements expected, but "+std::to_string(index.size())+" given");
      }

      for (size_t i=0; i<rank_; i++)
      {
        if (index.begin()[i] < 0 || index.begin()[i] >= shape_[i])
        {
          throw std::invalid_argument("Tensor::operator(): Out of bounds. Element "
            +std::to_string(index.begin()[i])+" >= "+std::to_string(shape_[i]));
        }
      }
#endif

      size_t k=0;
      for (size_t i=0; i<rank_; i++)
      {
        k+=index.begin()[i]*step_[i];
      }

      return value_[k];
    }

    const T &operator()(std::initializer_list<size_t> index) const
    {
#ifndef NDEBUG
      // this is only checked when compiled as debug to save time for accessing
      // individual elements

      if (index.size() != rank_)
      {
        throw std::invalid_argument("Tensor::operator(): "+std::to_string(rank_)
          +" elements expected, but "+std::to_string(index.size())+" given");
      }

      for (size_t i=0; i<rank_; i++)
      {
        if (index.begin()[i] < 0 || index.begin()[i] >= shape_[i])
        {
          throw std::invalid_argument("Tensor::operator(): Out of bounds. Element "
            +std::to_string(index.begin()[i])+" >= "+std::to_string(shape_[i]));
        }
      }
#endif

      size_t k=0;
      for (size_t i=0; i<rank_; i++)
      {
        k+=index.begin()[i]*step_[i];
      }

      return value_[k];
    }

    // ========== arithmetic with broadcasting ==========

    Tensor<T> operator+(const Tensor<T> &a) const
    {
      std::vector<size_t> shape1(shape_), step1(step_), shape2(a.shape_), step2(a.step_);
      broadcast(shape1, step1, shape2, step2);

      Tensor<T> ret(shape1);
      TIndex index(shape1);

      do
      {
        ret.at(index.position(ret.step_)) = value_[index.position(step1)] + a.at(index.position(step2));
      }
      while (index.increment());

      return ret;
    }

    Tensor<T> &operator+=(const Tensor<T> &a)
    {
      std::vector<size_t> shape1(shape_), step1(step_), shape2(a.shape_), step2(a.step_);
      broadcast(shape1, step1, shape2, step2);

      TIndex index(shape1);

      if (sameShape(shape1))
      {
        // in place operation if shape has not changed due to broadcasting

        do
        {
          value_[index.position(step1)] += a.at(index.position(step2));
        }
        while (index.increment());
      }
      else
      {
        Tensor<T> ret(shape1);

        do
        {
          ret.at(index.position(ret.step_)) = value_[index.position(step1)] + a.at(index.position(step2));
        }
        while (index.increment());

        *this=std::move(ret);
      }

      return *this;
    }

    Tensor<T> operator-(const Tensor<T> &a) const
    {
      std::vector<size_t> shape1(shape_), step1(step_), shape2(a.shape_), step2(a.step_);
      broadcast(shape1, step1, shape2, step2);

      Tensor<T> ret(shape1);
      TIndex index(shape1);

      do
      {
        ret.at(index.position(ret.step_)) = value_[index.position(step1)] - a.at(index.position(step2));
      }
      while (index.increment());

      return ret;
    }

    Tensor<T> &operator-=(const Tensor<T> &a)
    {
      std::vector<size_t> shape1(shape_), step1(step_), shape2(a.shape_), step2(a.step_);
      broadcast(shape1, step1, shape2, step2);

      TIndex index(shape1);

      if (sameShape(shape1))
      {
        // in place operation if shape has not changed due to broadcasting

        do
        {
          value_[index.position(step1)] -= a.at(index.position(step2));
        }
        while (index.increment());
      }
      else
      {
        Tensor<T> ret(shape1);

        do
        {
          ret.at(index.position(ret.step_)) = value_[index.position(step1)] - a.at(index.position(step2));
        }
        while (index.increment());

        *this=std::move(ret);
      }

      return *this;
    }

    Tensor<T> operator*(const Tensor<T> &a) const
    {
      std::vector<size_t> shape1(shape_), step1(step_), shape2(a.shape_), step2(a.step_);
      broadcast(shape1, step1, shape2, step2);

      Tensor<T> ret(shape1);
      TIndex index(shape1);

      do
      {
        ret.at(index.position(ret.step_)) = value_[index.position(step1)] * a.at(index.position(step2));
      }
      while (index.increment());

      return ret;
    }

    Tensor<T> &operator*=(const Tensor<T> &a)
    {
      std::vector<size_t> shape1(shape_), step1(step_), shape2(a.shape_), step2(a.step_);
      broadcast(shape1, step1, shape2, step2);

      TIndex index(shape1);

      if (sameShape(shape1))
      {
        // in place operation if shape has not changed due to broadcasting

        do
        {
          value_[index.position(step1)] *= a.at(index.position(step2));
        }
        while (index.increment());
      }
      else
      {
        Tensor<T> ret(shape1);

        do
        {
          ret.at(index.position(ret.step_)) = value_[index.position(step1)] * a.at(index.position(step2));
        }
        while (index.increment());

        *this=std::move(ret);
      }

      return *this;
    }

    Tensor<T> operator/(const Tensor<T> &a) const
    {
      std::vector<size_t> shape1(shape_), step1(step_), shape2(a.shape_), step2(a.step_);
      broadcast(shape1, step1, shape2, step2);

      Tensor<T> ret(shape1);
      TIndex index(shape1);

      do
      {
        ret.at(index.position(ret.step_)) = value_[index.position(step1)] / a.at(index.position(step2));
      }
      while (index.increment());

      return ret;
    }

    Tensor<T> &operator/=(const Tensor<T> &a)
    {
      std::vector<size_t> shape1(shape_), step1(step_), shape2(a.shape_), step2(a.step_);
      broadcast(shape1, step1, shape2, step2);

      TIndex index(shape1);

      if (sameShape(shape1))
      {
        // in place operation if shape has not changed due to broadcasting

        do
        {
          value_[index.position(step1)] /= a.at(index.position(step2));
        }
        while (index.increment());
      }
      else
      {
        Tensor<T> ret(shape1);

        do
        {
          ret.at(index.position(ret.step_)) = value_[index.position(step1)] / a.at(index.position(step2));
        }
        while (index.increment());

        *this=std::move(ret);
      }

      return *this;
    }

    // ========== apply to each element ==========

    /**
     * Applies the given function to all elements.
     */

    void apply(std::function<T(T)> fct)
    {
      for (size_t i=0; i<value_.size(); i++)
      {
        value_[i]=fct(value_[i]);
      }
    }

    // ========== min, max, sum and average over all values ==========

    T min() const
    {
      T v=value_[0];

      for (size_t i=1; i<value_.size(); i++) v=std::min(v, value_[i]);

      return v;
    }

    T max() const
    {
      T v=value_[0];

      for (size_t i=1; i<value_.size(); i++) v=std::max(v, value_[i]);

      return v;
    }

    T sum() const
    {
      T v=0;

      for (size_t i=0; i<value_.size(); i++) v+=value_[i];

      return v;
    }

    T average() const
    {
      return sum()/value_.size();
    }

    // ========== min, max, sum and average over given dimension ==========

    /**
     * Minimum over given dimension. Negative dimensions are counted from back.
     * The returned tensor has the same shape, except that the size of the
     * given dimension is set to 1.
     */

    Tensor<T> min(int dim) const
    {
      if (dim < 0) dim=static_cast<int>(rank_)+dim;

      std::vector<size_t> shape(shape_);
      shape[dim]=1;

      Tensor<T> ret(shape);
      TIndex index(shape);

      do
      {
        T v=value_[index.position(step_)];

        for (size_t i=1; i<shape_[dim]; i++)
        {
          index.set(dim, i);
          v=std::min(v, value_[index.position(step_)]);
        }

        index.set(dim, 0);
        ret.at(index.position(ret.step()))=v;
      }
      while (index.increment());

      return ret;
    }

    /**
     * Maximum over given dimension. Negative dimensions are counted from back.
     * The returned tensor has the same shape, except that the size of the
     * given dimension is set to 1.
     */

    Tensor<T> max(int dim) const
    {
      if (dim < 0) dim=static_cast<int>(rank_)+dim;

      std::vector<size_t> shape(shape_);
      shape[dim]=1;

      Tensor<T> ret(shape);
      TIndex index(shape);

      do
      {
        T v=value_[index.position(step_)];

        for (size_t i=1; i<shape_[dim]; i++)
        {
          index.set(dim, i);
          v=std::max(v, value_[index.position(step_)]);
        }

        index.set(dim, 0);
        ret.at(index.position(ret.step()))=v;
      }
      while (index.increment());

      return ret;
    }

    /**
     * Sum over given dimension. Negative dimensions are counted from back.
     * The returned tensor has the same shape, except that the size of the
     * given dimension is set to 1.
     */

    Tensor<T> sum(int dim) const
    {
      if (dim < 0) dim=static_cast<int>(rank_)+dim;

      std::vector<size_t> shape(shape_);
      shape[dim]=1;

      Tensor<T> ret(shape);
      TIndex index(shape);

      do
      {
        T v=value_[index.position(step_)];

        for (size_t i=1; i<shape_[dim]; i++)
        {
          index.set(dim, i);
          v+=value_[index.position(step_)];
        }

        index.set(dim, 0);
        ret.at(index.position(ret.step()))=v;
      }
      while (index.increment());

      return ret;
    }

    /**
     * Average over given dimension. Negative dimensions are counted from back.
     * The returned tensor has the same shape, except that the size of the
     * given dimension is set to 1.
     */

    Tensor<T> average(int dim) const
    {
      return sum(dim)/Tensor<T>(shape(dim));
    }
};

/**
 * Multiplication of two tensors according to the rules of matrix multiplication.
 *
 * If the second operand is a vector, then it is implicitely transposed.
 *
 * The shapes are equalized by broadcasting.
 */

template<class T>
Tensor<T> matmul(const Tensor<T> &a, const Tensor<T> &b)
{
  if (a.rank() == 0 || b.rank() == 0)
  {
    throw std::invalid_argument("Tensor::matmul: Rank must be > 0: "+std::to_string(a.rank())
      +" "+std::to_string(b.rank()));
  }

  std::vector<size_t> shape1(a.shape()), step1(a.step()), shape2(b.shape()), step2(b.step());

  if (shape1.size() == 1)
  {
    // if first operand is vector, extend to matrix with one line

    shape1.insert(shape1.begin(), 1);
    step1.insert(step1.begin(), 0);
  }

  if (shape2.size() == 1)
  {
    // if second operand is vector, extend to matrix with one column

    shape2.push_back(1);
    step2.push_back(0);
  }

  size_t am=shape1[shape1.size()-2];
  size_t an=shape1[shape1.size()-1];
  size_t bm=shape2[shape2.size()-2];
  size_t bn=shape2[shape2.size()-1];

  if (an != bm)
  {
    throw std::invalid_argument("Tensor::matmul: Incompatible shapes: "+
      std::to_string(an)+" != "+std::to_string(bm));
  }

  // change last two dimensions to target shape (broadcasting is not changing
  // it when they are equal)

  shape2[shape2.size()-2]=am;
  shape1[shape1.size()-1]=bn;

  // equalize both shapes

  broadcast(shape1, step1, shape2, step2);

  // perform matrix multiplication using last two dimensions and repeating that
  // for all higher dimensions

  Tensor<T> ret(shape1);
  TIndex index(shape1);

  do
  {
    for (size_t k=0; k<am; k++)
    {
      for (size_t i=0; i<bn; i++)
      {
        T v=0;
        for (size_t j=0; j<an; j++)
        {
          index.set({k, j});
          T av=a.at(index.position(step1));

          index.set({j, i});
          T bv=b.at(index.position(step2));

          v+=av*bv;
        }

        index.set({k, i});
        ret.at(index.position(ret.step()))=v;
      }
    }
  }
  while (index.increment(-3)); // incrementing higher dimensions until end

  return ret;
}

// ========== input / output operations ==========

template<class T, class Ch, class Tr>
std::basic_ostream<Ch, Tr> &operator<<(std::basic_ostream<Ch, Tr> &out, const Tensor<T> &a)
{
  // write shape

  out << "{";

  for (size_t i=0; i<a.rank(); i++)
  {
    out << a.shape(i);
    if (i+1 < a.rank()) out << ' ';
  }

  out << "}[";

  // write values

  if (a.isContiguous())
  {
    for (size_t i=0; i<a.size(); i++)
    {
      out << a.at(i);
      if (i+1 < a.size()) out << ' ';
    }
  }
  else
  {
    Tensor<T> t(a);
    t.makeContiguous();

    for (size_t i=0; i<t.size(); i++)
    {
      out << t.at(i);
      if (i+1 < t.size()) out << ' ';
    }
  }

  out << "]";

  return out;
}

template<class T, class Ch, class Tr>
std::basic_istream<Ch, Tr> &operator>>(std::basic_istream<Ch, Tr> &in, Tensor<T> &a)
{
  // read shape definition

  size_t              sv;
  std::vector<size_t> shape;
  char                c;

  in >> c;
  if (c == '{')
  {
    while (c != '}' && in)
    {
      in >> c;

      if (c != '}')
      {
        in.putback(c);

        in >> sv;
        shape.push_back(sv);
      }
    }

    // initialize

    a.init(shape);

    // read values

    T v;

    in >> c;
    if (c == '[')
    {
      size_t n=0;
      while (in && n < a.size())
      {
        in >> v;
        a.at(n++)=v;
      }

      in >> c;
      if (c != ']' || n != a.size())
      {
        in.setstate(std::ios_base::failbit);
      }
    }
  }
  else
  {
    in.setstate(std::ios_base::failbit);
  }

  return in;
}

typedef Tensor<double> Tensord;
typedef Tensor<float> Tensorf;

}

#endif
