/**
 * @file    average.hpp
 * @author  Morthine Xiang (xiang@morthine.com)
 * @brief   Average filter
 * @version 1.0
 * @date    2024-07-11
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef MX_ALGO_FILTER_AVERAGE_HPP
#define MX_ALGO_FILTER_AVERAGE_HPP

#include "mx_algo_defs.hpp"

namespace mx::algo {

/**
 * @brief Average filter class
 * 
 * @tparam _T Calculation type
 * @tparam _n Buffer size
 */
template <typename _T, size_t _n>
class AverageFilter {
public:

  AverageFilter(_T val) {
    reset(val);
  }

  inline _T operator()(_T val) {
    return update(val);
  }

  inline _T update(_T val) {
    std::copy(buffer_.begin() + 1, buffer_.end(), buffer_.begin());
    buffer_[_n - 1] = val;
    return std::accumulate(buffer_.begin(), buffer_.end(), 0) / _n;
  }

  inline void reset(_T val = 0) {
    buffer_.fill(val);
  }

private:

  std::array<_T, _n> buffer_;

};

} // namespace mx::algo

#endif // MX_ALGO_FILTER_AVERAGE_HPP
