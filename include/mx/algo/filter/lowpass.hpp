/**
 * @file    lowpass.hpp
 * @author  Morthine Xiang (xiang@morthine.com)
 * @brief   Low-pass filter
 * @version 1.0
 * @date    2024-07-11
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef MX_ALGO_FILTER_LOWPASS_HPP
#define MX_ALGO_FILTER_LOWPASS_HPP

#include "mx_algo_defs.hpp"

namespace mx::algo {

/**
 * @brief Low-pass filter class
 * 
 * @tparam _T Calculation type
 */
template <typename _T>
class LowPassFilter {
public:

  LowPassFilter(_T alpha) : alpha_(alpha) {
    reset();
  }

  inline _T operator()(_T val) { return update(val); }

  inline _T update(_T val) { return alpha_ * val + (1 - alpha_) * last_; }

  inline void reset(_T val = 0) { last_ = val; }

private:

  _T alpha_ = 0;
  _T last_ = 0

};

} // namespace mx:algo

#endif // MX_ALGO_FILTER_LOWPASS_HPP
