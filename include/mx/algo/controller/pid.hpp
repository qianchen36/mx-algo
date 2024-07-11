/**
 * @file    pid.hpp
 * @author  Morthine Xiang (xiang@morthine.com)
 * @brief   PID controller
 * @version 1.0
 * @date    2024-07-11
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef MX_ALGO_CONTROLLER_PID_HPP
#define MX_ALGO_CONTROLLER_PID_HPP

#include "mx_algo_defs.hpp"

namespace mx::algo {

/**
 * @brief PID controller class
 * 
 * @tparam _T Calculations type
 * @tparam _Freq Controller frequency
 */
template<typename _T, _T _Freq = 1000>
class PIDController {
public:

  PIDController(_T kp, _T ki, _T kd, _T deadband = 0, _T max_int = 0, _T max_out = 0, _T min_int = -max_int, _T min_out = -max_out)
    : kp_(kp), ki_(ki), kd_(kd), deadband_(deadband), max_int_(max_int), min_int_(min_int), max_out_(max_out), min_out_(min_out) {
    reset();
  }

  inline _T operator()(_T err) { return update(err); }

  inline _T update(_T err) {
    // Deadband
    if (std::abs(err) < deadband_) err = 0;
    // Integral
    buffer_[INT] += err / _Freq;
    if (max_int_ != 0 && min_int_ != 0)
      buffer_[INT] = std::clamp(buffer_[INT], min_int_, max_int_);
    // Derivative
    buffer_[DIV] = (err - buffer_[LST_ERR]) * _Freq;
    // Output
    buffer_[P_OUT] = kp_ * err;
    buffer_[I_OUT] = ki_ * buffer_[INT];
    buffer_[D_OUT] = kd_ * buffer_[DIV];
    _T out = buffer_[P_OUT] + buffer_[I_OUT] + buffer_[D_OUT];
    if (max_out_ != 0 && min_out_ != 0)
      out = std::clamp(out, min_out_, max_out_);
    // Update last error
    buffer_[LST_ERR] = err;
    return out;
  }

  inline void reset() { buffer_.fill(0); }

  inline void setGains(_T kp, _T ki, _T kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
  }

  inline void setDeadband(_T deadband) { deadband_ = deadband; }

  inline void setIntegralLimits(_T max_int, _T min_int = -max_int) {
    max_int_ = max_int;
    min_int_ = min_int;
  }

  inline void setOutputLimits(_T max_out, _T min_out = -max_out) {
    max_out_ = max_out;
    min_out_ = min_out;
  }

private:

  enum eBufferIndex {
    P_OUT = 0,
    I_OUT,
    D_OUT,
    INT,
    DIV,
    LST_ERR,
  };

  _T kp_, ki_, kd_;
  _T deadband_;
  _T max_int_, min_int_;
  _T max_out_, min_out_;

  std::array<_T, 6> buffer_;

};

} // namespace mx:algo

#endif // MX_ALGO_CONTROLLER_PID_HPP
