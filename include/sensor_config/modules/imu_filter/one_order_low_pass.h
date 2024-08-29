/*******************************************************************************
 *   Copyright (C) 2023 CASIA. All rights reserved.
 *
 *   @Filename: one_order_low_pass.h
 *
 *   @Author: shun li
 *
 *   @Email: shun.li.at.casia@outlook.com
 *
 *   @Date: 29/08/2024
 *
 *   @Description:
 *
 *******************************************************************************/

#ifndef _ONE_ORDER_LOW_PASS_H_
#define _ONE_ORDER_LOW_PASS_H_

template <typename T>
class OneOrderLowPassFilter {
 public:
  OneOrderLowPassFilter(float gain) : gain_(gain) {
    if (gain_ > 1.0)
      gain_ = 1.0;
    else if (gain_ < 0.0)
      gain_ = 0.0;

    last_output_ = T();
  }

  T GetOutput(const T& input) {
    T output = gain_ * input + (1 - gain_) * last_output_;
    return output;
  }

 private:
  float gain_;
  T last_output_;
};

#endif
