//
// Created by wiselook on 7/4/25.
//
#pragma once

#include <iostream>
#include <Eigen/Dense>
#include "eigen_types.h"

template <typename T>
class LQRController
{
public:
  LQRController(const int& STATE_DIM, const int& CONTROL_DIM) : STATE_DIM_(STATE_DIM), CONTROL_DIM_(CONTROL_DIM)
  {
    x_ = DMat<T>::Zero(STATE_DIM_, 1);
    u_ = DMat<T>::Zero(CONTROL_DIM_, 1);
    k_ = DMat<T>::Zero(CONTROL_DIM_, STATE_DIM_);
    x_.setZero();
    u_.setZero();
    k_.setZero();
  }

  void setK(const DMat<T>& k)
  {
    if (k.rows() != CONTROL_DIM_ || k.cols() != STATE_DIM_)
    {
      std::cout << "k.rows() != CONTROL_DIM_ or k.cols() != STATE_DIM_" << std::endl;
      return;
    }
    k_ = k;
  }

  void setTarget(const DMat<T>& target)
  {
    if (target.rows() != STATE_DIM_ || target.cols() != 1)
    {
      std::cout << "target.rows() != STATE_DIM_ or target.cols() != 1" << std::endl;
      return;
    }
    target_ = target;
  }

  void input(const DMat<T>& x)
  {
    if (x.rows() != STATE_DIM_ || x.cols() != 1)
    {
      std::cout << "x.rows() != STATE_DIM_ or x.cols() != 1" << std::endl;
      return;
    }
    x_ = x;
  }

  DMat<T> output()
  {
    if (k_.rows() != CONTROL_DIM_ || k_.cols() != STATE_DIM_)
    {
      std::cout << "k_.rows() != CONTROL_DIM_ or k_.cols() != STATE_DIM_" << std::endl;
    }
    if (x_.rows() != STATE_DIM_ || x_.cols() != 1)
    {
      std::cout << "x_.rows() != STATE_DIM_ or x_.cols() != 1" << std::endl;
    }
    u_ = k_ * (target_ - x_);
    return u_;
  }

private:
  const int STATE_DIM_{}, CONTROL_DIM_{};
  DMat<T> x_, u_, k_, target_;
};