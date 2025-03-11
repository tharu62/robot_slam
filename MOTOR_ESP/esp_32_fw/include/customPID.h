#ifndef PID_LIB_H
#define PID_LIB_H

#include <iostream>

#ifndef NDEBUG
#define LOG(x) std::cerr << x << std::endl
#else
#define LOG(x)
#endif

template <typename T>
int sign(T x){
    if(x > 0){
        return 1;
    }else if(x < 0){
        return -1;
    }else{
        return 0;
    }
}

/**
 * @brief PID Controller class
 * @tparam T Type of the PID controller tipically double
 * @tparam U Type of the target value tipically double
 */
template <typename T, typename U>
class PID{

public:

    PID(T kp, T ki, T kd, U target): kp_(kp), ki_(ki), kd_(kd), limit_min(0), limit_max(0),
        proportional_(0), integral_(0), derivative_(0), target_(target), error_(target), error_old(0), pid_(0)
    {
        LOG("PID Controller Initialized");
        LOG("kp: " << kp_ << " ki: " << ki_ << " kd: " << kd_ << " target: " << target_);
    }

    void setTarget(U target)
    {
        target_ = target;
    }

    void setLimit(T limit)
    {
        limit_max = limit;
    }
    
    T compute(U input, T time_interval=0.1)
    {

        error_ = (T)(target_ - input);

        proportional_ = kp_ * error_;

        integral_    += ki_ * error_ * time_interval;

        derivative_   = kd_ * (error_ - error_old) / time_interval;

        error_old = error_;
        
        pid_ = (proportional_ + integral_ + derivative_ );

        if(pid_ > limit_max){
            pid_ = limit_max;
        }
        else if(pid_ < -limit_max){
            pid_ = -limit_max;
        }

        if(pid_ < limit_min && pid_ > -limit_min){
            pid_ = 0;
        }

        return pid_;
    }

    const T getError(){
        return error_;
    }

    ~PID(){ }

private:

    T kp_;      // Proportional term
    T ki_;      // Integral term
    T kd_;      // Derivative term

    // PID terms
    T proportional_;
    T integral_;
    T derivative_;

    T pid_;

    // PID value limit
    T limit_max;
    T limit_min;
    
    // Target value to reach
    U target_;

    // Error terms
    T error_;
    T error_old;

};

#endif // PID_LIB_H