/*
 * encoder.hpp
 *
 *  Created on: Jul 25, 2022
 *      Author: Shao Yuan
 */

#ifndef ENCODER_HPP_
#define ENCODER_HPP_

class Encoder {
  private:
    TIM_HandleTypeDef timer_handle_;

    uint32_t counts_per_revolution_; // 4 triggers on both the rising/falling edges for both inputs per count.

    bool is_active_ = false;

    uint32_t timer_period_ = 0;

    uint32_t prev_count_ = 0;       //Stores the previous timer counter value
    uint32_t prev_time_millis_ = 0;          //Stores the difference in between the previous and current timer counter values

  public:
    Encoder() = delete;

    // Constructor for encoder driver
    // This constructor also activates the encoder.
    // timer_handle - Timer must be configured in encoder mode with T1 and T2.
    //                period should be set to max value.
    // counts_per_revolution  - 4 triggers on both the rising/falling edges for both inputs per count.
    Encoder(TIM_HandleTypeDef timer_handle,
            uint32_t counts_per_revolution) :
              timer_handle_(timer_handle),
              counts_per_revolution_(counts_per_revolution){
      timer_period_ = __HAL_TIM_GET_AUTORELOAD(&timer_handle_);
      activate();
    }

    ~Encoder(){
      if (is_active_)
        deactivate();
    };

    void activate();
    void deactivate();
    int32_t get_position();
    double get_rpm();
    void reset();
};



#endif /* ENCODER_HPP_ */
