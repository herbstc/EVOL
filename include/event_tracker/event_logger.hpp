/* Copyright (C) Ignacio Alzugaray, 2017 <aignacio at student dot ethz dot ch>
 * ETH Zurich - V4RL
 */

#ifndef EVENT_LOGGER_HPP
#define EVENT_LOGGER_HPP
#include "../event_tracker/event_logger.h" // #include<event_tracker/event_logger.h>

namespace event_tracker {
#define TEMPT template <uint16_t ROWS, uint16_t COLS>

TEMPT
EventLogger<ROWS,COLS>::EventLogger() {
  std::cout << "EventLogger Constructor" << std::endl;
//  // Initialize Feature Info Array
//  for (size_t x = 0; x < ROWS; x++) {
//    for (size_t y = 0; y < COLS; y++){
//      key_array_(x,y) = new EventKey{x,y};
//    }
//  }
}

TEMPT
inline bool EventLogger<ROWS,COLS>::AddEvent(const uint16_t& x,
  const uint16_t& y, const uint64_t& timestamp, const bool &polarity) {
  time_array_(x,y) = (int64_t) timestamp;
  polarity_array_(x,y) = polarity;
  return true;
}


#undef TEMPT
} // namespace event_tracker
#endif // EVENT_LOGGER_HPP
