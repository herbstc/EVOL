/* Copyright (C) Ignacio Alzugaray, 2017 <aignacio at student dot ethz dot ch>
 * ETH Zurich - V4RL
 */
#ifndef EVENT_LOGGER_H
#define EVENT_LOGGER_H

#include<stdint.h>
#include<Eigen/Core>
#include "../event_tracker/common.hpp"

namespace event_tracker {
template <uint16_t ROWS, uint16_t COLS>
class EventLogger {
 public:
  inline bool AddEvent(const uint16_t& x, const uint16_t& y,
                       const uint64_t& timestamp, const bool &polarity);

 protected:
  EventLogger();
  Eigen::Array<EventTime, ROWS, COLS> time_array_;
  Eigen::Array<bool, ROWS, COLS> polarity_array_;
//  Eigen::Array<EventKey*, ROWS, COLS> key_array_;

};

} // namespace event_tracker
#endif // EVENT_LOGGER_H
