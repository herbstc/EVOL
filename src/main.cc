

#include <thread>
#include "ros/ros.h"
#include "dvs_msgs/EventArray.h"

#include "event_tracker/elised.hpp" //#include "event_tracker/elised.hpp"
#include <aux/tic_toc.hpp>


#include <iostream>
#include <fstream>



#define ROWS 128
#define COLS 128

event_tracker::Elised<ROWS,COLS>* elised_;

void EmptyClusterMaintenance(ros::Duration time_delay) {
  while(ros::ok()) {
    elised_->FreeEmptyFeature();
    time_delay.sleep();
  }
}

std::ofstream log_file;

void RenderElised(ros::Duration time_delay) {
  while(ros::ok()) {
    elised_->RenderLines(1);    
    elised_->LogClusters(1,log_file);    
    // elised_->Render(1);
    time_delay.sleep();
  }
}

void EventArrayCallback(const dvs_msgs::EventArrayConstPtr & event_array_msg)
{

//  std::vector<uint64_t> time_vec(event_array_msg->events.size());
//  std::cout << std::endl;
//  uint64_t time=0, time_current = 0, time_max = 0;
//  std::cout << "EventArray message received (" << event_array_msg->events.size() << ")" <<std::endl;
//  tic();
  tic("nsec");
  uint64_t last_time = 0;
  for (size_t i = 0; i<event_array_msg->events.size(); i++){
    // for all events ins message
    const dvs_msgs::Event event = event_array_msg->events[i];

    assert(event.ts.toNSec()>=last_time);
    last_time = event.ts.toNSec();

//    tic("nsec");
    elised_->AddEvent(event.x, event.y, event.ts.toNSec(), true);
//    log_file << toc("muted") <<  std::endl;


  }
  // elised_->GetCurrentLine(); // ToDo(herbstc): Improve shoddy coding
  uint64_t time_event_array =  toc("muted");
  std::cout << "Mean:" <<time_event_array/event_array_msg->events.size() << " (" << event_array_msg->events.size() << ") RT: "<< 30e6 / time_event_array<< std::endl;
}


int main(int argc, char **argv)
{
 assert(0);
  ros::init(argc, argv, "event_tracker_node");
  ros::NodeHandle n;

  log_file.open ("log_file.txt");

  ros::Subscriber sub = n.subscribe("dvs/events", 0, EventArrayCallback);

  elised_ = new event_tracker::Elised<ROWS,COLS>();
  std::cout << "about to start render" << std::endl;
  std::thread render_thread(RenderElised, ros::Duration(0.01));
  std::thread maintenance_thread(EmptyClusterMaintenance, ros::Duration(0.03));

//  while(ros::ok()){
//    ros::spinOnce();
//  }

  ros::spin();



  delete elised_;
  log_file.close();
  return 0;
}

