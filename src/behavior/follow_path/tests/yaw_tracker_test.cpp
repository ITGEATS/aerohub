#include <cassert>
#include <cmath>
#include "uav_follow_path_execution/yaw_tracker.hpp"
using uav_follow_path_execution::YawTracker;
int main() {
  YawTracker yaw;yaw.start("task",0,1.57,2,0,1.5,1);
  double last=0,rate=0;
  for(int i=1;i<=75;++i) {
    const double a=yaw.step(.02,i*.02), v=YawTracker::wrap(a-last)/.02;
    assert(std::abs(v)<=1.5+1e-8);assert(std::abs(v-rate)<=4*.02+1e-8);
    last=a;rate=v;
  }
  assert(std::abs(YawTracker::wrap(last-1.57))<.1);
  const auto before=yaw.angle;
  yaw.update("task",2,-1.,3.,1.5);assert(yaw.angle==before);
  for(auto owner:{"wrong","task"}) {
    bool rejected=false;try{yaw.update(owner,2,0,3,1.5);}catch(...){rejected=true;}
    assert(rejected);
  }
  bool expired=false;try{yaw.step(.02,4);}catch(...){expired=true;}assert(expired);
  yaw.start("other",3.13,-3.13,2,0,1.5,1);
  assert(YawTracker::wrap(yaw.step(.02,.02)-3.13)>0);
  yaw.reset();assert(!yaw.active);
}
