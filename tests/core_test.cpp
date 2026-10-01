#include "hk_camera_driver/core.hpp"
#include <atomic>
#include <iostream>
#include <limits>
#include <thread>

static void expect(bool ok,const char *message){if(!ok)throw std::runtime_error(message);}
int main() {
  hk::Settings s;hk::validate(s);
  s.fps=0;hk::validate(s); // Explicit unlimited-camera request is valid.
  s.exposure=std::numeric_limits<double>::quiet_NaN();bool rejected=false;
  try{hk::validate(s);}catch(const std::invalid_argument&){rejected=true;}
  expect(rejected,"NaN must be rejected");
  s=hk::Settings{};s.pixel="BayerRG12Packed";rejected=false;
  try{hk::validate(s);}catch(const std::invalid_argument&){rejected=true;}
  expect(rejected,"Unsupported packed formats must be rejected");
  int hardware=10;bool invalid=false;
  auto reason=hk::transaction([&]{hardware=20;throw std::runtime_error("SDK failed after partial update");},
                              [&]{hardware=10;},[&]{invalid=true;});
  expect(!reason.empty()&&hardware==10&&!invalid,"Partial update rollback");
  reason=hk::transaction([&]{hardware=20;},[&]{hardware=10;},[&]{invalid=true;});
  expect(reason.empty()&&hardware==20,"Successful hardware commit");
  reason=hk::transaction([]{throw std::runtime_error("write failed");},
                        []{throw std::runtime_error("device unplugged");},[&]{invalid=true;});
  expect(!reason.empty()&&invalid,"Rollback failure must invalidate device");
  hk::LatestSlot<int> slot;
  expect(!slot.put(1),"First frame should not count as replacement");
  expect(slot.put(2),"Second frame replaces unread first frame");
  int value=0;expect(slot.take(value)&&value==2,"Latest frame must win");
  slot.put(3);slot.clear();slot.put(4);expect(slot.take(value)&&value==4,"Reconfigure clears old queue");
  std::atomic<bool> released{false};
  std::thread waiter([&]{int v=0;released=!slot.take(v);});
  slot.close();waiter.join();expect(released,"Close must release a blocked consumer");
  expect(!slot.put(5)&&!slot.take(value),"Closed slot must not deliver frames");
  // Concurrent replacement accounting: every produced frame is consumed or replaced.
  hk::LatestSlot<int> concurrent;std::atomic<int> replacements{0};int consumed=0,last=0;
  std::thread producer([&]{for(int i=1;i<=10000;++i)if(concurrent.put(i))++replacements;});
  while(last!=10000){expect(concurrent.take(last),"Premature close");++consumed;}
  producer.join();concurrent.close();
  expect(consumed+replacements==10000,"Queue frame accounting");
  std::cout<<"PASS: validation, partial-write rollback, rollback failure, latest-frame queue, shutdown and concurrent frame accounting\n";
}
