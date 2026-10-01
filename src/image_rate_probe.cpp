#include <algorithm>
#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

// A lightweight C++ subscriber. Counts actual received messages, not publisher calls.
class Probe : public rclcpp::Node {
public:
  Probe():Node("image_rate_probe") {
    auto topic=declare_parameter<std::string>("image_topic","/image_raw");
    start_=Clock::now();
    sub_=create_subscription<sensor_msgs::msg::Image>(topic,rclcpp::SensorDataQoS().keep_last(5),
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg){
        auto time=Clock::now();
        if(have_last_) {double dt=std::chrono::duration<double>(time-last_).count();
          min_=std::min(min_,dt);max_=std::max(max_,dt);}
        last_=time;have_last_=true;++count_;bytes_+=msg->data.size();
        width_=msg->width;height_=msg->height;encoding_=msg->encoding;
      });
    timer_=create_wall_timer(std::chrono::seconds(2),[this]{
      auto time=Clock::now();double dt=std::chrono::duration<double>(time-start_).count();start_=time;
      RCLCPP_INFO(get_logger(),"received_hz=%.2f MB_per_sec=%.2f interval_ms=[%.2f,%.2f] size=%ux%u encoding=%s messages=%llu",
        count_/dt,bytes_/dt/1e6,min_==std::numeric_limits<double>::infinity()?0:min_*1000,max_*1000,
        width_,height_,encoding_.c_str(),static_cast<unsigned long long>(count_));
      count_=bytes_=0;min_=std::numeric_limits<double>::infinity();max_=0;
    });
  }
private:
  using Clock=std::chrono::steady_clock;
  Clock::time_point start_,last_;bool have_last_=false;
  uint64_t count_=0,bytes_=0;unsigned int width_=0,height_=0;
  double min_=std::numeric_limits<double>::infinity(),max_=0;std::string encoding_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char **argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<Probe>());rclcpp::shutdown();}
