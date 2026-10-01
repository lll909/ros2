#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <iomanip>
#include <memory>
#include <sstream>
#include <thread>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "MvCameraControl.h"
#include "hk_camera_driver/core.hpp"

using Steady = std::chrono::steady_clock;
using namespace std::chrono_literals;

// New SDK releases require global initialization. CMake probes both symbols.
struct SdkRuntime {
  SdkRuntime() {
#ifdef HK_MVS_HAS_INITIALIZE
    int ret=MV_CC_Initialize();
    if(ret!=MV_OK) throw std::runtime_error("MV_CC_Initialize failed");
#endif
  }
  ~SdkRuntime() {
#ifdef HK_MVS_HAS_INITIALIZE
    MV_CC_Finalize();
#endif
  }
};

class HkCameraNode : public rclcpp::Node {
  struct Frame {
    std::vector<unsigned char> bytes;
    unsigned int width=0, height=0, pixel=0;
    uint64_t generation=0;
    builtin_interfaces::msg::Time stamp;
    Steady::time_point received;
  };
  struct Snapshot { double exposure=0, gain=0, fps=0; unsigned int pixel=0, exposure_auto=0, gain_auto=0; bool rate_enabled=false; };
  struct ControlPriority {
    std::atomic<unsigned int> &count;
    explicit ControlPriority(std::atomic<unsigned int> &c):count(c){++count;}
    ~ControlPriority(){--count;}
  };
public:
  HkCameraNode() : Node("hk_camera_node") {
    rcl_interfaces::msg::ParameterDescriptor fixed;
    fixed.read_only=true;
    fixed.description="Startup setting; restart node to change";
    ip_=declare_parameter<std::string>("camera_ip", "", fixed);
    serial_=declare_parameter<std::string>("serial_number", "", fixed);
    topic_=declare_parameter<std::string>("image_topic", "/image_raw", fixed);
    frame_id_=declare_parameter<std::string>("frame_id", "camera_optical_frame", fixed);
    output_=declare_parameter<std::string>("output_encoding", "bgr8", fixed);
    mode_=declare_parameter<std::string>("benchmark_mode", "publish", fixed);
    auto reliability=declare_parameter<std::string>("qos_reliability", "best_effort", fixed);
    timeout_=declare_parameter<int>("grab_timeout_ms",100,fixed);
    reconnect_seconds_=declare_parameter<double>("reconnect_timeout_sec",3.0,fixed);
    int cv_threads=declare_parameter<int>("opencv_threads",1,fixed);
    if(timeout_<1 || timeout_>1000 || !std::isfinite(reconnect_seconds_) || reconnect_seconds_<1.0 ||
       cv_threads<1 || cv_threads>64 || (output_!="bgr8" && output_!="native") ||
       (mode_!="publish" && mode_!="capture_only") ||
       (reliability!="best_effort" && reliability!="reliable"))
      throw std::runtime_error("Invalid startup settings; see README");
    cv::setNumThreads(cv_threads);
    settings_.exposure=declare_parameter<double>("exposure_time",5000.0);
    settings_.gain=declare_parameter<double>("gain",10.0);
    settings_.fps=declare_parameter<double>("frame_rate",30.0);
    settings_.pixel=declare_parameter<std::string>("pixel_format","BayerRG8");
    hk::validate(settings_);
    auto qos=rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile();
    if(reliability=="reliable") qos.reliable(); else qos.best_effort();
    publisher_=create_publisher<sensor_msgs::msg::Image>(topic_,qos);
    stats_pub_=create_publisher<std_msgs::msg::String>("~/statistics",10);
    config_callback_=add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &p){return updateParameters(p);});
    readback_=create_service<std_srvs::srv::Trigger>("~/readback",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response){
        ControlPriority priority(control_waiting_);
        std::lock_guard<std::mutex> lock(camera_mutex_);
        try { if(!open_) throw std::runtime_error("Camera disconnected");
          response->message=readbackText(); response->success=true;
        } catch(const std::exception &e){response->success=false;response->message=e.what();}
      });
    stats_time_=Steady::now();
    stats_timer_=create_wall_timer(2s,[this]{report();});
    // No acquisition timer: the camera's hardware frame rate drives blocking acquisition.
    try {
      capture_=std::thread([this]{captureLoop();});
      processing_=std::thread([this]{processLoop();});
    } catch(...) { stop(); throw; }
    RCLCPP_INFO(get_logger(),"HK_ACCEPTANCE_V2 SDK=0x%08X mode=%s output=%s; awaiting camera",
                static_cast<unsigned int>(MV_CC_GetSDKVersion()),mode_.c_str(),output_.c_str());
  }
  ~HkCameraNode() override { stop(); }

private:
  static void checked(int ret,const std::string &operation) {
    if(ret==MV_OK) return;
    char hex[20]; std::snprintf(hex,sizeof(hex),"0x%08X",static_cast<unsigned int>(ret));
    throw std::runtime_error(operation+" failed "+hex);
  }
  static uint64_t micros(Steady::time_point start) {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(Steady::now()-start).count());
  }
  template<size_t N> static std::string text(const unsigned char (&s)[N]) {
    size_t n=0;while(n<N && s[n])++n;return std::string(reinterpret_cast<const char*>(s),n);
  }
  static std::string ipText(unsigned int ip) {
    return std::to_string((ip>>24)&255)+"."+std::to_string((ip>>16)&255)+"."+
           std::to_string((ip>>8)&255)+"."+std::to_string(ip&255);
  }
  static unsigned int pixelValue(const std::string &s) {
    if(s=="Mono8")return PixelType_Gvsp_Mono8;
    if(s=="RGB8Packed")return PixelType_Gvsp_RGB8_Packed;
    if(s=="BGR8Packed")return PixelType_Gvsp_BGR8_Packed;
    if(s=="BayerRG8")return PixelType_Gvsp_BayerRG8;
    if(s=="BayerGR8")return PixelType_Gvsp_BayerGR8;
    if(s=="BayerGB8")return PixelType_Gvsp_BayerGB8;
    if(s=="BayerBG8")return PixelType_Gvsp_BayerBG8;
    throw std::runtime_error("Unsupported pixel format");
  }
  MVCC_FLOATVALUE getFloat(const char *name) {
    MVCC_FLOATVALUE v{};checked(MV_CC_GetFloatValue(handle_,name,&v),std::string("Get ")+name);return v;
  }
  unsigned int getEnum(const char *name) {
    MVCC_ENUMVALUE v{};checked(MV_CC_GetEnumValue(handle_,name,&v),std::string("Get ")+name);return v.nCurValue;
  }
  void setEnum(const char *name,unsigned int value) {
    checked(MV_CC_SetEnumValue(handle_,name,value),std::string("Set ")+name);
  }
  void setFloat(const char *name,double value) {
    auto limits=getFloat(name);
    if(!std::isfinite(value) || value<limits.fMin || value>limits.fMax) {
      std::ostringstream reason;reason<<name<<"="<<value<<" outside camera range ["<<limits.fMin<<","<<limits.fMax<<"]";
      throw std::runtime_error(reason.str());
    }
    checked(MV_CC_SetFloatValue(handle_,name,static_cast<float>(value)),std::string("Set ")+name);
  }
  Snapshot snapshot() {
    Snapshot s;s.exposure=getFloat("ExposureTime").fCurValue;s.gain=getFloat("Gain").fCurValue;
    s.fps=getFloat("AcquisitionFrameRate").fCurValue;s.pixel=getEnum("PixelFormat");
    s.exposure_auto=getEnum("ExposureAuto");s.gain_auto=getEnum("GainAuto");
    checked(MV_CC_GetBoolValue(handle_,"AcquisitionFrameRateEnable",&s.rate_enabled),"Get frame rate enable");
    return s;
  }
  void pauseAcquisition() {
    if(grabbing_) {checked(MV_CC_StopGrabbing(handle_),"StopGrabbing");grabbing_=false;}
  }
  void resumeAcquisition() {
    checked(MV_CC_StartGrabbing(handle_),"StartGrabbing");grabbing_=true;
  }
  void apply(const hk::Settings &s) {
    MVCC_ENUMVALUE formats{};checked(MV_CC_GetEnumValue(handle_,"PixelFormat",&formats),"Read supported PixelFormat");
    const auto pixel=pixelValue(s.pixel);
    const auto count=std::min<size_t>(formats.nSupportedNum,sizeof(formats.nSupportValue)/sizeof(formats.nSupportValue[0]));
    bool found=false;for(size_t i=0;i<count;++i)if(formats.nSupportValue[i]==pixel)found=true;
    if(!found)throw std::runtime_error("Camera does not support pixel_format="+s.pixel);
    setEnum("PixelFormat",pixel);
    setEnum("ExposureAuto",0);setEnum("GainAuto",0);
    setFloat("ExposureTime",s.exposure);setFloat("Gain",s.gain);
    checked(MV_CC_SetBoolValue(handle_,"AcquisitionFrameRateEnable",s.fps>0),"Set frame rate enable");
    if(s.fps>0)setFloat("AcquisitionFrameRate",s.fps);
    if(getEnum("PixelFormat")!=pixel || getEnum("ExposureAuto")!=0 || getEnum("GainAuto")!=0)
      throw std::runtime_error("Camera enum readback differs from request");
    // Quantization is camera-dependent; report actual floats instead of inventing equality.
    if(snapshot().rate_enabled!=(s.fps>0))throw std::runtime_error("Frame rate enable readback differs from request");
  }
  void restore(const Snapshot &s) {
    pauseAcquisition();setEnum("PixelFormat",s.pixel);
    setEnum("ExposureAuto",0);setEnum("GainAuto",0);
    setFloat("ExposureTime",s.exposure);setFloat("Gain",s.gain);
    checked(MV_CC_SetBoolValue(handle_,"AcquisitionFrameRateEnable",true),"Restore frame rate enable");
    setFloat("AcquisitionFrameRate",s.fps);
    checked(MV_CC_SetBoolValue(handle_,"AcquisitionFrameRateEnable",s.rate_enabled),"Restore frame rate enable");
    setEnum("ExposureAuto",s.exposure_auto);setEnum("GainAuto",s.gain_auto);
    resumeAcquisition();
  }
  std::string readbackText() {
    const auto actual=snapshot();MVCC_INTVALUE width{},height{};
    checked(MV_CC_GetIntValue(handle_,"Width",&width),"Get Width");
    checked(MV_CC_GetIntValue(handle_,"Height",&height),"Get Height");
    std::ostringstream out;
    out<<"serial="<<pinned_serial_<<" size="<<width.nCurValue<<"x"<<height.nCurValue
       <<" exposure_us="<<actual.exposure<<" gain="<<actual.gain
       <<" frame_rate_limit_enabled="<<actual.rate_enabled<<" frame_rate_register="<<actual.fps
       <<" pixel_value=0x"<<std::hex<<actual.pixel<<std::dec;
    MVCC_FLOATVALUE resulting{};
    if(MV_CC_GetFloatValue(handle_,"ResultingFrameRate",&resulting)==MV_OK)
      out<<" resulting_frame_rate_register="<<resulting.fCurValue;
    out<<" (register values are not measured FPS)";return out.str();
  }
  void connectCamera() {
    MV_CC_DEVICE_INFO_LIST devices{};checked(MV_CC_EnumDevices(MV_GIGE_DEVICE|MV_USB_DEVICE,&devices),"EnumDevices");
    MV_CC_DEVICE_INFO *selected=nullptr;std::string selected_serial;
    const std::string wanted=pinned_serial_.empty()?serial_:pinned_serial_;
    for(unsigned int i=0;i<devices.nDeviceNum;++i) {
      auto *d=devices.pDeviceInfo[i];if(!d)continue;
      std::string sn,current_ip;
      if(d->nTLayerType==MV_USB_DEVICE)sn=text(d->SpecialInfo.stUsb3VInfo.chSerialNumber);
      else if(d->nTLayerType==MV_GIGE_DEVICE) {
        sn=text(d->SpecialInfo.stGigEInfo.chSerialNumber);current_ip=ipText(d->SpecialInfo.stGigEInfo.nCurrentIp);
      }else continue;
      if((!wanted.empty()&&sn!=wanted)||(!ip_.empty()&&current_ip!=ip_))continue;
      selected=d;selected_serial=sn;break;
    }
    if(!selected)throw std::runtime_error("Requested camera not found (USB: leave camera_ip empty)");
    if(wanted.empty()&&ip_.empty()&&devices.nDeviceNum>1)
      RCLCPP_WARN(get_logger(),"Multiple cameras: selecting first, then pinning its serial number");
    checked(MV_CC_CreateHandle(&handle_,selected),"CreateHandle");
    checked(MV_CC_OpenDevice(handle_),"OpenDevice");open_=true;
    if(selected->nTLayerType==MV_GIGE_DEVICE) {
      const int packet=MV_CC_GetOptimalPacketSize(handle_);
      if(packet>0)checked(MV_CC_SetIntValue(handle_,"GevSCPSPacketSize",packet),"Set GigE packet size");
    }
    setEnum("TriggerMode",0);setEnum("AcquisitionMode",2);
    apply(settings_);
    checked(MV_CC_SetImageNodeNum(handle_,4),"SetImageNodeNum");
    resumeAcquisition();pinned_serial_=selected_serial;
    ++generation_;slot_.clear();connected_=true;last_good_=Steady::now();
    // Allow long requested exposures / low frame rate without declaring a false disconnect.
    RCLCPP_INFO(get_logger(),"Connected: %s",readbackText().c_str());
  }
  void closeCamera() noexcept {
    connected_=false;++generation_;slot_.clear();
    if(handle_) {
      if(grabbing_)MV_CC_StopGrabbing(handle_);
      if(open_)MV_CC_CloseDevice(handle_);
      MV_CC_DestroyHandle(handle_);
    }
    handle_=nullptr;open_=false;grabbing_=false;
  }
  rcl_interfaces::msg::SetParametersResult updateParameters(const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;result.successful=false;
    ControlPriority priority(control_waiting_);
    std::lock_guard<std::mutex> lock(camera_mutex_);
    hk::Settings next=settings_;bool changed=false;
    try {
      for(const auto &p:parameters) {
        if(p.get_name()=="exposure_time"){next.exposure=p.as_double();changed=true;}
        else if(p.get_name()=="gain"){next.gain=p.as_double();changed=true;}
        else if(p.get_name()=="frame_rate"){next.fps=p.as_double();changed=true;}
        else if(p.get_name()=="pixel_format"){next.pixel=p.as_string();changed=true;}
        else if(p.get_name()!="use_sim_time")throw std::runtime_error("This setting requires a node restart: "+p.get_name());
      }
      if(!changed){result.successful=true;return result;}
      hk::validate(next);
      if(!connected_)throw std::runtime_error("Camera disconnected; update rejected. Last accepted settings will be restored on reconnect");
      const auto previous=snapshot();
      ++generation_;slot_.clear();
      result.reason=hk::transaction([&]{pauseAcquisition();apply(next);resumeAcquisition();},
                                   [&]{restore(previous);},[&]{closeCamera();});
      if(!result.reason.empty())return result;
      settings_=next;last_good_=Steady::now();result.successful=true;
      RCLCPP_INFO(get_logger(),"Parameters accepted; use ~/readback to inspect actual camera registers");
    } catch(const std::exception &e){result.successful=false;result.reason=e.what();}
    return result;
  }
  void captureLoop() noexcept {
    auto retry=Steady::now();
    while(running_&&rclcpp::ok()) {
      if(control_waiting_){std::this_thread::sleep_for(1ms);continue;}
      try {
        Frame frame;bool have_frame=false;
        {
          std::lock_guard<std::mutex> lock(camera_mutex_);
          if(!connected_) {
            if(Steady::now()>=retry) {
              try {connectCamera();}
              catch(const std::exception &e){last_error_=e.what();closeCamera();retry=Steady::now()+1s;
                RCLCPP_WARN(get_logger(),"Connect/reconnect: %s",e.what());}
            }
          } else {
            MV_FRAME_OUT buffer{};auto start=Steady::now();
            int ret=MV_CC_GetImageBuffer(handle_,&buffer,static_cast<unsigned int>(timeout_));
            grab_us_+=micros(start);++grab_calls_;
            if(ret==MV_OK) {
              struct Release {void *h;MV_FRAME_OUT *b;~Release(){MV_CC_FreeImageBuffer(h,b);}} release{handle_,&buffer};
              ++captured_;last_good_=Steady::now();
              const auto &info=buffer.stFrameInfo;
              if(mode_!="capture_only") {
                frame.received=last_good_;frame.stamp=now();frame.generation=generation_.load();
                frame.width=info.nWidth;frame.height=info.nHeight;frame.pixel=info.enPixelType;
                const size_t channels=(frame.pixel==PixelType_Gvsp_RGB8_Packed||frame.pixel==PixelType_Gvsp_BGR8_Packed)?3:1;
                const size_t bytes=static_cast<size_t>(frame.width)*frame.height*channels;
                if(!buffer.pBufAddr||!bytes||info.nFrameLen<bytes)throw std::runtime_error("Invalid frame length");
                start=Steady::now();frame.bytes.assign(buffer.pBufAddr,buffer.pBufAddr+bytes);
                copy_us_+=micros(start);have_frame=true;
              }
            } else {
              ++grab_errors_;char code[20];std::snprintf(code,sizeof(code),"0x%08X",static_cast<unsigned int>(ret));
              last_error_=std::string("GetImageBuffer ")+code;
              double allowed=std::max(reconnect_seconds_,3.0*settings_.exposure/1e6+1.0);
              if(settings_.fps>0)allowed=std::max(allowed,3.0/settings_.fps+1.0);
              if(std::chrono::duration<double>(Steady::now()-last_good_).count()>allowed || !MV_CC_IsDeviceConnected(handle_)) {
                RCLCPP_WARN(get_logger(),"%s; reconnecting",last_error_.c_str());closeCamera();retry=Steady::now()+1s;
              }
            }
          }
        }
        if(have_frame && slot_.put(std::move(frame)))++queue_drops_;
        if(!connected_)std::this_thread::sleep_for(50ms);
        else std::this_thread::yield();
      } catch(const std::exception &e) {
        ++grab_errors_;std::lock_guard<std::mutex> lock(camera_mutex_);
        last_error_=e.what();closeCamera();retry=Steady::now()+1s;
        RCLCPP_ERROR(get_logger(),"Acquisition: %s",e.what());
      }
    }
  }
  void processLoop() noexcept {
    Frame frame;
    while(slot_.take(frame)) {
      if(!running_||!rclcpp::ok())break;
      if(frame.generation!=generation_)continue;
      try {
        auto start=Steady::now();
        std::string encoding;int conversion=-1;int channels=1;
        switch(frame.pixel) {
          case PixelType_Gvsp_Mono8:encoding="mono8";if(output_=="bgr8")conversion=cv::COLOR_GRAY2BGR;break;
          case PixelType_Gvsp_RGB8_Packed:encoding="rgb8";channels=3;if(output_=="bgr8")conversion=cv::COLOR_RGB2BGR;break;
          case PixelType_Gvsp_BGR8_Packed:encoding="bgr8";channels=3;break;
          case PixelType_Gvsp_BayerRG8:encoding="bayer_rggb8";conversion=cv::COLOR_BayerRGGB2BGR;break;
          case PixelType_Gvsp_BayerGR8:encoding="bayer_grbg8";conversion=cv::COLOR_BayerGRBG2BGR;break;
          case PixelType_Gvsp_BayerGB8:encoding="bayer_gbrg8";conversion=cv::COLOR_BayerGBRG2BGR;break;
          case PixelType_Gvsp_BayerBG8:encoding="bayer_bggr8";conversion=cv::COLOR_BayerBGGR2BGR;break;
          default:throw std::runtime_error("Unsupported incoming PixelFormat");
        }
        auto message=std::make_unique<sensor_msgs::msg::Image>();
        message->header.stamp=frame.stamp;message->header.frame_id=frame_id_;
        message->width=frame.width;message->height=frame.height;message->is_bigendian=0;
        if(output_=="bgr8"&&conversion>=0) {
          message->encoding="bgr8";message->step=frame.width*3;
          message->data.resize(static_cast<size_t>(message->step)*frame.height);
          cv::Mat raw(static_cast<int>(frame.height),static_cast<int>(frame.width),channels==3?CV_8UC3:CV_8UC1,frame.bytes.data());
          cv::Mat destination(static_cast<int>(frame.height),static_cast<int>(frame.width),CV_8UC3,message->data.data());
          cv::cvtColor(raw,destination,conversion);
        } else {
          message->encoding=encoding;message->step=frame.width*channels;
          message->data=std::move(frame.bytes);
        }
        conversion_us_+=micros(start);++processed_;
        if(frame.generation!=generation_)continue;
        start=Steady::now();publisher_->publish(std::move(message));
        publish_us_+=micros(start);age_us_+=micros(frame.received);++published_;
      } catch(const std::exception &e) {
        ++processing_errors_;
        RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),3000,"Process/publish: %s",e.what());
      }
    }
  }
  void report() {
    const auto time=Steady::now();double seconds=std::chrono::duration<double>(time-stats_time_).count();stats_time_=time;
    const auto cap=captured_.exchange(0),proc=processed_.exchange(0),pub=published_.exchange(0),calls=grab_calls_.exchange(0);
    std::ostringstream out;out<<std::fixed<<std::setprecision(2)
      <<"connected="<<connected_<<" mode="<<mode_
      <<" capture_hz="<<cap/seconds<<" process_hz="<<proc/seconds<<" publish_hz="<<pub/seconds
      <<" latest_queue_drops="<<queue_drops_.exchange(0)<<" grab_errors="<<grab_errors_.exchange(0)
      <<" process_errors="<<processing_errors_.exchange(0)
      <<" grab_wait_ms="<<grab_us_.exchange(0)/1000.0/std::max<uint64_t>(calls,1)
      <<" copy_ms="<<copy_us_.exchange(0)/1000.0/std::max<uint64_t>(cap,1)
      <<" convert_ms="<<conversion_us_.exchange(0)/1000.0/std::max<uint64_t>(proc,1)
      <<" publish_call_ms="<<publish_us_.exchange(0)/1000.0/std::max<uint64_t>(pub,1)
      <<" host_pipeline_ms="<<age_us_.exchange(0)/1000.0/std::max<uint64_t>(pub,1);
    // Avoid a blocking stats timer while the SDK is reconnecting.
    std::unique_lock<std::mutex> lock(camera_mutex_,std::try_to_lock);
    if(lock.owns_lock())out<<" requested_fps="<<settings_.fps<<" last_sdk_error="<<last_error_;
    std_msgs::msg::String msg;msg.data=out.str();stats_pub_->publish(msg);
    RCLCPP_INFO(get_logger(),"%s",msg.data.c_str());
  }
  void stop() noexcept {
    running_=false;slot_.close();
    if(capture_.joinable())capture_.join();
    if(processing_.joinable())processing_.join();
    std::lock_guard<std::mutex> lock(camera_mutex_);closeCamera();
  }
  // Only acquisition and parameter/service callbacks access the SDK, serialized by this mutex.
  std::mutex camera_mutex_;
  void *handle_=nullptr;bool open_=false,grabbing_=false;
  hk::Settings settings_;
  std::string ip_,serial_,pinned_serial_,topic_,frame_id_,output_,mode_,last_error_;
  int timeout_=100;double reconnect_seconds_=3;
  std::atomic<bool> running_{true},connected_{false};
  std::atomic<unsigned int> control_waiting_{0};
  std::atomic<uint64_t> generation_{0},captured_{0},processed_{0},published_{0},queue_drops_{0},grab_errors_{0},processing_errors_{0};
  std::atomic<uint64_t> grab_calls_{0},grab_us_{0},copy_us_{0},conversion_us_{0},publish_us_{0},age_us_{0};
  Steady::time_point last_good_,stats_time_;
  hk::LatestSlot<Frame> slot_;std::thread capture_,processing_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stats_pub_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr readback_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr config_callback_;
};

int main(int argc,char **argv) {
  rclcpp::init(argc,argv);int status=0;
  try {SdkRuntime sdk;auto node=std::make_shared<HkCameraNode>();rclcpp::spin(node);node.reset();}
  catch(const std::exception &e){RCLCPP_ERROR(rclcpp::get_logger("hk_camera_node"),"%s",e.what());status=1;}
  rclcpp::shutdown();return status;
}
