// SPDX-License-Identifier: Apache-2.0
#include "forklift_hardware/pty_system.hpp"
#include <algorithm>
#include <cerrno>
#include <fcntl.h>
#include <fstream>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace forklift_hardware {
using Callback=hardware_interface::CallbackReturn;
using Result=hardware_interface::return_type;
PtySystem::~PtySystem(){stop();}
Callback PtySystem::on_init(const hardware_interface::HardwareInfo &info){
  if(SystemInterface::on_init(info)!=Callback::SUCCESS) return Callback::ERROR;
  if(info.joints.size()!=5) return Callback::ERROR;
  try {
    port_file_=info.hardware_parameters.at("port_file");
    cpr_=std::stod(info.hardware_parameters.at("encoder_cpr"));
    lift_steps_per_m_=std::stod(info.hardware_parameters.at("lift_steps_per_m"));
    if(!std::isfinite(cpr_) || !std::isfinite(lift_steps_per_m_) || cpr_<=0 || lift_steps_per_m_<=0) return Callback::ERROR;
    for(size_t i=0;i<5;++i){
      auto &j=info.joints[i];
      if(j.name!=names_[i] || j.command_interfaces.size()!=1 || j.command_interfaces[0].name!=(i==4?"position":"velocity") ||
        j.state_interfaces.size()!=2 || j.state_interfaces[0].name!="position" || j.state_interfaces[1].name!="velocity") return Callback::ERROR;
    }
  } catch(const std::exception &){return Callback::ERROR;}
  return Callback::SUCCESS;
}
std::vector<hardware_interface::StateInterface> PtySystem::export_state_interfaces(){
  std::vector<hardware_interface::StateInterface> states;
  for(size_t i=0;i<5;++i){states.emplace_back(names_[i],"position",&position_[i]);states.emplace_back(names_[i],"velocity",&velocity_[i]);}
  return states;
}
std::vector<hardware_interface::CommandInterface> PtySystem::export_command_interfaces(){
  std::vector<hardware_interface::CommandInterface> commands;
  for(size_t i=0;i<5;++i) commands.emplace_back(names_[i],i==4?"position":"velocity",&command_[i]);
  return commands;
}
Callback PtySystem::on_configure(const rclcpp_lifecycle::State &){
  stop(); fault_=false; have_state_=false; consumed_seq_=consumed_us_=0;
  auto end=Clock::now()+std::chrono::seconds(5);std::string port;
  do {
    std::ifstream in(port_file_);std::getline(in,port);
    if(!port.empty()) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  } while(Clock::now()<end);
  // This revision is intentionally unable to open USB/UART hardware.
  if(port.size()<=9 || port.rfind("/dev/pts/",0)!=0 || port.find_first_not_of("0123456789",9)!=std::string::npos) return Callback::ERROR;
  fd_=::open(port.c_str(),O_RDWR|O_NOCTTY|O_NONBLOCK|O_CLOEXEC);
  if(fd_<0) return Callback::ERROR;
  termios tty{};
  if(tcgetattr(fd_,&tty)!=0){stop();return Callback::ERROR;}
  cfmakeraw(&tty);tcsetattr(fd_,TCSANOW,&tty);
  running_=true;thread_=std::thread(&PtySystem::worker,this);
  while(Clock::now()<end){
    {std::lock_guard<std::mutex> lock(mutex_);if(have_state_) return Callback::SUCCESS;}
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  stop();return Callback::ERROR;
}
Callback PtySystem::on_activate(const rclcpp_lifecycle::State &){
  std::lock_guard<std::mutex> lock(mutex_);
  if(!have_state_ || Clock::now()-received_at_>std::chrono::milliseconds(250) || received_.fault) return Callback::ERROR;
  for(size_t i=0;i<4;++i){position_[i]=received_.ticks[i]*2*M_PI/cpr_;previous_ticks_[i]=received_.ticks[i];command_[i]=outgoing_[i]=0.;}
  position_[4]=received_.lift/lift_steps_per_m_;command_[4]=outgoing_[4]=position_[4];previous_lift_=received_.lift;
  velocity_.fill(0.);consumed_seq_=received_.seq;consumed_us_=received_.us;
  written_at_=Clock::now();fault_=false;enabled_=true;return Callback::SUCCESS;
}
Callback PtySystem::on_deactivate(const rclcpp_lifecycle::State &){enabled_=false;return Callback::SUCCESS;}
Callback PtySystem::on_cleanup(const rclcpp_lifecycle::State &){stop();return Callback::SUCCESS;}
Callback PtySystem::on_error(const rclcpp_lifecycle::State &){enabled_=false;fault_=true;return Callback::SUCCESS;}
void PtySystem::stop(){
  enabled_=false;running_=false;
  if(thread_.joinable()) thread_.join();
  if(fd_>=0){::close(fd_);fd_=-1;}
}
Result PtySystem::read(const rclcpp::Time &,const rclcpp::Duration &){
  std::lock_guard<std::mutex> lock(mutex_);
  if(fault_ || !have_state_ || Clock::now()-received_at_>std::chrono::milliseconds(250)){
    fault_=true;enabled_=false;velocity_.fill(0.);return Result::ERROR;
  }
  if(received_.seq==consumed_seq_) return Result::OK;
  if(received_.us<=consumed_us_){fault_=true;enabled_=false;return Result::ERROR;}
  double dt=(received_.us-consumed_us_)/1e6;
  for(size_t i=0;i<4;++i){
    position_[i]=received_.ticks[i]*2*M_PI/cpr_;
    velocity_[i]=(received_.ticks[i]-previous_ticks_[i])*2*M_PI/cpr_/dt;
    previous_ticks_[i]=received_.ticks[i];
  }
  position_[4]=received_.lift/lift_steps_per_m_;
  velocity_[4]=(received_.lift-previous_lift_)/lift_steps_per_m_/dt;
  previous_lift_=received_.lift;consumed_us_=received_.us;consumed_seq_=received_.seq;return Result::OK;
}
Result PtySystem::write(const rclcpp::Time &,const rclcpp::Duration &){
  if(fault_) return Result::ERROR;
  for(size_t i=0;i<5;++i){
    if(!std::isfinite(command_[i]) || (i<4 && std::abs(command_[i])>12.001) ||
       (i==4 && (command_[i]<-.001 || command_[i]>.251))){fault_=true;enabled_=false;return Result::ERROR;}
  }
  std::lock_guard<std::mutex> lock(mutex_);outgoing_=command_;outgoing_[4]=std::clamp(outgoing_[4],0.,.25);written_at_=Clock::now();return Result::OK;
}
void PtySystem::worker(){
  auto node=rclcpp::Node::make_shared("forklift_serial_diagnostics");
  auto pub=node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics",10);
  std::string buffer;uint64_t tx_seq=0,bad=0;auto next=Clock::now(),diag_at=next;
  while(running_){
    pollfd p{fd_,POLLIN,0};::poll(&p,1,5);
    if(p.revents&(POLLERR|POLLHUP|POLLNVAL)){fault_=true;enabled_=false;}
    char data[512];ssize_t count=::read(fd_,data,sizeof(data));
    if(count>0) buffer.append(data,static_cast<size_t>(count));
    size_t at;
    while((at=buffer.find('\n'))!=std::string::npos){
      auto line=buffer.substr(0,at);buffer.erase(0,at+1);State state;
      if(!parse_state(line,state)){++bad;continue;}
      std::lock_guard<std::mutex> lock(mutex_);
      if(have_state_ && state.boot!=received_.boot){fault_=true;enabled_=false;}
      if(have_state_ && state.boot==received_.boot && state.seq<=received_.seq){++bad;continue;}
      if(state.fault){fault_=true;enabled_=false;}
      received_=state;received_at_=Clock::now();have_state_=true;
    }
    if(buffer.size()>1024){buffer.clear();++bad;}
    auto now=Clock::now();
    if(now>=next){
      std::array<double,5> cmd;bool enable;
      {std::lock_guard<std::mutex> lock(mutex_);cmd=outgoing_;enable=enabled_&&!fault_&&now-written_at_<std::chrono::milliseconds(150);}
      std::ostringstream s;s<<std::setprecision(9)<<"C,1,"<<++tx_seq;
      for(size_t i=0;i<4;++i) s<<','<<(enable?cmd[i]:0.);
      s<<','<<cmd[4]<<','<<(enable?1:0);auto bytes=frame(s.str());
      auto sent=::write(fd_,bytes.data(),bytes.size());
      // Partial writes are not replayed: discard and latch a transport fault.
      if(sent!=static_cast<ssize_t>(bytes.size())){fault_=true;enabled_=false;}
      next=now+std::chrono::milliseconds(20);
    }
    if(now>=diag_at){
      diagnostic_msgs::msg::DiagnosticArray a;a.header.stamp=node->now();
      diagnostic_msgs::msg::DiagnosticStatus status;
      status.name="forklift_hardware/pty";status.hardware_id="VIRTUAL_MCU_ONLY";
      status.level=fault_?2:0;status.message=fault_?"FAULT_LATCHED":"PTY_READY";
      diagnostic_msgs::msg::KeyValue value;value.key="invalid_frames";value.value=std::to_string(bad);status.values.push_back(value);
      a.status.push_back(status);pub->publish(a);diag_at=now+std::chrono::milliseconds(200);
    }
  }
  // MCU's independent watchdog also stops outputs if this final frame is lost.
  std::string stop_frame=frame("C,1,"+std::to_string(++tx_seq)+",0,0,0,0,0,0");
  (void)::write(fd_,stop_frame.data(),stop_frame.size());
}
}  // namespace forklift_hardware
PLUGINLIB_EXPORT_CLASS(forklift_hardware::PtySystem,hardware_interface::SystemInterface)
