// SPDX-License-Identifier: Apache-2.0
#include <QLabel>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_default_plugins/tools/goal_pose/goal_tool.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace forklift_rviz {
// Tên lớp giữ nguyên PickA/DropB để các file .rviz cũ còn mở được; nhãn đổi theo
// cách dùng mới: mỗi lần bấm là thêm một ô vào hàng đợi, không ghi đè ô trước.
class PickA : public rviz_default_plugins::tools::GoalTool {
public:
  PickA() {shortcut_key_='a';}
  void onInitialize() override {
    GoalTool::onInitialize();setName("Pick slot");
    getPropertyContainer()->subProp("Topic")->setValue("/mission/pick_pose");
  }
};
class DropB : public rviz_default_plugins::tools::GoalTool {
public:
  DropB() {shortcut_key_='b';}
  void onInitialize() override {
    GoalTool::onInitialize();setName("Drop slot");
    getPropertyContainer()->subProp("Topic")->setValue("/mission/drop_pose");
  }
};
class MissionPanel : public rviz_common::Panel {
  using Trigger=std_srvs::srv::Trigger;
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Client<Trigger>::SharedPtr start_,stop_,clear_,save_,load_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr input_,mission_;
  QLabel *status_,*response_;
  QTimer *timer_;
  void call(const rclcpp::Client<Trigger>::SharedPtr &client) {
    if(!client->service_is_ready()) {response_->setText("Service unavailable");return;}
    response_->setText("Request sent");
    client->async_send_request(std::make_shared<Trigger::Request>(),[this](rclcpp::Client<Trigger>::SharedFuture f) {
      response_->setText(QString::fromStdString(f.get()->message));
    });
  }
public:
  explicit MissionPanel(QWidget *parent=nullptr):rviz_common::Panel(parent) {
    auto layout=new QVBoxLayout;
    auto help=new QLabel("Pick slot (A): o lay hang, danh so 1..N\n"
                         "Drop slot (B): pose xe luc ha hang, o danh so N+1..2N\n"
                         "Click va keo de chon huong cang. Run chay lan luot tung cap.");
    help->setWordWrap(true);
    layout->addWidget(help);
    for(const auto &item:std::vector<std::pair<QString,int>>{
        {"Run mission",0},{"Stop mission",1},{"Clear slots",2},{"Save slots",3},{"Load slots",4}}) {
      auto button=new QPushButton(item.first);layout->addWidget(button);
      connect(button,&QPushButton::clicked,this,[this,id=item.second](){
        call(id==0?start_:id==1?stop_:id==2?clear_:id==3?save_:load_);});
    }
    status_=new QLabel("Waiting for mission nodes");status_->setWordWrap(true);layout->addWidget(status_);
    response_=new QLabel;response_->setWordWrap(true);layout->addWidget(response_);setLayout(layout);
    node_=std::make_shared<rclcpp::Node>("forklift_mission_panel",rclcpp::NodeOptions().use_global_arguments(false));
    start_=node_->create_client<Trigger>("/mission/start");stop_=node_->create_client<Trigger>("/mission/stop");clear_=node_->create_client<Trigger>("/mission/clear");
    save_=node_->create_client<Trigger>("/mission/save_slots");load_=node_->create_client<Trigger>("/mission/load_slots");
    input_=node_->create_subscription<std_msgs::msg::String>("/mission/input_status",10,[this](std_msgs::msg::String::ConstSharedPtr m){
      const auto obj=QJsonDocument::fromJson(QByteArray::fromStdString(m->data)).object();
      const int pairs=obj["pairs"].toInt(),trip=obj["trip"].toInt(),trips=obj["trips"].toInt();
      // trip = 0 khi khong co chuyen nao dang chay; chi hien tien trinh khi dang chay.
      const QString progress=trip>0?QString(" | chuyen %1/%2").arg(trip).arg(trips):QString();
      response_->setText(QString("O lay %1/%3 | O dat %2/%3%4\n%5")
        .arg(obj["picks"].toInt()).arg(obj["drops"].toInt()).arg(pairs)
        .arg(progress,obj["message"].toString()));
    });
    mission_=node_->create_subscription<std_msgs::msg::String>("/mission/status",10,[this](std_msgs::msg::String::ConstSharedPtr m){
      const auto obj=QJsonDocument::fromJson(QByteArray::fromStdString(m->data)).object();
      status_->setText(obj["state"].toString()+" "+obj["code"].toString());
    });
    executor_.add_node(node_);timer_=new QTimer(this);
    connect(timer_,&QTimer::timeout,this,[this](){executor_.spin_some(std::chrono::milliseconds(2));});timer_->start(50);
  }
  ~MissionPanel() override {timer_->stop();executor_.remove_node(node_);}
};
}
PLUGINLIB_EXPORT_CLASS(forklift_rviz::PickA,rviz_common::Tool)
PLUGINLIB_EXPORT_CLASS(forklift_rviz::DropB,rviz_common::Tool)
PLUGINLIB_EXPORT_CLASS(forklift_rviz::MissionPanel,rviz_common::Panel)
