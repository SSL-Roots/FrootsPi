// Copyright 2021 Roots
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <pigpiod_if2.h>

#include <algorithm>
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "frootspi_hardware/driver_component.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace frootspi_hardware
{

static const int GPIO_SHUTDOWN_SWITCH = 23;
static const int GPIO_KICK_STRAIGHT = 7;
static const int GPIO_KICK_CHIP = 24;
static const int GPIO_KICK_SUPPLY_POWER = 5;
static const int GPIO_KICK_ENABLE_CHARGE = 26;
static const int GPIO_KICK_CHARGE_COMPLETE = 12;
static const int GPIO_DRIBBLE_PWM = 13;
static const int DRIBBLE_PWM_FREQUENCY = 40000;  // kHz
static const int DRIBBLE_PWM_DUTY_CYCLE = 1e6 / DRIBBLE_PWM_FREQUENCY;  // usec

static const int GPIO_CENTER_LED = 14;
static const int GPIO_RIGHT_LED = 4;

Driver::Driver(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("hardware_driver", options),
  pi_(-1), enable_kicker_charging_(false), discharge_kick_count_(0)
{
}

Driver::~Driver()
{
  gpio_write(pi_, GPIO_DRIBBLE_PWM, PI_HIGH);  // 負論理のためHighでモータオフ
  // lcd_driver_.write_texts("ROS 2", "SHUTDOWN");
  gpio_write(pi_, GPIO_KICK_ENABLE_CHARGE, PI_LOW);
  gpio_write(pi_, GPIO_KICK_SUPPLY_POWER, PI_LOW);
  pigpio_stop(pi_);
}

void Driver::on_polling_timer()
{
  // ボール検出をパブリッシュ
  auto ball_detection_msg = std::make_unique<frootspi_msgs::msg::BallDetection>();
  ball_detection_msg->detected = gpio_read(pi_, gpio_ball_sensor_);
  front_indicate_data_.Parameter.BallSens = ball_detection_msg->detected;
  pub_ball_detection_->publish(std::move(ball_detection_msg));

  // バッテリー電圧をパブリッシュ
  auto battery_voltage_msg = std::make_unique<frootspi_msgs::msg::BatteryVoltage>();
  battery_monitor_.main_battery_info_read(
    battery_voltage_msg->voltage, battery_voltage_msg->voltage_status);
  front_indicate_data_.Parameter.BatVol = (unsigned char)(battery_voltage_msg->voltage*10);
  // frootspi_msgs::msg::BatteryVoltage::BATTERY_VOLTAGE_STATUS_FULL;
  pub_battery_voltage_->publish(std::move(battery_voltage_msg));

  // UPS(無停電電源装置)電圧をパブリッシュ
  auto ups_voltage_msg = std::make_unique<frootspi_msgs::msg::BatteryVoltage>();
  battery_monitor_.sub_battery_info_read(
    ups_voltage_msg->voltage, ups_voltage_msg->voltage_status);
  // frootspi_msgs::msg::BatteryVoltage::BATTERY_VOLTAGE_STATUS_TOO_LOW;
  pub_ups_voltage_->publish(std::move(ups_voltage_msg));

  // キッカー（昇圧回路）電圧をパブリッシュ
  capacitor_monitor_prescaler_count_++;
  if(capacitor_monitor_prescaler_count_ > 50){
    auto kicker_voltage_msg = std::make_unique<frootspi_msgs::msg::BatteryVoltage>();
    capacitor_monitor_.capacitor_info_read(
      kicker_voltage_msg->voltage, kicker_voltage_msg->voltage_status);
    front_indicate_data_.Parameter.CapVol = (unsigned char)(kicker_voltage_msg->voltage);
    if(kicker_voltage_msg->voltage_status >= frootspi_msgs::msg::BatteryVoltage::BATTERY_VOLTAGE_STATUS_OK){
      front_indicate_data_.Parameter.CapacitorSta = true;
    } else {
      front_indicate_data_.Parameter.CapacitorSta = false;
    }
    pub_kicker_voltage_->publish(std::move(kicker_voltage_msg));
    capacitor_monitor_prescaler_count_ = 0;
  }


  // スイッチ状態をパブリッシュ
  auto switches_state_msg = std::make_unique<frootspi_msgs::msg::SwitchesState>();
  io_expander_.read(
    switches_state_msg->pushed_button0, switches_state_msg->pushed_button1,
    switches_state_msg->pushed_button2, switches_state_msg->pushed_button3,
    switches_state_msg->turned_on_dip0, switches_state_msg->turned_on_dip1);
  // シャットダウンスイッチは負論理なので、XORでビット反転させる
  switches_state_msg->pushed_shutdown = gpio_read(pi_, GPIO_SHUTDOWN_SWITCH) ^ 1;
  pub_switches_state_->publish(std::move(switches_state_msg));

  // オムニホイール回転速度をパブリッシュ
  auto wheel_velocities_msg = std::make_unique<frootspi_msgs::msg::WheelVelocities>();
  wheel_velocities_msg->front_left = 1.0;  // 左前ホイール回転速度 [rad/sec]
  wheel_velocities_msg->front_right = 1.0;  // 右前ホイール回転速度 [rad/sec]
  wheel_velocities_msg->back_center = 1.0;  // 後ホイール回転速度 [rad/sec]
  pub_present_wheel_velocities_->publish(std::move(wheel_velocities_msg));

  // IMUセンサの情報をパブリッシュ

  // タイヤ目標速度のタイムアウト処理
  if (steady_clock_.now().seconds() - sub_wheel_timestamp_.seconds() >= 0.5) {
    // 通信タイムアウト
    wheel_controller_.set_wheel_velocities(0, 0, 0);
    if (timeout_has_printed_ == false) {
      RCLCPP_WARN(
        this->get_logger(),
        "通信タイムアウトのため、ホイールの回転速度を0 rad/sにします");
      timeout_has_printed_ = true;
    }
  } else {
    timeout_has_printed_ = false;
  }
  
  // フロント基板へ情報を送信
  // front_display_prescaler_count_++;
  // if((front_display_prescaler_count_ > 100) && (capacitor_monitor_prescaler_count_ != 0)){
  //   std::string node_namespace = (std::string)get_namespace();
  //   node_namespace.replace(0,6,"");
  //   front_indicate_data_.Parameter.RobotID = atoi(node_namespace.c_str());
  //   front_display_communicator_.send_data(&front_indicate_data_);
  //   front_display_prescaler_count_ = 0;
  // }
}

void Driver::on_discharge_kicker_timer()
{
  // キッカーコンデンサ放電用の定期実行関数
  // タイマーからこの関数が呼ばれるたびに弱キックしてコンデンサの電荷を放電する
  const int TIME_FOR_DISCHARGE_KICK = 1;
  const int NUM_OF_DISCHARGE_KICK = 30;

  // 充電許可フラグをオフにする
  gpio_write(pi_, GPIO_KICK_ENABLE_CHARGE, PI_LOW);
  enable_kicker_charging_ = false;

  // 威力の弱いキック
  gpio_write(pi_, GPIO_KICK_STRAIGHT, PI_HIGH);
  rclcpp::sleep_for(std::chrono::milliseconds(TIME_FOR_DISCHARGE_KICK));
  gpio_write(pi_, GPIO_KICK_STRAIGHT, PI_LOW);

  // 一定回数キックしたらタイマーをオフする
  discharge_kick_count_++;
  if (discharge_kick_count_ > NUM_OF_DISCHARGE_KICK) {
    discharge_kicker_timer_->cancel();
    discharge_kick_count_ = 0;
  }
}

void Driver::callback_dribble_power(const frootspi_msgs::msg::DribblePower::SharedPtr msg)
{
  double power = msg->power;
  if (power > 1.0) {
    power = 1.0;
  } else if (power < 0.0) {
    power = 0.0;
  }

  if(power > 0){
    front_indicate_data_.Parameter.DribbleReq = true;
  } else {
    front_indicate_data_.Parameter.DribbleReq = false;
  }
  
  // 負論理のため反転
  // 少数を切り上げるため0.1を足す (例：20.0 -> 19となるので、20.1 -> 20とさせる)
  int dribble_duty_cycle = DRIBBLE_PWM_DUTY_CYCLE * (1.0 - power) + 0.1;

  set_PWM_dutycycle(pi_, GPIO_DRIBBLE_PWM, dribble_duty_cycle);
}

void Driver::callback_wheel_velocities(const frootspi_msgs::msg::WheelVelocities::SharedPtr msg)
{
  wheel_controller_.set_wheel_velocities(
    msg->front_right, msg->front_left, msg->back_center);
  // 通信タイムアウト処理用に、タイムスタンプを取得する
  sub_wheel_timestamp_ = steady_clock_.now();
}

void Driver::on_kick(
  const frootspi_msgs::srv::Kick::Request::SharedPtr request,
  frootspi_msgs::srv::Kick::Response::SharedPtr response)
{
  const int MAX_SLEEP_TIME_MSEC_FOR_STRAIGHT = 25;
  const double MAX_KICK_SPEED = 6.5;  // m/s

  front_indicate_data_.Parameter.KickReq = true;

  if (request->kick_type == frootspi_msgs::srv::Kick::Request::KICK_TYPE_STRAIGHT) {
    // ストレートキック

    // kick_power を 0.0 ~ 1.0に落とし込む
    double kick_power = request->kick_power / MAX_KICK_SPEED;
    if (kick_power > 1.0) {
      kick_power = 1.0;
    } else if (kick_power < 0) {
      kick_power = 0;
    }
    int sleep_time_msec = MAX_SLEEP_TIME_MSEC_FOR_STRAIGHT * kick_power;

    // キックをする際は充電を停止する
    gpio_write(pi_, GPIO_KICK_ENABLE_CHARGE, PI_LOW);
    // GPIOをHIGHにしている時間を変化させて、キックパワーを変更する
    gpio_write(pi_, GPIO_KICK_STRAIGHT, PI_HIGH);
    rclcpp::sleep_for(std::chrono::milliseconds(sleep_time_msec));
    gpio_write(pi_, GPIO_KICK_STRAIGHT, PI_LOW);

    if (enable_kicker_charging_) {
      // 充電許可が出ていれば、充電を再開する
      gpio_write(pi_, GPIO_KICK_ENABLE_CHARGE, PI_HIGH);
    }

    response->success = true;
    response->message = std::to_string(sleep_time_msec) + " ミリ秒間ソレノイドをONしました";

  } else if (request->kick_type == frootspi_msgs::srv::Kick::Request::KICK_TYPE_CHIP) {
    // チップキックは未実装
    response->success = false;
    response->message = "チップキックデバイスが搭載されていません";

  } else if (request->kick_type == frootspi_msgs::srv::Kick::Request::KICK_TYPE_DISCHARGE) {
    // 放電
    // discharge_kicker();
    discharge_kicker_timer_->reset();  // 放電タイマーを再開して放電キック開始
    RCLCPP_INFO(this->get_logger(), "キッカーの充電停止.");  // 充電停止しているので通知

    response->success = true;
    response->message = "放電を開始しました";
  } else {
    // 未定義のキックタイプ
    response->success = false;
    response->message = "未定義のキックタイプです";
  }
  front_indicate_data_.Parameter.KickReq = false;
}

void Driver::on_set_kicker_charging(
  const frootspi_msgs::srv::SetKickerCharging::Request::SharedPtr request,
  frootspi_msgs::srv::SetKickerCharging::Response::SharedPtr response)
{
  if (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    // キッカーの充電はキック処理時にON/OFFされるので、
    // enable_kicker_charging_ 変数で充電フラグを管理する
    if (request->start_charging) {
      gpio_write(pi_, GPIO_KICK_ENABLE_CHARGE, PI_HIGH);
      enable_kicker_charging_ = true;
      RCLCPP_INFO(this->get_logger(), "キッカーの充電開始.");
      response->message = "充電を開始しました";
      front_indicate_data_.Parameter.ChargeReq = true;
    } else {
      gpio_write(pi_, GPIO_KICK_ENABLE_CHARGE, PI_LOW);
      enable_kicker_charging_ = false;
      RCLCPP_INFO(this->get_logger(), "キッカーの充電停止.");
      response->message = "充電を停止しました";
      front_indicate_data_.Parameter.ChargeReq = false;
    }

    response->success = true;
  } else {
    gpio_write(pi_, GPIO_KICK_ENABLE_CHARGE, PI_LOW);
    enable_kicker_charging_ = false;

    response->success = false;
    response->message = "ノードがアクティブではありません。安全のため充電を停止しました。";
  }
}

// void Driver::on_set_lcd_text(
//   const frootspi_msgs::srv::SetLCDText::Request::SharedPtr request,
//   frootspi_msgs::srv::SetLCDText::Response::SharedPtr response)
// {
//   if (lcd_driver_.write_texts(request->text1, request->text2)) {
//     response->success = true;
//     response->message = "LCDに " + request->text1 + ", " + request->text2 + " をセットしました";
//   } else {
//     response->success = false;
//     response->message = "LCDに文字列をセットできませんでした。";
//   }
// }

void Driver::on_set_left_led(
  const std_srvs::srv::SetBool::Request::SharedPtr request,
  std_srvs::srv::SetBool::Response::SharedPtr response)
{
  if (io_expander_.set_led(request->data)) {
    response->success = true;
    response->message = "LED操作成功";
  } else {
    response->success = false;
    response->message = "LED操作失敗";
  }
}

void Driver::on_set_center_led(
  const std_srvs::srv::SetBool::Request::SharedPtr request,
  std_srvs::srv::SetBool::Response::SharedPtr response)
{
  if (gpio_write(pi_, GPIO_CENTER_LED, request->data) == 0) {
    response->success = true;
    response->message = "LED操作成功";
  } else {
    response->success = false;
    response->message = "LED操作失敗";
  }
}

void Driver::on_set_right_led(
  const std_srvs::srv::SetBool::Request::SharedPtr request,
  std_srvs::srv::SetBool::Response::SharedPtr response)
{
  if (gpio_write(pi_, GPIO_RIGHT_LED, request->data) == 0) {
    response->success = true;
    response->message = "LED操作成功";
  } else {
    response->success = false;
    response->message = "LED操作失敗";
  }
}

CallbackReturn Driver::on_configure(const rclcpp_lifecycle::State &)
{
  using namespace std::placeholders;  // for _1, _2, _3...

  RCLCPP_INFO(this->get_logger(), "on_configure() is called.");

  // GPIOを定期的にread / writeするためのタイマー
  polling_timer_ = create_wall_timer(1ms, std::bind(&Driver::on_polling_timer, this));
  polling_timer_->cancel();  // ノードがActiveになるまでタイマーオフ
  // コンデンサ放電時に使うタイマー
  discharge_kicker_timer_ =
    create_wall_timer(500ms, std::bind(&Driver::on_discharge_kicker_timer, this));
  discharge_kicker_timer_->cancel();  // 放電処理が始まるまでタイマーオフ

  pub_ball_detection_ = create_publisher<frootspi_msgs::msg::BallDetection>("ball_detection", 1);
  pub_battery_voltage_ = create_publisher<frootspi_msgs::msg::BatteryVoltage>("battery_voltage", 1);
  pub_ups_voltage_ = create_publisher<frootspi_msgs::msg::BatteryVoltage>("ups_voltage", 1);
  pub_kicker_voltage_ = create_publisher<frootspi_msgs::msg::BatteryVoltage>("kicker_voltage", 1);
  pub_switches_state_ = create_publisher<frootspi_msgs::msg::SwitchesState>("switches_state", 1);
  pub_present_wheel_velocities_ = create_publisher<frootspi_msgs::msg::WheelVelocities>(
    "present_wheel_velocities", 1);
  pub_imu_ = create_publisher<sensor_msgs::msg::Imu>("imu", 1);

  sub_dribble_power_ = create_subscription<frootspi_msgs::msg::DribblePower>(
    "dribble_power", 1, std::bind(&Driver::callback_dribble_power, this, _1));
  sub_target_wheel_velocities_ = create_subscription<frootspi_msgs::msg::WheelVelocities>(
    "target_wheel_velocities", 1, std::bind(&Driver::callback_wheel_velocities, this, _1));

  srv_kick_ =
    create_service<frootspi_msgs::srv::Kick>("kick", std::bind(&Driver::on_kick, this, _1, _2));
  srv_set_kicker_charging_ = create_service<frootspi_msgs::srv::SetKickerCharging>(
    "set_kicker_charging", std::bind(&Driver::on_set_kicker_charging, this, _1, _2));
  // srv_set_lcd_text_ = create_service<frootspi_msgs::srv::SetLCDText>(
  //   "set_lcd_text", std::bind(&Driver::on_set_lcd_text, this, _1, _2));
  srv_set_left_led_ = create_service<std_srvs::srv::SetBool>(
    "set_left_led", std::bind(&Driver::on_set_left_led, this, _1, _2));
  srv_set_center_led_ = create_service<std_srvs::srv::SetBool>(
    "set_center_led", std::bind(&Driver::on_set_center_led, this, _1, _2));
  srv_set_right_led_ = create_service<std_srvs::srv::SetBool>(
    "set_right_led", std::bind(&Driver::on_set_right_led, this, _1, _2));

  pi_ = pigpio_start(NULL, NULL);

  if (pi_ < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to connect pigpiod.");
    return CallbackReturn::FAILURE;
  }

  declare_parameter("gpio_ball_sensor", 6);
  gpio_ball_sensor_ = get_parameter("gpio_ball_sensor").get_value<int>();


  // add pigpio port setting
  // ball sensor setup
  set_mode(pi_, gpio_ball_sensor_, PI_INPUT);
  set_pull_up_down(pi_, gpio_ball_sensor_, PI_PUD_UP);

  if (!io_expander_.open(pi_)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to connect IO expander.");
    return CallbackReturn::FAILURE;
  }

  if (!battery_monitor_.open(pi_)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to connect Battery Monitor.");
    return CallbackReturn::FAILURE;
  }

  // if (!lcd_driver_.open(pi_)) {
  //   RCLCPP_ERROR(this->get_logger(), "Failed to connect LCD Driver.");
  //   return CallbackReturn::FAILURE;
  // }
  // lcd_driver_.write_texts("FrootsPi", "ﾌﾙｰﾂﾊﾟｲ!");

  if (!capacitor_monitor_.open(pi_)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to connect Capacitor Monitor.");
    return CallbackReturn::FAILURE;
  }

  // if (!front_display_communicator_.open(pi_)) {
  //   RCLCPP_ERROR(this->get_logger(), "Failed to connect Front Dislplay Communicator.");
  //   return CallbackReturn::FAILURE;
  // }

  if (!wheel_controller_.device_open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to connect wheel controller.");
    return CallbackReturn::FAILURE;
  }

  set_mode(pi_, GPIO_SHUTDOWN_SWITCH, PI_INPUT);

  // dribbler setup
  set_mode(pi_, GPIO_DRIBBLE_PWM, PI_OUTPUT);
  set_PWM_frequency(pi_, GPIO_DRIBBLE_PWM, DRIBBLE_PWM_FREQUENCY);
  set_PWM_range(pi_, GPIO_DRIBBLE_PWM, DRIBBLE_PWM_DUTY_CYCLE);
  gpio_write(pi_, GPIO_DRIBBLE_PWM, PI_HIGH);  // 負論理のためHighでモータオフ

  // kicker setup
  set_mode(pi_, GPIO_KICK_STRAIGHT, PI_OUTPUT);
  gpio_write(pi_, GPIO_KICK_STRAIGHT, PI_LOW);
  set_mode(pi_, GPIO_KICK_CHIP, PI_OUTPUT);
  gpio_write(pi_, GPIO_KICK_CHIP, PI_LOW);
  set_mode(pi_, GPIO_KICK_SUPPLY_POWER, PI_OUTPUT);
  gpio_write(pi_, GPIO_KICK_SUPPLY_POWER, PI_LOW);
  set_mode(pi_, GPIO_KICK_ENABLE_CHARGE, PI_OUTPUT);
  gpio_write(pi_, GPIO_KICK_ENABLE_CHARGE, PI_LOW);
  set_mode(pi_, GPIO_KICK_CHARGE_COMPLETE, PI_INPUT);
  set_pull_up_down(pi_, GPIO_KICK_CHARGE_COMPLETE, PI_PUD_UP);  // 外部プルアップあり

  // led setup
  set_mode(pi_, GPIO_CENTER_LED, PI_OUTPUT);
  gpio_write(pi_, GPIO_CENTER_LED, PI_LOW);
  set_mode(pi_, GPIO_RIGHT_LED, PI_OUTPUT);
  gpio_write(pi_, GPIO_RIGHT_LED, PI_LOW);

  // 通信タイムアウト検知用のタイマー
  steady_clock_ = rclcpp::Clock(RCL_STEADY_TIME);
  sub_wheel_timestamp_ = steady_clock_.now();
  timeout_has_printed_ = false;

  return CallbackReturn::SUCCESS;
}

CallbackReturn Driver::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "on_activate() is called.");

  pub_ball_detection_->on_activate();
  pub_battery_voltage_->on_activate();
  pub_ups_voltage_->on_activate();
  pub_kicker_voltage_->on_activate();
  pub_switches_state_->on_activate();
  pub_present_wheel_velocities_->on_activate();
  pub_imu_->on_activate();

  polling_timer_->reset();

  gpio_write(pi_, GPIO_KICK_SUPPLY_POWER, PI_HIGH);

  return CallbackReturn::SUCCESS;
}

CallbackReturn Driver::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "on_deactivate() is called.");

  pub_ball_detection_->on_deactivate();
  pub_battery_voltage_->on_deactivate();
  pub_ups_voltage_->on_deactivate();
  pub_kicker_voltage_->on_deactivate();
  pub_switches_state_->on_deactivate();
  pub_present_wheel_velocities_->on_deactivate();
  pub_imu_->on_deactivate();
  polling_timer_->cancel();

  gpio_write(pi_, GPIO_DRIBBLE_PWM, PI_HIGH);  // 負論理のためHighでモータオフ

  gpio_write(pi_, GPIO_KICK_ENABLE_CHARGE, PI_LOW);
  gpio_write(pi_, GPIO_KICK_SUPPLY_POWER, PI_LOW);

  return CallbackReturn::SUCCESS;
}

CallbackReturn Driver::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "on_cleanup() is called.");

  pub_ball_detection_.reset();
  pub_battery_voltage_.reset();
  pub_ups_voltage_.reset();
  pub_kicker_voltage_.reset();
  pub_switches_state_.reset();
  pub_present_wheel_velocities_.reset();
  pub_imu_.reset();
  polling_timer_.reset();

  io_expander_.close();
  capacitor_monitor_.close();
  battery_monitor_.close();
  lcd_driver_.close();
  front_display_communicator_.close();
  pigpio_stop(pi_);

  return CallbackReturn::SUCCESS;
}

CallbackReturn Driver::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "on_shutdown() is called.");

  pub_ball_detection_.reset();
  pub_battery_voltage_.reset();
  pub_ups_voltage_.reset();
  pub_kicker_voltage_.reset();
  pub_switches_state_.reset();
  pub_present_wheel_velocities_.reset();
  pub_imu_.reset();
  polling_timer_.reset();

  io_expander_.close();
  capacitor_monitor_.close();
  battery_monitor_.close();
  lcd_driver_.close();
  front_display_communicator_.close();
  pigpio_stop(pi_);

  return CallbackReturn::SUCCESS;
}

}  // namespace frootspi_hardware

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(frootspi_hardware::Driver)
