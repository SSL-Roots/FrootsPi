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
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"

using namespace std::chrono_literals;

namespace frootspi_hardware
{

static const int GPIO_SHUTDOWN_SWITCH = 23;
static const int GPIO_CENTER_LED = 14;
static const int GPIO_RIGHT_LED = 4;


Driver::Driver(const rclcpp::NodeOptions & options)
: rclcpp::Node("hardware_driver", options),
  pi_(-1)
{
  using namespace std::placeholders;  // for _1, _2, _3...

  // GPIOを定期的にread / writeするためのタイマー
  high_rate_polling_timer_ =
    create_wall_timer(1ms, std::bind(&Driver::on_high_rate_polling_timer, this));
  high_rate_polling_timer_->cancel();  // ノードがActiveになるまでタイマーオフ
  low_rate_polling_timer_ =
    create_wall_timer(1000ms, std::bind(&Driver::on_low_rate_polling_timer, this));
  low_rate_polling_timer_->cancel();  // ノードがActiveになるまでタイマーオフ

  pub_ball_detection_ = create_publisher<frootspi_msgs::msg::BallDetection>("ball_detection", 1);
  pub_battery_voltage_ = create_publisher<frootspi_msgs::msg::BatteryVoltage>("battery_voltage", 1);
  pub_ups_voltage_ = create_publisher<frootspi_msgs::msg::BatteryVoltage>("ups_voltage", 1);
  pub_kicker_voltage_ = create_publisher<frootspi_msgs::msg::BatteryVoltage>("kicker_voltage", 1);
  pub_switches_state_ = create_publisher<frootspi_msgs::msg::SwitchesState>("switches_state", 1);
  pub_present_wheel_velocities_ = create_publisher<frootspi_msgs::msg::WheelVelocities>(
    "present_wheel_velocities", 1);
  pub_imu_ = create_publisher<sensor_msgs::msg::Imu>("imu", 1);
  pub_speaker_voice_ = create_publisher<SpeakerVoice>("speaker_voice", 1);

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
  srv_enable_gain_setting_ = create_service<std_srvs::srv::SetBool>(
    "enable_gain_setting", std::bind(&Driver::on_enable_gain_setting, this, _1, _2));

  // パラメータ作成
  this->declare_parameter("wheel_gain_p", 0.009);
  this->declare_parameter("wheel_gain_i", 0.001);
  this->declare_parameter("wheel_gain_d", 0.001);

  set_parameters_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&frootspi_hardware::Driver::parametersCallback, this, std::placeholders::_1));

  pi_ = pigpio_start(NULL, NULL);

  if (pi_ < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to connect pigpiod.");
    throw std::runtime_error("Failed to connect pigpiod.");
  }

  declare_parameter("gpio_ball_sensor", 6);
  gpio_ball_sensor_ = get_parameter("gpio_ball_sensor").get_value<int>();

  // add pigpio port setting

  if (!io_expander_.open(pi_)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to connect IO expander.");
    throw std::runtime_error("Failed to connect IO expander.");
  }

  if (!battery_monitor_.open(pi_)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to connect Battery Monitor.");
    throw std::runtime_error("Failed to connect Battery Monitor.");
  }

  // if (!lcd_driver_.open(pi_)) {
  //   RCLCPP_ERROR(this->get_logger(), "Failed to connect LCD Driver.");
  //   return CallbackReturn::FAILURE;
  // }
  // lcd_driver_.write_texts("FrootsPi", "ﾌﾙｰﾂﾊﾟｲ!");

  if (!capacitor_monitor_.open(pi_)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to connect Capacitor Monitor.");
    throw std::runtime_error("Failed to connect Capacitor Monitor.");
  }

  // if (!front_display_communicator_.open(pi_)) {
  //   RCLCPP_ERROR(this->get_logger(), "Failed to connect Front Dislplay Communicator.");
  //   return CallbackReturn::FAILURE;
  // }

  if (!wheel_controller_.device_open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to connect wheel controller.");
    throw std::runtime_error("Failed to connect wheel controller.");
  }

  set_mode(pi_, GPIO_SHUTDOWN_SWITCH, PI_INPUT);

  // kicker setup
  if (!kicker_.open(pi_, gpio_ball_sensor_)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to connect kicker.");
    throw std::runtime_error("Failed to connect kicker.");
  }

  // dribbler setup
  if (!this->dribbler_.open(pi_)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to connect dribbler.");
    throw std::runtime_error("Failed to connect dribbler.");
  }

  // led setup
  set_mode(pi_, GPIO_CENTER_LED, PI_OUTPUT);
  gpio_write(pi_, GPIO_CENTER_LED, PI_LOW);
  set_mode(pi_, GPIO_RIGHT_LED, PI_OUTPUT);
  gpio_write(pi_, GPIO_RIGHT_LED, PI_LOW);

  // 通信タイムアウト検知用のタイマー
  steady_clock_ = rclcpp::Clock(RCL_STEADY_TIME);
  sub_wheel_timestamp_ = steady_clock_.now();
  timeout_has_printed_ = false;

  // polling timer reset
  high_rate_polling_timer_->reset();
  low_rate_polling_timer_->reset();
}

Driver::~Driver()
{
  high_rate_polling_timer_->cancel();
  low_rate_polling_timer_->cancel();

  // lcd_driver_.write_texts("ROS 2", "SHUTDOWN");

  io_expander_.close();
  capacitor_monitor_.close();
  battery_monitor_.close();
  lcd_driver_.close();
  front_display_communicator_.close();
  dribbler_.close();

  pigpio_stop(pi_);
}

rcl_interfaces::msg::SetParametersResult Driver::parametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  // Here update class attributes, do some actions, etc.

  WheelController::ErrorCode gain_setting_result = WheelController::ErrorCode::ERROR_NONE;

  for (auto && param : parameters) {
    if (param.get_name() == "wheel_gain_p") {
      gain_setting_result = wheel_controller_.set_p_gain(param.as_double());
      if (gain_setting_result != WheelController::ErrorCode::ERROR_NONE) {break;}
    } else if (param.get_name() == "wheel_gain_i") {
      gain_setting_result = wheel_controller_.set_i_gain(param.as_double());
      if (gain_setting_result != WheelController::ErrorCode::ERROR_NONE) {break;}
    } else if (param.get_name() == "wheel_gain_d") {
      gain_setting_result = wheel_controller_.set_d_gain(param.as_double());
      if (gain_setting_result != WheelController::ErrorCode::ERROR_NONE) {break;}
    }
  }

  if (gain_setting_result == WheelController::ErrorCode::ERROR_NONE) {
    result.successful = true;
    result.reason = "success";
  } else if (gain_setting_result == WheelController::ErrorCode::ERROR_INVALID_MODE) {
    result.successful = false;
    result.reason = "ゲイン設定モードが無効になっています";
  } else if (gain_setting_result == WheelController::ErrorCode::ERROR_CAN_SEND_FAILED) {
    result.successful = false;
    result.reason = "CAN送信に失敗しました";
  }

  return result;
}

void Driver::on_high_rate_polling_timer()
{
  // ボール検出 変化があった場合のみpublish
  bool ball_detection = gpio_read(pi_, gpio_ball_sensor_);
  if (ball_detection != this->latest_ball_detection_) {
    auto ball_detection_msg = std::make_unique<frootspi_msgs::msg::BallDetection>();
    ball_detection_msg->detected = ball_detection;
    pub_ball_detection_->publish(std::move(ball_detection_msg));
    // ボール検出をした場合に音声再生
    if (ball_detection) {
      publish_speaker_voice(SpeakerVoice::VOICE_BALL_CATCH);
    }
  }
  this->latest_ball_detection_ = ball_detection;
  front_indicate_data_.Parameter.BallSens = ball_detection;

  // スイッチ状態をRead 変化があった場合のみpublish
  bool pushed_button0, pushed_button1, pushed_button2, pushed_button3, turned_on_dip0,
    turned_on_dip1;
  io_expander_.read(
    pushed_button0, pushed_button1, pushed_button2, pushed_button3,
    turned_on_dip0, turned_on_dip1);
  // シャットダウンスイッチは負論理なので、XORでビット反転させる
  bool pushed_shutdown = gpio_read(pi_, GPIO_SHUTDOWN_SWITCH) ^ 1;
  if (pushed_button0 != this->latest_pushed_button0_ ||
    pushed_button1 != this->latest_pushed_button1_ ||
    pushed_button2 != this->latest_pushed_button2_ ||
    pushed_button3 != this->latest_pushed_button3_ ||
    turned_on_dip0 != this->latest_turned_on_dip0_ ||
    turned_on_dip1 != this->latest_turned_on_dip1_ ||
    pushed_shutdown != this->latest_pushed_shutdown_)
  {
    auto switches_state_msg = std::make_unique<frootspi_msgs::msg::SwitchesState>();
    switches_state_msg->pushed_button0 = pushed_button0;
    switches_state_msg->pushed_button1 = pushed_button1;
    switches_state_msg->pushed_button2 = pushed_button2;
    switches_state_msg->pushed_button3 = pushed_button3;
    switches_state_msg->turned_on_dip0 = turned_on_dip0;
    switches_state_msg->turned_on_dip1 = turned_on_dip1;
    switches_state_msg->pushed_shutdown = pushed_shutdown;
    pub_switches_state_->publish(std::move(switches_state_msg));

    // 情報更新
    this->latest_pushed_button0_ = pushed_button0;
    this->latest_pushed_button1_ = pushed_button1;
    this->latest_pushed_button2_ = pushed_button2;
    this->latest_pushed_button3_ = pushed_button3;
    this->latest_turned_on_dip0_ = turned_on_dip0;
    this->latest_turned_on_dip1_ = turned_on_dip1;
    this->latest_pushed_shutdown_ = pushed_shutdown;

    /**
     * デバッグモード設定
    */
    // 車輪
    if (pushed_button3) {
      this->wheel_controller_.set_mode(WheelController::DEBUG_MODE);
      this->wheel_controller_.debug_set_wheel_velocities(10.0, 10.0, 10.0);
    } else {
      this->wheel_controller_.debug_set_wheel_velocities(0.0, 0.0, 0.0);
      this->wheel_controller_.set_mode(WheelController::NORMAL_MODE);
    }

    // ドリブラー
    if (pushed_button2) {
      this->dribbler_.enableDebugMode();
      this->dribbler_.debugDrive(1.0);
    } else {
      this->dribbler_.debugDrive(0.0);
      this->dribbler_.diasbleDebugMode();
    }

    // キッカー
    if (pushed_button1) {
      this->kicker_.enableDebugMode();
      this->kicker_.debugEnableCharging();
    } else {
      this->kicker_.debugDisableCharging();
      this->kicker_.disableDebugMode();
    }

    // 放電スイッチ
    if (pushed_button0) {
      this->kicker_.discharge();
    }
  }

  // オムニホイール回転速度をパブリッシュ
  // TODO(Roots): implement

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
      // 音声再生のためにpublish
      publish_speaker_voice(SpeakerVoice::VOICE_COMM_DISCONNECT);
    }
  } else {
    timeout_has_printed_ = false;
    // 直前の状態がタイムアウトだった場合
    if (timeout_has_printed_) {
      publish_speaker_voice(SpeakerVoice::VOICE_COMM_CONNECT);
    }
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


void Driver::on_low_rate_polling_timer()
{
  // バッテリー電圧をパブリッシュ
  auto battery_voltage_msg = std::make_unique<frootspi_msgs::msg::BatteryVoltage>();
  battery_monitor_.main_battery_info_read(
    battery_voltage_msg->voltage, battery_voltage_msg->voltage_status);
  front_indicate_data_.Parameter.BatVol = (unsigned char)(battery_voltage_msg->voltage * 10);

  // バッテリー電圧が低い場合
  if (battery_voltage_msg->voltage_status ==
    frootspi_msgs::msg::BatteryVoltage::BATTERY_VOLTAGE_STATUS_TOO_LOW)
  {
    publish_speaker_voice(SpeakerVoice::VOICE_BATTERY_LOW);
  }

  pub_battery_voltage_->publish(std::move(battery_voltage_msg));


  // UPS(無停電電源装置)電圧をパブリッシュ
  auto ups_voltage_msg = std::make_unique<frootspi_msgs::msg::BatteryVoltage>();
  battery_monitor_.sub_battery_info_read(
    ups_voltage_msg->voltage, ups_voltage_msg->voltage_status);
  pub_ups_voltage_->publish(std::move(ups_voltage_msg));

  // キッカー（昇圧回路）電圧をパブリッシュ
  auto kicker_voltage_msg = std::make_unique<frootspi_msgs::msg::BatteryVoltage>();
  capacitor_monitor_.capacitor_info_read(
    kicker_voltage_msg->voltage, kicker_voltage_msg->voltage_status);
  front_indicate_data_.Parameter.CapVol = (unsigned char)(kicker_voltage_msg->voltage);
  if (kicker_voltage_msg->voltage_status >=
    frootspi_msgs::msg::BatteryVoltage::BATTERY_VOLTAGE_STATUS_OK)
  {
    front_indicate_data_.Parameter.CapacitorSta = true;
  } else {
    front_indicate_data_.Parameter.CapacitorSta = false;
  }
  pub_kicker_voltage_->publish(std::move(kicker_voltage_msg));
}


void Driver::callback_dribble_power(const frootspi_msgs::msg::DribblePower::SharedPtr msg)
{
  double power = msg->power;
  if (power > 1.0) {
    power = 1.0;
  } else if (power < 0.0) {
    power = 0.0;
  }

  if (power > 0) {
    front_indicate_data_.Parameter.DribbleReq = true;
  } else {
    front_indicate_data_.Parameter.DribbleReq = false;
  }

  this->dribbler_.drive(power);
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
  front_indicate_data_.Parameter.KickReq = true;

  if (request->kick_type == frootspi_msgs::srv::Kick::Request::KICK_TYPE_STRAIGHT) {
    bool kick_result = kicker_.kickStraight(request->kick_power * 1000);

    if (!kick_result) {
      RCLCPP_ERROR(this->get_logger(), "Failed to add wave.");
      response->success = false;
      response->message = "キック処理に失敗しました";
      return;
    }

    response->success = true;
    publish_speaker_voice(SpeakerVoice::VOICE_KICK_EXECUTE);

  } else if (request->kick_type == frootspi_msgs::srv::Kick::Request::KICK_TYPE_CHIP) {
    // チップキックは未実装
    response->success = false;
    response->message = "チップキックデバイスが搭載されていません";

  } else if (request->kick_type == frootspi_msgs::srv::Kick::Request::KICK_TYPE_DISCHARGE) {
    // 放電
    kicker_.discharge();
    RCLCPP_INFO(this->get_logger(), "キッカーの充電停止.");  // 充電停止しているので通知

    response->success = true;
    response->message = "放電を開始しました";

    // 放電開始時に音声再生
    publish_speaker_voice(SpeakerVoice::VOICE_KICK_DISCHARGE);
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
  if (request->start_charging) {
    kicker_.enableCharging();
    RCLCPP_INFO(this->get_logger(), "キッカーの充電開始.");
    response->message = "充電を開始しました";
    front_indicate_data_.Parameter.ChargeReq = true;

    // 充電開始時に音声再生
    publish_speaker_voice(SpeakerVoice::VOICE_CHARGER_START);
  } else {
    kicker_.disableCharging();
    RCLCPP_INFO(this->get_logger(), "キッカーの充電停止.");
    response->message = "充電を停止しました";
    front_indicate_data_.Parameter.ChargeReq = false;

    // 充電停止時に音声再生
    publish_speaker_voice(SpeakerVoice::VOICE_CHARGER_STOP);
  }

  response->success = true;
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

void Driver::on_enable_gain_setting(
  const std_srvs::srv::SetBool::Request::SharedPtr request,
  std_srvs::srv::SetBool::Response::SharedPtr response)
{
  if (request->data) {
    // ゲイン設定を有効にする
    WheelController::ErrorCode error_code = wheel_controller_.set_mode(
      WheelController::GAIN_SETTING_MODE);
    if (error_code == WheelController::ErrorCode::ERROR_NONE) {
      response->success = true;
      response->message = "ゲイン設定モードを有効にしました。車輪が回らなくなります。";
    } else if (error_code == WheelController::ErrorCode::ERROR_WHEELS_ARE_MOVING) {
      response->success = false;
      response->message = "車輪が回転しているため、ゲイン設定モードを有効にできませんでした。";
    }
  } else {
    // ゲイン設定を無効にする
    WheelController::ErrorCode error_code =
      wheel_controller_.set_mode(WheelController::NORMAL_MODE);
    if (error_code == WheelController::ErrorCode::ERROR_NONE) {
      response->success = true;
      response->message = "ゲイン設定モードを無効にしました。車輪が回せます。";
    } else {
      response->success = false;
      response->message = "ゲイン設定モードを無効にできませんでした。";
    }
  }
}

void Driver::publish_speaker_voice(const uint8_t & voice_type)
{
  auto voice_msg = std::make_unique<SpeakerVoice>();
  voice_msg->voice_type = voice_type;
  pub_speaker_voice_->publish(std::move(voice_msg));
}

}  // namespace frootspi_hardware

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(frootspi_hardware::Driver)
