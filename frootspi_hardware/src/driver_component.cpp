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
: rclcpp::Node("hardware_driver", options),
  pi_(-1), enable_kicker_charging_(false), discharge_kick_count_(0)
{
  using namespace std::placeholders;  // for _1, _2, _3...

  // GPIOを定期的にread / writeするためのタイマー
  high_rate_polling_timer_ =
    create_wall_timer(1ms, std::bind(&Driver::on_high_rate_polling_timer, this));
  high_rate_polling_timer_->cancel();  // ノードがActiveになるまでタイマーオフ
  low_rate_polling_timer_ =
    create_wall_timer(1000ms, std::bind(&Driver::on_low_rate_polling_timer, this));
  low_rate_polling_timer_->cancel();  // ノードがActiveになるまでタイマーオフ
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
  // ball sensor setup
  set_mode(pi_, gpio_ball_sensor_, PI_INPUT);
  set_pull_up_down(pi_, gpio_ball_sensor_, PI_PUD_UP);

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

  // polling timer reset
  high_rate_polling_timer_->reset();
  low_rate_polling_timer_->reset();

  gpio_write(pi_, GPIO_KICK_SUPPLY_POWER, PI_HIGH);
}

Driver::~Driver()
{
  high_rate_polling_timer_->cancel();
  low_rate_polling_timer_->cancel();

  gpio_write(pi_, GPIO_DRIBBLE_PWM, PI_HIGH);  // 負論理のためHighでモータオフ
  // lcd_driver_.write_texts("ROS 2", "SHUTDOWN");
  gpio_write(pi_, GPIO_KICK_ENABLE_CHARGE, PI_LOW);
  gpio_write(pi_, GPIO_KICK_SUPPLY_POWER, PI_LOW);

  io_expander_.close();
  capacitor_monitor_.close();
  battery_monitor_.close();
  lcd_driver_.close();
  front_display_communicator_.close();

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
  } else if (gain_setting_result == WheelController::ErrorCode::ERROR_GAIN_SETTING_MODE_DISABLED) {
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

  if (power > 0) {
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
  const int MAX_SLEEP_TIME_USEC_FOR_STRAIGHT = 5000;

  front_indicate_data_.Parameter.KickReq = true;

  if (request->kick_type == frootspi_msgs::srv::Kick::Request::KICK_TYPE_STRAIGHT) {
    // ストレートキック
    uint32_t sleep_time_usec = 673 * request->kick_power + 100;  // constants based on test
    if (sleep_time_usec > MAX_SLEEP_TIME_USEC_FOR_STRAIGHT) {
      sleep_time_usec = MAX_SLEEP_TIME_USEC_FOR_STRAIGHT;
    }

    // GPIOをHIGHにしている時間を変化させて、キックパワーを変更する
    uint32_t bit_kick_straight = 1 << GPIO_KICK_STRAIGHT;
    uint32_t bit_kick_enable_charge = 1 << GPIO_KICK_ENABLE_CHARGE;
    uint32_t bit_kick_enable_charge_masked = enable_kicker_charging_ ? bit_kick_enable_charge : 0;

    gpioPulse_t pulses[] = {
      {0, bit_kick_enable_charge, 100},                                           // 充電を停止する
      {bit_kick_straight, 0, sleep_time_usec},                                    // キックON
      {0, bit_kick_straight, 100},                                                // キックOFF
      {bit_kick_enable_charge_masked, 0, 0},                                      // 充電を再開する
    };

    wave_clear(pi_);
    int num_pulse = wave_add_generic(pi_, sizeof(pulses) / sizeof(gpioPulse_t), pulses);
    if (num_pulse < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to add wave.");
      response->success = false;
      response->message = "キック処理に失敗しました";
      return;
    }

    int wave_id = wave_create(pi_);
    if (wave_id < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create wave.");
      response->success = false;
      response->message = "キック処理に失敗しました";
      return;
    }

    int result = wave_send_once(pi_, wave_id);
    if (result < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to send wave.");
      response->success = false;
      response->message = "キック処理に失敗しました";
      return;
    }

    response->success = true;
    response->message = std::to_string(sleep_time_usec) + " ミリ秒間ソレノイドをONしました";

    publish_speaker_voice(SpeakerVoice::VOICE_KICK_EXECUTE);

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
  // キッカーの充電はキック処理時にON/OFFされるので、
  // enable_kicker_charging_ 変数で充電フラグを管理する
  if (request->start_charging) {
    gpio_write(pi_, GPIO_KICK_ENABLE_CHARGE, PI_HIGH);
    enable_kicker_charging_ = true;
    RCLCPP_INFO(this->get_logger(), "キッカーの充電開始.");
    response->message = "充電を開始しました";
    front_indicate_data_.Parameter.ChargeReq = true;

    // 充電開始時に音声再生
    publish_speaker_voice(SpeakerVoice::VOICE_CHARGER_START);
  } else {
    gpio_write(pi_, GPIO_KICK_ENABLE_CHARGE, PI_LOW);
    enable_kicker_charging_ = false;
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
    WheelController::ErrorCode error_code = wheel_controller_.enable_gain_setting();
    if (error_code == WheelController::ErrorCode::ERROR_NONE) {
      response->success = true;
      response->message = "ゲイン設定モードを有効にしました。車輪が回らなくなります。";
    } else if (error_code == WheelController::ErrorCode::ERROR_WHEELS_ARE_MOVING) {
      response->success = false;
      response->message = "車輪が回転しているため、ゲイン設定モードを有効にできませんでした。";
    }
  } else {
    // ゲイン設定を無効にする
    WheelController::ErrorCode error_code = wheel_controller_.disable_gain_setting();
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
