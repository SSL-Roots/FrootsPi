# バッテリー電圧状態の定義
uint8 BATTERY_VOLTAGE_STATUS_UNKNOWN = 0
uint8 BATTERY_VOLTAGE_STATUS_TOO_LOW = 1
uint8 BATTERY_VOLTAGE_STATUS_FULL = 2

# バッテリー電圧 [V]
float32 voltage

# 電圧状態
uint8 voltage_status
