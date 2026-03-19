#define EINK_PARTIAL 0
// Настройки делителя напряжения
#define R1_POWER 2000 // Ом (верхний резистор)
#define R2_POWER 2000 // Ом (нижний резистор)

// Настройки делителя напряжения
#define R1_BATTERY 1000 // Ом (верхний резистор)
#define R2_BATTERY 3300 // Ом (нижний резистор)

// Диапазон напряжений для преобразования в проценты
#define BATTERY_VOLTAGE_MIN 3.1f // Минимальное напряжение в диапазоне (В)
#define BATTERY_VOLTAGE_MAX 4.1f // Максимальное напряжение в диапазоне (В)

// Диапазон напряжений для преобразования в проценты
#define POWER_VOLTAGE_MIN 3.0f // Минимальное напряжение в диапазоне (В)
#define POWER_VOLTAGE_MAX 5.0f // Максимальное напряжение в диапазоне (В)

// Диапазон напряжений для преобразования в проценты
#define RTC_VOLTAGE_MIN 2.1f // Минимальное напряжение в диапазоне (В)
#define RTC_VOLTAGE_MAX 3.0f // Максимальное напряжение в диапазоне (В)

#define ADC_MAX_VALUE 4095.0f
#define VOLTAGE_VREF 3.3f

#define ADC_CHANNEL_COUNT 3

#define TEMPERATURE_MAX 25.0f
#define TEMPERATURE_MIN 15.0f

#define HUMIDITY_MAX 50.0f
#define HUMIDITY_MIN 15.0f

#define PRESSURE_MAX 770.0f
#define PRESSURE_MIN 720.0f