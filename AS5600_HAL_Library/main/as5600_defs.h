/**
 * \file        as5600_defs.h
 * \brief
 * \details
 * \author      MaverickST
 * \version     0.0.3
 * \date        05/10/2023
 * \copyright   Unlicensed
 */

/**
 * @brief Register addresses for the AS5600 sensor
 */
typedef enum 
{
    AS5600_REG_ZMCO = 0x00,     ///< ZMCO shows how many times ZPOS and MPOS have been permanently written.
    AS5600_REG_ZPOS_H = 0x01,     /*!< Zero-Position Offset */
    AS5600_REG_ZPOS_L = 0x02,     /*!< Zero-Position Offset */
    AS5600_REG_MPOS_H = 0x03,     /*!< Magnet Position */
    AS5600_REG_MPOS_L = 0x04,     /*!< Magnet Position */
    AS5600_REG_MANG_H = 0x05,     /*!< Magnet Angle */
    AS5600_REG_MANG_L = 0x06,     /*!< Magnet Angle */
    AS5600_REG_CONF_H = 0x07,     /*!< Configuration */
    AS5600_REG_CONF_L = 0x08,     /*!< Configuration */
    AS5600_REG_STATUS = 0x0B,     /*!< Status */
    AS5600_REG_RAW_ANGLE_H = 0x0C,     /*!< Raw Angle */
    AS5600_REG_RAW_ANGLE_L = 0x0D,     /*!< Raw Angle */
    AS5600_REG_ANGLE_H = 0x0E,     /*!< Angle */
    AS5600_REG_ANGLE_L = 0x0F,     /*!< Angle */
    AS5600_REG_AGC = 0x1A,     /*!< Automatic Gain Control */
    AS5600_REG_MAGNITUDE_H = 0x1B,     /*!< Magnitude */
    AS5600_REG_MAGNITUDE_L = 0x1C,     /*!< Magnitude */
    AS5600_REG_BURN = 0xFF     /*!< Burn */
} AS5600_reg_t;


/**
 * @brief Power modes for PM bitfield at the CONF register
 * 
 */
typedef enum 
{
    AS5600_POWER_MODE_NOM = 0x00,    /*!< Normal mode */
    AS5600_POWER_MODE_LPM1 = 0x01,   /*!< Low power mode 1 */
    AS5600_POWER_MODE_LPM2 = 0x02,   /*!< Low power mode 2 */
    AS5600_POWER_MODE_LPM3 = 0x03,   /*!< Low power mode 3 */
    AS5600_POWER_MODE_COUNT = 0x04   /*!< Number of power modes */
} AS5600_power_mode_t;

/**
 * @brief PWM frequency for PW bitfield at the CONF register
 */
typedef enum
{
    AS5600_HYSTERESIS_OFF = 0x00,   /*!< Hysteresis off */
    AS5600_HYSTERESIS_1LSB = 0x01,  /*!< Hysteresis 1LSB */
    AS5600_HYSTERESIS_2LSB = 0x02,  /*!< Hysteresis 2LSB */
    AS5600_HYSTERESIS_3LSB = 0x03,  /*!< Hysteresis 3LSB */
    AS5600_HYSTERESIS_COUNT = 0x04  /*!< Number of hysteresis modes */
} AS5600_hysteresis_t;

/**
 * @brief Output stage types for OUTS bitfield at the CONF register
 */
typedef enum
{
    AS5600_OUTPUT_STAGE_ANALOG_FR = 0x00, ///< Analog output 0%-100%, FULL RANGE
    AS5600_OUTPUT_STAGE_ANALOG_RR = 0x01, ///< Analog output 10%-90%, REDUCED RANGE
    AS5600_OUTPUT_STAGE_DIGITAL_PWM = 0x02, ///< PWM output
    AS5600_OUTPUT_STAGE_COUNT = 0x03 ///< Number of output stages
} AS5600_output_stage_t;

/**
 * @brief Allowed PWM output frequencies at the PWMF bitfield at the CONF register
 */
typedef enum
{
    AS5600_PWM_FREQUENCY_115HZ = 0x00, ///< 115Hz
    AS5600_PWM_FREQUENCY_230HZ = 0x01, ///< 230Hz
    AS5600_PWM_FREQUENCY_460HZ = 0x02, ///< 460Hz
    AS5600_PWM_FREQUENCY_920HZ = 0x03, ///< 920Hz
    AS5600_PWM_FREQUENCY_COUNT = 0x04  ///< Number of PWM frequencies
} AS5600_pwm_frequency_t;

/**
 * @brief Slow filter step response delays for SF bitfield at CONF register.
 */
typedef enum
{
    AS5600_SLOW_FILTER_16X = 0x00,  ///< 16x
    AS5600_SLOW_FILTER_8X = 0x01,   ///< 8x
    AS5600_SLOW_FILTER_4X = 0x02,   ///< 4x
    AS5600_SLOW_FILTER_2X = 0x03,   ///< 2x
    AS5600_SLOW_FILTER_COUNT = 0x04 ///< Number of slow filter steps
} AS5600_slow_filter_t;

/**
 * @brief Fast filter threshold options for FF bitfield at CONF register.
 * 
 * For a fast step response and low noise after settling, the fast 
 * filter can be enabled
 * 
 * The fast filter works only if the input variation is greater than the fast filter threshold, 
 * otherwise the output response is determined only by the slow filter
 */
typedef enum
{
    AS5600_FF_THRESHOLD_SLOW_FILTER_ONLY = 0x00, ///< Slow filter only
    AS5600_FF_THRESHOLD_6LSB = 0x01, ///< 6LSB
    AS5600_FF_THRESHOLD_7LSB = 0x02, ///< 7LSB
    AS5600_FF_THRESHOLD_9LSB = 0x03, ///< 9LSB
    AS5600_FF_THRESHOLD_18LSB = 0x04, ///< 18LSB
    AS5600_FF_THRESHOLD_21LSB = 0x05, ///< 21LSB
    AS5600_FF_THRESHOLD_24LSB = 0x06, ///< 24LSB
    AS5600_FF_THRESHOLD_10LSB = 0x07, ///< 10LSB
    AS5600_FF_THRESHOLD_COUNT = 0x08 ///< Number of fast filter thresholds
} AS5600_ff_threshold_t;

/**
 * @brief BURN register commands
 */
typedef enum
{
    AS5600_BURN_MODE_BURN_SETTING = 0x40U, ///< Command for burning a setting configuration
    AS5600_BURN_MODE_BURN_ANGLE = 0x80U, ///< Command for burning start and end angles
    AS5600_BURN_MODE_COUNT, ///< Number of burn modes
} AS5600_burn_mode_t;

/**
 * @brief Watchdog options for WD bitfield at CONF register
 */
typedef enum
{
    AS5600_WATCHDOG_OFF = 0x00, ///< Watchdog off
    AS5600_WATCHDOG_ON = 0x01, ///< Watchdog on
    AS5600_WATCHDOG_COUNT = 0x02 ///< Number of watchdog options
} AS5600_watchdog_t;

/**
 * @brief Configuration bitfield for the CONF register
 */
typedef union
{
    uint16_t WORD;
	struct{
		AS5600_power_mode_t     PM     :2; ///< Power mode: 00 - NOM, 01 - LPM1, 10 - LPM2, 11 - LPM3
        AS5600_hysteresis_t     HYST   :2; ///< Hysteresis: 00 - OFF, 01 - 1LSB, 10 - 2LSB, 11 - 3LSB
        AS5600_output_stage_t   OUTS   :2; ///< Output stage: 00 - analog(0%-100%), 01 - analog(10%-90%), 10 - PWM 
        AS5600_pwm_frequency_t  PWMF   :2; ///< PWM frequency: 00 - 115Hz, 01 - 230Hz, 10 - 460Hz, 11 - 920Hz
        AS5600_slow_filter_t    SF     :2; ///< Slow filter: 00 - 16x, 01 - 8x, 10 - 4x, 11 - 2x
        AS5600_ff_threshold_t   FTH    :3; ///< FASt filter threshold
        AS5600_watchdog_t       WD     :1; ///< Watchdog: 0 - disabled, 1 - enabled
		uint16_t                       :2;
	};
} AS5600_config_t;

///< BITFIELD CONTANT VALUES
#define kAS5600_CONF_PM_NOM      0x00
#define kAS5600_CONF_PM_LPM1     0x01
#define kAS5600_CONF_PM_LPM2     0x02
#define kAS5600_CONF_PM_LPM3     0x03

#define kAS5600_CONF_HYST_OFF    0x00
#define kAS5600_CONF_HYST_1LSB   0x01
#define kAS5600_CONF_HYST_2LSB   0x02
#define kAS5600_CONF_HYST_3LSB   0x03

#define kAS5600_CONF_OUTS_ANALOG_0_100  0x00
#define kAS5600_CONF_OUTS_ANALOG_10_90  0x01
#define kAS5600_CONF_OUTS_PWM           0x02

#define kAS5600_CONF_PWMF_115HZ     0x00
#define kAS5600_CONF_PWMF_230HZ     0x01
#define kAS5600_CONF_PWMF_460HZ     0x02
#define kAS5600_CONF_PWMF_920HZ     0x03

#define kAS5600_CONF_SF_16X     0x00
#define kAS5600_CONF_SF_8X      0x01
#define kAS5600_CONF_SF_4X      0x02
#define kAS5600_CONF_SF_2X      0x03

#define kAS5600_CONF_FTH_SFO    0x00
#define kAS5600_CONF_FTH_1LSB   0x01
#define kAS5600_CONF_FTH_2LSB   0x02
#define kAS5600_CONF_FTH_3LSB   0x03
#define kAS5600_CONF_FTH_4LSB   0x04
#define kAS5600_CONF_FTH_5LSB   0x05
#define kAS5600_CONF_FTH_6LSB   0x06
#define kAS5600_CONF_FTH_7LSB   0x07

#define kAS5600_CONF_WD_OFF     0x00
#define kAS5600_CONF_WD_ON      0x01