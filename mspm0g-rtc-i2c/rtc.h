/*
 * rtc.h
 *
 *  Created on: Oct 27, 2022
 *      Author: munan
 */

#ifndef RTC_H_
#define RTC_H_

#include <stdint.h>
/*!****************************************************************************
 *  @file       rtc.h
 *  @brief      Real Time Clock (RTC) HAL Interface
 *  @defgroup   RTC Real Time Clock (RTC)
 *
 *  # Overview
 *  The RTC hal layer provides a layer of abstraction above the base MSPM0 RTC
 *  Although the MSPM0 RTC has several native hardware features to take advantage of
 *  For maximum flexibility we implement a HAL in case the code ever needs to be ported
 *  to a platform without a hardware RTC or if the feature set is slightly different.
 *
 *  <hr>
 ******************************************************************************
 */
/**
 *  RTC interface header
 *  	Functions associated with setting and configuring the RTC. 
 * 		This set of functions is used to interface with the hardware RTC on MSPM0
 * 		CHANGED define RTC struct/ object <br>
 *		TODO create set RTC function (superset) <br>
 *		TODO create set rtc_wake_int function <br>
 *		TODO create set intcfg function <br>
 *		TODO create clear intflags function <br>
 *		TODO create set outcfg function <br>
 *		TODO create set RTC seconds function <br>
 *		TODO create set RTC minutes function <br>
 *		TODO create set RTC hours function <br>
 * 		TODO create set RTC date function <br>
 *		TODO create set RTC month function <br>
 *		TODO create set RTC year function <br>
 *		TODO create set RTC weekday function <br>
 * 		TODO create set interval alarm function <br>
 *		TODO create set wake alarm function (target time/ date) <br>
 *		TODO create set RTC CLKOUT (enable or disable) <br>
 * 		TODO create get RTC function (superset) <br>
 *		TODO create get RTC seconds function <br>
 *		TODO create get RTC minutes function <br>
 *		TODO create get RTC hours function <br>
 *		TODO create get RTC date function <br>
 *		TODO create get RTC month function <br>
 *		TODO create get RTC year function <br>
 *		TODO create get RTC weekday function <br>
 *		TODO create get interval alarm function <br>
 *		TODO create get wake alarm function <br>
 *		TODO create get RTC CLKOUT <br>
 *		TODO create function to assert/ deassert wake  <br>
 *		TODO create RTC ISR that manages updating RTC object and alarms <br>
**/
/** @addtogroup RTC
 * @{
 */

/* Register address map
 *
 */

/**
 * @brief RTC ID register
 */
#define RTC_ID 						0xDA
#define RTC_REG_RTC_ID				0x00
#define RTC_REG_RTC_WAKE_INT		0x01
#define RTC_REG_RTC_INTCFG_H		0x02
#define RTC_REG_RTC_INTCFG_L		0x03
#define RTC_REG_RTC_INTCFG			RTC_REG_RTC_INTCFG_H
#define RTC_REG_RTC_INTFLAGS_H		0x04
#define RTC_REG_RTC_INTFLAGS_L		0x05
#define RTC_REG_RTC_INTFLAGS		RTC_REG_RTC_INTFLAGS_H
#define RTC_REG_RTC_OUTCFG			0x06
#define RTC_REG_RTC_SECONDS			0x07
#define RTC_REG_RTC_MINUTES			0x08
#define RTC_REG_RTC_HOURS			0x09
#define RTC_REG_RTC_DATE			0x0A
#define RTC_REG_RTC_MONTH			0x0B
#define RTC_REG_RTC_YEAR_H			0x0C
#define RTC_REG_RTC_YEAR_L			0x0D
#define RTC_REG_RTC_YEAR			RTC_REG_RTC_YEAR_H
#define RTC_REG_RTC_WEEKDAY			0x0E
#define RTC_REG_RTC_ALARM_INT_MSB	0x0F
#define RTC_REG_RTC_ALARM_INT_LSB	0x12
#define RTC_REG_RTC_ALARM_INT		RTC_REG_RTC_ALARM_INT_MSB
#define RTC_REG_RTC_SECONDS_ALARM	0x13
#define RTC_REG_RTC_MINUTES_ALARM	0x14
#define RTC_REG_RTC_HOURS_ALARM		0x15
#define RTC_REG_RTC_DATE_ALARM		0x16
#define RTC_REG_RTC_MONTH_ALARM		0x17
#define RTC_REG_RTC_YEAR_ALARM_H	0x18
#define RTC_REG_RTC_YEAR_ALARM_L	0x19
#define RTC_REG_RTC_YEAR_ALARM		RTC_REG_RTC_YEAR_ALARM_H
#define RTC_REG_RTC_WEEKDAY_ALARM	0x20

/**
 * @brief RTC global interrupt enable/ flag
**/
#define RTC_INT_G_ENABLE 		0x8000

/**
 * @brief RTC interval alarm interrupt enable/ flag
**/
#define RTC_INT_INT_ALARM		0x0800

/**
 * @brief RTC interval alarm one shot mode select
**/
#define RTC_INT_INT_ALARM_ONESHOT_MODE 0x0400

/**
 * @brief RTC second interrupt enable/ flag
**/
#define RTC_INT_SEC_ALARM		0x0001

/**
 * @brief RTC minute interrupt enable/ flag
**/
#define RTC_INT_MIN_ALARM		0x0002

/**
 * @brief RTC hour interrupt enable/ flag
**/
#define RTC_INT_HOUR_ALARM		0x0004

/**
 * @brief RTC day of month interrupt enable/ flag
**/
#define RTC_INT_DATE_ALARM		0x0008

/**
 * @brief RTC month alarm interrupt enable/ flag
**/
#define RTC_INT_MONTH_ALARM		0x0010

/**
 * @brief RTC year alarm interrupt enable/ flag
**/
#define RTC_INT_YEAR_ALARM		0x0020

/**
 * @brief RTC weekday interrupt enable/ flag
**/
#define RTC_INT_WEEKDAY_ALARM	0x0040

#define RTC_INT_ALL (RTC_INT_G_ENABLE | RTC_INT_INT_ALARM | \
					RTC_INT_INT_ALARM_ONESHOT_MODE | RTC_INT_SEC_ALARM | \
					RTC_INT_MIN_ALARM | RTC_INT_HOUR_ALARM | \
					RTC_INT_DATE_ALARM | RTC_INT_MONTH_ALARM | \
					RTC_INT_YEAR_ALARM | RTC_INT_WEEKDAY_ALARM)

#define RTC_BOUNDS_SECONDS		60
#define RTC_BOUNDS_MINUTES		60
#define RTC_BOUNDS_HOURS		24

#define RTC_BOUNDS_WAKEINT_MIN	(-1)
#define RTC_BOUNDS_WAKEINT_MAX	(12)

#define RTC_BOUNDS_DATE_JAN		    31
#define RTC_BOUNDS_DATE_FEB		    28
#define RTC_BOUNDS_DATE_FEB_LEAP    29
#define RTC_BOUNDS_DATE_MAR		    31
#define RTC_BOUNDS_DATE_APR		    30
#define RTC_BOUNDS_DATE_MAY		    31
#define RTC_BOUNDS_DATE_JUN		    30
#define RTC_BOUNDS_DATE_JUL		    31
#define RTC_BOUNDS_DATE_AUG		    31
#define RTC_BOUNDS_DATE_SEP		    30
#define RTC_BOUNDS_DATE_OCT		    31
#define RTC_BOUNDS_DATE_NOV		    30
#define RTC_BOUNDS_DATE_DEC		    31
#define RTC_BOUNDS_MONTH		    12
#define RTC_BOUNDS_WEEKDAY		    7

#define RTC_BOUNDS_OUTCFG_MAX	9
/**
 * Enum for RTC status
**/
typedef enum {
	RTC_S_OK,				/*!< RTC nominal return code */
	RTC_S_INVALID_PARAM,	/*!< RTC parameter is out of bounds or otherwise invalid */
	RTC_S_ERR				/*!< Not currently used */
} RTC_Status_t;

typedef enum {
	RTC_INT_OUT_DISABLED	= 0,		/*!< RTC interrupt output is disabled */
	RTC_INT_OUT_POS_1_TICK	= 1,		/*!< RTC interrupt output active high generates a pulse for 1 RTC tick */
	RTC_INT_OUT_NEG_1_TICK	= 2, 	/*!< RTC interrupt output active low generates a pulse for 1 RTC tick */
	RTC_INT_OUT_POS_2_TICKS = 3,	/*!< RTC interrupt output active high generates a pulse for 2 RTC ticks */
	RTC_INT_OUT_NEG_2_TICKS = 4,	/*!< RTC interrupt output active low generates a pulse for 2 RTC ticks */
	RTC_INT_OUT_POS_4_TICKS = 5,	/*!< RTC interrupt output active high generates a pulse for 4 RTC ticks */
	RTC_INT_OUT_NEG_4_TICKS = 6,	/*!< RTC interrupt output active low generates a pulse for 4 RTC ticks */
	RTC_INT_OUT_POS_8_TICKS = 7,	/*!< RTC interrupt output active high generates a pulse for 8 RTC ticks */
	RTC_INT_OUT_NEG_8_TICKS = 8 	/*!< RTC interrupt output active low generates a pulse for 8 RTC ticks */
} RTC_OutCfg_t;

/**
 * Type to define the day of week
**/
typedef enum  {
	sun = 0,			/*!< Sunday */
	mon = 1,			/*!< Monday */
	tue = 2,			/*!< Tuesday */
	wed = 3,			/*!< Wednesday */
	thu = 4,			/*!< Thursday */
	fri = 5,			/*!< Friday */
	sat = 6    			/*!< Saturday */
} weekday_t;

/**
 * Type to define Month
**/
typedef enum {
	jan = 0,			/*!< January */
	feb = 1, 			/*!< February */
	mar = 2, 			/*!< March */
	apr = 3, 			/*!< April */
	may = 4, 			/*!< May */
	jun = 5, 			/*!< June */
	jul = 6, 			/*!< July */
	aug = 7, 			/*!< August */
	sep = 8, 			/*!< September */
	oct = 9, 			/*!< October */
	nov = 10,			/*!< November */
	dec = 11 			/*!< December */
} month_t;

/**
 * RTC object containing current time and alarm information
 * 
**/
typedef struct s_rtc_t
{
	int8_t rtc_id;			/*!< RTC version ID */
	int8_t rtc_wake_int;	/*!< Internal wakeup interval for RTC/ RTC object will update based on this value
									value in power 2 Hz increments (-1 to 12) 
									to reflect a wakeup range of 0.5Hz to 4096 Hz. */
	uint16_t intcfg;		/*!< Interrupt config register */
	uint16_t intflags;		/*!< Interrupt flags register */
	uint8_t outcfg;			/*!< Output configuration for clock out/ wake/ int pins*/
	uint8_t seconds;		/*!< Current RTC seconds value in decimal format (0-59). */
	uint8_t minutes;		/*!< Current RTC minutes value in decimal format (0-59). */
	uint8_t hours;			/*!< Current RTC hours value in decimal format (0-23). */
	uint8_t date;			/*!< Current RTC date in decimal format (0-31). */
	month_t month;			/*!< Current RTC month enum (jan to dec). */
	uint16_t year;			/*!< Current RTC year in decimal (0 to 65535). */
	weekday_t weekday;		/*!< Current RTC day of the week enum (Sun to Sat). */
	uint32_t alarm_int;		/*!< Interval wakeup stored in number of RTC wake ints. 
									0 to 1-2^32-1 valid. */
	uint8_t seconds_alarm;	/*!< Alarm second setting in decimal format (0 to 59). */
	uint8_t minutes_alarm;	/*!< Alarm minute setting in decimal format (0 to 59). */
	uint8_t hours_alarm;	/*!< Alarm hour setting in decimal format (0 to 23). */
	uint8_t date_alarm;		/*!< Alarm date setting in decimal format (0-30). */
	month_t month_alarm;	/*!< Alarm month setting in enum format (0 to 59). */
	uint16_t year_alarm;	/*!< Alarm year setting in decimal format (0 to 65535). */
	weekday_t weekday_alarm;/*!< Alarm weekday setting in enum format (sun to sat). */
} rtc_t;

extern volatile uint8_t rtcFlag;

/**
 * @brief Set initial RTC configuration based on input structure
 * 
 * @param[in] rtcParams_s structure for initialization of internal RTC parameters
 * @return RTC_S_OK when parameters are updated successfully
 * @return RTC_S_INVALID_PARAM when one or more parameters are invalid in the struct
 * @return RTC_ERR when some other issue has gone wrong.
**/
RTC_Status_t rtc_init(rtc_t* rtcInitParams_s);

/**
 *  @brief Run RTC functions like checking for alarms, updating the GPIOs etc
 */
void rtc_runRTC();

/**
 * @brief Set RTC wake interval this reflects the minimum interval for an RTC update
 * 
 * This function sets the interval at which the RTC will generate an internal interrupt.
 * Upon an RTC ISR the code will update the internal RTC object as well as increment the counter for any 
 * active interval timers and check for any calendar alarms. There is a tradeoff here between update frequency
 * current consumption where a more frequent wake interval will ensure that there is a more precise time available
 * but at the cost of additional wakeups.
 * If the input parameter is incorrect the wake interval will remain unchanged. 
 * 
 * @param[in] rtcWakeInt rtc wake interval expressed in power of 2 Hz increments. Update interval = 2^(rtcWakeInt) Hz. Valid values are from (-1 to 12) for a range of 0.5 Hz to 4096 Hz.
 * @return RTC_S_OK when update interval is updated successfully
 * @return RTC_S_INVALID_PARAM when the update interval is incorrect
 *
**/
RTC_Status_t rtc_setWakeInt(int8_t rtcWakeInt);

/**
 * @brief Set RTC Interrupt enables as defined by the RTC interrupt bitfields.
 * This function will update the entire interrupt configuration of the RTC without regard to the existing contents
 * of the register. The user must set (or unset) the entire field every time this function is called.
 * @param[in] rtcIntCfg 16 bit value with the desired interrupt enable flags set.
 * @return RTC_S_OK invalid bitfields are ignored.
**/
RTC_Status_t rtc_setIntCfg(uint16_t rtcIntCfg);

/**
 * @brief Clear RTC interrupt flags
 * Clear specified interrupts to acknowledge an interrupt has been completed. No new interrupt actions will occur until the interrupt flag is cleared.
 * @param[in] rtcIntMask bitfield mask of interrupt flags to be cleared.
 * @return RTC_S_OK invalid bitfields are ignored.
**/
RTC_Status_t rtc_clearIntFlags(uint16_t rtcIntMask);

/**
 * @brief Set output pin configuration
 * Set the RTC interrupt pin configuration out of a list of options defined in @ref RTC_OutCfg.
 * @param[in] rtcOutCfg enum that describes the polarity and duration of the interrupt signal.
 * @return RTC_S_OK when a valid configuration is applied.
 * @return RTC_S_INVALID_PARAM when outside the specified values in the enum.
**/
RTC_Status_t rtc_setOutCfg(RTC_OutCfg_t rtcOutCfg);

/**
 * @brief Set the seconds field in the RTC object
 * Updates the software RTC object and physical RTC peripheral seconds value.
 * @param[in] rtcSecs value of updated seconds field. Must be between 0-59.
 * @return RTC_S_OK when value is successfully updated.
 * @return RTC_S_INVALID_PARAM when seconds value is outside defined range.
**/
RTC_Status_t rtc_setSeconds(uint8_t rtcSecs);

/**
 * @brief Set the minutes field in the RTC object
 * Updates the software RTC object and physical RTC peripheral minutes value.
 * @param[in] rtcMins value of updated minutes field. Must be between 0-59.
 * @return RTC_S_OK when value is successfully updated.
 * @return RTC_S_INVALID_PARAM when minutes value is outside defined range.
**/
RTC_Status_t rtc_setMinutes(uint8_t rtcMins);

/**
 * @brief Set the hours field in the RTC object
 * Updates the software RTC object and physical RTC peripheral hours value.
 * @param[in] rtcHours value of updated hours field. Must be between 0-23.
 * @return RTC_S_OK when value is successfully updated.
 * @return RTC_S_INVALID_PARAM when hours value is outside defined range.
**/
RTC_Status_t rtc_setHours(uint8_t rtcHours);

/**
 * @brief Set the date field in the RTC object
 * Updates the software RTC object and physical RTC peripheral date value.
 * @param[in] rtcDate value of updated date field. Must be between 0-30.
 * @return RTC_S_OK when value is successfully updated.
 * @return RTC_S_INVALID_PARAM when date value is outside defined range.
**/
RTC_Status_t rtc_setDate(uint8_t rtcDate);

/**
 * @brief Set the month field in the RTC object
 * Updates the software RTC object and physical RTC peripheral month value.
 * @param[in] rtcMonth value of updated month field. Must be between Jan to Dec based on the month_t enum.
 * @return RTC_S_OK when value is successfully updated.
 * @return RTC_S_INVALID_PARAM when month value is outside defined range.
**/
RTC_Status_t rtc_setMonth(month_t rtcMonth);

/**
 * @brief Set the year field in the RTC object
 * Updates the software RTC object and physical RTC peripheral year value.
 * @param[in] rtcYear value of updated year field. Hardware will support up to year 4095. Software can accept any value up to
 * 		the 2^16-1 but will only update the lower twelve bits in the hardware counter.
 * @return RTC_S_OK when value is successfully updated.
**/
RTC_Status_t rtc_setYear(uint16_t rtcYear);

/**
 * @brief Set the weekday field in the RTC object
 * Updates the software RTC object and physical RTC peripheral weekday value.
 * @param[in] rtcDay value of updated week field. Must be between Sun to Sat based on the weekday_t enum.
 * @return RTC_S_OK when value is successfully updated.
 * @return RTC_S_INVALID_PARAM when weekday value is outside defined range.
**/
RTC_Status_t rtc_setWeekday(weekday_t rtcDay);

/**
 * @brief Set interval alarm that will generate an interrupt every x number of RTC ticks
 * Set the alarm interval for a periodic wakeup. When the interrupt is enabled, the interrupt 
 * flag @ref RTC_INT_INT_ALARM will be set after the specified interval. If @ref RTC_INT_INT_ALARM_ONESHOT_MODE is asserted
 * then the interval alarm will automatically disable the interrupt once the interval has expired. Otherwise the interval
 * timer will generating interrupts at the specified interval until disabled.
 * If the output pin is configured /enabled the external interrupt pin will be asserted according to the selected policy.
 * @param[in] rtcIntAlarm number of RTC ticks.
 * @return RTC_S_OK 
**/
RTC_Status_t rtc_setIntAlarm(uint32_t rtcIntAlarm);

/**
 * @brief Set the seconds alarm field in the RTC object.
 * Updates the software RTC object seconds alarm value. When the seconds alarm interrupt is enabled (@ref RTC_INT_SEC_ALARM),
 * the RTC will set the @ref RTC_INT_SEC_ALARM flag every time the RTC seconds value matches the value in the seconds
 * alarm field in the RTC object. This will generate an interrupt every minute when configured.
 * When the output pin is configured, the interrupt will generate an output pulse on the external interrupt pin.
 * @param[in] rtcSecsAlr value of seconds field on which the interrupt will be generated. Must be between 0-59.
 * @return RTC_S_OK when value is successfully updated.
 * @return RTC_S_INVALID_PARAM when seconds value is outside defined range.
**/
RTC_Status_t rtc_setSecondsAlarm(uint8_t rtcSecsAlr);

/**
 * @brief Set the minutes alarm field in the RTC object.
 * Updates the software RTC object minutes alarm value. When the minutes alarm interrupt is enabled (@ref RTC_INT_MIN_ALARM),
 * the RTC will set the @ref RTC_INT_MIN_ALARM flag every time the RTC minutes value matches the value
 * in the minutes alarm field in the RTC object. This will generate an interrupt every hour when configured.
 * When the output pin is configured, the interrupt will generate an output pulse on the external interrupt pin according
 * to the selected policy.
 * @param[in] rtcMinsAlr value of minutes alarm field. Must be between 0-59.
 * @return RTC_S_OK when value is successfully updated.
 * @return RTC_S_INVALID_PARAM when minutes value is outside defined range.
**/
RTC_Status_t rtc_setMinutesAlarm(uint8_t rtcMinsAlr);

/**
 * @brief Set the hours alarm field in the RTC object.
 * Updates the software RTC object alarm value. When the hours alarm interrupt is enabled (@ref RTC_INT_HOUR_ALARM),
 * the rtc will set the @ref RTC_INT_HOUR_ALARM flag every time the RTC hours value matches the value in the hours
 * alarm field in the RTC object. This will generate an interrupt every 24 hours (1 day) when configured.
 * When the output pin is configured, the interrupt will generate an output pulse on the external interrupt pin according
 * to the selected policy.
 * @param[in] rtcHoursAlr value of hours alarm field. Must be between 0-23.
 * @return RTC_S_OK when value is successfully updated.
 * @return RTC_S_INVALID_PARAM when hours value is outside defined range.
**/
RTC_Status_t rtc_setHoursAlarm(uint8_t rtcHoursAlr);

/**
 * @brief Set the date alarm field in the RTC object.
 * Updates the software RTC object date alarm value. When the date alarm interrupt is enabled (@ref RTC_INT_DATE_ALARM),
 * the rtc will set the @ref RTC_INT_DATE_ALARM flag every time the RTC date value matches the value in the date alarm
 * field in the RTC object. This will generate an interrupt every month when configured.
 * When the output pin is configured, the interrupt will generate an output pulse on the external interrupt pin according
 * to the selected policy.
 * @param[in] rtcDateAlr value of date alarm field. Must be between 0-30.
 * @return RTC_S_OK when value is successfully updated.
 * @return RTC_S_INVALID_PARAM when date value is outside defined range.
**/
RTC_Status_t rtc_setDateAlarm(uint8_t rtcDateAlr);

/**
 * @brief Set the month alarm field in the RTC object.
 * Updates the software RTC object month alarm value. When the month alarm interrupt is enabled (@ref RTC_INT_MONTH_ALARM),
 * the rtc will set the @ref RTC_INT_MONTH_ALARM flag every time the RTC month value matches the value in the month alarm
 * field in the RTC object. This will generate an interrupt every year when configured.
 * When the output pin is configured, the interrupt will generate an output pulse on the external interrupt pin according
 * to the selected policy.
 * @param[in] rtcMonthAlr value of month alarm field. Must be between Jan to Dec based on the month_t enum.
 * @return RTC_S_OK when value is successfully updated.
 * @return RTC_S_INVALID_PARAM when month value is outside defined range.
**/
RTC_Status_t rtc_setMonthAlarm(month_t rtcMonthAlr);

/**
 * @brief Set the year alarm field in the RTC object.
 * Updates the software RTC object year alarm value. When the year alarm interrupt is enabled (@ref RTC_INT_YEAR_ALARM),
 * the rtc will set the @ref RTC_YEAR_ALARM flag every time the RTC year value matches the value in the year alarm field
 * in the RTC object. This will generate an interrupt once.
 * When the output pin is configured, the interrupt will generate an output pulse on the external interrupt pin according
 * to the selected policy.
 * @param[in] rtcYearAlr value of year alarm field. Hardware will support up to year 4095. Software can accept any value up to
 * 		the 2^16-1 but will only update the lower twelve bits in the hardware counter.
 * @return RTC_S_OK when value is successfully updated.
**/
RTC_Status_t rtc_setYearAlarm(uint16_t rtcYearAlr);

/**
 * @brief Set the weekday alarm field in the RTC object.
 * Updates the software RTC object weekday alarm value. When the weekday alarm interrupt is enabled (@ref RTC_INT_WEEKDAY_ALARM),
 * the rtc will set the @ref RTC_WEEKDAY_ALARM flag every time the RTC weekday value matches the value in the weekday alarm field
 * in the RTC object. This will generate an interrupt once a week.
 * When the output pin is configured, the interrupt will generate an output pulse on the external interrupt pin according
 * to the selected policy.
 * @param[in] rtcDayAlr value of weekday alarm field. Must be between Sun to Sat based on the weekday_t enum.
 * @return RTC_S_OK when value is successfully updated.
 * @return RTC_S_INVALID_PARAM when weekday value is outside defined range.
**/
RTC_Status_t rtc_setWeekdayAlarm(weekday_t rtcDayAlr);

/**
 * @brief Get RTC ID
 * @return RTC_ID
 */
uint8_t rtc_getRTCID( void );

/**
 * @brief Get RTC wakeup interval as power of 2 Hz
 * @return RTC wake interval
 */
int8_t rtc_getWakeInt( void );

/**
 * @brief Get interrupt configuration.
 * @return intCfg bitfield
 */
uint16_t rtc_getIntCfg( void );

/**
 * @brief Get currently active interrupt flags
 * For each valid interrupt flag bit position, returns a 1 if the interrupt is asserted.
 * @return interrupt status register
 */
uint16_t rtc_getIntFlags( void );

/**
 * @brief Get output pin configuration
 * @return Current output configuration
 */
RTC_OutCfg_t rtc_getOutCfg( void );

/**
 * @brief Get RTC Seconds
 * @return seconds value of the RTC object in decimal format
 */
uint8_t rtc_getSeconds( void );

/**
 * @brief Get RTC Minutes
 * @return minutes value of the RTC object in decimal format
 */
uint8_t rtc_getMinutes( void );

/**
 * @brief Get RTC Hours
 * @return hours value of the RTC object in decimal format
 */
uint8_t rtc_getHours( void );

/**
 * @brief Get RTC date
 * @return Day of month value of the RTC object in decimal format
 */
uint8_t rtc_getDate( void );

/**
 * @brief Get RTC Months
 * @return Month value of the RTC object in enum format
 */
month_t rtc_getMonth( void );


uint16_t rtc_getYear( void );

weekday_t rtc_getWeekday( void );

uint32_t rtc_getIntAlarm( void );

uint8_t rtc_getSecondsAlarm( void );

uint8_t rtc_getMinutesAlarm( void );

uint8_t rtc_getHoursAlarm( void );

uint8_t rtc_getDateAlarm( void );

month_t rtc_getMonthAlarm( void );

uint16_t rtc_getYearAlarm( void );

weekday_t rtc_getWeekdayAlarm( void );


#endif /* RTC_H_ */
/** @}*/
