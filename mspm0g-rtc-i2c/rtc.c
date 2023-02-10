/*
 * rtc.c
 *
 *  Created on: Oct 27, 2022
 *      Author: munan
 */

#include "ti_msp_dl_config.h"
#include "rtc.h"

volatile uint8_t rtcFlag;

// Temporary global structure for testing I2C interface
volatile static rtc_t RTC_Registers;
volatile static uint32_t nTicks, perAlarmTicks;
volatile static uint8_t outTicks;

static uint32_t ticksPerSec;

static const uint8_t RTC_BOUNDS_DATE_MONTH[] = {	RTC_BOUNDS_DATE_JAN, RTC_BOUNDS_DATE_FEB,
													RTC_BOUNDS_DATE_MAR, RTC_BOUNDS_DATE_APR,
													RTC_BOUNDS_DATE_MAY, RTC_BOUNDS_DATE_JUN,
													RTC_BOUNDS_DATE_JUL, RTC_BOUNDS_DATE_AUG,
													RTC_BOUNDS_DATE_SEP, RTC_BOUNDS_DATE_OCT,
													RTC_BOUNDS_DATE_NOV, RTC_BOUNDS_DATE_DEC
												  };

static void rtc_periph_init( void );
static void rtc_periph_updateRTCObj();
/**
 * These are temporary implementations of these functions for testing I2C interface.
 * DONE: Re-write and document these in a more proper way.
 *
 *    int8_t RTC_ID;           sub-address 0
 *    int8_t rtc_wake_int;     sub-address 1
 *    uint16_t intcfg;         sub-address 2, 3
 *    uint16_t intflags;       sub-address 4, 5
 *    uint8_t outcfg;          sub-address 6
 *    uint8_t seconds;         sub-address 7
 *    uint8_t minutes;         sub-address 8
 *    uint8_t hours;           sub-address 9
 *    uint8_t date;            sub-address 10
 *    month_t month;           sub-address 11
 *    uint16_t year;           sub-address 12, 13
 *    weekday_t weekday;       sub-address 14
 *    uint32_t alarm_int;      sub-address 15, 16, 17, 18
 *    uint8_t seconds_alarm;   sub-address 19
 *    uint8_t minutes_alarm;   sub-address 20
 *    uint8_t hours_alarm;     sub-address 21
 *    uint8_t date_alarm;      sub-address 22
 *    month_t month_alarm;     sub-address 23
 *    uint16_t year_alarm;     sub-address 24, 25
 *    weekday_t weekday_alarm; sub-address 26
 */

RTC_Status_t rtc_init(rtc_t* rtcInitParams_s) {
	RTC_Status_t status;
	
	rtcFlag = 0;

	RTC_Registers.rtc_id = RTC_ID;
	
	status = rtc_setWakeInt(rtcInitParams_s->rtc_wake_int);
	if(RTC_S_INVALID_PARAM == status) {
	    return status;
	}
	
	status = rtc_setIntCfg(rtcInitParams_s->intcfg);
	
	RTC_Registers.intflags = 0;
	
	status = rtc_setOutCfg(rtcInitParams_s->outcfg);
	if(RTC_S_INVALID_PARAM == status) {
        return status;
    }
	
	status = rtc_setSeconds(rtcInitParams_s->seconds);
	if(RTC_S_INVALID_PARAM == status) {
        return status;
    }

	status = rtc_setMinutes(rtcInitParams_s->minutes);
	if(RTC_S_INVALID_PARAM == status) {
        return status;
    }

	status = rtc_setHours(rtcInitParams_s->hours);
	if(RTC_S_INVALID_PARAM == status) {
        return status;
    }
	
	// Set date parameters in YYYY MM DD order
	status = rtc_setYear(rtcInitParams_s->year);
	
	status = rtc_setMonth(rtcInitParams_s->month);
	if(RTC_S_INVALID_PARAM == status) {
        return status;
    }

	status = rtc_setDate(rtcInitParams_s->date);
	if(RTC_S_INVALID_PARAM == status) {
        return status;
    }
	
	status = rtc_setWeekday(rtcInitParams_s->weekday);
	if(RTC_S_INVALID_PARAM == status) {
        return status;
    }
	
	status = rtc_setIntAlarm(rtcInitParams_s->alarm_int);
	
	status = rtc_setSecondsAlarm(rtcInitParams_s->seconds_alarm);
	if(RTC_S_INVALID_PARAM == status) {
        return status;
    }

	status = rtc_setMinutesAlarm(rtcInitParams_s->minutes_alarm);
	if(RTC_S_INVALID_PARAM == status) {
        return status;
    }

	status = rtc_setHoursAlarm(rtcInitParams_s->hours_alarm);
	if(RTC_S_INVALID_PARAM == status) {
        return status;
    }

	status = rtc_setMonthAlarm(rtcInitParams_s->month_alarm);
	if(RTC_S_INVALID_PARAM == status) {
        return status;
    }

	status = rtc_setDateAlarm(rtcInitParams_s->date_alarm);
	if(RTC_S_INVALID_PARAM == status) {
        return status;
    }

	status = rtc_setYearAlarm(rtcInitParams_s->year_alarm);
	
	status = rtc_setWeekdayAlarm(rtcInitParams_s->weekday_alarm);
	if(RTC_S_INVALID_PARAM == status) {
        return status;
    }
	
	rtc_periph_init();
	
	return RTC_S_OK;
}

/**** setters ****/
/*     int8_t rtc_wake_int;     sub-address 1 */
inline RTC_Status_t rtc_setWakeInt(int8_t rtcWakeInt)
{
    DL_RTC_PRESCALER1_DIVIDE ps1 = 0x00;
    DL_RTC_PRESCALER0_DIVIDE ps0 = 0x00;

    if ( rtcWakeInt <= RTC_BOUNDS_WAKEINT_MAX && rtcWakeInt > RTC_BOUNDS_WAKEINT_MIN)
    	RTC_Registers.rtc_wake_int = rtcWakeInt;
	else
		return RTC_S_INVALID_PARAM;
	switch(RTC_Registers.rtc_wake_int) {
        case 0:
            ps1 = DL_RTC_PRESCALER1_DIVIDE_128;
            break;
        case 1:
            ps1 = DL_RTC_PRESCALER1_DIVIDE_64;
            break;
        case 2:
            ps1 = DL_RTC_PRESCALER1_DIVIDE_32;
            break;
        case 3:
            ps1 = DL_RTC_PRESCALER1_DIVIDE_16;
            break;
        case 4:
            ps1 = DL_RTC_PRESCALER1_DIVIDE_8;
            break;
        case 5:
            ps1 = DL_RTC_PRESCALER1_DIVIDE_4;
            break;
        case 6:
            ps1 = DL_RTC_PRESCALER1_DIVIDE_2;
            break;
        case 7:
            ps0 = DL_RTC_PRESCALER0_DIVIDE_256;
            break;
        case 8:
            ps0 = DL_RTC_PRESCALER0_DIVIDE_128;
            break;
        case 9:
            ps0 = DL_RTC_PRESCALER0_DIVIDE_64;
            break;
        case 10:
            ps0 = DL_RTC_PRESCALER0_DIVIDE_32;
            break;
        case 11:
            ps0 = DL_RTC_PRESCALER0_DIVIDE_16;
            break;
        case 12:
            ps0 = DL_RTC_PRESCALER0_DIVIDE_8;
            break;
        default:
            break;
    }

	// disable interrupts to prevent excessive event trips
	DL_RTC_disableInterrupt(RTC, DL_RTC_INTERRUPT_PRESCALER0);
	DL_RTC_disableInterrupt(RTC, DL_RTC_INTERRUPT_PRESCALER1);

	DL_RTC_clearInterruptStatus(RTC, DL_RTC_INTERRUPT_PRESCALER0| DL_RTC_INTERRUPT_PRESCALER1);

	DL_RTC_setPrescalerEvents(RTC, ps0, ps1);

    if (ps0) {
        DL_RTC_enableInterrupt(RTC, DL_RTC_INTERRUPT_PRESCALER0);
    }
    if (ps1) {
        DL_RTC_enableInterrupt(RTC, DL_RTC_INTERRUPT_PRESCALER1);
    }

    ticksPerSec = 0x00000001 << RTC_Registers.rtc_wake_int;

    return (RTC_S_OK);
}

RTC_Status_t rtc_setIntCfg(uint16_t rtcIntCfg) {
	RTC_Registers.intcfg = RTC_INT_ALL & rtcIntCfg;
	return RTC_S_OK;
}

RTC_Status_t rtc_clearIntFlags(uint16_t rtcIntMask) {
	RTC_Registers.intflags = RTC_INT_ALL & ~(RTC_INT_ALL & rtcIntMask);
	return RTC_S_OK;
}

/*    uint8_t outcfg;          sub-address 6 */
inline RTC_Status_t rtc_setOutCfg(RTC_OutCfg_t rtcOutCfg)
{
	if (rtcOutCfg< RTC_BOUNDS_OUTCFG_MAX)
    	RTC_Registers.outcfg = rtcOutCfg;
	else
		return RTC_S_INVALID_PARAM;
    return (RTC_S_OK);
}

/*    uint8_t seconds;         sub-address 7 */
inline RTC_Status_t rtc_setSeconds(uint8_t rtcSecs)
{
	if (rtcSecs < RTC_BOUNDS_SECONDS)
		RTC_Registers.seconds = rtcSecs;
    else
		return RTC_S_INVALID_PARAM;

	DL_RTC_setCalendarSecondsBinary(RTC, RTC_Registers.seconds);

	return (RTC_S_OK);
}

/*    uint8_t minutes;         sub-address 8 */
inline RTC_Status_t rtc_setMinutes(uint8_t rtcMins)
{
	if (rtcMins < RTC_BOUNDS_MINUTES)
    	RTC_Registers.minutes = rtcMins;
	else 
		return RTC_S_INVALID_PARAM;

	DL_RTC_setCalendarMinutesBinary(RTC, RTC_Registers.minutes);

    return (RTC_S_OK);
}

/*    uint8_t hours;           sub-address 9 */
inline RTC_Status_t rtc_setHours(uint8_t rtcHours)
{
	if (rtcHours < RTC_BOUNDS_HOURS)
    	RTC_Registers.hours = rtcHours;
    else
		return RTC_S_INVALID_PARAM;

	DL_RTC_setCalendarHoursBinary(RTC, RTC_Registers.hours);

	return (RTC_S_OK);
}

RTC_Status_t rtc_setDate(uint8_t rtcDate) {
	if (rtcDate < RTC_BOUNDS_DATE_MONTH[RTC_Registers.month]) {
	    RTC_Registers.date = rtcDate;
	} else if (RTC_Registers.month == feb && RTC_Registers.year % 4 == 0) {
	    if (rtcDate< RTC_BOUNDS_DATE_FEB_LEAP) {
	        RTC_Registers.date = rtcDate;
	    }
	} else {
	    return RTC_S_INVALID_PARAM;
	}

	DL_RTC_setCalendarDayOfMonthBinary(RTC, RTC_Registers.date);

	return RTC_S_OK;
}

RTC_Status_t rtc_setMonth(month_t rtcMonth) {
	if (rtcMonth < RTC_BOUNDS_MONTH)
		RTC_Registers.month = rtcMonth;
	else
		return RTC_S_INVALID_PARAM;

	DL_RTC_setCalendarMonthBinary(RTC, RTC_Registers.month + 1);

	return RTC_S_OK;
}

RTC_Status_t rtc_setYear(uint16_t rtcYear) {
	RTC_Registers.year = rtcYear;

	DL_RTC_setCalendarYearBinary(RTC, RTC_Registers.year);
	return RTC_S_OK;
}

RTC_Status_t rtc_setWeekday(weekday_t rtcDay) {
	if (rtcDay < RTC_BOUNDS_WEEKDAY)
		RTC_Registers.weekday = rtcDay;
	else
		return RTC_S_INVALID_PARAM;

	DL_RTC_setCalendarDayOfWeekBinary(RTC, RTC_Registers.weekday);

	return RTC_S_OK;
}

RTC_Status_t rtc_setIntAlarm(uint32_t rtcIntAlarm) {
	RTC_Registers.alarm_int = rtcIntAlarm;
	return RTC_S_OK;
}

RTC_Status_t rtc_setSecondsAlarm(uint8_t rtcSecsAlr) {
	if (rtcSecsAlr < RTC_BOUNDS_SECONDS)
		RTC_Registers.seconds_alarm = rtcSecsAlr;
	else
		return RTC_S_INVALID_PARAM;
	return RTC_S_OK;
}

RTC_Status_t rtc_setMinutesAlarm(uint8_t rtcMinsAlr) {
	if (rtcMinsAlr < RTC_BOUNDS_MINUTES)
		RTC_Registers.minutes_alarm = rtcMinsAlr;
	else
		return RTC_S_INVALID_PARAM;
	return RTC_S_OK;
}

RTC_Status_t rtc_setHoursAlarm(uint8_t rtcHoursAlr) {
	if (rtcHoursAlr < RTC_BOUNDS_HOURS)
		RTC_Registers.hours_alarm;
	else
		return RTC_S_INVALID_PARAM;
	return RTC_S_OK;
}

RTC_Status_t rtc_setDateAlarm(uint8_t rtcDateAlr) {
	if (rtcDateAlr < RTC_BOUNDS_DATE_MONTH[RTC_Registers.month_alarm])
		RTC_Registers.date_alarm = rtcDateAlr;
	else
		return RTC_S_INVALID_PARAM;
	return RTC_S_OK;
}

RTC_Status_t rtc_setMonthAlarm(month_t rtcMonthAlr) {
	if (rtcMonthAlr < RTC_BOUNDS_MONTH)
		RTC_Registers.month_alarm = rtcMonthAlr;
	else
		return RTC_S_INVALID_PARAM;
	return RTC_S_OK;
}

RTC_Status_t rtc_setYearAlarm(uint16_t rtcYearAlr) {
	RTC_Registers.year_alarm = rtcYearAlr;
	return RTC_S_OK;
}

RTC_Status_t rtc_setWeekdayAlarm(weekday_t rtcDayAlr) {
	if (rtcDayAlr < RTC_BOUNDS_WEEKDAY)
		RTC_Registers.weekday_alarm = rtcDayAlr;
	else
		return RTC_S_INVALID_PARAM;
	return RTC_S_OK;
}

/**
 *** getters ****
 **/
inline uint8_t rtc_getRTCID( void ) {
	return RTC_Registers.rtc_id;
}

inline int8_t rtc_getWakeInt() {
	return RTC_Registers.rtc_wake_int;
}

inline uint16_t rtc_getIntCfg( void ) {
	return RTC_Registers.intcfg;
}

inline uint16_t rtc_getIntFlags( void ) {
	return RTC_Registers.intflags;
}

inline RTC_OutCfg_t rtc_getOutCfg( void ) {
	return RTC_Registers.outcfg;
}

inline uint8_t rtc_getSeconds( void ) {
	return RTC_Registers.seconds;
}

inline uint8_t rtc_getMinutes( void ) {
	return RTC_Registers.minutes;
}

inline uint8_t rtc_getHours( void ) {
	return RTC_Registers.hours;
}

inline uint8_t rtc_getDate( void ) {
	return RTC_Registers.date;
}

inline month_t rtc_getMonth( void ) {
	return RTC_Registers.month;
}

inline uint16_t rtc_getYear( void ) {
	return RTC_Registers.year;
}

inline weekday_t rtc_getWeekday( void ) {
	return RTC_Registers.weekday;
}

inline uint32_t rtc_getIntAlarm( void ) {
	return RTC_Registers.alarm_int;
}

inline uint8_t rtc_getSecondsAlarm( void ) {
	return RTC_Registers.seconds_alarm;
}

inline uint8_t rtc_getMinutesAlarm( void ) {
	return RTC_Registers.minutes_alarm;
}

inline uint8_t rtc_getHoursAlarm( void ) {
	return RTC_Registers.hours_alarm;
}

inline uint8_t rtc_getDateAlarm( void ) {
	return RTC_Registers.date_alarm;
}

inline month_t rtc_getMonthAlarm( void ) {
	return RTC_Registers.month_alarm;
}

inline uint16_t rtc_getYearAlarm( void ) {
	return RTC_Registers.year_alarm;
}

inline weekday_t rtc_getWeekdayAlarm( void ) {
	return RTC_Registers.weekday_alarm;
}

static DL_RTC_Calendar rtcData;

static void rtc_periph_init( void ) {

    rtcData.seconds = RTC_Registers.seconds;
    rtcData.minutes = RTC_Registers.minutes;
    rtcData.hours = RTC_Registers.hours;
    rtcData.dayOfWeek = RTC_Registers.weekday;
    rtcData.dayOfMonth = RTC_Registers.date;
    rtcData.month = RTC_Registers.month + 1;
    rtcData.year = RTC_Registers.year;

    DL_RTC_initCalendar(RTC, rtcData, DL_RTC_FORMAT_BINARY);

    DL_RTC_enableClockControl(RTC);

    NVIC_EnableIRQ(RTC_INT_IRQn);

}

static void rtc_periph_updateRTCObj(void) {
    while(!DL_RTC_isSafetoRead(RTC));
    rtcData = DL_RTC_getCalendarTime(RTC);

    RTC_Registers.seconds = rtcData.seconds;
    RTC_Registers.minutes = rtcData.minutes;
    RTC_Registers.hours = rtcData.hours;
    RTC_Registers.weekday = rtcData.dayOfWeek;
    RTC_Registers.date = rtcData.dayOfMonth;
    RTC_Registers.month = rtcData.month - 1;
    RTC_Registers.year = rtcData.year;
}

void rtc_runRTC() {

    rtc_t RTC_temp;
    rtcFlag = 0;
    nTicks++;
    perAlarmTicks++;
    RTC_temp = RTC_Registers;
    if(nTicks == ticksPerSec) {
        nTicks = 0;
        DL_GPIO_togglePins(GPIO_GRP_0_PORT,GPIO_GRP_0_PIN_0_PIN);

        rtc_periph_updateRTCObj();

        if (RTC_Registers.seconds != RTC_temp.seconds && RTC_Registers.seconds == RTC_Registers.seconds_alarm) {
            RTC_Registers.intflags |= RTC_INT_SEC_ALARM;
        }

        if (RTC_Registers.minutes != RTC_temp.minutes && RTC_Registers.minutes == RTC_Registers.minutes_alarm) {
            RTC_Registers.intflags |= RTC_INT_MIN_ALARM;
        }

        if (RTC_Registers.hours != RTC_temp.hours && RTC_Registers.hours == RTC_Registers.hours_alarm) {
            RTC_Registers.intflags |= RTC_INT_HOUR_ALARM;
        }

        if (RTC_Registers.date != RTC_temp.date && RTC_Registers.date == RTC_Registers.date_alarm) {
            RTC_Registers.intflags |= RTC_INT_DATE_ALARM;
        }

        if (RTC_Registers.weekday != RTC_temp.weekday && RTC_Registers.weekday == RTC_Registers.weekday_alarm) {
            RTC_Registers.intflags |= RTC_INT_WEEKDAY_ALARM;
        }

        if (RTC_Registers.month != RTC_temp.month && RTC_Registers.month == RTC_Registers.month_alarm) {
            RTC_Registers.intflags |= RTC_INT_MONTH_ALARM;
        }

        if (RTC_Registers.year != RTC_temp.year && RTC_Registers.year == RTC_Registers.year_alarm) {
            RTC_Registers.intflags |= RTC_INT_YEAR_ALARM;
        }
    }

}

void RTC_IRQHandler() {
    DL_RTC_IIDX pendingIIDX = DL_RTC_getPendingInterrupt(RTC);

    if((pendingIIDX & DL_RTC_IIDX_PRESCALER0 ) || (pendingIIDX & DL_RTC_IIDX_PRESCALER1)) {
        rtcFlag = 1;
    }
}
