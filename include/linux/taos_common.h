/*******************************************************************************
*                                                                              *
*       File Name:      taos_common.h                                          *
*       Description:    Common file for ioctl and configuration definitions.   *
*       		Used by kernel driver and driver access applications.  *
*       		Please include this file, and <sys/ioctl.h> in your    * 
*                       driver access application program source.	       *
*       Author:         John Koshi                                             *
*       History:        09/16/2009 - Initial creation                          *
*       		02/07/2010 - Add proximity			       *
*                                                                              *
********************************************************************************
*       Proprietary to Taos Inc., 1001 Klein Road #300, Plano, TX 75074        *
*******************************************************************************/
#ifndef __TAOS_COMMON_H
#define __TAOS_COMMON_H

// ioctl numbers
#define TAOS_IOCTL_MAGIC        	0XCF
#define TAOS_IOCTL_ALS_ON       	_IO(TAOS_IOCTL_MAGIC, 1)
#define TAOS_IOCTL_ALS_OFF      	_IO(TAOS_IOCTL_MAGIC, 2)
#define TAOS_IOCTL_ALS_DATA     	_IOR(TAOS_IOCTL_MAGIC, 3, short)
#define TAOS_IOCTL_ALS_CALIBRATE	_IO(TAOS_IOCTL_MAGIC, 4)
#define TAOS_IOCTL_CONFIG_GET   	_IOR(TAOS_IOCTL_MAGIC, 5, struct taos_cfg)
#define TAOS_IOCTL_CONFIG_SET		_IOW(TAOS_IOCTL_MAGIC, 6, struct taos_cfg)
#define TAOS_IOCTL_PROX_ON		_IO(TAOS_IOCTL_MAGIC, 7)
#define TAOS_IOCTL_PROX_OFF		_IO(TAOS_IOCTL_MAGIC, 8)
#define TAOS_IOCTL_PROX_DATA		_IOR(TAOS_IOCTL_MAGIC, 9, struct taos_prox_info)
#define TAOS_IOCTL_PROX_EVENT           _IO(TAOS_IOCTL_MAGIC, 10)
#define TAOS_IOCTL_PROX_CALIBRATE	_IO(TAOS_IOCTL_MAGIC, 11)

#define TAOS_IOCTL_SENSOR_ON	_IO(TAOS_IOCTL_MAGIC, 12)
#define TAOS_IOCTL_SENSOR_OFF	_IO(TAOS_IOCTL_MAGIC, 13)
#define TAOS_IOCTL_SENSOR_CONFIG	_IOW(TAOS_IOCTL_MAGIC, 14, struct taos_cfg)
#define TAOS_IOCTL_SENSOR_CHECK	_IO(TAOS_IOCTL_MAGIC, 15)
#define TAOS_IOCTL_SENSOR_test	_IO(TAOS_IOCTL_MAGIC, 16)
#define TAOS_IOCTL_READ_ID _IO(TAOS_IOCTL_MAGIC,17)
#define TAOS_IOCTL_SENSOR_READ_DATA  _IO(TAOS_IOCTL_MAGIC, 18)

#define ALS_PS_INT MSM_GPIO_TO_INT(ALS_PS_GPIO)
#define TAOS_INPUT_NAME "taos"

// device configuration
struct taos_cfg {
    u16 als_time;
    u16 scale_factor;
	u8  als_gain;
	u16	prox_threshold_hi;
	u16 prox_threshold_lo;
	u16 als_threshold_hi;
	u16 als_threshold_lo;
	u8	prox_als_time;
	u8	prox_time;
	u8	wait_time;
	u8	pers;
	u8	prox_config;
	u8	prox_pulse_cnt;
	u8	prox_gain;
	u8  prox_offset;
};

// proximity data
struct taos_prox_info {
        u16 prox_clear;
        u16 prox_data;
        int prox_event;
};

struct taos_platform_data {
	int	(*init_hw_power)(void);

};

#endif 
