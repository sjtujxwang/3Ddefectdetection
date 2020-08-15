#pragma once
/*
	此文件定义总线数据通讯协议
*/
#include "buscfg.h"

#ifdef __cplusplus
BUS_NS_BEGIN
#endif
//////////////////////////////////////////////////////////////////////
// 总线ID
#define		BUS_SERVER_ID							0x0
// 定义设备有效ID范围
#define		DEVICE_ID_START							0x100
#define		DEVICE_ID_END							0x1000
// 定义APP订阅者有效ID范围
#define		SUBSCRIBER_ID_START						0x1001
#define		SUBSCRIBER_ID_END						0x2000

// Hololens APP ID range
#define		SUB_HOLOLENS_ID_START					0x1100
#define 	SUB_HOLOLENS_ID_END						0x1200

// CPS APP ID range
#define		SUB_CPS_ID_START						0x1500
#define		SUB_CPS_ID_END							0x1600

//////////////////////////////////////////////////////////////////////
// 1. message types. datatype: short, should be in range[SHRT_MIN, SHRT_MAX]
#define		MSG_TYPE_REGISTER_DEVICE				0x100
#define		MSG_TYPE_UNREGISTER_DEVICE				0x101

#define		MSG_TYPE_SUBSCRIBE_DEVICE				0x102
#define		MSG_TYPE_UNSUBSCRIBE_DEVICE				0x103
typedef struct
{
	int		subscriber_id;
	int		dev_id;
}ST_SubscribeDevice;

#define		MSG_TYPE_RAW_DATA						0x104
/// 设备数据结构体
typedef struct
{
	int		dev_id;
	int		dev_type;
	int		sensor_type;
	long long data_time;
	int		data_len;
}ST_RawData;
//////////////////////////////////////////////////////////////////////
// 2. device types. datatype: int
#define		DEV_TYPE_START				0x500

#define		DEV_TYPE_ROBOT				0x500
#define		DEV_TYPE_DIGITAL_MACHINE	0x501
#define		DEV_TYPE_VIRTUAL_ASSEMBLY	0x502
#define		DEV_TYPE_3D_PRINTER			0x503
#define		DEV_TYPE_OLD_MACHINE		0x504
#define		DEV_TYPE_FESTO_ROBOTINO		0x505
#define		DEV_TYPE_FESTO_ORDER		0x506
#define		DEV_TYPE_FESTO_CUR_ORDER	0x507

#define		DEV_TYPE_END				0x1000

//////////////////////////////////////////////////////////////////////
// 3. sensor types. datatype: int
#define		SENSOR_TYPE_POS_2D				0x1000
typedef struct
{
	double	x;
	double	y;
}ST_Vec2d;
#define		SENSOR_TYPE_POS_3D				0x1001
typedef struct
{
	double	x;
	double	y;
	double	z;
}ST_Vec3d;

#define		SENSOR_TYPE_SCALE_INT			0x1002
#define		SENSOR_TYPE_SCALE_DOUBLE			0x1003
#define		SENSOR_TYPE_SCALE_FLOAT			0x1004
#define		SENSOR_TYPE_CHAR				0x1005


#ifdef __cplusplus
BUS_NS_END
#endif