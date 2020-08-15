#pragma once
#include "MsgDef.h"

#define POLISH_DEV_ID		0x600
#define POLISH_DEV_TYPE		0x600

#pragma pack(push, 4)
enum POLISH_MSG_TYPE {
	E_POLISH_MSG_CAPTURE,						// 执行拍照
	E_POLISH_MSG_CAPTURE_MOVE_OK,				// 拍照机器人就位
	E_POLISH_MSG_CATURE_OK,						// 拍照完成
	E_POLISH_MSG_EXECUTE,						// 执行打磨
	E_POLISH_MSG_EXECUTE_MOVE_OK,				// 打磨机器人就位
	E_POLISH_MSG_EXECUTE_OK,					// 打磨完成
	E_POLISH_MSG_CAPTURE_RESET,					// 拍照机器人归零
	E_POLISH_MSG_CAPTURE_RESET_OK,				// 拍照机器人归零完成
	E_POLISH_MSG_EXECUTE_RESET,					// 打磨机器人归零
	E_POLISH_MSG_EXECUTE_RESET_OK,				// 打磨机器人归零
	E_POLISH_MSG_ERROR							// 发生错误
};

typedef struct {
	double		pos[3];
	double		norm[3];
}POLISH_MSG_CAPTURE;

typedef struct {
	double		pos[3];
	double		norm[3];
}POLISH_MSG_EXECUTE;

typedef struct {
	int			error_id;
	char		error_msg[256];
}POLISH_MSG_ERROR;


#pragma pack(pop)