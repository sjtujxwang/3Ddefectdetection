#pragma once
#include "MsgDef.h"

#define POLISH_DEV_ID		0x600
#define POLISH_DEV_TYPE		0x600

#pragma pack(push, 4)
enum POLISH_MSG_TYPE {
	E_POLISH_MSG_CAPTURE,						// ִ������
	E_POLISH_MSG_CAPTURE_MOVE_OK,				// ���ջ����˾�λ
	E_POLISH_MSG_CATURE_OK,						// �������
	E_POLISH_MSG_EXECUTE,						// ִ�д�ĥ
	E_POLISH_MSG_EXECUTE_MOVE_OK,				// ��ĥ�����˾�λ
	E_POLISH_MSG_EXECUTE_OK,					// ��ĥ���
	E_POLISH_MSG_CAPTURE_RESET,					// ���ջ����˹���
	E_POLISH_MSG_CAPTURE_RESET_OK,				// ���ջ����˹������
	E_POLISH_MSG_EXECUTE_RESET,					// ��ĥ�����˹���
	E_POLISH_MSG_EXECUTE_RESET_OK,				// ��ĥ�����˹���
	E_POLISH_MSG_ERROR							// ��������
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