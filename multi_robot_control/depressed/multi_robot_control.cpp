// 这个程序是当时和段师兄之间的通讯控制程序，后续可能没用
#include "ros/ros.h"
#include "multi_robot_control/multi_ur.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include "multi_robot_control/PolishMsg.h"
#include "multi_robot_control/MsgDef.h"
#include "multi_robot_control/busapi.h"

bus_api_t * g_api = NULL;

USING_BUS_NS

void send_ok_msg(int msg_type)
{
    unsigned int buf_len = sizeof(ST_RawData);
    char * buf = (char *)malloc(buf_len);
    memset(buf, 0, buf_len);

    ST_RawData * raw = (ST_RawData*)buf;
    raw->dev_id = POLISH_DEV_ID;
    raw->dev_type = POLISH_DEV_TYPE;
    raw->sensor_type = msg_type;
    raw->data_len = 0;

    // send to bus
    bus_api_send_msg(g_api, MSG_TYPE_RAW_DATA, buf, buf_len);
    free(buf);
}

void on_connected(void * ctx)
{
	printf("libbus connected.\n");
	if (bus_api_register_device(g_api, POLISH_DEV_ID) != 0)
	{
		printf("bus_api_register_device %d failed.\n", POLISH_DEV_ID);
	}
}

void on_disconnected(void * ctx)
{
	printf("libbus disconnected\n");
}

void on_polish_msg(ST_RawData * raw, const char * data, unsigned int len, void * ctx)
{
	printf("received ur data from %d, sensor type is %d, data len is %d\n",
		raw->dev_id, raw->sensor_type, raw->data_len);

    multi_ur::ur * ur = (multi_ur::ur *) ctx;

	switch (raw->sensor_type)
	{
	case E_POLISH_MSG_CAPTURE:
    {
        POLISH_MSG_CAPTURE * cap_data = (POLISH_MSG_CAPTURE*)data;
        ur->ur_capture.eef_pose[0] = cap_data->pos[0];
        ur->ur_capture.eef_pose[1] = cap_data->pos[1];
        ur->ur_capture.eef_pose[2] = cap_data->pos[2];
        ur->ur_capture.eef_pose[3] = cap_data->norm[0];
        ur->ur_capture.eef_pose[4] = cap_data->norm[1];
        ur->ur_capture.eef_pose[5] = cap_data->norm[2];

        if(ur->ur_capture.Move(ur->ur_capture.eef_pose, 0.5, 0.3))
        {
            send_ok_msg(E_POLISH_MSG_CAPTURE_MOVE_OK);
        }
        break;
    }
    case E_POLISH_MSG_EXECUTE:
    {
        POLISH_MSG_EXECUTE * cap_data = (POLISH_MSG_EXECUTE*)data;
        ur->ur_polish.eef_pose[0] = cap_data->pos[0];
        ur->ur_polish.eef_pose[1] = cap_data->pos[1];
        ur->ur_polish.eef_pose[2] = cap_data->pos[2];
        ur->ur_polish.eef_pose[3] = cap_data->norm[0];
        ur->ur_polish.eef_pose[4] = cap_data->norm[1];
        ur->ur_polish.eef_pose[5] = cap_data->norm[2];
        if(ur->ur_polish.Move(ur->ur_polish.eef_pose, 0.5, 0.3))
        {
            send_ok_msg(E_POLISH_MSG_EXECUTE_MOVE_OK);
        }
        break;
    }
    case E_POLISH_MSG_CAPTURE_RESET:
    {
        std::vector<double> polish_ur_reset_pose(6);
        polish_ur_reset_pose = {0,0,0,0,0,0};
        for(int i = 0; i < 6; i ++)
        {
            ur->ur_capture.eef_pose[i] = polish_ur_reset_pose[i];
        }
        if(ur->ur_capture.Move(ur->ur_capture.eef_pose, 0.5, 0.3))
        {
            send_ok_msg(E_POLISH_MSG_CAPTURE_RESET_OK);
        }
        break;
    }
    case E_POLISH_MSG_EXECUTE_RESET:
    {
        std::vector<double> polish_ur_reset_pose(6);
        polish_ur_reset_pose = {0,0,0,0,0,0};
        for(int i = 0; i < 6; i ++)
        {
            ur->ur_polish.eef_pose[i] = polish_ur_reset_pose[i];
        }
        if(ur->ur_polish.Move(ur->ur_polish.eef_pose, 0.5, 0.3))
        {
            send_ok_msg(E_POLISH_MSG_EXECUTE_RESET_OK);
        }
        break;
    }
	default:
		break;
	}
}

void on_msg(short msg_type, const char * msg, unsigned int len, void * ctx)
{
	if (msg_type == MSG_TYPE_RAW_DATA)
	{
		ST_RawData * raw_data = (ST_RawData*)msg;
		switch (raw_data->dev_type)
		{
		case POLISH_DEV_TYPE:
			on_polish_msg(raw_data, msg + sizeof(ST_RawData), raw_data->data_len, ctx);
			break;
		default:
			break;
		}
	}
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "multi_robot_control");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle nh;
    multi_ur::ur ur(nh);

    char bus_ip[32] = "192.168.1.100";  // ip address of server
	int bus_port = 5000;

	bus_api_cb_t cb = { on_msg, on_connected, on_disconnected };

	bus_api_t * api = bus_api_init(bus_ip, bus_port, &cb, &ur);
	if (!api)
	{
		printf("bus_api_init failed.");
		return -1;
	}
	g_api = api;
    // ���е�׼���������Ѿ���ɣ��ᶨ�ڽ���ص�������ͨ�����벻ͬ�����ݣ�ִ�в�ͬ�Ĺ���

    // std::vector<double> pose(6);
    // pose = {-0.781, -0.032, 0.752, -2.42, -0.571, 1.69};
    // bool status = ur1.Move(pose, 0.5, 0.3);

    // if(status == true)
    //     ROS_INFO("Robot has arrived destination!");

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        /* code for loop body */
    
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    bus_api_free(api);

    return 0;
}