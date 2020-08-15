//将实时的机器人的关节数据发送给

#include <stdio.h>  
#include <sys/types.h>  
#include <sys/socket.h>  
#include <netinet/in.h>  
#include <arpa/inet.h>  
#include <ros/ros.h>
#include "tooling_controller/tool_command.h"

#define PI 3.14159265354

class TOOLING_CONTROLLER
{
public:
    TOOLING_CONTROLLER()
    {
        tool_command_vector.resize(5); // set_f_target, set_f_zero, set_t_ramp, set_payload, Reserve.
        tool_command_sub = nh.subscribe("tool_command", 1000, &TOOLING_CONTROLLER::ToolCommandsubCallback,this);
        ros::Duration(1.0).sleep();
        start();
    }
    void start();

private:
    ros::NodeHandle nh;
    ros::Subscriber tool_command_sub;
    tooling_controller::tool_command tool_command_msg;
    std::vector<double> tool_command_vector;
    
    

    // tcpip通信相关的变量
    int client_sockfd;  
    int len;  
    struct sockaddr_in remote_addr; //服务器端网络地址结构体  
    char buf[BUFSIZ];  //数据传送的缓冲区 
    void ToolCommandsubCallback(const tooling_controller::tool_commandConstPtr& msg); 

    std::string combine(std::vector<double> tooling_input); 
    std::string double2string(double input);
    bool SetTcpIp();
};





int main(int argc, char *argv[])  
{  
    ros::init(argc, argv, "tooling");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    TOOLING_CONTROLLER tooling_controller;
    tooling_controller.start();
    ros::waitForShutdown();

    return 0;  
}  

void TOOLING_CONTROLLER::start()
{
    SetTcpIp();


    std::string msg;
    std::vector<double> tooling_input;
    tooling_input.resize(6);


    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        msg = combine(tooling_input);
        ROS_INFO_STREAM("SENDING MSG IS "<<msg);
        strcpy(buf,msg.c_str());
        len=send(client_sockfd,buf,strlen(buf),0);

        len = recv(client_sockfd,buf,BUFSIZ,0);
        buf[len] = '\0';
        printf("%s",buf);
        ros::spinOnce();
        loop_rate.sleep();
    }
    close(client_sockfd);//关闭套接字  
}

void TOOLING_CONTROLLER::ToolCommandsubCallback(const tooling_controller::tool_commandConstPtr& msg)
{
    tool_command_msg = * msg;
    tool_command_msg.data.resize(5);
    for(int i = 0; i < 5 ; i ++)
    {
        tool_command_vector[i] = tool_command_msg.data[i];
    }
}

bool TOOLING_CONTROLLER::SetTcpIp()
{
    memset(&remote_addr,0,sizeof(remote_addr)); //数据初始化--清零  
    remote_addr.sin_family=AF_INET; //设置为IP通信  
    remote_addr.sin_addr.s_addr=inet_addr("192.168.99.1");//服务器IP地址  
    remote_addr.sin_port=htons(7070); //服务器端口号  
 
    if((client_sockfd=socket(AF_INET,SOCK_STREAM,0))<0)  
    {  
        perror("connect failure!");  
        return 1;  
    }  
    if(connect(client_sockfd,(struct sockaddr *)&remote_addr,sizeof(struct sockaddr))<0)  
    {  
        perror("connect success!");  
        return 1;  
    }  
}
std::string TOOLING_CONTROLLER::combine(std::vector<double> tooling_input)
{
    for(int i = 0 ; i < 6 ; i ++)
    {
        tooling_input[i] = tooling_input[i]*180/PI;
    }
    std::string msg;
    msg = double2string(tooling_input[0]) + ",";
    msg = msg + double2string(tooling_input[1]) + ",";
    msg = msg + double2string(tooling_input[2]) + ",";
    msg = msg + double2string(tooling_input[3]) + ",";
    msg = msg + double2string(tooling_input[4]) + ",";
    msg = msg + double2string(tooling_input[5]);
    return msg;
}
std::string TOOLING_CONTROLLER::double2string(double input)
{
    std::string string_temp;
    std::stringstream stream;
    stream<<input;
    string_temp = stream.str();
    return string_temp;
}