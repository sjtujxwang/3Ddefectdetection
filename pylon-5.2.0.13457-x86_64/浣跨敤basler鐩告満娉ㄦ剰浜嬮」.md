# 相机驱动安装
1. 参考INSTALL文件即可，或者参考博客https://blog.csdn.net/qq_29229045/article/details/78482069，基本问题不大；
2. 其中source ./pylon5/bin/pylon-setup-env.sh pylon5 可以换成 source /opt/pylon5/bin/pylon-setup-env.sh /opt/pylon5并写在.bashrc中

# 相机IP设置
2. 刚开始连接时可以按照这个博客https://blog.csdn.net/sazass/article/details/81281604，在本项目中建议设置成固定IP，所有设备在一个IP范围内

# 其他设置：为了优化basler相机的数据传输，建议在使用相机时注意以下事项
# PC端修改内容
3. 将Maximum Transfer Unit (MTU) 设置成 8192
4. 增大UDP包的数据大小，可以通过sudo sysctl net.core.rmem_max=2097152设置，或者将 net.core.rmem_max=2097152 写到/etc/sysctl.conf文件中去
5. 注意网卡适配器（官方有建议的网卡型号比如Intel PRO 1000,I210, I340 and I350系列）
6. Permissions for Real-Time Thread Priorities（这块没仔细研究，设置前两个可能就可以正常使用了）
# 相机端修改内容
7. 通过pylon Viewer应用，选择相机，在Device->Features->Transport Layer下面设置Packet Size为8192

# 相机ros驱动分及配置
8. ros驱动安装，参考官网的wiki教程 http://wiki.ros.org/pylon_camera
9. 相机标定，参考wiki教程 http://wiki.ros.org/camera_calibration