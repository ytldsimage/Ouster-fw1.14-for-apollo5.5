# Ouster-fw1.14-for-apollo5.5
By WhaleDynamics 202003



请测试附件移植到apollo5.5的驱动，使用是你3月ROS beta4的driver代码.

1. 将ouster目录拷贝到apollo的“**modules/drivers**”目录下；
2. 修改”**ouster/conf/ouster128.pb.txt**“中的”**hostname**“为Lidar的IP。
3. "**apollo.sh build_opt_gpu**"编译成功后启动“**cyber_launch start modules/drivers/ouster/launch/ouster128.launch**”，然后通过cyber中查看相关的输出信息。

