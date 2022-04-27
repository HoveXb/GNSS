1.编译功能包
2.使用sudo cutecom命令查看GPS端口号，并修改功能包中的配置文件config_gnss_prep.yaml中的kComPort口。
3.连接GNSS wifi，登录网址 http://192.168.200.1/ 查看惯导状态，但状态为组合导航时，可进行GPS数据录制
4.运行节点 rosrun n_gnss_prep n_gnss_prep
5.录制数据 rosbag record /msg_gnss_prep_long
