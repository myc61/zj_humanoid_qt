# mpc\-tele部署文档

1. 进入pico查看是否安装docker

```Plain Text
docker ps -a
```

2. 安装docker

```Plain Text
*##1. 更新索引*
sudo apt update
*##2. 直接安装*
sudo apt install -y docker.io
*##3. 把当前用户加入 docker 组，避免每次sudo*
sudo usermod -aG docker $USER
*##4. 激活组权限（或者重新登录一次 shell）*
newgrp docker
*##5. 验证*
docker version 
```

- 若docker因为网络问题安装失败

- 在上位机中间件部署页面，手动选择“离线Docker包”（docker-28.2.1.tgz），一键MPC全部部署会在在线安装失败后自动使用该离线包继续安装。

```Plain Text
scp docker-28.2.1.tgz nav01@192.168.217.66:/home/nav01
tar xzvf docker-28.2.1.tgz 

cd docker/
sudo cp * /usr/bin/

sudo nano /etc/systemd/system/docker.service
```

```Plain Text
[Unit]
Description=Docker Application Container Engine
Documentation=https://docs.docker.com
After=network-online.target firewalld.service
Wants=network-online.target

[Service]
Type=notify
# 这里的路径确保执行的是你刚才移动的文件
ExecStart=/usr/bin/dockerd
ExecReload=/bin/kill -s HUP $MAINPID
LimitNOFILE=infinity
LimitNPROC=infinity
LimitCORE=infinity
TimeoutStartSec=0
Delegate=yes
KillMode=process
Restart=on-failure
StartLimitBurst=3
StartLimitInterval=60s

[Install]
WantedBy=multi-user.target
```

**保存退出：** `Ctrl+O` \-\> `Enter` \-\> `Ctrl+X`。

```Plain Text
*# 1. 创建 docker 组*
sudo groupadd docker
*# 2. 将当前用户 nav01 加入组*
sudo usermod -aG docker $USER
*# 3. 立即刷新组权限（仅对当前窗口生效，建议重启）*
newgrp docker

*# 重新加载配置并设置开机自启*
sudo systemctl daemon-reload
sudo systemctl enable docker
sudo systemctl start docker

*# 检查状态*
sudo systemctl status docker

sudo systemctl daemon-reload
sudo systemctl restart docker

docker version
```

3. 容器部署

```Plain Text
*##本地传输tar包*
scp tele-image-delivery-v1.tar nav01@192.168.217.66:/home/nav01
*##加载镜像到本地Docker环境（进入pico）*
docker load -i tele-image-delivery-v1.tar
*##查看镜像*
docker images

mkdir -p tele-workspace
cd tele-workspace
mkdir -p tele-delivery
*#查看ip*
echo $ROS_MASTER_URI

*#创建开发容器*
docker run -d -it --name tele-delivery-container --privileged --network host --shm-size 32g --cap-add sys_nice \
  -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /dev:/dev \
  -v /home/nav01/tele-workspace:/root/workspace \
  -e ROS_MASTER_URI=http://192.168.217.1:11311 \
  -e ROS_IP=192.168.217.66 \
  tele-image:delivery-v1 /bin/bash
```

4. 代码部署

```Plain Text
*##传输zip文件  **  *
scp mpc-delivery-install-v0.0.20.zip nav01@192.168.217.66:/home/nav01/tele-workspace/tele-delivery

*##解压zip（进入pico）*
cd tele-workspace/tele-delivery/
sudo apt install p7zip-full
7z x mpc-delivery-install-v0.0.20.zip

*##网络问题解压文件*
unzip mpc-delivery-install-v0.0.20.zip
```

- 编译整个工作空间可能会网络错误

```Plain Text
*##查看wifi*
sudo nmcli device wifi
*##重启wifi*
sudo systemctl restart NetworkManager.service
*##连接wifi*
sudo nmcli dev wifi connect "wifi名字" password "密码"
```

功能测试

```Plain Text
docker start tele-delivery-container
docker exec -it tele-delivery-container bash
cd workspace/tele-delivery
source install/setup.bash

*# for wa1*
rosrun mpc_hardware_interface wa1_hardware_interface_node
*# for wa2*
rosrun mpc_hardware_interface wa2_hardware_interface_node
*# for wa2 ls*
rosrun mpc_hardware_interface wa2ls_hardware_interface_node

*##另起终端*
docker exec -it tele-delivery-container bash
cd workspace/tele-delivery
source install/setup.bash

*# for wa1*
roslaunch mpc_target wa1_mm_realworld_base_target.launch
*# for wa2*
roslaunch mpc_target wa2_mm_realworld_base_target.launch
*# for wa2 ls*
roslaunch mpc_target wa2ls_mm_realworld_base_target.launch

*# 双臂协同 *
roslaunch mpc_target wa2ls_mm_realworld_collaborative_target.launch 


```



