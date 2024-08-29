# MoveIt on Ubuntu

**文档说明：**

- Tutorials: 创建你的第一个Movelt工程
- How-to Guide: 回答“如何用Movelt去完成 X”
- Concepts: 介绍Movelt的设计
- Contribution: 介绍如对Movelt进行修改
- API Documentaion: 去往API 参考链接



## 开始：

首先需要进行环境的设置，以便最好的运行相关教程案例。这将创建一个colcon工作区，下载最新的Movelt源代码，并从中构建所有内容，确保一切都是最新的。

根据使用的计算机的CPU 以及可用的RAM，构建Movelt的源码需要20-30分钟，如果使用的系统性能较差，或者想要更快速的入門，可以参考Docker Guide



**ROS2 和 colcon的安装**

MoveIt 支持多版本ROS，可以根据个人喜好安装，目前主要支持在Ubuntu 22.04上安装的ROS

> ROS的安装参考：
>
> Iron Irwini: https://docs.ros.org/en/iron/Installation.html
>
> Humble Hawksbill: https://docs.ros.org/en/humble/Installation.html

ROS2安装完成后，记得source一下：`source /opt/ros/<version that you installed>/setup.bash`


>colcon的安装：
>`
>  sudo apt install python3-colcon-common-extensions
>  sudo apt install python3-colcon-mixin
>  colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
>  colcon mixin update default
>  `

**依赖安装：**通过`rosdep`进行相关依赖的安装:    

```bash
sudo rosdep init
rosdep update
sudo apt update
sudo apt dist-upgrade
```

>若运行`rosdep update'失败，可以参考以下内容：
>
>- https://fishros.org.cn/forum/topic/676/rosdepc-update%E5%B8%B8%E8%A7%81%E9%94%99%E8%AF%AF%E5%8F%8A%E8%A7%A3%E5%86%B3%E6%96%B9%E6%A1%88
>- https://blog.csdn.net/weixin_53660567/article/details/120607176?spm=1001.2014.3001.5501
>- https://blog.csdn.net/qq_43439214/article/details/124785883

**安装`vcstoll`:**`sudo apt install python3-vcstool`

**创建colcon workspace:**`mkdir -p ~/ws_moveit/src`

**下载MoveIt源码以及Tutorials:**

```bash
cd ~/ws_moveit/src
git clone -b <branch> https://github.com/moveit/moveit2_tutorials
vcs import --recursive < moveit2_tutorials/moveit2_tutorials.repos
```

<branch>是ROS对应的版本名称，例如在ROS Humble下：`git clone -b humble https://github.com/moveit/moveit2_tutorials`，`main`对应最新的tutorials

**构建colcon workspace:**

```bash
sudo apt remove ros-$ROS_DISTRO-moveit*
sudo apt update && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
cd ~/ws_moveit
colcon build --mixin release
```

如果一切顺利，应该会看到提示：**Summary: X packages finished**，其中 X 大概是 50左右

>源码构建会消耗大量RAM资源，可以通过一下命令进行设置：
>
>```bash
>sudo swapoff /swapfile		#关闭当前的swapfile
>sudo fallocate -l 4G /swapfile 	#重新设置swapfile的大小 4G
>sudo chmod 600 /swapfile 	#设置访问权限
>sudo mkswap /swapfile  	        #将文件格式设置为 swapfile
>sudo swapon /swapfile	      #使能对应文件
>```

**colcon workspace的设置：**

```bash
source ~/ws_moveit/install/setup.bash
echo 'source ~/ws_moveit/install/setup.bash' >> ~/.bashrc
```

