# hk_driver_ros
基于ros(noetic)的海康相机驱动
>断线重连
>设置曝光与增益

# TIPS
## 使用时记得在SVM上设置输出格式为BGR8

![相机设置.png](.//相机设置.png "相机设置(像素格式)")

## 如果编译有libusb报错
  ` Error loading xxx : undefined symbol: libusb_set_option      `

### 原因：

安装海康相机SDK后，系统原本的libusb依赖会被重新链接至海康SDK设置的路径，在使用外设时会引发冲突

### 解决方法：
需要检查以下文件 

    sudo gedit ~/.bashrc
    sudo gedit ~/.profile
    sudo gedit /etc/ld.so.conf
    sudo gedit /etc/profile

把 LD_LIBRARY_PATH = /opt/MVS 的部分注释或删去即可 

甚至可以链接回原本位置，在 ~/.bashrc 增加一行

  ` export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH     `
