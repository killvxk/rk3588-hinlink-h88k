## 这是存放驱动

- fm-160-linux-usb-gobinet-diver用法

```bash
# 这是fm-160驱动必须打进去，不然你速度只有6m网速？骚不骚？

# kernel配置要打开
CONFIG_USB_USBNET=y
CONFIG_USB_NET_QMI_WWAN=y

# 驱动使用方法，将下面驱动文件拖进路径：kernel/drivers/net/usb/*
driverLoader.sh
GobiUSBNet.c
QMI.c
QMI.h
QMIDevice.c
QMIDevice.h
Structs.h
# 修改Makefile添加下面，不要编进内核，必须用ko，直接闭眼复制进去完事
obj-m := GobiNet.o
GobiNet-objs := GobiUSBNet.o QMIDevice.o QMI.o
# 最后编译。开机，上电，测速，网速450M左右，差不多就到这了。我仓库kernel默认加了，可以不打
```
