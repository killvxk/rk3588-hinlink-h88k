## Introduction

将构建 GNU/Linux 发行版 rootfs 映像的 shell 脚本适用于瑞芯微平台。

## 可用发行版

* ubuntu 20.04 (Focal-X11)

```
sudo apt-get install binfmt-support qemu-user-static
sudo dpkg -i ubuntu-build-service/packages/*
sudo apt-get install -f
```

## 64 位 ubuntu 20.04 的使用

```
# 构建gnome版本基础镜像
./mk-base-gnome-ubuntu.sh

# 添加 rk overlay 层,并打包ubuntu-rootfs镜像
VERSION=debug ARCH=arm64 SOC=rk3588 ./mk-gnome-rootfs.sh
```

## Cross Compile for ARM Debian

[Docker + Multiarch](http://opensource.rock-chips.com/wiki_Cross_Compile#Docker)

## 特色功能

- Focal/20.04 与 Gnome X11 在 rk3588 上工作

- GPU/RGA 硬件加速图形显示

- 使用 gdm3 服务和自动登录

- QT+Gstreamer 和 MPV+ffmpeg 视频编码/解码工作

- 添加 rockchip-test 进行压力测试
