# rk3588-hinlink-h88k  教程正在完整中。。。。
### 内核 5.10.110，不适用5.10.66

# 建构Ubuntu20.04
- 本次教程仅用于ubuntu-rkr2版本。

### 编译环境搭建
- cpu ：4核以上
- 内存 ：8GB以上，如果cpu线程越多，内存必须给大空间。
- M2 nvme固态：200GB
- 系统要求Ubuntu20.04
- 安装依赖
```bash
sudo apt install git ssh make gcc libssl-dev liblz4-tool u-boot-tools curl\
expect g++ patchelf chrpath gawk texinfo chrpath diffstat binfmt-support \
qemu-user-static live-build bison flex fakeroot cmake gcc-multilib g++-multilib \
unzip device-tree-compiler python-pip libncurses5-dev python3-pyelftools dpkg-dev \
```
 ### 下载Ubuntu仓库
 ```
 待定
 ```
- 进入目录 ```cd rk3588-hinlink-h88k```
 
- 执行设置SDK配置文件
 ```./build.sh lunch```
 
 - 选择h88k，然后执行 ```./build.sh kernel kerneldeb uboot```编译kernel和uboot。内核deb在当前路径存着  <---这个必须有，不然Ubuntu建构会失败~
 - 其他编译结束后，进入Ubuntu目录，先安装依赖：
 ```bash
sudo apt-get install binfmt-support qemu-user-static
sudo dpkg -i ubuntu-build-service/packages/*
sudo apt-get install -f
```
- 构建desktop版本
```bash
构建 desktop 版本基础镜像
ARCH=arm64 ./mk-base-gnome-ubuntu.sh

添加 rk overlay 层,并打包ubuntu-rootfs镜像
VERSION=debug ./mk-gnome-rootfs.sh
```
- 最后形成rootfs-gnome.img创建超链rootfs.img接到rockdev文件夹下；
- 然后执行./build.sh updateimg打包
- 最终rockdev出update.img镜像包

