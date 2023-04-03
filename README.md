# rk3588-hinlink-h88k  正在建设。。。。
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
 git clone https://github.com/DHDAXCW/rk3588-hinlink-h88k
 ```
 进入目录 ```cd rk3588-hinlink-h88k```
 
- 执行设置SDK配置文件
 ```./build.sh lunch```
 
 - 选择h88k，然后执行 ```./build.sh kernel kerneldeb uboot```编译kernel和uboot。内核deb在当前路径存着  <---这个必须有，不然Ubuntu建构会失败~
 
