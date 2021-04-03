# 虚拟机调试记录

system base: vmware ubuntu 18.05, 20G, 8Core, 150G

### V2ray

```bash
sudo apt install root-system-bin
```

### 共享文件夹

vmware中虚拟机->设置->选项->共享文件夹->总是启用

路径在虚拟机中为

```bash
/mnt/hgfs/folder_name
```

###  cmake

```bash
sudo apt-get install gcc g++ cmake
wget https://github.com/Kitware/CMake/releases/download/v3.17.2/cmake-3.17.2.tar.gz \
    && tar -zxvf cmake-3.17.2.tar.gz \
    && cd cmake-3.17.2 \
    && ./bootstrap \
    && make \
    && sudo make install \
    && cd .. \
    && rm -rf cmake-3.17* 
```

* 先 `sudo apt-get update` `sudo apt upgrade`
* `sudo apt-get install libssl-dev`

###  交叉编译工具链

https://pan.horizon.ai/index.php/s/d3QH3MfzHT5fwd2

/opt

```bash
cd /opt
sudo xz -d gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu.tar.xz
sudo tar xvf gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu.tar
# 关注解压后的路径
sudo vim /etc/environment
# 修改为
PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin"
source /etc/environment
```

### 执行编译

```bash
cd /home/docker/AI-EXPRESS-master
bash build.sh x3
bash deploy.sh
# 此时在当前目录下生成deploy文件
# 板端
du -h -d 1
fdisk -l
cd /userdata
mkdir demo_trial_1
# 服务器端
scp -r deploy root@192.168.1.10:/userdata/demo_trial_1

```

* Permission denied的问题，转到目录后用chmod：`cd /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu`  `chmod -R 777`

* 找不到cc1的问题，把下面的路径加入path `/opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/libexec/gcc/aarch64-linux-gnu/6.5.0/`  之前报错是因为用了错误的cc1，不是gcc里的cc1！

  最终的环境变量长下面这样，还要把as加进去。

  ```
  PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin:/opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/libexec/gcc/aarch64-linux-gnu/6.5.0/:/opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/aarch64-linux-gnu/bin:/snap/bin"
  ```

  

* 这个可能没有用，先放这存档吧

  ```
  ARCH=arm64
  export CROSS_COMPILE=/opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-
  ```

* 然后，因为我是复制到linux里解压的，不是直接从git上clone的，软连接全部爆炸了，需要手动cd到各个库里修复，就比较惨。举例：

  ``` 
  cd /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/aarch64-linux-gnu/lib64
  ln -sf libgcc_s.so.1 libgcc_s.so
  ln -sf libasan.so.3.0.0 libasan.so
  ln -sf libatomic.so.1.2.0 libatomic.so
  ln -sf libgfortran.so.3.0.0 libgfortran.so
  ln -sf libgomp.so.1.0.0 libgomp.so
  ln -sf libitm.so.1.0.0 libitm.so
  ln -sf liblsan.so.0.0.0 liblsan.so
  ln -sf libssp.so.0.0.0 libssp.so
  ln -sf libstdc++.so.6.0.22 libstdc++.so
  ln -sf libtsan.so.0.0.0 libtsan.so
  ln -sf libubsan.so.0.0.0 libubsan.so
  ```

  实操是直接`head -c 50 *.so* > ins.txt` 然后去txt里把所有ln -sf写好

  

### usb摄像机连接检测

``` bash
ls /sys/class/video4linux/ -all
```

![image-20210402004113223](H:\开发板\开发记录\虚拟机调试记录.assets\image-20210402004113223.png)



```bash
hrut_ipfull s 192.168.1.100 255.255.255.0 192.168.1.1
```

192.168.184.2