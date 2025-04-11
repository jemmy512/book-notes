# install toolchain

Need to install cross-compiling toolchain since Ubunut is x86_64 arch.

```sh
# arm
sudo apt-get install gcc-aarch64-linux-gnu libncurses5-dev  build-essential git bison flex libssl-dev

# x86_64
sudo apt install -y build-essential libncurses-dev bison flex libssl-dev libelf-dev bc git
```

# download kernel

```sh
nohup git clone git://git.kernel.org/pub/scm/linux/kernel/git/stable/linux-stable.git ./linux &
```

# make ramfs

Linux needs root fs to start, we can do this by make a ramfs by busybox.

## make busybox

```sh
wget  https://busybox.net/downloads/busybox-1.33.1.tar.bz2
tar -xjf busybox-1.33.1.tar.bz2
cd busybox-1.33.1
```

* config busybox

    ```sh
    make menuconfig ---> Settings --->  [*] Build static binary (no shared libs)
    ```

* specify compiler platform

    ```sh
    export ARCH=arm64 export CROSS_COMPILE=aarch64-linux-gnu-
    ```
* make

    ```sh
    make install
    ```

_install dir under busybox is generated after successful make

## config ramfs

We need additional config to start init process:

* mkdir mkdir etc dev lib

    ```sh
    cd busybox-1.33.1/_install
    mkdir -p etc/init.d dev lib
    ls
    # bin  dev  etc  lib  linuxrc  sbin  usr
    ```

* vim etc/profile

    ```sh
    #!/bin/sh
    export HOSTNAME=jemmy
    export USER=root
    export HOME=/home
    export PS1="[$USER@$HOSTNAME \W]\# "
    PATH=/bin:/sbin:/usr/bin:/usr/sbin
    LD_LIBRARY_PATH=/lib:/usr/lib:$LD_LIBRARY_PATH
    export PATH LD_LIBRARY_PATH
    ```

* vim etc/inittab

    ```sh
    ::sysinit:/etc/init.d/rcS
    ::respawn:-/bin/sh
    ::askfirst:-/bin/sh
    ::ctrlaltdel:/bin/umount -a -r
    ```

* vim etc/fstab

    ```sh
    #device  mount-point        type     options    dump fsck order
    proc    /proc               proc    defaults    0   0
    tmpfs   /tmp                tmpfs   defaults    0   0
    sysfs   /sys                sysfs   defaults    0   0
    tmpfs   /dev                tmpfs   defaults    0   0
    debugfs /sys/kernel/debug   debugfs defaults    0   0
    kmod_mount /mnt             9p      trans=virtio 0  0
    ```

* vim etc/init.d/rcS

    ```sh
    mkdir -p /sys
    mkdir -p /tmp
    mkdir -p /proc
    mkdir -p /mnt
    /bin/mount -a
    mkdir -p /dev/pts
    mount -t devpts devpts /dev/pts
    mkdir -p /proc/sys/kernel/hotplug
    echo /sbin/mdev > /proc/sys/kernel/hotplug
    mdev -s
    ```

    ```sh
    chmod 0777 etc/init.d/rcS
    ```

* Note:
    1. busybox 作为linuxrc启动后， 会读取/etc/profile, 这里面设置了一些环境变量和shell的属性
    2. 根据/etc/fstab提供的挂载信息， 进行文件系统的挂载
    3. busybox 会从 /etc/inittab中读取sysinit并执行， 这里sysinit指向了/etc/init.d/rcS
    4. /etc/init.d/rcS 中 ，mdev -s 这条命令很重要， 它会扫描/sys目录，查找字符设备和块设备，并在/dev下mknod

* dev dir

    ```sh
    cd busybox-1.33.1/_install/dev
    sudo mknod console c 5 1
    sudo mknod /dev/null c 1 3
    ```

    这一步很重要， 没有console这个文件， 用户态的输出没法打印到串口上

* lib dir

    copy system lib to lib

    ```sh
    cd busybox-1.33.1/_install/lib
    cp /usr/aarch64-linux-gnu/lib/*.so*  -a .
    ```

# make diskfs

1. Create an Empty Disk Image

    ```sh
    cd /code
    dd if=/dev/zero of=rootfs.ext4 bs=1M count=512
    ```

2. Format the Image

    ```sh
    mkfs.ext4 rootfs.ext4
    ```


3. Mount and Populate the Image

    ```sh
    # copy busybox
    mkdir /code/mnt
    sudo mount -o loop rootfs.ext2 /code/mnt

    sudo cp -r /path/to/busybox-1.36.1/_install/* /code/mnt/

    # copy LTP
    sudo mkdir -p mnt/opt/ltp
    sudo cp -rL /opt/ltp/* mnt/opt/ltp/  # LTP for aarch64
    ```

4. Verify Size

    ```sh
    du -sh mnt/  # Before unmounting, if still mounted
    ls -lh rootfs.ext4
    ```

5. Unmount

    ```sh
    sudo umount mnt
    ```

6. kernel .config

    ```sh
    CONFIG_BLK_DEV_INITRD=n
    CONFIG_INITRAMFS_SOURCE=""
    CONFIG_EXT4_FS=y
    CONFIG_VIRTIO_BLK=y
    ```

## bash

有些内核测试脚本需要bash才能执行，busybox里面自带的是sh，只能支持posix的标准shell，因此需要自己下载编译bash，并安装到内核的root目录下

```sh
wget https://ftp.gnu.org/gnu/bash/bash-5.0.tar.gz
tar xf bash-5.0.tar.gz
cd bash-5.0
./configure --host=aarch64-linux-gnu --prefix=/home/xxx/linux/linux-stable/root
make
make install
```

fmt

```sh
wget http://ftp.gnu.org/gnu/coreutils/coreutils-9.0.tar.xz
tar xf coreutils-9.0.tar.xz
cd coreutils-9.0
./configure --host=aarch64-linux-gnu --disable-nls  --prefix=/home/xxx/linux/linux-stable/root/gnu
make
make install
```

# make kernel

## config

linux内核源码可以在github上直接下载。

我们将之前制作好的根文件系统cp到root目录下：

copy the ramfs to ../ramfs
```sh
cd linux
cp -r ../busybox-1.33.1/_install/* ../ramfs
```

generate .config by arch/arm64/configs/defconfig

```shell
make defconfig ARCH=arm64
```

Add additional config to .config:

```s
CONFIG_DEBUG_INFO=y
CONFIG_INITRAMFS_SOURCE="../ramfs" # the root dir of the ramfs
CONFIG_INITRAMFS_ROOT_UID=0
CONFIG_INITRAMFS_ROOT_GID=0
CONFIG_DEBUG_SECTION_MISMATCH=y # prohibit inline while debuging
```

## make

```sh
make -j$(nproc) ARCH=arm64 Image  CROSS_COMPILE=aarch64-linux-gnu-
```

To speed up compile we only build Image, exclude modules.

Just press Enter to accept the default when encounter a choice.

## start

```sh
# with ramfs root fs
qemu-system-aarch64 \
    -m 4096M -smp 8 \
    -cpu cortex-a57 \
    -machine virt \
    -kernel arch/arm64/boot/Image \
    -append "rdinit=/linuxrc nokaslr console=ttyAMA0 loglevel=8" \
    -nographic

# with diskfs as root fs
qemu-system-aarch64 \
    -m 4096M -smp 8 \
    -cpu cortex-a57 \
    -machine virt \
    -kernel arch/arm64/boot/Image \
    -hda /code/rootfs.ext4 \
    -append "root=/dev/vda rw nokaslr console=ttyAMA0 loglevel=8" \
    -nographic
```

## gdb

vs code installs c/c++ plugin

```sh
qemu-system-aarch64 \
    -m 4096M -smp 8 -cpu cortex-a57 \
    -machine virt -kernel arch/arm64/boot/Image \
    -append "rdinit=/linuxrc nokaslr console=ttyAMA0 loglevel=8" \
    -nographic -S -gdb tcp::6688
```

./vscode/launch.json:

```json
{
    // Use IntelliSense to learn about possible attributes.
    // Hover to view descriptions of existing attributes.
    // For more information, visit: https://go.microsoft.com/fwlink/?linkid=830387
    "version": "0.2.0",
    "configurations": [
        {
            "name": "kernel debug",
            "type": "cppdbg",
            "request": "launch",
            "program": "/code/linux/vmlinux",
            "cwd": "${workspaceFolder}",
            "MIMode": "gdb",
            "miDebuggerPath":"/usr/bin/gdb-multiarch",
            "miDebuggerServerAddress": "localhost:6688",
            "stopAtEntry": true,
            "setupCommands": [
                {
                    "description": "enable pretty print for gdb",
                    "text": "-enable-pretty-printing",
                    "ignoreFailures": true
                }
            ]
        }
    ]
}
```

or config it to xxx.code-workspace:

```json
{
    "launch": {
        "version": "0.2.0",
        "configurations": [
            {
                "name": "kernel debug",
                "type": "cppdbg",
                "request": "launch",
                "program": "/code/linux/vmlinux",
                "cwd": "${workspaceFolder}",
                "MIMode": "gdb",
                "miDebuggerPath":"/usr/bin/gdb-multiarch",
                "miDebuggerServerAddress": "localhost:6688",
                "stopAtEntry": true,
                "setupCommands": [
                    {
                        "description": "enable pretty print for gdb",
                        "text": "-enable-pretty-printing",
                        "ignoreFailures": true
                    }
                ]
            }
        ]
    }
}
```

## cleanup

```sh
# Removes most generated files (e.g., object files, binaries) but keeps the configuration (.config)
make clean

# Removes all generated files, including the .config file and backup files, returning the source tree to a pristine state
make mrproper

# Goes even further than mrproper by removing editor backup files, patch files, and other miscellaneous leftovers.
make distclean
```

# ltp

```sh
git clone https://github.com/linux-test-project/ltp.git
cd ltp
make autotools
./configure
make -j$(nproc)
sudo make install # Installs to /opt/ltp on your workstation

# Copy LTP into Rootfs
sudo cp -r /opt/ltp/* /code/kern/root/ltp

sudo /opt/ltp/runltp -f sched
sudo /opt/ltp/bin/ltp -t rt_sched
```

# mail config

Configuring Custom SMTP Settings for Gmail
1. In your Google/Gmail account, go to svgexport-10Settings - See all settings.
2. Select the Forwarding and POP/IMAP tab.
3. Under the IMAP access section, toggle on the option to Enable IMAP.

Enable 2-Step Verification
1. Navigate to your [Google Account: App Passwords](https://myaccount.google.com/apppasswords)
2. Name your app Accredible, then click the Create button.
3. A new app password will be generated for you. Copy this password.

## .gitconfig

```sh
# /etc/.gitconfig  # system-wide config
[b4]
    midmask = https://lore.kernel.org/%s
    thread = true
    save-to = /var/cache/b4/%s.mbox

[user]
    name = xxx
    email = xxx@gmail.com

[sendemail]
	smtpServer = smtp.gmail.com
	smtpServerPort = 587
	smtpEncryption = tls
	smtpUser = xxx@gmail.com
	smtpPass = xxx
	suppresscc = self
	confirm = always
```

## msmtprc

```sh
sudo apt-get install msmtp
sudo apt install git-email

chmod 600 ~/.msmtprc
sudo ln -s /usr/bin/msmtp /usr/sbin/sendmail
```

# send patch

* https://blog.xzr.moe/archives/293/
* https://tinylab.org/mailing-list-intro/

* config ~/.gitconfig
    ```sh
    [user]
        name = xxx
        email = xxx@gmail.com
    ```

    ```sh
    git config --global user.name "xxx"
    git config --global user.email "xxx@gmail.com"
    ```

* First send

    ```sh
    cd linux

    git add xxx.c
    git commit -s
    git format-patch -<n> -v<n> origin --cover-letter

    ./scripts/checkpatch.pl  v2-0001-xxx.patch v2-0002-xxx.patch
    ./scripts/get_maintainer.pl v2-0001-xxx.patch v2-0002-xxx.patch

    git send-email --to=xxx@gmail.com --cc=xxx@gmail.com v2-0001-xxx.patch
    ```

* subsequent send

    ```sh
    git add fs/namespace.c
    git commit -s --amend --no-edit
    git format-patch -v2 origin

    ./scripts/checkpatch.pl  v2-0001-xxx.patch v2-0002-xxx.patch

    git send-email \
    --in-reply-to=<message-id> \
    --to=dnlplm@xxx.yyy \
    --cc=bjorn@xxx.yyy \
    --subject='Re: [PATCH net 1/1] xxx' \
    ./v2-0001-xxx.patch ./v2-0002-xxx.patch
    ```

* `-<n>`: The latest n commits are all patched
* `--cover-latter`: Generate a [PATCH 0/n] introduction for the latest n patches
* `--annotate`: Reply code comment
* `--subject-prefix="PATCH <MOD NAME>"`
* `--in-reply-to`: Message id can be found at https://lore.kernel.org/all/ according to the patch name
# vpn

## download

* [clash-linux-amd64-v1.18.0.gz - For x86 / x64 Linux](https://pub-eac3eb5670f44f09984dee5c57939316.r2.dev/clash-linux-amd64-v1.18.0.gz)
* [clash-linux-arm64-v1.18.0.gz - For ARM Linux](https://pub-eac3eb5670f44f09984dee5c57939316.r2.dev/clash-linux-arm64-v1.18.0.gz)
* [clash-darwin-amd64-v1.18.0.gz - For intel macOS](https://pub-eac3eb5670f44f09984dee5c57939316.r2.dev/clash-darwin-amd64-v1.18.0.gz)
* [clash-darwin-arm64-v1.18.0.gz - For Apple silicon macOS](https://pub-eac3eb5670f44f09984dee5c57939316.r2.dev/clash-darwin-arm64-v1.18.0.gz)


```sh
gzip -dk clash-*-*-v1.18.0.gz
chmod +x clash-*-*-v1.18.0
cp clash-*-*-v1.18.0 /usr/local/bin/clash

mkdir /usr/local/etc/clash
# download proxy nodes
wget -P /usr/local/etc/clash https://***.*/feeds/***/clash.yml
mv /usr/local/etc/clash/clash.yml /usr/local/etc/clash/config.yaml

# run clash
# Country.mmdb may cant download, copy it from other place
clash -d /usr/local/etc/clash # default 127.0.0.1:7890
```

## config

* config terminal

    ```sh
    # ~/.bashrc
    export socks5='socks5://127.0.0.1:7891'
    export http_proxy='http://127.0.0.1:7890'
    export https_proxy='http://127.0.0.1:7890'
    ```

* config github

    ```sh
    # ~/.ssh/config
    Host github.com
        Hostname github.com
        ProxyCommand nc -x localhost:7890 %h %p
        # git-for-windows can replace nc with connect
        # ProxyCommand connect -S localhost:1085 %h %p
    ```

## clash.service

```sh
# sudo touch /etc/systemd/system/clash.service
[Unit]
Description=Clash VPN Daemon
After=network.target

[Service]
Type=simple
ExecStart=/usr/local/bin/clash -d /etc/clash
Restart=on-failure
User=nobody
Group=nogroup

[Install]
WantedBy=multi-user.target
```

```sh
sudo systemctl daemon-reload
sudo systemctl enable clash.service
sudo systemctl start clash.service
sudo systemctl status clash.service
sudo systemctl stop clash.service
```

# ssh

```sh
sudo apt install openssh-server
sudo service ssh status
sudo service ssh start
sudo nano /etc/ssh/sshd_config
sudo systemctl restart sshd.service

sudo passed # set root user pwd

ssh [username]@[public_IP] -p[port_number]
```

Avoid SSH timeout from the server

```sh
# /etc/ssh/sshd_config
TCPKeepAlive no
ClientAliveInterval 300
ClientAliveCountMax 2400

PermitRootLogin yes # allow root user ssh login
````

Avoid SSH timeout from the client

```sh
# ~/.ssh/config
ServerAliveInterval 300
```

# ubuntu on mac

* [Linux support for Apple devices with the T2 security chip](https://t2linux.org/#Installation)
* [Getting Wi-Fi and Bluetooth to work](https://wiki.t2linux.org/guides/wifi-bluetooth/)
* [Install drivers for the fan (if not working automatically or want to force a certain speed)](https://wiki.t2linux.org/guides/fan/)

# frp

* [GitHub Doc](https://github.com/fatedier/frp/blob/dev/README.md)
* download: https://github.com/fatedier/frp/releases

    ```sh
    cd /user/local/bin
    wget xxxx
    gunzip xxx # tar -zvf xxx

    # mv frpc to clinet: /usr/local/bin
    # mv frps to server: /usr/local/bin
    ```

## server

* config frps.toml

    ```sh
    mkdir -p /usr/local/etc/frps
    vim /usr/local/etc/frpc/frps.toml
    ```

    ```sh
    [common]
    # frp listen port
    bind_port = 7000
    token = 52010

    # frp management port
    dashboard_port = 7500
    dashboard_user = admin
    dashboard_pwd = admin
    enable_prometheus = true

    # frp log config
    log_file = /var/log/frps.log
    log_level = info
    log_max_days = 3
    ```
* create frps.service

    ```sh
    sudo vim /etc/systemd/system/frps.service
    ```

    ```sh
    [Unit]
    # service name
    Description = frp server
    After = network.target syslog.target
    Wants = network.target

    [Service]
    Type = simple
    ExecStart = /usr/local/bin/frps -c /usr/local/etc/frps/frps.toml
    Restart=on-failure
    RestartSec=20s
    StartLimitIntervalSec=60s
    StartLimitBurst=3

    [Install]
    WantedBy = multi-user.target
    ```

* enable service

    ```sh
    sudo systemctl daemon-reload
    sudo systemctl enable frps
    sudo systemctl start frps
    ```

## client

* config frpc.toml

    ```sh
    mkdir -p /usr/local/etc/frpc
    vim /usr/local/etc/frpc/frpc.toml
    ```

    ```sh
    serverAddr = "139.196.161.94"
    # serverPort is used for communication between frps and frpc
    serverPort = 7000

    [[proxies]]
    name = "ssh"
    type = "tcp"
    localIP = "127.0.0.1"
    # localPort (listened on the client) and remotePort (exposed on the server)
    # are used for traffic going in and out of the frp system
    localPort = 22
    remotePort = 6000
    ```

* config frpc.service

    ```sh
    sudo vim /etc/systemd/system/frpc.service
    ```

    ```sh
    [Unit]
    # service name
    Description = frp client
    After = network.target syslog.target
    Wants = network.target

    [Service]
    Type = simple
    ExecStart = /usr/local/bin/frpc -c /usr/local/etc/frpc/frpc.toml
    Restart=on-failure
    RestartSec=20s
    StartLimitIntervalSec=60s
    StartLimitBurst=3

    [Install]
    WantedBy = multi-user.target
    ```

* enable service

    ```sh
    sudo systemctl daemon-reload
    sudo systemctl enable frpc
    sudo systemctl start frpc
    ```

## test

```sh
ssh -o Port=6000 user@server-ip-addr
```

```sh
# ~/.ssh/config
Host mbp2018-remote
  HostName server-ip-addr
  User root
  Port 6000
```

# prevent mac from auto sleep


```sh
sudo vim /etc/systemd/logind.conf
```

```sh
# Controls the action when the lid is closed (on battery or AC).
HandleLidSwitch=ignore

# Applies when the laptop is on AC power.
HandleLidSwitchExternalPower=ignore

# Applies when connected to a dock or external display (if applicable).
HandleLidSwitchDocked=ignore
```

```sh
sudo systemctl restart systemd-logind
sudo reboot
```

# vscode

## gtags

```sh
sudo apt install global
```

[VS Code install C/C++ GNU Global plugin](https://marketplace.visualstudio.com/items/?itemName=jaycetyle.vscode-gnu-global)

VS code config:
```sh
# ubuntu
"gnuGlobal.globalExecutable": "/usr/bin/global",
"gnuGlobal.gtagsExecutable": "/usr/bin/gtags",

# macOS M chip
"gnuGlobal.globalExecutable": "/opt/homebrew/bin/global",
"gnuGlobal.gtagsExecutable": "/opt/homebrew/bin/gtags",
```

# replace ubuntu kernel

```sh
# cp kernel config from current system
cp /boot/config-$(uname -r) .config

# reset config in .config
CONFIG_SYSTEM_TRUSTED_KEYS=""
CONFIG_SYSTEM_REVOCATION_KEYS=""
```

```sh
make -j$(nproc)
make modules -j$(nproc)

sudo make modules_install
sudo make install

sudo update-grub
sudo reboot

uname -r
```