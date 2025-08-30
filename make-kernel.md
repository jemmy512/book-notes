# install toolchain

Need to install cross-compiling toolchain since Ubunut is x86_64 arch.

```sh
# arm
sudo apt-get install gcc-aarch64-linux-gnu libncurses5-dev  build-essential git bison flex libssl-dev

# x86_64
sudo apt install -y build-essential libncurses-dev bison flex libssl-dev libelf-dev bc git
```

# clone kernel

```sh
makdir -P /code && cd /code

# vpn doesn't support git:// protocol
nohup git clone https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git /code/linux &

git remote add stable https://git.kernel.org/pub/scm/linux/kernel/git/stable/linux.git
```

## fetch tag

```sh
# fetch all tags
git fetch --tags

# fetch src(remote) tag as dst(local) tag
git fetch stable refs/tags/v6.1.120:refs/tags/v6.1.120

# list specific tag commit hash
git ls-remote stable refs/tags/v6.1.120

# filter tags
git tag -l | grep "v6.1.120*"

git show v6.1.120
```

# make ramfs

Linux needs root fs to start, we can do this by make a ramfs by busybox.

## make busybox

```sh
wget -P /code/ https://busybox.net/downloads/busybox-1.33.1.tar.bz2
tar -xjf busybox-1.33.1.tar.bz2 -C /code/
cd /code/busybox-1.33.1/
```

* specify compiler platform

    ```sh
    export ARCH=arm64 export CROSS_COMPILE=aarch64-linux-gnu-
    ```

* config busybox

    Remove networking/tc.c

    ```sh
    make menuconfig ARCH=arm64 ---> Settings --->  [*] Build static binary (no shared libs)
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
    cd /code/busybox-1.33.1/_install
    mkdir -p etc/init.d dev lib
    ls
    # bin  dev  etc  lib  linuxrc  sbin  usr
    ```

* vim etc/profile

    This is a shell configuration file executed when a user logs into a shell

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

    This is the configuration file for the init process (PID 1), which is the first process started by the Linux kernel during boot. It tells init how to manage system initialization, runlevels, and processes.

    ```sh
    ::sysinit:/etc/init.d/rcS   #Runs the /etc/init.d/rcS script at boot.
    ::respawn:-/bin/sh          # Starts a login shell
    ::askfirst:-/bin/sh
    ::ctrlaltdel:/bin/umount -a -r  # Defines behavior for Ctrl+Alt+Del
    # ::ctrlaltdel:/sbin/reboot
    ```

* vim etc/fstab

    This file lists filesystems to be mounted during boot, specifying their mount points, types, and options.

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

    This is a shell script executed during system initialization (often triggered by inittab’s sysinit entry). It performs early setup tasks.

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

    Without console, user output can't print

* lib dir

    copy system lib to lib

    ```sh
    cd /code/busybox-1.33.1/_install/lib
    cp /usr/aarch64-linux-gnu/lib/*.so*  -a .
    ```

copy the ramfs to /code/ramfs:

```sh
cp -r /code/busybox-1.33.1/_install/* /code/ramfs
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

generate .config by arch/arm64/configs/defconfig

```shell
make defconfig ARCH=arm64
```

* .config for ramfs:

    ```sh
    CONFIG_DEBUG_INFO=y
    CONFIG_INITRAMFS_SOURCE="/code/ramfs" # the root dir of the ramfs
    CONFIG_INITRAMFS_ROOT_UID=0
    CONFIG_INITRAMFS_ROOT_GID=0
    CONFIG_DEBUG_SECTION_MISMATCH=y # prohibit inline while debuging
    ```

* .config for diskfs

    ```sh
    CONFIG_BLK_DEV_INITRD=n
    CONFIG_INITRAMFS_SOURCE=""
    CONFIG_EXT4_FS=y
    CONFIG_VIRTIO_BLK=y
    ```

## make

```sh
make -j$(nproc) ARCH=arm64 Image  CROSS_COMPILE=aarch64-linux-gnu-
```

To speed up compile we only build Image, exclude modules.

Just press Enter to accept the default when encounter a choice.

## start

```sh
# with ramfs as root fs
qemu-system-aarch64 \
    -m 4096M -smp 8 \
    -cpu cortex-a57 \
    -machine virt \
    -kernel ./arch/arm64/boot/Image \
    -append "rdinit=/linuxrc nokaslr console=ttyAMA0 loglevel=8" \
    -nographic

# terminate qemu
ctl + a, x

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

[vs code installs c/c++ plugin](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools)

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

or config "configurations" to xxx.code-workspace:

```json
{
    "launch": {
        "version": "0.2.0",
        "configurations": [
            ...
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

## make on macOS

* [Kernel Programming Template for macOS + arm64 + VS Code](https://github.com/mastermakrela/kernel-dev)
* [Compiling Linux kernel on macOS](https://mastermakrela.com/kernel/lkp/kernel-dev-on-macos)
* [Building Linux kernel on macOS natively](https://seiya.me/blog/building-linux-on-macos-natively)

1. make a case-sensitive partiton

    Open Disk Utility.app and create an APFS (Case-sensitive) partition. You don't need to specify the size because it will scale automatically.

2. install tools and config environment

    ```sh
    brew install make llvm lld findutils gnu-sed libelf qemu
    ```

    ```sh
    # ~/.zshrc
    export OPENSSL_ROOT=$(brew --prefix openssl)
    # prefer llvm rather than default clang provided by xcode
    export PATH="$(brew --prefix llvm)/bin:$PATH"
    ```

3. make kernel

    ```sh
    # apply patch
    git clone https://github.com/jemmy512/kernel-dev.git /Volumes/code/kernel-dev
    cd /Volumes/code/linux
    patch -p1 < /Volumes/code/kernel-dev/mac_patch_6-5-7.patch

    make defconfig ARCH=arm64 LLVM=1

    time make Image -j$(sysctl -n hw.ncpu) ARCH=arm64 LLVM=1 HOSTCFLAGS="-Iscripts/macos-include -I$(brew --prefix libelf)/include"
    ```
4. start qemu

    [Download the Arch Linux for UTM](https://mac.getutm.app/gallery/archlinux-arm)

    ```sh
    # -nic vmnet-shared needs root privilege
    sudo qemu-system-aarch64 \
        -machine virt \
        -cpu max \
        -m 8196 \
        -drive file=/Volumes/code/ArchLinux.utm/Data/BB208CBD-BFB4-4895-9542-48527C9E5473.qcow2,format=qcow2 \
        -kernel "/Volumes/code/linux/arch/arm64/boot/Image" \
        -nic vmnet-shared \
        -append "root=/dev/vda2" -nographic
    ```

## hvp

* [在 MacOS 上做 Linux 内核开发的一些有趣经验和坑](https://blog.hackret.com/2023/07/564/)

```sh
brew install libvirt
sudo brew services start libvirt
sudo virtlogd -d

mkdir -p /Users/jemmy/.config/libvirt/qemu/nvram/
cp /opt/homebrew/Cellar/qemu/10.0.3/share/qemu/edk2-arm-vars.fd /Users/jemmy/.config/libvirt/qemu/nvram/Arch_VARS.fd

# arch-linux.xml is in the same path with make-kernel.md
sudo virsh define /Users/jemmy/.config/libvirt/arch-linux.xml
sudo virsh undefine Arch --nvram

sudo virsh list --all

sudo virsh start Arch
sudo virsh shutdown Arch
sudo virsh destroy Arch

sudo virsh console Arch

# mount shared directory
sudo mount -t 9p code -o trans=virtio /code
# -t 9p: Specifies the 9p filesystem.
# code: The target name from the XML.
# -o trans=virtio: Uses the virtio transport for performance.
# /code: The local mount point in the guest.

mount | grep 9p
df -h /code
ls /code

# persistent the mount, add the cfg to /etc/fstab
code /code 9p trans=virtio,nofail 0 0

# unmount
lsof /code
sudo fuser -km /code
sudo umount /code
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
# /etc/gitconfig  # system-wide config
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
    # only enable if send mail by msmtp
    sendmailCmd = /usr/bin/msmtp
[https]
	proxy = http://localhost:7890
[http]
	proxy = http://localhost:7890
```

## msmtprc

```sh
sudo apt-get install msmtp
sudo apt install git-email

chmod 600 ~/.msmtprc
# allow gmail 587 port
sudo iptables -A OUTPUT -p tcp --dport 587 -j ACCEPT
```

```sh
# ~/.msmtprc
account default
host smtp.gmail.com
port 587
auth on
user xxx@gmail.com
password xxx
from xxx@gmail.com
tls on
tls_starttls on
logfile ~/.msmtp.log
```

# send patch

* https://blog.xzr.moe/archives/293/
* https://tinylab.org/mailing-list-intro/

* send at first time

    ```sh
    cd linux

    git add xxx.c
    git commit -s
    git format-patch -<n> -v<n> origin --cover-letter

    ./scripts/checkpatch.pl  vx-000x-name.patch
    ./scripts/get_maintainer.pl vx-000x-name.patch

    git send-email --to=xxx@gmail.com --cc=xxx@gmail.com vx-000x-name.patch
    ```

* send at subsequent time

    ```sh
    git add fs/namespace.c
    git commit -s --amend --no-edit
    git format-patch -<n> -v<n> origin

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

Test
```sh
curl --proxy http://localhost:7890 https://kernel.org

curl --proxy http://localhost:7890 https://github.com
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

    Host kernel.org
        Hostname kernel.org
        ProxyCommand nc -x localhost:7890 %h %p
    ```

* config apt

    ```sh
    # /etc/apt/apt.conf.d/95proxies
    Acquire::http::Proxy "http://127.0.0.1:7890/";
    Acquire::https::Proxy "http://127.0.0.1:7890/";

    # update apt cache
    sudo apt-get update
    ```

## clash.service

```sh
# sudo vim /etc/systemd/system/clash.service
[Unit]
Description=Clash VPN Daemon
After=network.target

[Service]
Type=simple
ExecStart=/usr/local/bin/clash -d /usr/local/etc/clash
Restart=on-failure
RestartSec=20s
StartLimitIntervalSec=60s
StartLimitBurst=3
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

## share macOS vpn to qemu

1. surge enables Enhanced Mode
2. macOS enables shareing network: setting -> general -> sharing -> internet sharing: from surge to thunderbolt bridge
3. add arg to qemu cmd: -nic vmnet-shared


# ssh

```sh
sudo apt install openssh-server
sudo service ssh status
sudo service ssh start
sudo nano /etc/ssh/sshd_config
sudo systemctl restart sshd.service

sudo passwd root # set root user pwd

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

## Remove GUI

```sh
sudo apt purge ubuntu-desktop gnome-shell gdm3
sudo apt autoremove

# Set the system to boot into multi-user (non-GUI) mode:
sudo systemctl set-default multi-user.target
```

## Power Management:

Install tlp to optimize battery life:
```sh
sudo apt install tlp
sudo systemctl enable tlp
```

## network

static ip:
```yaml
# /etc/netplan/01-netcfg.yaml
# /etc/netplan/90-NM-e3f02c4a-7917-42a7-811c-c01f902752db.yaml
network:
  version: 2
  renderer: NetworkManager
  wifis:
    wlp3s0:
      addresses:
        - 192.168.1.100/24
      gateway4: 192.168.1.1
      nameservers:
        addresses: [192.168.1.1, 114.114.114.114, 8.8.8.8, 8.8.4.4]
      access-points:
        "Jemmy_5G":
          auth:
            key-management: "psk"
            password: "jm123456"
      networkmanager:
        uuid: "e3f02c4a-7917-42a7-811c-c01f902752db"
        name: "Jemmy_5G"
```

If using networkd, disable NetworkManager to avoid conflicts:
```sh
sudo systemctl disable NetworkManager
sudo systemctl stop NetworkManager # ssh will disconnect
sudo systemctl enable systemd-networkd
sudo systemctl start systemd-networkd
```

Apply the Configuration
```sh
sudo chmod 600 /etc/netplan/01-network-manager-all.yaml

sudo netplan generate
sudo netplan apply
```

Verify the Static IP
```sh
ip addr show wlp3s0
ping 8.8.8.8
ping google.com
```

## Firewall (UFW):

```sh
sudo apt install ufw
sudo ufw allow OpenSSH
sudo ufw enable
```

## screen off

```sh
gsettings get org.gnome.desktop.session idle-delay

# set screen turn off after 30s inactivity
gsettings set org.gnome.desktop.session idle-delay 30
```

```sh
sudo sh -c 'echo 30 > /sys/module/kernel/parameters/consoleblank'
# To make this persistent
sudo nano /etc/default/grub
# Modify or add the consoleblank parameter in the GRUB_CMDLINE_LINUX_DEFAULT line:
GRUB_CMDLINE_LINUX_DEFAULT="quiet splash consoleblank=30"

sudo update-grub
```

## Troubleshooting

* The brightness of the screen is not fully dark, or screen shows nothing after reboot:

    ```sh
    # restart display-manager
    systemctl restart display-manager
    ```

* errors in systemctl status display-manager

    ```sh
    systemctl status display-manager

    gkr-pam: couldn't unlock the login keyring.
    gdm3[4388]: Gdm: on_display_added: assertion 'GDM_IS_REMOTE_DISPLAY (display)' failed
    gdm3[4388]: Gdm: on_display_removed: assertion 'GDM_IS_REMOTE_DISPLAY (display)' failed
    ```

    * Fix Keyring Issue

        ```sh
        rm -rf ~/.local/share/keyrings/*

        # /etc/pam.d/gdm-password and /etc/pam.d/gdm-launch-environment
        auth    optional    pam_gnome_keyring.so
        session optional    pam_gnome_keyring.so auto_start

        sudo systemctl restart display-manager
        ```

    * Address GDM Assertion Failures

        ```sh
        # /etc/gdm3/custom.conf
        [daemon]
        # Ensure Wayland or Xorg is correctly set
        # If you’re not using Wayland, set WaylandEnable=false and test.
        WaylandEnable=true

        sudo systemctl restart display-manager
        ```

    * Reinstall GDB

        If the above steps don’t resolve the issue, reinstall GDM:
        ```sh
        sudo apt install --reinstall gdm3
        sudo systemctl restart display-manager
        ```

    * Fallback to Another Display Manager

        ```sh
        sudo apt install lightdm
        sudo systemctl disable gdm3
        sudo systemctl enable lightdm
        sudo systemctl start lightdm
        ````
## change boot menuentry

look up entry index in [/boot/grub/grub.cfg](/boot/grub/grub.cfg)
```sh
menuentry 'Ubuntu                                                       # 0
submenu 'Advanced options for Ubuntu' $menuentry_id_option
    menuentry 'Ubuntu, with Linux 6.15.0-rc2+'                          # 1>0
    menuentry 'Ubuntu, with Linux 6.15.0-rc2+ (recovery mode)'          # 1>1
    menuentry 'Ubuntu, with Linux 6.11.0-21-generic'                    # 1>2
    menuentry 'Ubuntu, with Linux 6.11.0-21-generic (recovery mode)'    # 1>3
menuentry "Memory test (memtest86+x64.efi)"                             # 2
menuentry 'Memory test (memtest86+x64.efi, serial console)'             # 3
```

set defualt entry in [/etc/default/grub](/etc/default/grub)
```sh
GRUB_DEFAULT="1>2" # set entry to "menuentry 'Ubuntu, with Linux 6.11.0-21-generic'"
```

```sh
update-grub

reboot
```

## prevent auto sleep


```sh
sudo vim /etc/systemd/logind.conf
```

```sh
# Controls the action when the lid is closed (on battery or AC).
# HandleLidSwitch=suspend
HandleLidSwitch=ignore

# Applies when the laptop is on AC power.
# HandleLidSwitchExternalPower=suspend
HandleLidSwitchExternalPower=ignore

# Applies when connected to a dock or external display (if applicable).
HandleLidSwitchDocked=ignore
```

```sh
# turn off screen, max 65535
echo 0 > /sys/class/backlight/gmux_backlight/brightness

cat /sys/class/backlight/gmux_backlight/max_brightness

sudo systemctl restart systemd-logind
sudo reboot
```

# frp

* [GitHub Doc](https://github.com/fatedier/frp/blob/dev/README.md)
* download: https://github.com/fatedier/frp/releases

    ```sh
    cd /usr/local/bin
    wget xxxx
    gunzip xxx # tar -zvf xxx

    # mv frpc to client: /usr/local/bin
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

# vscode

## gtags

```sh
# macOs
brew install global

# ubuntu
sudo apt install global
```

[VS Code install C/C++ GNU Global plugin](https://marketplace.visualstudio.com/items/?itemName=jaycetyle.vscode-gnu-global)

VS code config:
```sh
# ubuntu
"gnuGlobal.gtagsExecutable": "/usr/bin/gtags",
"gnuGlobal.globalExecutable": "/usr/bin/global",

# macOS M chip
"gnuGlobal.gtagsExecutable": "/opt/homebrew/bin/gtags",
"gnuGlobal.globalExecutable": "/opt/homebrew/bin/global",

# macOs Intel chip
"gnuGlobal.gtagsExecutable": "/usr/local/bin/gtags",
"gnuGlobal.globalExecutable": "/usr/local/bin/global",
```

Usge:
```sh
# execute cmd in the root directory of the code
gtags -i

# enter F12 in vs code to goto the definition of symbol
```

[gtags.conf](https://fossies.org/linux/global/gtags.conf)
```sh
# ~/.globalrc
default:\
    :tc=native:

native:\
    :tc=gtags:

common:\
    :skip=out/,*.pid,*.sock:

gtags:\
    :tc=common:

# gtags --gtagsconf=~/.globalrc
```

# update ubuntu kernel

```sh
# cp kernel config from current system
cp /boot/config-$(uname -r) /code/ubuntu/.config

# reset config in .config
CONFIG_SYSTEM_TRUSTED_KEYS=""
CONFIG_SYSTEM_REVOCATION_KEYS=""
```

```sh
make -j$(nproc) && make modules -j$(nproc)

sudo make modules_install && sudo make install && sudo update-grub && sudo reboot

uname -a
```

| key | value
| :-: | :-:
| Linux | Kernel name: Indicates the operating system kernel is Linux, managing hardware and resources.
| jemmy | Hostname: The unique name of the machine, "jemmy," used for network or local identification.
| 6.15.0-rc2+ | Kernel version: Breaks down as 6.15.0 (major.minor.patch), rc2 (Release Candidate 2, a pre-release test version), + (custom patches or modifications applied).
| #10 | Build number: The 10th build of this kernel configuration on this system.
| SMP | Symmetric Multiprocessing: Kernel supports multiple CPU cores for parallel processing.
| PREEMPT_RT | Preempt Real-Time: Kernel is patched for real-time capabilities, prioritizing low-latency task scheduling for time-critical applications.
| Sun Apr 20 14:59:36 CST 2025  | Build timestamp: When the kernel was compiled (April 20, 2025, at 14:59:36 China Standard Time).
| x86_64 | CPU architecture (running): The kernel is running on a 64-bit x86 processor.
| x86_64 | Compiled architecture: The kernel was compiled for a 64-bit x86 architecture.
| x86_64 | User-space architecture: User programs interact with the system as 64-bit applications.
| GNU/Linux | OS flavor: Indicates a Linux system using GNU userland tools (standard utilities and libraries).

# new partition

```sh
# Identify the new partition:
sudo fdisk -l

# Format the partition as ext4:
sudo mkfs.ext4 /dev/nvme0n1p3

# Verify the filesystem:
sudo fsck /dev/nvme0n1p3

# Temporarily mount the new partition to test:
sudo mount /dev/nvme0n1p3 /code

# Verify the mount:
df -h /code

# Adjust permissions:
sudo chown root:root /code
sudo chmod u+rw /code

# Make the Mount Permanent
# Get the UUID of the new partition:
lsblk -f

# Edit /etc/fstab:
sudo nano /etc/fstab

# Add an entry:
UUID=4b363ad7-7792-4bdf-b2cb-069a8c278e4e /code ext4 defaults 0 2
# defaults: Standard mount options.
# 0 2: No fsck for non-root, check order 2.

# Test the fstab, If no errors, the configuration is correct.
sudo mount -a

# Verify:
df -h /code
```

# build perf

```sh
cd kernel_path/tools/perf
mkdir arm64-cross
cd arm64-cross
```

download following libs from [archlinuxarm.org](https://archlinuxarm.org/packages/aarch64/perf), unzip to current dir:

```sh
audit-4.0.5-1-aarch64.pkg.tar
bash-5.2.037-5-aarch64.pkg.tar
binutils-2.44-1-aarch64.pkg.tar
coreutils-9.7-1-aarch64.pkg.tar
elfutils-0.193-2-aarch64.pkg.tar
filesystem-2025.05.03-1-any.pkg.tar
glib2-2.84.2-1-aarch64.pkg.tar
glibc-2.41+r6+gcf88351b685d-1-aarch64.pkg.tar
iana-etc-20250502-1-any.pkg.tar
jansson-2.14.1-1-aarch64.pkg.tar
json-c-0.18-2-aarch64.pkg.tar
libcap-2.76-1-aarch64.pkg.tar
libelf-0.193-2-aarch64.pkg.tar
libtraceevent-1.8.4-1-aarch64.pkg.tar
libunwind-1.8.1-3-aarch64.pkg.tar
llvm-libs-19.1.7-2-aarch64.pkg.tar
numactl-2.0.19-1-aarch64.pkg.tar
openssl-3.5.0-1-aarch64.pkg.tar
perf-6.14-1-aarch64.pkg.tar
perl-5.40.2-1-aarch64.pkg.tar
python-3.13.3-1-aarch64.pkg.tar
slang-2.3.3-3-aarch64.pkg.tar
xz-5.8.1-1-aarch64.pkg.tar
zlib-1_1.3.1-2-aarch64.pkg.t
```

copy libs under /usr to lib
```sh
cp usr/include/* include/ -r
cp usr/lib/* lib/ -r
```

build cmd:
```sh
# 1. clean build dir
make clean

# 2. setup env
CROSS_PATH=$(realpath arm64-cross)
export CFLAGS="--sysroot ${CROSS_PATH} -isystem ${CROSS_PATH}/include -isystem ${CROSS_PATH}/include/gnu -O2 -Wall"
export LDFLAGS="-L${CROSS_PATH}/lib -Wl,--sysroot=${CROSS_PATH} -lelf -ltraceevent -lcrypto -ldl -lm"
export PKG_CONFIG_PATH="${CROSS_PATH}/lib/pkgconfig"
export ARCH=arm64
export CROSS_COMPILE=aarch64-linux-gnu-

# 3. build - forcely skip denpendencies check
make -j$(nproc) ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- \
    NO_SDT=1 NO_SLANG=1 NO_PERL=1 NO_PYTHON=1 \
    LIBELF_FOUND=yes LIBTRACEEVENT_FOUND=yes LIBCRYPTO_FOUND=yes \
    CC="aarch64-linux-gnu-gcc ${CFLAGS}" \
    CXX="aarch64-linux-gnu-g++ ${CFLAGS}"

# cp perf to root
cp perf ../../root/usr/bin

# cp .so to root since static compilation doesnt work
cp arm64-cross/include/ ../../root/ -r
cp cp arm64-cross/lib/* ../../lib/ -r

# open PERF and BPF related config in kernel
```