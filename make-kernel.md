# 安装编译工具链

由于Ubuntu是X86架构，为了编译arm64的文件，需要安装交叉编译工具链

```sh
sudo apt-get install gcc-aarch64-linux-gnu libncurses5-dev  build-essential git bison flex libssl-dev
```

# 下载linux最新代码

```sh
nohup git clone git://git.kernel.org/pub/scm/linux/kernel/git/stable/linux-stable.git ./linux &
```

# 制作根文件系统

linux的启动需要配合根文件系统，这里我们利用busybox来制作一个简单的根文件系统

## 编译busybox

```sh
wget  https://busybox.net/downloads/busybox-1.33.1.tar.bz2
tar -xjf busybox-1.33.1.tar.bz2
cd busybox-1.33.1
```

打开静态库编译选项
```sh
make menuconfig ---> Settings --->  [*] Build static binary (no shared libs)
```

指定编译工具
```sh
export ARCH=arm64 export CROSS_COMPILE=aarch64-linux-gnu-
```

编译
```sh
make install
```

编译完成，在busybox目录下生成_install目录

## 定制文件系统

为了init进程能正常启动， 需要再额外进行一些配置

根目录添加etc、dev和lib目录

```sh
cd busybox-1.33.1/_install
mkdir etc dev lib
ls
# bin  dev  etc  lib  linuxrc  sbin  usr
```

在etc分别创建文件：
```sh
cd busybox-1.33.1/_install/etc
touch profile inittab fstab
```

profile 文件内容：
```sh
#!/bin/sh
export HOSTNAME=bryant
export USER=root
export HOME=/home
export PS1="[$USER@$HOSTNAME \W]\# "
PATH=/bin:/sbin:/usr/bin:/usr/sbin
LD_LIBRARY_PATH=/lib:/usr/lib:$LD_LIBRARY_PATH
export PATH LD_LIBRARY_PATH
```

inittab 文件内容：
```sh
::sysinit:/etc/init.d/rcS
::respawn:-/bin/sh
::askfirst:-/bin/sh
::ctrlaltdel:/bin/umount -a -r
```

fstab 文件内容：
```sh
#device  mount-point    type     options   dump   fsck order
proc /proc proc defaults 0 0
tmpfs /tmp tmpfs defaults 0 0
sysfs /sys sysfs defaults 0 0
tmpfs /dev tmpfs defaults 0 0
debugfs /sys/kernel/debug debugfs defaults 0 0
kmod_mount /mnt 9p trans=virtio 0 0
```

init.d/rcS 文件内容：
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

这里对这几个文件做一点说明：
1. busybox 作为linuxrc启动后， 会读取/etc/profile, 这里面设置了一些环境变量和shell的属性
2. 根据/etc/fstab提供的挂载信息， 进行文件系统的挂载
3. busybox 会从 /etc/inittab中读取sysinit并执行， 这里sysinit指向了/etc/init.d/rcS
4. /etc/init.d/rcS 中 ，mdev -s 这条命令很重要， 它会扫描/sys目录，查找字符设备和块设备，并在/dev下mknod

修改rcS的执行权限：
```sh
chmod 0777 rcS
```

dev目录：
```sh
cd busybox-1.33.1/_install/dev
sudo mknod console c 5 1
sudo mknod /dev/null c 1 3
```
这一步很重要， 没有console这个文件， 用户态的输出没法打印到串口上

lib目录：拷贝lib库，支持动态编译的应用程序运行：
```sh
cd busybox-1.33.1/_install/lib
cp /usr/aarch64-linux-gnu/lib/*.so*  -a .
```
## diskfs

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

# 编译内核
## 配置内核

linux内核源码可以在github上直接下载。

我们将之前制作好的根文件系统cp到root目录下：
```sh
cd linux
cp -r ../busybox-1.33.1/_install root
```

根据arch/arm64/configs/defconfig 文件生成.config:

```shell
make defconfig ARCH=arm64
```

将下面的配置加入.config文件中

```s
CONFIG_DEBUG_INFO=y
CONFIG_INITRAMFS_SOURCE="/code/rootfs"
CONFIG_INITRAMFS_ROOT_UID=0
CONFIG_INITRAMFS_ROOT_GID=0
CONFIG_DEBUG_SECTION_MISMATCH=y #调试时禁止内联。
```

## 修改优化等级

将优化等级修改为-O1，如果修改为-O0会编译不过去。同时在.config中增加CONFIG_DEBUG_SECTION_MISMATCH=y，调试时禁止内联。

```diff
diff --git a/Makefile b/Makefile
index 0992f827888d..87b2642baf52 100644
--- a/Makefile
+++ b/Makefile
@@ -452,7 +452,7 @@ HOSTRUSTC = rustc
 HOSTPKG_CONFIG = pkg-config

 KBUILD_USERHOSTCFLAGS := -Wall -Wmissing-prototypes -Wstrict-prototypes \
-                        -O2 -fomit-frame-pointer -std=gnu11 \
+                        -O1 -fomit-frame-pointer -std=gnu11 \
                         -Wdeclaration-after-statement
 KBUILD_USERCFLAGS  := $(KBUILD_USERHOSTCFLAGS) $(USERCFLAGS)
 KBUILD_USERLDFLAGS := $(USERLDFLAGS)
@@ -474,7 +474,7 @@ export rust_common_flags := --edition=2021 \
                            -Wclippy::dbg_macro

 KBUILD_HOSTCFLAGS   := $(KBUILD_USERHOSTCFLAGS) $(HOST_LFS_CFLAGS) $(HOSTCFLAGS)
-KBUILD_HOSTCXXFLAGS := -Wall -O2 $(HOST_LFS_CFLAGS) $(HOSTCXXFLAGS)
+KBUILD_HOSTCXXFLAGS := -Wall -O1 $(HOST_LFS_CFLAGS) $(HOSTCXXFLAGS)
 KBUILD_HOSTRUSTFLAGS := $(rust_common_flags) -O -Cstrip=debuginfo \
                        -Zallow-features= $(HOSTRUSTFLAGS)
 KBUILD_HOSTLDFLAGS  := $(HOST_LFS_LDFLAGS) $(HOSTLDFLAGS)
@@ -821,7 +821,7 @@ KBUILD_CFLAGS       += $(call cc-disable-warning, format-overflow)
 KBUILD_CFLAGS  += $(call cc-disable-warning, address-of-packed-member)

 ifdef CONFIG_CC_OPTIMIZE_FOR_PERFORMANCE
-KBUILD_CFLAGS += -O2
+KBUILD_CFLAGS += -O1
 KBUILD_RUSTFLAGS += -Copt-level=2
 else ifdef CONFIG_CC_OPTIMIZE_FOR_SIZE
 KBUILD_CFLAGS += -Os
diff --git a/include/linux/compiler_types.h b/include/linux/compiler_types.h
index eb0466236661..e650b0d66f01 100644
--- a/include/linux/compiler_types.h
+++ b/include/linux/compiler_types.h
@@ -354,7 +354,7 @@ struct ftrace_likely_data {
  * compiler has support to do so.
  */
 #define compiletime_assert(condition, msg) \
-       _compiletime_assert(condition, msg, __compiletime_assert_, __COUNTER__)
+       do { } while (0)

 #define compiletime_assert_atomic_type(t)                              \
        compiletime_assert(__native_word(t),                            \
diff --git a/include/linux/mm_types.h b/include/linux/mm_types.h
```

## 执行编译

```sh
make ARCH=arm64 Image -j$(nproc)  CROSS_COMPILE=aarch64-linux-gnu-
```

这里指定target为Image 会只编译kernel, 不会编译modules， 这样会增加编译速度遇到选择直接按回车默认。

## 启动内核

With ramfs
```sh
qemu-system-aarch64 \
    -m 4096M -smp 8 \
    -cpu cortex-a57 \
    -machine virt \
    -kernel arch/arm64/boot/Image \
    -append "rdinit=/linuxrc nokaslr console=ttyAMA0 loglevel=8" \
    -nographic
```

With diskfs
```sh
qemu-system-aarch64 \
    -m 4096M \
    -smp 8 \
    -cpu cortex-a57 \
    -machine virt \
    -kernel arch/arm64/boot/Image \
    -hda /code/rootfs.ext4 \
    -append "root=/dev/vda rw nokaslr console=ttyAMA0 loglevel=8" \
    -nographic
```

## gdb调试

vs code installs c/c++ plugin

```sh
qemu-system-aarch64 \
    -m 4096M -smp 8 -cpu cortex-a57 \
    -machine virt -kernel arch/arm64/boot/Image \
    -append "rdinit=/linuxrc nokaslr console=ttyAMA0 loglevel=8" \
    -nographic -S -gdb tcp::6688
```

创建./vscode/launch.json:
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

或者配置到 xxx.code-workspace:
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

# mail 配置

Configuring Custom SMTP Settings for Gmail
1. In your Google/Gmail account, go to svgexport-10Settings - See all settings.
2. Select the Forwarding and POP/IMAP tab.
3. Under the IMAP access section, toggle on the option to Enable IMAP.

Enable 2-Step Verification
1. Navigate to your [Google Account: App Passwords](https://myaccount.google.com/apppasswords)
2 Name your app Accredible, then click the Create button.
3. A new app password will be generated for you. Copy this password.

## .gitconfig

```sh
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

修改~/.msmtprc
```sh
defaults
auth           on
tls            on
tls_trust_file /etc/ssl/certs/ca-certificates.crt
logfile        ~/.msmtp.log

account        gmail
host           smtp.gmail.com
port           587
from           xxx@gmail.com
user           xxx@gmail.com
password       xxx
```

```sh
sudo apt-get install msmtp

chmod 600 ~/.msmtprc
sudo ln -s /usr/bin/msmtp /usr/sbin/sendmail

```

# 提交代码

* https://blog.xzr.moe/archives/293/
* https://tinylab.org/mailing-list-intro/

修改~/.gitconfig:
```sh
[user]
    name = xxx
    email = xxx@gmail.com
[sendemail]
    smtpserver = /usr/bin/msmtp
```

```sh
git config --global user.name "xxx"
git config --global user.email "xxx@gmail.com"
```

首次提交

```sh
cd linux

git add xxx.c
git commit -s
git format-patch -<n> -v<n> origin

./scripts/checkpatch.pl  v2-0001-xxx.patch v2-0002-xxx.patch
./scripts/get_maintainer.pl v2-0001-xxx.patch v2-0002-xxx.patch

git send-email --to=xxx@gmail.com --cc=xxx@gmail.com v2-0001-xxx.patch
```

后续提交

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

* `-<n>` 最近的n次commit都分别打成patch
* `--cover-latter` 为最近n次提交的patch生成一个 [PATCH 0/n] 的介绍
* `--annotate` 回复代码 comment
* `--subject-prefix="PATCH <MOD NAME>"`
* `--in-reply-to` message id 可以在 https://lore.kernel.org/all/ 根据patch名字找到

# Q&A

1.如果启动的时候发现rcS里面的配置没有生效，即没有映射/sys /tmp /proc等，需要将rcS修改为可执行权限
2.如果发现无法进入shell，可能是在root/dev下面没有console和null，这个拷贝需要sudo



# macos

brew install aarch64-linux-gnu-binutils
brew tap messense/macos-cross-toolchains
brew install aarch64-unknown-linux-gnu

aarch64-linux-gnu-gcc --version

export CROSS_COMPILE=aarch64-linux-gnu-

echo 'export PATH="/opt/homebrew/opt/ncurses/bin:$PATH"' >> /Users/jemmy/.zshrc
export LDFLAGS="-L/opt/homebrew/opt/ncurses/lib"
export CPPFLAGS="-I/opt/homebrew/opt/ncurses/include"
