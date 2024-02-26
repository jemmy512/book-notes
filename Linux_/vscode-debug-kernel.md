# Download Kernel

> wget https://mirrors.tuna.tsinghua.edu.cn/kernel/v5.x/linux-5.16.10.tar.gz

> make menuconfig

enable these options:
```
Kernel hacking  --->
    [*] Kernel debugging
    Compile-time checks and compiler options  --->
        [*] Compile the kernel with debug info
        [*]   Provide GDB scripts for kernel debugging
```

disable these options:
```
Processor type and features ---->
    [] Randomize the address of the kernel image (KASLR)
```

> make

# dependencies

> sudo apt install libncurses5-dev libssl-dev bison flex libelf-dev gcc make openssl libc6-dev

## gcc

> sudo apt install software-properties-common
> sudo apt-get update
> sudo apt install gcc-9 g++-9

## gdb
wget https://mirror.bjtu.edu.cn/gnu/gdb/gdb-11.2.tar.xz

> tar -xvf gdb-8.3.tar.xz
> ./configure
> ./make
> ./make install

# qemu

> sudo apt install qemu qemu-system qemu-kvm

> qemu-system-x86_64 -no-reboot -nographic -kernel ./arch/x86/boot/bzImage -initrd ./rootfs -append "console=ttyS0 spectre_v2=off l1tf=off quiet kernel.panic=-1 tsc=unstable root=/dev/ram rdinit=/fakeinit" -smp 2 -s -S

[QEMU Doc](https://www.qemu.org/docs/master/system/invocation.html)

* -s
    * Shorthand for -gdb tcp::1234, i.e. open a gdbserver on TCP port 1234 (see the GDB usage chapter in the System Emulation Users Guide).
* -S
    * Do not start CPU at startup (you must type ‘c’ in the monitor).


# initrd
```c++
#include <stdio.h>
int main()
{
    printf("hello world!");
    printf("hello world!");
    printf("hello world!");
    printf("hello world!");
    fflush(stdout);
    while(1);
    return 0;
}
```

> gcc --static -g -o fakeinit main.c
> echo fakeinit | cpio -o --format=newc > rootfs

# vscode
```json
{
	"folders": [
		{
			"path": "."
		}
	],
	"settings": {},
	"tasks": {
		"version": "2.0.0",
		"tasks": [
            {
                "label": "vm",
                "type": "shell",
                "command": "qemu-system-x86_64 -no-reboot -nographic -kernel ${workspaceFolder}/arch/x86/boot/bzImage -initrd ${workspaceFolder}/rootfs -append \"console=ttyS0 spectre_v2=off l1tf=off quiet kernel.panic=-1 tsc=unstable root=/dev/ram rdinit=${workspaceFolder}/fakeinit\" -smp 2 -s -S",
                "presentation": {
                    "echo": true,
                    "clear": true,
                    "group": "vm"
                },
                "isBackground": true,
                "problemMatcher": [
                    {
                    "pattern": [
                        {
                        "regexp": ".",
                        "file": 1,
                        "location": 2,
                        "message": 3
                        }
                    ],
                    "background": {
                        "activeOnStart": true,
                        "beginsPattern": ".",
                        "endsPattern": "."
                    }
                    }
                ]
			},
			{
                "label": "build",
                "type": "shell",
                "command": "make",
                "group": {
                    "kind": "build",
                    "isDefault": true
                },
                "presentation": {
                    "echo": false,
                    "group": "build"
                }
			}
		]
	},
	"launch": {
		"configurations": [
			{
                "name": "(gdb) linux",
                "type": "cppdbg",
                "request": "launch",
                // "preLaunchTask": "vm",
                "program": "${workspaceRoot}/vmlinux",
                "miDebuggerServerAddress": "localhost:1234",
                "args": [],
                // "stopAtEntry": true,
                "cwd": "${workspaceFolder}",
                "environment": [],
                "externalConsole": false,
                "MIMode": "gdb",
                "miDebuggerArgs": "-n",
                "targetArchitecture": "x64",
                "setupCommands": [
                    {
                        "text": "set arch i386:x86-64:intel",
                        "ignoreFailures": false
                    },
                    {
                        "text": "dir .",
                        "ignoreFailures": false
                    },
                    {
                        "text": "add-auto-load-safe-path ./",
                        "ignoreFailures": false
                    },
                    {
                        "text": "-enable-pretty-printing",
                        "ignoreFailures": true
                    }
                ]
			}
		]
	},
}
```