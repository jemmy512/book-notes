# Network Config:
    ifconfig
    sudo ifconfig eth0 192.168.1.212 netmask 255.255.255.0

## 1 Million Connenction Config
```
# 1. port config
    echo "5000 65000" > /proc/sys/net/ipv4/ip_local_port_range

# 2. Config multiple ip addresses

# 3. max opened file
    echo 200000 > /proc/sys/fs/file-max

# vi /etc/sysctl.conf
    fs.nr_open=210000
# sysctl -p

# vi /etc/security/limits.conf
    soft  nofile  200000
    hard  nofile  200000
```

# FTP service：
    check service: lsof -i :21
    Install service: apt-get install vsftpd

# SSH service：
    check service：lsof -i :22
    Install service: apt-get install openssh-server
Allow root login:
    vi /etc/ssh/sshd_config -> PermitRootLogin yes

* max_user_watches
    * /proc/sys/fs/inotify/max_user_watches
    * /etc/sysctl.conf
    * fs.inotify.max_user_watches=524288
    * sudo sysctl -p

* session timeout
    * /etc/ssh/sshd_config
    * ClientAliveInterval  1200
    * ClientAliveCountMax 3
    * sudo systemctl reload sshd

# Statistic port traffic:
    iptables -A OUTPUT -p tcp --sport 8512  // add input port need statistic
    iptables -A INPUT -p tcp --dport 8512   // add output port need statistic
    iptables -L -v -n -x                    // view statistic traffic
    Iptable -Z INPUT/OUTPUT                 // reset all statistic port
    iptables -D INPUT -p tcp --dport 8080   // remove statistic port

# lsof: (list open file)
    lsof -u root    // list open file by user
    lsof -i TCP:22  // find process running on specific port
    lsof -i TCP:1-1024  // list open files of port ranges 1 - 1024
    lsof -i 4 <6>   // list only IPV4 or IPV6 open files
    lsof -i -u root // find out who's looking what files and commands
    lsof -p 878     // search by pid
    kill -9 'lsof -t -u <user>' // kill all activity of particular user
    lsof <filename> // find out all processes open specific file
    lsof -d /DIR/   // list files opened by process under /DIR/]
    lsof -d FD      //

# iptables:
    -t   <table>
    -A： append rule to chain
    -D： delete rule from chain
    -i： insert rule to chain
    -R： replace rule in chain
    -X   delete user defined chain
    -L： list runles in chain
    -F： remove all rules existing
    -Z： zero counter in chain
    -N： created user fined chain
    -P： define default target in chain
    -p： protocal
    -s:  sourc ip
    -j:  jump ip
    -i  <interface>:    interface which data in
    -o  <interface>:    interface which data out

    iptables -t "tableNmae" <-A/I/D/R> "ruleChainName" [ruleNum] <-i/o "interface"> -p "protocal"
    <-s "sourceIp"/ "sourcr child net"> --sport "sourcePort" <-d "dest IP"/"dest chid net"> --dport "dest port" -j "action"

    Table: raw, mangle, filter, nat

    RuleChain:
        INPUT, OUTPUT, FORWARD, PREROUTING, POSTROUTING

    Action:
        accetp, drop, redirect, snat, dnat, masquerade, log

    Practice:
        iptables -L -n -v   // list existing rules
        iptables -L -n --line-numbers   // list all rules by number
        empty rule:
            iptables -F
            iptables -X
            iptables -Z
        iptables -A INPUT -p tcp --dport 9000 -j DROP   // deny access of port 9000
        iptables -I INPUT -s 192.168.31.31 -j DROP      // deny access of host ip is 192.168.31.31
        iptables -A FORWARD -i eth0 -o eth1 -j ACCEPT   // interface forward
        iptables -t nat -A PREROUTING -p tcp -d 192.168.102.37 --dport 422 -j DNAT --to 192.168.102.37:22   // port forward
        iptables -A INPUT -p tcp --dport 80 -m limit --limit 25/minute --limit-burst 100 -j ACCEPT          // DDos

# ifconfig:
    ifconfig eth0 up/down   // up or down a interface
    ifconfig eth0 mtu 1500
    ifconfig eth0 hw ether 00:AA:BB:CC:dd:EE
    ifconfig eth0 172.172.21.50 netmask 255.255.255.0 broadcast 172.172.21.255

# tcpdump:
    -A display package in ASCII not display header
    -n display host by IP not host name, display port by number not service name
    -i interface to listen '-i any': listen all interface
    -v echo more detial information, eg: show TTL and TOS information in IP
    -t not echo time stamp
    -e display ethernet frame header information
    -c only get specified number of data packages
    -x diplay package in hexadecimal, not display header information
    -X same to -x but also display each hexadecimal byte's ASCII character
    -XX same to -X but also display packege head information
    -s capture length when capturing packege
    -S display absolute value of TCP sequence nuber
    -w redirect tcpdump output to a file in a special format
    -r read and display from file

    <type: host, net, port, portrange> <dir> <proto>
    tcpdump net 1.2.3.0/24
    tcpdump dst port 13459
    tcpdump icmp
    tcpdump ip host ernest-laptop && ! Kongming20
    tcpdump 'src 10.0.2.4 && (dst port 3389 || 22)'
    tcpdump 'tcp[13] & 2 != 0' // filer package
    tcpdump 'tcp[tcpflags' & tcp-syn != 0'

# Linux modulation, debuggin and testing
### Max Fd:
    ulimit -n // view user-level max fd
    ulimit -SHn max-file-number // set user-level max fd to max-file-number, only valid in current session
    /etc/security/limits.conf:  // set user-level max fd to max-file-number, permanently
        hard nofile max-file-number
        soft nofile max-file-number
    sysctl -w fs.file-max=max-file-number // set system-level max fd, only valid in current session
    /etc/sysctl.conf:   set system-level max fd, permanent
        1. add: fs.file-max=max-file-number
        2. run cmd: sysctl -p
### /proc/sys/fs:
```
/proc/sys/fs/file-max // system fd limitation, /proc/sys/fs/inode-max is its 3~4 times the size
/proc/sys/fs/epoll/max_user_watches // The total number of epoll events that a user can register
```
### /proc/sys/net:
```
/proc/sys/net/core/somaxconn // The max number of socks which in listen queue can transfer to ESTANBLISHED status
                                or SYN_RCVD status
/proc/sys/net/ipv4/tcp_wmem // A socket TCP write buffer's minmum, default and maxmum size
/proc/sys/net/ipv4/tcp_rmem // A socket TCP reade buffer's minmum, default adn maxmum size
/proc/sys/net/ipv4/syncookies
```

# Include path

* Include Path
    ```shell
    # C
    export C_INCLUDE_PATH=XXXX:$C_INCLUDE_PATH
    # CPP
    export CPLUS_INCLUDE_PATH=XXX:$CPLUS_INCLUDE_PATH

    # /etc/profile
    # /home/{user}/.bashrc/.bash_profile
    ```

* Link Path
    ```shell
    # .so
    export LD_LIBRARY_PATH=XXX:$LD_LIBRARY_PATH
    # .lib
    export LIBRARY_PATH=XXX:$LIBRARY_PATH
    ```

# GCC 7.2.0 Install
1. downlaod src code:
    wget https://ftp.gnu.org/gnu/gcc/gcc-7.2.0/gcc-7.2.0.tar.gz
2. install prerequsities:
    yum install libmpc-devel mpfr-devel gmp-devel zlib-devel*
    yum groupinstall "Development Tools"
3. extract src code:
     tar -xzvf gcc-7.2.0.tar.gz
4. cd gcc-7.2.0
5. ./contrib/download_prerequisites
6. cd ..
7. mkdir objdir
8. cd objdir
9. $PWD/../gcc-7.2.0/configure --prefix=$HOME/gcc-7.2.0 --disable-multilib --enable-languages=c,c++
    ../gcc-7.2.0/configure --prefix=/usr/local/gcc7 --enable-languages=c,c++,go  --disable-multilib
    ./configure –enable-checking=release –enable-languages=c,c++ –disable-multilib
10. make -j n // n is amount number of processor core
11. make install

# Linux File and Management
    ls:
        ls [-aAdfFhilnrRSt]
        ls [--color={never,auto,always}]
        ls [--full-time]
    cp:
        cp [-adfilprsu] sourc destination
        cp [options] source1 source2 source3 .... directory # last file must be a directory
    mv:
        mv [-fiu] source destination
        mv [options] source1 source2 source3 .... directory
    rm: rm [-fir]
    basename  dirname
    cat [-AbEnTv]
    tail -n num filename
    more:
        <space>                 Display next k lines of text [current screen size]
        z                       Display next k lines of text [current screen size]*
        <return>                Display next k lines of text [1]*
        d or ctrl-D             Scroll k lines [current scroll size, initially 11]*
        q or Q or <interrupt>   Exit from more
        s                       Skip forward k lines of text [1]
        f                       Skip forward k screenfuls of text [1]
        b or ctrl-B             Skip backwards k screenfuls of text [1]
        '                       Go to place where previous search started
        =                       Display current line number
        /<regular expression>   Search for kth occurrence of regular expression [1]
        n                       Search for kth occurrence of last r.e [1]
        !<cmd> or :!<cmd>       Execute <cmd> in a subshell
        v                       Start up /usr/bin/vi at current line
        ctrl-L                  Redraw screen
        :n                      Go to kth next file [1]
        :p                      Go to kth previous file [1]
        :f                      Display current file name and line number
        .                       Repeat previous command
    touch: # change time of file: modification time(mtime), access time(atime), status time(ctime)
        touch [-acdmt] file

    which [-a] name
        # search instruction by name
        # which: search file according to $PATH variable
    whereis [-lbmsu] name
        # search file by name
        # whereis: search file in specific pathes. /bin/sbin/  /usr/share/man
    locate/updatedb [-iclSr] keyword
        # updatedb: search file according to /etc/updatedb.conf and update /var/lib/mlocation
        # locate: search file according to /var/lib/mlocate
        # find: search file within whole disk

    tar: # A(catenate), c(create), d(diff, delete), r(append), t(list), u(update) x(extract)
        tar -cjv -f ***.tar.bz2 srcFile # -z(.tar.gz), -j(tar.bz2), -J(tar.xz)
        tar -tjv -f ***.tar.bz srcFile
        tar -xjv -f ***.tar.bz2 -C srcFile
        tar -czvf combined.tar.gz t1.tar.gz t2.tar.gz

# Linux File System
    Superblock:
        the total amount of block and inode
        the number of inode/block that are not uesed and used
        size of block and inode
        system mount time, the last write time, check time(fsck) and other system information
        valid bit: 0(system has been mounted), 1(system is not mounted)
    inode table:
    inode:
        read and wirte mode(read/write/excute)
        file's owner and group(owner/group)
        file capacity
        ctime, atime, mtime
        flag(SetUID)
        pointer
    block bitmap:
    inode bitmap:

    XFS File System:
        data section: inode/data block/superblock
        log section:
        realtime section:

        df [-ahikHTm] name # list usage of file system
        du [-ahskm] name # evaluate the disk usage of file system
        xfs_info /dev/vda1

    Hard, Symbolic Link:
        ln [-sf] source destination

    Partition:
        lsblk [-dfimpt] [device] # list all disk lists on system
        blkid # list device UUID
        parted device_name print # list disk partition table type and partition info

        Wizard to part the bisk:
            1. use lsblk to find disk
            2. use pared /dev/xxx print get the partition table type
            3. use appropriate method (gdisk/fdisk) to part disk

# Process
    ps -[a,A,N,e,H,au,aux,line<num>,width<num>]
    ps [a,c,e,f,r,T,u]

    grep [-acinrv] [--color=auto] 'search characters' filename

# VIM
    command mode:
    Cursor:
        ctrl + f # page down
        ctrl + b # page up
        0 and $ # to and curretn line head and real
        H # to the most left top head of the current screen
        M # to the middle line head of the current screen
        L # to the bottom line head of the current screen
        G # to the bottom of the file
        nG # to the n line of the file
        gg (1G) # to the first line of the file
        n<Enter> # down n line
    Search:
        /word `# down search
        ?word `# up search
        n `# key, repeat previous search, down
        N `# key, repeat previous search, up
        : n1, n2s/word1/word2/g `# search word1 and replace it with word2 between n1 and n2
        :1, $s/word1/word2/g `# search word and replace between first line and last one
        :1, $s/word1/word2/gc `# replace need c(confirm)
    delete, copy and past:
        x,X # backwords and forward delete
        nx # delete n characters backwords
        dd # delete current line
        d<n>G # delete line from curretn to <n> line
        dG # delete line from current to last
        d$, d0 # delete from cursor to last or first character of current line

        yy # copy current line
        nyy # copy n line down from current line
        y1G # copy from current line to first one
        yG # copy from current line to the last one
        y0 # copy from cursor to first character of this line
        y$ # copy from cursor to last charactter of this line

        p,P # past the data to the previous/next line
        J # combine the this lint with next line into one line
        c # repeat delete mutil data eg: 10cj
        u # undo previous operation
        ctrl + r # redo the previous operation
        . # repeat previous action
    insert mode:
        ZZ # save if changes and quit
        :w[fileName] # save file with name
        :r[fileName] # read other file's date and append to cursor of this file
        :!command #
        :n1,n2 w [file] # save data from n1 to n2 to file
    Additional operation:
        visual Block:
            v # character selection, will pass the cursor through the place to choose
            V # line selection
            ctrl + v # visual block selection, can select column
            y # copy the visual block data
            d # delete the vidua block data
            p # append visual block data to the cursor
        multi file Edtion:
            :n # edit next file
            :N # edit previous file
            :file # show all files opened by vim
        multi windows:
            :sp [filename] #
            ctr + W + #
            ctr + w + q
        Automatic comletion:
            ctrl + x -> ctrl + n # completion based on content of this file
            ctrl + x -> ctrl + f # completion based on the file name of the file within this directory
            ctrl + x -> ctrl + o # completion based on the syntax of file suffix name

        command line mode:

        Enviroment Setting:
            /etc/vimrc		~/.vimrc(no exsit by default)
            :set all
            :set # show the different setings with the default of system
            :set ruler # status instraction in right bottom corner
            :set backspace=(012)
            :set bg=dark/light # show different color tones


# Bash
    history: ~/.bash_historys
    type [tpa] name # check weather the instruction is built in or not
    ctr + u/k # delete all input before/after cursor
    ctr +a/e # move cursor to line head/rear
    ctr + l # clean current screen
    set variable:
        set name, unset name, export # make local variable to be global variable
    enviroment variable:
        env #



# Linux File System

* Hard Link
    * Have same inode and data block
    * Only can be created for existed files
    * Can not cross file systems
    * Only can be ceated for files not directories
    * Delete a hard link doesn't affect other files with same inode

* Sysmbol Link
    * Has its owen file profile and cred
    * Can be created for non-exited files
    * Can cross files systems
    * Can be creatd for both files and directories
    * Doesn't increment i_nlink
    * becomes danging link if orginal file is deleted

* crontab:
    /etc/cron.deny  # user in this file don't allow use crontab
    /etc/cron.allow
    /var/spool/cron/  # all user's crontab file reserved here and named with user name

    /sbin/service cron [start, stop, restart, reload, status]
    /etc/init.d/cron [start, stop, restart, reload, status]

* File System:
    chgrp [-R] dirname/filename...
    chown [-R] owner:group dir/file
    chmod [-R] xyz dir/file
    chmod | u g o a | + - = | r w x | dir or file |

    mkdir[-mp] # m:
    rmdir [-P] # P, remove current directory and it's parent and empty directory
    rm [-r] # delete all file include it's children
    pwd [-P] # show real path(linked file)


*  rpm:
        QUERYING AND VERIFYING PACKAGES:
            rpm {-q|--query} [select-options] [query-options]
            rpm {-V|--verify} [select-options] [verify-options]
            rpm --import PUBKEY ...
            rpm {-K|--checksig} [--nosignature] [--nodigest] PACKAGE_FILE ...
        INSTALLING, UPGRADING, AND REMOVING PACKAGES:
            rpm {-i|--install} [install-options] PACKAGE_FILE ...
            rpm {-U|--upgrade} [install-options] PACKAGE_FILE ...
            rpm {-F|--freshen} [install-options] PACKAGE_FILE ...
            rpm {-e|--erase} [--allmatches] [--nodeps] [--noscripts]
            [--notriggers] [--repackage] [--test] PACKAGE_NAME ...
        MISCELLANEOUS:
            rpm {--initdb|--rebuilddb}
            rpm {--addsign|--resign} PACKAGE_FILE ...
            rpm {--querytags|--showrc}
            rpm {--setperms|--setugids} PACKAGE_NAME ...

        select-options

        [PACKAGE_NAME] [-a,--all] [-f,--file FILE]
        [-g,--group GROUP] {-p,--package PACKAGE_FILE]
        [--fileid MD5] [--hdrid SHA1] [--pkgid MD5] [--tid TID]
        [--querybynumber HDRNUM] [--triggeredby PACKAGE_NAME]
        [--whatprovides CAPABILITY] [--whatrequires CAPABILITY]

        query-options
        [--changelog] [-c,--configfiles] [-d,--docfiles] [--dump]
        [--filesbypkg] [-i,--info] [--last] [-l,--list]
        [--provides] [--qf,--queryformat QUERYFMT]
        [-R,--requires] [--scripts] [-s,--state]
        [--triggers,--triggerscripts]

        verify-options
        [--nodeps] [--nofiles] [--noscripts]
        [--nodigest] [--nosignature]
        [--nolinkto] [--nomd5] [--nosize] [--nouser]
        [--nogroup] [--nomtime] [--nomode] [--nordev]

        install-options
        [--aid] [--allfiles] [--badreloc] [--excludepath OLDPATH]
        [--excludedocs] [--force] [-h,--hash]
        [--ignoresize] [--ignorearch] [--ignoreos]
        [--includedocs] [--justdb] [--nodeps]
        [--nodigest] [--nosignature] [--nosuggest]
        [--noorder] [--noscripts] [--notriggers]
        [--oldpackage] [--percent] [--prefix NEWPATH]
        [--relocate OLDPATH=NEWPATH]
        [--repackage] [--replacefiles] [--replacepkgs]
        [--test]


# Linux Performance Tunning
### Ipv4
Phase | Tuning Parameter
--- | ---
SYN_SEND | /proc/sys/net/ipv4/tcp_syn_retries
SYN_RECV queue | tcp_max_syn_backlog, /proc/sys/net/core/somaxconn, backlog, tcp_syncookies
accept queue | min(somaconn, backlog), tcp_abort_on_overflow
SYN_RECV | tcp_synack_retries
FIN_WAIT1, CLOSE_WAIT | tcp_orphan_retries, tcp_max_orphans
FIN_WAIT2 | tcp_fin_timeout
TIME_WAIT | tcp_max_tw_buckets, tcp_tw_reuse+tcp_timestamps

### Network Data Transfer
Item | Tunning Parameter
--- | ---
sending window | tcp_window_scaling
sending/receive buffer | tcp_{w, r}mem
dynamic tunning sending/receve buffer | tcp_moderate_{rcv, w}buf, SO_{SND, WRT}BUF will close dynamic tunning
Tcp memory | tcp_mem(unit: page)

#### SYN Flood
1. tcp_syncookies
2. tcp_synack_retries + tcp_max_syn_backlog + tcp_abort_on_overflow

### TIME_WAIT
1. tcp_tw_reuse + tcp_timestamps
2. tcp_tw_recycle
3. tcp_max_tw_buckets

# swap

[How To Add Swap Space on Ubuntu 22.04](https://www.digitalocean.com/community/tutorials/how-to-add-swap-space-on-ubuntu-22-04#step-1-checking-the-system-for-swap-information)

1. Step 1 – Checking the System for Swap Information

    > sudo swapon --show

    > free -h

    ```text
    Output
                total        used        free      shared  buff/cache   available
    Mem:          981Mi       122Mi       647Mi       0.0Ki       211Mi       714Mi
    Swap:            0B          0B          0B
    ```

2. Step 2 – Checking Available Space on the Hard Drive Partition

    > df -h

    ```text
    Output
    Filesystem      Size  Used Avail Use% Mounted on
    udev            474M     0  474M   0% /dev
    tmpfs            99M  932K   98M   1% /run
    /dev/vda1        25G  1.4G   23G   7% /
    tmpfs           491M     0  491M   0% /dev/shm
    tmpfs           5.0M     0  5.0M   0% /run/lock
    tmpfs           491M     0  491M   0% /sys/fs/cgroup
    /dev/vda15      105M  3.9M  101M   4% /boot/efi
    /dev/loop0       55M   55M     0 100% /snap/core18/1705
    /dev/loop1       69M   69M     0 100% /snap/lxd/14804
    /dev/loop2       28M   28M     0 100% /snap/snapd/7264
    tmpfs            99M     0   99M   0% /run/user/1000
    ```

3. Step 3 – Creating a Swap File

    > sudo fallocate -l 1G /swapfile
    > ls -lh /swapfile

4. Step 4 – Enabling the Swap File

    > sudo chmod 600 /swapfile
    > ls -lh /swapfile
    > sudo mkswap /swapfile

    ```text
    Output
    Setting up swapspace version 1, size = 1024 MiB (1073737728 bytes)
    no label, UUID=6e965805-2ab9-450f-aed6-577e74089dbf
    ```

    > sudo swapon /swapfile
    > sudo swapon --show

    ```text
    Output
    NAME      TYPE  SIZE USED PRIO
    /swapfile file 1024M   0B   -2
    ```

    > free -h

    ```text
    Output
                total        used        free      shared  buff/cache   available
    Mem:          981Mi       123Mi       644Mi       0.0Ki       213Mi       714Mi
    Swap:         1.0Gi          0B       1.0Gi
    ```

5. Step 5 – Making the Swap File Permanent

    Back up the /etc/fstab file in case anything goes wrong:
    > sudo cp /etc/fstab /etc/fstab.bak

    Add the swap file information to the end of your /etc/fstab file by typing:
    > echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab

6. Step 6 – Tuning your Swap Settings

    1. The **swappiness** parameter configures how often your system swaps data out of RAM to the swap space. This is a value between 0 and 100 that represents a percentage.

        We can see the current swappiness value by typing:
        > cat /proc/sys/vm/swappiness

        At the bottom of `/etc/sysctl.conf`, you can add:
        > vm.swappiness=10

    2. Adjusting the **Cache Pressure** Setting

        At the bottom of `/etc/sysctl.conf`, add the line that specifies your new value:
        > vm.vfs_cache_pressure=50