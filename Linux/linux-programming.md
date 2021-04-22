
# Make File
1. syntax
    target: prerequisites...
        command # command line must must start with tab
    ...

2. file name rules: Makefile / makefile / GNUmakefile

3. Five parts of makefile:
    1. explicit rules
    2. implicit rules
    3. comments
    4. variable defination
    5. file instruction:
        * include other makefile
        * specify valid parts of makefile according to cases
        * define a mutil-line command
4. wildcards: *, ? [...]
    object = *.o # value of variable object is '*.o' not all .o suffix files
    objects := $(wildcard *.o) # value of variable object is all .o suffix files

5. File Search:
    1. makefile special variable:
        VPATH = <path1> : <path2> : <path3> ...
    2. makefile vpath keyword
        vapth <pattern> <directories> # vpath %.h ../header
        vpath <path>    # clear search directories for files that match pattern <pattern>
        vpath           # clear all file search directories that have been set up
6. Mutil Target:
    bigoutput littleoutput: text.gcc
        generate text.g  -$(func args,,$@)>$@ # $:function, $@: sets of target
7. Static Mode:
    <target...> : <target-pattern> : <prereq-pattern>
        <command>
    eg:
    objects = foo.o bar.o
    all: $(objects)
    $(objects): %.o: %.c
           $(CC) -c $(CFLAGS) $< -o $@ # $<: first file of dependences, $^: all dependences, $@: sets of targets
    `#` $?: all dependences that newer than target file, $*: represent '%' in the target pasttern and its predecessor
8. Automatically generate dependencies
    %.d: %.c
        @set -e; rm -f $@; \
        $(CC) -M $(CPPFLAGS) $< > $@.  //-MM include std head files, -M just include dependent head files
    ; \
                sed 's,$∗\.o[ :]*,\1.o $@ : ,g' < $@.
        > $@; \
                rm -f $@.

9. ; # if two next instruction dependent on previous instruction, thems should write in one line, sperate by ;
    exec:
        cd /home/hchen; pwd # result is: pwd = /home/hchen
10. clean
    .PHONE clean
    clean:
        rm -i -f -k # -i(-ignore-errors) -k(-keep-going) -f(-force)
11. Nested Make
    subsystem:
           cd subdir && $(MAKE) # execute makefile in sudir directory may be with arguments $(MAKE)
    (un)export<variable ...>  # pass variables in current makefile to child makefile
                              # SHELL and MAKEFLAGS varialbles always pass to child makefile
12. define instruction pakage
    define  pakageName
        command...
        command...
    enddef
13. Variable
    13.1 define
        x = $(y) # current variable can use next defined varaible
        y = z

        x := foo # current variable can use only previous defined variables
        FOO ?= bar # if FOO is not defined its value is bar, otherwise is previous defined value
    13.2 value replacement
        foo := a.o b.o c.o
        bar := $(foo:.o=.c) # a.c b.c c.c

        foo := a.o b.o c.o
        bar := $(foo:%.o=%.c) # static mode

        first_second = Hello
        a = first
        b = second
        all = $($a_$b) # all = first_sencond
    13.3 append value to value
        objects = main.o foo.o bar.o utils.o
        objects += another.o
    13.4 override
        override <variable> = <value> # override variables defined  make instruction line
        override <variable> := <value>
        override <variable> = <value>
    13.5 multi-line variable
        define two-lines
        echo foo
        echo $(bar)
        endef

# GDB
 ```C++
__LINE__, __FILE__, __func__, __DATE__, __TIME__, __STDC__, __cplusplus__

#define DEBUG

#ifdef DEBUG
 #define DBG_PRINT(fmt, args...) do {printf("[*]%s: <%s>: "fmt"\n"\
    , __FILE__, __func__, ##args); } while (FALSE);

//DBG_PRINT(format,...) printf("FILE: "__FILE__", LINE: %d: "format"/n", __LINE__, ##__VA_ARGS__)
#else
 #define BEG_PRINT(...) do {} while (FALSE)
#endif
 ```

* core dump:
    1. suffix with pid
        * echo "1" > /proc/sys/kernel/core_uses_pid  # 0 without suffix, 1 with pid suffix
    2. name form and path
        * echo "/corefile/core-%e-%p-%t" > proc/sys/kernel/core_pattern
        * %e(name) %p(pid) %u(uid) %g(gid) %s(signal) %t(time) %h(host name)

* Gdb:
    ```C++
    file <program>
    kill <program>
    make source file to be debug:
        gcc -g -o ObjFileName sourceFileName
    open gdb:
        1. gdb <program>
        2. gdb <attach> <pid>
        3. gdb <program> core
    run program: r
    run program with args:
        r <arg1><arg2>...
        set args <arg1><arg2>...
        show args
        show path
    exit gdb: quit/q
    Show content of file: list/l
        set listsize <num>
        show listsize
        list <linenum>
        list <function>
        list <filename:linenum>
        list <filename:funciton>
        list <+offset>
        list <-offset>
        list <firstLine>, <lastLine>
        list , <last>
        list + <num>
        l 10
        l main
        l hello.c:20
        l hello.c:main
    show content in GUI: focus, layout
    BreakPoint: break/b
        b <linenum>
        b <function>
        b <funcion(type...)>
        b <filenname:linenum>
        b *<address> # memory breakpoint
        b if <conditon>
        b <linenum> thread <threadno> # if there is no threadno specified, bk will apply to all threads
        b <linenum> thread <threadno> if... # when one thread stop, all threads will be stopped by gdb
        ignore <bk no> <count>
    view breakpoint info: info breakpoint
    clean breakpoint: clear
    disable/enable breakpoint:
        disable/enables breakpoint bkNum
        delete <bk no><range>
    condition bk:
        condition <bk no> <expr>
        condition <bk no> # clear bk which num is no
    command: # when bk we can do other things
        command <bkno>
        ... commnad-list...
        end
    watch point:
        watch <expr>
        rwatch <expr> # stop program when expr is readed
        awatch <expr> # stop program when expr is read or writed
    catch point:
        catch <event> #event: throw, catch, exec, fork, vfork, load, unload
        tcatch <event> # one time catch point

    go: run/r
    Step into: step/s  <count>
    Step over: next/n  <count>
    run to next bk: continue/c <ignor-count>
    finish for loop: util/u <linenum>
    finish current function: finish/f
    set step-mode <on/off>

    step machine code:
        setpi/si
        nexti
        display/i $pc
    function call stack: bt <n> #backtrack
    switch current stack:
        frame <n>
        up <n>
        down <n>
        info frame
        info args
        info locals
        info catch #
        info program #check the satus of current program and pid, stop reason
        info threads
        info watchpoints
        info signals
        info handle
    Methods of stopping program:
        BreakPoint, WatchPoint, CatchPoint, Signals, Thread Stops
    Signals:
        handle <signal> <keywords...>
        #keywords: nostop, stop, print, noprint, pass noignore, nopass ignore
    memory: examine/x (x/n,f,u)
        x [Address expression]
        x /[Format] [Address expression]
        x /[Length][Format] [Address expression]

    view registers: info register (i r)
    view vriable:
        info local variableName
        info local file::variableName
        info local function::variable
        info registers variableName
        whatis <var> # show var type and value
        ptype <var> # show var type
    view: print/f
        x d u o t a c f

        set print pretty  = on/off
        $1 = {a = 100, b = 200, c = 300}

        print var = value # change the local variable's value
    display:
        display/<fmt> <varname>
        display/i $pc
        info display
        disalbe enable
    jump:
        jump <linenum>
        jump <address>
        set $pc = address
    call:
        call <funname>
    view array:
        print *<array>@<len>
        int a[10]-> p *a@10
    run shell in gdb: shell <command>
    ```

Search:
    forward-search <regexp>
    search <regexp>
    reverse-search <regexp>

source path:
        directory <dirname...> # :(unix), ;(window)
        directory # clear path info
        show directories

define own enviroment variable int GDB:
    set $<varname> = <value> # enviromet var have no type
    show convenience # view all enviroment variable

# File I/O

* By convention: <unistd.h>
    * 0, 1, 2(<STDIN, STDOUT, STDERR>_FILENO): stand for standard input, output and error, respectively.
    * Range: 0 - (OPEN_MAX - 1)

```C++
#include <fcntl.h>
int open(const char *path, int oflag, ... /*mode_t mode*/);
int openat(int fd, const char *path , int oflag, ... /*mode_t mode*/);
    // oflag: O_RDONLY, O_WRONLY, O_RDWR, O_EXEC, O_SEARCH, O_TTY_INIT, O_SYNC, O_DSYNC, O_RSYNC
    // O_APPEND, O_CLOEXEC, O_CREAT, O_DIRECTORY, O_EXCL, O_NOCTTY, O_NOFOLLOW, O_NOBLOCK,  O_TRUNC
int create(const char *path, mode_t mode); //
int close(int fd); <unistd.h> == [ open(path, O_WRONLY | O_CREATE | O_TRUNC, mode)]
off_t lseek(int fd, off_t offset, int whence);  // move the read/write offset
    // whence: SEEK_SET, SEEK_CUR, SEEK_END, SEEK_DATA, SEEK_HOLE
    // We should be careful to compare the return value from lseek as being equal to or not equals to -1,
        rather than testing whether it is less than 0. Because offset is possible a negative value.
off_t fseek(FILE *stream, long int offset, int whence);  <stdio.h>
off_t ftell(FILE *stream); <stdio.h> // return current offset
ssize_t read(int fd, void *buf, size_t nbytes); <unistd.h>
    // There are several cases actual bytes read less than amount requested:
    // 1. Reading from a regular file, end of the file is reached before requested number of bytes has been read.
    // 2. Reading from a terminal device. Normally, up to one line is read at a time.
    // 3. Reading from a network. Buffering with network may cause less than request amount  to be return.
    // 4. Reading from a fifo or PIPE. If fifo contain fewer bytes by requested, read return only what is available
    // 5. Reading from a record-oritend device.
    // 6. Interrupted by a signal and partial data have been read.
```

## FD Duplicate:
```C++
int dup(int fd);            // equal to fcntl(fd, F_DUPFD, 0);
    // Always return the minimum fd number of the system.
int dup2(int fd, int fd2);  // equal to close(fd2); fcntl(fd, F_DUPFD, fd2);
                            // fd2 shares the same file table as fd.
    // new fd created by dup or dup2 doesn't inherit the property of the old fd such as close-on-exec, non-blocking
```

## Data sync between kernel buffer and disk:
```C++
void sync(void);    // Queues all the modified block buffers for writing and returns.
int fsync(int fd);
int fdatasync(int fd);
```

## Aotomic Operation
```C++
#include <unistd.h>
ssize_t pread(int fd, void *buf, size_t nbytes, off_t offset);
ssize_t pwrite(int fd, const void *buf, size_t nbyte, off_t offset);
```

## sendfile
```C++
#include <sys/sendfile.h> // designed to transfer file over internet
int sendfile(int out_fd, int in_fd, off_t *offset, size_t count);
// out_fd can be any fd while in_fd must refer to real file, not socket or pipe.
// Operation is completed in kernel, so it's very efficient and is a 0 COPY.
```

## File property change
```C++
#include <fcntl.h>
int fcntl(int fd, int cmd, ... /*int arg*/);
    // cmd: F_DUPFD (the new fd shares the same file table entry, new has its own FL and FD_CLOEXEC is cleared)
    // F_DUPFD_CLOEXEC, F_{Get, SET}FD, F_{GET, SET}FL, F_GETOWN, F_GETLK, F_GETLK, F_SETLKW, F_RDLCK, F_UNLCK
    // F_GETFL{O_RDONLY, O_WRONLY, O_RDWR, O_EXEC, O_SEARCH} {O_APPEND, O_NONBLOCK, O_SYNC, O_DSYNC, O_RSYNC}

int posix_fadvise(int fd, off_t offset, off_t len, int advise); // predeclare an access pattern for file data
    // advise: POSIX_FADV_NORMAL, *_SEQUENTIAL, *_RANDOM, *_NOREUSE, *_WILLNEED, *_DONTNEED
int fallocate(int fd, int mode, off_t offset, off_t len);       // allocate file space, nonportable
int posix_fallocate(int fd, int mode, off_t offset, off_t len);
    // After a successful call, subsequent writes to the specified range guaranteed not to fail because of lock.
    Allocating disk space:      0, FALLOC_FL_KEEP_SIZE, *_UNSHARE
    Deallocating disk space:    FALLOC_FL_PUNCH_HOLE,
    Collapsing file space:      FALLOC_FL_COLLAPSE_RANGE
    Zeroing file spece:         FALLOC_FL_ZERO_RANGE(nonportable)
    Increasing file space:      FALLOC_FL_INSERT_RANGE
ioctl: <sys/ioctl.h>
    int ioctl(int fd, int request, ...);
```

## Parse Command Line Option
```C++
#include <unistd.h>
#include <getopt.h>
int getopt(int argc, char *const argvp[], const char *optstring); <unistd.h>
    // If an option followed by a colon (:) it takes an argument, otherwise it exits by itself.
    // If an option followed by two colons(::), it's free to take or not take argument, but there must be
        no any space between option and arguement. eg: "a:b::c" --> -a aa -bbb -c / -a aa -b -c
    // Encounters a invalid option, returs a question mark (?) instead of characters.
    // If an option's missing, returns a question mark(?); if first character of options is colon, return colon(:)
    // Pattern -- can stop getopt processing options and return -1
    // optopt, optarg, opterr, optind
int getopt_long(int argc, char * const argv[], const char *optstring,
        const struct option *longopts, int *longindex); <getopt.h> _GNU_SOURCE
    // longopts' last element must be a zero array {0, 0, 0, 0}
    struct option{
        const char  *name;      // long option name
        int         has_flag;   // 1--has, 0--hasn't
        int         *flag;      // specifies how results returned for long option
        int         val;        // return value; flag is NULL, return 0;
    };
```

## Syslog
```C++
#include <syslog.h>
void openlog(const char *ident, int logopt, int facility);
void syslog(int priority, const char *message, .../*arguments*/);
int  setlogmask(int maskpri);
void closelog(void);
// priority: LOG_EMERG, LOG_ALERT, LOG_CRIT, LOG_ERR, LOG_WARNING, LOG_NOTICE, LOG_INFO, LOG_DEBUG
// logopt: LOG_PID, LOG_CONS, LOG_ODELAY, LOG_NDELAY
// facility: LOG_AUTH, LOG_DEAMON, LOG_KERN, LOG_LPR, LOG_LOCAL0~LOG_LOCAL7, LOG_MAIN, LOG_NEWS, LOG_USER
```

# File and Directories

* TOCTTOU: time-of-check-to-time-of-use
    * This error in file system namespace generally deal with attempts to subvert file permission by tricking privileged
    * program into either reducing permissions on privileged file or modifying privileged file to open up a secure hole.

## Stata
```C++
#include <sys/stat.h>
int stat(const char *pathname, struct stat *buf); // get file stat
int fstat(int fd, struct stat *buf);
int lstat(const char *pathname, struct stat *buf);  // return symbolic infomation, not referenced file
int fstatat(int fd, consth char *pathname, struct stat *buf, int flag); // AT_SYMLINK_NOFOLLOW, AT_FDCWD
struct stat {
    dev_t       st_dev;     // Id of device containing file nane and i-node
    ino_t       st_ino;     // inode number
    mode_t      st_mode;    // file type and mode (permission)
    nlink_t     st_nlink;   // number of hard link
    uid_t       st_uid;     // user id
    gid_t       st_git;
    dev_t       st_rdev;    // Device Id (if special file: character and block special file)
    off_t       st_size;    // Total size, in bytes (files, directories, symbolic links)
    blksize_t   st_blksize; // block size for filesystem I/O
    blksize_t   st_blocks;  // Number of 512B blocks allocatd

    struct timespec st_atim;    // last access time
    struct timespec st_mtim;    // last modification time
    struct timespec st_ctim;    // last stat change  time
};
```
mode_t:
* mode: S_IFMT, S_IFSOCK, S_IFLNK, S_IFREG, S_IFBLK, S_IFDIR, S_IFCHR, S_IFIFO

* Macro to check file type:
    * S_ISERG(m); S_ISDIR(m); S_ISCHR(m); S_ISBLK(m); S_ISFIFO(m); S_IFLNK(M), S_ISSOCK(m)
    * S_TYPEISMQ(struct stat *) S_TYPEISSEM() S_TYPEISSHM()

* Access permission:
    * S_ISUID, S_ISGID, S_IRWXU, S_I{R, W, X}USR, S_I{R, W, X}GRP, S_I{R, W, X}OTH

##  File Type:
    1. Regular
    2. Directory
    3. Block special file
    4. Character special file
    5. FIFO
    6. Socket
    7. Symbolic link


## File Access Permission:
* st_mode mask:
    * S_ISUID, S_ISGID, S_IRWXU, S_I{R, W, X}USR, S_I{R, W, X}GRP, S_I{R, W, X}OTH

* Whenever we open any type of file by name, we must have execute permission on each directory.
* The read permission determines whether we can open an existing file for reading.
* We must have write permission for a file to specify the O_TRUNC flag in the open function
* We can only create/delete new file in a directory when we have write permission and execute permission
* Execute permission for a file must be on if we use exec function and file also has to be a regular file.

* Ownership of New Files and Directory:
    * The user ID of the file is set to the effective user ID of the process, group ID can be following rules:
        1. Can be the effective group ID of the process.
        2. Can be the group ID of the directory in which the file is being created.

* access and faccessat: <unistd.h> determine accessibility based on real user and group id
```C++
    int access(const chat *pathname, int mode);
    int faccessat(int fd, const chatr *pathname, int mode, int flag);
    // mode: F_OK, R_OK, W_OK, X_OK; flag: AR_EACCESS
```

## umask
```C++
#include <sys/stat.h>  // set and get file mode creation mask
mode_t umask(mode_t cmask);
// any bits that are on in the file mode creation mask are tured off in the file's mode
// return previous value of the file mode creation mask
```

## chmod, fchmod, fchmodat
```C++
#include <sys/stat.h>
int chmod(const char *pathname, mode_t mode);
int fchmod(int fd, mode_t mode);
int fchmodat(int fd, const char *pathname, mode_t mode, int flag);
    // flag: AT_SYMLINKS_NOFOLLOW doesn't follow symbolic links
// effective user ID of the process must be equal to owner of the file or have supersuser permissions
// S_ISUID, S_ISGID, S_ISVTX; S_IRWXU, S_IRUSR, S_IWUSR, S_IXUSR
// S_IRWXG, S_IRGRP, S_IWGRP, S_IXGRP; S_IRWXO, S_IROTH, S_IWOTH, S_IXOTH
// S_ISVTX(sticky bit): only owner or superuser can rename or delete permission.
//              Two permissions are automatically remvoed by chomd:
// 1. Only superuser can set sticky bit of a regular file.
// 2. If the group id of new file doesn't equal to either effective group id of the  process or one the process's
//    supplementary group IDs and if process doesn't have superuser privileges,set-group-id bit is removed.
```

## chown, fchown, fchownat, lchown
```C++
#include <unistd.h>
int chown(const char *pathname, uid_t owner, git_t group);
    // can't used to change ownership of symbolic link
int fchown(int fd, uit_t owner, git_t group);
int fchownat(int fd, const char *pathname, uit_t owner, git_t group, int flag); // AT_SYMLINK_NOFOLLOW
int lchown(const char *pathname, uit_t owner, git_t group);
// with _POSIX_CHOWN_RESTRICTED, you can only change the group ID of the files you own, superuser can change UID
```

## File size:
1. Regular file size 0 is allowed.
2. Directory size is usually a multiple of a number, such as 16 or 512.
3. Symbolic link size is the number of byts in filename. (Symbolic link don't contain normal c null byte)

* File Truncation:    <unistd.h> truncate a file to specified length
    ```C++
    int truncate(const char *pathname, off_t length);
    int ftruncate(int fd, off_t length);
    // if previous file size is less than length, file size will increases, a hole probably created
    ```

## link
* link, linkat, unlink, unlinkat, remove: <unistd.h>
    ```C++
    int link(const char *existingpath, const char *newpath);
    int linkat(const efd, const char *existingpath, int nfd, const char newpath, int flag); // AT_SYMLINK_NOFOLLOW
    int unlink(const char *pathname);
    int unlinkat(int fd, const char *pathname, int flag); // AT_REMOVEDIR
    int remove(const char *pathname);
    int symlink(const char *actualpath, const char *sympath);
    int symlinkat(const char *actualpath, int fd, const char *sympath);
    // actualpath and sympath need not reside in the same file system.
    ssize_t readlink(const char *pathname, char *buf, size_t bufsize);
    ssize_t readlinkat(int fd, const char *pathname, char *buf, size_t bufsize);
    // combine the action of the open, read and close, return the number of bytes placed into buf. Contents returned
    // in the buf are not null terminated.
    ```

* limitation of hard link:
    1. Hard links normally require that link and the file reside in the same file system.
    2. Only the superuser can create a hard link to a directory

## rename, renameat: <stdio.h>
```C++
int rename(const char *oldname, const char *name);
int renameat(int oldfd, const char *oldname, int newfd, const char *newname); // AT_FDCWD
```
1. If oldname specifies a file, if newname is exists, it can't refer to a directory, it is remvoed, and oldname renamed to newname
2. If oldname specifies a directory, newname must refer to a directory. If newname exits, it must be empty, then it is removed. newname can't contain a path prefix that names oldname, because oldname cant removed.
3. If either oldname or newname refers to a symbolic link, then the link itself is proceessed, not the file to which it resolves.
4. Can't rename dot or dot-dot.
5. As a special case, if oldname and newname refer to the same file, the function return success and without changing anything.

## File Time:
* st_atim, st_mtim(contents change time), st_ctim(i-node change time)

* Action changes i-node:

* change access permission, change uid, change number of links

* futimens, utimensat, utimes
    ```C++
    #include <sys/stat.h> // change the access time and modification time of a file
    int futimens(int fd, const struct timespec times[2]); // calendar times
    int utimensat(int fd, const char *path, const struct timespec times[2], int flag);
    // First element of times array contain the access time and second contains the modification time.
    //                  Timestamps can be specified in four forms:
    // 1. If times[2] Null pointer. The times are set to the current time.
    // 2. Pointer to an timespec structure array. If tv_nsec is UTIME_NOW, time set to current time, tv_sec ignored.
    // 3. Pointer to an timespec structure array. If tv_nsec is UTIME_OMIT, time unchange, tv_sec ignored.
    // 4. Pointer to an timespec structure array. If tv_nsec is not UTIME_OMIT|UTIME_NOW, timestamp is set to the
    //    value specified by the corresponding tv_sec and tv_nsec fields.
    ```

## mkdir, mkdirat, rmdir
```C++
#include <sys/stat.h>
int mkdir(const char *pathname, mode_t mode);
int mkdirat(int fd, const char *pathname, mode_t mode);
int rmdir(const char *pathname);<unistd.h>
```

## Directories
```C++
#include <dirent.h>
struct dirent { // incomplete type
    inot_t  d_ino;          // file serial number;
    char    d_name[];       // filename string of entry
    ...
};
DIR *opendir(const char *pathname);
DIR *fdopendir(int fd);
struct dirent *readdir(DIR *dp);
void rewinddir(DIR *dp);    // reset the position of a directory stream to the begining of a directory.
int closedir(DIR *dp);
long telldir(DIR *dp);      // current location of a named directory stream
void seekdir(DIR *dp, long loc);    // set the position of a directory stream
// Only kernel has permission to write to directory itself.
// Ordering of entries within the directory is implementation dependent and usually not alphabetical.
```

## chdir, fchdir, getcwd
```C++
#include <unistd.h>
int chdir(const char *pathname);
int chroot(const char *pathname);   // change process root directory
int fchdir(int fd);
char *getcwd(char *buf, size_t size);
char *getwd(char *buf);
char *get_current_dir_name(void);
```

## File Tree walk
```C++
#include <ftw.h>
int nftw(
    const char *dirpath,
    int (*fn)(const char *fpath, const struct stat *sb, int typeflag, struct FTW *ftwbuf),
    int nopenfd, int flags
);
//typeflag: FTW_F, FTW_D, FTW_DNR, FTW_DP, FTW_NS, FTW_SL, FTW_SLN
//flags: FTW_CHDIR, FTW_DEPTH, FTW_MOUNT, FTW_PHYS,
FTW_ACTIONRETVAL:  // Enable this feature _GNU_SOURCE must be defind before any including header files
    fn() return following value: FTW_CONTINUE, FTW_SKIP_SIBLINGS, FTW_SKIP_SUBTREE, FTW_STOP

struct FTW {
    int base;   // offset of the filename in the pathname given in fpath
    int level;  // depth of the fpath in the directory tree, relative to the root of the tree(dirpath, depth 0)
};

ssize_t pwrite(int fildes, const void *buf, size_t nbyte, off_t offset);;
ssize_t write(int fildes, const void *buf, size_t nbyte);
```

## rlimit
```C++
#include<sys/resource.h>
struct rlimit {
    rlim_t rlim_cur;  /* Soft limit */
    rlim_t rlim_max;  /* Hard limit (ceiling for rlim_cur) */
};
setrlimit(int resource, const struct rlimit *rlp);
getrlimit(int resourcr struct rlimit *rlp);
    RLIMIT_CORE, RLIMIT_CPU, RLIMIT_DATA, RLIMIT_FSIZE, RLIMIT_NOFILE, RLIMIT_STACK, RLIMIT_AS, RLIM_INFINITY
getrusage(int who, struct rusage *r_usgage); // get information about resource utilization
    // who: RUSAGE_SELF, RUSAGE_CHILDREN
```

# Standard I/O Library
* Standard input, output, error:
    * File descriptor: STDIN_FILENO, STDOUT_FILENO, STDERR_FILENO
    * File pointer: stdin, stdout, stderr

## Buffering
```C++
#include <stdio.h>
// Full buffered. (File in disk) Actual I/O takes place when buffer is filled.
// Line bufferd. (Terminal) Actual I/O takes place when newline character encountered on input and output.
// unbuffered. (error msg)

void setbuf(FILE *fp, char *buf); //  BUFSIZE; turn buffering on or off
int setvbuf(FILE *fp, char *buf, int mode, size_t size); // mode: _IOFBF, _IOLBF, _IONBF
int fflush(FILE *fp);
```

## Opening a Stream:
```C++
#include <stdio.h>
#include <wchar.h>
int fwide(FILE *fd, int mode); // set a stream's oritention
    // Not change the orientation of stream that has already oriented.
    // If mode is negtive, make stream byte oriented, return negtive;
    // If mode is positive, make stream wide oriented, return positive;
    // If mode is zero, return a value specifing the stream's oriention

FILE *fopen(const char *pathname, const char *type);
FILE *freopen(const char *pathname, const char *type, FILE *fp);
FILE *fdopen(int fd, const char *type); // type: r, w, a, b, +
int fclose(FILE *fp);
```

## Reading and Writing a Stream:
1. Character-at-a-time I/O:
    ```C++
    int getc(FILE *fp); // can be implementated as a macro
    int fgetc(FILE *fp);
    int getchar(void);
    int ungetc(int c, FILE *fp); // push back charactes in reverse order.

    int putc(int c, FILE *fp);
    int fputc(int c, FILE *fp);     // return c if ok, EOF on error
    int putchar(int c);

    int ferror(FILE *fp);
    int feof(FILE *fp);     // return nonzero if condition is true, 0 otherwise
    void clearerr(FILE *fp);
    ```
2. Line-at-a-time I/O
    ```C++
    char *gets(char *buf);                  // reads from standard input
        // unsafe, may cause data overwritten.
    char *fgets(char *buf, int n, FILE *fp); // buf if ok, NULL on error
        // Read through and include next newline, but no more than n - 1 characters
        // If line is longer than n - 1, only partial line is read, but buffer is always null terminated.
    int fputs(const char *str, FILE *fp);
        // writes the null terminated string to stream, but null null bytes and the end is not written.
        // This need not be line-at-a-time ouput. Last non-null character is newline is not required.
    int puts(const char *str); // unsage as gets(...);
    ```

## Binary I/O:
```C++
size_t fread(void *ptr, size_t size, size_t nobj, FILE *fp);    // return number of obj read/write
size_t fwrite(const void *ptr, size_t size, size_t nojb, FILE *fp);
```

## Position a Stream:
1. ftell, fseek: file's position can be stored in a long integer.
2. ftello, fseeko: replace the long integer with off_t data type. SUS
3. fgetpos, fsetpos: fpot_t, can be made as big as necessary to record a file's position. ISO C

```C++
long ftell(FILE *fp);
int fseek(FILE *fp, long offset, int whence); // SEEK_SET, SEEK_CUR, SEEK_END
void rewind(FILE *fp);

int fgetpos(FILE *fp, fpost_t *pos);
int fsetpos(FILE *fp, const fpot_t *post);
```

## Formatted I/O:
```C++
int printf(const char *format, ...);
int fprintf(FILE *fp, const char *format, ...);
int dprintf(int fd, const char *format, ...);
int sprintf(char *buf, const cahr *format, ...);
int snprintf(char *buf, size_t n, const char *format, ...);

int scanf(const char format, ...);
int fscanf(FILE *fp, const char *format, ...);
int sscanf(const char *buf, const char *format, ...);
```

## Memory Stream:
```C++
FILE *fmemopen(void *buf, size_t size, const char *type); // type: r, w, a, b, +
    // If buf is NULL, fmemopen allocates a buffer of size bytes, and will be freed when stream is closed.
```


# Advanced I/O
## I/O Moudles:
1. blocking I/O
2. nonblocing I/O
3. I/O multiplexing(select, poll, epoll)
4. signal driven I/O(SIGIO)
5. asynchronous I/O(aio_)

## FD Readable condition:
1. The size of socket buffer in kernel is not less than SO_RECLOWAT
2. The peer of socket close connection, read return 0
3. There is a new connection in the scoket
4. There are errors to be handled in the socket, getsockopt() can get and clear the errors

## FD Writeble condition:
1. The size of socket buffer in kernel is not big that SO_SNDLOWAT
2. The write of socket is closed, write to such socket will trigger SIGPIPE
3. Using non-blocking connect connect success or fail
4. There are errors to be handled in the socket, getsockopt() can get and clear the errors

## Nonblocking I/O:
* Two ways to specify nonblocking I/O for a given descriptor:
    1. Specify O_NONBLOCK when call open.
    2. Call fcntl to turn on O_NONBLOCK file status flag on a opened file descriptor.

## Record Locking:
```C++
int fcntl(int fd, int cmd, ... /*struct flock *flockptr*/);
    // cmd: F_SETLK, F_SETLKW(wait), F_GETLK
struct flock {
    short   l_type;     // F_RDLCK, F_WRLCK, F_UNLCK
    short   l_whence;   // SEEK_SET, SEEK_CUR, SEEK_END
    off_t   l_start;    // offset in bytes, relative to l_whence
    off_t   l_len;      // length, in bytes; 0 means lock to EOF
    pid_t   l_pid;      // returned with F_GETLK
};
// Lokcs can start and extend beyond the current end of file, but can't start extend before the beginning.
// When setting or releasing a lock on a file, the system combines or splits adjacent areas as required.
```

## Memory-Mapped I/O
```C++
// <sys/mman.h>
void *mmap(void *addr, size_t len, int prot, int flag, int fd, off_t off);
    // protection: PROT_READ, PROT_WRITE, PROT_EXEC, PROT_NONE
    // flag: MAP_FIXED, MAP_SHARED, MAP_PRIVATE, 0, MAP_ANONYMOUS, MAP_HUGETLB
    // If file size is 12 bytes, system page size is 512bytes, system will provides 512 for mapped region, but
    //    any modification out of 12 bytes are not reflected in the file.
    // A memory-mapped region is inherited by a child across a fork.
int mprotec(void *addr, size_t len, int prot); // change the premission on an existing mapping.
// When modify pages using MAP_SHARED, the changes aren't written back to the file immediately.
// When the changes are written back, they are written in units of pages. If modify only one bytes, entire
    page will be written.
int msync(void *addr, size_t len, int flags);   // MS_SYNC, MS_ASYNC, MS_INVALIDATE
int munmap(void *adrr, size_t len); // automatically unmapped when process terminates or call this function.
    // The call this function doesn't cause the contents of the mapped region to be written to the disk file.
    // It's determined by the kernel's virual memory algorithm sometime after we store into the region.
```

## I/O Multiplexing:
1. select: <sys/select.h> <sys/time.h>
    ```C++
    int select(int nfd, fd_set *readfds, fd_set *writefds, fd_set *errorfds, const struct timeval *timeout)
    // nfds is the value of max fd plus one
    // timeout and every fs_set needs clean up first and then init at each time select execute
    // timeout == NULL (wait forever), timeout == 0(return immediately)
    // Normal data and OOB data arrive case select return, but select in readable and exception state,
        respectively, we can use recv(fd, buf, bufsize, MSG_OOB) get OOB data.
    void FD_ZERO(fd_set *fdset);
    void FD_SET(int fd, fd_set *fdset);
    void FD_CLR(int fd, fd_set *fdset);
    void FD_ISSET(int fd, fd_set *fdset);
    struct timeval{
        long tv_sec;   //seconds
        long tv_usec;  //microseconds
    };
    ```

2. poll: <poll.h>
    ```C++
    int poll(struct pollfd *fds, unsigned int nfds, int timeout);
    // timeout == -1(wait forever), timeout == 0(no wait) ; milliseconds
    struct pollfd {
        int fd;
        short events;       // request event
        short revents;      // return event
    } ;
    Event: POLLIN, POLLRDNORM, POLLBAND, POLLPRI, POLLOUT, POLLWRNORM, POLLWRBAND, POLLMSGSIGPOLL, POLLER, POLLHUP, POLLNVAL
    ```

3. epoll: <sys/epoll.h>
    ```C++
    int epoll_create(int size);
    int epoll_ctl(int epfd, int op, int fd, struct epoll_event *event);
        // op: EPOLL_CTL_ADD, EPOLL_CTL_MOD, EPOLL_CTL_DEL
        // Event: EPOLLIN, EPOLLOUT, EPOLLPRI, EPOLLET, EPOLLERR, EPOLLHUP, EPOLLRDHUP, EPOLLONESHOT, EPOLLEXCLUSIVE
        struct epoll_event {
            uint32_t        events; /* Epoll events */
            epoll_data_t    data;   /* User data variable */
        };
        typedef union epoll_data {
            voit        *ptr;
            int         fd;
            uint32_t    u32;
            uint64_t    u64;
        } epoll_data_t;
    int epoll_wait(int epfd, struct epoll_event * events, int maxevents, int timeout);
    ```

* Most of the epoll critique is based on two fundamental design issues:
    1. Sometimes it is desirable to scale application by using multi-threading. This was not supported by early implementations of epoll and was fixed by `EPOLLONESHOT` and `EPOLLEXCLUSIVE` flags.
    2. Epoll registers the file description, the kernel data structure, not file descriptor, the userspace handler pointing to it.
        * Epoll is broken because it mistakes the "file descriptor" with the underlying kernel object (the "file description"). The issue shows up when relying on the close() semantics to clean up the epoll subscriptions.
        * You always must always explicitly call `epoll_ctl(EPOLL_CTL_DEL)` before calling close().

* The debate is heated because it's technically possible to avoid both pitfalls with careful defensive programming:
    * Avoid using epoll for load balancing across threads.
    * Avoid sharing epoll file descriptor across threads.
    * Avoid sharing epoll-registered file descriptors.
    * Avoid forking, and if you must: close all epoll-registered file descriptors before calling execve.
    * Explicitly deregister affected file descriptors from epoll set before calling dup/dup2/dup3 or close.

## Epoll Load balancing
1. Scaling out accept()
    * Solution 1: The best and the only scalable approach is to use recent Kernel 4.5+ and use `level-triggered` events with `EPOLLEXCLUSIVE` flag. This will ensure only one thread is woken for an event, avoid "thundering herd" issue and scale properly across multiple CPU's.
    * Solution 2: Without EPOLLEXCLUSIVE, similar behavior it can be emulated with `edge-triggered` and `EPOLLONESHOT`, at a cost of one extra epoll_ctl() syscall after each event.
    * Solution 3 (without epoll): Use `SO_REUSEPORT` and create multiple listen sockets sharing the same port number. This approach has problems though - when one of the file descriptors is closed, the sockets already waiting in the accept queue will be dropped. Read more in this [Yelp blog post](https://engineeringblog.yelp.com/2015/04/true-zero-downtime-haproxy-reloads.html) and this [LWN comment](https://lwn.net/Articles/542866/).
        * Kernel 4.5 introduced `SO_ATTACH_REUSEPORT_CBPF` and `SO_ATTACH_REUSEPORT_EBP`F socket options. When used properly, with a bit of magic, it should be possible to substitute `SO_INCOMING_CPU` and overcome the usual `SO_REUSEPORT` dropped connections on rebalancing problem.
2. Scaling out read()
    * The correct solution is to use `EPOLLONESHOT` and `re-arm` the file descriptor manually. This is the only way to guarantee that the data will be delivered to only one thread and avoid race conditions.
    * One thread one epoll.

[Epoll is fundamentally broken 1/2](https://idea.popcount.org/2017-02-20-epoll-is-fundamentally-broken-12)

[Epoll is fundamentally broken 2/2](https://idea.popcount.org/2017-03-20-epoll-is-fundamentally-broken-22)


## Asynchronous I/O:
```C++
struct aiocb {
    int             aio_fildes;     // file descriptor
    volatile void   *aio_buf;       // buffer for I/O
    int             aio_nbytes;     // number of bytes to transfer
    off_t           aio_offset;     // file offset for I/O
    int             aio_reqprio;    // request priority
    struct sigevent aio_sigevent;   // signal information
    int             aio_lio_opcode; // LIO_READ, LIO_WRITE, LIO_NOP
};

struct sigevent {
    int             sigev_notify;                   // notify type: SIGEV_NONE, SIGEV_SIGNAL, SIGEV_THREAD
    int             sigev_signo;                    // signal number
    union sigval    sigev_value;                    // notify argument
    void (*sigev_nofity_function)(union sigval);    // notify function
    pthread_attr_t  *sigev_notify_attributes;       // notify attrs
};

/* This I/O interfaces doesn't affect file offset maintained by operating system.
 * operation:
 * simultaneous aio_read or aio_write using same aiocb yield undefined result
 * errno: EINPROGRESS, ENOSYS, EINTER, ECANCELED */
aio_read();     aio_write();    aio_fsync();    aio_error();    aio_return();
aio_suspend();  lio_listio();

int aio_suspend(const struct aiocb *const aiocb_list[], // don't forget const
            int  nitems, const struct timespec *timeout);
    // block until an operation complete.
    // aio_suspend && lio_listio --> 0:ok, else: -1
int aio_cancel(int fd, struct aiocb *aiocb);
    // if aiocb is NULL, cancel all opt on the fd.
    // return value: AIO_ALLOWED, AIO_CANCELED, AIO_NOTCANCELED, -1
int lio_listio(int mode, struct aiocb *const aiocb_list[], int nitems, struct sigevent *sevp);
    //mode: LIO_WAIT, LIO_NOWAIT
```

## Scatter Read and Gather Write
```C++
// <sys/uio.h>
ssize_t readv(int fd, const struct iovec *iov, int iovcnt);
ssize_t writev(int fd, const struct iovec *iov, int iovcnt);
struct iovec {
    void    *iov_base; // starting addr of the buffer
    size_t iov_len;    // size of the buffer
};
```

## readn and writen:
```C++
ssize_t readn(int fd, void *buf, size_t nbytes);
ssize_t writen(int fd, void *buf, size_t nbytes);

-- P524 -- review code
```

# System Data Files and Information

## System Identification:
```C++
int uname(struct utsname *name); // return information on the current host and operating system.
struct utsname {
    char sysname[];     // operating system name
    char nodename[];    // name of this node
    char release[];     // current release of operating system
    char version[];     // current version of this release
    char machine[];     // name of hardware type
};
```

## Time and Data Rountine:
```C++
            string              formatted string
                \                     /
                    \                 /
                struct tm(broken-down time)
                            |
                            |
    timeval ---> time_t(calendar time) <--- timespec
                        \     |      /
                        \   |    /
                                |
                            kernel

time_t              --------------> time
struct timespec {   --------------> clock_gettime
    time_t  tv_sec;
    long    tv_nsec;
};
struct timeval{     --------------> gettimeofday
    long tv_sec;   //seconds
    long tv_usec;  //microseconds 999999 20bits
};
struct tm { broken-down time
    int tm_sec;     // second after minues: [0-60]
    int tm_min;     // minutes after hours: [0-59]
    int tm_hour;    // hours after midnight: [0-23]
    int tm_mday;    // day of the month: [1-31]
    int tm_mon;     // months since January: [0-11]
    int tm_year;    // year since 1900
    int tm_wday;    // day since Sunday:[0-6]
    int tm_yday;    // day since January: [0-365]
    int tm_isdst;   // daylight saving time flag: 0 and positive value
};
time_t time(time_t *calptr); // <time.h>
int clock_gettime(clokid_t clock_id, struct timespec *tsp); // <sys/time.h>
    // clock_id: CLOCK_REALTIME, CLOCK_MONOTONIC, CLOCK_PROCESS_CPUTIME_ID, CLOCK_THRAD_CPUTIME_ID
int clock_getres(clockid_t clock_id, struct timespec *tsp); // determine the resolution of given system clock
int clock_settime(clock_id clock_id, const struct timespec *tsp);
int gettimeofday(struct timeval *tp, void *tzp);

struct tm *gmtime(const time_t *calptr);
struct tm *localtime(const time_t *calptr);
time_t mktime(struct tm *tmptr);
size_t strftime(char *buf, size_t maxsize, const char *format, const struct tm *tmptr);
size_t strftime(char *buf, size_t maxsize, const char *format, const struct tm *tmptr, local_t local);
char *strptime(const char *buf, const char *format, struct tm *tmptr);
```

# Network Programming
## Data trasformation:
```C++
#include <netinet/in.h>
uint32_t htonl(uint32_t hostlong);
uint16_t htons(uint16_t hostshort);
uint32_t ntohl(uint32_t netlong);
uint16_t ntohs(uint16_t netshort);

#include <arpa/inet.h>
in_addr_t       inet_addr(const char *cp);
int             inet_aton(const char *cp, struct in_addr *inp);
char           *inet_ntoa(struct in_addr in);
    // return a pointer to a static variable in funciton which store the rst, func can't reenter.

const char     *inet_ntop(int af, const void *src, char *dst, socklen_t len); // len: INET_ADDRSTRLEN
int             inet_pton(int af, const char *src, void *dst);  // convert IP from text to binary form

in_addr_t       inet_network(const char *cp);
struct in_addr  inet_makeaddr(int net, int host);
in_addr_t       inet_lnaof(struct in_addr in);
in_addr_t       inet_netof(struct in_addr in);
```

## socket
1. Create Soket (client and server)
    ```C++
    int socket(int domain, int type, int protocol);
    // doamin: AF_INET, AF_INET6, AF_LOCAL(AF_UNIX), AF_ROUTE
    // type: SOCK_STREAM, SOCK_DGRAM, SOCK_RAW, SOCK_PACKET, SOCK_SEQPACKET, SOCK_NONBLOCK
    // protocol: IPPROTO_TCP, IPPROTO_UDP, IPPROTO_SCTP
    // type can combine with SOCK_CLOEXEC: when create new socket by fork, close this socket in child socket.

    int bind(int sockfd, const struct sockaddr *addr, socklen_t addrlen);

    struct sockaddr {
        sa_family_t sa_family;
        char sa_data[14];
    };

    struct sockaddr_in {    <netinet/in.h>
        sa_family_t sin_family; // address family
        in_port_t sin_port;     // port in network byte order, > 1024
        struct in_addr sin_addr;// internet address: INADDR_<ANY, BROADCASE>
    };
    typedef unsigned short sa_family_t;

    struct in_addr {
        uint32_t s_addr;    // 32-bit address in network byte order
    };
    // If a TCP client or server do not specify port, the kernel chooes an ephemeral port for the socket
        when either connect or listen is called. We can call getsockname return the protocol addr.
    // If a TCP server do not bind and IP addr to its socket, the kernel uses the destination IP addr of the
        client's SYN as the server's source IP address.
    // INADDR_ANY(0), INADDR_XXX defined by the <netinet/in.h> header are defined in host byte order.
    ```

2. listen(sever) and connect(client)
    ```C++
    int listen (int sockfd, int backlog);
    int connect(int sockfd, const struct sockaddr *addr, socklen_t addrlen);
    // Three conditions generate RST:
        1. When a SYN arrives for a port that has no listening server
        2. When TCP wants to abort an existing connection.
        3. When TCP receives a segment for a connection that does not exist.
    // Do no specify backlog to 0, different implementation interpret this different. If don't want any
        client connecting to listening socket, just close the listening socket.
    // Berkeley-derived implementation add a fudge factor to backlog: it is multiplied by 1.5.(MacOS)
    ```

3. accept (client and server)
    ```C++
    // return the next completed connetion from the front of the completed conn queue, don't care the status
    // of the socket(may be ESTABLISHED or CLOSE_WAIT) and the changes of the network.
    int accept(int sockfd, struct sockaddr* addr, socklen_t *addrlen);
    ```
    * Connection Abort befor 'accept' Returns:
        1. When client call connect, three-way handshake completes, the connection established, and then client TCP send an RST. On server side, the connection is queued by its TCP, waiting for the server process to call accept when the RST arrives. Sometime later, ther server process calls accept.
        2. What happens to the aborted connection is implementation-dependent. Berkeley-derived implementation handle the aborted connection completely within the kernel, and the server process never sees it. Most SVR4 implementations, return error to the process as the return from accept, and error depends on the implementation. SVR4 return an errno of EPROTO but POSIX return ECONNABORTED.


4. read and write (client and server)
    ```C++
    ssize_t read(int fd, void *buf, size_t count);
        // The behavior of multi-thread read or write is undefined
    ssize_t write(int fd, const void *buf, size_t count);

        // flags: MSG_PEEK, MSG_OOB, MSG_WAITALL, MSG_CONFIRM, MSG_DONTROUTE
            MSG_DONTWAIT, MSG_EOF, MSG_EOR, MSG_MORE, MSG_NOSIGNAL
    ssize_t send(int sockfd, const *buf, size_t len, int flags);
        // return success doesn't mean the peer process recvices data, just means the data
        //    has been delivered to network drivers without error
        // send data out of boundary will cause send fail with errno set to EMSGSIZE
        //   with byte-stream protocol send will block until entire amount of data has been transmitted
    ssize_t recv(int sockfd, void *buf, size_t len, int flags);

    ssize_t sendto(int sockfd, const void *buf, size_t len, int flags,
                    const struct sockaddr *dest_addr, socklen_t addrlen);
    ssize_t recvfrom(int sockfd, void *buf, size_t len, int flags,
                    struct sockaddr *stc_addr, socklen_t *addrlen);

    ssize_t sendmsg(int sockfd, const struct msghdr *msg, int flags);
    ssize_t recvmsg(int sockfd, struct msghdr *msg, int flags);
    struct msghdr {
        void *msg_name;
        socklen_t msg_namelen;

        struct iovec* msg_iov;
        int msg_iovlen;

        void* msg_control;
        socklen_t msg_controllen;

        int msg_flags;
    };
    ```

5. shutdown(int socket, int how);
    ```C++
    // how: SHUT_RD: close read peer, discard data in read buffer
    // SHUT_WR: close write peer, send data in write buffer before real close
    // SHUT_RDWR, shutdown soket send and recv
    // In multi-threads one thread calls shutdown will affect other threads' action to the same socket
    ```
6. close <unistd.h>
    ```C++
    int close(int fd);
    /* decrement the reference by one, if the refrence count is 0, close the socket.
     * The default action of close with a TCP socket is to mark the socket as closed
     * and return to the process immediately. The scoket descriptor is no longer usable by the process:
     * It can not be used as argument to read or write. But, TCP try to send any data that
     * is already queued to be sent to the other end, and after this occurs,
     * the normal TCP connection termination sequence takes palce.
     * In multi-thread one thread close socket just decrement its reference count,
     * not affect other threads action to the same socket. Once all threads call close() to the same socket,
     * socket will close at the end that is the reference count is down to 0. */
    int getsockname(int sockfd, struct sockaddr *restrict addr, socklen_t *alenp);
    int getpeername(int sockfd, struct sockaddr *restrict addr, socklen_t *alenp);
    ```

## Raw-socket:
```C++
int setsockopt (int socket, int level, int optname, const void *optvalue, socklen_t optlen);  // ok:0
int getsockopt (int sockfd, int level, int optname, void *optval, socklen_t *optlen);
/* level:
SOL_SOKCET:
    SO_BROADCASE(bool)  SO_RCVTIMEO(struct timeval) SO_SNDTIMEO(struct timeval)
    SO_OOBINLINE    SO_LINGER   SO_TYPE     SO_RCBLOWAT     SO_SNDLOWAT
    SO_DONTROUTE    SO_ERROR    SO_DEBUG
    SO_KEEPALIVE(only have meaning for stream sockets)
    SO_RCVBUF [/proc/sys/net/ipv4/tcp_rmem]
    SO_SNDBUF [/proc/sys/net/ipv4/tcp_wmem]
    SO_BROADCASE(have no meaning for stream sockets)
    SO_REUSEADDR [/proc/sys/net/ipv4/tcp_tw_recycle]
IPPROTO_IP:
    IP_HDRINCL(bool)    IP_TOS  IP_TTL
    IP_MULTICAST_LOOP   IP_MULTICAST_TTL
    IP_ADD_MEMBERSHIP(struct ip_mreq)
    IP_DROP_MEMBERSHIP  IP_MULTICAST_IF(DWORD)
IPPROTO_IPV6:
    IPV6_NEXTHOP IPV6_RECVPKTINFO IPV6_DONTFRAG IPV6_RECVTCLASS
IPPROTO_TCP:
    TCP_MAXSEG  TCP_NODELAY */

/* For server programming: some options should set before listening since the socket
 * accepted has at least finished two steps of handshake with the client(SYN_RECV).
 * Solution: set those options for the listening socket, sockets accepted will inherit those options.
 * SO_DONTROUTE, SO_KEEPLIVE, SO_LINGER, SO_OOBINLINE, SO_RECVBUF, SO_RCVLOWAT,
 * SO_SNDLOWAT, SO_SNDBUF, SO_RCVBUF, TCP_MAXSEG, TCP_NODELAY */

struct linger {
    int l_onoff;
    int l_linger;
};
/* 1. l_onoff = 0: SO_LINGER doesn't work, close socket by default
 * 2. l_onoff =1, l_linger = 0: close() return immediately, discard write buffer data, send RST to peer
 * 3. l_onoff =1, l_linger > 0:
 *   3.1 block socket: close() waits l_linger time and close, if data sent not finished, return -1, errno is EWOULDBLOCK
 *   3.2 close() return immediately, detect whether residual data send finished by errno and return value */
```

## Out-of-Band data:
* int sockatmark(int fd);
* OOB data is only supported on AF_INET and AF_INET6, the data can be sent by specifying the MSG_OOB fly on send function.
* The SIOCATMARK ioctl() request determine if the read pointer is pointing to the last OOB byte.
* If multiple occurrences of OOB data are sent, the OOB marker pointes to the last OOB byte of the final OOB data occurrence.
* SIGURG donesn't indicate the actual arrival of the OOB data, but only notification that is pending.

* Two method to detect OOB data's arrival:
    * SIGURG will be triggered.
    * I/O multiplexing return exception event.

* Reciev OOB data:
    * The socket API keeps track of recieved OOB data by using OOB marker witch points to the last byte of
            the OOB data.
    * Independent of the OOB data is recieved inline, an input operation processes data up to OOB marker,
            if OOB data sent.
    * If SO_OOBINLINE is not set:
            if MSG_OOB is specified the data will be recieved, otherwise not. if multi-data sent, only final
            OOB is remebered.
    * If SO_OOBINLINE is set: all OOB data sent is valid, read function must read them without specifying
            MSG_OOB(EINVAL).

## Architecture of Server:
1. IO communication unit
    1. Port multiplexing: select 1000, poll, epoll 30W
    2. Synchronization: http https
    3. asynchroniztion: signal, sigaction
    4. asynchronous IO: aio_read, aio_write
2. Data Process Unit
    1. Multiple Thread
        * Thread Mutext: mutex, spin, atomic
        * Thread Synchronization: PV, semaphore
        * Thread Communication
    2. Multiple Prcesses
        * IPC pip, fifo, socketpair
    3. Multiple Thread + Multiple Process
        * Thread Pool
        * Process Pool
3. Date Storage

* Two High-performance Event Handle Architectures:
    1. Reactor: implemented by synchronize I/O
    2. Proactor: implemented by asynchronize I/O

* Two high-performance Concurrent Architectures:
    1. half-sync/half-async
    2. leader/follwer

## Network configuration information:
```C++
#include <netdb.h>
struct hostnet *gethostent(void);
void sethostent(int stayopen);
void endhostent(void);
struct hostent {
    char     *h_name;       // "offical" name
    char    **h_aliases;    //
    int       h_addrtype;   // AF_INET, AF_INET6
    int       h_length;
    char    **h_addr_list;
    char    *h_addr;        // h_addr_list[0]
    ...
};
struct hostent* gethostbyname (const char *name);
struct hostent* gethostbyaddr (const void *addr, socklen_t length, int format); AF_INET, AF_INET6
```

## Termination of Server Process:
1. Kill server process. All open fd in the process are closed, this cause a FIN to be sent to the client and
    client TCP response with ACK.
2. Nothing happens at the client. The TCP receives the FIN from the server TCP and responds with an ACK, but
    client process is blocked in the call to `fgets` waiting for a line from the terminal.
3. When we type "another line", client TCP sents the data to the server. This is allowed by TCP because the
    receipt of the FIN by the client TCP only indicates that the server process has closed its end of the
    connection and will not be sending any more data. The receipt of the FIN doesn't tell the client TCP that
    the server process has terminated.
4. When the server TCP receives the data from the client, it responds with an RST since the process that had
    that socket open has terminated.
5. The client process will not see the RST because it calls readline immediately after the call to writen and
    readline return 0(EOF) immediately because of the FIN that was received in Step 2.
6. When the client terminates, all its open descriptors are closed.

## SIGPIPE
* When a process writes to a socket that has received an RST, the SIGPIPE signal is sent to the process.
* The default action of this signal is to terminate the process.

## Crashing Server Host
1. When the server host crashed, nothing is sent out on the existing network connection.
2. We type a line of input to the client, it is written by writen, and is sent by the client TCP as data segment. The client then blocks in the call to readline, waiting for the echoed rely.
3. Client TCP continually retransmitting the data segment, trying to receive an ACK from the server. When the client TCP finnaly gives up, an error is returned to client process. `ETIMEDOUT, EHOSTUNREACH`

## Crashing and Rebooting of Server Host:
1. The server host crashed and reboots, its TCP loses all information about connection that existed befor the crash.
2. Client sends data to server, server TCP responds to the received data segment from client with an RST.
3. Client blocked in the call to readline when RST is received, causing readline to return the error ECONNRESET

## Shutdown of Server Host:
* Init process normally sends the SIGTERM signal to all processes, waits some fixed amount of time, and then send SIGKILL signal to any processes still running.

# Signal Programming
## Methods of Local Processes Communication(IPC):
1. Messaging (pipeline, FIFO, message queue)
2. Synchronization (mutex, conditional variable, read and write lock, file and wirte and lock lock, semaphore)
3. Shared memory (anonymous and named)
4. Remote procedure Call (Solaris gate and sun RPC)


* checking for the existence of a process:
    ```C++
    wait();
    semaphore and exclusive file locks;
    IPC(pipes, FIFOs...);
    /proc/PID
    ```

Signal dispositions: Term, Ign, Core, Stop, Cont

## Linux Signals
```C++
Name    Number  Description
SIGHUP  1       Hangup (POSIX)
SIGINT  2       Terminal interrupt (ANSI)
SIGQUIT 3       Terminal quit (POSIX)
SIGILL  4       Illegal instruction (ANSI)
SIGTRAP 5       Trace trap (POSIX)
SIGIOT  6       IOT Trap (4.2 BSD)
SIGBUS  7       BUS error (4.2 BSD)
SIGFPE  8       Floating point exception (ANSI)
SIGKILL 9       Kill(can't be caught or ignored) (POSIX)
SIGUSR1 10      User defined signal 1 (POSIX)
SIGSEGV 11      Invalid memory segment access (ANSI)
SIGUSR2 12      User defined signal 2 (POSIX)
SIGPIPE 13      Write on a pipe with no reader, Broken pipe (POSIX)
SIGALRM 14      Alarm clock (POSIX)
SIGTERM 15      Termination (ANSI)
SIGSTKFLT   16  Stack fault
SIGCHLD 17      Child process has stopped or exited, changed (POSIX)
SIGCONT 18      Continue executing, if stopped (POSIX)
SIGSTOP 19      Stop executing(can't be caught or ignored) (POSIX)
SIGTSTP 20      Terminal stop signal (POSIX)
SIGTTIN 21      Background process trying to read, from TTY (POSIX)
SIGTTOU 22      Background process trying to write, to TTY (POSIX)
SIGURG  23      Urgent condition on socket (4.2 BSD)
SIGXCPU 24      CPU limit exceeded (4.2 BSD)
SIGXFSZ 25      File size limit exceeded (4.2 BSD)
SIGVTALRM       26  Virtual alarm clock (4.2 BSD)
SIGPROF 27      Profiling alarm clock (4.2 BSD)
SIGWINCH    28  Window size change (4.3 BSD, Sun)
SIGIO   29      I/O now possible (4.2 BSD)
SIGPWR  30      Power failure restart (System V)

// SIGSTOP and SIGKILL default actions cannot be changed.

union sigval {
    int     sival_int;  // Integer value
    void    *sival_ptr;
};

struct sigevent {
    int             sigev_notify;   // SIGEV_THREAD, SIGEN_NONE, SIGEN_SIGNAL, SIGEV_THREAD_ID
    void (*sigev_notify_function) (union sigval);   //
    void            *sigev_notify_attributes;       // SIGEV_THREAD
    int             sigev_signo;
    union sigval    sigev_value;                    // Data passed with notifications
    pid_t           sigev_notify_thread_id;         // SIGEV_THREAD_ID
};

typedef void(*sighandler_t)(int)
sighandler_t signal(int signum, sighandler_t handler);
/* multi-process is unspecified behavior is undefined
 * if a process ignores SIGFPE, SIGKILL, SIGSEGV
 *
 * Two common design:
 * 1. set a global flag and exits;
 * 2. perform some type of cleanup and then either teriminates the process
 *
 * or uses a nonlocal goto to unwind the stack and return control to a predetermined location in the main program;
 * Linux doesn't recommand to use signal func directly, since it set some
 * default flags: SA_ONESHOT | SA_NOMASK | SA_INTERRUPT;
```

* sending a signal:
    ```C++
    raise(int sig); kill(getpid(), int sig); killpg(pid_t pgrp , int sig);
    pthread_kill(); tgkill(); sigqueue();
    ```

* signal description:
    ```C++
    sys_siglist[SIGPIPE]; strsignal(int sig); psignal(int sig, const char *msg);
    ```

* waiting for a signal to be caught:
    ```C++
    sigsuspend();  pause(); // -1
    ```

* syschronously accepting a signal:
    ```C++
    sigwaitinfo(); sigtimedwait(); sigwait(const sigset_t *set, int *sig);
    signalfd();
    ```

* signal mask:
    ```C++
    // specify which signals can not be sent to the process
    pthread_sigmask();
    int sigpending(sigset_t *set); // get the current pending signals
    int sigprocmask (int how, const sigset_t *restrict set, sigset_t *restrict oldset);
    /* how: SIG_BLOCK, SIG_UNBLOCK, SIG_SETMASK
     * when a signal which is masked is sent to the process,
     * the system set this signal as pending signal in the process,
     * when we unmask the pending signals, signals will be received by the process immediately */
    ```

* sigset_t:
    ```C++
    sigemptyset(sigset_t *set); sigaddset(sigset_t *set, int signum);
    sigfillset(sigset_t *set);   sigdelset(sigset_t *set, int signum);
    sigismember(const sigset_t *set, int signum); // ok: 1
    sigaltstack();
    ```

* sigaction:
    ```C++
    int sigaction(int signum, const struct sigaction *act, struct sigaction *oldact);

    struct sigaction {
        void (*sa_handler)(int); // SIG_IGN, SIG_DFL
        void (*sa_sigaction)(int, siginfo_t *, void *);   // si_code, si_signo, si_value
        sigset_t  sa_mask;
        int       sa_flags; // set the behavior when program receives a signal
        void (*sa_restorer)(void);
    };
    /* sa_flags: SA_NOCLDSTOP, SA_NOCLDWAIT(SIGCHLD),
     * SA_NODEFER, SA_ONSTACK/SA_RESETHAND, SA_RESETORE, SA_SIGINFO, SA_ONESHOT
     * SA_RESTART: a system call interrupted by this signal will be automatically restarted by the kernel. */
    struct siginfo_t {
        int     si_signo;   // signal number
        int     si_code;    // signal code
        pid_t   si_pid;     // sending process ID
        pid_t   si_uid;     // real user ID of sending process
        void   *si_addr;    // address of faulting instruction
        int     si_status;  // exit value of signal
        long    si_band;    // band event for SIGPOLL
        union sigval si_val;// signal value
    }
    ```

* signaltstack():

* int kill(pid_t pid , int sig ); // (> 0), (0), (-1), (< -1)

* signal handler:
* reentrant function:
    * A function is one, whose effect, when called by two or more threads, is guaranted to be as if threads each executed the function one after the other in undefined order, even if the actual execution is interleaved.
    * use global or static data structure is nonreentrant function.

* async-signal-safe function:

* implementation guarantees to be safe when called form a signal handler, either it is reentrant or is not interruptible by a signal handler.

* tips for writing async-signal-safe functions:
    1. ensure reentrant and call only async-signal-safe function inside;
    2. block delivery of signal while executing code in main program that calls unsafe functions or work with global data structures also updata by signal handler

* use erro inside the signal handler

* global variable should be declared by using volatile: `volatile sig_atomic_t flag;` in order to prevent compiler from performing optimizations that result in varaible being stored in register

* terminating a signal handler:
    ```C++
    _exit(); kill(); raise(); nonlocal goto; abort();
    ```

* performing a nonlocal goto from a signal handler: <setjmp.h>

* int sigsetjmp(sigjump_buf env, int savesigs);    void siglongjmp(sigjmp_buf env, int val);

* POSIX Signal Semantics:
    1. Once a signal handler is installed, it remains installed.
    2. While a signal handler is executing, the signal being delivered is blocked.
    3. If the signal is generated one or more times while it is blocked, it is normally delivered only one time after the signal is unlocked. By default, Unix signal are not queued.
    4. It is possible to selectively block and unblock a set of signal using the sigprocmask function.

# IPC Programming

* Unix inter-domain communication:
    ```C++
    // Unix communicatin:
    socketpair, pipe(PIPE_BUF), FIFO, signal
    // System V:
    semaphore, message queue, shared memory
    ```

* Unix outer-domain communication:

* sockets, STREAMS

* Advantages and Disabvantages of XSI IPC:
    1. They are systemwide have no reference count, once created they'll exite unless deleted explicitly
    2. They are not known by names in the file system
    3. Compare this with a pipe, which is completely removed when the last process to reference it terminates. With a FIFO, although the name stays in the file system until explicitly removed, any data left in a FIFO is removed when the last process to reference the FIFO terminates.
    4. We can't use **ls** **rm** **chmod** to access them, rather than **ipcs, ipcrm**
    5. Since these forms of IPC don’t use file descriptors, we can’t use the multiplexed I/O functions (select and poll) with them.

## Pipe
```C++
<unistd.h>
int pipe(int pipefd[2]);    int pipe2(int pipefd[2], int flag);
// flag:   0, O_CLOSEXEC, O_DIRECT, O_NONBLOCK
// when last process referred to it terminates, the pipe is completely removed.
// If the fd[1]'s reference is down to 0, the read() from fd[0] will return 0;
// If the fd[0]'s reference is down to 0, the write() to fd[1] will fail, SIGPIPE will trigger.
// The capacity of the pipe can be changed by fcntl funciton.
<sys/types.h>
<sys/socket.h>
int socketpair(int domain, int type, int proto, int fd[2]);
```

## FIFO:
```C++
mkfifo(const char *path, mode_t mode);              // absolute path, mode same to open
mkfifoat(int fd, const char *path, mode_t mode);    // path relative to the directory of fd
// Once created, FIFO file exites in the file system
// without O_NONBLOCK: read block untill other process write, the same to write;
// O_NONBLOCK: read return immediately if no write, but write return -1 with errno  ENXIO if no read
// As with pipe if write to a FIFO without reader SIGPIPE is generated
// When last process referred to it terminates, the FIFO name remains in system, but any data removed.
```

## popen: pipe stream to or from a process
```C++
FILE *popen(const char *command, const char *type); // type: "r", "w","e"
int pclose(FILE *stream);
```

## Identifiers and Keys:
```C++
key_t ftok(const char *pathname, int proj_id);
key: 31-24(proj_id low 8 bits), 23-16(st_dev low 8 bits), 15-0(st_ino low 16 bits)
```

## msg_queue
```C++
struct msqid_ds {
    struct ipc_perm msg_perm;
    time_t          msg_stime;
    time_t          msg_rtime;
    time_t          msg_ctime;
    unsigned long   __msg_cbytes;
    msgqnum_t       msg_qum;
    msglen_t        msg_qbytes;     // MSGMNB, MSGMNI
    pid_t           msg_lspid;
    pid_t           msg_lrpid;
};
struct ipc_perm {
    key_t           __key;
    uid_t           uid;
    gid_t           gid;
    uid_t           cuid;
    git_t           cgid;
    unsigned short  mode;   // permission
    unsigned short  __seq;  // sequence number
};
struct msgbuf{
    long mtype;     // who
    char mtext[1];  // msg data
};
struct msg_info {
    int     msgpool;    // kibibytes of buffer pool, hold msg data
    int     msgmap;     // Maximun num of entries in msg map
    int     msgmax;     // Single message max size
    int     msgmnb;     // queue max size
    int     msgmni;     // message queues max num
    int     msgssz;     // message segment size
    int     msgtql;     // message max size on all queues in system
    unsigned short int msgseg;  // segment max size
};
int msgget(key_t key, int msgflag); // open or creat a new one
    msgflag: IPC_CREAT|666, IPC_EXCL|666, IPC_NOWAIT|666
int msgctl(int msgid, int cmd, struct msqid_ds *buf);
    cmd: IPC_STAT, IPC_SET, IPC_RMID, IPC_INFO, MSG_INFO, MSG_STAT
int msgsnd(int msgid, const void *msgbuf, size_t msgsz, int msgflag);
ssize_t msgrcv(int msgid, void *msgbuf, size_t msgsz, long msgtyp, int msgflag);
    flag: MSG_NOERROR, MSG_COPY, MSG_EXCEPT, MSG_COPY, IPC_NOWAIT
    type: 0, >0, <0(the first msg whose type is the lowest value less than or euqal to absolute value of type)
```

## Semaphore:
is a counter used to provide access to the shared object for multiple processes
* category:
    1. kernel semaphore
    2. user semaphore: POXIS, SYSTEM V

1. POXIS unamed semaphore:
    ```C++
    int sem_init(sem_t *sem, int pshared, unsigned int value);
    // pshard: zero[shared between threads within process], non-zero[shared between processes]
    // The behavior of initializing a initiazlied thread is undefined
    int sem_getvalue(sem_t *sem, int *sval);
    int sem_destroy(sem_t*sem); // destory a semaphore waited by other threads is unfined
    int sem_wait(sem_t *sem);
    int sem_post(sem_t *sem);
    int sem_trywait(sem_t *sem);
    ```

2. POSIX named semaphore:
    ```C++
    sem_t* sem_open(const char* name,int oflag,mode_t mode);
    sem_t *sem_open(const char *name, int oflag, mode_t mode, int value);
        oflag: O_CREAT, O_CREATE|O_EXCL
    int sem_wait(sem_t *sem);
    int sem_post(sem_t *sem);
    int sem_close(sem_t *sem); // system will auto close if forget call
    int sem_unlink(const char *name); // destroy
    ```

3. SYSTEM V semaphore:
    ```C++
    struct ipc_perm {
        key_t key;
        uid_t uid;
        gid_t gid;
        uid_t cuid;
        gid_t cgid;
        mode_t mode;
    };
    struct semid_ds {
        struct ipc_perm sem_perm;
        struct sem      *sem_base;  // ptr to first sem in array
        ushort          sem_nsems;  // num of the sem array
        time_t          sem_otime;  // last semop time
        time_t          sem_ctime;  // last semctl time
        ...
    };
    struct sem {
        ushort  semval;
        short   sempid;  // the last process id who executes semop
        ushort  semncnt; // number of the processes  waiting for the semaphore to be incremented
        ushort  semzcnt; // number of the processes waiting for the semaphore to be 0
        ...
    };
    struct sembuf {
        short   sem_num;    // semaphore index in the semaphore container
        short   sem_op;     // PV operation: >0, =0, <0
        short   sem_flg;    // SEM_UNDO, IPC_NOWAIT
    };
    int semget(key_t key, int num_sems, int oflag);
    int semop(int semid, struct sembuf *sem_ops, size_t nops);
    /* if no IPC_NOWAIT flag, semop can return from 3 cases:
    *   1. the op condition is satisfied(semval equels to 0, can be increment/decrement)
    *   2. the semaphore container operating is removed, return -1, errno set to EIDRM
    *   3. interuped, return -1, errno set to EINTR, decrement semzcnt/semncnt
    */
    int semctl(int semid, int semnum, int cmd, ../*union semun arg*/);
        union semun {
            int             val;    // cmd == SETVAL
            struct semid_ds *buf;   // cmd == IPC_SET, IPC_STAT
            ushort          *array; // cmd == SETALL, GETALL
        };
        cmd: IPC_RMID, IPC_INFO, GETALL, GETNCNT, GETPID, GETVAL
    ```

## POSIX Shared Memory:
```C++
#include <sys/mman.h>
#include <sys/stat.h>        /* For mode constants */
#include <fcntl.h>           /* For O_* constants */
int shm_open(const char *name, int oflag, mode_t mode);
int shm_unlink(const char *name);
```

## System V Shared memory: (fastest IPC)
```C++
struct shmid_ds{
    struct ipc_perm shm_perm;
    size_t          shm_segsz;  // round up to system multiple page sizes
    time_t          shm_ctime;  // last shmctl time
    time_t          shm_atime;  // last shmat time
    time_t          shm_dtime;  // last shmdt time
    pid_t           shm_cpid;   // pid the of creator
    pid_t           shm_lpid;   // pid execute last shmat/shmat
    shmatt_t        shm_nattch; // reference count of process
    ...
};
int shmget(key_t key, size_t size, int shmflag);
    shmflag: SHM_R, SHM_W, 0
void *shmat(int shmid, const void *shmaddr, int shmfalg);
    // return addr of the shm in the process addr space
    // shmflag: SHM_EXEC, SHM_RDONLY, 0, SHM_RND
int shmctl(int shmid, int cmd, struct shmid_ds *buf);
    // cmd: <IPC/SHM>_STAT, IPC_SET, IPC_RMID,  SHM_LOCK, SHM_UNLOCK
int shmdt(const char *shmaddr);
    // shm remains in existence untill call shmctl with IPC_RMID flag
```

## Summary:
* Learn pipe and FIFOs avoid using message queue and semaphore, Full-duplex pipes and record locking should be
* considered instead as they are far simpler. Shared memory still has its use.


# Thread Process Programming
## Thread & Signal
```C++
pthread_sigmask(int how, const sigset_t *set, sigset_t *oldset); //how: SIG_BLOCK, SIG_UNBLOCK, SIG_SETMASK
pthread_kill(pthread_t pthread, int sig); // send a signal to a specific pthread
```

* All threads share the signals and sigHandler of the process which they belong to, system sends process signal to thread according to the thread signal mask.

* When we set a new sigHandler in a thread, it will overwrite all other threads sigHandler for this signal.

* We should define a dedicated thread to handle all signals:
    1. Set signals mask before main thread creates any thread, the new thread will inherite this sigmask.
    2. Call sigwait(const sigset_t *set, int *sig) in one thread to handle all signals.

## Methods of Local Processes Communication:
1. Messaging (pipeline, FIFO, message queue)
2. Synchronization (mutex, conditional variable, read and write lock, file and wirte and lock lock, semaphore)
3. Shared memory (anonymous and named)
4. Remote procedure Call (Solaris gate and sun RPC)

## Main function
1. When a C program is executed by the kernal-by one of the exec functions, a special start-up routine is called before the main is called.
2. The executable program file specifies this routine as the starting address for the program. This is set by the link editor when it is invoked by the C compiler.
3. This start-up routine takes up values from the kernel and sets things up so that the main functions is called.

    3. parent exit after child with call wait, both parent and child exit normally.

## Handling Zombie process
1. When child exit it will send SIGCHLD to parent, parent can call wait in SIGCHLD's handler to terminate zombie `while ((pid = waitpid(-1, &stat, WNOHANG)) >0)  printf("child %d terminated.\n", pid);`
2. fork twice.
3. Ignore the SIGCHLD signal. (only works under System V and Unix 98, unspecified in POSIX)

* The purpose of the zombie state:
    * is to maintain information about the child for the parent to fetch at some later time.

* Results of different order of parent and child process exit:
    1. parent exit before child process: child process becomes orphan process, init process takes over it.
    2. parent exit after child without call wait or waitpid, child becomes zombie process.

* Salient points of Zombie Processes
    1. All the memory and resources allocated to a process are deallocated when the process terminates using the exit() system call. But the process’s entry in the process table is still available. This process is now a zombie process.
    2. The exit status of the zombie process zombie process can be read by the parent process using the wait() system call. After that, the zombie process is removed from the system. Then the process ID and the process table entry of the zombie process can be reused.
    3. If the parent process does not use the wait() system call, the zombie process is left in the process table. This creates a resource leak.
    4. If the parent process is not running anymore, then the presence of a zombie process indicates an operating system bug. This may not be a serious problem if there are a few zombie processes but under heavier loads, this can create issues for the system such as running out of process table entries.
    5. The zombie processes can be removed from the system by sending the SIGCHLD signal to the parent, using the kill command. If the zombie process is still not eliminated from the process table by the parent process, then the parent process is terminated if that is acceptable.

## Handling Interrupted System Calls
* When a process is blocked in a slow system call and process catches a signal and the signal handler returns, the system call can return an error of EINTER. We should handle this error to restart system call.
* Even if an implementation supports the `SA_RESATRT` flag, not all interrupted system call may automatically be restarted.
* We can recall accept, read write, select and open to handle those system call's interruption, but can't recall connect for which happen we must call select to wait for the connection to comlete.

## Session:
```C++
int setsid(void); // <unistd.h>
// if the calling process is not the process grop leader:
// 1. The process becomes the session leader and the only process of this session.
// 2. Create a new process group and the process becomes the new group leader, and the process group ID is the
//     calling process ID.
// 3. The process has no controling terminal.
getsid(pid_t pid); //the same as the getpgid(pid_t pid);
// The function returns an error is the process is already a process leader.
```

## Enviroment List
```C++
#include <stdlib.h> // Global variable: environ;
char* getenv(const char *name);
int putenv(char *str); //form: "name=value"; already exited one will be remvoed
int setenv(const char *name, const char *value, int rewrite);
int unsetenv(const chart *name);
```

## Memory layout of Program:
```C++
                    |---------------------------------------------------------------------------------
                    |                               |
                    |                               | command-line arguments and environment variables
                    |                               |
        0xC0000000  |-------------------------------|-------------------------------------------------
                    |   stack                       | automatic variable, function called  info
                    |- - - - - - - - - - - - - - - -| - - - - - - - - - - - - - - - -
                    |                               |
                    |                               |
                    |                               |
                    |- - - - - - - - - - - - - - - -| - - - - - - - - - - - - - - - -
                    |   heap                        | dynamic memeory allocation
                    |-------------------------------|--------------------------------
                    |uninitialized static data(.data)| initialized to zero by exex
                    |-------------------------------|-------------------------------------------------------------
                    | initialized static data (.bss)|
                    |       constance               |
                    |-------------------------------|--------- read from program file by exec
                    |      .rodata                  | (machine instructions, sharable, read only)
                    |      .text                    |
        0x08048000  |-------------------------------|--------------------------------------------------
    notes:
        Only initialized data and text segment are stored in the program file on disk
Memory Allocation: <stdlib.h>
    Implemented with sbrk() system call.
    Freed space is not returned to the kernel, instead, it's kept in the malloc pool and can be use for next call.
    Writing past of the end or before the begining of a allocated block is fatal.
```

## Process Enviroment:
```C++
// Exit Function:
_exit(...), _Exit(...)  // return to the kernel immediately
exit(...)               // performs certain cleanup processing then returns to kernel
pthread_exit();
returns of the last thread from its start routine
response of the last thread to the cancellation request
int atexit(void *(func)(void));
    // call on registered func in reverse order and as many times as registered
    // register at leat 32 exit handlers that are automatically called by exit.

fork();
// Unique: PID, memory locks, record locks, CPU time counter, pending signals,
// semaphore adjustments, outstanding async I/O operations, timer
// Child process copys parent's data, head and stack but shares text segment and file descriptors including server's listenfd, and connectfds.
// After the fork, the fds opened in the parent process are still open, and reference counts incremented by 1.
// There are two ways to handling descriptors after fork:
//   1. Parent waits for child to complete.
//   2. Both parent and child go their way(close descriptors don't need, respectively)

pthread_atfork(void (*prepare)(void), void (*parent)(void), void (*child)(void)); // clean up lock status
// child process can inherits parent's lock, but don't know whether it is locked or unlocked

int execve(const char *path, char *const argv[], char *const envp[]);
//The new program still has the same PID, and it inherits all of the file descriptors that
// were open at the time of the call to the execve function.

pid_t wait(int *stat_loc);  <sys/wait.h>
    WIFEXITED(sta)--WEXITSTATUS()   WIFSIGNALED()-WTERMSIG()    WIFSTOPED()--WSTOPSIG() WIFCONTINUED(sta)
    // suspends execution of the calling thread until one of its children terminates.
    // return immediately with terminate status if one child is zombie.

pid_t waitpid(pid_t pid, int *stat_loc, int options); // options: WCONTINUED, WNOHANG, WUNTRACED
// pid > 0; wait the process whose process ID equals pid
// pid == 0; wait the process whose process group ID equals calling process group ID
// pid == -1; wait for any process
// pid < -1; wait the process whose process group ID equals absolute value of pid
// If the calling process has no children, returns −1 and sets errno to ECHILD
// If the waitpid function was interrupted by a signal, returns −1 and sets errno to EINTR

int waittid(idtype_t idtype, id_t id, siginfo_t *infop, int options);
// idtype: P_PID, P_PGID, P_ALL
// options: WCONTINUED, WEXITED, WNOHANG, WNOWAIT, WSTOPED

#include<sys/types.h> <sys/wait.h> <sys/time.h> <sys/resource.h>
pid_t wait3(int *statloc, int options, struct rusage *rusage);
pid_t wait4(pid_t pid, int *statloc, int options, struct rusage *rusage);
// Additional feature is allow kernel to return a summary of resources used by the terminated
// process and all its child processes.
// Resource information includes such  statistics as the amount of the user CPU time,
// amount of the system CPU time, number of page faults, number of signals received...
```

## Memory Allocation:
```C++
void *malloc(size_t size);  // initial value of the memory is indeterminate.
void *calloc(size_t nobj, size_t size);     // initial value is 0.
void *reallocc(void *ptr, size_t newsize);  // contents in newly increased space is indeterminate
    // Because old erea may move, we shouldn't have any pointers into this area.
void free(void *ptr);
    // The freed space is usually not return to kernel but put into a pool of available memory(malloc pool)and
    // can be used again by call to one of the three alloc functions.
```

## Process Scheduling: <unistd.h>
```C++
// nice value range: 0 ~ 2 * NZERO - 1
// modify priority: int nice(incr); // new nice value - NZERO if ok,-1 on error
// priority: <sys/resource.h>
int getpriority(int which, id_t who);
    // which: PRIO_PROCESS, PRIO_PGRP, PRIO_USER
    // Return: nice value between -NZERO and NZERO - 1 if ok, -1 on error
int setpriority(int which, id_t who, int value);
    // The value is added to NZERO and this becomes new nice value
    // Return: 0 if ok, -1 on error
```

## Process Time: <sys/times.h>
```C++
struct tms {
    clock_t tms_utime;  // user CPU time
    clock_t tms_stime;  // system CPU time
    clock_t tms_cutime; // user CPU time, terminated children
    clock_t tms_cstime; // system CPU time, terminated children
};
clock_t times(struct tms *buf); // return wall clock time
```

## Process exit functions:
* Five normal terminations:
    1. Executing return from the main function.
    2. Executing return from the start routine of the last thread in the process.
    3. Calling the exit(...) function.
    4. Calling the _exit(...), _Exit(...) function.
    5. Calling the pthread_exit(...) from the last thread in the process.

* Three abnormal terminations:
    1. Calling the abort(...) function. SIGABRT will generate.
    2. When process receive certain signals.
    3. The last thread responds to the cancellation request.

* Whenever a process terminates, either normally or abnormally, SIGCHLD will send to parent

## pthread
1. Basic
    ```C++
    pthread_t pthread_self(void); // get current thread ID
    int pthread_equal(pthread_t tid1, pthread_t ti2);
    pthread_create(pthread_t *thread, const pthread_attr_t *attr, void *(*start_rountin)(void *), void *arg);
        // inherits a copy of the creating thread's signal mask; pending signal is empty; don't inherit singal stack.
        // Newly created threads have access to the process address space and inherit calling thread's floating-point
            environment.
        // The order of executing between child and parent process depends on the scheduling algorithm used by kernel.
    pthread_join(pthread_t thread, void **retval);
    pthread_exit(void *retval);
    pthread_detach(pthread_t thread);

    pthread_attr_setdetachstate(pthread_att_t *attr, int detachstate); // PTHREAD_CREATE_<DETACHED, JOINABLE>
    pthread_attr_getdetachstate(const pthread_attr_t *attr, int *detachstate); //
        // A thread's termination status is retained until we call pthread_join for the thread.
        // A thread underlying storage can be reclaimed immediately on termination if the thread has been detached.
    pthread_cancel(pthread_t); //re-acquire mutex before acting cancle and ensure it not consume any condtion signal
        // This fun doesn't wait for the thread to terminate, it merely makes a request.
        // Calling on the detached thread rereults in undefined behavior.
    pthread_setcancelstate(int state, int *oldstate);   // PTHREAD_CANCEL_ENABLE, PTHREAD_CANEL_DISABLE
    pthread_setcanceltype(int type, int *oldtype);      // PTREAD_CANCEL_<ASYNCHRONOUS, DEFERRED>
    pthread_testcancel(void); // create a cancellation point

    pthread_cleanup_push(void (*routine)(void *), void *arg);
    pthread_cleanup_pop (int execute);
        // execute: wether execute the clean up handler routines when pop them.
        // Cleanup handler are recorded in the stack which means they are executed in a reverse order.
        // Cleanup functions can be called when:
            a. a call to pthread_exit
            b. response to a cancellation request
            c. pthread_cleanup_pop with nonzero execute argument.
        // Those functions can be implemented as macros and must be used in matched paires within
            same scop in the thread.
    atexit(void (*function)(void));
    // process completion status:
    int WIFEXITED(int)-WEXITSTATUS, WIFSIGNALED-WTERMSIG, WCOREDUMP, WIFSTOPED, WSTOPSIG
    ```

2. pthread_attr:
    ```C++
    typedef struct {
        int     detachstate;    // PTHREAD_CREATE_<JOINABLE, DETACHED>, td ID, rsc can be reused; join not reuse
        int     schedpolicy;    // SCHED_<FIFO,RR,OTHER>,determine which thread to proceed not the ordering rule
        int     inheritsched;
        struct sched_param  schedparam;
        int     scope;          // SCOPE_SYSTEM, SCOPT_PROCESS
        size_t  guardsize;      // ignored if stackaddr is set
        int     stackaddr_set;
        void *  stackaddr;      // PTHREAD_STACK_SIZE
        size_t  stacksize;      // with JOINABLE attr, can't free untill pthread_join() called.
    } pthread_attr_t;

    pthread_attr_init(struct pthread_attr_t *attr);
        scope           PTHREAD_SCOPE_PROCESS
        detachstate     PTTHREAD_CREATE_JOINABLE
        stackaddr       NULL
        stacksize       1M
        inheritsched    PTHREAD_INHERIT_SCHED
        schedpolicy     SCHED_OTHER
    pthread_attr_destroy(pthread_attr_t *attr);
    pthread_attr_setdetachstate(pthread_attr_t *attr, int state);
    pthread_attr_getdetachstate(pthread_attr_t *attr, int *state);
    pthread_attr_setstacksize(pthread_attr_t *attr, size_t size);   // PTHREAD_STACK_MIN
    pthread_attr_getstacksize(pthread_attr_t *attr, size_t *size);  // get mim stack size
    pthread_attr_setguardsize(pthread_attr_t *attr, size_t size);   // 0[no guard], round up to multiple PAGESIZE
    pthread_attr_getguardsize(pthread_attr_t *attr, size_t *size);
    ```

3. pthread_key:
    ```C++
    pthread_key_create(pthread_key_t *key, void (*destructor)(void *));
    pthread_key_delete(pthread_key_t key);
    pthread_setspecific(pthread_key_t key, const void *pointer);
    void *pthread_getspecific(pthread_key_t key);
    ```

3. pthread_mutex:
    ```C++
    pthread_mutex_init(pthread_mutex_t *mutex, const pthread_mutexattr_t *attr);
        // initialization will not execute immediately until first call
    pthread_mutex_t lock = PTHREAD_MUTEXT_INITIALIZER;
    pthread_mutex_destroy(pthread_mutex_t *mutex);
    pthread_mutex_lock(pthread_mutex_t *mutex);
    phtread_mutex_trylock(pthread_mutex_t *mutex);
    phtread_mutex_unlock(phtread_mutex_t *mutex);

    pthread_mutexattr_init(pthread_mutexattr_t *attr);
    pthread_mutexattr_destroy(pthread_mutexattr_t *attr);
    pthread_mutexattr_getpshared(const pthread_mutexattr_t *attr, int *pshared);
    pthread_mutexattr_setpshared(pthread_mutexattr_t *attr, int pshared);
    // pshared: PTHREAD_PROCESS_PRIVATE PTHREAD_PROCESS_SHARED
    pthread_mutexattr_gettype(const phread_mutexattr_t *attr, int *type);
    pthread_mutexattr_settype(pthread_mutexattr_t *attr, int type);
    // Type: PTHREAD_MUTEX_NORMAL PTHREAD_MUTEX_ERRORCHECK PTHREAD_MUTEX_RECURSIVE PTHREAD_MUTEX_DEFAULT
    ```

4. phtread_cond:
    ```C++
    pthread_cond_init(pthread_cond_t *cond, const pthread_condattr_t *attr);
    pthread_cond_t cond = PTHREAD_COND_INITIALIZER;
    pthread_cond_destroy(pthread_cond_t *cond);
    pthread_cond_signal(pthread_cond_t *cond);
    pthread_cond_broadcast(pthread_cond_t *cond);
    pthread_cond_wait(pthread_cond_t *cond, pthread_mutex_t *mutex); // use predicate to avoid lost wakeup and spurious wakeup
    pthread_cond_timewait(pthread_cond_t *cond, pthread_mutex_t *mutex); // cancelation point
    ```

    [Synchronization with Atomics in C++20](http://www.modernescpp.com/index.php/synchronization-with-atomics-in-c-20)
    * Condition variable has two cons:
        1. `Lost wakeup`: The phenomenon of the lost wakeup is that the sender sends its notification before the receiver gets to its wait state. The consequence is that the notification is lost. The C++ standard describes condition variables as a simultaneous synchronisation mechanism: "The condition_variable class is a synchronisation primitive that can be used to block a thread, or multiple threads at the same time, ...". So the notification gets lost, and the receiver is waiting and waiting and ... .

        2. `Spurious wakeup`: usually happen because, in between the time when the condition variable was signaled and when the waiting thread finally ran, another thread ran and changed the condition. There was a race condition between the threads, with the typical result that sometimes, the thread waking up on the condition variable runs first, winning the race, and sometimes it runs second, losing the race.

    * The predicate protects against both flaws:
        * Lost wakeup: the thread checks the predicate when it is going to sleep. If it's ture, will not go to sleep.
        * Spurious wakeup: if the predicate is false, continue waiting.
    *  The notification would be lost when the sender sends its notification before the receiver is in the wait state and does not use a predicate. Consequently, the receiver waits for something that never happens. This is a deadlock.

    * [C++ Core Guidelines: Be Aware of the Traps of Condition Variables](http://www.modernescpp.com/index.php/c-core-guidelines-be-aware-of-the-traps-of-condition-variables)

    * When you only need a one-time notification such as in the previous program, promises and futures are a better choice than condition variables. Promise and futures cannot be victims of spurious or lost wakeups.

5. pthread_rwlock:
    ```C++
    pthread_rwlock_init(pthread_rwlock_t *lock, const pthread_rwlockattr_t *attr);
    pthread_rwlock_destroy(pthread_rwlock_t *rwlock);
    pthread_rwlock_rdlock(pthread_rwlock_t *rwlock);
    pthread_rwlock_tryrdlock(phtread_rwlock_t *rwlock);
    pthread_rwlock_wrlock(pthread_rwlock_t *rwlock);
    pthread_rwlock_trywrlock(pthread_rwlock_t *rwlock);
    pthread_rwlock_unlock(pthread_rwlock_t *rwlock);
    pthread_rwlock_timed***(...);
    ```

6. pthread asynchronous:
    ```C++
    pthread_kill(pthread_t thread, int sig); // 0: test the existence of thread
    phread_sigmask(int how, const sigset_t *set, sigset_t *oldset); // sigprocmaks(...);
        how: SIG_BLOCK, SIG_UNBLOCK, SIG_SETMASK
    ```

7. Spin Block:
    ```C++
    //pshared: PTHREAD_PROCESS_SHARED(can be access by threads from different process)
    //         PTHREAD_PROCESS_PRIVATE(can be access only by threads within same process)
    int pthread_spin_init(pthread_spinlock_t *lock, int pshared);
    int pthread_spin_destroy(pthread_psinlock_t *lock);
    int pthread_spin_lock(pthread_spinlock_t *lock);    // lock the lock already locked, behavior is undefined.
    int pthread_spin_trylock(pthread_psinlock_t *lock); // if locked, return 0
        // Doesn't spin. Return EBUSY if not acquired immediately.
    int pthread_spin_unlock(pthread_spin_lock_t *lock); // unlock the lock doesn't locked, behavior is undefined.
    // A spin lock like a mutex, it blocks a process by busy-waiting(spinnig) rather that sleeping(mutex);
    // Used in situation where locks are held for a short periods of times and threads don't want to incur the cost of being descheduled.
    // They are useful in nonpreemptive kernel: providing a mutual exclusion mechanism,
    // interrupt handler can't deadlock system by trying to acquire a spin lock that is already locked.
    // At user level they aren't as useful unless running in a real-time-sharing shceduling not allow preemption
    ```

8. Barriers:
    * It's a synchronization mechanism that can be used to coordinate multiple threads working in parallel.
    * A barrier allows each thread to wait unitl all cooperrating threads have reached the same point, and then continue executing from there.

    ```C++
    int pthread_barrier_init(pthread_barrier_t *barrier, const pthread_barrierattr_t *attr, unsigned int count);
    int pthread_barrier_destroy(pthread_barrier_t *barreir);
    int pthread_barrier_wait(pthread_barrier_t *barrier);
        // The thread calling this is put to sleep if the barrier count is not yet staisfied.
        /* To one arbitary thread, it will appear as this return PTHREAD_BARRIER_SERIAL_THREAD, the remaining
        *    threads see a return value of 0. This allows this thread to continue as the master to act the results
        *    of the work done by all of the other threads.
        */
        // Once barrier count is reached and all threads is unblocked, barrier can be used aggain.
        // But barrier count can be changed.
    ```

# HTTPS Programming

## SSL:
* Consists of two layers of protocol:
    1. High-level protocol:
        * SSL HandShake Protocol: Algorithm negotiation, identity authentication, private key determination

            > categories: Full HandShake, Resume session HandShake, <Server, Client> Re-nogotiation HandShake
        * Change Cipher Spec Protocol
        * AlertProtocol
    2. Low-level protocol:
        > SSL Record Protocol(certificate information, test data is tampered or not)
        >        | type(1 byte) | version(2 byte) | length(2)byte | data | HMAC | Filling 0 | Fill length |

Connection Process: https://www.cnblogs.com/yuweifeng/p/5641280.html
1. init
    ```C++
    SSL_library_init();
    OpenSSL_add_all_algorithms();
    SSL_load_error_strings();       // void ERR_print_errors_fp(FILE *fp);
    ```
2. Specify method
    ```C++
    SSL_METHOD* method = SSLv23_server_method(void); // server method must is the same as the client
    ```
3. Set CTX
    ```C++
    SSL_CTX *ctx = SSL_CTX_new(SSL_METHOD*);
        // set the verify method
    void SSL_set_verify(SSL_CTX *, int, int *(int, X509_STORE_CTX *)); // SSL_VERIFY_<PEER, NONE>
        // load certificate
    void SSL_CTX_load_verify_locations(SSL_CTS *, const char *file, const char *path);
        // load local certificate
    int SSL_CTX_use_certificate_file(SSL_CTX *, const char *file, int type);
        // load private key
    int SSL_CTX_use_PrivateKey_file(SSL_CTX *, const char *file, int type);
        // validata private key and certificate
    BOOL SSL_CTX_check_private_key(SSL_CTX *);
    ```
4. Attach to a connected socket
    ```C++
    SSL *SSL_new(SSL_CTX *);
    int SSL_set_fd(SSL *ssl, int fd); int SSL_set_rfd(SSL *, int); int SSL_set_wfd(SSL *, int);
    ```
5. SSL connect and communication
    ```C++
    int SSL_connect(SSL *ssl); int SSL_accept(SSL *ssl);
    int SSL_read(SSL *ssl, char *buf, int len);
    int SSL_write(SSL *ssl, char *buf, int len);
    ```
6. Free
    ```C++
    int SSL_shutdown(SSL *ssl);
    void SSL_free(SSL *ssl);
    void SSL_CTX_free(SSL_CTX *);
    ```

# MYSQL Programming
1. Direct Execute:
    ```C++
    MYSQL *mysql_init(MYSQL *mysql);                        // init MYSQL struct
    MYSQL *mysql_real_connect(MYSQL *mysql, const char *host, const char *user, const char *pwd, const char *db,
        unsigned int port, const char *unix_socket, unsigned long client_flag); // connect to mysql server
    int mysql_query(MYSQL *mysql, const char *stmt_str);    // execute query
    MYSQL_RES *mysql_store_result(MYSQL *mysql);            // store query result
    unsigned int mysql_field_count(MYSQL *mysql);            // get last result column amount
    unsigned int mysq_num_field(MYSQl_RES *result);          // get result column amount
    my_ulonglong mysql_num_row(MYSQL_RES *result);          // get result row amount
    MYSQL_ROW mysql_fetch_row(MYSQL_RES *result);           // get next data from result
    void mysql_free_result(MYSQL_RES *result);              // free MYSQL_RES struct
    void mysql_close(MYSQL *connection);
    ```

2. Prepared Statment:
    ```C++
    MYSQL_STMT *mysql_stmt_init();
    mysql_stmt_prepare();       // prepare stmt on server
    mysql_stmt_bind_param();    // set the values of any parameters
    mysql_stmt_execute();
    mysql_stmt_result_metadata(); obtain the result set meta data
    mysql_stmt_bind_result();   // retrieve result set row
    mysql_stmt_fetch();         // fetch data row by row
    mysql_stmt_next_result();   // more result: -1 = no, >0 = error, 0 = yes

    unsigned int mysql_errno(MYSQL *connection);
    char *mysql_error(MYSQL *connection);
    ```

# Kernel Programming
* Advantages:
    1. Don' have to rebuild kernel as often.
    2. Help diagnose system problems.
    3. Save memory.
    4. Much faster to maintain and debug.

* Used for:
    1. Device drivers
    2. Filesystem drivers
    3. System calls
    4. Network drivers
    5. TTY line disciplines
    6. Executeable interpreters

LKM utilities:
* insmod  rmmod   depmod  kerneld  ksyms  lsmod(/proc/modules)   modinfo   modprobe

* What happen when An LKM loads:
    1. Search system for device it is know how to drive.
    2. Register itself as the driver for particular major number /proc/devices
    3. May register itself as the handler for the interrupt level that device uses / proc/interrupts
    4. May send setup commands to the device
    5. A nice device driver issuses kernel msg (/var/log/message) telling what devices if found and is prepared to drive. (dmesg can display recent kernel message)
    6. A network device driver works similarly, except that the LKM registers a device name of its choosing (eth0) rather than major number. /proc/net/dev
    7. A filesystem driver, upon loading, register itself as the driver for a filesystem type of a certain name.

* LKM procedure:
    1. create, init and add struct file_operatiosn, cdev, custom LKM object.
        ```C++
        /*-- create object --*/
        dev_t MKDEV(ma, mi); // make a device
        int register_chrdev_region (dev_t from, unsigned count, const char *name); // register a range of devide num
        int alloc_chrdev_region (dev_t *dev, unsigned baseminor,
            unsigned count, const char *name);  // register a range of char device number
        void unregister_chrdev_region(dev_t from, unsigned count); // return a range of devices number
        void *kmalloc(size t, gfp_t flags); // alloc memory for ojbects smaller than page size in kernel
        void *kzalloc(size t, gfp_t flags); // allocate memory, the memory is set to zero
            flags:  GFP_USER, GFP_KERNEL, GFP_ATOMIC, GFP_HIGHUSER, GFP_NOIO, GFP_NOFS, GFP_NOWAIT
        void kzfree(const void *);
        void *krealloc(const void *, size_t, gfp_t);

        /*-- init object --*/
        void cdev_init(struct cdev *cdev, const struct file_operations *fops);
        int cdev_add(struct cdev *p, dev_t dev, unsigned count); // make device live immediately

        // create and register device with sysfs
        struct class *class_create(struct module *owner, const char *name);
        void class_destroy(struct class *cls);
        struct device *device_create(struct class *cls, struct device *parent, dev_t devt, const char *fmg, ...);
        void device_destroy(struct class *cls, dev_t devt);

        list_del_init (struct list_head * entry);// &__this_module.list, hide module from lsmod(/proc/modules) cmd
        kobject_del(&THIS_MODULE->mkobj.kobj);   // hide module from sysfs /sys/module
        moduel_init(...);
        module_exit(...); // if a module not define exit handler this means this modules is not allowed to remove.
        MODULE_LICENSE("GPL v2");
        ```
    2. Hijack System call

* Process Managemenet:
    ```C++
    struct task_struct {

    };
    ```

* File System:
    ```C++
    struct file {
        unsigned short f_mode;      // file type and property
        unsigned short f_flags;     // file flag
        unsigned short f_count;     // file reference amount
        struct m_inode * f_inode;
        off_t f_pos;                // file_operation
    };
    1. Process to find a file:
        fd -> struct file -> struct m_inode -> i_zone -> position in the disk
    ```

* Kernel Network:
    ```C++
    struct ifreq {  //<net/if.h>
       char ifr_name[IFNAMSIZ]; /* Interface name */
       union {
           struct sockaddr ifr_addr;
           struct sockaddr ifr_dstaddr;
           struct sockaddr ifr_broadaddr;
           struct sockaddr ifr_netmask;
           struct sockaddr ifr_hwaddr;
           short           ifr_flags;
           int             ifr_ifindex;
           int             ifr_metric;
           int             ifr_mtu;
           struct ifmap    ifr_map;
           char            ifr_slave[IFNAMSIZ];
           char            ifr_newname[IFNAMSIZ];
           char           *ifr_data;
       };
   };
   TUN/TAP: virtual network device:
   ```


# Utility
```C++
<cstdarg>:
void my_err(char *msg, ...) { // my_err("Error connecting to tun/tap interface %s!\n", if_name)
    va_list argp;
    va_start(argp, msg);
    vfprintf(stderr, msg, argp);
    va_end(argp);
}
```