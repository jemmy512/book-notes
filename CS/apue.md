# 9. Process Relationships
## 9.3 Process Group
A process group is a collection of one or more processes, usually associated with the same job (job control is discussed in Section 9.8), that can receive signals from the same terminal.

```c++
#include <unistd.h>
pid_t getpgid(viid);
pid_t getpgid(pid_t pid);
int setpgid(pid_t pid, pid_t pgid);
```
If the two arguments are equal, the process specified by pid becomes a process group leader. If pid is 0, the process ID of the caller is used. Also, if pgid is 0, the process ID specified by pid is used as the process group ID.

A process can set the process group ID of only itself or any of its children. Furthermore, it can’t change the process group ID of one of its children after that child has called one of the exec functions.

It is possible for a process group leader to create a process group, create processes in the group, and then terminate. The process group still exists, as long as at least one process is in the group, regardless of whether the group leader terminates.

This is called the process group lifetime—the period of time that begins when the group is created and ends when the last remaining process leaves the group. The last remaining process in the process group can either terminate or enter some other process group.