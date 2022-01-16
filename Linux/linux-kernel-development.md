# 3 The Process
## 3.1 Process Descriptor and the Task Structure
### 3.1.1 Allocating the Process Descriptor
### 3.1.Storing the Process Descriptor
### 3.1.Process State
### 3.1.Manipulating the Current Process State
### 3.1.Process Context
### 3.1.The Process Family Tree

## 3.2 Process Creation
### 3.2.1 Copy-on-Write
The only overhead incurred by fork() is the duplication of the parent’s page tables and the creation of a unique process descriptor for the child.

### 3.2.2 Forking
### 3.2.3 vfork()

## 3.3 The Linux Implementation of Threads
### 3.3.1 Creating Threads
### 3.3.2 Kernel Threads

## 3.4 Process Termination
### 3.4.1 Removing the Process Descriptor
### 3.4.2 The Dilemma of the Parentless Task



# 13 The Virtual Filesystem
## The Dentry Object
The VFS often needs to perform directory-specific operations, such as path name lookup. Path name lookup involves translating each component of a path, ensuring it is valid, and following it to the next component. To facilitate this, the VFS employs the concept of a directory entry (dentry). A dentry is a specific component in a path.

Dentry objects are all components in a path, including files. Resolving a path and walking its components is a nontrivial exercise, time-consuming and heavy on string operations, which are expensive to execute and cumbersome to code.The dentry object makes the whole process easier.

Dentries might also include mount points.

The dentry object does not correspond to any sort of on-disk data structure.TheVFS creates it on-the-fly from a string representation of a path name.

### Dentry State
* Three states:
    * **used**: corresponds to a valid inode (d_inode points to an associated inode) and indicates that there are one or more users of the object (d_count is positive). A used dentry is in use by the VFS and points to valid data and, thus, cannot be discarded.
    * **unused**: corresponds to a valid inode (d_inode points to an inode), but the VFS is not currently using the dentry object (d_count is zero). Because the dentry object still points to a valid object, the dentry is kept around—cached—in case it is needed again. If it is necessary to reclaim memory, however, the dentry can be discarded because it is not in active use.
    * **negative**: is not associated with a valid inode (d_inode is NULL) because either the inode was deleted or the path name was never correct to begin with.The dentry is kept around, however, so that future lookups are resolved quickly.
