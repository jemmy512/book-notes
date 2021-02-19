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
