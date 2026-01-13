
```sh
                +---------------------------+
                |     struct bus_type       |
                +---------------------------+
                    |                      |
            match/probe/remove        drivers list
                    |                      |
                    v                      v
    +-------------------+    +-----------------------+
    |   struct device   |<-->|   struct device_driver|
    +-------------------+    +-----------------------+
    | parent                 | probe/remove
    | kobject                | bus
    | bus -------------------+
    | driver
    | class
    v
    +-------------------+
    |    struct class   |
    +-------------------+
    | Groups devices
    | (e.g. block, net,
    | input, tty, etc.)
    v
    /sys/class/<class>/...
```

* [Linux 设备驱动模型 device-bus-driver](https://mp.weixin.qq.com/s/09VGwzEa4ndc8urd5RfJxA)

# core

```c
struct device {
    struct kobject              kobj;
    struct device               *parent;

    struct device_private       *p;

    const char                  *init_name; /* initial name of the device */
    const struct device_type    *type;

    const struct bus_type       *bus; /* type of bus device is on */
    struct device_driver        *driver;
    void                        *platform_data;
    void                        *driver_data;
    struct mutex                mutex;

    struct dev_links_info       links;
    struct dev_pm_info          power;
    struct dev_pm_domain        *pm_domain;
};

struct device_type {
    const char *name;
    const struct attribute_group **groups;
    int (*uevent)(const struct device *dev, struct kobj_uevent_env *env);
    char *(*devnode)(const struct device *dev, umode_t *mode, kuid_t *uid, kgid_t *gid);
    void (*release)(struct device *dev);

    const struct dev_pm_ops *pm;
};

struct device_driver {
    const char              *name;
    const struct bus_type   *bus;

    struct module           *owner;
    const char              *mod_name;    /* used for built-in modules */

    bool suppress_bind_attrs;    /* disables bind/unbind via sysfs */
    enum probe_type probe_type;

    const struct of_device_id       *of_match_table;
    const struct acpi_device_id     *acpi_match_table;

    int (*probe) (struct device *dev);
    void (*sync_state)(struct device *dev);
    int (*remove) (struct device *dev);
    void (*shutdown) (struct device *dev);
    int (*suspend) (struct device *dev, pm_message_t state);
    int (*resume) (struct device *dev);
    const struct attribute_group **groups;
    const struct attribute_group **dev_groups;

    const struct dev_pm_ops *pm;
    void (*coredump) (struct device *dev);

    struct driver_private *p;
};

struct of_device_id {
    char        name[32];
    char        type[32];
    char        compatible[128];
    const void  *data;
};
```

## mknode

## device_add

## requset_mem_region

## relese_mem_region

## ioremap

## iounmap

## readl

## writel

## remap_pfn_range


# bus

```c
/* PCI, USB, I2C, platform
 * glues devices and drivers together */
struct bus_type {
    const char        *name;
    const char        *dev_name;
    const struct attribute_group **bus_groups;
    const struct attribute_group **dev_groups;
    const struct attribute_group **drv_groups;

    int (*match)(struct device *dev, const struct device_driver *drv);
    int (*uevent)(const struct device *dev, struct kobj_uevent_env *env);
    int (*probe)(struct device *dev);
    void (*sync_state)(struct device *dev);
    void (*remove)(struct device *dev);
    void (*shutdown)(struct device *dev);
    const struct cpumask *(*irq_get_affinity)(struct device *dev, unsigned int irq_vec);

    int (*online)(struct device *dev);
    int (*offline)(struct device *dev);

    int (*suspend)(struct device *dev, pm_message_t state);
    int (*resume)(struct device *dev);

    int (*num_vf)(struct device *dev);

    int (*dma_configure)(struct device *dev);
    void (*dma_cleanup)(struct device *dev);

    const struct dev_pm_ops *pm;

    bool need_parent_lock;
};

struct attribute_group {
    const char  *name;
    umode_t (*is_visible)(struct kobject *, struct attribute *, int);
    umode_t (*is_bin_visible)(struct kobject *, const struct bin_attribute *, int);
    size_t  (*bin_size)(struct kobject *, const struct bin_attribute *, int);
    struct attribute    **attrs;
    union {
        const struct bin_attribute    *const *bin_attrs;
        const struct bin_attribute    *const *bin_attrs_new;
    };
};

struct bin_attribute {
    struct attribute    attr;
    size_t              size;
    void                *private;
    struct address_space *(*f_mapping)(void);
    ssize_t (*read)(struct file *, struct kobject *, const struct bin_attribute *, char *, loff_t, size_t);
    ssize_t (*read_new)(struct file *, struct kobject *, const struct bin_attribute *, char *, loff_t, size_t);
    ssize_t (*write)(struct file *, struct kobject *, const struct bin_attribute *, char *, loff_t, size_t);
    ssize_t (*write_new)(struct file *, struct kobject *, const struct bin_attribute *, char *, loff_t, size_t);
    loff_t (*llseek)(struct file *, struct kobject *, const struct bin_attribute *, loff_t, int);
    int     (*mmap)(struct file *, struct kobject *, const struct bin_attribute *attr, struct vm_area_struct *vma);
};

struct pci_dev {
    struct device dev;   // embedded generic device
    ...
};

struct usb_device {
    struct device dev;
    ...
};

struct platform_device {
    struct device dev;
    ...
};
```

## bus_register

```c
struct subsys_private {
    struct kset subsys;
    struct kset *devices_kset;
    struct list_head interfaces;
    struct mutex mutex;

    struct kset *drivers_kset;
    struct klist klist_devices;
    struct klist klist_drivers;
    struct blocking_notifier_head bus_notifier;
    unsigned int drivers_autoprobe:1;
    const struct bus_type *bus;
    struct device *dev_root;

    struct kset glue_dirs;
    const struct class *class;

    struct lock_class_key lock_key;
};
```

```c
int bus_register(const struct bus_type *bus)
{
    int retval;
    struct subsys_private *priv;
    struct kobject *bus_kobj;
    struct lock_class_key *key;

    priv = kzalloc(sizeof(struct subsys_private), GFP_KERNEL);
    if (!priv)
        return -ENOMEM;

    priv->bus = bus;

    BLOCKING_INIT_NOTIFIER_HEAD(&priv->bus_notifier);

    bus_kobj = &priv->subsys.kobj;
    retval = kobject_set_name(bus_kobj, "%s", bus->name);
    if (retval)
        goto out;

    bus_kobj->kset = bus_kset;
    bus_kobj->ktype = &bus_ktype;
    priv->drivers_autoprobe = 1;

    retval = kset_register(&priv->subsys);
    if (retval)
        goto out;

    retval = bus_create_file(bus, &bus_attr_uevent);
    if (retval)
        goto bus_uevent_fail;

    priv->devices_kset = kset_create_and_add("devices", NULL, bus_kobj);
    if (!priv->devices_kset) {
        retval = -ENOMEM;
        goto bus_devices_fail;
    }

    priv->drivers_kset = kset_create_and_add("drivers", NULL, bus_kobj);
    if (!priv->drivers_kset) {
        retval = -ENOMEM;
        goto bus_drivers_fail;
    }

    INIT_LIST_HEAD(&priv->interfaces);
    key = &priv->lock_key;
    lockdep_register_key(key);
    __mutex_init(&priv->mutex, "subsys mutex", key);
    klist_init(&priv->klist_devices, klist_devices_get, klist_devices_put);
    klist_init(&priv->klist_drivers, NULL, NULL);

    retval = add_probe_files(bus) {
        int retval;

        retval = bus_create_file(bus, &bus_attr_drivers_probe);
        if (retval)
            goto out;

        retval = bus_create_file(bus, &bus_attr_drivers_autoprobe);
        if (retval)
            bus_remove_file(bus, &bus_attr_drivers_probe);
    out:
        return retval;
    }
    if (retval)
        goto bus_probe_files_fail;

    retval = sysfs_create_groups(bus_kobj, bus->bus_groups) {
        return internal_create_groups(kobj, 0, groups) {
            int error = 0;
            int i;

            if (!groups)
                return 0;

            for (i = 0; groups[i]; i++) {
                error = internal_create_group(kobj, update, groups[i]) {
                    struct kernfs_node *kn;
                    kuid_t uid;
                    kgid_t gid;
                    int error;

                    if (WARN_ON(!kobj || (!update && !kobj->sd)))
                        return -EINVAL;

                    /* Updates may happen before the object has been instantiated */
                    if (unlikely(update && !kobj->sd))
                        return -EINVAL;

                    if (!grp->attrs && !grp->bin_attrs) {
                        pr_debug("sysfs: (bin_)attrs not set by subsystem for group: %s/%s, skipping\n",
                            kobj->name, grp->name ?: "");
                        return 0;
                    }

                    kobject_get_ownership(kobj, &uid, &gid);
                    if (grp->name) {
                        umode_t mode = __first_visible(grp, kobj);

                        if (mode & SYSFS_GROUP_INVISIBLE)
                            mode = 0;
                        else
                            mode = S_IRWXU | S_IRUGO | S_IXUGO;

                        if (update) {
                            kn = kernfs_find_and_get(kobj->sd, grp->name);
                            if (!kn) {
                                pr_debug("attr grp %s/%s not created yet\n",
                                    kobj->name, grp->name);
                                /* may have been invisible prior to this update */
                                update = 0;
                            } else if (!mode) {
                                sysfs_remove_group(kobj, grp);
                                kernfs_put(kn);
                                return 0;
                            }
                        }

                        if (!update) {
                            if (!mode)
                                return 0;
                            kn = kernfs_create_dir_ns(kobj->sd, grp->name, mode,
                                        uid, gid, kobj, NULL);
                            if (IS_ERR(kn)) {
                                if (PTR_ERR(kn) == -EEXIST)
                                    sysfs_warn_dup(kobj->sd, grp->name);
                                return PTR_ERR(kn);
                            }
                        }
                    } else {
                        kn = kobj->sd;
                    }

                    kernfs_get(kn);
                    error = create_files(kn, kobj, uid, gid, grp, update);
                    if (error) {
                        if (grp->name)
                            kernfs_remove(kn);
                    }
                    kernfs_put(kn);

                    if (grp->name && update)
                        kernfs_put(kn);

                    return error;
                }
                if (error) {
                    while (--i >= 0)
                        sysfs_remove_group(kobj, groups[i]);
                    break;
                }
            }
            return error;
        }
    }
    if (retval)
        goto bus_groups_fail;

    pr_debug("bus: '%s': registered\n", bus->name);
    return 0;
}
```

# platform

```c
struct platform_device {
    const char      *name;
    int             id;
    bool            id_auto;
    struct device   dev;
    u64             platform_dma_mask;
    struct device_dma_parameters    dma_parms;
    u32                 num_resources;
    struct resource     *resource;

    const struct platform_device_id    *id_entry;
    /* Driver name to force a match.  Do not set directly, because core
     * frees it.  Use driver_set_override() to set or clear it. */
    const char          *driver_override;

    /* MFD cell pointer */
    struct mfd_cell     *mfd_cell;

    /* arch specific additions */
    struct pdev_archdata    archdata;
};

struct resource {
    resource_size_t start;
    resource_size_t end;
    const char      *name;
    unsigned long   flags;
    unsigned long   desc;
    struct resource *parent, *sibling, *child;
};

struct platform_driver {
    int (*probe)(struct platform_device *);
    void (*remove)(struct platform_device *);
    void (*shutdown)(struct platform_device *);
    int (*suspend)(struct platform_device *, pm_message_t state);
    int (*resume)(struct platform_device *);
    struct device_driver driver;
    const struct platform_device_id *id_table;
    bool prevent_deferred_probe;
    /* For most device drivers, no need to care about this flag as long as
     * all DMAs are handled through the kernel DMA API. For some special
     * ones, for example VFIO drivers, they know how to manage the DMA
     * themselves and set this flag so that the IOMMU layer will allow them
     * to setup and manage their own I/O address space. */
    bool driver_managed_dma;
};

const struct bus_type platform_bus_type = {
    .name           = "platform",
    .dev_groups     = platform_dev_groups,
    .match          = platform_match,
    .uevent         = platform_uevent,
    .probe          = platform_probe,
    .remove         = platform_remove,
    .shutdown       = platform_shutdown,
    .dma_configure  = platform_dma_configure,
    .dma_cleanup    = platform_dma_cleanup,
    .pm             = &platform_dev_pm_ops,
};
```

## platform_add_devices

```c
int platform_add_devices(struct platform_device **devs, int num)
{
    int i, ret = 0;

    for (i = 0; i < num; i++) {
        ret = platform_device_register(devs[i]);
        if (ret) {
            while (--i >= 0)
                platform_device_unregister(devs[i]);
            break;
        }
    }

    return ret;
}

int platform_device_register(struct platform_device *pdev)
{
    device_initialize(&pdev->dev) {
        dev->kobj.kset = devices_kset;
        kobject_init(&dev->kobj, &device_ktype);
        INIT_LIST_HEAD(&dev->dma_pools);
        mutex_init(&dev->mutex);
        lockdep_set_novalidate_class(&dev->mutex);
        spin_lock_init(&dev->devres_lock);
        INIT_LIST_HEAD(&dev->devres_head);
        device_pm_init(dev);
        set_dev_node(dev, NUMA_NO_NODE);
        INIT_LIST_HEAD(&dev->links.consumers);
        INIT_LIST_HEAD(&dev->links.suppliers);
        INIT_LIST_HEAD(&dev->links.defer_sync);
        dev->links.status = DL_DEV_NO_DRIVER;
    #if defined(CONFIG_ARCH_HAS_SYNC_DMA_FOR_DEVICE) || \
        defined(CONFIG_ARCH_HAS_SYNC_DMA_FOR_CPU) || \
        defined(CONFIG_ARCH_HAS_SYNC_DMA_FOR_CPU_ALL)
        dev->dma_coherent = dma_default_coherent;
    #endif
        swiotlb_dev_init(dev);
    }

    setup_pdev_dma_masks(pdev) {
        pdev->dev.dma_parms = &pdev->dma_parms;

        if (!pdev->dev.coherent_dma_mask)
            pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
        if (!pdev->dev.dma_mask) {
            pdev->platform_dma_mask = DMA_BIT_MASK(32);
            pdev->dev.dma_mask = &pdev->platform_dma_mask;
        }
    }

    return platform_device_add(pdev) {
        struct device *dev = &pdev->dev;
        u32 i;
        int ret;

        if (!dev->parent)
            dev->parent = &platform_bus;

        dev->bus = &platform_bus_type;

        switch (pdev->id) {
        default:
            dev_set_name(dev, "%s.%d", pdev->name,  pdev->id);
            break;
        case PLATFORM_DEVID_NONE:
            dev_set_name(dev, "%s", pdev->name);
            break;
        case PLATFORM_DEVID_AUTO:
            /* Automatically allocated device ID. We mark it as such so
            * that we remember it must be freed, and we append a suffix
            * to avoid namespace collision with explicit IDs. */
            ret = ida_alloc(&platform_devid_ida, GFP_KERNEL);
            if (ret < 0)
                return ret;
            pdev->id = ret;
            pdev->id_auto = true;
            dev_set_name(dev, "%s.%d.auto", pdev->name, pdev->id);
            break;
        }

        for (i = 0; i < pdev->num_resources; i++) {
            struct resource *p, *r = &pdev->resource[i];

            if (r->name == NULL)
                r->name = dev_name(dev);

            p = r->parent;
            if (!p) {
                if (resource_type(r) == IORESOURCE_MEM)
                    p = &iomem_resource;
                else if (resource_type(r) == IORESOURCE_IO)
                    p = &ioport_resource;
            }

            if (p) {
                ret = insert_resource(p, r) {
                    struct resource *conflict;

                    conflict = insert_resource_conflict(parent, new);
                    return conflict ? -EBUSY : 0;
                }
                if (ret) {
                    dev_err(dev, "failed to claim resource %d: %pR\n", i, r);
                    goto failed;
                }
            }
        }

        pr_debug("Registering platform device '%s'. Parent at %s\n", dev_name(dev),
            dev_name(dev->parent));

        ret = device_add(dev);
        if (ret)
            goto failed;

        return 0;

    failed:
        if (pdev->id_auto) {
            ida_free(&platform_devid_ida, pdev->id);
            pdev->id = PLATFORM_DEVID_AUTO;
        }

        while (i--) {
            struct resource *r = &pdev->resource[i];
            if (r->parent)
                release_resource(r);
        }

        return ret;
    }
}
```

# cdev

```c
struct cdev {
    struct kobject                  kobj;
    struct module                   *owner;
    const struct file_operations    *ops;
    struct list_head                list;
    dev_t                           dev;
    unsigned int                    count;
};
```

```c
struct xxx_dev_t {
    struct cdev             cdev;
    struct file_operation   *fops;
} xxx_dev;

static int __init xxx_init(void)
{
    cdev_init(&xxx_dev.cdev, &xxx_dev.fops);
    xxx_dev.cdev.owner = THIS_MODULE;

    if (xxx_major) {
        register_chrdev_region(xxx_dev_no, 1, DEV_NAME);
    } else {
        alloc_chrdev_region(&xxx_dev.no, 0, 1, DEV_NAME);
    }

    ret = cdev_add(&xxx_dev, xxx_dev_no, 1);

    return ret;
}

static void __exit xxx_exit(void)
{
    unregister_chrdev_region(xxx_dev_no, 1);
    cdev_del(&xxx_dev.cdev);
}
```

## register_chrdev

## register_chrdev_region

## alloc_chrdev_region

## cdev_init

## cdev_add

# block

```c
struct gendisk {
    /* major/first_minor/minors should not be set by any new driver, the
     * block core will take care of allocating them automatically. */
    int                 major;
    int                 first_minor;
    int                 minors;

    char disk_name[DISK_NAME_LEN];  /* name of major driver */

    unsigned short      events; /* supported events */
    unsigned short      event_flags;    /* flags related to event processing */

    struct xarray part_tbl;
    struct block_device *part0;

    const struct block_device_operations *fops;
    struct request_queue *queue;
    void *private_data;

    struct bio_set bio_split;

    int flags;
    unsigned long state;
#define GD_NEED_PART_SCAN       0
#define GD_READ_ONLY            1
#define GD_DEAD                 2
#define GD_NATIVE_CAPACITY      3
#define GD_ADDED                4
#define GD_SUPPRESS_PART_SCAN   5
#define GD_OWNS_QUEUE           6

    struct mutex    open_mutex; /* open/close mutex */
    unsigned        open_partitions;    /* number of open partitions */

    struct backing_dev_info *bdi;
    struct kobject          queue_kobj; /* the queue/ directory */
    struct kobject          *slave_dir;
#ifdef CONFIG_BLOCK_HOLDER_DEPRECATED
    struct list_head slave_bdevs;
#endif
    struct timer_rand_state     *random;
    struct disk_events          *ev;

#ifdef CONFIG_BLK_DEV_ZONED
    /* Zoned block device information. Reads of this information must be
     * protected with blk_queue_enter() / blk_queue_exit(). Modifying this
     * information is only allowed while no requests are being processed.
     * See also blk_mq_freeze_queue() and blk_mq_unfreeze_queue(). */
    unsigned int        nr_zones;
    unsigned int        zone_capacity;
    unsigned int        last_zone_capacity;
    unsigned long __rcu    *conv_zones_bitmap;
    unsigned int        zone_wplugs_hash_bits;
    atomic_t        nr_zone_wplugs;
    spinlock_t        zone_wplugs_lock;
    struct mempool_s    *zone_wplugs_pool;
    struct hlist_head    *zone_wplugs_hash;
    struct workqueue_struct *zone_wplugs_wq;
#endif /* CONFIG_BLK_DEV_ZONED */

#if IS_ENABLED(CONFIG_CDROM)
    struct cdrom_device_info *cdi;
#endif
    int                 node_id;
    struct badblocks    *bb;
    struct lockdep_map  lockdep_map;
    u64                 diskseq;
    blk_mode_t          open_mode;

    /* Independent sector access ranges. This is always NULL for
     * devices that do not have multiple independent access ranges. */
    struct blk_independent_access_ranges *ia_ranges;

    struct mutex rqos_state_mutex;    /* rqos state change mutex */
};
```

# DMA

## dma_alloc_coherent

## dma_map_single

## dma_sync_single_for_cpu

## dma_request_slave_channel

##

# MSI

## pci_enable_msi

```c
int pci_enable_msi(struct pci_dev *dev)
{
    int rc = __pci_enable_msi_range(dev, 1, 1, NULL) {

    }
    if (rc < 0)
        return rc;
    return 0;
}
```

# thermal

[WOWO tech :one:](http://www.wowotech.net/pm_subsystem/519.html) ⊙ [:two:](http://www.wowotech.net/pm_subsystem/520.html) ⊙ [:three:](http://www.wowotech.net/pm_subsystem/521.html) ⊙ [:four:](http://www.wowotech.net/pm_subsystem/522.html) ⊙ [:five:](http://www.wowotech.net/pm_subsystem/523.html)

# module

* [ELF format Checksheet](https://gist.github.com/x0nu11byt3/bcb35c3de461e5fb66173071a2379779)

```c
LIST_HEAD(modules);
```

## init_module

```c
SYSCALL_DEFINE3(init_module, void __user *, umod,
        unsigned long, len, const char __user *, uargs)
{
    int err;
    struct load_info info = { };

    err = may_init_module() {
        if (!capable(CAP_SYS_MODULE) || modules_disabled)
            return -EPERM;

        return 0;
    }
    if (err)
        return err;

    pr_debug("init_module: umod=%p, len=%lu, uargs=%p\n",
           umod, len, uargs);

    err = copy_module_from_user(umod, len, &info);
    if (err) {
        mod_stat_inc(&failed_kreads);
        mod_stat_add_long(len, &invalid_kread_bytes);
        return err;
    }

    return load_module(&info, uargs, 0);
}

int load_module(struct load_info *info, const char __user *uargs,
               int flags)
{
    struct module *mod;
    bool module_allocated = false;
    long err = 0;
    char *after_dashes;

    /* Do the signature check (if any) first. All that
     * the signature check needs is info->len, it does
     * not need any of the section info. That can be
     * set up later. This will minimize the chances
     * of a corrupt module causing problems before
     * we even get to the signature check.
     *
     * The check will also adjust info->len by stripping
     * off the sig length at the end of the module, making
     * checks against info->len more correct. */
    err = module_sig_check(info, flags);
    if (err)
        goto free_copy;

    /* Do basic sanity checks against the ELF header and
     * sections. Cache useful sections and set the
     * info->mod to the userspace passed struct module. */
    err = elf_validity_cache_copy(info, flags);
    if (err)
        goto free_copy;

    err = early_mod_check(info, flags) {
        int err;

        /* Now that we know we have the correct module name, check
        * if it's blacklisted. */
        if (blacklisted(info->name)) {
            pr_err("Module %s is blacklisted\n", info->name);
            return -EPERM;
        }

        err = rewrite_section_headers(info, flags) {
            unsigned int i;

            /* This should always be true, but let's be sure. */
            info->sechdrs[0].sh_addr = 0;

            for (i = 1; i < info->hdr->e_shnum; i++) {
                Elf_Shdr *shdr = &info->sechdrs[i];

                /* Mark all sections sh_addr with their address in the
                * temporary image. */
                shdr->sh_addr = (size_t)info->hdr + shdr->sh_offset;

            }

            /* Track but don't keep modinfo and version sections. */
            info->sechdrs[info->index.vers].sh_flags &= ~(unsigned long)SHF_ALLOC;
            info->sechdrs[info->index.vers_ext_crc].sh_flags &=
                ~(unsigned long)SHF_ALLOC;
            info->sechdrs[info->index.vers_ext_name].sh_flags &=
                ~(unsigned long)SHF_ALLOC;
            info->sechdrs[info->index.info].sh_flags &= ~(unsigned long)SHF_ALLOC;

            return 0;
        }
        if (err)
            return err;

        /* Check module struct version now, before we try to use module. */
        ret = check_modstruct_version(info, info->mod) {
            struct find_symbol_arg fsa = {
                .name   = "module_layout",
                .gplok  = true,
            };
            bool have_symbol;

            /* Since this should be found in kernel (which can't be removed), no
            * locking is necessary. Regardless use a RCU read section to keep
            * lockdep happy. */
            scoped_guard(rcu)
                have_symbol = find_symbol(&fsa);
            BUG_ON(!have_symbol);

            return check_version(info, "module_layout", mod, fsa.crc);
        }
        if (!ret)
            return -ENOEXEC;

        err = check_modinfo(info->mod, info, flags) {
            const char *modmagic = get_modinfo(info, "vermagic");
            int err;

            if (flags & MODULE_INIT_IGNORE_VERMAGIC)
                modmagic = NULL;

            /* This is allowed: modprobe --force will invalidate it. */
            if (!modmagic) {
                err = try_to_force_load(mod, "bad vermagic");
                if (err)
                    return err;
            } else if (!same_magic(modmagic, vermagic, info->index.vers)) {
                pr_err("%s: version magic '%s' should be '%s'\n",
                    info->name, modmagic, vermagic);
                return -ENOEXEC;
            }

            err = check_modinfo_livepatch(mod, info);
            if (err)
                return err;

            return 0;
        }
        if (err)
            return err;

        mutex_lock(&module_mutex);
        err = module_patient_check_exists(info->mod->name, FAIL_DUP_MOD_BECOMING);
        mutex_unlock(&module_mutex);

        return err;
    }
    if (err)
        goto free_copy;

    /* Figure out module layout, and allocate all the memory. */
    mod = layout_and_allocate(info, flags);
    if (IS_ERR(mod)) {
        err = PTR_ERR(mod);
        goto free_copy;
    }

    module_allocated = true;

    audit_log_kern_module(info->name);

    /* Reserve our place in the list. */
    err = add_unformed_module(mod);
    if (err)
        goto free_module;

    /* We are tainting your kernel if your module gets into
     * the modules linked list somehow. */
    module_augment_kernel_taints(mod, info);

    /* To avoid stressing percpu allocator, do this once we're unique. */
    err = percpu_modalloc(mod, info);
    if (err)
        goto unlink_mod;

    /* Now module is in final location, initialize linked lists, etc. */
    err = module_unload_init(mod);
    if (err)
        goto unlink_mod;

    init_param_lock(mod);

    /* Now we've got everything in the final locations, we can
     * find optional sections. */
    err = find_module_sections(mod, info);
    if (err)
        goto free_unload;

    err = check_export_symbol_versions(mod);
    if (err)
        goto free_unload;

    /* Set up MODINFO_ATTR fields */
    err = setup_modinfo(mod, info);
    if (err)
        goto free_modinfo;

    /* Fix up syms, so that st_value is a pointer to location. */
    err = simplify_symbols(mod, info);
    if (err < 0)
        goto free_modinfo;

    err = apply_relocations(mod, info);
    if (err < 0)
        goto free_modinfo;

    err = post_relocation(mod, info);
    if (err < 0)
        goto free_modinfo;

    flush_module_icache(mod);

    /* Now copy in args */
    mod->args = strndup_user(uargs, ~0UL >> 1);
    if (IS_ERR(mod->args)) {
        err = PTR_ERR(mod->args);
        goto free_arch_cleanup;
    }

    init_build_id(mod, info);

    /* Ftrace init must be called in the MODULE_STATE_UNFORMED state */
    ftrace_module_init(mod);

    /* Finally it's fully formed, ready to start executing. */
    err = complete_formation(mod, info);
    if (err)
        goto ddebug_cleanup;

    err = prepare_coming_module(mod);
    if (err)
        goto bug_cleanup;

    mod->async_probe_requested = async_probe;

    /* Module is ready to execute: parsing args may do that. */
    after_dashes = parse_args(mod->name, mod->args, mod->kp, mod->num_kp,
                  -32768, 32767, mod,
                  unknown_module_param_cb);
    if (IS_ERR(after_dashes)) {
        err = PTR_ERR(after_dashes);
        goto coming_cleanup;
    } else if (after_dashes) {
        pr_warn("%s: parameters '%s' after `--' ignored\n",
               mod->name, after_dashes);
    }

    /* Link in to sysfs. */
    err = mod_sysfs_setup(mod, info, mod->kp, mod->num_kp);
    if (err < 0)
        goto coming_cleanup;

    if (is_livepatch_module(mod)) {
        err = copy_module_elf(mod, info);
        if (err < 0)
            goto sysfs_cleanup;
    }

    if (codetag_load_module(mod))
        goto sysfs_cleanup;

    /* Get rid of temporary copy. */
    free_copy(info, flags);

    /* Done! */
    trace_module_load(mod);

    return do_init_module(mod);

 sysfs_cleanup:
    mod_sysfs_teardown(mod);
 coming_cleanup:
    mod->state = MODULE_STATE_GOING;
    destroy_params(mod->kp, mod->num_kp);
    blocking_notifier_call_chain(&module_notify_list,
                     MODULE_STATE_GOING, mod);
    klp_module_going(mod);
 bug_cleanup:
    mod->state = MODULE_STATE_GOING;
    /* module_bug_cleanup needs module_mutex protection */
    mutex_lock(&module_mutex);
    module_bug_cleanup(mod);
    mutex_unlock(&module_mutex);

 ddebug_cleanup:
    ftrace_release_mod(mod);
    synchronize_rcu();
    kfree(mod->args);
 free_arch_cleanup:
    module_arch_cleanup(mod);
 free_modinfo:
    free_modinfo(mod);
 free_unload:
    module_unload_free(mod);
 unlink_mod:
    mutex_lock(&module_mutex);
    /* Unlink carefully: kallsyms could be walking list. */
    list_del_rcu(&mod->list);
    mod_tree_remove(mod);
    wake_up_all(&module_wq);
    /* Wait for RCU-sched synchronizing before releasing mod->list. */
    synchronize_rcu();
    mutex_unlock(&module_mutex);
 free_module:
    mod_stat_bump_invalid(info, flags);
    /* Free lock-classes; relies on the preceding sync_rcu() */
    for_class_mod_mem_type(type, core_data) {
        lockdep_free_key_range(mod->mem[type].base,
                       mod->mem[type].size);
    }

    module_memory_restore_rox(mod);
    module_deallocate(mod, info);
 free_copy:
    /* The info->len is always set. We distinguish between
     * failures once the proper module was allocated and
     * before that. */
    if (!module_allocated) {
        audit_log_kern_module(info->name ? info->name : "?");
        mod_stat_bump_becoming(info, flags);
    }
    free_copy(info, flags);
    return err;
}
```

### apply_relocations

```c
int apply_relocations(struct module *mod, const struct load_info *info)
{
    unsigned int i;
    int err = 0;

    /* Now do relocations. */
    for (i = 1; i < info->hdr->e_shnum; i++) {
        unsigned int infosec = info->sechdrs[i].sh_info;

        /* Not a valid relocation section? */
        if (infosec >= info->hdr->e_shnum)
            continue;

        /* Don't bother with non-allocated sections.
         * An exception is the percpu section, which has separate allocations
         * for individual CPUs. We relocate the percpu section in the initial
         * ELF template and subsequently copy it to the per-CPU destinations. */
        if (!(info->sechdrs[infosec].sh_flags & SHF_ALLOC) &&
            (!infosec || infosec != info->index.pcpu))
            continue;

        if (info->sechdrs[i].sh_flags & SHF_RELA_LIVEPATCH)
            err = klp_apply_section_relocs(mod, info->sechdrs,
                               info->secstrings,
                               info->strtab,
                               info->index.sym, i,
                               NULL);
        else if (info->sechdrs[i].sh_type == SHT_REL)
            err = apply_relocate(info->sechdrs, info->strtab, info->index.sym, i, mod);
        else if (info->sechdrs[i].sh_type == SHT_RELA)
            err = apply_relocate_add(info->sechdrs, info->strtab, info->index.sym, i, mod);
        if (err < 0)
            break;
    }
    return err;
}

int apply_relocate_add(Elf64_Shdr *sechdrs,
               const char *strtab,
               unsigned int symindex,
               unsigned int relsec,
               struct module *me)
{
    unsigned int i;
    int ovf;
    bool overflow_check;
    Elf64_Sym *sym;
    void *loc;
    u64 val;
    Elf64_Rela *rel = (void *)sechdrs[relsec].sh_addr;

    for (i = 0; i < sechdrs[relsec].sh_size / sizeof(*rel); i++) {
        /* loc corresponds to P in the AArch64 ELF document. */
        loc = (void *)sechdrs[sechdrs[relsec].sh_info].sh_addr
            + rel[i].r_offset;

        /* sym is the ELF symbol we're referring to. */
        sym = (Elf64_Sym *)sechdrs[symindex].sh_addr
            + ELF64_R_SYM(rel[i].r_info);

        /* val corresponds to (S + A) in the AArch64 ELF document. */
        val = sym->st_value + rel[i].r_addend;

        /* Check for overflow by default. */
        overflow_check = true;

        /* Perform the static relocation. */
        switch (ELF64_R_TYPE(rel[i].r_info)) {
        /* Null relocations. */
        case R_ARM_NONE:
        case R_AARCH64_NONE:
            ovf = 0;
            break;

        /* Data relocations. */
        case R_AARCH64_ABS64:
            overflow_check = false;
            ovf = reloc_data(RELOC_OP_ABS, loc, val, 64, me);
            break;
        case R_AARCH64_ABS32:
            ovf = reloc_data(RELOC_OP_ABS, loc, val, 32, me);
            break;
        case R_AARCH64_ABS16:
            ovf = reloc_data(RELOC_OP_ABS, loc, val, 16, me);
            break;
        case R_AARCH64_PREL64:
            overflow_check = false;
            ovf = reloc_data(RELOC_OP_PREL, loc, val, 64, me);
            break;
        case R_AARCH64_PREL32:
            ovf = reloc_data(RELOC_OP_PREL, loc, val, 32, me);
            break;
        case R_AARCH64_PREL16:
            ovf = reloc_data(RELOC_OP_PREL, loc, val, 16, me);
            break;

        /* MOVW instruction relocations. */
        case R_AARCH64_MOVW_UABS_G0_NC:
            overflow_check = false;
            fallthrough;
        case R_AARCH64_MOVW_UABS_G0:
            ovf = reloc_insn_movw(RELOC_OP_ABS, loc, val, 0,
                          AARCH64_INSN_IMM_MOVKZ, me);
            break;
        case R_AARCH64_MOVW_UABS_G1_NC:
            overflow_check = false;
            fallthrough;
        case R_AARCH64_MOVW_UABS_G1:
            ovf = reloc_insn_movw(RELOC_OP_ABS, loc, val, 16,
                          AARCH64_INSN_IMM_MOVKZ, me);
            break;
        case R_AARCH64_MOVW_UABS_G2_NC:
            overflow_check = false;
            fallthrough;
        case R_AARCH64_MOVW_UABS_G2:
            ovf = reloc_insn_movw(RELOC_OP_ABS, loc, val, 32,
                          AARCH64_INSN_IMM_MOVKZ, me);
            break;
        case R_AARCH64_MOVW_UABS_G3:
            /* We're using the top bits so we can't overflow. */
            overflow_check = false;
            ovf = reloc_insn_movw(RELOC_OP_ABS, loc, val, 48,
                          AARCH64_INSN_IMM_MOVKZ, me);
            break;
        case R_AARCH64_MOVW_SABS_G0:
            ovf = reloc_insn_movw(RELOC_OP_ABS, loc, val, 0,
                          AARCH64_INSN_IMM_MOVNZ, me);
            break;
        case R_AARCH64_MOVW_SABS_G1:
            ovf = reloc_insn_movw(RELOC_OP_ABS, loc, val, 16,
                          AARCH64_INSN_IMM_MOVNZ, me);
            break;
        case R_AARCH64_MOVW_SABS_G2:
            ovf = reloc_insn_movw(RELOC_OP_ABS, loc, val, 32,
                          AARCH64_INSN_IMM_MOVNZ, me);
            break;
        case R_AARCH64_MOVW_PREL_G0_NC:
            overflow_check = false;
            ovf = reloc_insn_movw(RELOC_OP_PREL, loc, val, 0,
                          AARCH64_INSN_IMM_MOVKZ, me);
            break;
        case R_AARCH64_MOVW_PREL_G0:
            ovf = reloc_insn_movw(RELOC_OP_PREL, loc, val, 0,
                          AARCH64_INSN_IMM_MOVNZ, me);
            break;
        case R_AARCH64_MOVW_PREL_G1_NC:
            overflow_check = false;
            ovf = reloc_insn_movw(RELOC_OP_PREL, loc, val, 16,
                          AARCH64_INSN_IMM_MOVKZ, me);
            break;
        case R_AARCH64_MOVW_PREL_G1:
            ovf = reloc_insn_movw(RELOC_OP_PREL, loc, val, 16,
                          AARCH64_INSN_IMM_MOVNZ, me);
            break;
        case R_AARCH64_MOVW_PREL_G2_NC:
            overflow_check = false;
            ovf = reloc_insn_movw(RELOC_OP_PREL, loc, val, 32,
                          AARCH64_INSN_IMM_MOVKZ, me);
            break;
        case R_AARCH64_MOVW_PREL_G2:
            ovf = reloc_insn_movw(RELOC_OP_PREL, loc, val, 32,
                          AARCH64_INSN_IMM_MOVNZ, me);
            break;
        case R_AARCH64_MOVW_PREL_G3:
            /* We're using the top bits so we can't overflow. */
            overflow_check = false;
            ovf = reloc_insn_movw(RELOC_OP_PREL, loc, val, 48,
                          AARCH64_INSN_IMM_MOVNZ, me);
            break;

        /* Immediate instruction relocations. */
        case R_AARCH64_LD_PREL_LO19:
            ovf = reloc_insn_imm(RELOC_OP_PREL, loc, val, 2, 19,
                         AARCH64_INSN_IMM_19, me);
            break;
        case R_AARCH64_ADR_PREL_LO21:
            ovf = reloc_insn_imm(RELOC_OP_PREL, loc, val, 0, 21,
                         AARCH64_INSN_IMM_ADR, me);
            break;
        case R_AARCH64_ADR_PREL_PG_HI21_NC:
            overflow_check = false;
            fallthrough;
        case R_AARCH64_ADR_PREL_PG_HI21:
            ovf = reloc_insn_adrp(me, sechdrs, loc, val, me);
            if (ovf && ovf != -ERANGE)
                return ovf;
            break;
        case R_AARCH64_ADD_ABS_LO12_NC:
        case R_AARCH64_LDST8_ABS_LO12_NC:
            overflow_check = false;
            ovf = reloc_insn_imm(RELOC_OP_ABS, loc, val, 0, 12,
                         AARCH64_INSN_IMM_12, me);
            break;
        case R_AARCH64_LDST16_ABS_LO12_NC:
            overflow_check = false;
            ovf = reloc_insn_imm(RELOC_OP_ABS, loc, val, 1, 11,
                         AARCH64_INSN_IMM_12, me);
            break;
        case R_AARCH64_LDST32_ABS_LO12_NC:
            overflow_check = false;
            ovf = reloc_insn_imm(RELOC_OP_ABS, loc, val, 2, 10,
                         AARCH64_INSN_IMM_12, me);
            break;
        case R_AARCH64_LDST64_ABS_LO12_NC:
            overflow_check = false;
            ovf = reloc_insn_imm(RELOC_OP_ABS, loc, val, 3, 9,
                         AARCH64_INSN_IMM_12, me);
            break;
        case R_AARCH64_LDST128_ABS_LO12_NC:
            overflow_check = false;
            ovf = reloc_insn_imm(RELOC_OP_ABS, loc, val, 4, 8,
                         AARCH64_INSN_IMM_12, me);
            break;
        case R_AARCH64_TSTBR14:
            ovf = reloc_insn_imm(RELOC_OP_PREL, loc, val, 2, 14,
                         AARCH64_INSN_IMM_14, me);
            break;
        case R_AARCH64_CONDBR19:
            ovf = reloc_insn_imm(RELOC_OP_PREL, loc, val, 2, 19,
                         AARCH64_INSN_IMM_19, me);
            break;
        case R_AARCH64_JUMP26:
        case R_AARCH64_CALL26:
            ovf = reloc_insn_imm(RELOC_OP_PREL, loc, val, 2, 26,
                         AARCH64_INSN_IMM_26, me);
            if (ovf == -ERANGE) {
                val = module_emit_plt_entry(me, sechdrs, loc, &rel[i], sym);
                if (!val)
                    return -ENOEXEC;
                ovf = reloc_insn_imm(RELOC_OP_PREL, loc, val, 2,
                             26, AARCH64_INSN_IMM_26, me);
            }
            break;

        default:
            pr_err("module %s: unsupported RELA relocation: %llu\n",
                   me->name, ELF64_R_TYPE(rel[i].r_info));
            return -ENOEXEC;
        }

        if (overflow_check && ovf == -ERANGE)
            goto overflow;

    }

    return 0;

overflow:
    pr_err("module %s: overflow in relocation type %d val %Lx\n",
           me->name, (int)ELF64_R_TYPE(rel[i].r_info), val);
    return -ENOEXEC;
}

int reloc_data(enum aarch64_reloc_op op, void *place, u64 val, int len,
              struct module *me)
{
    s64 sval = do_reloc(op, place, val) {
        switch (reloc_op) {
        case RELOC_OP_ABS:
            return val;
        case RELOC_OP_PREL:
            return val - (u64)place;
        case RELOC_OP_PAGE:
            return (val & ~0xfff) - ((u64)place & ~0xfff);
        case RELOC_OP_NONE:
            return 0;
        }

        pr_err("do_reloc: unknown relocation operation %d\n", reloc_op);
        return 0;
    }

    /* The ELF psABI for AArch64 documents the 16-bit and 32-bit place
     * relative and absolute relocations as having a range of [-2^15, 2^16)
     * or [-2^31, 2^32), respectively. However, in order to be able to
     * detect overflows reliably, we have to choose whether we interpret
     * such quantities as signed or as unsigned, and stick with it.
     * The way we organize our address space requires a signed
     * interpretation of 32-bit relative references, so let's use that
     * for all R_AARCH64_PRELxx relocations. This means our upper
     * bound for overflow detection should be Sxx_MAX rather than Uxx_MAX. */

    switch (len) {
    case 16:
        WRITE_PLACE((s16 *)place, sval, me);
        switch (op) {
        case RELOC_OP_ABS:
            if (sval < 0 || sval > U16_MAX)
                return -ERANGE;
            break;
        case RELOC_OP_PREL:
            if (sval < S16_MIN || sval > S16_MAX)
                return -ERANGE;
            break;
        default:
            pr_err("Invalid 16-bit data relocation (%d)\n", op);
            return 0;
        }
        break;
    case 32:
        WRITE_PLACE((s32 *)place, sval, me);
        switch (op) {
        case RELOC_OP_ABS:
            if (sval < 0 || sval > U32_MAX)
                return -ERANGE;
            break;
        case RELOC_OP_PREL:
            if (sval < S32_MIN || sval > S32_MAX)
                return -ERANGE;
            break;
        default:
            pr_err("Invalid 32-bit data relocation (%d)\n", op);
            return 0;
        }
        break;
    case 64:
        WRITE_PLACE((s64 *)place, sval, me);
        break;
    default:
        pr_err("Invalid length (%d) for data relocation\n", len);
        return 0;
    }
    return 0;
}

enum aarch64_insn_movw_imm_type {
    AARCH64_INSN_IMM_MOVNZ,
    AARCH64_INSN_IMM_MOVKZ,
};
```

### simplify_symbols

```c
int simplify_symbols(struct module *mod, const struct load_info *info)
{
    Elf_Shdr *symsec = &info->sechdrs[info->index.sym];
    Elf_Sym *sym = (void *)symsec->sh_addr;
    unsigned long secbase;
    unsigned int i;
    int ret = 0;
    const struct kernel_symbol *ksym;

    for (i = 1; i < symsec->sh_size / sizeof(Elf_Sym); i++) {
        const char *name = info->strtab + sym[i].st_name;

        switch (sym[i].st_shndx) {
        case SHN_COMMON:
            /* Ignore common symbols */
            if (!strncmp(name, "__gnu_lto", 9))
                break;

            /* We compiled with -fno-common.  These are not
             * supposed to happen. */
            pr_debug("Common symbol: %s\n", name);
            pr_warn("%s: please compile with -fno-common\n",
                   mod->name);
            ret = -ENOEXEC;
            break;

        case SHN_ABS:
            /* Don't need to do anything */
            pr_debug("Absolute symbol: 0x%08lx %s\n",
                 (long)sym[i].st_value, name);
            break;

        case SHN_LIVEPATCH:
            /* Livepatch symbols are resolved by livepatch */
            break;

        case SHN_UNDEF:
            ksym = resolve_symbol_wait(mod, info, name) {
                const struct kernel_symbol *ksym;
                char owner[MODULE_NAME_LEN];

                if (wait_event_interruptible_timeout(module_wq,
                        !IS_ERR(ksym = resolve_symbol(mod, info, name, owner))
                        || PTR_ERR(ksym) != -EBUSY, 30 * HZ) <= 0) {
                    pr_warn("%s: gave up waiting for init of module %s.\n",
                        mod->name, owner);
                }
                return ksym;
            }
            /* Ok if resolved.  */
            if (ksym && !IS_ERR(ksym)) {
                sym[i].st_value = kernel_symbol_value(ksym) {
                    #ifdef CONFIG_HAVE_ARCH_PREL32_RELOCATIONS
                        return (unsigned long)offset_to_ptr(&sym->value_offset);
                    #else
                        return sym->value;
                    #endif
                }
                break;
            }

            /* Ok if weak or ignored.  */
            if (!ksym &&
                (ELF_ST_BIND(sym[i].st_info) == STB_WEAK ||
                 ignore_undef_symbol(info->hdr->e_machine, name)))
                break;

            ret = PTR_ERR(ksym) ?: -ENOENT;
            pr_warn("%s: Unknown symbol %s (err %d)\n",
                mod->name, name, ret);
            break;

        default:
            /* Divert to percpu allocation if a percpu var. */
            if (sym[i].st_shndx == info->index.pcpu)
                secbase = (unsigned long)mod_percpu(mod);
            else
                secbase = info->sechdrs[sym[i].st_shndx].sh_addr;
            sym[i].st_value += secbase;
            break;
        }
    }

    return ret;
}

const struct kernel_symbol *resolve_symbol(struct module *mod,
                          const struct load_info *info,
                          const char *name,
                          char ownername[])
{
    struct find_symbol_arg fsa = {
        .name   = name,
        .gplok  = !(mod->taints & (1 << TAINT_PROPRIETARY_MODULE)),
        .warn   = true,
    };
    int err;

    /* The module_mutex should not be a heavily contended lock;
     * if we get the occasional sleep here, we'll go an extra iteration
     * in the wait_event_interruptible(), which is harmless. */
    sched_annotate_sleep();
    mutex_lock(&module_mutex);
    if (!find_symbol(&fsa))
        goto unlock;

    if (fsa.license == GPL_ONLY)
        mod->using_gplonly_symbols = true;

    if (!inherit_taint(mod, fsa.owner, name)) {
        fsa.sym = NULL;
        goto getname;
    }

    if (!check_version(info, name, mod, fsa.crc)) {
        fsa.sym = ERR_PTR(-EINVAL);
        goto getname;
    }

    err = verify_namespace_is_imported(info, fsa.sym, mod);
    if (err) {
        fsa.sym = ERR_PTR(err);
        goto getname;
    }

    err = ref_module(mod, fsa.owner);
    if (err) {
        fsa.sym = ERR_PTR(err);
        goto getname;
    }

getname:
    /* We must make copy under the lock if we failed to get ref. */
    strscpy(ownername, module_name(fsa.owner), MODULE_NAME_LEN);
unlock:
    mutex_unlock(&module_mutex);
    return fsa.sym;
}

bool find_symbol(struct find_symbol_arg *fsa)
{
    static const struct symsearch arr[] = {
        { __start___ksymtab, __stop___ksymtab, __start___kcrctab,
          NOT_GPL_ONLY },
        { __start___ksymtab_gpl, __stop___ksymtab_gpl,
          __start___kcrctab_gpl,
          GPL_ONLY },
    };
    struct module *mod;
    unsigned int i;

    for (i = 0; i < ARRAY_SIZE(arr); i++) {
        ret = find_exported_symbol_in_section(&arr[i], NULL, fsa) {
            struct kernel_symbol *sym;

            if (!fsa->gplok && syms->license == GPL_ONLY)
                return false;

            sym = bsearch(fsa->name, syms->start, syms->stop - syms->start,
                    sizeof(struct kernel_symbol), cmp_name);
            if (!sym)
                return false;

            fsa->owner = owner;
            fsa->crc = symversion(syms->crcs, sym - syms->start);
            fsa->sym = sym;
            fsa->license = syms->license;

            return true;
        }
        if (ret)
            return true;
    }

    list_for_each_entry_rcu(mod, &modules, list, lockdep_is_held(&module_mutex)) {
        struct symsearch arr[] = {
            { mod->syms, mod->syms + mod->num_syms, mod->crcs,
              NOT_GPL_ONLY },
            { mod->gpl_syms, mod->gpl_syms + mod->num_gpl_syms,
              mod->gpl_crcs,
              GPL_ONLY },
        };

        if (mod->state == MODULE_STATE_UNFORMED)
            continue;

        for (i = 0; i < ARRAY_SIZE(arr); i++)
            if (find_exported_symbol_in_section(&arr[i], mod, fsa))
                return true;
    }

    pr_debug("Failed to find symbol %s\n", fsa->name);
    return false;
}
```

## request_module

```c
#define request_module(mod...) __request_module(true, mod)

char modprobe_path[KMOD_PATH_LEN] = CONFIG_MODPROBE_PATH;

int __request_module(bool wait, const char *fmt, ...)
{
    va_list args;
    char module_name[MODULE_NAME_LEN];
    int ret, dup_ret;

    /* We don't allow synchronous module loading from async.  Module
     * init may invoke async_synchronize_full() which will end up
     * waiting for this task which already is waiting for the module
     * loading to complete, leading to a deadlock. */
    WARN_ON_ONCE(wait && current_is_async());

    if (!modprobe_path[0])
        return -ENOENT;

    va_start(args, fmt);
    ret = vsnprintf(module_name, MODULE_NAME_LEN, fmt, args);
    va_end(args);
    if (ret >= MODULE_NAME_LEN)
        return -ENAMETOOLONG;

    ret = security_kernel_module_request(module_name);
    if (ret)
        return ret;

    ret = down_timeout(&kmod_concurrent_max, MAX_KMOD_ALL_BUSY_TIMEOUT * HZ);
    if (ret) {
        pr_warn_ratelimited("request_module: modprobe %s cannot be processed, kmod busy with %d threads for more than %d seconds now",
                    module_name, MAX_KMOD_CONCURRENT, MAX_KMOD_ALL_BUSY_TIMEOUT);
        return ret;
    }

    trace_module_request(module_name, wait, _RET_IP_);

    if (kmod_dup_request_exists_wait(module_name, wait, &dup_ret)) {
        ret = dup_ret;
        goto out;
    }

    ret = call_modprobe(module_name, wait ? UMH_WAIT_PROC : UMH_WAIT_EXEC);

out:
    up(&kmod_concurrent_max);

    return ret;
}

int call_modprobe(char *orig_module_name, int wait)
{
    struct subprocess_info *info;
    static char *envp[] = {
        "HOME=/",
        "TERM=linux",
        "PATH=/sbin:/usr/sbin:/bin:/usr/bin",
        NULL
    };
    char *module_name;
    int ret;

    char **argv = kmalloc(sizeof(char *[5]), GFP_KERNEL);
    if (!argv)
        goto out;

    module_name = kstrdup(orig_module_name, GFP_KERNEL);
    if (!module_name)
        goto free_argv;

    argv[0] = modprobe_path;
    argv[1] = "-q";
    argv[2] = "--";
    argv[3] = module_name;    /* check free_modprobe_argv() */
    argv[4] = NULL;

    info = call_usermodehelper_setup(modprobe_path, argv, envp, GFP_KERNEL, NULL, free_modprobe_argv, NULL) {
        struct subprocess_info *sub_info;
        sub_info = kzalloc(sizeof(struct subprocess_info), gfp_mask);
        if (!sub_info)
            goto out;

        INIT_WORK(&sub_info->work, call_usermodehelper_exec_work);

    #ifdef CONFIG_STATIC_USERMODEHELPER
        sub_info->path = CONFIG_STATIC_USERMODEHELPER_PATH;
    #else
        sub_info->path = path;
    #endif
        sub_info->argv = argv;
        sub_info->envp = envp;

        sub_info->cleanup = cleanup;
        sub_info->init = init;
        sub_info->data = data;
    out:
        return sub_info;
    }
    if (!info)
        goto free_module_name;

    ret = call_usermodehelper_exec(info, wait | UMH_KILLABLE) {
        unsigned int state = TASK_UNINTERRUPTIBLE;
        DECLARE_COMPLETION_ONSTACK(done);
        int retval = 0;

        if (!sub_info->path) {
            call_usermodehelper_freeinfo(sub_info);
            return -EINVAL;
        }
        helper_lock();
        if (usermodehelper_disabled) {
            retval = -EBUSY;
            goto out;
        }

        /* If there is no binary for us to call, then just return and get out of
        * here.  This allows us to set STATIC_USERMODEHELPER_PATH to "" and
        * disable all call_usermodehelper() calls. */
        if (strlen(sub_info->path) == 0)
            goto out;

        /* Set the completion pointer only if there is a waiter.
        * This makes it possible to use umh_complete to free
        * the data structure in case of UMH_NO_WAIT. */
        sub_info->complete = (wait == UMH_NO_WAIT) ? NULL : &done;
        sub_info->wait = wait;

        /* call_usermodehelper_exec_work */
        queue_work(system_unbound_wq, &sub_info->work);
        if (wait == UMH_NO_WAIT)    /* task has freed sub_info */
            goto unlock;

        if (wait & UMH_FREEZABLE)
            state |= TASK_FREEZABLE;

        if (wait & UMH_KILLABLE) {
            retval = wait_for_completion_state(&done, state | TASK_KILLABLE);
            if (!retval)
                goto wait_done;

            /* umh_complete() will see NULL and free sub_info */
            if (xchg(&sub_info->complete, NULL))
                goto unlock;

            /* fallthrough; in case of -ERESTARTSYS now do uninterruptible
            * wait_for_completion_state(). Since umh_complete() shall call
            * complete() in a moment if xchg() above returned NULL, this
            * uninterruptible wait_for_completion_state() will not block
            * SIGKILL'ed processes for long. */
        }
        wait_for_completion_state(&done, state);

    wait_done:
        retval = sub_info->retval;
    out:
        call_usermodehelper_freeinfo(sub_info);
    unlock:
        helper_unlock();
        return retval;
    }
    kmod_dup_request_announce(orig_module_name, ret);
    return ret;

free_module_name:
    kfree(module_name);
free_argv:
    kfree(argv);
out:
    kmod_dup_request_announce(orig_module_name, -ENOMEM);
    return -ENOMEM;
}

void call_usermodehelper_exec_work(struct work_struct *work)
{
    struct subprocess_info *sub_info =
        container_of(work, struct subprocess_info, work);

    if (sub_info->wait & UMH_WAIT_PROC) {
        call_usermodehelper_exec_sync(sub_info) {
            pid_t pid;

            /* If SIGCLD is ignored do_wait won't populate the status. */
            kernel_sigaction(SIGCHLD, SIG_DFL);
            pid = user_mode_thread(call_usermodehelper_exec_async, sub_info, SIGCHLD);
            if (pid < 0)
                sub_info->retval = pid;
            else
                kernel_wait(pid, &sub_info->retval);

            /* Restore default kernel sig handler */
            kernel_sigaction(SIGCHLD, SIG_IGN);
            umh_complete(sub_info);
        }
    } else {
        pid_t pid;
        /* Use CLONE_PARENT to reparent it to kthreadd; we do not
         * want to pollute current->children, and we need a parent
         * that always ignores SIGCHLD to ensure auto-reaping. */
        pid = user_mode_thread(call_usermodehelper_exec_async, sub_info, CLONE_PARENT | SIGCHLD) {
            struct kernel_clone_args args = {
                .flags  = ((flags | CLONE_VM | CLONE_UNTRACED) & ~CSIGNAL),
                .exit_signal    = (flags & CSIGNAL),
                .fn             = fn,
                .fn_arg         = arg,
            };

            return kernel_clone(&args);
        }
        if (pid < 0) {
            sub_info->retval = pid;
            umh_complete(sub_info);
        }
    }
}

/* This is the task which runs the usermode application */
int call_usermodehelper_exec_async(void *data)
{
    struct subprocess_info *sub_info = data;
    struct cred *new;
    int retval;

    spin_lock_irq(&current->sighand->siglock);
    flush_signal_handlers(current, 1);
    spin_unlock_irq(&current->sighand->siglock);

    /* Initial kernel threads share ther FS with init, in order to
     * get the init root directory. But we've now created a new
     * thread that is going to execve a user process and has its own
     * 'struct fs_struct'. Reset umask to the default. */
    current->fs->umask = 0022;

    /* Our parent (unbound workqueue) runs with elevated scheduling
     * priority. Avoid propagating that into the userspace child. */
    set_user_nice(current, 0);

    retval = -ENOMEM;
    new = prepare_kernel_cred(current);
    if (!new)
        goto out;

    spin_lock(&umh_sysctl_lock);
    new->cap_bset = cap_intersect(usermodehelper_bset, new->cap_bset);
    new->cap_inheritable = cap_intersect(usermodehelper_inheritable,
                         new->cap_inheritable);
    spin_unlock(&umh_sysctl_lock);

    if (sub_info->init) {
        retval = sub_info->init(sub_info, new);
        if (retval) {
            abort_creds(new);
            goto out;
        }
    }

    commit_creds(new);

    wait_for_initramfs();
    retval = kernel_execve(sub_info->path,
                   (const char *const *)sub_info->argv,
                   (const char *const *)sub_info->envp);
out:
    sub_info->retval = retval;
    /* call_usermodehelper_exec_sync() will call umh_complete
     * if UHM_WAIT_PROC. */
    if (!(sub_info->wait & UMH_WAIT_PROC))
        umh_complete(sub_info);
    if (!retval)
        return 0;
    do_exit(0);
}
```