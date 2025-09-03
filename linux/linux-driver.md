
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

```c
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
```

# platform

```c
struct device {
    struct kobject          kobj;
    struct device           *parent;

    struct device_private   *p;

    const char                  *init_name; /* initial name of the device */
    const struct device_type    *type;

    const struct bus_type       *bus;    /* type of bus device is on */
    struct device_driver        *driver;    /* which driver has allocated this
                       device */
    void        *platform_data;    /* Platform specific data, device
                       core doesn't touch it */
    void        *driver_data;    /* Driver data, set and get with
                       dev_set_drvdata/dev_get_drvdata */
    struct mutex    mutex;    /* mutex to synchronize calls to
                     * its driver. */

    struct dev_links_info   links;
    struct dev_pm_info      power;
    struct dev_pm_domain    *pm_domain;
}

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

void cdev_init(struct cdev *, const struct file_operations *);

struct cdev *cdev_alloc(void);

void cdev_put(struct cdev *p);

int cdev_add(struct cdev *, dev_t, unsigned);

void cdev_set_parent(struct cdev *p, struct kobject *kobj);
int cdev_device_add(struct cdev *cdev, struct device *dev);
void cdev_device_del(struct cdev *cdev, struct device *dev);

void cdev_del(struct cdev *);

void cd_forget(struct inode *);

int register_chrdev_region(dev_t from, unsigned count, const char *name);
int alloc_chrdev_region(dev_t *dev, unsigned baseminor, unsigned count, const char *name)
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

# core

## device_add

# requset_mem_region

# relese_mem_region

# ioremap

# iounmap

# readl

# writel

# remap_pfn_range

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