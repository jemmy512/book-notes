# Table of Contents
* [segment](#segment)
* [paging](#paging)
* [mm_init](#mm_init)
* [user virtual space](#user-virtual-space)
* [kernel virtual space](#kernel-virtual-space)
* [numa](#numa)
    * [node](#node)
    * [zone](#zone)
    * [page](#page)
* [buddy system](#buddy-system)
* [alloc_pages](#alloc_pages)
* [kmem_cache](#kmem_cache)
    * [kmem_cache_create](#kmem_cache_create)
    * [kmem_cache_alloc](#kmem_cache_alloc)
    * [kmem_cache_alloc_node](#kmem_cache_alloc_node)
* [slab_alloc](#slab_alloc)
* [kswapd](#kswapd)
* [brk](#brk)
* [mmap](#mmap)
* [page fault](#page-fault)
    * [do_anonymous_page](#do_anonymous_page)
    * [do_fault](#do_fault)
    * [do_swap_page](#do_swap_page)
* [pgd](#pgd)
* [kernel mapping](#kernel-mapping)
* [kmalloc](#kmalloc)
    * [kmalloc_caches](#kmalloc_caches)
* [kmap_atomic](#kmap_atomic)
* [page_address](#page_address)
* [vmalloc](#vmalloc)
* [vmalloc_fault](#vmalloc_fault)

![](../Images/Kernel/kernel-structual.svg)

---

# segment
```C++
#define GDT_ENTRY_INIT(flags, base, limit) { { { \
    .a = ((limit) & 0xffff) | (((base) & 0xffff) << 16), \
    .b = (((base) & 0xff0000) >> 16) | (((flags) & 0xf0ff) << 8) | \
      ((limit) & 0xf0000) | ((base) & 0xff000000), \
  } } }

DEFINE_PER_CPU_PAGE_ALIGNED(struct gdt_page, gdt_page) = { .gdt = {
#ifdef CONFIG_X86_64
  [GDT_ENTRY_KERNEL32_CS]       = GDT_ENTRY_INIT(0xc09b, 0, 0xfffff),
  [GDT_ENTRY_KERNEL_CS]         = GDT_ENTRY_INIT(0xa09b, 0, 0xfffff),
  [GDT_ENTRY_KERNEL_DS]         = GDT_ENTRY_INIT(0xc093, 0, 0xfffff),
  [GDT_ENTRY_DEFAULT_USER32_CS] = GDT_ENTRY_INIT(0xc0fb, 0, 0xfffff),
  [GDT_ENTRY_DEFAULT_USER_DS]   = GDT_ENTRY_INIT(0xc0f3, 0, 0xfffff),
  [GDT_ENTRY_DEFAULT_USER_CS]   = GDT_ENTRY_INIT(0xa0fb, 0, 0xfffff),
#else
  [GDT_ENTRY_KERNEL_CS]         = GDT_ENTRY_INIT(0xc09a, 0, 0xfffff),
  [GDT_ENTRY_KERNEL_DS]         = GDT_ENTRY_INIT(0xc092, 0, 0xfffff),
  [GDT_ENTRY_DEFAULT_USER_CS]   = GDT_ENTRY_INIT(0xc0fa, 0, 0xfffff),
  [GDT_ENTRY_DEFAULT_USER_DS]   = GDT_ENTRY_INIT(0xc0f2, 0, 0xfffff),

#endif
} };
EXPORT_PER_CPU_SYMBOL_GPL(gdt_page);

#define __KERNEL_CS      (GDT_ENTRY_KERNEL_CS*8)
#define __KERNEL_DS      (GDT_ENTRY_KERNEL_DS*8)
#define __USER_DS      (GDT_ENTRY_DEFAULT_USER_DS*8 + 3)
#define __USER_CS      (GDT_ENTRY_DEFAULT_USER_CS*8 + 3)
```
![](../Images/Kernel/mem-segment.png)

# paging
![](../Images/Kernel/mem-segment-page.png)

![](../Images/Kernel/mem-kernel-page-table.png)

# mm_init
```c++
void start_kernel(void) {
  mm_init();
}

static void __init mm_init(void)
{
  page_ext_init_flatmem();
  mem_init();
  kmem_cache_init();
  pgtable_init();
  vmalloc_init();
  ioremap_huge_init();
  init_espfix_bsp();
  pti_init();
}
```

# user virtual space
```C++

#ifdef CONFIG_X86_32
/* User space process size: 3GB (default). */
#define TASK_SIZE    PAGE_OFFSET
#define TASK_SIZE_MAX    TASK_SIZE
/* config PAGE_OFFSET
        hex
        default 0xC0000000
        depends on X86_32 */
#else
/* User space process size. 47bits minus one guard page. */
#define TASK_SIZE_MAX  ((1UL << 47) - PAGE_SIZE)
#define TASK_SIZE    (test_thread_flag(TIF_ADDR32) ? \
          IA32_PAGE_OFFSET : TASK_SIZE_MAX)
struct mm_struct {
  unsigned long mmap_base;  /* base of mmap area */
  unsigned long total_vm;   /* Total pages mapped */
  unsigned long locked_vm;  /* Pages that have PG_mlocked set */
  unsigned long pinned_vm;  /* Refcount permanently increased */
  unsigned long data_vm;    /* VM_WRITE & ~VM_SHARED & ~VM_STACK */
  unsigned long exec_vm;    /* VM_EXEC & ~VM_WRITE & ~VM_STACK */
  unsigned long stack_vm;   /* VM_STACK */
  unsigned long start_code, end_code, start_data, end_data;
  unsigned long start_brk, brk, start_stack;
  unsigned long arg_start, arg_end, env_start, env_end;

  unsigned long task_size; /* size of task vm space */

  struct vm_area_struct *mmap;    /* list of VMAs */
  struct rb_root mm_rb;
};

struct vm_area_struct {
  /* The first cache line has the info for VMA tree walking. */
  unsigned long vm_start;    /* Our start address within vm_mm. */
  unsigned long vm_end;    /* The first byte after our end address within vm_mm. */
  /* linked list of VM areas per task, sorted by address */
  struct vm_area_struct *vm_next, *vm_prev;
  struct rb_node vm_rb;

  struct mm_struct *vm_mm;  /* The address space we belong to. */
  struct list_head anon_vma_chain; /* Serialized by mmap_sem &
            * page_table_lock */
  const struct vm_operations_struct *vm_ops;

  struct anon_vma *anon_vma;  /* Serialized by page_table_lock */
  /* Function pointers to deal with this struct. */
  struct file * vm_file;    /* File we map to (can be NULL). */
  void * vm_private_data;    /* was vm_pte (shared mem) */
} __randomize_layout;
```
![](../Images/Kernel/mem-vm.png)

# kernel virtual space
```C++
/* PKMAP_BASE:
 * use alloc_pages() get struct page, user kmap() map the page to this area */

/* FIXADDR_START:
 * use kmap_atomic() to map a file to write it back to physic disk */
```
![](../Images/Kernel/mem-kernel.png)

![](../Images/Kernel/mem-kernel-2.png)

![](../Images/Kernel/mem-user-kernel-32.png)

![](../Images/Kernel/mem-user-kernel-64.png)

# numa
## node
```C++
struct pglist_data *node_data[MAX_NUMNODES];

typedef struct pglist_data {
  struct zone node_zones[MAX_NR_ZONES];
  /* backup area if current node run out */
  struct zonelist node_zonelists[MAX_ZONELISTS];
  int nr_zones;
  struct page *node_mem_map;
  unsigned long node_start_pfn;     /* start page number of this node */
  unsigned long node_present_pages; /* total number of physical pages */
  unsigned long node_spanned_pages; /* total size of physical page range, including holes */
  int node_id;
} pg_data_t;

enum zone_type {
  ZONE_DMA,
  ZONE_DMA32,
  ZONE_NORMAL, /* direct mmapping area */
  ZONE_HIGHMEM,
  ZONE_MOVABLE,
  __MAX_NR_ZONES
};
```

## zone
```C++
struct zone {
  struct pglist_data  *zone_pgdat;
  struct per_cpu_pageset *pageset; // hot/cold page

  unsigned long    zone_start_pfn;
  unsigned long    managed_pages; // managed_pages = present_pages - reserved_pages
  unsigned long    spanned_pages; // spanned_pages = zone_end_pfn - zone_start_pfn
  unsigned long    present_pages; // present_pages = spanned_pages - absent_pages(pages in holes)

  const char    *name;
  /* free areas of different sizes */
  struct free_area  free_area[MAX_ORDER];
  /* zone flags, see below */
  unsigned long    flags;

  /* Primarily protects free_area */
  spinlock_t    lock;
};

struct per_cpu_pageset {
  struct per_cpu_pages pcp;
  s8 expire;
  u16 vm_numa_stat_diff[NR_VM_NUMA_STAT_ITEMS];
}
struct per_cpu_pages {
  int count; /* number of pages in the list */
  int high;  /* high watermark, emptying needed */
  int batch; /* chunk size for buddy add/remove */

  /* Lists of pages, one per migrate type stored on the pcp-lists */
  struct list_head lists[MIGRATE_PCPTYPES];
};

enum migratetype {
  MIGRATE_UNMOVABLE,
  MIGRATE_MOVABLE,
  MIGRATE_RECLAIMABLE,
  MIGRATE_PCPTYPES,  /* the number of types on the pcp lists */
  MIGRATE_HIGHATOMIC = MIGRATE_PCPTYPES,
  MIGRATE_CMA,
  MIGRATE_ISOLATE,
  MIGRATE_TYPES
};
```

## page
```C++
struct page {
  /* Atomic flags, some possibly updated asynchronously */
  unsigned long flags;

  union {
    /* 1. Page cache and anonymous pages */
    struct {
      struct list_head lru; /* See page-flags.h for PAGE_MAPPING_FLAGS */
      /* lowest bit is 1 for anonymous mapping, 0 for file mapping */
      struct address_space *mapping;
      pgoff_t index;          /* Our offset within mapping. */
      unsigned long private; /* struct buffer_head */
    };

    struct {  /* page_pool used by netstack */
      dma_addr_t dma_addr;
    };


   /* 2. slab, slob and slub*/
    struct {
      union {
        struct list_head slab_list;
        struct {  /* Partial pages */
          struct page *next;
          int pages;    /* Nr of pages left */
          int pobjects; /* Approximate count */
        };
      };

      struct kmem_cache *slab_cache; /* not slob */
      void *freelist;           /* first free object */

      union {
        void *s_mem;            /* slab: first object */
        unsigned long counters; /* SLUB */

        struct {                /* SLUB */
          unsigned inuse:16;
          unsigned objects:15;
          unsigned frozen:1;
        };
      };
    };


    /* 3. Tail pages of compound page */
    struct {
      unsigned long compound_head; /* Bit zero is set */
      unsigned char compound_dtor; /* First tail page only */
      unsigned char compound_order;
      atomic_t compound_mapcount;
    };


    /* 4. Second tail page of compound page */
    struct {
      unsigned long _compound_pad_1;  /* compound_head */
      unsigned long _compound_pad_2;
      struct list_head deferred_list; /* For both global and memcg */
    };


    /* 5. Page table pages */
    struct {
      unsigned long _pt_pad_1; /* compound_head */
      pgtable_t pmd_huge_pte;  /* protected by page->ptl */
      unsigned long _pt_pad_2; /* mapping */

      union {
        struct mm_struct *pt_mm;  /* x86 pgds only */
        atomic_t pt_frag_refcount;/* powerpc */
      };
      spinlock_t *ptl;
    };

    struct {  /* ZONE_DEVICE pages */
      struct dev_pagemap *pgmap;
      void *zone_device_data;
    };

    struct rcu_head rcu_head;
  };

  union {    /* This union is 4 bytes in size. */
    atomic_t _mapcount;
    unsigned int page_type;
    unsigned int active;  /* SLAB */
    int units;            /* SLOB */
  };

  atomic_t _refcount;

  struct mem_cgroup *mem_cgroup;

  /* Kernel virtual address (NULL if not kmapped, ie. highmem) */
  void *virtual;
  int _last_cpupid;
};
```
![](../Images/Kernel/mem-physic-numa-1.png)
![](../Images/Kernel/mem-physic-numa-2.png)

# buddy system
```C++
struct free_area  free_area[MAX_ORDER];
#define MAX_ORDER 11
```
![](../Images/Kernel/mem-buddy-freepages.png)

# alloc_pages
```C++
#define alloc_page(gfp_mask) alloc_pages(gfp_mask, 0)

static inline struct page* alloc_pages(gfp_t gfp_mask, unsigned int order)
{
  return alloc_pages_current(gfp_mask, order);
}

/*  %GFP_USER     user allocation,
 *  %GFP_KERNEL   kernel allocation, direct mapping area
 *  %GFP_HIGHMEM  highmem allocation,
 *  %GFP_FS       don't call back into a file system.
 *  %GFP_ATOMIC   don't sleep.
 *  @order: Power of two of allocation size in pages. 0 is a single page. */
struct page *alloc_pages_current(gfp_t gfp, unsigned order)
{
  struct mempolicy *pol = &default_policy;
  struct page *page;

  page = __alloc_pages_nodemask(gfp, order, policy_node(gfp, pol, numa_node_id()), policy_nodemask(gfp, pol));
  return page;
}

// __alloc_pages_nodemask ->
static struct page* get_page_from_freelist(
  gfp_t gfp_mask, unsigned int order,
  int alloc_flags, const struct alloc_context *ac)
{
  for_next_zone_zonelist_nodemask(zone, z, ac->zonelist, ac->high_zoneidx, ac->nodemask) {
    struct page *page;
    page = rmqueue(ac->preferred_zoneref->zone, zone, order,
        gfp_mask, alloc_flags, ac->migratetype);
  }
}

/* rmqueue -> __rmqueue -> __rmqueue_smallest */
static inline
struct page *__rmqueue_smallest(
  struct zone *zone, unsigned int order, int migratetype)
{
  unsigned int current_order;
  struct free_area *area;
  struct page *page;

  /* Find a page of the appropriate size in the preferred list */
  for (current_order = order; current_order < MAX_ORDER; ++current_order) {
    area = &(zone->free_area[current_order]);
    page = list_first_entry_or_null(&area->free_list[migratetype],
              struct page, lru);
    if (!page)
      continue;

    list_del(&page->lru);
    rmv_page_order(page);
    area->nr_free--;
    expand(zone, page, order, current_order, area, migratetype);
    set_pcppage_migratetype(page, migratetype);
    return page;
  }
  return NULL;
}

static inline void expand(struct zone *zone, struct page *page,
  int low, int high, struct free_area *area,
  int migratetype)
{
  unsigned long size = 1 << high;

  while (high > low) {
    area--;
    high--;
    size >>= 1;

    list_add(&page[size].lru, &area->free_list[migratetype]);
    area->nr_free++;
    set_page_order(&page[size], high);
  }
}
```

# kmem_cache
![](../Images/Kernel/mem-kmem-cache-cpu-node.png)
![](../Images/Kernel/mem-kmem-cache.png)

```c++
/* linux-4.19.y/mm/slab_common.c */
enum slab_state slab_state;
LIST_HEAD(slab_caches);
DEFINE_MUTEX(slab_mutex);
struct kmem_cache *kmem_cache;

/* linux-4.19.y/mm/slab.c */
struct kmem_cache kmem_cache_boot = {
  .name   = "kmem_cache",
  .size   = sizeof(struct kmem_cache),
  .flags  = SLAB_PANIC,
  .aligs  = ARCH_KMALLOC_MINALIGN,
};

void kmem_cache_init(void)
{
  static struct kmem_cache boot_kmem_cache, boot_kmem_cache_node;

  kmem_cache_node = &boot_kmem_cache_node;
  kmem_cache = &boot_kmem_cache;

  create_boot_cache(kmem_cache_node, "kmem_cache_node",
    sizeof(struct kmem_cache_node), SLAB_HWCACHE_ALIGN, 0, 0);

  register_hotmemory_notifier(&slab_memory_callback_nb);

  /* Able to allocate the per node structures */
  slab_state = PARTIAL;

  create_boot_cache(kmem_cache, "kmem_cache",
      offsetof(struct kmem_cache, node) +
        nr_node_ids * sizeof(struct kmem_cache_node *),
           SLAB_HWCACHE_ALIGN, 0, 0);

  kmem_cache = bootstrap(&boot_kmem_cache);
  kmem_cache_node = bootstrap(&boot_kmem_cache_node);

  /* Now we can use the kmem_cache to allocate kmalloc slabs */
  setup_kmalloc_cache_index_table();
  create_kmalloc_caches(0);

  /* Setup random freelists for each cache */
  init_freelist_randomization();

  cpuhp_setup_state_nocalls(CPUHP_SLUB_DEAD, "slub:dead", NULL,
          slub_cpu_dead);
}

struct kmem_cache {
  /* each NUMA node has one kmem_cache_cpu kmem_cache_node */
  struct kmem_cache_cpu  *cpu_slab;
  struct kmem_cache_node *node[MAX_NUMNODES];

  /* Used for retriving partial slabs etc */
  unsigned long flags;
  unsigned long min_partial;
  int size;         /* The size of an object including meta data */
  int object_size;  /* The size of an object without meta data */
  int offset;       /* Free pointer offset. */
  int cpu_partial;  /* Number of per cpu partial objects to keep around */

  struct kmem_cache_order_objects oo;
  /* Allocation and freeing of slabs */
  struct kmem_cache_order_objects max;
  struct kmem_cache_order_objects min;
  gfp_t allocflags;  /* gfp flags to use on each alloc */
  int refcount;      /* Refcount for slab cache destroy */
  void (*ctor)(void *);
  const char *name;       /* Name (only for display!) */
  struct list_head list;  /* List of slab caches */
};

struct kmem_cache_cpu {
  void **freelist;      /* Pointer to next available object */
  struct page *page;    /* The slab from which we are allocating */
  struct page *partial; /* Partially allocated frozen slabs */
  unsigned long tid;    /* Globally unique transaction id */
};

struct kmem_cache_node {
  unsigned long     nr_partial;
  struct list_head  partial;
};
```

## kmem_cache_create
```C++
static struct kmem_cache *task_struct_cachep;

task_struct_cachep = kmem_cache_create("task_struct",
      arch_task_struct_size, align,
      SLAB_PANIC|SLAB_NOTRACK|SLAB_ACCOUNT, NULL);

struct kmem_cache *kmem_cache_create(
  const char *name, unsigned int size, unsigned int align,
  slab_flags_t flags, void (*ctor)(void *))
{
  return kmem_cache_create_usercopy(name, size, align, flags, 0, 0, ctor);
}

struct kmem_cache *kmem_cache_create_usercopy(
  const char *name, /* name in /proc/slabinfo to identify this cache */
  unsigned int size,
  unsigned int align,
  slab_flags_t flags,
  unsigned int useroffset, /* Usercopy region offset */
  unsigned int usersize,  /* Usercopy region size */
  void (*ctor)(void *))
{
  struct kmem_cache *s = NULL;
  const char *cache_name;
  int err;

  get_online_cpus();
  get_online_mems();
  memcg_get_cache_ids();

  mutex_lock(&slab_mutex);

  if (!usersize)
    s = __kmem_cache_alias(name, size, align, flags, ctor);
  if (s)
    goto out_unlock;

  cache_name = kstrdup_const(name, GFP_KERNEL);

  s = create_cache(cache_name, size,
       calculate_alignment(flags, align, size),
       flags, useroffset, usersize, ctor, NULL, NULL);

out_unlock:
  mutex_unlock(&slab_mutex);

  memcg_put_cache_ids();
  put_online_mems();
  put_online_cpus();

  return s;
}

static struct kmem_cache *create_cache(
  const char *name,
  unsigned int object_size, unsigned int align,
  slab_flags_t flags, unsigned int useroffset,
  unsigned int usersize, void (*ctor)(void *),
  struct mem_cgroup *memcg, struct kmem_cache *root_cache)
{
  struct kmem_cache *s;
  int err;

  /* 1. alloc */
  /* kmem_cache = &boot_kmem_cache; */
  s = kmem_cache_zalloc(kmem_cache, GFP_KERNEL);

  s->name = name;
  s->size = s->object_size = object_size;
  s->align = align;
  s->ctor = ctor;
  s->useroffset = useroffset;
  s->usersize = usersize;

  /* 2. init */
  err = init_memcg_params(s, memcg, root_cache);
  err = __kmem_cache_create(s, flags);

  /* 3. link */
  s->refcount = 1;
  list_add(&s->list, &slab_caches);
  memcg_link_cache(s);

  return s;
}

/* 1. alloc */
static inline void *kmem_cache_zalloc(struct kmem_cache *k, gfp_t flags)
{
  return kmem_cache_alloc(k, flags | __GFP_ZERO);
}

/* 2. init */
int __kmem_cache_create(struct kmem_cache *s, slab_flags_t flags)
{
  int err;

  err = kmem_cache_open(s, flags);

  memcg_propagate_slab_attrs(s);
  err = sysfs_slab_add(s);
  if (err)
    __kmem_cache_release(s);

  return err;
}

static int kmem_cache_open(struct kmem_cache *s, slab_flags_t flags)
{
  if (!calculate_sizes(s, -1))
    goto error;
  if (disable_higher_order_debug) {
    if (get_order(s->size) > get_order(s->object_size)) {
      s->flags &= ~DEBUG_METADATA_FLAGS;
      s->offset = 0;
      if (!calculate_sizes(s, -1))
        goto error;
    }
  }

  set_min_partial(s, ilog2(s->size) / 2);
  set_cpu_partial(s);

  if (slab_state >= UP) {
    if (init_cache_random_seq(s))
      goto error;
  }

  if (!init_kmem_cache_nodes(s))
    goto error;

  if (alloc_kmem_cache_cpus(s))
    return 0;
}

/* kmem_cache_node */
static int init_kmem_cache_nodes(struct kmem_cache *s)
{
  for_each_node_state(node, N_NORMAL_MEMORY) {
      struct kmem_cache_node *n;

      if (slab_state == DOWN) {
          early_kmem_cache_node_alloc(node);
          continue;
      }
      n = kmem_cache_alloc_node(
        kmem_cache_node, GFP_KERNEL, node);

      if (!n) {
          free_kmem_cache_nodes(s);
          return 0;
      }

      init_kmem_cache_node(n);
      s->node[node] = n;
  }
}

void *kmem_cache_alloc_node(struct kmem_cache *cachep, gfp_t gfp, int node)
{
  return slab_alloc_node(cachep, gfp, node);
}

/* kmem_cache_cpu */
static inline int alloc_kmem_cache_cpus(struct kmem_cache *s)
{
  s->cpu_slab = __alloc_percpu(sizeof(struct kmem_cache_cpu),
             2 * sizeof(void *));

  init_kmem_cache_cpus(s);

  return 1;
}

static void init_kmem_cache_cpus(struct kmem_cache *s)
{
  int cpu;

  for_each_possible_cpu(cpu)
    per_cpu_ptr(s->cpu_slab, cpu)->tid = init_tid(cpu);
}

#define per_cpu_ptr(ptr, cpu) \
  ((typeof(ptr)) ((char *) (ptr) + PERCPU_OFFSET * cpu))
```

```c++
kmem_cache_create();
    __kmem_cache_alias();
        find_mergable();
    create_cache();
        kmem_cache_zalloc();
           kmem_cache_alloc();

        __kmem_cache_create();
            kmem_cache_open();
                caculate_size();
                    caculate_order();
                    oo_make();
                set_min_partial();
                set_cpu_partial();

                init_kmem_cache_nodes();
                    kmem_alloc_cache_node();
                    init_keme_cache_node();
                alloc_kmem_cache_cpus();
                    init_keme_cache_cpu();

        list_add();
```

## kmem_cache_alloc
```c++
void *kmem_cache_alloc(struct kmem_cache *cachep, gfp_t flags)
{
  void *ret = slab_alloc(cachep, flags, _RET_IP_);

  return ret;
}
```

## kmem_cache_alloc_node
```c++
static inline struct task_struct *alloc_task_struct_node(int node)
{
  return kmem_cache_alloc_node(task_struct_cachep, GFP_KERNEL, node);
}

void *kmem_cache_alloc_node(struct kmem_cache *cachep, gfp_t gfp, int node)
{
  return slab_alloc_node(cachep, gfp, node);
}

static inline void free_task_struct(struct task_struct *tsk)
{
  kmem_cache_free(task_struct_cachep, tsk);
}
```

# slab_alloc
```C++
static __always_inline void *slab_alloc(
  struct kmem_cache *s, gfp_t gfpflags, unsigned long addr)
{
  return slab_alloc_node(s, gfpflags, NUMA_NO_NODE, addr);
}

static void *slab_alloc_node(struct kmem_cache *s,
    gfp_t gfpflags, int node, unsigned long addr)
{
  void *object;
  struct kmem_cache_cpu *c;
  struct page *page;
  unsigned long tid;

  tid = this_cpu_read(s->cpu_slab->tid);
  c = raw_cpu_ptr(s->cpu_slab);

  object = c->freelist;
  page = c->page;
  if (unlikely(!object || !node_match(page, node))) {
    object = __slab_alloc(s, gfpflags, node, addr, c);
    stat(s, ALLOC_SLOWPATH);
  } else {
    /* update the freelist and tid to new values */
    void *next_object = get_freepointer_safe(s, object);

    if (unlikely(!this_cpu_cmpxchg_double(
        s->cpu_slab->freelist, s->cpu_slab->tid,
        object, tid,
        next_object, next_tid(tid))))
    {
      note_cmpxchg_failure("slab_alloc", s, tid);
      goto redo;
    }

    prefetch_freepointer(s, next_object);
    stat(s, ALLOC_FASTPATH);
  }
  return object;
}

static void *__slab_alloc(
  struct kmem_cache *s, gfp_t gfpflags, int node,
  unsigned long addr, struct kmem_cache_cpu *c)
{
  void *freelist;
  struct page *page;
redo:
  // 1. try kmem_cache_cpu freelist
  freelist = c->freelist;
  if (freelist)
    goto load_freelist;

  freelist = get_freelist(s, page);
  if (!freelist) {
    c->page = NULL;
    stat(s, DEACTIVATE_BYPASS);
    goto new_slab;
  }

load_freelist:
  c->freelist = get_freepointer(s, freelist);
  c->tid = next_tid(c->tid);
  return freelist;

new_slab:
  // 2. try kmem_cache_cpu partial
  if (slub_percpu_partial(c)) { /* (c)->partial */
    page = c->page = slub_percpu_partial(c);
    slub_set_percpu_partial(c, page); /* slub_percpu_partial(c) = (p)->next; */
    stat(s, CPU_PARTIAL_ALLOC);
    goto redo;
  }

  // 3. try kmem_cache_node
  freelist = new_slab_objects(s, gfpflags, node, &c);
  return freelist;
}

static inline void *new_slab_objects(
  struct kmem_cache *s, gfp_t flags,
  int node, struct kmem_cache_cpu **pc)
{
  void *freelist;
  struct kmem_cache_cpu *c = *pc;
  struct page *page;

  // 3.1. try kmem_cache_node partial
  freelist = get_partial(s, flags, node, c); // -> get_partial_node()
  if (freelist)
    return freelist;

  // 3.2. alloc_page
  page = new_slab(s, flags, node);
  if (page) {
    c = raw_cpu_ptr(s->cpu_slab);
    if (c->page)
      flush_slab(s, c);

    freelist = page->freelist;
    page->freelist = NULL;

    stat(s, ALLOC_SLAB);
    c->page = page;
    *pc = c;
  } else
    freelist = NULL;

  return freelis
}

/* 3.1. get_partial -> */
static void *get_partial_node(
  struct kmem_cache *s, struct kmem_cache_node *n,
  struct kmem_cache_cpu *c, gfp_t flags)
{
  struct page *page, *page2;
  void *object = NULL;
  int available = 0;
  int objects;

  list_for_each_entry_safe(page, page2, &n->partial, lru) {
    void *t;
    t = acquire_slab(s, n, page, object == NULL, &objects);
    if (!t)
      break;

    available += objects;
    if (!object) {
      c->page = page;
      stat(s, ALLOC_FROM_PARTIAL);
      object = t;
    } else {
      put_cpu_partial(s, page, 0);
      stat(s, CPU_PARTIAL_NODE);
    }
    if (!kmem_cache_has_cpu_partial(s)
      || available > slub_cpu_partial(s) / 2)
      break;
  }

  return object;
}

static inline void *acquire_slab(struct kmem_cache *s,
    struct kmem_cache_node *n, struct page *page,
    int mode, int *objects)
{
  void *freelist;
  unsigned long counters;
  struct page new;

  freelist = page->freelist;
  counters = page->counters;
  new.counters = counters;
  *objects = new.objects - new.inuse;
  if (mode) {
    new.inuse = page->objects;
    new.freelist = NULL;
  } else {
    new.freelist = freelist;
  }

  new.frozen = 1;

  if (!__cmpxchg_double_slab(s, page,
      freelist, counters,
      new.freelist, new.counters,
      "acquire_slab"))
    return NULL;

  remove_partial(n, page);
  return freelist;
}

// 3.2. new_slab_objects -> new_slab, no memory in kmem_cache_node
static struct page *allocate_slab(struct kmem_cache *s, gfp_t flags, int node)
{
  struct page *page;
  struct kmem_cache_order_objects oo = s->oo;
  gfp_t alloc_gfp;
  void *start, *p;
  int idx, order;
  bool shuffle;

  flags &= gfp_allowed_mask;

  page = alloc_slab_page(s, alloc_gfp, node, oo);
  if (unlikely(!page)) {
    oo = s->min;
    alloc_gfp = flags;

    page = alloc_slab_page(s, alloc_gfp, node, oo);
    if (unlikely(!page))
      goto out;
    stat(s, ORDER_FALLBACK);
  }
  return page;
}

static inline struct page *alloc_slab_page(struct kmem_cache *s,
    gfp_t flags, int node, struct kmem_cache_order_objects oo)
{
  struct page *page;
  unsigned int order = oo_order(oo);

  if (node == NUMA_NO_NODE)
    page = alloc_pages(flags, order);
  else
    page = __alloc_pages_node(node, flags, order);

  if (page && memcg_charge_slab(page, flags, order, s)) {
    __free_pages(page, order);
    page = NULL;
  }

  return page;
}
```
```c++
slab_alloc()
  slab_alloc_node()
    if (cpu_slab->freelist)
      this_cpu_cmpxchg_double()
    else
      __slab_alloc()
        redo:
        // 1. try kmem_cache_cpu freelist

        new_slab:
        // 2. try kmem_cache_cpu partial
          if (slub_percpu_partial(c))
            goto redo

        // 3. try kmem_cache_node
          freelist = new_slab_objects()
            // 3.1 try kmem_cache_node partial
            get_partial()
              get_partial_node()
                list_for_each_entry_safe()
                  acquire_slab()
            // 3.2 alloc_page
            new_slab()
              allocate_slab()
                alloc_slab_page()
                  alloc_pages()
```

![](../Images/Kernel/mem-kmem-cache-alloc.png)

![](../Images/Kernel/mem-slub-structure.png)

![](../Images/Kernel/mem-mng.png)

* Reference:
  * [slaballocators.pdf](https://events.static.linuxfound.org/sites/events/files/slides/slaballocators.pdf)
  * [Slub allocator](https://www.cnblogs.com/LoyenWang/p/11922887.html)

# kswapd
```C++
//1. active page out when alloc
get_page_from_freelist();
    node_reclaim();
        __node_reclaim();
            shrink_node();

// 2. positive page out by kswapd
static int kswapd(void *p)
{
  unsigned int alloc_order, reclaim_order;
  unsigned int classzone_idx = MAX_NR_ZONES - 1;
  pg_data_t *pgdat = (pg_data_t*)p;
  struct task_struct *tsk = current;

  for ( ; ; ) {
    kswapd_try_to_sleep(pgdat, alloc_order, reclaim_order,
        classzone_idx);
    reclaim_order = balance_pgdat(pgdat, alloc_order, classzone_idx);
  }
}
// balance_pgdat->kswapd_shrink_node->shrink_node

/* This is a basic per-node page freer.  Used by both kswapd and direct reclaim. */
static void shrink_node_memcg(struct pglist_data *pgdat, struct mem_cgroup *memcg,
            struct scan_control *sc, unsigned long *lru_pages)
{
  unsigned long nr[NR_LRU_LISTS];
  enum lru_list lru;

  while (nr[LRU_INACTIVE_ANON] || nr[LRU_ACTIVE_FILE] ||
          nr[LRU_INACTIVE_FILE]) {
    unsigned long nr_anon, nr_file, percentage;
    unsigned long nr_scanned;

    for_each_evictable_lru(lru) {
      if (nr[lru]) {
        nr_to_scan = min(nr[lru], SWAP_CLUSTER_MAX);
        nr[lru] -= nr_to_scan;

        nr_reclaimed += shrink_list(lru, nr_to_scan,
                  lruvec, memcg, sc);
      }
    }
  }
}

/* There are two kinds pages:
 * Anonynmous page: mapped to virtual address space
 * File mapped page: mapped to both virtual address space and a file.
 *
 * Each kind page has active and inactive queues. */
enum lru_list {
  LRU_INACTIVE_ANON = LRU_BASE,
  LRU_ACTIVE_ANON   = LRU_BASE + LRU_ACTIVE,

  LRU_INACTIVE_FILE = LRU_BASE + LRU_FILE,
  LRU_ACTIVE_FILE   = LRU_BASE + LRU_FILE + LRU_ACTIVE,

  LRU_UNEVICTABLE,
  NR_LRU_LISTS
};

#define for_each_evictable_lru(lru) for (lru = 0; lru <= LRU_ACTIVE_FILE; lru++)

static unsigned long shrink_list(enum lru_list lru, unsigned long nr_to_scan,
         struct lruvec *lruvec, struct mem_cgroup *memcg,
         struct scan_control *sc)
{
  if (is_active_lru(lru)) {
    if (inactive_list_is_low(lruvec, is_file_lru(lru),
           memcg, sc, true))
      shrink_active_list(nr_to_scan, lruvec, sc, lru);
    return 0;
  }

  return shrink_inactive_list(nr_to_scan, lruvec, sc, lru);
}
```

# brk
```C++
/* mm/mmap.c */
SYSCALL_DEFINE1(brk, unsigned long, brk)
{
    unsigned long retval;
    unsigned long newbrk, oldbrk;
    struct mm_struct *mm = current->mm;
    struct vm_area_struct *next;

    newbrk = PAGE_ALIGN(brk);
    oldbrk = PAGE_ALIGN(mm->brk);
    if (oldbrk == newbrk)
        goto set_brk;

    /* Always allow shrinking brk. */
    if (brk <= mm->brk) {
        if (!do_munmap(mm, newbrk, oldbrk-newbrk, &uf))
            goto set_brk;
        goto out;
    }

    /* Check against existing mmap mappings. */
    next = find_vma(mm, oldbrk);
    if (next && newbrk + PAGE_SIZE > vm_start_gap(next))
        goto out;

    /* Ok, looks good - let it rip. */
    if (do_brk(oldbrk, newbrk-oldbrk, &uf) < 0)
        goto out;

set_brk:
  mm->brk = brk;
//
  return brk;
out:
  retval = mm->brk;
  return retval
}

static int do_brk(unsigned long addr, unsigned long len, struct list_head *uf)
{
  return do_brk_flags(addr, len, 0, uf);
}

int do_brk_flags(unsigned long addr, unsigned long len, unsigned long flags, struct list_head *uf)
{
  struct mm_struct *mm = current->mm;
  struct vm_area_struct *vma, *prev;
  struct rb_node **rb_link, *rb_parent;
  pgoff_t pgoff = addr >> PAGE_SHIFT;
  int error;

  error = get_unmapped_area(NULL, addr, len, 0, MAP_FIXED);
  if (offset_in_page(error))
    return error;

  /* Clear old maps.  this also does some error checking for us */
  while (find_vma_links(mm, addr, addr + len, &prev, &rb_link, &rb_parent)) {
    if (do_munmap(mm, addr, len, uf))
      return -ENOMEM;
  }

  /* Can we just expand an old private anonymous mapping? */
  vma = vma_merge(mm, prev, addr, addr + len, flags, NULL, NULL, pgoff, NULL, NULL_VM_UFFD_CTX);
  if (vma)
    goto out;

  /* create a vma struct for an anonymous mapping */
  vma = vm_area_alloc(mm);
  if (!vma) {
    vm_unacct_memory(len >> PAGE_SHIFT);
    return -ENOMEM;
  }

  vma_set_anonymous(vma);
  vma->vm_start = addr;
  vma->vm_end = addr + len;
  vma->vm_pgoff = pgoff;
  vma->vm_flags = flags;
  vma->vm_page_prot = vm_get_page_prot(flags);
  vma_link(mm, vma, prev, rb_link, rb_parent);

out:
  perf_event_mmap(vma);
  mm->total_vm += len >> PAGE_SHIFT;
  mm->data_vm += len >> PAGE_SHIFT;
  if (flags & VM_LOCKED)
    mm->locked_vm += (len >> PAGE_SHIFT);
  vma->vm_flags |= VM_SOFTDIRTY;
  return 0;
}

unsigned long get_unmapped_area(
  struct file *file, unsigned long addr, unsigned long len,
  unsigned long pgoff, unsigned long flags)
{
  unsigned long (*get_area)(struct file *, unsigned long,
          unsigned long, unsigned long, unsigned long);

  unsigned long error = arch_mmap_check(addr, len, flags);
  if (error)
    return error;

  /* Careful about overflows, 3G */
  if (len > TASK_SIZE)
    return -ENOMEM;

  get_area = current->mm->get_unmapped_area;
  if (file) {
    if (file->f_op->get_unmapped_area)
      get_area = file->f_op->get_unmapped_area;
  } else if (flags & MAP_SHARED) {
    /* mmap_region() will call shmem_zero_setup() to create a file,
     * so use shmem's get_unmapped_area in case it can be huge.
     * do_mmap_pgoff() will clear pgoff, so match alignment. */
    pgoff = 0;
    get_area = shmem_get_unmapped_area;
  }

  addr = get_area(file, addr, len, pgoff, flags);
  if (IS_ERR_VALUE(addr))
    return addr;

  if (addr > TASK_SIZE - len)
    return -ENOMEM;
  if (offset_in_page(addr))
    return -EINVAL;

  error = security_mmap_addr(addr);
  return error ? error : addr;
}
```

# mmap
![](../Images/Kernel/mem-mmap-vma-file-page.png)
```C++
struct mm_struct {
  pgd_t                 *pgd;
  struct vm_area_struct *mmap;  /* list of VMAs */
}

struct vm_area_struct {
  /* For areas with an address space and backing store,
   * linkage into the address_space->i_mmap interval tree. */
  struct {
    struct rb_node rb;
    unsigned long rb_subtree_last;
  } shared;

  /* A file's MAP_PRIVATE vma can be in both i_mmap tree and anon_vma list
   * An anonymous MAP_PRIVATE, stack or brk vma can only be in an anon_vma list.
   * A MAP_SHARED vma can only be in the i_mmap tree. */
  struct list_head anon_vma_chain;
  struct anon_vma *anon_vma;

  const struct vm_operations_struct *vm_ops;

  unsigned long vm_pgoff; /* Offset (within vm_file) in PAGE_SIZE units */
  struct file * vm_file;  /* File we map to (can be NULL). */
  void * vm_private_data; /* was vm_pte (shared mem) */
};

struct anon_vma_chain {
  struct vm_area_struct *vma;
  struct anon_vma *anon_vma;
  struct list_head same_vma;  /* locked by mmap_sem & page_table_lock */
  struct rb_node rb;          /* locked by anon_vma->rwsem */
  unsigned long rb_subtree_last;
};

struct anon_vma {
  struct anon_vma       *root;  /* Root of this anon_vma tree */
  struct rw_semaphore   rwsem;  /* W: modification, R: walking the list */
  atomic_t              refcount;
  unsigned              degree;
  struct anon_vma       *parent;  /* Parent of this anon_vma */
  struct rb_root_cached rb_root;
};

SYSCALL_DEFINE6(
  mmap, unsigned long, addr, unsigned long, len,
  unsigned long, prot, unsigned long, flags,
  unsigned long, fd, unsigned long, off)
{
  error = sys_mmap_pgoff(addr, len, prot, flags, fd, off >> PAGE_SHIFT);
}

SYSCALL_DEFINE6(mmap_pgoff, unsigned long, addr, unsigned long, len,
    unsigned long, prot, unsigned long, flags,
    unsigned long, fd, unsigned long, pgoff)
{
  struct file *file = NULL;

  file = fget(fd);

  retval = vm_mmap_pgoff(file, addr, len, prot, flags, pgoff);
  return retval;
}

/*  vm_mmap_pgoff -> do_mmap_pgoff -> do_mmap */
unsigned long do_mmap(struct file *file, unsigned long addr,
      unsigned long len, unsigned long prot,
      unsigned long flags, vm_flags_t vm_flags,
      unsigned long pgoff, unsigned long *populate,
      struct list_head *uf)
{
  addr = get_unmapped_area(file, addr, len, pgoff, flags);
  if (IS_ERR_VALUE(addr))
    return addr;

  addr = mmap_region(file, addr, len, vm_flags, pgoff, uf);
  return addr;
}

/* 1. get_unmapped_area */
unsigned long get_unmapped_area(
  struct file *file, unsigned long addr, unsigned long len,
  unsigned long pgoff, unsigned long flags)
{
  unsigned long (*get_area)(struct file *, unsigned long,
          unsigned long, unsigned long, unsigned long);
  get_area = current->mm->get_unmapped_area;
  if (file) {
    if (file->f_op->get_unmapped_area)
      get_area = file->f_op->get_unmapped_area;
  } else if (flags & MAP_SHARED) {
    get_area = shmem_get_unmapped_area;
  }
  addr = get_area(file, addr, len, pgoff, flags);
}

const struct file_operations ext4_file_operations = {
  .mmap               = ext4_file_mmap
  .get_unmapped_area  = thp_get_unmapped_area,
};

unsigned long __thp_get_unmapped_area(
  struct file *filp, unsigned long len,
  loff_t off, unsigned long flags, unsigned long size)
{
  unsigned long addr;
  loff_t off_end = off + len;
  loff_t off_align = round_up(off, size);
  unsigned long len_pad;
  len_pad = len + size;

  addr = current->mm->get_unmapped_area(
    filp, 0, len_pad, off >> PAGE_SHIFT, flags);
  addr += (off - addr) & (size - 1);
  return addr;
}

/* 2. mmap_region */
unsigned long mmap_region(struct file *file, unsigned long addr,
    unsigned long len, vm_flags_t vm_flags, unsigned long pgoff,
    struct list_head *uf)
{
  struct mm_struct *mm = current->mm;
  struct vm_area_struct *vma, *prev;
  struct rb_node **rb_link, *rb_parent;

  vma = vma_merge(mm, prev, addr, addr + len, vm_flags,
      NULL, file, pgoff, NULL, NULL_VM_UFFD_CTX);
  if (vma)
    goto out;

  vma = kmem_cache_zalloc(vm_area_cachep, GFP_KERNEL);

  vma->vm_mm = mm;
  vma->vm_start = addr;
  vma->vm_end = addr + len;
  vma->vm_flags = vm_flags;
  vma->vm_page_prot = vm_get_page_prot(vm_flags);
  vma->vm_pgoff = pgoff;
  INIT_LIST_HEAD(&vma->anon_vma_chain);

  if (file) {
    vma->vm_file = get_file(file);
    /* 2.1. link the file to vma */
    call_mmap(file, vma);
    addr = vma->vm_start;
    vm_flags = vma->vm_flags;
  } else if (vm_flags & VM_SHARED) {
    shmem_zero_setup(vma);
  } else {
    vma_set_anonymous(vma);
  }

  /* 2.2. link the vma to the file */
  vma_link(mm, vma, prev, rb_link, rb_parent);

  return addr;
}

static inline int call_mmap(struct file *file, struct vm_area_struct *vma)
{
  return file->f_op->mmap(file, vma);
}

static int ext4_file_mmap(struct file *file, struct vm_area_struct *vma)
{
  vma->vm_ops = &ext4_file_vm_ops;
}

struct address_space {
  struct inode      *host;
  /* tree of private and shared mappings. e.g., vm_area_struct */
  struct rb_root    i_mmap; // link the vma to the file
  const struct address_space_operations *a_ops;
}

/* 2.2. link the vma to the file */
static void vma_link(struct mm_struct *mm, struct vm_area_struct *vma,
      struct vm_area_struct *prev, struct rb_node **rb_link,
      struct rb_node *rb_parent)
{
  struct address_space *mapping = NULL;

  if (vma->vm_file) {
    mapping = vma->vm_file->f_mapping;
    i_mmap_lock_write(mapping);
  }

  __vma_link(mm, vma, prev, rb_link, rb_parent);
  __vma_link_file(vma);

  if (mapping)
    i_mmap_unlock_write(mapping);

  mm->map_count++;
  validate_mm(mm);
}

static void __vma_link_file(struct vm_area_struct *vma)
{
  struct file *file;

  file = vma->vm_file;
  if (file) {
    struct address_space *mapping = file->f_mapping;
    vma_interval_tree_insert(vma, &mapping->i_mmap);
  }
}

static void __vma_link(
  struct mm_struct *mm, struct vm_area_struct *vma,
  struct vm_area_struct *prev, struct rb_node **rb_link,
  struct rb_node *rb_parent)
{
  __vma_link_list(mm, vma, prev, rb_parent);
  __vma_link_rb(mm, vma, rb_link, rb_parent);
}
```

## get_unmapped_area
```c++
void setup_new_exec(struct linux_binprm * bprm)
{
  arch_pick_mmap_layout(current->mm, &bprm->rlim_stack);
}

void arch_pick_mmap_layout(struct mm_struct *mm, struct rlimit *rlim_stack)
{
  if (mmap_is_legacy())
    mm->get_unmapped_area = arch_get_unmapped_area;
  else
    mm->get_unmapped_area = arch_get_unmapped_area_topdown;

  arch_pick_mmap_base(&mm->mmap_base, &mm->mmap_legacy_base,
      arch_rnd(mmap64_rnd_bits), task_size_64bit(0),
      rlim_stack);
}

static void arch_pick_mmap_base(unsigned long *base, unsigned long *legacy_base,
    unsigned long random_factor, unsigned long task_size,
    struct rlimit *rlim_stack)
{
  *legacy_base = mmap_legacy_base(random_factor, task_size);
  if (mmap_is_legacy())
    *base = *legacy_base;
  else
    *base = mmap_base(random_factor, task_size, rlim_stack);
}

unsigned long mmap_base(unsigned long rnd, unsigned long task_size,
             struct rlimit *rlim_stack)
{
  unsigned long gap = rlim_stack->rlim_cur;
  unsigned long pad = stack_maxrandom_size(task_size) + stack_guard_gap;
  unsigned long gap_min, gap_max;

  /* Values close to RLIM_INFINITY can overflow. */
  if (gap + pad > gap)
    gap += pad;

  /* Top of mmap area (just below the process stack).
   * Leave an at least ~128 MB hole with possible stack randomization. */
  gap_min = SIZE_128M;
  gap_max = (task_size / 6) * 5;

  if (gap < gap_min)
    gap = gap_min;
  else if (gap > gap_max)
    gap = gap_max;

  return PAGE_ALIGN(task_size - gap - rnd);
}
```

```c++
unsigned long
arch_get_unmapped_area(struct file *filp, unsigned long addr,
    unsigned long len, unsigned long pgoff, unsigned long flags)
{
  struct mm_struct *mm = current->mm;
  struct vm_area_struct *vma;
  struct vm_unmapped_area_info info;
  unsigned long begin, end;

  addr = mpx_unmapped_area_check(addr, len, flags);
  if (IS_ERR_VALUE(addr))
    return addr;

  if (flags & MAP_FIXED)
    return addr;

  find_start_end(addr, flags, &begin, &end);

  if (len > end)
    return -ENOMEM;

  if (addr) {
    addr = PAGE_ALIGN(addr);
    vma = find_vma(mm, addr);
    if (end - len >= addr && (!vma || addr + len <= vm_start_gap(vma)))
      return addr;
  }

  info.flags = 0;
  info.length = len;
  info.low_limit = begin;
  info.high_limit = end;
  info.align_mask = 0;
  info.align_offset = pgoff << PAGE_SHIFT;
  if (filp) {
    info.align_mask = get_align_mask();
    info.align_offset += get_align_bits();
  }
  return vm_unmapped_area(&info);
}

static void find_start_end(unsigned long addr, unsigned long flags,
    unsigned long *begin, unsigned long *end)
{
  if (!in_compat_syscall() && (flags & MAP_32BIT)) {
    /* This is usually used needed to map code in small
      model, so it needs to be in the first 31bit. Limit
      it to that.  This means we need to move the
      unmapped base down for this case. This can give
      conflicts with the heap, but we assume that glibc
      malloc knows how to fall back to mmap. Give it 1GB
      of playground for now. -AK */
    *begin = 0x40000000;
    *end = 0x80000000;
    if (current->flags & PF_RANDOMIZE) {
      *begin = randomize_page(*begin, 0x02000000);
    }
    return;
  }

  *begin  = get_mmap_base(1);
  if (in_compat_syscall())
    *end = task_size_32bit();
  else
    *end = task_size_64bit(addr > DEFAULT_MAP_WINDOW);

  // TODO::?  shouldn't [begin, end) be [brk, mmap_base)
}

unsigned long get_mmap_base(int is_legacy)
{
  struct mm_struct *mm = current->mm;

#ifdef CONFIG_HAVE_ARCH_COMPAT_MMAP_BASES
  if (in_compat_syscall()) {
    return is_legacy ? mm->mmap_compat_legacy_base
         : mm->mmap_compat_base;
  }
#endif
  return is_legacy ? mm->mmap_legacy_base : mm->mmap_base;
}

/* Search for an unmapped address range.
 *
 * We are looking for a range that:
 * - does not intersect with any VMA;
 * - is contained within the [low_limit, high_limit) interval;
 * - is at least the desired size.
 * - satisfies (begin_addr & align_mask) == (align_offset & align_mask) */
unsigned long vm_unmapped_area(struct vm_unmapped_area_info *info)
{
  if (info->flags & VM_UNMAPPED_AREA_TOPDOWN)
    return unmapped_area_topdown(info);
  else
    return unmapped_area(info);
}

unsigned long unmapped_area(struct vm_unmapped_area_info *info)
{
  /* We implement the search by looking for an rbtree node that
   * immediately follows a suitable gap. That is,
   * - gap_start = vma->vm_prev->vm_end <= info->high_limit - length;
   * - gap_end   = vma->vm_start        >= info->low_limit  + length;
   * - gap_end - gap_start >= length */

  struct mm_struct *mm = current->mm;
  struct vm_area_struct *vma;
  unsigned long length, low_limit, high_limit, gap_start, gap_end;

  /* Adjust search length to account for worst case alignment overhead */
  length = info->length + info->align_mask;
  if (length < info->length)
    return -ENOMEM;

  /* Adjust search limits by the desired length */
  if (info->high_limit < length)
    return -ENOMEM;
  high_limit = info->high_limit - length;

  if (info->low_limit > high_limit)
    return -ENOMEM;
  low_limit = info->low_limit + length;

  /* Check if rbtree root looks promising */
  if (RB_EMPTY_ROOT(&mm->mm_rb))
    goto check_highest;
  vma = rb_entry(mm->mm_rb.rb_node, struct vm_area_struct, vm_rb);
  if (vma->rb_subtree_gap < length)
    goto check_highest;

  while (true) {
    /* Visit left subtree if it looks promising */
    gap_end = vm_start_gap(vma);
    if (gap_end >= low_limit && vma->vm_rb.rb_left) {
      struct vm_area_struct *left =
        rb_entry(vma->vm_rb.rb_left,
           struct vm_area_struct, vm_rb);
      if (left->rb_subtree_gap >= length) {
        vma = left;
        continue;
      }
    }

    gap_start = vma->vm_prev ? vm_end_gap(vma->vm_prev) : 0;
check_current:
    /* Check if current node has a suitable gap */
    if (gap_start > high_limit)
      return -ENOMEM;
    if (gap_end >= low_limit &&
        gap_end > gap_start && gap_end - gap_start >= length)
      goto found;

    /* Visit right subtree if it looks promising */
    if (vma->vm_rb.rb_right) {
      struct vm_area_struct *right =
        rb_entry(vma->vm_rb.rb_right,
           struct vm_area_struct, vm_rb);
      if (right->rb_subtree_gap >= length) {
        vma = right;
        continue;
      }
    }

    /* Go back up the rbtree to find next candidate node */
    while (true) {
      struct rb_node *prev = &vma->vm_rb;
      if (!rb_parent(prev))
        goto check_highest;
      vma = rb_entry(rb_parent(prev),
               struct vm_area_struct, vm_rb);
      if (prev == vma->vm_rb.rb_left) {
        gap_start = vm_end_gap(vma->vm_prev);
        gap_end = vm_start_gap(vma);
        goto check_current;
      }
    }
  }

check_highest:
  /* Check highest gap, which does not precede any rbtree node */
  gap_start = mm->highest_vm_end;
  gap_end = ULONG_MAX;  /* Only for VM_BUG_ON below */
  if (gap_start > high_limit)
    return -ENOMEM;

found:
  /* We found a suitable gap. Clip it with the original low_limit. */
  if (gap_start < info->low_limit)
    gap_start = info->low_limit;

  /* Adjust gap address to the desired alignment */
  gap_start += (info->align_offset - gap_start) & info->align_mask;

  return gap_start;
}
```

```C++
mmap();
  sys_mmap_pgoff();
    vm_mmap_pgoff();
      do_mmap_pgoff();
        do_mmap();

          get_unmapped_area();
            get_area = current->mm->get_unmapped_area;
            if (file) {
              if (file->f_op->get_unmapped_area)
                get_area = file->f_op->get_unmapped_area;
                  __thp_get_unmapped_area();
                    current->mm->get_unmapped_area();
            } else if (flags & MAP_SHARED) {
              get_area = shmem_get_unmapped_area;
            }
            addr = get_area(file, addr, len, pgoff, flags);

          map_region();
            vma_merge();

            struct vm_area_struct *vma = kmem_cache_zalloc();

            if (file) {
              vma->vm_file = get_file(file);
              /* 2.1. link the file to vma */
              call_mmap(file, vma);
                file->f_op->mmap(file, vma);
                  ext4_file_mmap();
                    vma->vm_ops = &ext4_file_vm_ops;
            } else if (vm_flags & VM_SHARED) {
              shmem_zero_setup(vma);
            } else {
              vma_set_anonymous(vma);
            }

            /* 2.2. link the vma to the file */
            vma_link(mm, vma, prev, rb_link, rb_parent);
              vma_interval_tree_insert(vma, &mapping->i_mmap);


setup_new_exec();
  arch_pick_mmap_layout();
    mm->get_unmapped_area = arch_get_unmapped_area;
    arch_pick_mmap_base();
      mmap_base();

mm->get_unmapped_area();
  arch_get_unmapped_area();
    find_start_end();
      *begin  = get_mmap_base(1);
        return mm->mmap_base;
      *end = task_size_64bit(addr > DEFAULT_MAP_WINDOW);
    vm_unmapped_area();
      unmapped_area();
```

# page fault
```C++
struct file {
  struct file_operations* f_op;
  struct address_space*   f_mapping;
};

// page cache in memory
struct address_space {
  struct inode            *host;
  struct radix_tree_root  i_pages; /* cached physical pages */
  struct rb_root_cached   i_mmap;  /* tree of private and shared vma mappings */
  struct rw_semaphore     i_mmap_rwsem;
  atomic_t                i_mmap_writable;      /* count VM_SHARED mappings */
  const struct address_space_operations *a_ops; /* methods */
  void                    *private_data;

  unsigned long           nrpages;
  unsigned long           nrexceptional;
  pgoff_t                 writeback_index;/* writeback starts here */
  struct list_head        private_list;   /* for use by the address_space */
};

static void __init kvm_apf_trap_init(void)
{
  update_intr_gate(X86_TRAP_PF, async_page_fault);
}

ENTRY(async_page_fault)
  ASM_CLAC
  pushl $do_async_page_fault
  jmp   common_exception
END(async_page_fault)

ENTRY(page_fault)
  ASM_CLAC
  pushl  $do_page_fault
  ALIGN
  jmp common_exception
END(page_fault)

common_exception:
  /* the function address is in %gs's slot on the stack */
  pushl  %fs
  pushl  %es
  pushl  %ds
  pushl  %eax
  movl   $(__USER_DS), %eax
  movl  %eax, %ds
  movl  %eax, %es
  movl  $(__KERNEL_PERCPU), %eax
  movl  %eax, %fs
  pushl  %ebp
  pushl  %edi
  pushl  %esi
  pushl  %edx
  pushl  %ecx
  pushl  %ebx
  SWITCH_TO_KERNEL_STACK
  ENCODE_FRAME_POINTER
  cld
  UNWIND_ESPFIX_STACK
  GS_TO_REG %ecx
  movl  PT_GS(%esp), %edi    # get the function address
  movl  PT_ORIG_EAX(%esp), %edx    # get the error code
  movl  $-1, PT_ORIG_EAX(%esp)    # no syscall to restart
  REG_TO_PTGS %ecx
  SET_KERNEL_GS %ecx
  TRACE_IRQS_OFF
  movl  %esp, %eax      # pt_regs pointer
  CALL_NOSPEC %edi
  jmp  ret_from_exception
END(common_exception)

void do_async_page_fault(struct pt_regs *regs, unsigned long error_code)
{
  enum ctx_state prev_state;

  switch (kvm_read_and_reset_pf_reason()) {
  default:
    do_page_fault(regs, error_code);
    break;
  }
}

// linux-4.19.y/arch/x86/mm/fault.c

void do_page_fault(struct pt_regs *regs, unsigned long error_code)
{
  unsigned long address = read_cr2();
  enum ctx_state prev_state;

  prev_state = exception_enter();
  if (trace_pagefault_enabled())
    trace_page_fault_entries(address, regs, error_code);

  __do_page_fault(regs, error_code, address);
  exception_exit(prev_state);
}

void __do_page_fault(struct pt_regs *regs, unsigned long error_code,
    unsigned long address)
{
  struct vm_area_struct *vma;
  struct task_struct *tsk;
  struct mm_struct *mm;
  vm_fault_t fault, major = 0;
  unsigned int flags = FAULT_FLAG_ALLOW_RETRY | FAULT_FLAG_KILLABLE;
  u32 pkey;

  tsk = current;
  mm = tsk->mm;

  if (unlikely(fault_in_kernel_space(address))) {
    if (!(error_code & (X86_PF_RSVD | X86_PF_USER | X86_PF_PROT))) {
      if (vmalloc_fault(address) >= 0)
        return;
    }

    return;
  }

  /* It's safe to allow irq's after cr2 has been saved and the
   * vmalloc fault has been handled.
   *
   * User-mode registers count as a user access even for any
   * potential system fault or CPU buglet: */
  if (user_mode(regs)) {
    local_irq_enable();
    error_code |= X86_PF_USER;
    flags |= FAULT_FLAG_USER;
  } else {
    if (regs->flags & X86_EFLAGS_IF)
      local_irq_enable();
  }

  if (error_code & X86_PF_WRITE)
    flags |= FAULT_FLAG_WRITE;
  if (error_code & X86_PF_INSTR)
    flags |= FAULT_FLAG_INSTRUCTION;

retry:
  vma = find_vma(mm, address);
  if (unlikely(!vma)) {
    bad_area(regs, error_code, address);
    return;
  }
  if (likely(vma->vm_start <= address))
    goto good_area;
  if (unlikely(!(vma->vm_flags & VM_GROWSDOWN))) {
    bad_area(regs, error_code, address);
    return;
  }

good_area:

  pkey = vma_pkey(vma);
  fault = handle_mm_fault(vma, address, flags);
  major |= fault & VM_FAULT_MAJOR;
}

static int __handle_mm_fault(
  struct vm_area_struct *vma,
  unsigned long address,
  unsigned int flags)
{
  struct vm_fault vmf = {
    .vma = vma,
    .address = address & PAGE_MASK,
    .flags = flags,
    .pgoff = linear_page_index(vma, address),
    .gfp_mask = __get_fault_gfp_mask(vma),
  };

  struct mm_struct *mm = vma->vm_mm;
  pgd_t *pgd;
  p4d_t *p4d;
  int ret;

  pgd = pgd_offset(mm, address);
  p4d = p4d_alloc(mm, pgd, address);

  vmf.pud = pud_alloc(mm, p4d, address);
  vmf.pmd = pmd_alloc(mm, vmf.pud, address);

  return handle_pte_fault(&vmf);
}

static int handle_pte_fault(struct vm_fault *vmf)
{
  pte_t entry;
  vmf->pte = pte_offset_map(vmf->pmd, vmf->address);
  vmf->orig_pte = *vmf->pte;

  if (!vmf->pte) {
    if (vma_is_anonymous(vmf->vma)) /* return !vma->vm_ops; */
      return do_anonymous_page(vmf);
    else
      return do_fault(vmf);
  }

  if (!pte_present(vmf->orig_pte))
    return do_swap_page(vmf);
}
```

## do_anonymous_page
```c++
/* 1. map to anonymouse page */
static vm_fault_t do_anonymous_page(struct vm_fault *vmf)
{
  struct vm_area_struct *vma = vmf->vma;
  struct mem_cgroup *memcg;
  struct page *page;
  vm_fault_t ret = 0;
  pte_t entry;

  /* File mapping without ->vm_ops ? */
  if (vma->vm_flags & VM_SHARED)
    return VM_FAULT_SIGBUS;

  if (pte_alloc(vma->vm_mm, vmf->pmd, vmf->address))
    return VM_FAULT_OOM;

  /* See the comment in pte_alloc_one_map() */
  if (unlikely(pmd_trans_unstable(vmf->pmd)))
    return 0;

  /* Allocate our own private page. */
  if (unlikely(anon_vma_prepare(vma)))
    goto oom;

  page = alloc_zeroed_user_highpage_movable(vma, vmf->address);
  if (!page)
    goto oom;

  if (mem_cgroup_try_charge_delay(page, vma->vm_mm, GFP_KERNEL, &memcg,
          false))
    goto oom_free_page;

  __SetPageUptodate(page);

  entry = mk_pte(page, vma->vm_page_prot);

  /* if can write page, set the wr flag in pte */
  if (vma->vm_flags & VM_WRITE)
    entry = pte_mkwrite(pte_mkdirty(entry));

  vmf->pte = pte_offset_map_lock(vma->vm_mm, vmf->pmd, vmf->address,
      &vmf->ptl);
  if (!pte_none(*vmf->pte))
    goto release;

  /* add map from physic memory page to virtual address
   * add pte mapping to a new anonymous page
   * rmap: Reverse Mapping */
  page_add_new_anon_rmap(page, vma, vmf->address, false);

  mem_cgroup_commit_charge(page, memcg, false, false);
  lru_cache_add_active_or_unevictable(page, vma);

setpte:
  set_pte_at(vma->vm_mm, vmf->address, vmf->pte, entry);

  /* No need to invalidate - it was non-present before */
  update_mmu_cache(vma, vmf->address, vmf->pte);

  return VM_FAULT_OOM;
}

static inline pte_t mk_pte(struct page *page, pgprot_t prot)
{
  return pfn_pte(page_to_pfn(page), prot);
}

void page_add_new_anon_rmap(struct page *page,
  struct vm_area_struct *vma, unsigned long address, bool compound)
{
  int nr = compound ? hpage_nr_pages(page) : 1;

  __mod_node_page_state(page_pgdat(page), NR_ANON_MAPPED, nr);
  __page_set_anon_rmap(page, vma, address, 1);
}

static void __page_set_anon_rmap(struct page *page,
  struct vm_area_struct *vma, unsigned long address, int exclusive)
{
  struct anon_vma *anon_vma = vma->anon_vma;

  if (PageAnon(page))
    return;

  if (!exclusive)
    anon_vma = anon_vma->root;

  anon_vma = (void *) anon_vma + PAGE_MAPPING_ANON;
  page->mapping = (struct address_space *) anon_vma;
  page->index = linear_page_index(vma, address);
}

void *alloc_zeroed_user_highpage(
  gfp_t movableflags,
  struct vm_area_struct *vma,
  unsigned long vaddr)
{
  struct page *page = alloc_page_vma(GFP_HIGHUSER | movableflags,
      vma, vaddr);

  if (page)
    clear_user_highpage(page, vaddr);

  return page;
}

#define alloc_page_vma(gfp_mask, vma, addr)      \
  alloc_pages_vma(gfp_mask, 0, vma, addr, numa_node_id(), false)

struct page *alloc_pages_vma(gfp_t gfp, int order, struct vm_area_struct *vma,
    unsigned long addr, int node, bool hugepage)
{
  struct mempolicy *pol;
  struct page *page;
  int preferred_nid;
  nodemask_t *nmask;

  pol = get_vma_policy(vma, addr);

  nmask = policy_nodemask(gfp, pol);
  preferred_nid = policy_node(gfp, pol, node);
  page = __alloc_pages_nodemask(gfp, order, preferred_nid, nmask);
  mpol_cond_put(pol);
out:
  return page;
}
```

## do_fault
```C++
/* 2. map to a file */
static vm_fault_t do_fault(struct vm_fault *vmf)
{
  struct vm_area_struct *vma = vmf->vma;
  struct mm_struct *vm_mm = vma->vm_mm;
  vm_fault_t ret;

  if (!vma->vm_ops->fault) {
    if (unlikely(!pmd_present(*vmf->pmd)))
      ret = VM_FAULT_SIGBUS;
    else {
      vmf->pte = pte_offset_map_lock(vmf->vma->vm_mm,
                   vmf->pmd,
                   vmf->address,
                   &vmf->ptl);
      if (unlikely(pte_none(*vmf->pte)))
        ret = VM_FAULT_SIGBUS;
      else
        ret = VM_FAULT_NOPAGE;

      pte_unmap_unlock(vmf->pte, vmf->ptl);
    }
  } else if (!(vmf->flags & FAULT_FLAG_WRITE))
    ret = do_read_fault(vmf);
  else if (!(vma->vm_flags & VM_SHARED))
    ret = do_cow_fault(vmf);
  else
    ret = do_shared_fault(vmf);

  /* preallocated pagetable is unused: free it */
  if (vmf->prealloc_pte) {
    pte_free(vm_mm, vmf->prealloc_pte);
    vmf->prealloc_pte = NULL;
  }
  return ret;
}

static vm_fault_t do_read_fault(struct vm_fault *vmf)
{
  struct vm_area_struct *vma = vmf->vma;
  vm_fault_t ret = 0;

  if (vma->vm_ops->map_pages && fault_around_bytes >> PAGE_SHIFT > 1) {
    ret = do_fault_around(vmf);
    if (ret)
      return ret;
  }

  ret = __do_fault(vmf);

  ret |= finish_fault(vmf);

  return ret;
}


static int __do_fault(struct vm_fault *vmf)
{
  struct vm_area_struct *vma = vmf->vma;
  if (pmd_none(*vmf->pmd) && !vmf->prealloc_pte) {
    vmf->prealloc_pte = pte_alloc_one(vmf->vma->vm_mm);
    if (!vmf->prealloc_pte)
      return VM_FAULT_OOM;
  }

  int ret;
  ret = vma->vm_ops->fault(vmf);
  return ret;
}

static const struct vm_operations_struct ext4_file_vm_ops = {
  .fault        = ext4_filemap_fault,
  .map_pages    = filemap_map_pages,
  .page_mkwrite = ext4_page_mkwrite,
};

int ext4_filemap_fault(struct vm_fault *vmf)
{
  int err;
  struct inode *inode = file_inode(vmf->vma->vm_file);

  down_read(&EXT4_I(inode)->i_mmap_sem);
  err = filemap_fault(vmf);
  up_read(&EXT4_I(inode)->i_mmap_sem);

  return err;
}

/* read in file data for page fault handling */
int filemap_fault(struct vm_fault *vmf)
{
  int error;
  struct file *file = vmf->vma->vm_file;
  struct address_space *mapping = file->f_mapping;
  struct inode *inode = mapping->host;
  pgoff_t offset = vmf->pgoff;
  struct page *page;
  int ret = 0;

  page = find_get_page(mapping, offset); // find the physical cache page
  if (likely(page) && !(vmf->flags & FAULT_FLAG_TRIED)) {
    do_async_mmap_readahead(vmf->vma, ra, file, page, offset);
  } else if (!page) {
    /* Synchronous readahead happens when we don't even find
     * a page in the page cache at all.*/
    do_sync_mmap_readahead(vmf->vma, ra, file, offset);

retry_find:
    page = find_get_page(mapping, offset);
    if (!page)
      goto no_cached_page;
  }

  vmf->page = page;
  return ret | VM_FAULT_LOCKED;
no_cached_page:
  error = page_cache_read(file, offset, vmf->gfp_mask);
}

/* This adds the requested page to the page cache if it isn't already there,
 * and schedules an I/O to read in its contents from disk. */
static int page_cache_read(struct file *file, pgoff_t offset, gfp_t gfp_mask)
{
  struct address_space *mapping = file->f_mapping;
  struct page *page;

  page = __page_cache_alloc(gfp_mask|__GFP_COLD); // invoke buddy system to alloc physical page
  ret = add_to_page_cache_lru(page, mapping, offset, gfp_mask & GFP_KERNEL);
  ret = mapping->a_ops->readpage(file, page);
}

static const struct address_space_operations ext4_aops = {
  .readpage   = ext4_readpage,
  .readpages  = ext4_readpages,
};

static int ext4_read_inline_page(struct inode *inode, struct page *page)
{
  void *kaddr;

  kaddr = kmap_atomic(page);
  ret = ext4_read_inline_data(inode, kaddr, len, &iloc);
  flush_dcache_page(page);
  kunmap_atomic(kaddr);
}

/* finish page fault once we have prepared the page to fault */
vm_fault_t finish_fault(struct vm_fault *vmf)
{
  struct page *page;
  vm_fault_t ret = 0;

  /* Did we COW the page? */
  if ((vmf->flags & FAULT_FLAG_WRITE) &&
      !(vmf->vma->vm_flags & VM_SHARED))
    page = vmf->cow_page;
  else
    page = vmf->page;

  if (!(vmf->vma->vm_flags & VM_SHARED))
    ret = check_stable_address_space(vmf->vma->vm_mm);
  if (!ret)
    ret = alloc_set_pte(vmf, vmf->memcg, page);
  if (vmf->pte)
    pte_unmap_unlock(vmf->pte, vmf->ptl);
  return ret;
}

#define pte_unmap_unlock(pte, ptl)  do {    \
  spin_unlock(ptl);        \
  pte_unmap(pte);          \
} while (0)

/* setup new PTE entry for given page and add reverse page mapping */
vm_fault_t alloc_set_pte(struct vm_fault *vmf, struct mem_cgroup *memcg,
    struct page *page)
{
  struct vm_area_struct *vma = vmf->vma;
  bool write = vmf->flags & FAULT_FLAG_WRITE;
  pte_t entry;
  vm_fault_t ret;

  if (pmd_none(*vmf->pmd) && PageTransCompound(page) &&
      IS_ENABLED(CONFIG_TRANSPARENT_HUGE_PAGECACHE)) {

    ret = do_set_pmd(vmf, page);
    if (ret != VM_FAULT_FALLBACK)
      return ret;
  }

  if (!vmf->pte) {
    ret = pte_alloc_one_map(vmf);
    if (ret)
      return ret;
  }

  /* Re-check under ptl */
  if (unlikely(!pte_none(*vmf->pte)))
    return VM_FAULT_NOPAGE;

  flush_icache_page(vma, page);
  entry = mk_pte(page, vma->vm_page_prot);
  if (write)
    entry = maybe_mkwrite(pte_mkdirty(entry), vma);

  /* copy-on-write page */
  if (write && !(vma->vm_flags & VM_SHARED)) {
    inc_mm_counter_fast(vma->vm_mm, MM_ANONPAGES);
    page_add_new_anon_rmap(page, vma, vmf->address, false);
    mem_cgroup_commit_charge(page, memcg, false, false);
    lru_cache_add_active_or_unevictable(page, vma);
  } else {
    inc_mm_counter_fast(vma->vm_mm, mm_counter_file(page));
    page_add_file_rmap(page, false);
  }

  set_pte_at(vma->vm_mm, vmf->address, vmf->pte, entry);

  /* no need to invalidate: a not-present page won't be cached */
  update_mmu_cache(vma, vmf->address, vmf->pte);

  return 0;
}
```

## do_swap_page
```c++
// 3. map to a swap
int do_swap_page(struct vm_fault *vmf)
{
  struct vm_area_struct *vma = vmf->vma;
  struct page *page, *swapcahe;
  struct mem_cgroup *memcg;c
  swp_entry_t entry;
  pte_t pte;

  entry = pte_to_swp_entry(vmf->orig_pte);
  page = lookup_swap_cache(entry);
  if (!page) {
    page = swapin_readahead(entry, GFP_HIGHUSER_MOVABLE, vma,
          vmf->address);
  }

  swapcache = page;
  pte = mk_pte(page, vma->vm_page_prot);
  set_pte_at(vma->vm_mm, vmf->address, vmf->pte, pte);
  vmf->orig_pte = pte;
  swap_free(entry);
}

// swapin_readahead ->
int swap_readpage(struct page *page, bool do_poll)
{
  struct bio *bio;
  int ret = 0;
  struct swap_info_struct *sis = page_swap_info(page);
  blk_qc_t qc;
  struct block_device *bdev;

  if (sis->flags & SWP_FILE) {
    struct file *swap_file = sis->swap_file;
    struct address_space *mapping = swap_file->f_mapping;
    ret = mapping->a_ops->readpage(swap_file, page);
    return ret;
  }
}
```

```c++
do_page_fault();
  __do_page_fault();
    vmalloc_fault(); /* kernel fault */

    vma = find_vma(mm, address);

    handel_mm_fault();
      handle_pte_fault();

        /* 1. anonymous fault */
        do_anonymous_page();
          pte_alloc();
          alloc_zeroed_user_highpage_movable();
            alloc_pages_vma();
                __alloc_pages_nodemask();
                  get_page_from_freelist();
          mk_pte();
          page_add_new_anon_rmap()
            __page_set_anon_rmap();
              anon_vma = vma->anon_vma;
              page->mapping = (struct address_space *) anon_vma;
          set_pte_at()
          update_mmu_cache()

        /* 2. file fault */
        do_fault()

          do_read_fault()
            __do_fault();
              vma->vm_ops->fault();
                ext4_filemap_fault();
                  filemap_fault();
                    page = find_get_page();
                    if (page) {
                      do_async_mmap_readahead();
                    } else if (!page) {
                      do_async_mmap_readahead();
                    }

                    page_cache_read();
                      page = __page_cache_alloc();
                      address_space.a_ops.readpage();
                        ext4_read_inline_page();
                          kmap_atomic();
                          ext4_read_inline_data();
                          kumap_atomic();
            finish_fault()
              alloc_set_pte()
              pte_unmap_unlock()

          do_cow_fault()

          do_shared_fault()

        /* 3. swap fault */
        do_swap_page();
```
![](../Images/Kernel/mem-page-fault.png)

# pgd
`cr3` register points to current process's `pgd`, which is set by `load_new_mm_cr3`.
```C++
/* alloc pgd in mm_struct when forking */
static struct mm_struct *dup_mm(struct task_struct *tsk)
{
  struct mm_struct *mm, *oldmm = current->mm;
  mm = allocate_mm();
  memcpy(mm, oldmm, sizeof(*mm));
  if (!mm_init(mm, tsk, mm->user_ns))
    goto fail_nomem;
  err = dup_mmap(mm, oldmm);
  return mm;
}
// mm_init->
static inline int mm_alloc_pgd(struct mm_struct *mm)
{
  mm->pgd = pgd_alloc(mm);
  return 0;
}

static void pgd_ctor(struct mm_struct *mm, pgd_t *pgd)
{
  /* If the pgd points to a shared pagetable level (either the
     ptes in non-PAE, or shared PMD in PAE), then just copy the
     references from swapper_pg_dir.
     swapper_pg_dir: kernel's pgd */
  if (CONFIG_PGTABLE_LEVELS == 2
    || (CONFIG_PGTABLE_LEVELS == 3 && SHARED_KERNEL_PMD)
    || CONFIG_PGTABLE_LEVELS >= 4) {
    clone_pgd_range(pgd + KERNEL_PGD_BOUNDARY,
        swapper_pg_dir + KERNEL_PGD_BOUNDARY,
        KERNEL_PGD_PTRS);
  }
}
```

# kernel mapping
```C++
// arch/x86/include/asm/pgtable_64.h
extern pud_t level3_kernel_pgt[512];
extern pud_t level3_ident_pgt[512];

extern pmd_t level2_kernel_pgt[512];
extern pmd_t level2_fixmap_pgt[512];
extern pmd_t level2_ident_pgt[512];

extern pte_t level1_fixmap_pgt[512];
extern pgd_t init_top_pgt[];

#define swapper_pg_dir init_top_pgt

// arch\x86\kernel\head_64.S
__INITDATA
NEXT_PAGE(init_top_pgt)
  .quad   level3_ident_pgt - __START_KERNEL_map + _KERNPG_TABLE
  .org    init_top_pgt + PGD_PAGE_OFFSET*8, 0
  .quad   level3_ident_pgt - __START_KERNEL_map + _KERNPG_TABLE
  .org    init_top_pgt + PGD_START_KERNEL*8, 0
  /* (2^48-(2*1024*1024*1024))/(2^39) = 511 */
  .quad   level3_kernel_pgt - __START_KERNEL_map + _PAGE_TABLE

NEXT_PAGE(level3_ident_pgt)
  .quad  level2_ident_pgt - __START_KERNEL_map + _KERNPG_TABLE
  .fill  511, 8, 0
NEXT_PAGE(level2_ident_pgt)
  /* Since I easily can, map the first 1G.
   * Don't set NX because code runs from these pages.
   */
  PMDS(0, __PAGE_KERNEL_IDENT_LARGE_EXEC, PTRS_PER_PMD)


NEXT_PAGE(level3_kernel_pgt)
  .fill  L3_START_KERNEL,8,0
  /* (2^48-(2*1024*1024*1024)-((2^39)*511))/(2^30) = 510 */
  .quad  level2_kernel_pgt - __START_KERNEL_map + _KERNPG_TABLE
  .quad  level2_fixmap_pgt - __START_KERNEL_map + _PAGE_TABLE


NEXT_PAGE(level2_kernel_pgt)
  /* 512 MB kernel mapping. We spend a full page on this pagetable
   * anyway.
   *
   * The kernel code+data+bss must not be bigger than that.
   *
   * (NOTE: at +512MB starts the module area, see MODULES_VADDR.
   *  If you want to increase this then increase MODULES_VADDR
   *  too.) */
  PMDS(0, __PAGE_KERNEL_LARGE_EXEC,
    KERNEL_IMAGE_SIZE/PMD_SIZE)


NEXT_PAGE(level2_fixmap_pgt)
  .fill  506,8,0
  .quad  level1_fixmap_pgt - __START_KERNEL_map + _PAGE_TABLE
  /* 8MB reserved for vsyscalls + a 2MB hole = 4 + 1 entries */
  .fill  5,8,0


NEXT_PAGE(level1_fixmap_pgt)
  .fill  51


PGD_PAGE_OFFSET = pgd_index(__PAGE_OFFSET_BASE)
PGD_START_KERNEL = pgd_index(__START_KERNEL_map)
L3_START_KERNEL = pud_index(__START_KERNEL_map)
```
![](../Images/Kernel/mem-kernel-page-table.png)

```C++
// kernel mm_struct
struct mm_struct init_mm = {
  .mm_rb      = RB_ROOT,
  .pgd        = swapper_pg_dir,
  .mm_users   = ATOMIC_INIT(2),
  .mm_count   = ATOMIC_INIT(1),
  .mmap_sem   = __RWSEM_INITIALIZER(init_mm.mmap_sem),
  .page_table_lock =  __SPIN_LOCK_UNLOCKED(init_mm.page_table_lock),
  .mmlist     = LIST_HEAD_INIT(init_mm.mmlist),
  .user_ns    = &init_user_ns,
  INIT_MM_CONTEXT(init_mm)
};

// init kernel mm_struct
void __init setup_arch(char **cmdline_p)
{
  clone_pgd_range(swapper_pg_dir + KERNEL_PGD_BOUNDARY,
      initial_page_table + KERNEL_PGD_BOUNDARY,
      KERNEL_PGD_PTRS);

  load_cr3(swapper_pg_dir);
  __flush_tlb_all();

  init_mm.start_code = (unsigned long) _text;
  init_mm.end_code = (unsigned long) _etext;
  init_mm.end_data = (unsigned long) _edata;
  init_mm.brk = _brk_end;
  init_mem_mapping();
}

// init_mem_mapping ->
unsigned long kernel_physical_mapping_init(
  unsigned long paddr_start,
  unsigned long paddr_end,
  unsigned long page_size_mask)
{
  unsigned long vaddr, vaddr_start, vaddr_end, vaddr_next, paddr_last;

  paddr_last = paddr_end;
  vaddr = (unsigned long)__va(paddr_start);
  vaddr_end = (unsigned long)__va(paddr_end);
  vaddr_start = vaddr;

  for (; vaddr < vaddr_end; vaddr = vaddr_next) {
    pgd_t *pgd = pgd_offset_k(vaddr);
    p4d_t *p4d;

    vaddr_next = (vaddr & PGDIR_MASK) + PGDIR_SIZE;

    if (pgd_val(*pgd)) {
      p4d = (p4d_t *)pgd_page_vaddr(*pgd);
      paddr_last = phys_p4d_init(p4d, __pa(vaddr),
               __pa(vaddr_end),
               page_size_mask);
      continue;
    }

    p4d = alloc_low_page();
    paddr_last = phys_p4d_init(p4d, __pa(vaddr), __pa(vaddr_end),
             page_size_mask);

    p4d_populate(&init_mm, p4d_offset(pgd, vaddr), (pud_t *) p4d);
  }
  __flush_tlb_all();

  return paddr_last;
}
```

# kmalloc
```c++
/* kmalloc is the normal method of allocating memory
 * for objects smaller than page size in the kernel. */
static void *kmalloc(size_t size, gfp_t flags)
{
  if (__builtin_constant_p(size)) {
    if (size > KMALLOC_MAX_CACHE_SIZE)
      return kmalloc_large(size, flags);

#ifndef CONFIG_SLOB
    if (!(flags & GFP_DMA)) {
      unsigned int index = kmalloc_index(size);

      if (!index)
        return ZERO_SIZE_PTR;

      return kmem_cache_alloc_trace(kmalloc_caches[index],
          flags, size);
    }
#endif
  }

  return __kmalloc(size, flags);
}

/* slub.c */
void *__kmalloc(size_t size, gfp_t flags)
{
  struct kmem_cache *s;
  void *ret;

  if (unlikely(size > KMALLOC_MAX_CACHE_SIZE))
    return kmalloc_large(size, flags);

  s = kmalloc_slab(size, flags);
  if (unlikely(ZERO_OR_NULL_PTR(s)))
    return s;

  ret = slab_alloc(s, flags, _RET_IP_);

  return ret;
}
```

```c++
static __always_inline void *kmalloc_large(size_t size, gfp_t flags)
{
  unsigned int order = get_order(size);
  return kmalloc_order_trace(size, flags, order);
}

static __always_inline void *
kmalloc_order_trace(size_t size, gfp_t flags, unsigned int order)
{
  return kmalloc_order(size, flags, order);
}

/* To avoid unnecessary overhead, we pass through large allocation requests
 * directly to the page allocator. We use __GFP_COMP, because we will need to
 * know the allocation order to free the pages properly in kfree. */
void *kmalloc_order(size_t size, gfp_t flags, unsigned int order)
{
  void *ret;
  struct page *page;

  flags |= __GFP_COMP;
  page = alloc_pages(flags, order);
  ret = page ? page_address(page) : NULL;
  kmemleak_alloc(ret, size, 1, flags);
  kasan_kmalloc_large(ret, size, flags);
  return ret;
}
```

## kmalloc_caches
```c++
/* linux-4.19.y/mm/slab_common.c */
struct kmem_cache *kmalloc_caches[KMALLOC_SHIFT_HIGH + 1];

void kmem_cache_init(void)
{
  setup_kmalloc_cache_index_table();
  create_kmalloc_caches(0);
}

void create_kmalloc_caches(slab_flags_t flags)
{
  int i;

  for (i = KMALLOC_SHIFT_LOW; i <= KMALLOC_SHIFT_HIGH; i++) {
    if (!kmalloc_caches[i])
      new_kmalloc_cache(i, flags);

    if (KMALLOC_MIN_SIZE <= 32 && !kmalloc_caches[1] && i == 6)
      new_kmalloc_cache(1, flags);
    if (KMALLOC_MIN_SIZE <= 64 && !kmalloc_caches[2] && i == 7)
      new_kmalloc_cache(2, flags);
  }

  /* Kmalloc array is now usable */
  slab_state = UP;

#ifdef CONFIG_ZONE_DMA
  for (i = 0; i <= KMALLOC_SHIFT_HIGH; i++) {
    struct kmem_cache *s = kmalloc_caches[i];

    if (s) {
      unsigned int size = kmalloc_size(i);
      kmalloc_dma_caches[i] = create_kmalloc_cache(n,
        size, SLAB_CACHE_DMA | flags, 0, 0);
    }
  }
#endif
}

void new_kmalloc_cache(int idx, slab_flags_t flags)
{
  kmalloc_caches[idx] = create_kmalloc_cache(kmalloc_info[idx].name,
          kmalloc_info[idx].size, flags, 0,
          kmalloc_info[idx].size);
}

const struct kmalloc_info_struct kmalloc_info[] __initconst = {
  {NULL,                      0},    {"kmalloc-96",             96},
  {"kmalloc-192",           192},    {"kmalloc-8",               8},
  {"kmalloc-16",             16},    {"kmalloc-32",             32},
  {"kmalloc-64",             64},    {"kmalloc-128",           128},
  {"kmalloc-256",           256},    {"kmalloc-512",           512},
  {"kmalloc-1024",         1024},    {"kmalloc-2048",         2048},
  {"kmalloc-4096",         4096},    {"kmalloc-8192",         8192},
  {"kmalloc-16384",       16384},    {"kmalloc-32768",       32768},
  {"kmalloc-65536",       65536},    {"kmalloc-131072",     131072},
  {"kmalloc-262144",     262144},    {"kmalloc-524288",     524288},
  {"kmalloc-1048576",   1048576},    {"kmalloc-2097152",   2097152},
  {"kmalloc-4194304",   4194304},    {"kmalloc-8388608",   8388608},
  {"kmalloc-16777216", 16777216},    {"kmalloc-33554432", 33554432},
  {"kmalloc-67108864", 67108864}
};

struct kmem_cache* create_kmalloc_cache(const char *name,
    unsigned int size, slab_flags_t flags,
    unsigned int useroffset, unsigned int usersize)
{
  struct kmem_cache *s = kmem_cache_zalloc(kmem_cache, GFP_NOWAIT);

  create_boot_cache(s, name, size, flags, useroffset, usersize);
  list_add(&s->list, &slab_caches);
  memcg_link_cache(s);
  s->refcount = 1;
  return s;
}

/* Find the kmem_cache structure that serves a given size of
 * allocation */
struct kmem_cache *kmalloc_slab(size_t size, gfp_t flags)
{
  unsigned int index;

  if (size <= 192) {
    if (!size)
      return ZERO_SIZE_PTR;

    index = size_index[size_index_elem(size)];
  } else {
    if (unlikely(size > KMALLOC_MAX_CACHE_SIZE)) {
      WARN_ON(1);
      return NULL;
    }
    index = fls(size - 1);
  }

#ifdef CONFIG_ZONE_DMA
  if (unlikely((flags & GFP_DMA)))
    return kmalloc_dma_caches[index];

#endif
  return kmalloc_caches[index];
}

/* Conversion table for small slabs sizes / 8 to the index in the
 * kmalloc array. This is necessary for slabs < 192 since we have non power
 * of two cache sizes there. The size of larger slabs can be determined using
 * fls. */
u8 size_index[24] = {
  3,  /* 8 */
  4,  /* 16 */
  5,  /* 24 */
  5,  /* 32 */
  6,  /* 40 */
  6,  /* 48 */
  6,  /* 56 */
  6,  /* 64 */
  1,  /* 72 */
  1,  /* 80 */
  1,  /* 88 */
  1,  /* 96 */
  7,  /* 104 */
  7,  /* 112 */
  7,  /* 120 */
  7,  /* 128 */
  2,  /* 136 */
  2,  /* 144 */
  2,  /* 152 */
  2,  /* 160 */
  2,  /* 168 */
  2,  /* 176 */
  2,  /* 184 */
  2   /* 192 */
};

unsigned int size_index_elem(unsigned int bytes)
{
  return (bytes - 1) / 8;
}
```

# kmap_atomic
```C++
void *kmap_atomic(struct page *page)
{
  return kmap_atomic_prot(page, kmap_prot);
}

#define __fix_to_virt(x)  (FIXADDR_TOP - ((x) << PAGE_SHIFT))

void *kmap_atomic_prot(struct page *page, pgprot_t prot)
{
  // 64 bit machine doesn't have high memory
  if (!PageHighMem(page))
    return page_address(page);

  // 32 bit machine
  type = kmap_atomic_idx_push();
  idx = type + KM_TYPE_NR*smp_processor_id();
  vaddr = __fix_to_virt(FIX_KMAP_BEGIN + idx);
  set_pte(kmap_pte-idx, mk_pte(page, prot));

  return (void *)vaddr;
}
```

# page_address
```c++
/* get the mapped virtual address of a page */
void *page_address(const struct page *page)
{
  unsigned long flags;
  void *ret;
  struct page_address_slot *pas;

  if (!PageHighMem(page))
    return lowmem_page_address(page);

  pas = page_slot(page);
  ret = NULL;
  spin_lock_irqsave(&pas->lock, flags);
  if (!list_empty(&pas->lh)) {
    struct page_address_map *pam;

    list_for_each_entry(pam, &pas->lh, list) {
      if (pam->page == page) {
        ret = pam->virtual; // set_page_address()
        goto done;
      }
    }
  }
done:
  spin_unlock_irqrestore(&pas->lock, flags);
  return ret;
}

// page_address ->
static  void *lowmem_page_address(const struct page *page)
{
  return page_to_virt(page);
}

#define page_to_virt(x)  __va(PFN_PHYS(page_to_pfn(x)
```

# vmalloc
```C++
/* The kmalloc() function guarantees that the pages are
 * physically contiguous (and virtually contiguous).
 * The vmalloc() function ensures only that the pages are
 * contiguous within the virtual address space. */
void *vmalloc(unsigned long size)
{
  return __vmalloc_node_flags(
    size, NUMA_NO_NODE, GFP_KERNEL);
}

static void *__vmalloc_node(
  unsigned long size, unsigned long align,
  gfp_t gfp_mask, pgprot_t prot,
  int node, const void *caller)
{
  return __vmalloc_node_range(size, align, VMALLOC_START, VMALLOC_END,
        gfp_mask, prot, 0, node, caller);
}

void *__vmalloc_node_range(
  unsigned long size, unsigned long align,
  unsigned long start, unsigned long end, gfp_t gfp_mask,
  pgprot_t prot, unsigned long vm_flags, int node,
  const void *caller)
{
  struct vm_struct *area;
  void *addr;
  unsigned long real_size = size;

  size = PAGE_ALIGN(size);
  if (!size || (size >> PAGE_SHIFT) > totalram_pages)
    goto fail;

  area = __get_vm_area_node(size, align, VM_ALLOC | VM_UNINITIALIZED |
        vm_flags, start, end, node, gfp_mask, caller);
  if (!area)
    goto fail;

  addr = __vmalloc_area_node(area, gfp_mask, prot, node);
  if (!addr)
    return NULL;

  return addr;
}

static void *__vmalloc_area_node(
  struct vm_struct *area, gfp_t gfp_mask,
  pgprot_t prot, int node)
{
  struct page **pages;

  nr_pages = get_vm_area_size(area) >> PAGE_SHIFT;
  array_size = (nr_pages * sizeof(struct page *));

  /* Please note that the recursion is strictly bounded. */
  if (array_size > PAGE_SIZE) {
    pages = __vmalloc_node(array_size, 1, nested_gfp|highmem_mask,
        PAGE_KERNEL, node, area->caller);
  } else {
    pages = kmalloc_node(array_size, nested_gfp, node);
  }

  if (!pages) {
    remove_vm_area(area->addr);
    kfree(area);
    return NULL;
  }

  area->pages = pages;
  area->nr_pages = nr_pages;

  for (i = 0; i < area->nr_pages; i++) {
    struct page *page;

    if (node == NUMA_NO_NODE)
      page = alloc_page(alloc_mask|highmem_mask);
    else
      page = alloc_pages_node(node, alloc_mask|highmem_mask, 0);

    if (unlikely(!page)) {
      area->nr_pages = i;
      goto fail;
    }
    area->pages[i] = page;
    if (gfpflags_allow_blocking(gfp_mask|highmem_mask))
      cond_resched();
  }

  if (map_vm_area(area, prot, pages))
    goto fail;
  return area->addr;

fail:
  vfree(area->addr);
  return NULL;
}
```

# vmalloc_fault
```C++
static int vmalloc_fault(unsigned long address)
{
  unsigned long pgd_paddr;
  pmd_t *pmd_k;
  pte_t *pte_k;

  if (!(address >= VMALLOC_START && address < VMALLOC_END))
    return -1;

  pgd_paddr = read_cr3_pa();
  pmd_k = vmalloc_sync_one(__va(pgd_paddr), address);
  if (!pmd_k)
    return -1;

  pte_k = pte_offset_kernel(pmd_k, address);
  if (!pte_present(*pte_k))
    return -1;

  return 0
}
```