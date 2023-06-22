[:link: Linux - Memory Management Documentation](https://docs.kernel.org/mm/index.html)

# Table of Contents
* [_start:](#_start)
* [segment](#segment)
* [paging](#paging)
* [mm_core_init](#mm_core_init)
* [user virtual space](#user-virtual-space)
* [kernel virtual space](#kernel-virtual-space)
* [numa](#numa)
    * [node](#node)
    * [zone](#zone)
    * [page](#page)
* [buddy system](#buddy-system)
  * [alloc_pages](#alloc_pages)
  * [free_pages](#free_pages)
* [kmem_cache](#kmem_cache)
    * [kmem_cache_create](#kmem_cache_create)
    * [kmem_cache_alloc](#kmem_cache_alloc)
    * [kmem_cache_alloc_node](#kmem_cache_alloc_node)

* [slab](#slab)
    * [slab_alloc](#slab_alloc)
* [slub](#slub)
    * [slub_alloc](#slub_alloc)
* [kswapd](#kswapd)
* [brk](#brk)
* [mmap](#mmap)
* [page fault](#page-fault)
    * [asm_exc_page_fault](#asm_exc_page_fault)
    * [do_kern_addr_fault](#do_kern_addr_fault)
    * [do_user_addr_fault](#do_user_addr_fault)
        * [do_anonymous_page](#do_anonymous_page)
        * [do_fault](#do_fault)
        * [do_swap_page](#do_swap_page)
        * [hugetlb_fault](#hugetlb_fault)
* [pgd](#pgd)
* [kernel mapping](#kernel-mapping)
* [kmalloc](#kmalloc)
    * [kmalloc_caches](#kmalloc_caches)
* [kmap_atomic](#kmap_atomic)
* [page_address](#page_address)
* [vmalloc](#vmalloc)
* [vmalloc_fault](#vmalloc_fault)
* [page reclaim](#page_reclaim)

![](../Images/Kernel/kernel-structual.svg)


* Folio
    * [LWN - Folio](https://lwn.net/Kernel/Index/#Memory_management-Folios)
    * [YouTube - Large Pages in the Linux Kernel - 2020.12.03 Matthew Wilcox, Oracle](https://www.youtube.com/watch?v=hoSpvGxXgNg)
    * [YouTuBe - Folios - 2022.6.20 Matthew Wilcox](https://www.youtube.com/watch?v=URTuP6wXYPA)
    * [YouTube - Memory Folios - 2022.6.23 Matthew Wilcox, Oracle](https://www.youtube.com/watch?v=nknQML80w3E)
* [LWN - Compund Page](https://lwn.net/Kernel/Index/#Memory_management-Compound_pages)
* [wowo tech - memory management :cn:](http://www.wowotech.net/sort/memory_management)
* [LoyenWang :cn:]()
    * [ARM V8 MMU and mem mapping](https://www.cnblogs.com/LoyenWang/p/11406693.html)
    * [Physical memory init](https://www.cnblogs.com/LoyenWang/p/11440957.html)
    * [paging_init](https://www.cnblogs.com/LoyenWang/p/11483948.html)
    * [sparse memory model](https://www.cnblogs.com/LoyenWang/p/11523678.html)
    * [zone_sizes_init](https://www.cnblogs.com/LoyenWang/p/11568481.html)
    * [zoned page frame allocator](https://www.cnblogs.com/LoyenWang/p/11626237.html)
    * [buddy system](https://www.cnblogs.com/LoyenWang/p/11666939.html)
    * [watermark](https://www.cnblogs.com/LoyenWang/p/11708255.html)
    * [memory compaction](https://www.cnblogs.com/LoyenWang/p/11746357.html)
    * [memory reclaim](https://www.cnblogs.com/LoyenWang/p/11827153.html)
    * [slub allocator](https://www.cnblogs.com/LoyenWang/p/11922887.html)
    * [vmap vmalloc](https://www.cnblogs.com/LoyenWang/p/11965787.html)
    * [vma malloc mmap ](https://www.cnblogs.com/LoyenWang/p/12037658.html)
    * [page fault](https://www.cnblogs.com/LoyenWang/p/12116570.html)
    * [rmap](https://www.cnblogs.com/LoyenWang/p/12164683.html)
    * [cma](https://www.cnblogs.com/LoyenWang/p/12182594.html)

---

# _start:
```s
/* arch/arm64/kernel/head.S */
_start:
_primary_entry
    bl record_mmu_state

    /* Preserve the arguments passed by the bootloader in x0 .. x3 */
    bl preserve_boot_args

    bl create_idmap

    bl __cpu_setup

    b __primary_switch {
        adrp x1, reserved_pg_dir
        adrp x2, init_idmap_pg_dir
        bl __enable_mmu

        bl clear_page_tables
        bl create_kernel_mapping

        adrp x1, init_pg_dir
        load_ttbr1 x1, x1, x2 /* install x1 as a TTBR1 page table */

        bl __primary_switched {
            adr_l x4, init_task
            init_cpu_task x4, x5, x6

            adr_l x8, vectors /* load VBAR_EL1 with virtual */
            msr vbar_el1, x8 /* vector table address */

            /* Save the offset between
             * the kernel virtual and physical mappings*/
            ldr_l    x4, kimage_vaddr
            sub    x4, x4, x0
            str_l    x4, kimage_voffset, x5

            bl set_cpu_boot_mode_flag

            bl __pi_memset

            mov x0, x21    // pass FDT address in x0
            bl early_fdt_map
                /* populate pud, pmd, pte, p4d */
                early_fixmap_init()
                    __p4d_populate(p4dp, __pa_symbol(bm_pud))
                    __pud_populate(pudp, __pa_symbol(bm_pmd))
                    __pmd_populate(pmdp, __pa_symbol(bm_pte))

                /* map phys & virt */
                early_fdt_ptr = fixmap_remap_fdt(dt_phys, size)
                    create_mapping_noalloc()

            mov x0, x20 /* pass the full boot status */
            bl init_feature_override  /* Parse cpu feature overrides */

            bl start_kernel
        }
    }
```

```c
/* init/main.c */
start_kernel()
    setup_arch(&command_line) {
        setup_initial_init_mm()
        early_fixmap_init()
        early_ioremap_init();
        setup_machine_fdt(__fdt_pointer);
        arm64_memblock_init() {

        }
        paging_init() {
            /* map kernel: text, rodata init, data, bss*/
            map_kernel(pgdp);
            /* map all the memory banks */
            map_mem(pgdp);
            pgd_clear_fixmap();
            cpu_replace_ttbr1(lm_alias(swapper_pg_dir), init_idmap_pg_dir);
            init_mm.pgd = swapper_pg_dir;
            memblock_phys_free(__pa_symbol(init_pg_dir));
            memblock_allow_resize();
            create_idmap();
        }
        acpi_table_upgrade();
        bootmem_init() {
            dma_pernuma_cma_reserve();
            sparse_init();
            zone_sizes_init();
            dma_contiguous_reserve(arm64_dma_phys_limit);
            reserve_crashkernel();
            memblock_dump_all();
        }
    }

    setup_per_cpu_areas() {

    }

    mm_core_init() {
        build_all_zonelists(NULL) {
            __build_all_zonelists(NULL) {
                for_each_node(nid) {
                    pg_data_t *pgdat = NODE_DATA(nid);

                    build_zonelists(pgdat);
                }
            }
            for_each_possible_cpu(cpu)
                per_cpu_pages_init(&per_cpu(boot_pageset, cpu), &per_cpu(boot_zonestats, cpu));

            mminit_verify_zonelist();
            cpuset_init_current_mems_allowed();
        }
        page_alloc_init_cpuhp();
        page_ext_init_flatmem();
        mem_debugging_and_hardening_init();
        kfence_alloc_pool();
        report_meminit();
        kmsan_init_shadow();
        stack_depot_early_init();
        mem_init();
        mem_init_print_info();
        kmem_cache_init();

        page_ext_init_flatmem_late();
        kmemleak_init();
        ptlock_cache_init();
        pgtable_cache_init();
        debug_objects_mem_init();
        vmalloc_init();
        /* If no deferred init page_ext now, as vmap is fully initialized */
        if (!deferred_struct_pages)
            page_ext_init();
        /* Should be run before the first non-init thread is created */
        init_espfix_bsp();
        /* Should be run after espfix64 is set up. */
        pti_init();
        kmsan_init_runtime();
        mm_cache_init();
    }

    kmem_cache_init_late() {
        flushwq = alloc_workqueue("slub_flushwq", WQ_MEM_RECLAIM, 0);
    }

    setup_per_cpu_pageset() {

    }

    anon_vma_init() {

    }

    thread_stack_cache_init() {
        thread_stack_cache = kmem_cache_create_usercopy(
            "thread_stack",
            THREAD_SIZE, THREAD_SIZE, 0, 0,
            THREAD_SIZE, NUL
        );
    }

    proc_caches_init() {

    }

    vfs_caches_init() {

    }

    pagecache_init() {
        for (i = 0; i < PAGE_WAIT_TABLE_SIZE; i++)
            init_waitqueue_head(&folio_wait_table[i]);
        page_writeback_init();
    }
```

# setup_arch

```c
setup_arch(&command_line);
    setup_initial_init_mm(_stext, _etext, _edata, _end) {
        init_mm.start_code = (unsigned long)start_code;
        init_mm.end_code = (unsigned long)end_code;
        init_mm.end_data = (unsigned long)end_data;
        init_mm.brk = (unsigned long)brk;
    }

    early_fixmap_init() {
        unsigned long addr = FIXADDR_TOT_START;

        early_fixmap_init_pud(p4dp, addr, end);
            early_fixmap_init_pmd(pudp, addr, end);
                early_fixmap_init_pte(pmdp, addr);
                    if (pmd_none(pmd)) {
                        ptep = bm_pte[BM_PTE_TABLE_IDX(addr)];
                        __pmd_populate(pmdp, __pa_symbol(ptep), PMD_TYPE_TABLE);
                    }


    }

    early_ioremap_init();

    setup_machine_fdt(__fdt_pointer);
        --->
    arm64_memblock_init();
        --->
    paging_init();
        --->
    acpi_table_upgrade();

    bootmem_init();
        --->
```

## setup_machine_fdt
```c
setup_machine_fdt(__fdt_pointer)
    void *dt_virt = fixmap_remap_fdt(dt_phys, &size, PAGE_KERNEL) {
        const u64 dt_virt_base = __fix_to_virt(FIX_FDT);
        dt_phys_base = round_down(dt_phys, PAGE_SIZE);
        offset = dt_phys % PAGE_SIZE;
        dt_virt = (void *)dt_virt_base + offset;

        create_mapping_noalloc(dt_phys_base, dt_virt_base, PAGE_SIZE, prot);
            __create_pgd_mapping();

        *size = fdt_totalsize(dt_virt);
        if (*size > MAX_FDT_SIZE)
            return NULL;

        if (offset + *size > PAGE_SIZE) {
            create_mapping_noalloc(dt_phys_base, dt_virt_base,
                        offset + *size, prot);
        }

        return dt_virt;
    }
    if (dt_virt)
        memblock_reserve(dt_phys, size);

    early_init_dt_scan(dt_virt) {
        status = early_init_dt_verify(params);
        early_init_dt_scan_nodes() {

        }
    }

    const char * name = of_flat_dt_get_machine_name();

    /* Early fixups are done, map the FDT as read-only now */
    fixmap_remap_fdt(dt_phys, &size, PAGE_KERNEL_RO) {

    }

    name = of_flat_dt_get_machine_name();

```

## arm64_memblock_init

```c
arm64_memblock_init() {
    s64 linear_region_size = PAGE_END - _PAGE_OFFSET(vabits_actual);

    /* Remove memory above our supported physical address size */
    memblock_remove(1ULL << PHYS_MASK_SHIFT, ULLONG_MAX);

    /* Select a suitable value for the base of physical memory. */
    memstart_addr = round_down(memblock_start_of_DRAM(), ARM64_MEMSTART_ALIGN);

    /* Register the kernel text, kernel data, initrd, and initial
        * pagetables with memblock. */
    memblock_reserve(__pa_symbol(_stext), _end - _stext);

    /* Remove the memory that we will not be able to cover
     * with the linear mapping. */
    memblock_remove(max_t(u64, memstart_addr + linear_region_size, __pa_symbol(_end)), ULLONG_MAX);

    if (IS_ENABLED(CONFIG_BLK_DEV_INITRD) && phys_initrd_size) {
        /* the generic initrd code expects virtual addresses */
        initrd_start = __phys_to_virt(phys_initrd_start);
        initrd_end = initrd_start + phys_initrd_size;
    }

    early_init_fdt_scan_reserved_mem() {

    }

    high_memory = __va(memblock_end_of_DRAM() - 1) + 1;
}
```


## paging_init
```c
/* arch/arm64/mm/mmu.c */
paging_init()
    pgd_t *pgdp = pgd_set_fixmap(__pa_symbol(swapper_pg_dir));

    map_kernel(pgdp);
        map_kernel_segment(pgdp, _stext, _etext, text_prot);
            __create_pgd_mapping(pgdp, phys, virt, size)
                --->
            vm_area_add_early(vma) {
            vm->next = *p;
                *p = vm;
            }

        map_kernel_segment(pgdp, __start_rodata, __inittext_begin);
        map_kernel_segment(pgdp, __inittext_begin, __inittext_end);
        map_kernel_segment(pgdp, __initdata_begin, __initdata_end);
        map_kernel_segment(pgdp, _data, _end);

        fixmap_copy(pgdp);

    /* map all the memory banks */
    map_mem(pgdp);
        memblock_mark_nomap(kernel_start, kernel_end - kernel_start);
        for_each_mem_range(i, &start, &end) {
            if (start >= end)
                break;
            __map_memblock(pgdp, start, end, pgprot_tagged(PAGE_KERNEL), flags);
                __create_pgd_mapping(pgdp, phys, virt, size);
                    --->
        }

    pgd_clear_fixmap();

    cpu_replace_ttbr1(lm_alias(swapper_pg_dir), init_idmap_pg_dir);
    init_mm.pgd = swapper_pg_dir;

    /* free boot memory block */
    memblock_phys_free(init_pg_dir, size);
        memblock_remove_range(&memblock.reserved, base, size)

    memblock_allow_resize();

    create_idmap();
```

## bootmem_init
```c
bootmem_init() {
    min = PFN_UP(memblock_start_of_DRAM());
    max = PFN_DOWN(memblock_end_of_DRAM());

    early_memtest(min << PAGE_SHIFT, max << PAGE_SHIFT);

    max_pfn = max_low_pfn = max;
    min_low_pfn = min;

    arch_numa_init();

    sparse_init() {
        /* Mark all memblocks as present */
        memblocks_present() {
            for_each_mem_pfn_range()
                memory_present(nid, start, end) {
                    if (unlikely(!mem_section)) {
                        mem_section = memblock_alloc(size, align);
                    }

                    mminit_validate_memmodel_limits(&start, &end);

                    for (pfn = start; pfn < end; pfn += PAGES_PER_SECTION) {
                        unsigned long section = pfn_to_section_nr(pfn);
                        struct mem_section *ms;

                        sparse_index_init(section, nid) {
                            unsigned long root = SECTION_NR_TO_ROOT(section_nr);
                            struct mem_section *section;

                            if (mem_section[root])
                                return 0;

                            section = sparse_index_alloc(nid);
                            if (!section)
                                return -ENOMEM;

                            mem_section[root] = section;

                            return 0;
                        }
                        set_section_nid(section, nid);

                        ms = __nr_to_section(section);
                        if (!ms->section_mem_map) {
                            ms->section_mem_map = sparse_encode_early_nid(nid) |
                                            SECTION_IS_ONLINE;
                            __section_mark_present(ms, section);
                        }
                    }
                }
        }

        for_each_present_section_nr(pnum_begin + 1, pnum_end) {
            int nid = sparse_early_nid(__nr_to_section(pnum_end));

            if (nid == nid_begin) {
                map_count++;
                continue;
            }
            /* Init node with sections in range [pnum_begin, pnum_end) */
            sparse_init_nid(nid_begin, pnum_begin, pnum_end, map_count);
            nid_begin = nid;
            pnum_begin = pnum_end;
            map_count = 1;
        }

        /* cover the last node */
        sparse_init_nid(nid_begin, pnum_begin, pnum_end, map_count) {
            struct mem_section_usage *usage;
            unsigned long pnum;
            struct page *map;

            usage = sparse_early_usemaps_alloc_pgdat_section(NODE_DATA(nid), mem_section_usage_size() * map_count);

            sparse_buffer_init(map_count * section_map_size(), nid);

            for_each_present_section_nr(pnum_begin, pnum) {
                unsigned long pfn = section_nr_to_pfn(pnum);

                if (pnum >= pnum_end)
                    break;

                map = __populate_section_memmap(pfn, PAGES_PER_SECTION, nid, NULL, NULL) {
                    unsigned long start = (unsigned long) pfn_to_page(pfn);
                    unsigned long end = start + nr_pages * sizeof(struct page);

                    if (vmemmap_can_optimize(altmap, pgmap))
                        r = vmemmap_populate_compound_pages(pfn, start, end, nid, pgmap);
                    else {
                        r = vmemmap_populate(start, end, nid, altmap) {
                            if (!IS_ENABLED(CONFIG_ARM64_4K_PAGES)) {
                                return vmemmap_populate_basepages(start, end, node, altmap) {
                                    for (; addr < end; addr += PAGE_SIZE) {
                                        pte = vmemmap_populate_address(addr, node, altmap, reuse) {
                                            pgd = vmemmap_pgd_populate(addr, node) {
                                                pgd_t *pgd = pgd_offset_k(addr);
                                                if (pgd_none(*pgd)) {
                                                    void *p = vmemmap_alloc_block_zero(PAGE_SIZE, node) {
                                                        p = vmemmap_alloc_block(size, node) {
                                                            if (slab_is_available()) {
                                                                gfp_t gfp_mask = GFP_KERNEL|__GFP_RETRY_MAYFAIL|__GFP_NOWARN;

                                                                page = alloc_pages_node(node, gfp_mask, order) {
                                                                    __alloc_pages();
                                                                }
                                                                if (page)
                                                                    return page_address(page);

                                                                return NULL;
                                                            } else {
                                                                return __earlyonly_bootmem_alloc(node, size, size, __pa(MAX_DMA_ADDRESS)) {
                                                                    memblock_alloc_try_nid_raw()
                                                                        memblock_alloc_internal()
                                                                            --->
                                                                }
                                                            }
                                                        }
                                                        memset(p, 0, size);
                                                    }
                                                    if (!p)
                                                        return NULL;
                                                    pgd_populate(&init_mm, pgd, p);
                                                }
                                                return pgd;
                                            }

                                            p4d = vmemmap_p4d_populate(pgd, addr, node);

                                            pud = vmemmap_pud_populate(p4d, addr, node);

                                            pmd = vmemmap_pmd_populate(pud, addr, node);

                                            pte = vmemmap_pte_populate(pmd, addr, node, altmap, reuse);

                                            vmemmap_verify(pte, node, addr, addr + PAGE_SIZE);
                                        }
                                        if (!pte)
                                            return -ENOMEM;
                                    }
                                }
                            } else {
                                return vmemmap_populate_hugepages(start, end, node, altmap);
                            }
                        }
                    }
                }

                check_usemap_section_nr(nid, usage);
                sparse_init_one_section(__nr_to_section(pnum), pnum, map, usage, SECTION_IS_EARLY);
                usage = (void *) usage + mem_section_usage_size();
            }
            sparse_buffer_fini();
        }
    }

    zone_sizes_init() {
        /* Initialise all pg_data_t and zone data */
        free_area_init(max_zone_pfns) {

            find_zone_movable_pfns_for_nodes();

            for_each_mem_pfn_range(i, MAX_NUMNODES, &start_pfn, &end_pfn, &nid) {
                subsection_map_init(start_pfn, end_pfn - start_pfn);
            }

            setup_nr_node_ids();
            for_each_node(nid) {
                pg_data_t *pgdat;

                if (!node_online(nid)) {

                    pgdat = arch_alloc_nodedata(nid) {

                    }

                    arch_refresh_nodedata(nid, pgdat);
                    free_area_init_memoryless_node(nid);

                    continue;
                }

                pgdat = NODE_DATA(nid);
                free_area_init_node(nid) {
                    get_pfn_range_for_nid(nid, &start_pfn, &end_pfn);

                    pgdat->node_id = nid;
                    pgdat->node_start_pfn = start_pfn;
                    pgdat->per_cpu_nodestats = NULL;

                    calculate_node_totalpages(pgdat, start_pfn, end_pfn) {
                        spanned = zone_spanned_pages_in_node();
                        absent = zone_absent_pages_in_node()
                        if (size)
                            zone->zone_start_pfn = zone_start_pfn;
                        else
                            zone->zone_start_pfn = 0;
                        zone->spanned_pages = size;
                        zone->present_pages = real_size;

                    }

                    alloc_node_mem_map() { #ifdef CONFIG_FLATMEM
                        start = pgdat->node_start_pfn & ~(MAX_ORDER_NR_PAGES - 1);
                        offset = pgdat->node_start_pfn - start;

                        end = pgdat_end_pfn(pgdat);
                        end = ALIGN(end, MAX_ORDER_NR_PAGES);
                        size =  (end - start) * sizeof(struct page);
                        map = memmap_alloc(size);

                        pgdat->node_mem_map = map + offset;

                        if (pgdat == NODE_DATA(0)) {
                            mem_map = NODE_DATA(0)->node_mem_map;
                            if (page_to_pfn(mem_map) != pgdat->node_start_pfn)
                                mem_map -= offset;
                        }
                    }

                    /* Set up the zone data structures */
                    free_area_init_core(pgdat) {
                        /* calculate the freesize of pgdat */
                        zone_init_internals(zone, j, nid, freesize);
                        set_pageblock_order();
                        setup_usemap(zone);
                        init_currently_empty_zone();
                    }
                    lru_gen_init_pgdat(pgdat);
                }

                /* Any memory on that node */
                if (pgdat->node_present_pages)
                    node_set_state(nid, N_MEMORY);
                check_for_memory(pgdat, nid);
            }

            memmap_init() {
                for_each_mem_pfn_range(i, MAX_NUMNODES, &start_pfn, &end_pfn, &nid) {
                    struct pglist_data *node = NODE_DATA(nid);

                    for (j = 0; j < MAX_NR_ZONES; j++) {
                        struct zone *zone = node->node_zones + j;

                        /* return zone->present_pages */
                        if (!populated_zone(zone))
                            continue;

                        memmap_init_zone_range(zone, start_pfn, end_pfn, &hole_pfn) {
                            memmap_init_range() {
                                for (pfn = start_pfn; pfn < end_pfn; ) {
                                    if (context == MEMINIT_EARLY) {
                                        if (overlap_memmap_init(zone, &pfn))
                                            continue;
                                        if (defer_init(nid, pfn, zone_end_pfn)) {
                                            deferred_struct_pages = true;
                                            break;
                                        }
                                    }

                                    page = pfn_to_page(pfn); /* vmemmap + pfn */
                                    __init_single_page(page, pfn, zone, nid) {
                                        mm_zero_struct_page(page); /* memset(0) */
                                        set_page_links(page, zone, nid, pfn);
                                        init_page_count(page); /* 1 */
                                        page_mapcount_reset(page);
                                        page_cpupid_reset_last(page);
                                        page_kasan_tag_reset(page);

                                        INIT_LIST_HEAD(&page->lru);
                                    }
                                    if (context == MEMINIT_HOTPLUG)
                                        __SetPageReserved(page);

                                    if (pageblock_aligned(pfn)) {
                                        set_pageblock_migratetype(page, migratetype);
                                        cond_resched();
                                    }

                                    pfn++;
                                }
                            }
                        }
                        zone_id = j;
                    }
                }
            }
        }
    }

    dma_contiguous_reserve(arm64_dma_phys_limit);

    reserve_crashkernel();

    memblock_dump_all();
}
```

# mm_core_init
```c
void start_kernel(void) {
    mm_core_init() {
        build_all_zonelists(NULL);
        page_alloc_init_cpuhp();

        page_ext_init_flatmem();
        mem_debugging_and_hardening_init();
        kfence_alloc_pool();
        report_meminit();
        kmsan_init_shadow();
        stack_depot_early_init();

        mem_init() {
            /* release free pages to the buddy allocator */
            memblock_free_all() {
                /* The mem_map array can get very big.
                 * Free the unused area of the memory map. */
                free_unused_memmap() {
                    for_each_mem_pfn_range(i, MAX_NUMNODES, &start, &end, NULL) {
                        free_memmap(prev_end, start) {
                            /* Free boot memory block previously allocated by memblock_phys_alloc_xx() API.
                            * The freeing memory will not be released to the buddy allocator. */
                            memblock_phys_free(pg, pgend - pg);
                                memblock_remove_range(&memblock.reserved, base, size) {
                                    memblock_isolate_range()
                                    memblock_remove_region()
                                }
                        }
                    }
                }
                reset_all_zones_managed_pages();
                    atomic_long_set(&z->managed_pages, 0);

                pages = free_low_memory_core_early() {
                    memmap_init_reserved_pages() {
                        /* initialize struct pages for the reserved regions */
                        for_each_reserved_mem_range(i, &start, &end)
                            reserve_bootmem_region(start, end) {
                                for (; start_pfn < end_pfn; start_pfn++) {
                                    struct page *page = pfn_to_page(start_pfn);
                                    init_reserved_page(start_pfn) {

                                    }
                                    /* Avoid false-positive PageTail() */
                                    INIT_LIST_HEAD(&page->lru);

                                    __SetPageReserved(page);

                                }
                            }

                        /* and also treat struct pages for the NOMAP regions as PageReserved */
                        for_each_mem_region(region) {
                            if (memblock_is_nomap(region)) {
                                start = region->base;
                                end = start + region->size;
                                reserve_bootmem_region(start, end);
                            }
                        }
                    }

                    for_each_free_mem_range()
                        __free_pages()
                }
                totalram_pages_add(pages);
            }
        }

        mem_init_print_info();
        kmem_cache_init();

        page_ext_init_flatmem_late();
        kmemleak_init();
        ptlock_cache_init();
        pgtable_cache_init();
        debug_objects_mem_init();
        vmalloc_init() {

        }
        /* If no deferred init page_ext now, as vmap is fully initialized */
        if (!deferred_struct_pages)
            page_ext_init();
        /* Should be run before the first non-init thread is created */
        init_espfix_bsp();
        /* Should be run after espfix64 is set up. */
        pti_init();
        kmsan_init_runtime();
        mm_cache_init();
    }
}
```

# memblock

```c
memmap_alloc()
    if (exact_nid)
        ptr = memblock_alloc_exact_nid_raw();
    else
        ptr = memblock_alloc_try_nid_raw() {
            memblock_alloc_internal()
                alloc = memblock_alloc_range_nid() {
                    found = memblock_find_in_range_node(size, align, start, end, nid, flags);
                    if (found && !memblock_reserve(found, size))
                        goto done;
                }
                return phys_to_virt(alloc) {
                    /* PAGE_OFFSET - the virtual address of the start of the linear map
                     * at the start of the TTBR1 address space. */
                    #define __phys_to_virt(x) ((unsigned long)((x) - PHYS_OFFSET) | PAGE_OFFSET)
                    #define PHYS_OFFSET ({ VM_BUG_ON(memstart_addr & 1); memstart_addr; })
                    memstart_addr = round_down(memblock_start_of_DRAM(), ARM64_MEMSTART_ALIGN);
                    #define vmemmap ((struct page *)VMEMMAP_START - (memstart_addr >> PAGE_SHIFT))
                }
        }

    return ptr;

```

```c

```

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
  unsigned long vm_start; /* Our start address within vm_mm. */
  unsigned long vm_end; /* The first byte after our end address within vm_mm. */
  /* linked list of VM areas per task, sorted by address */
  struct vm_area_struct *vm_next, *vm_prev;
  struct rb_node vm_rb;

  struct mm_struct *vm_mm; /* The address space we belong to. */
  struct list_head anon_vma_chain; /* Serialized by mmap_sem &
            * page_table_lock */
  const struct vm_operations_struct *vm_ops;

  struct anon_vma *anon_vma; /* Serialized by page_table_lock */
  /* Function pointers to deal with this struct. */
  struct file * vm_file; /* File we map to (can be NULL). */
  void * vm_private_data; /* was vm_pte (shared mem) */
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
Zone | Memory Region
--- | ---
ZONE_DMA | First 16MiB of memory
ZONE_NORMAL | 16MiB - 896MiB
ZONE_HIGHMEM | 896 MiB - End

* [Chapter 2 Describing Physical Memory](https://www.kernel.org/doc/gorman/html/understand/understand005.html)

## zone
```C++
struct zone {
  struct pglist_data  *zone_pgdat;
  struct per_cpu_pageset *pageset; /* hot/cold page */

  unsigned long    zone_start_pfn;
  unsigned long    managed_pages; /* managed_pages = present_pages - reserved_pages */
  unsigned long    spanned_pages; /* spanned_pages = zone_end_pfn - zone_start_pfn */
  unsigned long    present_pages; /* present_pages = spanned_pages - absent_pages(pages in holes) */

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
    /* Atomic flags + zone number, some possibly updated asynchronously */
    unsigned long flags; /* include/linux/page-flags.h */

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

![](../Images/Kernel/mem-physic-numa-3.png)

# buddy system
```C++
struct free_area  free_area[MAX_ORDER];
#define MAX_ORDER 11
```
![](../Images/Kernel/mem-buddy-freepages.png)

## alloc_pages

* [Memory Folio](https://lwn.net/Kernel/Index/#Memory_management-Folios)

```c
alloc_pages(make, order)
    alloc_pages_node(numa_node_id(), mask, order)
        __alloc_pages(mask, order, nid, NULL)
            page = get_page_from_freelist() {
            retry:
                for_next_zone_zonelist_nodemask() {
                try_this_zone:
                    return page = rmqueue() {
                        if (pcp_allowed_order(order)) {
                            page = rmqueue_pcplist()
                            if (likely(page))
                                goto out;
                        }

                        page = rmqueue_buddy() {
                            do {
                                __rmqueue() {
                                    if (alloc_flags & ALLOC_CMA) {
                                        page = __rmqueue_cma_fallback(zone, order);
                                        if (page)
                                            return page;
                                    }

                                retry:
                                    page = __rmqueue_smallest(zone, order, migratetype) {
                                        for (; current_order < MAX_ORDER; ++current_order) {
                                            area = &(zone->free_area[current_order]);
                                            page = get_page_from_free_area(area, migratetype);
                                            if (!page)
                                                continue;
                                            del_page_from_free_list(page, zone, current_order) {
                                                list_del(&page->buddy_list) {
                                                    __list_del_entry(entry);
                                                    entry->next = LIST_POISON1;
                                                    entry->prev = LIST_POISON2;
                                                }
                                                __ClearPageBuddy(page);
                                                set_page_private(page, 0);
                                                zone->free_area[order].nr_free--;
                                            }
                                            expand(zone, page, order, current_order, migratetype) {
                                                unsigned long size = 1 << high;

                                                while (high > low) {
                                                    high--;
                                                    size >>= 1;

                                                    if (set_page_guard(zone, &page[size], high, migratetype))
                                                        continue;

                                                    add_to_free_list(&page[size], zone, high, migratetype) {
                                                        struct free_area *area = &zone->free_area[order];
                                                        list_add(&page->buddy_list, &area->free_list[migratetype]);
                                                        area->nr_free++;
                                                    }
                                                    set_buddy_order(&page[size], high);
                                                }
                                            }
                                            set_pcppage_migratetype(page, migratetype) {
                                                page->index = migratetype;
                                            }
                                            return page;
                                        }

                                        return NULL;
                                    }
                                    if (unlikely(!page)) {
                                        if (alloc_flags & ALLOC_CMA)
                                            page = __rmqueue_cma_fallback(zone, order)
                                                __rmqueue_smallest(zone, order, MIGRATE_CMA);
                                            __rmqueue_fallback = [](zone, order, migratetype, alloc_flags) {

                                            }
                                        if (!page && __rmqueue_fallback())
                                            goto retry;
                                    }
                                    return page;
                                }
                            } while (check_new_pages(page, order));
                        }
                    }
                }
            }

            if (likely(page))
                goto out;

            page = __alloc_pages_slowpath()
                wake_all_kswapds(order, gfp_mask, ac)
                page = get_page_from_freelist()
                if (page)
                    goto got_pg;
                __alloc_pages_direct_reclaim()
                __alloc_pages_direct_compact()
                __alloc_pages_cpuset_fallback()

                __alloc_pages_may_oom()
                    out_of_memory()
                        oom_kill_process()
                            do_send_sig_info(SIGKILL, SEND_SIG_PRIV)

              /* sched current process */
              cond_resched()
```

## free_pages

```c
free_pages(vaddr, order)
    __free_pages(virt_to_page((void *)addr), order)
        while (order-- > 0)
            free_the_page(page + (1 << order), order)
                __free_pages_ok(page, order, FPI_NONE)
                    __free_one_page(page, pfn, zone, order, migratetype, fpi_flags)
                        while (order < MAX_ORDER - 1) {
                            buddy = find_buddy_page_pfn(page, pfn)
                                if (!page_is_guard(buddy))
                                    del_page_from_free_list(buddy, zone, order)
                            if (!buddy)
                                goto done_merging;

                            combined_pfn = buddy_pfn & pfn;
                            page = page + (combined_pfn - pfn);
                            pfn = combined_pfn;
                            order++;
                        }

                    done_merging:
                        if (to_tail)
                            add_to_free_list_tail(page, zone, order, migratetype);
                        else
                            add_to_free_list(page, zone, order, migratetype);
```

# kmem_cache
![](../Images/Kernel/mem-kmem-cache-cpu-node.png)
![](../Images/Kernel/mem-kmem-cache.png)

```c
/* mm/slab_common.c */
enum slab_state slab_state;
LIST_HEAD(slab_caches);
DEFINE_MUTEX(slab_mutex);
struct kmem_cache *kmem_cache;

/* mm/slab.c */
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
      SLAB_HWCACHE_ALIGN, 0, 0
  );

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

```c
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
```c
void *kmem_cache_alloc(struct kmem_cache *cachep, gfp_t flags)
{
  void *ret = slab_alloc(cachep, flags, _RET_IP_);

  return ret;
}
```

## kmem_cache_alloc_node
```c
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
# slab
## slab_alloc

![](../Images/Kernel/mem-slab-struct.png)

![](../Images/Kernel/mem-kmem-cache-alloc.png)

![](../Images/Kernel/mem-mng.png)

* Reference:
  * [slaballocators.pdf](https://events.static.linuxfound.org/sites/events/files/slides/slaballocators.pdf)
  * [Slub allocator](https://www.cnblogs.com/LoyenWang/p/11922887.html)


```c
slab_alloc()
  slab_alloc_node()
    c = raw_cpu_ptr(s->cpu_slab);
    object = c->freelist;

    if (cpu_slab->freelist)   /* ALLOC_FASTPATH */
      next_object = get_freepointer_safe(s, object);
      this_cpu_cmpxchg_double(object, next_objext);
    else                      /* ALLOC_SLOWPATH */
      __slab_alloc()
        redo:
        /* 1. try kmem_cache_cpu freelist */

        new_slab:
        /* 2. try kmem_cache_cpu partial */
          if (slub_percpu_partial(c))
            goto redo

        /* 3. try kmem_cache_node */
          freelist = new_slab_objects()
            /* 3.1 try kmem_cache_node partial */
            get_partial()
              get_partial_node()
                list_for_each_entry_safe()
                  acquire_slab()
            /* 3.2 alloc_page */
            new_slab()
              allocate_slab()
                alloc_slab_page()
                  alloc_pages()
```

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
        next_object, next_tid(tid)))
    ) {
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
  /* 1. try kmem_cache_cpu freelist */
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
  /* 2. try kmem_cache_cpu partial */
  if (slub_percpu_partial(c)) { /* (c)->partial */
    page = c->page = slub_percpu_partial(c);
    slub_set_percpu_partial(c, page); /* slub_percpu_partial(c) = (p)->next; */
    stat(s, CPU_PARTIAL_ALLOC);
    goto redo;
  }

  /* 3. try kmem_cache_node */
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

  /* 3.1. try kmem_cache_node partial */
  freelist = get_partial(s, flags, node, c); /* -> get_partial_node() */
  if (freelist)
    return freelist;

  /* 3.2. alloc_page */
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

/* 3.2. new_slab_objects -> new_slab, no memory in kmem_cache_node */
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

# slub

![](../Images/Kernel/mem-slub-structure.png)

```c
kmem_cache_alloc(s, flags)
    __kmem_cache_alloc_lru(s, lru, flags)
        slab_alloc()
            slab_alloc_node()
                slab_pre_alloc_hook()
                kfence_alloc()
                __slab_alloc_node(s, gfpflags, node, addr, orig_size)
                    if (ifdef_CONFIG_SLUB_TINY) {
                        obj = get_partial(s, node) {
                            obj = get_partial_node(s, n) {
                                list_for_each_entry_safe(&n->partial) {
                                    obj = alloc_single_from_partial(s, n, slab, pc->orig_size);
                                        object = slab->freelist;
                                        slab->freelist = get_freepointer(s, object);
                                            /* `object + s->offset` stores next free obj addr */
                                            return freelist_dereference(s, object + s->offset);
                                        slab->inuse++;

                                        /* slab is runing out, move it from parital list to full list */
                                        if (slab->inuse == slab->objects) {
                                            remove_partial(n, slab);
                                                list_del(&slab->slab_list);
                                                n->nr_partial--;
                                            add_full(s, n, slab);
                                                list_add(&slab->slab_list, &n->full);
                                        }

                                    if (obj)
                                        return obj;
                                    /* Remove slab from the partial list, freeze it and
                                     * return the pointer to the freelist.*/
                                    t = acquire_slab(s, n)
                                        __cmpxchg_double_slab(s, slab,
                                            freelist, counters,
                                            new.freelist, new.counters,
                                            "acquire_slab"
                                        )
                                        remove_partial(n, slab)
                                            list_del(&slab->slab_list);
                                            n->nr_partial--;

                                    if (!t)
                                        break;

                                    if (!obj) {
                                        *pc->slab = slab;
                                        obj = t;
                                    } else {
                                        put_cpu_partial(s, slab, 0);
                                        partial_slabs++;
                                    }
                                }

                                return obj;
                            }

                            if (obj)
                                return obj;
                            obj = get_any_partial(s)
                                return NULL;
                        }

                        if (obj)
                            return obj;

                        slab = new_slab(s, gfpflags, node)  {
                            allocate_slab(s, f, n)
                                slab = alloc_slab_page(alloc_gfp, node, oo);
                                    alloc_pages()
                                slab->objects = oo_objects(oo);
                                slab->inuse = 0;
                                slab->frozen = 0;
                                account_slab(slab, oo_order(oo), s, flags);
                                slab->slab_cache = s;
                                kasan_poison_slab(slab);
                                start = slab_address(slab);
                                /* Shuffle the single linked freelist based on a random pre-computed sequence */
                                shuffle = shuffle_freelist(s, slab);
                                if (!shuffle) {
                                    start = fixup_red_left(s, start);
                                    start = setup_object(s, start);
                                    slab->freelist = start;
                                    for (idx = 0, p = start; idx < slab->objects - 1; idx++) {
                                        next = p + s->size;
                                        next = setup_object(s, next);
                                        set_freepointer(s, p, next);
                                        p = next;
                                    }
                                    set_freepointer(s, p, NULL);
                                }

                                return slab;
                        }

                        alloc_single_from_new_slab(s, slab, orig_size)
                            object = slab->freelist;
                            slab->freelist = get_freepointer(s, object);
                            slab->inuse = 1;

                            if (slab->inuse == slab->objects)
                                add_full(s, n, slab);
                            else
                                add_partial(n, slab, DEACTIVATE_TO_HEAD);

                            inc_slabs_node(s, nid, slab->objects);

                    } else { /* ifndef_CONFIG_SLUB_TINY */
                        redo:
                            /* 1. get free obj from cpu cache */
                            object = c->freelist;
                            slab = c->slab;

                            if (unlikely(!object || !slab || !node_match(slab, node))) {
                                object = __slab_alloc(s, gfpflags, node, addr, c, orig_size);
                                    ___slab_alloc(s, gfpflags, node, addr, c, orig_size) {
                                        reread_slab:
                                            slab = READ_ONCE(c->slab);
                                            if (!slab) {
                                                goto new_slab;
                                            }

                                        redo:
                                            freelist = c->freelist;
                                            if (freelist)
                                                goto load_freelist;

                                            freelist = get_freelist(s, slab);
                                            if (!freelist) {
                                                c->slab = NULL;
                                                c->tid = next_tid(c->tid);
                                                local_unlock_irqrestore(&s->cpu_slab->lock, flags);
                                                stat(s, DEACTIVATE_BYPASS);
                                                goto new_slab;
                                            }

                                        load_freelist:
                                            c->freelist = get_freepointer(s, freelist);
                                            c->tid = next_tid(c->tid);
                                            return freelist;

                                        deactivate_slab:
                                            freelist = c->freelist;
                                            c->slab = NULL;
                                            c->freelist = NULL;
                                            c->tid = next_tid(c->tid);
                                            deactivate_slab(s, slab, freelist);

                                        new_slab:
                                            if (slub_percpu_partial(c)) {
                                                if (unlikely(c->slab)) {
                                                    goto reread_slab;
                                                }
                                                if (unlikely(!slub_percpu_partial(c))) {
                                                    /* we were preempted and partial list got empty */
                                                    goto new_objects;
                                                }

                                                slab = c->slab = slub_percpu_partial(c);
                                                slub_set_percpu_partial(c, slab);
                                                stat(s, CPU_PARTIAL_ALLOC);
                                                goto redo;
                                            }

                                        new_objects:
                                            /* get partial from node */
                                            freelist = get_partial(s, node, &pc);
                                            if (freelist)
                                                goto check_new_slab;

                                            slub_put_cpu_ptr(s->cpu_slab);
                                            /* node has no partial, alloc_page */
                                            slab = new_slab(s, gfpflags, node);
                                                alloc_pages()
                                            c = slub_get_cpu_ptr(s->cpu_slab);

                                            if (unlikely(!slab)) {
                                                slab_out_of_memory(s, gfpflags, node);
                                                return NULL;
                                            }

                                            freelist = slab->freelist;
                                            slab->freelist = NULL;
                                            slab->inuse = slab->objects;
                                            slab->frozen = 1;

                                        check_new_slab:

                                        retry_load_slab:
                                            if (unlikely(c->slab)) {
                                                void *flush_freelist = c->freelist;
                                                struct slab *flush_slab = c->slab;

                                                c->slab = NULL;
                                                c->freelist = NULL;
                                                c->tid = next_tid(c->tid);

                                                deactivate_slab(s, flush_slab, flush_freelist);

                                                stat(s, CPUSLAB_FLUSH);

                                                goto retry_load_slab;
                                            }
                                            c->slab = slab;

                                            goto load_freelist;
                                    }

                            } else {
                                void *next_object = get_freepointer_safe(s, object);

                                if (unlikely(!this_cpu_cmpxchg_double(
                                    s->cpu_slab->freelist, s->cpu_slab->tid,
                                    object, tid,
                                    next_object, next_tid(tid))))
                                {
                                    note_cmpxchg_failure("slab_alloc", s, tid);
                                    goto redo;
                                }
                            }
                    }

                maybe_wipe_obj_freeptr()
                slab_want_init_on_alloc()
                slab_post_alloc_hook()

```

```c
/* Reuses the bits in struct page */
struct slab {
  unsigned long __page_flags;

#if defined(CONFIG_SLAB)

#elif defined(CONFIG_SLUB)

  struct kmem_cache *slab_cache;
  union {
    struct {
      union {
        struct list_head slab_list;
#ifdef CONFIG_SLUB_CPU_PARTIAL
        struct {
          struct slab *next;
          int slabs;  /* Nr of slabs left */
        };
#endif
      };

      /* Double-word boundary */
      void *freelist; /* first free object, only used if CONFIG_SLUB_TINY */
      union {
        unsigned long counters;
        struct {
          unsigned inuse:16;
          unsigned objects:15;
          unsigned frozen:1;
        };
      };
    };

    struct rcu_head rcu_head;
  };

  unsigned int __unused;

#endif

  atomic_t __page_refcount;
};
```

```c
struct kmem_cache {
#ifndef CONFIG_SLUB_TINY
  struct kmem_cache_cpu __percpu *cpu_slab;
#endif

  /* Used for retrieving partial slabs, etc. */
  slab_flags_t            flags;
  unsigned long           min_partial;
  unsigned int            size;  /* The size of an object including metadata */
  unsigned int            object_size;/* The size of an object without metadata */
  struct reciprocal_value reciprocal_size;
  unsigned int            offset;  /* Free pointer offset */

#ifdef CONFIG_SLUB_CPU_PARTIAL
  /* Number of per cpu partial objects to keep around */
  unsigned int cpu_partial;
  /* Number of per cpu partial slabs to keep around */
  unsigned int cpu_partial_slabs;
#endif

  struct kmem_cache_order_objects oo;
  /* Allocation and freeing of slabs */
  struct kmem_cache_order_objects min;

  gfp_t             allocflags;  /* gfp flags to use on each alloc */
  int               refcount; /* Refcount for slab cache destroy */
  void (*ctor)(void *);
  unsigned int      inuse;    /* Offset to metadata */
  unsigned int      align;    /* Alignment */
  unsigned int      red_left_pad;  /* Left redzone padding size */
  const char        *name;  /* Name (only for display!) */
  struct list_head  list;  /* List of slab caches */

  struct kmem_cache_node *node[MAX_NUMNODES];
};

struct kmem_cache_node {
#ifdef CONFIG_SLAB

#endif

#ifdef CONFIG_SLUB
  spinlock_t        list_lock;
  unsigned long     nr_partial;
  struct list_head  partial;

#ifdef CONFIG_SLUB_DEBUG
  atomic_long_t     nr_slabs;
  atomic_long_t     total_objects;
  struct list_head full;
#endif /* CONFIG_SLUB_DEBUG */

#endif /* CONFIG_SLUB */
};

struct kmem_cache_cpu {
  void          **freelist;  /* Pointer to next available object */
  unsigned long tid;  /* Globally unique transaction id */
  struct slab   *slab;  /* The slab from which we are allocating */
  struct slab   *partial;  /* Partially allocated frozen slabs */
  local_lock_t  lock;  /* Protects the fields above */
};
```

# kswapd
```C++
//1. active page out when alloc
get_page_from_freelist();
    node_reclaim();
        __node_reclaim();
            shrink_node();

/* 2. positive page out by kswapd */
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
/* balance_pgdat->kswapd_shrink_node->shrink_node */

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
SYSCALL_DEFINE1(brk)
    if (brk <= mm->brk) {
        do_munmap()
    }

    do_brk(oldbrk, len)



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

# create_pgd_mapping

```c
__create_pgd_mapping(pgdir, phys, virt, size, prot, pgd_pgtable_alloc, flags) {
    do {
        next = pgd_addr_end(addr, end);
        alloc_init_pud(pgdp, addr, next, phys, prot, pgtable_alloc, flags) {
            if (pgd_none(pgd)) {
                pud_phys = pgtable_alloc(PUD_SHIFT);
                __p4d_populate(p4dp, pud_phys, p4dval);
                p4d = READ_ONCE(*p4dp);
            }
            BUG_ON(p4d_bad(p4d));

            pudp = pud_set_fixmap_offset(p4dp, addr);
            do {
                alloc_init_cont_pmd(pudp, addr, next, phys, prot, pgtable_alloc, flags) {
                    if (pud_none(pud)) {
                        pmd_phys = pgtable_alloc(PMD_SHIFT);
                        __pud_populate(pudp, pmd_phys, pudval);
                        pud = READ_ONCE(*pudp);
                    }
                    BUG_ON(pud_bad(pud));

                    pmdp = pmd_set_fixmap_offset(pudp, addr);
                    do {
                        next = pmd_cont_addr_end(addr, end);
                        init_pmd(pudp, addr, next, phys, __prot, pgtable_alloc, flags) {
                            do {
                                next = pmd_addr_end(addr, end);
                                alloc_init_cont_pte() {
                                    if (pmd_none(pmd)) {
                                        pte_phys = pgtable_alloc(PAGE_SHIFT);
                                        __pmd_populate(pmdp, pte_phys, pmdval);
                                        pmd = READ_ONCE(*pmdp);
                                    }
                                    BUG_ON(pmd_bad(pmd));

                                    do {
                                        next = pte_cont_addr_end(addr, end);
                                        init_pte(pmdp, addr, next, phys, __prot) {
                                            ptep = pte_set_fixmap_offset(pmdp, addr);
                                            do {
                                                set_pte(ptep, pfn_pte(__phys_to_pfn(phys), prot)) {
                                                    WRITE_ONCE(*ptep, pte);
                                                    if (pte_valid_not_user(pte)) {
                                                        dsb(ishst);
                                                        isb();
                                                    }
                                                }
                                                phys += PAGE_SIZE;
                                            } while (ptep++, addr += PAGE_SIZE, addr != end);
                                            pte_clear_fixmap();
                                        }
                                        phys += next - addr;
                                    } while (addr = next, addr != end);
                                }
                                phys += next - addr;
                            } while (pmdp++, addr = next, addr != end);
                            phys += next - addr;
                        }
                        phys += next - addr;
                    } while (addr = next, addr != end);
                    pmd_clear_fixmap();
                }
                phys += next - addr;
            } while (pudp++, addr = next, addr != end);
            pud_clear_fixmap();
        }
        phys += next - addr;

        phys += next - addr;
    } while (pgdp++, addr = next, addr != end);
```

# remove_pgd_mapping

```c
__remove_pgd_mapping()
    /* free phys mem which virt addr is [start, end] */
    unmap_hotplug_range(asid, pgdir, start, end, 0, tlb) {
        do {
            if (pgd_none(pgd))
                continue;
        /* 1. pgd */
            unmap_hotplug_p4d_range(asid, pgdp, addr, next, free_mapped, tlb) {
                do {
                    if (p4d_none(p4d))
                        continue;
        /* 2. pud */
                    unmap_hotplug_pud_range(asid, p4dp, addr, next, free_mapped, tlb) {
                        do {
                            if (pud_none(pud))
                                continue;
                            if (pud_sect(pud)) {
                                pud_clear(pudp);
                                // tlb_batach_tlb_gather(tlb, addr | ARM64_TLB_FLUSH_PUD);

                                flush_tlb_kernel_range(addr, addr + PAGE_SIZE);
                                    --->
                                if (free_mapped) {
                                    free_hotplug_page_range(pud_page(pud), PUD_SIZE, tlb)
                                        free_pages()
                                }
                                continue;
                            }
        /* 2. pmd */
                            unmap_hotplug_pmd_range(asid, pudp, addr, next, free_mapped, tlb) {
                                do {
                                    if (pmd_none(pmd))
                                        continue;

                                    flush_tlb_kernel_range(addr, addr + PAGE_SIZE);
                                        --->
                                    if (pmd_sect(pmd)) {
                                        pmd_clear(pmdp);
                                        if (free_mapped)
                                            free_hotplug_page_range(pmd_page(pmd), PMD_SIZE, tlb)
                                               free_pages()
                                        continue;
                                    }
        /* 3. pte */
                                    unmap_hotplug_pte_range(asid, pmdp, addr, next, free_mapped, tlb);
                                        do {
                                            if (pte_none(pte))
                                                continue;
                                            pte_clear(NULL, addr, ptep);

                                            flush_tlb_kernel_range(addr, addr + PAGE_SIZE);
                                                --->
                                            if (free_mapped) {
                                                free_hotplug_page_range(pte_page(pte), PAGE_SIZE, tlb)
                                                   free_pages()
                                            }
                                        } while (addr += PAGE_SIZE, addr < end);
                                    }
                                } while (addr = next, addr < end);
                            }
                        } while (addr = next, addr < end);
                    }
                } while (addr = next, addr < end);
            }
        } while (addr = next, addr < end)

    /* free phsy mem of pgtable which is used to map virt addr [start, end] */
    free_empty_tables()
        do {
            pgd = READ_ONCE(*pgdp);
            if (pgd_none(pgd))
                continue;
            free_empty_p4d_table(asid, pgdp, addr, next, floor, ceiling, tlb) {
                do {
                    if (p4d_none(p4d))
                        continue;
                    free_empty_pud_table(asid, p4dp, addr, next, floor, ceiling, tlb) {
                        do {
                            pud = READ_ONCE(*pudp);
                            free_empty_pmd_table(asid, pudp, addr, next, floor, ceiling, tlb) {
                                do {
                                    pmd = READ_ONCE(*pmdp);
                                    if (pmd_none(pmd))
                                        continue;
                                    free_empty_pte_table(asid, pmdp, addr, next, floor, ceiling, tlb) {
                                        do {
                                            WARN_ON(!pte_none(pte));
                                        } while ();

                                        pmd_clear(pmdp);
                                        __flush_tlb_kernel_pgtable(start);
                                        free_hotplug_pgtable_page(virt_to_page(ptep), tlb);
                                            free_pages()
                                    }
                                } while (addr = next, addr < end);

                                pud_clear(pudp);
                                __flush_tlb_kernel_pgtable(start);
                                free_hotplug_pgtable_page(virt_to_page(pmdp), tlb);
                                    free_pages()
                            }
                        } while (addr = next, addr < end);

                        p4d_clear(p4dp);
                        __flush_tlb_kernel_pgtable(start);
                        free_hotplug_pgtable_page(virt_to_page(pudp), tlb);
                            free_pages()
                    }
                } while (addr = next, addr < end);
            }
        } while (addr = next, addr < end);
```

# mmap

```C++
mmap() {
    sys_mmap_pgoff() {
        vm_mmap_pgoff() {
            do_mmap_pgoff() {
                do_mmap()

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

                map_region() {
                    if (may_expand_vm()) {
                        return -ENOMEM;
                    }

                    /* Unmap any existing mapping in the area */
                    do_vmi_munmap(&vmi, mm);

                    vma_merge();

                    struct vm_area_struct *vma = kmem_cache_zalloc();

                    if (file) {
                        vma->vm_file = get_file(file);
                        /* 2.1. link the file to vma */
                        rc = call_mtmap(file, vma);
                            file->f_op->mmap(file, vma);
                            ext4_file_mmap();
                                vma->vm_ops = &ext4_file_vm_ops;

                        if (rc) {
                            unmap_region()
                                --->
                        }
                    } else if (vm_flags & VM_SHARED) {
                        shmem_zero_setup(vma);
                    } else {
                        vma_set_anonymous(vma); /* vma->vm_ops = NULL; */
                    }
                }

                /* 2.2. link the vma to the file */
                vma_link(mm, vma, prev, rb_link, rb_parent);
                    vma_interval_tree_insert(vma, &mapping->i_mmap);

            }
        }
    }
}


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

/* page cache in memory */
struct address_space {
  struct inode          *host;
  struct xarray         i_pages; /* cached physical pages */
  struct rw_semaphore   invalidate_lock;
  gfp_t                 gfp_mask;
  atomic_t              i_mmap_writable; /* Number of VM_SHARED mappings. */
  struct rb_root_cached i_mmap; /* Tree of private and shared mappings. vm_area_struct */
  struct rw_semaphore   i_mmap_rwsem;
  unsigned long         nrpages;
  pgoff_t               writeback_index; /* Writeback starts here */
  const struct address_space_operations *a_ops;
  unsigned long         flags;
  errseq_t              wb_err;
  spinlock_t            private_lock;
  struct list_head      private_list;
  void*                 private_data;
};
```

# page fault

![](../Images/Kernel/mem-page-fault.png)


```c
/* arm64
 * arch/arm64/mm/fault.c */
static const struct fault_info fault_info[] = {
    do_translation_fault, SIGSEGV, SEGV_MAPERR, "level 0 translation"
};

el1h_64_sync_handler() {
    switch (esr) {
        el1_abort(regs, esr) {
            do_mem_abort()
                do_translation_fault()
                    do_page_fault()
                        __do_page_fault()
                            fault = handle_mm_fault()
                                --->
                            if (fault & VM_FAULT_OOM) {
                                pagefault_out_of_memory() {

                                }
                                return 0;
                            }

                            if (fault & VM_FAULT_SIGBUS) {
                                arm64_force_sig_fault(SIGBUS, BUS_ADRERR, far, inf->name);
                            } else if (fault & (VM_FAULT_HWPOISON_LARGE | VM_FAULT_HWPOISON)) {
                                unsigned int lsb;

                                lsb = PAGE_SHIFT;
                                if (fault & VM_FAULT_HWPOISON_LARGE)
                                    lsb = hstate_index_to_shift(VM_FAULT_GET_HINDEX(fault));

                                arm64_force_sig_mceerr(BUS_MCEERR_AR, far, lsb, inf->name);
                            } else {
                                arm64_force_sig_fault(SIGSEGV,
                                    fault == VM_FAULT_BADACCESS ? SEGV_ACCERR : SEGV_MAPERR,
                                    far, inf->name
                                );
                            }
        }
    }
}

el0t_64_sync_handler() {
    switch (ESR_ELx_EC(esr)) {
    case ESR_ELx_EC_SVC64:
        el0_svc(regs) {

        }
    }
}


/* mm/memory.c */
handle_mm_fault(vma, address, flags, regs);

    hugetlb_fault();

    __handle_mm_fault()

        pgd = pgd_offset(mm, address)
        p4d = p4d_alloc(pgd)
        if (!p4d)
            return VM_FAULT_OOM;
        pud = pud_alloc(p4d)
        if (!pud)
            return VM_FAULT_OOM;
        pmd = pmd_alloc(pud)
        if (!pmd)
            return VM_FAULT_OOM;

        handle_pte_fault() {

            /* 1. anonymous fault */
            do_anonymous_page();
                pte = pte_alloc(pmd)
                if (pte)
                    return VM_FAULT_OOM;
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
                /* 2.1 read fault */
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

                /* 2.2 cow fault */
                do_cow_fault()

                /* 2.3 shared fault */
                do_shared_fault()

            /* 3. swap fault */
            do_swap_page();

            update_mmu_tlb()
        }
```

```c
/* x86 */
exc_page_fault();
    handle_page_fault();
        do_kern_addr_fault();

        do_user_addr_fault();
            vma = find_vma(mm, address);

```

```C++
struct file {
  struct file_operations* f_op;
  struct address_space*   f_mapping;
};

/* page cache in memory */
struct address_space {
  struct inode          *host;
  struct xarray         i_pages; /* cached physical pages */
  struct rw_semaphore   invalidate_lock;
  gfp_t                 gfp_mask;
  atomic_t              i_mmap_writable; /* Number of VM_SHARED mappings. */
  struct rb_root_cached i_mmap; /* Tree of private and shared mappings. vm_area_struct */
  struct rw_semaphore   i_mmap_rwsem;
  unsigned long         nrpages;
  pgoff_t               writeback_index; /* Writeback starts here */
  const struct address_space_operations *a_ops;
  unsigned long         flags;
  errseq_t              wb_err;
  spinlock_t            private_lock;
  struct list_head      private_list;
  void*                 private_data;
};

```

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
/* mm_init-> */
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

# munmap
```c
munmap()
SYSCALL_DEFINE2(munmap) {
    __vm_munmap(addr, len, true) {
        do_vmi_munmap() {
            do_vmi_align_munmap() {
                unmap_region() {
                    lru_add_drain();
                    tlb_gather_mmu(&tlb, mm);
                    update_hiwater_rss(mm);

                    /* 1. just unmap the phys which is mmaped with vma, vma is free by vm_area_free */
                    unmap_vmas(&tlb, mt, vma, start, end, mm_wr_locked) {
                        do {
                            unmap_single_vma();
                                unmap_page_range() {
                                    tlb_start_vma(tlb, vma);

                                    pgd = pgd_offset(vma->vm_mm, addr);
                                    do {
                                        next = pgd_addr_end(addr, end);
                                        if (pgd_none_or_clear_bad(pgd))
                                            continue;
                                        next = zap_p4d_range(tlb, vma, pgd, addr, next, details); {
                                            p4d = p4d_offset(pgd, addr);
                                            do {
                                                next = p4d_addr_end(addr, end);
                                                if (p4d_none_or_clear_bad(p4d))
                                                    continue;
                                                next = zap_pud_range(tlb, vma, p4d, addr, next, details) {
                                                    do {
                                                        if (pud_none_or_clear_bad(pud))
                                                            continue;
                                                        next = zap_pmd_range(tlb, vma, pud, addr, next, details); {
                                                            zap_pte_range() {
                                                                do {
                                                                    pte_t ptent = *pte;
                                                                    struct page *page;

                                                                    if (pte_present(ptent)) {
                                                                        unsigned int delay_rmap;

                                                                        page = vm_normal_page(vma, addr, ptent);

                                                                        ptent = ptep_get_and_clear_full(mm, addr, pte, tlb->fullmm);
                                                                        tlb_remove_tlb_entry(tlb, pte, addr) {
                                                                            tlb_flush_pte_range()
                                                                                __tlb_adjust_range(tlb, address, size);
                                                                                tlb->cleared_ptes = 1;
                                                                            __tlb_remove_tlb_entry()
                                                                        }
                                                                        zap_install_uffd_wp_if_needed(vma, addr, pte, details, ptent);
                                                                        if (unlikely(!page))
                                                                            continue;

                                                                        delay_rmap = 0;
                                                                        if (!PageAnon(page)) {
                                                                            if (pte_dirty(ptent)) {
                                                                                set_page_dirty(page);
                                                                                if (tlb_delay_rmap(tlb)) {
                                                                                    delay_rmap = 1;
                                                                                    force_flush = 1;
                                                                                }
                                                                            }
                                                                        }

                                                                        __tlb_remove_page(tlb, page, delay_rmap) {
                                                                            batch = tlb->active;
                                                                            batch->encoded_pages[batch->nr++] = page;
                                                                            if (batch->nr == batch->max) {
                                                                                if (!tlb_next_batch(tlb))
                                                                                    return true;
                                                                                batch = tlb->active;
                                                                            }
                                                                        }
                                                                    }


                                                                    pte_clear_not_present_full(mm, addr, pte, tlb->fullmm);
                                                                    zap_install_uffd_wp_if_needed(vma, addr, pte, details, ptent);
                                                                } while (pte++, addr += PAGE_SIZE, addr != end);
                                                            }
                                                        }
                                                    next:
                                                        cond_resched();
                                                    } while (pud++, addr = next, addr != end);
                                                }
                                            } while (p4d++, addr = next, addr != end);
                                        }
                                    } while (pgd++, addr = next, addr != end);

                                    tlb_end_vma(tlb, vma) {
                                        tlb_flush_mmu_tlbonly(tlb);
                                    }
                                }
                        } while ((vma = mas_find(&mas, end_addr - 1)) != NULL);
                    }

                    /* 2 free pg table */
                    free_pgtables(&tlb) {
                        do {
                            unlink_anon_vmas(vma);
                            unlink_file_vma(vma);

                            /* frees user-level page tables */
                            free_pgd_range() {
                                do {
                                    next = pgd_addr_end(addr, end);
                                    if (pgd_none_or_clear_bad(pgd))
                                        continue;
                                    free_p4d_range(tlb, pgd, addr, next, floor, ceiling); {
                                        do {
                                            next = p4d_addr_end(addr, end);
                                            if (p4d_none_or_clear_bad(p4d))
                                                continue;
                                            free_pud_range(tlb, p4d, addr, next, floor, ceiling) {
                                                do {
                                                    next = pud_addr_end(addr, end);
                                                    if (pud_none_or_clear_bad(pud))
                                                        continue;
                                                    free_pmd_range(tlb, pud, addr, next, floor, ceiling); {
                                                        do {
                                                            next = pmd_addr_end(addr, end);
                                                            if (pmd_none_or_clear_bad(pmd))
                                                                continue;
                                                            free_pte_range(tlb, pmd, addr); {
                                                                pgtable_t token = pmd_pgtable(*pmd);
                                                                pmd_clear(pmd);
                                                                pte_free_tlb(tlb, token, addr) {
                                                                    tlb_flush_pmd_range(tlb, address, PAGE_SIZE)
                                                                    tlb->freed_tables = 1;
                                                                    __pte_free_tlb(tlb, ptep, address) {
                                                                        pgtable_pte_page_dtor(pte);
                                                                        tlb_remove_table(tlb, pte) {
                                                                            /* add to tlb mmu_gather */
                                                                            (*batch)->tables[(*batch)->nr++] = table;
                                                                            if ((*batch)->nr == MAX_TABLE_BATCH)
                                                                                tlb_table_flush(tlb)
                                                                                    --->
                                                                        }
                                                                    }
                                                                }
                                                                mm_dec_nr_ptes(tlb->mm);
                                                            }
                                                        } while (pmd++, addr = next, addr != end);

                                                        pud_clear(pud);
                                                        pmd_free_tlb(tlb, pmd, start) {
                                                            tlb_flush_pud_range(tlb, address, PAGE_SIZE);
                                                            tlb->freed_tables = 1;
                                                            __pmd_free_tlb(tlb, pmdp, address) {
                                                                pgtable_pmd_page_dtor(page);
                                                                tlb_remove_table(tlb, page);
                                                                    --->
                                                            }
                                                        }
                                                        mm_dec_nr_pmds(tlb->mm);
                                                    }
                                                } while (pud++, addr = next, addr != end);

                                                pud = pud_offset(p4d, start);
                                                p4d_clear(p4d);
                                                pud_free_tlb(tlb, pud, start) {
                                                    tlb_remove_table(tlb, pud);
                                                        --->
                                                }
                                                mm_dec_nr_puds(tlb->mm);
                                            }

                                            p4d = p4d_offset(pgd, start);
                                            pgd_clear(pgd);
                                            p4d_free_tlb(tlb, p4d, start) {

                                            }
                                        } while (p4d++, addr = next, addr != end);
                                    }
                                } while (pgd++, addr = next, addr != end)
                            }
                        } while (vma);
                    }

                    /* 3. finish gather */
                    tlb_finish_mmu(&tlb) {
                        tlb_flush_mmu(tlb) {
                            tlb_flush_mmu_tlbonly(tlb)  {
                                if (!(tlb->freed_tables
                                    || tlb->cleared_ptes
                                    || tlb->cleared_pmds
                                    || tlb->cleared_puds
                                    || tlb->cleared_p4ds)
                                ) {
                                    return;
                                }
                                tlb_flush(tlb);
                                    __flush_tlb_range()
                                __tlb_reset_range(tlb);
                            }

                            tlb_flush_mmu_free(tlb) {
                                tlb_table_flush(tlb)  {
                                    if (*batch) {
                                        tlb_table_invalidate(tlb) {
                                            if (tlb_needs_table_invalidate()) {
                                                tlb_flush_mmu_tlbonly(tlb)
                                                    --->
                                            }
                                        }
                                        tlb_remove_table_free(*batch) {
                                            for (i = 0; i < batch->nr; i++)
                                                __tlb_remove_table(batch->tables[i]);
                                                    free_page_and_swap_cache();
                                                        free_page();
                                            free_page((unsigned long)bat ch);
                                        }
                                        *batch = NULL;
                                    }
                                }

                                tlb_batch_pages_flush(tlb) {
                                    free_pages_and_swap_cache(pages, nr);
                                }
                            }
                        }

                        tlb_batch_list_free() {
                            for (batch = tlb->local.next; batch; batch = next) {
                                next = batch->next;
                                free_pages((unsigned long)batch, 0);
                            }
                        }
                    }
                }

                remove_mt(mm, &mas_detach) {
                    mas_for_each(mas, vma, ULONG_MAX) {
                        remove_vma(vma) {
                            vm_area_free(vma) {
                                free_anon_vma_name(vma);
                                kmem_cache_free(vm_area_cachep, vma);
                            }
                        }
                    }
                }
            }
        }
    }
}
```

# rmap

* [五花肉 - linux内核反向映射(RMAP)技术分析 - 知乎](https://zhuanlan.zhihu.com/p/564867734)
* [linux 匿名页反向映射 - 知乎](https://zhuanlan.zhihu.com/p/361173109)

```c
struct vm_area_struct {
    struct list_head    anon_vma_chain;
    struct anon_vma*    anon_vma;
}

struct anon_vma {
    struct anon_vma *root;
    struct rw_semaphore rwsem;

    unsigned degree;
    atomic_t refcount;
    struct anon_vma *parent;
    struct rb_root_cached rb_root;
};

struct page {
    /* If low bit clear, points to inode address_space, or NULL.
     * If page mapped as anonymous memory, low bit is set,
     * and it points to anon_vma object */
    struct address_space *mapping;
​
    union {
        /* page offset in user virtual address space for anon mapping
         * page offset in file for file mapping */
        pgoff_t index;
        union {
            atomic_t _mapcount;
        }
    }
};

wp_page_copy()
do_anonymous_page()
do_cow_fault()

migrate_vma_insert_page()

expand_upwards()
expand_downwards()

    anon_vma_prepare()
        avc = anon_vma_chain_alloc(GFP_KERNEL);
        anon_vma = anon_vma_alloc()
        vma->anon_vma = anon_vma;
        anon_vma_chain_link(vma, avc, anon_vma);
            avc->vma = vma;
            avc->anon_vma = anon_vma;
            list_add(&avc->same_vma, &vma->anon_vma_chain);
            anon_vma_interval_tree_insert(avc, &anon_vma->rb_root);

page_add_anon_rmap()
    __page_set_anon_rmap()
        struct anon_vma *anon_vma = vma->anon_vma;
        anon_vma = (void *) anon_vma + PAGE_MAPPING_ANON;
        WRITE_ONCE(folio->mapping, (struct address_space *) anon_vma);
        folio->index = linear_page_index(vma, address);

page_add_new_anon_rmap()
    folio_add_new_anon_rmap()
        __page_set_anon_rmap(folio, &folio->page, vma, address, 1);
            --->

try_to_unmap()
    struct rmap_walk_control rwc = {
        .rmap_one = try_to_unmap_one,
        .arg = (void *)flags,
        .done = folio_not_mapped,
        .anon_lock = folio_lock_anon_vma_read,
    };

    rmap_walk_locked(folio, &rwc);
        if (folio_test_anon(folio))
            rmap_walk_anon(folio, rwc, true) {
                pgoff_start = folio_pgoff(folio);
                pgoff_end = pgoff_start + folio_nr_pages(folio) - 1;
                anon_vma_interval_tree_foreach(avc, &anon_vma->rb_root,
                        pgoff_start, pgoff_end) {
                    struct vm_area_struct *vma = avc->vma;
                    unsigned long address = vma_address(&folio->page, vma);

                    VM_BUG_ON_VMA(address == -EFAULT, vma);
                    cond_resched();

                    if (rwc->invalid_vma && rwc->invalid_vma(vma, rwc->arg))
                        continue;

                    ret = rwc->rmap_one(folio, vma, address, rwc->arg) {
                        try_to_unmap_one()


                    }
                    if (!ret) {
                        break;
                    }
                    if (rwc->done && rwc->done(folio))
                        break;
                }
            }
        else
            rmap_walk_file(folio, rwc, true);
```

```c
anon_vma_fork()
    anon_vma_clone(vma, pvma) {
        list_for_each_entry_reverse(pavc, &src->anon_vma_chain, same_vma) {
            struct anon_vma *anon_vma;

            avc = anon_vma_chain_alloc(GFP_NOWAIT | __GFP_NOWARN);

            anon_vma = pavc->anon_vma;
            root = lock_anon_vma_root(root, anon_vma);
            anon_vma_chain_link(dst, avc, anon_vma);

            if (!dst->anon_vma && src->anon_vma
                && anon_vma->num_children < 2
                && anon_vma->num_active_vmas == 0) {

                dst->anon_vma = anon_vma;
            }
        }
    }

    anon_vma = anon_vma_alloc()
    avc = anon_vma_chain_alloc(GFP_KERNEL)
    anon_vma->root = pvma->anon_vma->root;
    anon_vma->parent = pvma->anon_vma;
    vma->anon_vma = anon_vma;
    anon_vma_chain_link(vma, avc, anon_vma);
    anon_vma->parent->num_children++;
```

# remove_memory
```c
/* mm/memory_hotplug.c
 * Remove memory if every memory block is offline */
remove_memory(start, size)
    try_remove_memory()
        arch_remove_memory()

/* arch/arm64/mm/mmu.c */
arch_remove_memory(start, size, altmap)
    __remove_pages(start_pfn, nr_pages, altmap);
        for () {
            __remove_section(pfn, cur_nr_pages, map_offset, altmap)
                sparse_remove_section(ms, pfn, nr_pages, map_offset, altmap)
                    section_deactivate()
                    ....

        }

    __remove_pgd_mapping(swapper_pg_dir, __phys_to_virt(start), size)
        /* 1. free phys mem which virt addr is [start, end] */
        unmap_hotplug_range(start, end, false, NULL) {
            do {
            /* 1. pgd */
                unmap_hotplug_p4d_range(pgdp, addr, next, free_mapped, altmap) {
                    do {
            /* 2. pud */
                        unmap_hotplug_pud_range(p4dp, addr, next, free_mapped, altmap) {
                            do {
                                if (pud_sect(pud)) {
                                    pud_clear(pudp);
                                    /* arch/arm64/include/asm/tlbflush.h */
                                    flush_tlb_kernel_range(addr, addr + PAGE_SIZE) {
                                        if ((end - start) > (MAX_TLBI_OPS * PAGE_SIZE)) {
                                            flush_tlb_all();
                                            return;
                                        }

                                        start = __TLBI_VADDR(start, 0);
                                        end = __TLBI_VADDR(end, 0);

                                        dsb(ishst);
                                        for (addr = start; addr < end; addr += 1 << (PAGE_SHIFT - 12))
                                            __tlbi(vaale1is, addr);
                                        dsb(ish);
                                        isb();
                                    }

                                    if (free_mapped) {
                                        free_hotplug_page_range(pud_page(pud), PUD_SIZE, altmap)
                                            free_pages()
                                    }
                                    continue;
                                }
            /* 3. pmd */
                                unmap_hotplug_pmd_range(pudp, addr, next, free_mapped, altmap) {
                                    do {
                                        if (pmd_sect(pmd)) {
                                            pmd_clear(pmdp);
                                            flush_tlb_kernel_range(addr, addr + PAGE_SIZE);
                                                --->
                                            if (free_mapped)
                                                free_hotplug_page_range() {
                                                    free_pages()
                                                }
                                            continue;
                                        }
            /* 4. pte */
                                        unmap_hotplug_pte_range(pmdp, addr, next, free_mapped, altmap) {
                                            do {
                                                pte_clear(&init_mm, addr, ptep);
                                                flush_tlb_kernel_range(addr, addr + PAGE_SIZE);
                                                    --->
                                                if (free_mapped)
                                                    free_hotplug_page_range() {
                                                        free_pages()
                                                    }
                                            } while (addr += PAGE_SIZE, addr < end);
                                        }
                                    } while (addr = next, addr < end);
                                }
                            } while (addr = next, addr < end);
                        }
                    } while (addr = next, addr < end);
                }
            } while (addr = next, addr < end)
        }

        /* 2. free phsy mem of pgtable which is used to map virt addr [start, end] */
        free_empty_tables(start, end, PAGE_OFFSET, PAGE_END) {
            do {
                next = pgd_addr_end(addr, end);
                pgdp = pgd_offset_k(addr);
                pgd = READ_ONCE(*pgdp);
                if (pgd_none(pgd))
                    continue;

                free_empty_p4d_table(pgdp, addr, next, floor, ceiling) {
                    do {
                        next = p4d_addr_end(addr, end);
                        p4dp = p4d_offset(pgdp, addr);
                        p4d = READ_ONCE(*p4dp);
                        if (p4d_none(p4d))
                            continue;

                        free_empty_pud_table(p4dp, addr, next, floor, ceiling) {
                            do {
                                next = pud_addr_end(addr, end);
                                pudp = pud_offset(p4dp, addr);
                                pud = READ_ONCE(*pudp);
                                if (pud_none(pud))
                                    continue;

                                free_empty_pmd_table(pudp, addr, next, floor, ceiling) {
                                    do {
                                        next = pmd_addr_end(addr, end);
                                        pmdp = pmd_offset(pudp, addr);
                                        pmd = READ_ONCE(*pmdp);
                                        if (pmd_none(pmd))
                                            continue;

                                        free_empty_pte_table(pmdp, addr, next, floor, ceiling) {
                                            do {
                                                ptep = pte_offset_kernel(pmdp, addr);
                                                pte = READ_ONCE(*ptep);
                                            } while (addr += PAGE_SIZE, addr < end);

                                            pmd_clear(pmdp);
                                            __flush_tlb_kernel_pgtable(start);
                                                --->
                                            free_hotplug_pgtable_page(virt_to_page(ptep)) {
                                                free_pages()
                                            }
                                        }
                                    } while (addr = next, addr < end);

                                    pud_clear(pudp);
                                    __flush_tlb_kernel_pgtable(start);
                                        --->
                                    free_hotplug_pgtable_page(virt_to_page(pmdp)) {
                                        free_pages()
                                    }
                                }
                            } while (addr = next, addr < end);

                            p4d_clear(p4dp);
                            __flush_tlb_kernel_pgtable(start) {
                                dsb(ishst);
                                __tlbi(vaae1is, addr); /* invalidate the TLB */
                                dsb(ish);
                                isb();
                            }

                            free_hotplug_pgtable_page(virt_to_page(pudp)) {
                                free_pages()
                            }
                        }
                    } while (addr = next, addr < end);
                }
            } while (addr = next, addr < end);
        }
```

# kernel mapping
```C++
/* arch/x86/include/asm/pgtable_64.h */
extern pud_t level3_kernel_pgt[512];
extern pud_t level3_ident_pgt[512];

extern pmd_t level2_kernel_pgt[512];
extern pmd_t level2_fixmap_pgt[512];
extern pmd_t level2_ident_pgt[512];

extern pte_t level1_fixmap_pgt[512];
extern pgd_t init_top_pgt[];

#define swapper_pg_dir init_top_pgt

/* arch\x86\kernel\head_64.S */
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
   * Don't set NX because code runs from these pages. */
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
/* kernel mm_struct */
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

/* init kernel mm_struct */
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

/* init_mem_mapping -> */
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
```c
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

```c
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
```c
/* mm/slab_common.c */
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
  /* 64 bit machine doesn't have high memory */
  if (!PageHighMem(page))
    return page_address(page);

  /* 32 bit machine */
  type = kmap_atomic_idx_push();
  idx = type + KM_TYPE_NR*smp_processor_id();
  vaddr = __fix_to_virt(FIX_KMAP_BEGIN + idx);
  set_pte(kmap_pte-idx, mk_pte(page, prot));

  return (void *)vaddr;
}
```

# page_address
```c
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
        ret = pam->virtual; /* set_page_address() */
        goto done;
      }
    }
  }
done:
  spin_unlock_irqrestore(&pas->lock, flags);
  return ret;
}

/* page_address -> */
static  void *lowmem_page_address(const struct page *page)
{
  return page_to_virt(page);
}

#define page_to_virt(x)  __va(PFN_PHYS(page_to_pfn(x)
```

# vmalloc

```c
vmalloc(size);
    __vmalloc_node_range(size, align, VMALLOC_START, VMALLOC_END, gfp_mask, PAGE_KERNEL, 0, node, caller)
    struct vm_struct *area = __get_vm_area_node();
        area = kzalloc_node(sizeof(*area));
        /* Allocate a region of KVA */
        va = alloc_vmap_area(size);
            va = kmem_cache_alloc_node(vmap_area_cachep);
            addr = __alloc_vmap_area(&free_vmap_area_root, &free_vmap_area_list, size, align, vstart, vend);
                va = find_vmap_lowest_match();
                adjust_va_to_fit_type();
                    kmem_cache_alloc(vmap_area_cachep, GFP_NOWAIT);
            va->va_start = addr;
            va->va_end = addr + size;
            va->vm = NULL;
            insert_vmap_area(va, &vmap_area_root, &vmap_area_list);
        setup_vmalloc_vm(area, va, flags, caller);

    /* Allocate physical pages and map them into vmalloc space. */
    __vmalloc_area_node(area, gfp_mask, prot, shift, node);
        vm_area_alloc_pages();
            alloc_pages();

        vmap_pages_range(addr, addr + size, prot, area->pages, page_shift);
            vmap_range_noflush();
                pgd = pgd_offset_k(addr);
                vmap_p4d_range();
                    p4d = p4d_alloc_track(&init_mm, pgd, addr, mask);
                    vmap_pud_range();
                        pud = pud_alloc_track(&init_mm, p4d, addr, mask);
                        vmap_pmd_range();
                            pmd = pmd_alloc_track(&init_mm, pud, addr, mask);
                            vmap_pte_range();
                                pte = pte_alloc_kernel_track(pmd, addr, mask);
                                set_pte_at(&init_mm, addr, pte, pfn_pte(pfn, prot));
```

```C++
struct vm_struct {
  struct vm_struct  *next;
  void              *addr;
  unsigned long     size;
  unsigned long     flags;
  struct page       **pages;
  unsigned int      nr_pages;
  phys_addr_t       phys_addr;
  const void        *caller;
};

struct vmap_area {
  unsigned long va_start;
  unsigned long va_end;

  struct rb_node rb_node; /* address sorted rbtree */
  struct list_head list; /* address sorted list */

  union {
    unsigned long subtree_max_size; /* in "free" tree free_vmap_area_root */
    struct vm_struct *vm;           /* in "busy" tree vmap_area_root */
  };
};

/* The kmalloc() function guarantees that the pages are
 * physically contiguous (and virtually contiguous).
 *
 * The vmalloc() function ensures only that the pages are
 * contiguous within the virtual address space. */
void *vmalloc(unsigned long size)
{
  return __vmalloc_node(size, 1, GFP_KERNEL, NUMA_NO_NODE, __builtin_return_address(0));
}

void *__vmalloc_node(unsigned long size, unsigned long align,
          gfp_t gfp_mask, int node, const void *caller)
{
  return __vmalloc_node_range(size, align, VMALLOC_START, VMALLOC_END,
        gfp_mask, PAGE_KERNEL, 0, node, caller);
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

  area = __get_vm_area_node(size, align, VM_ALLOC | VM_UNINITIALIZED |
        vm_flags, start, end, node, gfp_mask, caller);

  /* Allocate physical pages and map them into vmalloc space. */
  ret = __vmalloc_area_node(area, gfp_mask, prot, shift, node);

  return area->addr;
}

struct vm_struct *__get_vm_area_node(unsigned long size,
    unsigned long align, unsigned long shift, unsigned long flags,
    unsigned long start, unsigned long end, int node,
    gfp_t gfp_mask, const void *caller)
{
  struct vmap_area *va;
  struct vm_struct *area;
  unsigned long requested_size = size;

  size = ALIGN(size, 1ul << shift);
  if (unlikely(!size))
    return NULL;

  if (flags & VM_IOREMAP)
    align = 1ul << clamp_t(int, get_count_order_long(size), PAGE_SHIFT, IOREMAP_MAX_ORDER);

  area = kzalloc_node(sizeof(*area), gfp_mask & GFP_RECLAIM_MASK, node);
  if (unlikely(!area))
    return NULL;

  if (!(flags & VM_NO_GUARD))
    size += PAGE_SIZE;

  va = alloc_vmap_area(size, align, start, end, node, gfp_mask);
  if (IS_ERR(va)) {
    kfree(area);
    return NULL;
  }

  setup_vmalloc_vm(area, va, flags, caller);

  if (!(flags & VM_ALLOC))
    area->addr = kasan_unpoison_vmalloc(area->addr, requested_size,
                KASAN_VMALLOC_PROT_NORMAL);

  return area;
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
    area->pages = __vmalloc_node(array_size, 1, nested_gfp|highmem_mask,
        PAGE_KERNEL, node, area->caller);
  } else {
    area->pages = kmalloc_node(array_size, nested_gfp, node);
  }

  if (!pages) {
    remove_vm_area(area->addr);
    kfree(area);
    return NULL;
  }

  area->nr_pages = vm_area_alloc_pages(gfp_mask | __GFP_NOWARN,
    node, page_order, nr_small_pages, area->pages);

  do {
    ret = vmap_pages_range(addr, addr + size, prot, area->pages, page_shift);
    if (nofail && (ret < 0))
      schedule_timeout_uninterruptible(1);
  } while (nofail && (ret < 0));

  return area->addr;

fail:
  vfree(area->addr);
  return NULL;
}

unsigned int
vm_area_alloc_pages(gfp_t gfp, int nid,
    unsigned int order, unsigned int nr_pages, struct page **pages)
{
  unsigned int nr_allocated = 0;
  struct page *page;
  int i;

  if (!order) {
    gfp_t bulk_gfp = gfp & ~__GFP_NOFAIL;

    while (nr_allocated < nr_pages) {
      unsigned int nr, nr_pages_request;

      nr_pages_request = min(100U, nr_pages - nr_allocated);

      if (IS_ENABLED(CONFIG_NUMA) && nid == NUMA_NO_NODE)
        nr = alloc_pages_bulk_array_mempolicy(bulk_gfp,
              nr_pages_request,
              pages + nr_allocated);

      else
        nr = alloc_pages_bulk_array_node(bulk_gfp, nid,
              nr_pages_request,
              pages + nr_allocated);

      nr_allocated += nr;
      cond_resched();

      if (nr != nr_pages_request)
        break;
    }
  }

  /* High-order pages or fallback path if "bulk" fails. */
  while (nr_allocated < nr_pages) {
    if (fatal_signal_pending(current))
      break;

    if (nid == NUMA_NO_NODE)
      page = alloc_pages(gfp, order);
    else
      page = alloc_pages_node(nid, gfp, order);
    if (unlikely(!page))
      break;
    /*
     * Higher order allocations must be able to be treated as
     * indepdenent small pages by callers (as they can with
     * small-page vmallocs). Some drivers do their own refcounting
     * on vmalloc_to_page() pages, some use page->mapping,
     * page->lru, etc.
     */
    if (order)
      split_page(page, order);

    for (i = 0; i < (1U << order); i++)
      pages[nr_allocated + i] = page + i;

    cond_resched();
    nr_allocated += 1U << order;
  }

  return nr_allocated;
}

/* map pages to a kernel virtual address */
int vmap_pages_range(unsigned long addr, unsigned long end,
    pgprot_t prot, struct page **pages, unsigned int page_shift)
{
  int err;

  err = vmap_pages_range_noflush(addr, end, prot, pages, page_shift);
  flush_cache_vmap(addr, end);
  return err;
}

int vmap_pages_range_noflush(unsigned long addr, unsigned long end,
    pgprot_t prot, struct page **pages, unsigned int page_shift)
{
  unsigned int i, nr = (end - addr) >> PAGE_SHIFT;

  WARN_ON(page_shift < PAGE_SHIFT);

  if (!IS_ENABLED(CONFIG_HAVE_ARCH_HUGE_VMALLOC) || page_shift == PAGE_SHIFT)
    return vmap_small_pages_range_noflush(addr, end, prot, pages);

  for (i = 0; i < nr; i += 1U << (page_shift - PAGE_SHIFT)) {
    int err;

    err = vmap_range_noflush(
      addr,
      addr + (1UL << page_shift),
      __pa(page_address(pages[i])), prot,
      page_shift
    );
    if (err)
      return err;

    addr += 1UL << page_shift;
  }

  return 0;
}

int vmap_range_noflush(unsigned long addr, unsigned long end,
      phys_addr_t phys_addr, pgprot_t prot,
      unsigned int max_page_shift)
{
  pgd_t *pgd;
  unsigned long start;
  unsigned long next;
  int err;
  pgtbl_mod_mask mask = 0;

  might_sleep();
  start = addr;
  pgd = pgd_offset_k(addr);
  do {
    next = pgd_addr_end(addr, end);
    err = vmap_p4d_range(pgd, addr, next, phys_addr, prot,
          max_page_shift, &mask);
    if (err)
      break;
  } while (pgd++, phys_addr += (next - addr), addr = next, addr != end);

  if (mask & ARCH_PAGE_TABLE_SYNC_MASK)
    arch_sync_kernel_mappings(start, end);

  return err;
}

int vmap_p4d_range(pgd_t *pgd, unsigned long addr, unsigned long end,
      phys_addr_t phys_addr, pgprot_t prot,
      unsigned int max_page_shift, pgtbl_mod_mask *mask)
{
  p4d_t *p4d;
  unsigned long next;

  p4d = p4d_alloc_track(&init_mm, pgd, addr, mask);
  if (!p4d)
    return -ENOMEM;
  do {
    next = p4d_addr_end(addr, end);

    if (vmap_try_huge_p4d(p4d, addr, next, phys_addr, prot, max_page_shift)) {
      *mask |= PGTBL_P4D_MODIFIED;
      continue;
    }

    if (vmap_pud_range(p4d, addr, next, phys_addr, prot, max_page_shift, mask))
      return -ENOMEM;
  } while (p4d++, phys_addr += (next - addr), addr = next, addr != end);
  return 0;
}

int vmap_pud_range(p4d_t *p4d, unsigned long addr, unsigned long end,
      phys_addr_t phys_addr, pgprot_t prot,
      unsigned int max_page_shift, pgtbl_mod_mask *mask)
{
  pud_t *pud;
  unsigned long next;

  pud = pud_alloc_track(&init_mm, p4d, addr, mask);
  if (!pud)
    return -ENOMEM;
  do {
    next = pud_addr_end(addr, end);

    if (vmap_try_huge_pud(pud, addr, next, phys_addr, prot, max_page_shift)) {
      *mask |= PGTBL_PUD_MODIFIED;
      continue;
    }

    if (vmap_pmd_range(pud, addr, next, phys_addr, prot,
          max_page_shift, mask))
      return -ENOMEM;
  } while (pud++, phys_addr += (next - addr), addr = next, addr != end);
  return 0;
}

int vmap_pmd_range(pud_t *pud, unsigned long addr, unsigned long end,
      phys_addr_t phys_addr, pgprot_t prot,
      unsigned int max_page_shift, pgtbl_mod_mask *mask)
{
  pmd_t *pmd;
  unsigned long next;

  pmd = pmd_alloc_track(&init_mm, pud, addr, mask);
  if (!pmd)
    return -ENOMEM;
  do {
    next = pmd_addr_end(addr, end);

    if (vmap_try_huge_pmd(pmd, addr, next, phys_addr, prot, max_page_shift)) {
      *mask |= PGTBL_PMD_MODIFIED;
      continue;
    }

    if (vmap_pte_range(pmd, addr, next, phys_addr, prot, max_page_shift, mask))
      return -ENOMEM;
  } while (pmd++, phys_addr += (next - addr), addr = next, addr != end);
  return 0;
}

int vmap_pte_range(pmd_t *pmd, unsigned long addr, unsigned long end,
      phys_addr_t phys_addr, pgprot_t prot,
      unsigned int max_page_shift, pgtbl_mod_mask *mask)
{
  pte_t *pte;
  u64 pfn;
  unsigned long size = PAGE_SIZE;

  pfn = phys_addr >> PAGE_SHIFT;
  pte = pte_alloc_kernel_track(pmd, addr, mask);
  if (!pte)
    return -ENOMEM;

  do {
#ifdef CONFIG_HUGETLB_PAGE
    size = arch_vmap_pte_range_map_size(addr, end, pfn, max_page_shift);
    if (size != PAGE_SIZE) {
      pte_t entry = pfn_pte(pfn, prot);

      entry = arch_make_huge_pte(entry, ilog2(size), 0);
      set_huge_pte_at(&init_mm, addr, pte, entry);
      pfn += PFN_DOWN(size);
      continue;
    }
#endif

    set_pte_at(&init_mm, addr, pte, pfn_pte(pfn, prot));
    pfn++;
  } while (pte += PFN_DOWN(size), addr += size, addr != end);

  *mask |= PGTBL_PTE_MODIFIED;
  return 0;
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

# page_reclaim
```c
typedef struct pglist_data {
    struct lruvec        lruvec;
    struct task_struct *kswapd;
};

struct lruvec {
    struct list_head        lists[NR_LRU_LISTS];
    spinlock_t                lru_lock;

    unsigned long            anon_cost;
    unsigned long            file_cost;

    atomic_long_t            nonresident_age;

    unsigned long            refaults[ANON_AND_FILE];

    unsigned long            flags;

    struct pglist_data *pgdat;
};

enum lru_list {
    LRU_INACTIVE_ANON = LRU_BASE,
    LRU_ACTIVE_ANON = LRU_BASE + LRU_ACTIVE,
    LRU_INACTIVE_FILE = LRU_BASE + LRU_FILE,
    LRU_ACTIVE_FILE = LRU_BASE + LRU_FILE + LRU_ACTIVE,
    LRU_UNEVICTABLE,
    NR_LRU_LISTS
};

struct pagevec {
    unsigned char nr;
    bool percpu_pvec_drained;
    struct page *pages[PAGEVEC_SIZE];
};

struct cpu_fbatches {
    local_lock_t lock;
    struct folio_batch lru_add;
    struct folio_batch lru_deactivate_file;
    struct folio_batch lru_deactivate;
    struct folio_batch lru_lazyfree;
    struct folio_batch activate;
};

static DEFINE_PER_CPU(struct cpu_fbatches, cpu_fbatches) = {
    .lock = INIT_LOCAL_LOCK(lock),
};

struct folio_batch {
    unsigned char nr;
    bool percpu_pvec_drained;
    struct folio *folios[PAGEVEC_SIZE];
};

struct lru_rotate {
    local_lock_t lock;
    struct folio_batch fbatch;
};
static DEFINE_PER_CPU(struct lru_rotate, lru_rotate) = {
    .lock = INIT_LOCAL_LOCK(lock),
};
```

# fork
```c
kernel_clone()
    copy_mm() {
        if (clone_flags & CLONE_VM) {
            mmget(oldmm);
            mm = oldmm;
        } else {
            mm = dup_mm(tsk, current->mm) {
                mm = allocate_mm();
                memcpy(mm, oldmm, sizeof(*mm));

                mm_init(mm, tsk, mm->user_ns)
                dup_mmap(mm, oldmm) {
                    for_each_vma(old_vmi, mpnt) {
                        struct file *file;

                        if (mpnt->vm_flags & VM_DONTCOPY) {
                            vm_stat_account(mm, mpnt->vm_flags, -vma_pages(mpnt));
                            continue;
                        }
                        charge = 0;
                        /* Don't duplicate many vmas if we've been oom-killed (for example) */
                        if (fatal_signal_pending(current)) {
                            retval = -EINTR;
                            goto loop_out;
                        }
                        if (mpnt->vm_flags & VM_ACCOUNT) {
                            unsigned long len = vma_pages(mpnt);
                            charge = len;
                        }

                        tmp = vm_area_dup(mpnt);

                        retval = vma_dup_policy(mpnt, tmp);

                        tmp->vm_mm = mm;
                        retval = dup_userfaultfd(tmp, &uf);

                        if (tmp->vm_flags & VM_WIPEONFORK) {
                            /* VM_WIPEONFORK gets a clean slate in the child.
                             * Don't prepare anon_vma until fault since we don't
                             * copy page for current vma. */
                            tmp->anon_vma = NULL;
                        } else {
                            anon_vma_fork(tmp, mpnt) {
                                --->
                            }
                        }

                        vm_flags_clear(tmp, VM_LOCKED_MASK);
                        file = tmp->vm_file;
                        if (file) {
                            struct address_space *mapping = file->f_mapping;

                            get_file(file);
                            i_mmap_lock_write(mapping);
                            if (tmp->vm_flags & VM_SHARED)
                                mapping_allow_writable(mapping);

                            flush_dcache_mmap_lock(mapping);
                            /* insert tmp into the share list, just after mpnt */
                            vma_interval_tree_insert_after(tmp, mpnt, &mapping->i_mmap);
                            flush_dcache_mmap_unlock(mapping);
                            i_mmap_unlock_write(mapping);
                        }

                        /* Link the vma into the MT */
                        if (vma_iter_bulk_store(&vmi, tmp))
                            goto fail_nomem_vmi_store;

                        mm->map_count++;
                        if (!(tmp->vm_flags & VM_WIPEONFORK))
                            retval = copy_page_range(tmp, mpnt);

                        if (tmp->vm_ops && tmp->vm_ops->open)
                            tmp->vm_ops->open(tmp);

                        if (retval)
                            goto loop_out;
                    }
                    /* a new mm has just been created */
                    retval = arch_dup_mmap(oldmm, mm);
                }
            }
        }

        tsk->mm = mm;
        tsk->active_mm = mm;
    }
```