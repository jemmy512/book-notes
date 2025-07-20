# Table of Contents

<details>
<summary>Open * Close</summary>

* [socket](#socket)
* [bind](#bind)
* [listen](#listen)
* [connect](#connect)
    * [send](#send)
    * [receive](#receive)
        * [tcp_conn_request](#tcp_conn_request)
        * [tcp_rcv_synsent_state_process](#tcp_rcv_synsent_state_process)
        * [tcp_check_req](#tcp_check_req)
    * [call graph](#call-graph-connect)
* [accept](#accept)
* [accept queue](#accept-queue)
    * [TCP_LISTEN inet_csk_reqsk_queue_hash_add](#inet_csk_reqsk_queue_hash_add)
    * [TCP_NEW_SYN_RECV inet_ehash_nolisten](#inet_ehash_nolisten)
    * [TCP_NEW_SYN_RECV inet_csk_reqsk_queue_drop](#inet_csk_reqsk_queue_drop)
    * [TCP_NEW_SYN_RECV inet_csk_reqsk_queue_add](#inet_csk_reqsk_queue_add)
    * [Accept reqsk_queue_remove](#reqsk_queue_remove)
    * [queue_is_full_len](#queue_is_full_len)
* [shutdown](#shutdown)
* [sk_buf](#sk_buff)
    * [skb_clone](#skb_clone)
    * [kfree_skb_partial](#kfree_skb_partial)
* [write](#write)
    * [vfs layer](#vfs-layer-tx)
    * [socket layer](#socket-layer-tx)
    * [tcp layer](#tcp-layer-tx)
        * [copy data](#copy-data)
        * [tso_fragment](#tso_fragment)
        * [tcp_transmit_skb](#tcp_transmit_skb)
    * [ip layer](#ip-layer-tx)
        * [build ip header](#build-ip-header)
        * [send package](#send-package)
        * [skb_gso_segment](#skb_gso_segment)
            * [skb_mac_gso_segment](#skb_mac_gso_segment)
            * [inet_gso_segment](#inet_gso_segment)
            * [tcp4_gso_segment](#tcp4_gso_segment)
        * [ip_fragment](#ip_fragment)
    * [mac layer](#mac-layer-tx)
        * [neighbour](#neighbour)
        * [neigh_output](#neigh_output)
    * [dev layer](#dev-layer-tx)
    * [driver layer](#driver-layer-tx)
    * [call graph](#call-graph-write)

* [read](#read)
    * [driver layer](#driver-layer-rx)
    * [mac layer](#mac-layer-rx)
        * [vlan_do_receive](#vlan_do_receive)
        * [:link: br_handle_frame](#br_handle_frame)
    * [ip layer](#ip-layer-rx)
    * [tcp layer](#tcp-layer-rx)
        * [tcp_v4_rcv-TCP_NEW_SYN_RECV](#tcp_v4_rcv-TCP_NEW_SYN_RECV)
        * [tcp_v4_do_rcv-TCP_ESTABLISHED](#tcp_v4_do_rcv-TCP_ESTABLISHED)
        * [tcp_rcv_state_process](#tcp_rcv_state_process)
        * [tcp_data_queue](#tcp_data_queue)
    * [vfs layer](#vfs-layer-rx)
    * [socket layer](#socket-layer-rx])
    * [call graph](#call-graph-read)

* [epoll](#epoll)
    * [call graph](#call-graph-epoll)
    * [epoll_create](#epoll_create)
    * [epoll_ctl](#epoll_ctl)
        * [ep_insert](#ep_insert)
            * [ep_item_poll](#ep_item_poll)
        * [ep_modify](#ep_modify)
        * [ep_remove](#ep_remove)
    * [epoll_wait](#epoll_wait)
    * [sock_def_readable](#sock_def_readable)
    * [sock_def_write_space](#sock_def_write_space)
    * [eventpoll_init](#eventpoll_init)
    * [evetfd](#evetfd)

* [select](#select)
* [poll](#poll)

* [route](#route)
  * [route out](#route-out)
  * [route in](#route-in)

* [bridge](#bridge)
    * [br_add_bridge](#br_add_bridge)
    * [br_add_if](#br_add_if)
    * [br_handle_frame](#br_handle_frame)
        * [br_forward](#br_forward)
        * [br_pass_frame_up](#br_pass_frame_up)

* [congestion control](#congestion-control)
    * [tcp_ca_state](#tcp_ca_state)
        * [TCPF_CA_Recovery](#TCPF_CA_Recovery)
        * [TCP_CA_CWR](#TCP_CA_CWR)
        * [TCP_CA_Loss](#TCP_CA_Loss)
        * [TCP_CA_Disorder](#TCP_CA_Disorder)
    * [ecn](#ecn)
    * [tcp_ack](#tcp_ack)
    * [tcp_congestion_ops](#tcp_congestion_ops)
        * [ssthresh](#ssthresh)
        * [cong_avoid](#cong_avoid)
        * [cwnd_event](#cwnd_event)
        * [pkts_acked](#pkts_acked  )
        * [hystart](#hystart)
        * [set_state](#set_state)
    * [tcp_fastretrans_alert](#tcp_fastretrans_alert)
    * [tcp_cong_control](#tcp_cong_control)
    * [tcp_slow_start](#tcp_slow_start)
    * [tcp_cong_avoid](#tcp_cong_avoid)
    * [tcp_xmit_recovery](#tcp_xmit_recovery)
    * [tcp_cwnd_test](#tcp_cwnd_test)
    * [tcp_cwnd_validate](#tcp_cwnd_validate)

* [tcp_timer](#tcp_timer)
    * [timer_lifecycle](#timer_lifecycle)
      * [timer_init](#timer_init)
      * [timer_reset](#timer_reset)
      * [timer_clear](#timer_clear)

    * [tcp_write_timer](#tcp_write_timer)
        * [tcp_retransmit_skb](#tcp_retransmit_skb)
    * [tcp_delack_timer](#tcp_delack_timer)
    * [tcp_keepalive_timer](#tcp_keepalive_timer)
* [tcpdump](#tcpdump)
    * [register_prot_hook](#register_prot_hook)
    * [receive](#tcpdump_rcv)
    * [send](#tcpdump_snd)
* [ACK, SYN, FIN](#ACK-SYN-FIN)
    * [tcp_ack](#tcp_ack)
    * [tcp_send_ack](#tcp_send_ack)
    * [tcp_send_delayed_ack](#tcp_send_delayed_ack)
    * [tcp_send_synack](#tcp_send_synack)
    * [tcp_fin](#tcp_fin)
    * [tcp_send_fin](#tcp_send_fin)
* [tcp timer](#tcp_timer)
* [inet_init](#inet_init)
* [net_dev_init](#net_dev_init)
* [ebpf](#ebpf)

</details>

---

* [iximiuz Labs - Computer Networking Fundamentals For Developers, DevOps, and Platform Engineers](https://labs.iximiuz.com/courses/computer-networking-fundamentals)
* [Linux Thundering Herd Problem :cn:](https://mp.weixin.qq.com/s/dQWKBujtPcazzw7zacP1lg)
* [Linux Kernel TCP/IP Stack :cn:](https://mp.weixin.qq.com/s/EAEzLbsRIWyAv2x8tR3gNA)
* [十年码农内功 - 网络收发包详细流程图](https://mp.weixin.qq.com/s/JXYNQlGffSgwmQfxIi0ryQ)
* [The TTY demystified](http://www.linusakesson.net/programming/tty/index.php)
* [NAT - Network Address Translation](https://www.karlrupp.net/en/computer/nat_tutorial)
* [How NAT traversal works](https://tailscale.com/blog/how-nat-traversal-works)
* [Introduction to modern network load balancing and proxying](https://blog.envoyproxy.io/introduction-to-modern-network-load-balancing-and-proxying-a57f6ff80236)
* [[译] [论文] BBR：基于拥塞（而非丢包）的拥塞控制（ACM, 2017）](http://arthurchiao.art/blog/bbr-paper-zh/)
* [Linux 网络栈原理、监控与调优：前言（2022）](http://arthurchiao.art/blog/linux-net-stack-zh/)
    * [Linux 网络栈接收数据（RX）：配置调优（2022）](http://arthurchiao.art/blog/linux-net-stack-tuning-rx-zh/)
    * [Linux 网络栈接收数据（RX）：原理及内核实现（2022）](http://arthurchiao.art/blog/linux-net-stack-implementation-rx-zh/)
    * [Linux 中断（IRQ/softirq）基础：原理及内核实现（2022）](http://arthurchiao.art/blog/linux-irq-softirq-zh/)
* [Linux Network Performance Ultimate Guide](https://ntk148v.github.io/posts/linux-network-performance-ultimate-guide/)

---

<img src='../images/kernel/kernel-structual.svg' style='max-height:850px'/>

---

![](../images/kernel/net-read-write-route-bridge.svg)

* Each RX queue is associated with a separate IRQ, which can be routed to a different CPU.

| Concept | Type | Primary Function | Key Feature |
| :-: | :-: | :-: | :-: |
| **RSS** | Hardware | Distributes packets across NIC queues | Hardware-based load balancing via hash |
| **RPS** | Software | Distributes received packets across CPUs | Software-based load balancing |
| **RFS** | Software | Steers packets to application’s CPU | Improves cache locality with flow table |
| **aRFS** | Hardware | Hardware-accelerated packet steering | Uses NIC filters for application locality |
| **LRO** | Hardware | Aggregates packets in NIC | Hardware-based packet coalescing |
| **GRO** | Software | Aggregates incoming packets into larger ones | Software-based packet coalescing |
| **XPS** | Software | Maps CPUs to transmit queues | Optimizes transmit queue selection |
| **TSO/GSO** | Hardware/Software | Segments large packets for transmission | Offloads segmentation to NIC or kernel |
| **NAPI** | Software | Polling-based packet processing | Reduces interrupt overhead in high traffic |
| **BQL** | Software | Limits transmit queue buffering | Self-tuning for low-latency transmission |
| **QDisc** | Software | Manages transmit traffic | Configurable traffic prioritization |
| **Interrupt Coalescing** | Hardware | Batches NIC interrupts | Reduces CPU overhead with tunable batching |

* **GRO**: Enabled/disabled via `ethtool -K <interface> gro on|off`. Requires NAPI driver support.
* **RPS**: Configured via `/sys/class/net/<device>/queues/rx-<queue>/rps_cpus` with CPU bitmaps. Disabled by default (set to 0).
* **RFS**: Enabled by setting `/proc/sys/net/core/rps_sock_flow_entries` and `/sys/class/net/<device>/queues/rx-<queue>/rps_flow_cnt`.
* **aRFS**: Requires NIC support for ntuple filters (`ethtool -K <interface> ntuple on`) and driver support for ndo_rx_flow_steer.
* **XPS**: Configured via `/sys/class/net/<device>/queues/tx-<queue>/xps_cpus`.
* **LRO**: Enabled/disabled via `ethtool -K <interface> lro on|off`, but often disabled when GRO is available.
* **TSO/GSO**: Enabled/disabled via `ethtool -K <interface> tso|gso on|off`.
* **NAPI**: Enabled by default in modern drivers; no manual configuration typically needed.
* **BQL**: Self-tuning, but limits can be adjusted via ethtool or driver parameters.
* **QDisc**: Configured via the tc command for advanced traffic control.
* **Interrupt Coalescing**: Tuned via `ethtool -C <interface> rx-usecs <value>` or driver-specific settings.

---

![](../images/kernel/net-socket-sock.svg)
* While <span style="color:yellowgreen">struct proto</span> is protocol-specific (e.g., TCP vs. UDP), <span style="color:yellowgreen">struct proto_ops</span> is address-family-specific (e.g., IPv4 vs. IPv6).

---

<img src='../images/kernel/net-send-data-flow.png' style='max-height:850px'/>

| **Device** | **Function** | **Key Feature** |
| :-: | :-: | :-: |
| **Router** | Connects different networks, directs data packets | Uses IP addresses, often includes Wi-Fi |
| **Switch** | Connects devices within a network | Uses MAC addresses, efficient data flow |
| **Hub** | Connects multiple devices, broadcasts data | Outdated, less efficient than switches |
| **Access Point (AP)** | Provides wireless connectivity to a wired network | Extends Wi-Fi coverage |
| **Modem** | Converts digital/analog signals for internet access | Connects to ISP (cable, DSL, fiber) |
| **Firewall** | Monitors and controls network traffic for security | Can be hardware or software |
| **Bridge** | Connects two network segments into one | Filters traffic by MAC addresses |
| **Gateway** | Links networks with different protocols | Often a router, translates network types |
| **Network Interface Card (NIC)** | Connects a device to a network (wired/wireless) | Essential for network access |
| **Repeater** | Amplifies/regenerates signals to extend range | Boosts Wi-Fi or wired signals |
| **Load Balancer** | Distributes traffic across multiple servers | Optimizes performance, prevents overload |
| **Wireless Controller** | Manages multiple access points in large Wi-Fi networks | Centralizes control in enterprises |
| **Network Attached Storage (NAS)** | Provides file storage and sharing over a network | Acts as a networked file server |
| **Proxy Server** | Intermediary between users and the internet | Enhances security, caches data |

```c
struct socket_alloc {
  struct socket socket;
  struct inode  vfs_inode;
};

struct socket {
  socket_state            state;
  short                   type;
  unsigned long           flags;

  struct file             *file;
  struct sock             *sk;
  struct socket_wq        wq;
  const struct proto_ops  *ops;
};

struct sock {
  struct sock_common    __sk_common;
  struct sk_buff        *sk_rx_skb_cache;

  struct page_frag      sk_frag; /* copy user space data to this page */
  int                   sk_sndbuf; /* size of send buffer in bytes */
  int                   sk_rcvbuf; /* size of receive buffer in bytes*/
  struct sk_buff_head   sk_receive_queue;  /* incoming packets */
  struct sk_buff_head   sk_write_queue;    /* outgoing Packets */

  u32                   sk_ack_backlog;   /* current listen backlog */
  u32                   sk_max_ack_backlog; /* listen backlog set in listen() */

#define sk_nulls_node   __sk_common.skc_nulls_node; /* main hash linkage for TCP/UDP/UDP-Lite protocol */

  union {
    struct sk_buff    *sk_send_head;
    struct rb_root    tcp_rtx_queue; /* re-transmit queue */
  };

  struct {
    atomic_t          rmem_alloc;
    int               len;
    struct sk_buff    *head;
    struct sk_buff    *tail;
  } sk_backlog;

  struct sk_filter    *sk_filter;

  union {
    struct socket_wq  *sk_wq;     /* private: */
    struct socket_wq  *sk_wq_raw; /* public: */
  };

  struct dst_entry    *sk_rx_dst;
  struct dst_entry    *sk_dst_cache;

  unsigned int        sk_ll_usec;
  /* ===== mostly read cache line ===== */
  unsigned int        sk_napi_id; /* set by sk_mark_napi_id */
};

struct socket_wq {
  wait_queue_head_t      wait;
  struct fasync_struct   *fasync_list;
  unsigned long          flags;
  struct rcu_head        rcu;
};

typedef struct wait_queue_head {
  spinlock_t        lock;
  struct list_head  head;
} wait_queue_head_t;

struct tcp_sock {
  struct inet_connection_sock  inet_conn;

  struct list_head tsq_node; /* anchor in tsq_tasklet.head list */
  struct list_head tsorted_sent_queue; /* time-sorted sent but un-SACKed skbs */

  /* OOO segments go in this rbtree. Socket lock must be held. */
  struct rb_root    out_of_order_queue;
  struct sk_buff    *ooo_last_skb; /* cache rb_last(out_of_order_queue) */

  /* TCP fastopen related information */
  struct tcp_fastopen_request *fastopen_req;
  /* fastopen_rsk points to request_sock that resulted in this big
    * socket. Used to retransmit SYNACKs etc. */
  struct request_sock *fastopen_rsk;

  u32  packets_out;  /* Packets which are "in flight"  */
  u32  retrans_out;  /* Retransmitted packets out    */
  u32  max_packets_out;  /* max packets_out in last window */
  u32  max_packets_seq;  /* right edge of max_packets_out flight */

  /* Slow start and congestion control (see also Nagle, and Karn & Partridge) */
  u32  snd_ssthresh;  /* Slow start size threshold    */
  u32  snd_cwnd;  /* Sending congestion window    */
  u32  snd_cwnd_cnt;  /* Linear increase counter    */
  u32  snd_cwnd_clamp; /* Do not allow snd_cwnd to grow above this */
  u32  snd_cwnd_used;
  u32  snd_cwnd_stamp;
  u32  prior_cwnd;  /* cwnd right before starting loss recovery */
  u32  prr_delivered;  /* Number of newly delivered packets to
         * receiver in Recovery. */
  u32  prr_out;  /* Total number of pkts sent during Recovery. */
  u32  delivered;  /* Total data packets delivered incl. rexmits */
  u32  delivered_ce;  /* Like the above but only ECE marked packets */
  u32  lost;    /* Total data packets lost incl. rexmits */
  u32  app_limited;  /* limited until "delivered" reaches this val */
  u64  first_tx_mstamp;  /* start of window send phase */
  u64  delivered_mstamp; /* time we reached "delivered" */
  u32  rate_delivered;    /* saved rate sample: packets delivered */
  u32  rate_interval_us;  /* saved rate sample: time elapsed */

  u32  rcv_wnd;  /* Current receiver window    */
  u32  write_seq;  /* Tail(+1) of data held in tcp send buffer */
  u32  notsent_lowat;  /* TCP_NOTSENT_LOWAT */
  u32  pushed_seq;  /* Last pushed seq, required to talk to windows */
  u32  lost_out;  /* Lost packets      */
  u32  sacked_out;  /* SACK'd packets      */
};

/* INET connection oriented sock */
struct inet_connection_sock {
  struct inet_sock            icsk_inet;
  struct inet_bind_bucket     *icsk_bind_hash;   /* Bind node */
  struct request_sock_queue   icsk_accept_queue; /* FIFO of established children */
  struct tcp_congestion_ops           *icsk_ca_ops;
  struct inet_connection_sock_af_ops  *icsk_af_ops; /* Operations which are AF_INET{4,6} specific */

  struct timer_list           sk_timer;
  struct timer_list           icsk_retransmit_timer;
  struct timer_list           icsk_delack_timer;

  __u32                       icsk_rto;
};

struct request_sock_queue {
  spinlock_t            rskq_lock;
  u8                    rskq_defer_accept;

  u32                   synflood_warned;
  atomic_t              qlen; /* half connect queue len */
  atomic_t              young;

  struct request_sock  *rskq_accept_head;
  struct request_sock  *rskq_accept_tail;
  struct fastopen_queue  fastopenq;
  /* Check max_qlen != 0 to determine if TFO is enabled. */
};

/* representation of INET sockets */
struct inet_sock {
  /* sk and pinet6 has to be the first two members of inet_sock */
  struct sock    sk;
  struct ipv6_pinfo  *pinet6; /* pointer to IPv6 control block */

  /* Socket demultiplex comparisons on incoming packets. */
#define inet_daddr        sk.__sk_common.skc_daddr
#define inet_dport        sk.__sk_common.skc_dport
#define inet_rcv_saddr    sk.__sk_common.skc_rcv_saddr
#define inet_num          sk.__sk_common.skc_num /* Local port */

  __be32        inet_saddr;
  __be16        inet_sport;

  __s16         uc_ttl;
  __u16         cmsg_flags;
  struct ip_options_rcu __rcu  *inet_opt;
  __u16         inet_id;

  __u8          tos;
  __u8          min_ttl;
  __u8          mc_ttl;
  __u8          pmtudisc;
  __u8          recverr:1,
                is_icsk:1,
                freebind:1,
                hdrincl:1,
                mc_loop:1,
                transparent:1,
                mc_all:1,
                nodefrag:1;
  __u8          bind_address_no_port:1,
                recverr_rfc4884:1,
                defer_connect:1; /* Indicates that fastopen_connect is set
                * and cookie exists so we defer connect
                * until first data frame is written */
  __u8          rcv_tos;
  __u8          convert_csum;
  int           uc_index;
  int           mc_index;
  __be32        mc_addr;
  struct ip_mc_socklist __rcu  *mc_list;
  struct inet_cork_full  cork;
};
```

# socket


```c
socket() {
/* 1. create */
    sock_create() {
        struct socket *sock = _sock_create() {
            sock = sock_alloc() {
/* 1.1 create socket inode */
                inode = new_inode_pseudo(super_block *sb) {
                    alloc_inode(sb) {
                        if (sb->s_op->alloc_inode) {
                            inode = sb->s_op->alloc_inode(sb) {
                                sock_alloc_inode() {
                                    struct socket_alloc *ei = kmem_cache_alloc(sock_inode_cachep);
                                    struct socket_wq *wq = kmalloc(sizeof(*wq), GFP_KERNEL);
                                }
                            }
                        } else {
                            inode = kmem_cache_alloc(inode_cachep, GFP_KERNEL);
                        }
                    }
                }
                sock = SOCKET_I(inode);
                inode->i_op = &sockfs_inode_ops;
            }

/* 1.2 create sock */
            pf = net_families[family]; /* get AF */
            pf->create() { /* inet_family_ops.inet_create */
                inet_create() {
                    inet_protosw *answer = inetsw[sock->type]; /* get socket */
                    sock->ops = answer->ops;
                    struct sock *sk = sk_alloc() {
                        sk_prot_alloc() {
                            slab = prot->slab;
                            if (slab)
                                kmem_cache_alloc(slab);
                            else
                                kmalloc(prot->obj_size);
                        }
                    }

/* 1.3 init sock */
                    sock_init_data(sock, sk) {
                        sk_init_common(sk);

                        /* 1.3.1 init sock callback */
                        sk->sk_state_change = sock_def_wakeup;
                        sk->sk_data_ready = sock_def_readable;
                        sk->sk_write_space = sock_def_write_space;

                        sock->sk = sk;
                        sk->sk_socket = sock;
                    }

                    /* 1.3.2 no-listen hash */
                    sk->sk_prot->hash(sk) { /* inet_hash */
                        if (sk->sk_state != TCP_LISTEN) {
                            inet_ehash_nolisten(sk, osk, NULL);
                            return 0;
                        }
                    }

                    sk->sk_prot->init(sk) {
                        tcp_v4_init_sock() {
                            tcp_init_sock() {
                                tp->out_of_order_queue = RB_ROOT;
                                sk->tcp_rtx_queue = RB_ROOT;

                                /* 1.3.3 init timers */
                                tcp_init_xmit_timers() {
                                    inet_csk_init_xmit_timers(sk,
                                        &tcp_write_timer,
                                        &tcp_delack_timer,
                                        &tcp_keepalive_timer
                                    );
                                }

                                /* 1.3.4 init congestion control */
                                sk->sk_state = TCP_CLOSE;
                                tp->snd_cwnd = TCP_INIT_CWND;
                                tp->snd_ssthresh = TCP_INFINITE_SSTHRESH;
                                tcp_assign_congestion_control() {
                                    icsk->icsk_ca_ops = rcu_dereference(net->ipv4.tcp_congestion_control);
                                }

                                /* 1.3.5 init rx tx buffer */
                                sk->sk_sndbuf = sock_net(sk)->ipv4.sysctl_tcp_wmem[1];
                                sk->sk_rcvbuf = sock_net(sk)->ipv4.sysctl_tcp_rmem[1];
                            }
                        }
                        icsk->icsk_af_ops = &ipv4_specific;
                        tcp_sk(sk)->af_specific = &tcp_sock_ipv4_specific;
                    }
                }
            }
        }
    }


/* 2. map */
    sock_map_fd() {
        fd = get_unused_fd_flags();

        new_file = sock_alloc_file() {
            alloc_file_pseudo(inode, mnt, dname, flags, &socket_file_ops) {
                d_alloc_pseudo();
                d_set_d_op(&anon_ops);
                alloc_file() {
                    struct file *file = alloc_empty_file(flags, current_cred()) {
                        kmem_cache_zalloc(filp_cachep);
                    }
                    file->f_path = *path;
                    file->f_inode = path->dentry->d_inode;
                    file->f_mapping = path->dentry->d_inode->i_mapping;
                    file->f_op = socket_file_ops;
                }
            }
            sock->file = file;
            file->private_data = sock;
        }

        fd_install(fd, newfile);
    }
}
```

```c
SYSCALL_DEFINE3(socket, int, family, int, type, int, protocol)
{
  int retval;
  struct socket *sock;
  int flags;

  if (SOCK_NONBLOCK != O_NONBLOCK && (flags & SOCK_NONBLOCK))
    flags = (flags & ~SOCK_NONBLOCK) | O_NONBLOCK;

  retval = sock_create(family, type, protocol, &sock);
  retval = sock_map_fd(sock, flags & (O_CLOEXEC | O_NONBLOCK));

  return retval;
}

int sock_create(int family, int type, int protocol, struct socket **res)
{
  return __sock_create(current->nsproxy->net_ns, family, type, protocol, res, 0);
}

int __sock_create(
  struct net *net, int family, int type, int protocol,
  struct socket **res, int kern)
{
  int err;
  struct socket *sock;
  const struct net_proto_family *pf;

  /* Allocate the socket and allow the family to set things up */
  sock = sock_alloc();
  sock->type = type;

  pf = rcu_dereference(net_families[family]);
  err = pf->create(net, sock, protocol, kern);

  *res = sock;

  return 0;
}

struct socket *sock_alloc(void)
{
  struct inode *inode;
  struct socket *sock;

  inode = new_inode_pseudo(sock_mnt->mnt_sb);
  sock = SOCKET_I(inode);

  inode->i_ino = get_next_ino();
  inode->i_mode = S_IFSOCK | S_IRWXUGO;
  inode->i_uid = current_fsuid();
  inode->i_gid = current_fsgid();
  inode->i_op = &sockfs_inode_ops;

  return sock;
}

struct inode *new_inode_pseudo(struct super_block *sb)
{
  struct inode *inode = alloc_inode(sb);

  if (inode) {
    spin_lock(&inode->i_lock);
    inode->i_state = 0;
    spin_unlock(&inode->i_lock);
    INIT_LIST_HEAD(&inode->i_sb_list);
  }
  return inode;
}

const struct super_operations sockfs_ops = {
  .alloc_inode    = sock_alloc_inode,
  .destroy_inode  = sock_destroy_inode,
  .statfs         = simple_statfs,
};

struct inode *alloc_inode(struct super_block *sb)
{
  struct inode *inode;

  if (sb->s_op->alloc_inode)
    inode = sb->s_op->alloc_inode(sb); /* sock_alloc_inode */
  else
    inode = kmem_cache_alloc(inode_cachep, GFP_KERNEL);

  if (!inode)
    return NULL;

  if (unlikely(inode_init_always(sb, inode))) {
    if (inode->i_sb->s_op->destroy_inode)
      inode->i_sb->s_op->destroy_inode(inode);
    else
      kmem_cache_free(inode_cachep, inode);
    return NULL;
  }

  return inode;
}

struct kmem_cache *sock_inode_cachep __ro_after_init;

struct inode *sock_alloc_inode(struct super_block *sb)
{
  struct socket_alloc *ei;
  struct socket_wq *wq;

  ei = kmem_cache_alloc(sock_inode_cachep, GFP_KERNEL);
  if (!ei)
    return NULL;
  wq = kmalloc(sizeof(*wq), GFP_KERNEL);
  if (!wq) {
    kmem_cache_free(sock_inode_cachep, ei);
    return NULL;
  }
  init_waitqueue_head(&wq->wait);
  wq->fasync_list = NULL;
  wq->flags = 0;
  ei->socket.wq = wq;

  ei->socket.state = SS_UNCONNECTED;
  ei->socket.flags = 0;
  ei->socket.ops = NULL;
  ei->socket.sk = NULL;
  ei->socket.file = NULL;

  return &ei->vfs_inode;
}

enum sock_type {
  SOCK_STREAM = 1,
  SOCK_DGRAM  = 2,
  SOCK_RAW    = 3,
};

/* Supported address families. */
#define AF_UNSPEC 0
#define AF_UNIX   1   /* Unix domain sockets */
#define AF_LOCAL  1   /* POSIX name for AF_UNIX */
#define AF_INET   2   /* Internet IP Protocol */
#define AF_INET6  10  /* IP version 6 */
#define AF_MPLS   28  /* MPLS */
#define AF_MAX    44  /* For now */
#define NPROTO    AF_MAX

struct net_proto_family *net_families[NPROTO];
/* net/ipv4/af_inet.c */
const struct net_proto_family inet_family_ops = {
  .family = PF_INET,
  .create = inet_create
}

int inet_create(
  struct net *net, struct socket *sock, int protocol, int kern)
{
  struct sock *sk;
  struct inet_protosw *answer;
  struct inet_sock *inet;
  struct proto *answer_prot;
  unsigned char answer_flags;
  int try_loading_module = 0;
  int err;

  sock->state = SS_UNCONNECTED;

  /* Look for the requested type/protocol pair. */
lookup_protocol:
  list_for_each_entry_rcu(answer, &inetsw[sock->type], list) {
    err = 0;
    /* Check the non-wild match. */
    if (protocol == answer->protocol) {
      if (protocol != IPPROTO_IP)
        break;
    } else {
      /* Check for the two wild cases. */
      if (IPPROTO_IP == protocol) {
        protocol = answer->protocol;
        break;
      }
      if (IPPROTO_IP == answer->protocol)
        break;
    }
    err = -EPROTONOSUPPORT;
  }

  sock->ops = answer->ops;
  answer_prot = answer->prot;
  answer_flags = answer->flags;

  sk = sk_alloc(net, PF_INET, GFP_KERNEL, answer_prot, kern);

  inet = inet_sk(sk);
  inet->nodefrag = 0;
  if (SOCK_RAW == sock->type) {
    inet->inet_num = protocol;
    if (IPPROTO_RAW == protocol)
      inet->hdrincl = 1;
  }
  inet->inet_id = 0;
  sock_init_data(sock, sk);

  sk->sk_destruct     = inet_sock_destruct;
  sk->sk_protocol     = protocol;
  sk->sk_backlog_rcv = sk->sk_prot->backlog_rcv;

  inet->uc_ttl  = -1;
  inet->mc_loop  = 1;
  inet->mc_ttl  = 1;
  inet->mc_all  = 1;
  inet->mc_index  = 0;
  inet->mc_list  = NULL;
  inet->rcv_tos  = 0;

  if (inet->inet_num) {
    inet->inet_sport = htons(inet->inet_num);
    err = sk->sk_prot->hash(sk); /* inet_hash */
  }

  if (sk->sk_prot->init) {
    err = sk->sk_prot->init(sk); /* tcp_v4_init_sock() */
  }
}

struct sock *sk_alloc(
  struct net *net, int family, gfp_t priority,
  struct proto *prot, int kern)
{
  struct sock *sk;

  sk = sk_prot_alloc(prot, priority | __GFP_ZERO, family);
  if (sk) {
    sk->sk_family = family;
    sk->sk_prot = sk->sk_prot_creator = prot;
    sk->sk_kern_sock = kern;
    sock_lock_init(sk);
    sk->sk_net_refcnt = kern ? 0 : 1;
    if (likely(sk->sk_net_refcnt)) {
      get_net(net);
      sock_inuse_add(net, 1);
    }

    sock_net_set(sk, net);
    refcount_set(&sk->sk_wmem_alloc, 1);

    mem_cgroup_sk_alloc(sk);
    cgroup_sk_alloc(&sk->sk_cgrp_data);
    sock_update_classid(&sk->sk_cgrp_data);
    sock_update_netprioidx(&sk->sk_cgrp_data);
    sk_tx_queue_clear(sk);
  }

  return sk;
}

struct sock *sk_prot_alloc(
  struct proto *prot, gfp_t priority, int family)
{
  struct sock *sk;
  struct kmem_cache *slab;

  slab = prot->slab;
  if (slab != NULL) {
    sk = kmem_cache_alloc(slab, priority & ~__GFP_ZERO);
    if (!sk)
      return sk;
    if (priority & __GFP_ZERO)
      sk_prot_clear_nulls(sk, prot->obj_size);
  } else
    sk = kmalloc(prot->obj_size, priority);

  if (sk != NULL) {
    if (security_sk_alloc(sk, family, priority))
      goto out_free;

    if (!try_module_get(prot->owner))
      goto out_free_sec;
    sk_tx_queue_clear(sk);
  }

  return sk;

out_free_sec:
  security_sk_free(sk);
out_free:
  if (slab != NULL)
    kmem_cache_free(slab, sk);
  else
    kfree(sk);
  return NULL;
}

/* sw: switch */
struct list_head inetsw[SOCK_MAX];
int __init inet_init(void)
{
  /* Register the socket-side information for inet_create. */
  for (r = &inetsw[0]; r < &inetsw[SOCK_MAX]; ++r)
    INIT_LIST_HEAD(r);
  for (q = inetsw_array; q < &inetsw_array[INETSW_ARRAY_LEN]; ++q)
    inet_register_protosw(q);
}

struct inet_protosw inetsw_array[] =
{
  {
    .type       = SOCK_STREAM,
    .protocol   = IPPROTO_TCP,
    .prot       = &tcp_prot,
    .ops        = &inet_stream_ops,
    .flags      = INET_PROTOSW_PERMANENT | INET_PROTOSW_ICSK,
  },
  {
    .type       = SOCK_DGRAM,
    .protocol   = IPPROTO_UDP,
    .prot       = &udp_prot,
    .ops        = &inet_dgram_ops,
    .flags      = INET_PROTOSW_PERMANENT,
  },
  {
    .type       = SOCK_DGRAM,
    .protocol   = IPPROTO_ICMP,
    .prot       = &ping_prot,
    .ops        = &inet_sockraw_ops,
    .flags      = INET_PROTOSW_REUSE,
  },
  {
    .type       = SOCK_RAW,
    .protocol   = IPPROTO_IP,  /* wild card */
    .prot       = &raw_prot,
    .ops        = &inet_sockraw_ops,
    .flags      = INET_PROTOSW_REUSE,
  }
};

/* sock_ops is better, sock_{steam, dgram, raw}_ops */
const struct proto_ops inet_stream_ops = {
  .family       = PF_INET,
  .owner        = THIS_MODULE,
  .bind         = inet_bind,
  .connect      = inet_stream_connect,
  .accept       = inet_accept,
  .listen       = inet_listen
  .sendmsg      = inet_sendmsg,
  .recvmsg      = inet_recvmsg
};

struct proto tcp_prot = {
  .name         = "TCP",
  .owner        = THIS_MODULE,
  .connect      = tcp_v4_connect,
  .recvmsg      = tcp_recvmsg,
  .sendmsg      = tcp_sendmsg,
  .get_port     = inet_csk_get_port,
  .backlog_rcv  = tcp_v4_do_rcv,
  .hash         = inet_hash,
  .unhash       = inet_unhash,
  .h.hashinfo   = &tcp_hashinfo
}
```

```c
int sock_map_fd(struct socket *sock, int flags)
{
  struct file *newfile;
  int fd = get_unused_fd_flags(flags);
  if (unlikely(fd < 0)) {
    sock_release(sock);
    return fd;
  }

  newfile = sock_alloc_file(sock, flags, NULL);
  if (likely(!IS_ERR(newfile))) {
    fd_install(fd, newfile);
    return fd;
  }

  put_unused_fd(fd);
  return PTR_ERR(newfile);
}

struct file *sock_alloc_file(struct socket *sock, int flags, const char *dname)
{
  struct file *file;

  if (!dname)
    dname = sock->sk ? sock->sk->sk_prot_creator->name : "";

  file = alloc_file_pseudo(SOCK_INODE(sock), sock_mnt, dname,
        O_RDWR | (flags & O_NONBLOCK),
        &socket_file_ops);
  if (IS_ERR(file)) {
    sock_release(sock);
    return file;
  }

  sock->file = file;
  file->private_data = sock;
  return file;
}

struct file *alloc_file_pseudo(
  struct inode *inode, struct vfsmount *mnt,
  const char *name, int flags,
  const struct file_operations *fops)
{
  static const struct dentry_operations anon_ops = {
    .d_dname = simple_dname
  };

  struct qstr this = QSTR_INIT(name, strlen(name));
  struct path path;
  struct file *file;

  path.dentry = d_alloc_pseudo(mnt->mnt_sb, &this);

  if (!mnt->mnt_sb->s_d_op)
    d_set_d_op(path.dentry, &anon_ops);

  path.mnt = mntget(mnt);
  d_instantiate(path.dentry, inode);

  file = alloc_file(&path, flags, fops);
  if (IS_ERR(file)) {
    ihold(inode);
    path_put(&path);
  }
  return file;
}

struct file *alloc_file(
  const struct path *path, int flags,
  const struct file_operations *fop)
{
  struct file *file = alloc_empty_file(flags, current_cred());

  file->f_op = fop;
  file->f_path = *path;
  file->f_inode = path->dentry->d_inode;
  file->f_mapping = path->dentry->d_inode->i_mapping;
  file->f_wb_err = filemap_sample_wb_err(file->f_mapping);

  if ((file->f_mode & FMODE_READ) && likely(fop->read || fop->read_iter))
    file->f_mode |= FMODE_CAN_READ;
  if ((file->f_mode & FMODE_WRITE) && likely(fop->write || fop->write_iter))
    file->f_mode |= FMODE_CAN_WRITE;

  file->f_mode |= FMODE_OPENED;

  if ((file->f_mode & (FMODE_READ | FMODE_WRITE)) == FMODE_READ)
    i_readcount_inc(path->dentry->d_inode);

  return file;
}
```

```c
void sock_init_data(struct socket *sock, struct sock *sk)
{
  sk_init_common(sk);
  sk->sk_send_head  =  NULL;

  timer_setup(&sk->sk_timer, NULL, 0);

  sk->sk_allocation  =  GFP_KERNEL;
  sk->sk_rcvbuf    =  sysctl_rmem_default;
  sk->sk_sndbuf    =  sysctl_wmem_default;
  sk->sk_state    =  TCP_CLOSE;

  sk_set_socket(sk, sock);

  sock_set_flag(sk, SOCK_ZAPPED);

  if (sock) {
    sk->sk_type  =  sock->type;
    sk->sk_wq  =  sock->wq;
    sock->sk  =  sk;
    sk->sk_uid  =  SOCK_INODE(sock)->i_uid;
  } else {
    sk->sk_wq  =  NULL;
    sk->sk_uid  =  make_kuid(sock_net(sk)->user_ns, 0);
  }

  rwlock_init(&sk->sk_callback_lock);
  if (sk->sk_kern_sock)
    lockdep_set_class_and_name(
      &sk->sk_callback_lock,
      af_kern_callback_keys + sk->sk_family,
      af_family_kern_clock_key_strings[sk->sk_family]);
  else
    lockdep_set_class_and_name(
      &sk->sk_callback_lock,
      af_callback_keys + sk->sk_family,
      af_family_clock_key_strings[sk->sk_family]);

  sk->sk_state_change  =  sock_def_wakeup;
  sk->sk_data_ready  =  sock_def_readable;
  sk->sk_write_space  =  sock_def_write_space;
  sk->sk_error_report  =  sock_def_error_report;
  sk->sk_destruct    =  sock_def_destruct;

  sk->sk_frag.page  =  NULL;
  sk->sk_frag.offset  =  0;
  sk->sk_peek_off    =  -1;

  sk->sk_peer_pid   =  NULL;
  sk->sk_peer_cred  =  NULL;
  sk->sk_write_pending  =  0;
  sk->sk_rcvlowat    =  1;
  sk->sk_rcvtimeo    =  MAX_SCHEDULE_TIMEOUT;
  sk->sk_sndtimeo    =  MAX_SCHEDULE_TIMEOUT;

  sk->sk_stamp = SK_DEFAULT_STAMP;
#if BITS_PER_LONG==32
  seqlock_init(&sk->sk_stamp_seq);
#endif
  atomic_set(&sk->sk_zckey, 0);

#ifdef CONFIG_NET_RX_BUSY_POLL
  sk->sk_napi_id    =  0;
  sk->sk_ll_usec    =  sysctl_net_busy_read;
#endif

  sk->sk_max_pacing_rate = ~0U;
  sk->sk_pacing_rate = ~0U;
  sk->sk_pacing_shift = 10;
  sk->sk_incoming_cpu = -1;

  sk_rx_queue_clear(sk);
  /* Before updating sk_refcnt, we must commit prior changes to memory
   * (Documentation/RCU/rculist_nulls.txt for details) */
  smp_wmb();
  refcount_set(&sk->sk_refcnt, 1);
  atomic_set(&sk->sk_drops, 0);
}

/* sk_prot->init(sk); */
int tcp_v4_init_sock(struct sock *sk)
{
  struct inet_connection_sock *icsk = inet_csk(sk);

  tcp_init_sock(sk);

  icsk->icsk_af_ops = &ipv4_specific;

  tcp_sk(sk)->af_specific = &tcp_sock_ipv4_specific;

  return 0;
}

void tcp_init_sock(struct sock *sk)
{
  struct inet_connection_sock *icsk = inet_csk(sk);
  struct tcp_sock *tp = tcp_sk(sk);

  tp->out_of_order_queue = RB_ROOT;
  sk->tcp_rtx_queue = RB_ROOT;
  tcp_init_xmit_timers(sk);
  INIT_LIST_HEAD(&tp->tsq_node);
  INIT_LIST_HEAD(&tp->tsorted_sent_queue);

  icsk->icsk_rto = TCP_TIMEOUT_INIT;
  tp->mdev_us = jiffies_to_usecs(TCP_TIMEOUT_INIT);
  minmax_reset(&tp->rtt_min, tcp_jiffies32, ~0U);

  /* So many TCP implementations out there (incorrectly) count the
   * initial SYN frame in their delayed-ACK and congestion control
   * algorithms that we must have the following bandaid to talk
   * efficiently to them.  -DaveM */
  tp->snd_cwnd = TCP_INIT_CWND;

  /* There's a bubble in the pipe until at least the first ACK. */
  tp->app_limited = ~0U;

  /* See draft-stevens-tcpca-spec-01 for discussion of the
   * initialization of these values. */
  tp->snd_ssthresh = TCP_INFINITE_SSTHRESH;
  tp->snd_cwnd_clamp = ~0;
  tp->mss_cache = TCP_MSS_DEFAULT;

  tp->reordering = sock_net(sk)->ipv4.sysctl_tcp_reordering;
  tcp_assign_congestion_control(sk);

  tp->tsoffset = 0;
  tp->rack.reo_wnd_steps = 1;

  sk->sk_state = TCP_CLOSE;

  sk->sk_write_space = sk_stream_write_space;
  sock_set_flag(sk, SOCK_USE_WRITE_QUEUE);

  icsk->icsk_sync_mss = tcp_sync_mss;

  sk->sk_sndbuf = sock_net(sk)->ipv4.sysctl_tcp_wmem[1];
  sk->sk_rcvbuf = sock_net(sk)->ipv4.sysctl_tcp_rmem[1];

  sk_sockets_allocated_inc(sk);
  sk->sk_route_forced_caps = NETIF_F_GSO;
}
```

# bind
```c
SYSCALL_DEFINE3(bind, int, fd, struct sockaddr __user *, umyaddr, int, addrlen)
{
  struct socket *sock;
  struct sockaddr_storage address;
  int err, fput_needed;

  sock = sockfd_lookup_light(fd, &err, &fput_needed);
  if (sock) {
    err = move_addr_to_kernel(umyaddr, addrlen, &address);
    if (err >= 0) {
      err = sock->ops->bind(sock, (struct sockaddr *)&address, addrlen);
    }
    fput_light(sock->file, fput_needed);
  }
  return err;
}

/* inet_stream_ops.bind */
int inet_bind(struct socket *sock, struct sockaddr *uaddr, int addr_len)
{
  struct sock *sk = sock->sk;
  u32 flags = BIND_WITH_LOCK;
  int err;

  if (sk->sk_prot->bind) {
    return sk->sk_prot->bind(sk, uaddr, addr_len);
  }
  if (addr_len < sizeof(struct sockaddr_in))
    return -EINVAL;

  if (err)
    return err;

  return __inet_bind(sk, uaddr, addr_len, flags);
}

int __inet_bind(struct socket *sock, struct sockaddr *uaddr, int addr_len)
{
  struct sockaddr_in *addr = (struct sockaddr_in *)uaddr;
  struct sock *sk = sock->sk;
  struct inet_sock *inet = inet_sk(sk);
  struct net *net = sock_net(sk);
  unsigned short snum;
  snum = ntohs(addr->sin_port);

  inet->inet_rcv_saddr = inet->inet_saddr = addr->sin_addr.s_addr;
  /* Make sure we are allowed to bind here. */
  if (snum || !(inet->bind_address_no_port || force_bind_address_no_port)) {
    /* use inet_csk_get_port to check confilct of port */
    if (sk->sk_prot->get_port(sk, snum)) {
      inet->inet_saddr = inet->inet_rcv_saddr = 0;
      err = -EADDRINUSE;
      goto out_release_sock;
    }
    err = BPF_CGROUP_RUN_PROG_INET4_POST_BIND(sk);
    if (err) {
      inet->inet_saddr = inet->inet_rcv_saddr = 0;
      goto out_release_sock;
    }
  }
  inet->inet_sport = htons(inet->inet_num);
  inet->inet_daddr = 0;
  inet->inet_dport = 0;
  sk_dst_reset(sk);
}
```

# listen


```c
listen() {
    sockfd_lookup_light();
    sock->ops->listen() { /*inet_stream_ops.listen */
        inet_listen() {
            inet_csk_listen_start() {
                /* allock request sock queue */
                reqsk_queue_alloc() {
                    queue->fastopenq.rskq_rst_head = NULL;
                    queue->fastopenq.rskq_rst_tail = NULL;
                    queue->fastopenq.qlen = 0;
                }

                inet_csk_delack_init(sk);
                sk_state_store(sk, TCP_LISTEN);

                sk->sk_prot->get_port();

                /* insert into lhash2 */
                sk->sk_prot->hash(sk) {
                    inet_hash() {
                        if (sk->sk_state != TCP_LISTEN) {
                            inet_ehash_nolisten(sk, osk);
                            return 0;
                        }

                        if (sk->sk_reuseport) {
                            inet_reuseport_add_sock();
                            return;
                        }

                        struct inet_listen_hashbucket* ilb2 = inet_lhash2_bucket_sk(hashinfo, sk);
                        __sk_nulls_add_node_rcu(sk, &ilb2->nulls_head);
                    }
                }
            }
            sk->sk_max_ack_backlog = backlog;
        }
    }
}
```

```c
SYSCALL_DEFINE2(listen, int, fd, int, backlog)
{
    struct socket *sock;
    int err, fput_needed;
    int somaxconn;

    sock = sockfd_lookup_light(fd, &err, &fput_needed);
    if (sock) {
        somaxconn = sock_net(sock->sk)->core.sysctl_somaxconn;
        if ((unsigned int)backlog > somaxconn)
            backlog = somaxconn;
        err = sock->ops->listen(sock, backlog);
        fput_light(sock->file, fput_needed);
    }
    return err;
}

/* inet_stream_ops.listen */
int inet_listen(struct socket *sock, int backlog)
{
    struct sock *sk = sock->sk;
    unsigned char old_state;
    int err, tcp_fastopen;

    lock_sock(sk);

    err = -EINVAL;
    if (sock->state != SS_UNCONNECTED || sock->type != SOCK_STREAM)
        goto out;

    old_state = sk->sk_state;
    if (!((1 << old_state) & (TCPF_CLOSE | TCPF_LISTEN)))
        goto out;

    /* Really, if the socket is already in listen state
     * we can only allow the backlog to be adjusted. */
    if (old_state != TCP_LISTEN) {
        /* Enable TFO w/o requiring TCP_FASTOPEN socket option.
        * Note that only TCP sockets (SOCK_STREAM) will reach here.
        * Also fastopen backlog may already been set via the option
        * because the socket was in TCP_LISTEN state previously but
        * was shutdown() rather than close(). */
        tcp_fastopen = sock_net(sk)->ipv4.sysctl_tcp_fastopen;
        if ((tcp_fastopen & TFO_SERVER_WO_SOCKOPT1) &&
            (tcp_fastopen & TFO_SERVER_ENABLE) &&
            !inet_csk(sk)->icsk_accept_queue.fastopenq.max_qlen)
        {
            fastopen_queue_tune(sk, backlog);
            tcp_fastopen_init_key_once(sock_net(sk));
        }

        err = inet_csk_listen_start(sk, backlog);
        if (err)
            goto out;
        tcp_call_bpf(sk, BPF_SOCK_OPS_TCP_LISTEN_CB, 0, NULL);
    }
    sk->sk_max_ack_backlog = backlog;
    err = 0;

out:
    release_sock(sk);
    return err;
}

int inet_csk_listen_start(struct sock *sk, int backlog)
{
    struct inet_connection_sock *icsk = inet_csk(sk);
    struct inet_sock *inet = inet_sk(sk);
    int err = -EADDRINUSE;

    reqsk_queue_alloc(&icsk->icsk_accept_queue); /* FIFO of established children */

    sk->sk_max_ack_backlog = backlog;            /* listen backlog set in listen() */
    sk->sk_ack_backlog = 0;                      /* current listen backlog */
    inet_csk_delack_init(sk);

    sk_state_store(sk, TCP_LISTEN);
    /* socket enters to hash table only after validation is complete */
    if (!sk->sk_prot->get_port(sk, inet->inet_num)) {
        inet->inet_sport = htons(inet->inet_num);

        sk_dst_reset(sk);
        err = sk->sk_prot->hash(sk);

        if (likely(!err))
        return 0;
    }

    inet_sk_set_state(sk, TCP_CLOSE);
    return err;
}

void reqsk_queue_alloc(struct request_sock_queue *queue)
{
    spin_lock_init(&queue->rskq_lock);

    spin_lock_init(&queue->fastopenq.lock);
    queue->fastopenq.rskq_rst_head = NULL;
    queue->fastopenq.rskq_rst_tail = NULL;
    queue->fastopenq.qlen = 0;

    queue->rskq_accept_head = NULL;
}

struct proto tcp_prot = {
    .hash = inet_hash;
}

int inet_hash(struct sock *sk)
{
    int err = 0;

    if (sk->sk_state != TCP_CLOSE) {
        local_bh_disable();
        err = __inet_hash(sk, NULL);
        local_bh_enable();
    }

    return err;
}

/* net/ipv4/tcp_ipv4.c */
struct inet_hashinfo tcp_hashinfo;

int __inet_hash(struct sock *sk, struct sock *osk)
{
    struct inet_hashinfo *hashinfo = sk->sk_prot->h.hashinfo; /* tcp_hashinfo; */
    struct inet_listen_hashbucket *ilb;
    int err = 0;

    if (sk->sk_state != TCP_LISTEN) {
        inet_ehash_nolisten(sk, osk);
        return 0;
    }

    ilb = &hashinfo->listening_hash[inet_sk_listen_hashfn(sk)];

    spin_lock(&ilb->lock);
    if (sk->sk_reuseport) {
        err = inet_reuseport_add_sock(sk, ilb);
        if (err)
        goto unlock;
    }
    if (IS_ENABLED(CONFIG_IPV6) && sk->sk_reuseport && sk->sk_family == AF_INET6)
        __sk_nulls_add_node_tail_rcu(sk, &ilb->nulls_head);
    else
        __sk_nulls_add_node_rcu(sk, &ilb->nulls_head);

    inet_hash2(hashinfo, sk);
    ilb->count++;
    sock_set_flag(sk, SOCK_RCU_FREE);
    sock_prot_inuse_add(sock_net(sk), sk->sk_prot, 1);
unlock:
    spin_unlock(&ilb->lock);

    return err;
}

void inet_hash2(struct inet_hashinfo *h, struct sock *sk)
{
    struct inet_listen_hashbucket *ilb2;

    if (!h->lhash2)
        return;

    ilb2 = inet_lhash2_bucket_sk(h, sk);

    spin_lock(&ilb2->lock);
    if (sk->sk_reuseport && sk->sk_family == AF_INET6)
        hlist_add_tail_rcu(&inet_csk(sk)->icsk_listen_portaddr_node, &ilb2->head);
    else
        hlist_add_head_rcu(&inet_csk(sk)->icsk_listen_portaddr_node, &ilb2->head);
    ilb2->count++;
    spin_unlock(&ilb2->lock);
}
```

# connect

<img src='../images/kernel/net-hand-shake.svg' style='max-height:850px'/>


```c
send:
connect() {
    sock = sockfd_lookup_light();
    sock->ops->connect(sock) {
        __inet_stream_connect() {
            sk->sk_prot->connect() { /* SS_UNCONNECTED */
                tcp_v4_connect() {
                    ip_route_connect();
                    inet_hash_connect();
                    secure_tcp_seq();
                    secure_tcp_ts_off();
                    tcp_set_state(sk, TCP_SYN_SENT);

                    tcp_connect() {
                        tcp_connect_init() {
                            tcp_select_initial_window();
                            tp->snd_una = tp->write_seq;
                            tp->snd_nxt = tp->write_seq;
                        }

                        sk_stream_alloc_skb();
                        tcp_init_nondata_skb(TCPHDR_SYN);
                        tcp_transmit_skb();
                        tcp_send_head();
                        inet_csk_reset_xmit_timer();
                    }
                }
            }

            if (!(flags & O_NONBLOCK)) {
                inet_wait_for_connect()
                    /* woken up by sk->sk_state_change() at tcp_rcv_synsent_state_process */
            }
        }
    }
}



receive:
tcp_v4_rcv() {
    sk = __inet_lookup_skb() {
        __inet_lookup_established();
        __inet_lookup_listener();
    }

    if (sk->sk_state == TCP_NEW_SYN_RECV) {
        struct request_sock *req = inet_reqsk(sk);
        sk = req->rsk_listener;

        newsk = tcp_check_req() {
            child = inet_csk(sk)->icsk_af_ops->syn_recv_sock() {
                tcp_v4_syn_recv_sock() {
                    if (sk_acceptq_is_full());
                        goto drop;
                    newsk = tcp_create_openreq_child(sk, req, skb) {
                        inet_csk_clone_lock() {
                            inet_sk_set_state(newsk, TCP_SYN_RECV) {
                                sk->sk_state = TCP_SYN_RECV;
                            }
                        tcp_init_xmit_timers();
                        }
                    }

                    inet_ehash_nolisten() {
                        inet_ehash_insert() { /* hash insert the new sk and remove the old request_sk */
                            inet_ehash_bucket(hashinfo, sk->sk_hash);
                            __sk_nulls_add_node_rcu();
                                hlist_nulls_add_head_rcu(&sk->sk_nulls_node, list);
                            if (osk)
                                sk_nulls_del_node_init_rcu(osk);
                        }
                    }
                }
            }

            inet_csk_complete_hashdance(child) {
                inet_csk_reqsk_queue_drop() {
                    reqsk_queue_unlink() {
                        __sk_nulls_del_node_init_rcu(sk) {
                            hlist_nulls_del_init_rcu(&sk->sk_nulls_node);
                        }
                    }
                    reqsk_queue_removed() {
                        --icsk_accept_queue.yong;
                        --icsk_accept_queue.qlen;
                    }
                }
                reqsk_queue_removed();

                inet_csk_reqsk_queue_add() {
                    sk->rskq_accept_tail = reqst_sk;
                    sk_acceptq_added() {
                        icsk_accept_queue->rskq_accept_tail = req;
                        sk_acceptq_added();
                            ++sk->sk_ack_backlog
                    }
                }
            }
        }

        tcp_child_process() {
            if (!sock_owned_by_user(child)) {
                tcp_rcv_state_process(child, skb);
                sk_listener->sk_data_ready() /* sock_def_readable */
            } else {
                __sk_add_backlog(child, skb);
            }
        }
    }

    tcp_filter(sk, skb);
    tcp_v4_fill_cb(skb, iph, th);

    tcp_v4_do_rcv() {
        tcp_rcv_state_process() {
            if (sk->sk_state == TCP_LISTEN) {
                icsk->icsk_af_ops->conn_request() { /* ipv4_specific.conn_request */
                    tcp_v4_conn_request() {
                        tcp_conn_request() {
                            if (inet_csk_reqsk_queue_is_full(sk));
                                drop;
                            if (sk_acceptq_is_full(sk));
                                drop;

                            inet_reqsk_alloc() {
                                sk = kmem_cache_alloc(request_sock_ops->slab)
                                sk->sk_state = TCP_NEW_SYN_RECV;
                            }

                            dst = af_ops->route_req(sk, skb, &fl, req);
                            if (!dst)
                                drop;

                            af_ops->init_req(req, sk, skb) {
                                tcp_v4_init_req() {
                                    sk_rcv_saddr_set();
                                    sk_daddr_set();
                                }
                            }


                            if (fastopen) {
                                af_ops->send_synack();
                                inet_csk_reqsk_queue_add() {
                                    sk->rskq_accept_tail = reqst_sk;
                                    sk_acceptq_added() {
                                        icsk_accept_queue->rskq_accept_tail = req;
                                        sk_acceptq_added() {
                                            ++sk->sk_ack_backlog
                                        }
                                    }
                                }
                                sk->sk_data_ready(sk);
                            } else {
                                inet_csk_reqsk_queue_hash_add() {
                                    reqsk_queue_hash_req() {
                                        timer_setup();
                                        mod_timer(&req->rsk_timer, jiffies + timeout);
                                        inet_ehash_insert() { /* add to established hash table */
                                            inet_ehash_bucket(hashinfo, sk->sk_hash);
                                            __sk_nulls_add_node_rcu() {
                                                hlist_nulls_add_head_rcu(&sk->sk_nulls_node, list);
                                            }
                                            if (osk);
                                                sk_nulls_del_node_init_rcu(osk);
                                        }
                                    }
                                    inet_csk_reqsk_queue_added() {
                                        ++icsk_accept_queue->young;
                                        --icsk_accept_queue->qlen;
                                    }
                                }

                                af_ops->send_synack() {
                                    tcp_v4_send_synack();
                                }
                            }
                        }
                    }
                }
            }

            if (sk->sk_state == TCP_SYN_SENT) {
                tcp_rcv_synsent_state_process() {
                    tcp_ack(sk, skb, FLAG_SLOWPATH);
                    tcp_finish_connect(sk, skb) {
                            tcp_set_state(sk, TCP_ESTABLISHED);
                            tcp_init_transfer() {
                                tcp_call_bpf(sk, bpf_op, 0, NULL);
                                tcp_init_congestion_control(sk);
                                tcp_init_buffer_space(sk);
                            }
                            inet_csk_reset_keepalive_timer();

                            sk->sk_state_change() {
                                /* wakup `connect` slept at inet_wait_for_connect */
                                sock_def_wakeup();
                            }
                    tcp_send_ack(sk);
                    }
                }

                tcp_data_snd_check(sk);
                return 0;
            }

            tcp_check_req(); /* tp->fastopen_rsk */

            /* Step 1 2 3 4 */
            tcp_validate_incoming() {
                /* RFC1323: H1. Apply PAWS check first. */
                /* Step 1: check sequence number */
                /* Step 2: check RST bit */
                /* Step 3: check security and precedence [ignored] */
                /* Step 4: Check for a SYN RFC 5961 4.2 : Send a challenge ack */
            }

            /* step 5: check the ACK field */
            tcp_ack();

            if (sk->sk_state == TCP_SYN_RECV) {
                tcp_init_transfer();
                    tcp_init_buffer_space();
                    tcp_init_congestion_control(sk);
                tcp_set_state(sk, TCP_ESTABLISHED);

                sk->sk_state_change() {
                    sock_def_wakeup() {
                        wake_up_interruptible_all() {
                            __wake_up() {
                                __wake_up_common_lock() {
                                    __wake_up_common();
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
```

## send
```c
SYSCALL_DEFINE3(connect, int, fd, struct sockaddr __user *, uservaddr, int, addrlen)
{
  struct socket *sock;
  struct sockaddr_storage address;
  int err, fput_needed;
  sock = sockfd_lookup_light(fd, &err, &fput_needed);
  err = move_addr_to_kernel(uservaddr, addrlen, &address);
  err = sock->ops->connect(sock, (struct sockaddr *)&address, addrlen, sock->file->f_flags);
}

const struct proto_ops inet_stream_ops = {
  .connect = inet_stream_connect,
}

int __inet_stream_connect(
  struct socket *sock, struct sockaddr *uaddr,
  int addr_len, int flags, int is_sendmsg)
{
  struct sock *sk = sock->sk;
  int err;
  long timeo;

  switch (sock->state) {
    case SS_UNCONNECTED:
      err = -EISCONN;
      if (sk->sk_state != TCP_CLOSE)
        goto out;

      err = sk->sk_prot->connect(sk, uaddr, addr_len);
      sock->state = SS_CONNECTING;
      break;
  }

  timeo = sock_sndtimeo(sk, flags & O_NONBLOCK);
  if ((1 << sk->sk_state) & (TCPF_SYN_SENT | TCPF_SYN_RECV)) {
    /* woken up by sk->sk_state_change(sk) at tcp_rcv_synsent_state_process */
    if (!timeo || !inet_wait_for_connect(sk, timeo, writebias))
      goto out;

    err = sock_intr_errno(timeo);
    if (signal_pending(current))
      goto out;
  }
  sock->state = SS_CONNECTED;
}

struct proto tcp_prot = {
  .connect = tcp_v4_connect
}

/* initiate an outgoing connection */
int tcp_v4_connect(struct sock *sk, struct sockaddr *uaddr, int addr_len)
{
  struct sockaddr_in *usin = (struct sockaddr_in *)uaddr;
  struct inet_sock *inet = inet_sk(sk);
  struct tcp_sock *tp = tcp_sk(sk);
  __be16 orig_sport, orig_dport;
  __be32 daddr, nexthop;
  struct flowi4 *fl4;
  struct rtable *rt;

  orig_sport = inet->inet_sport;
  orig_dport = usin->sin_port;
  rt = ip_route_connect(fl4, nexthop, inet->inet_saddr,
            RT_CONN_FLAGS(sk), sk->sk_bound_dev_if,
            IPPROTO_TCP,
            orig_sport, orig_dport, sk);

  tcp_set_state(sk, TCP_SYN_SENT);

  /* Bind a port for a connect operation and hash it. */
  err = inet_hash_connect(tcp_death_row, sk);
  sk_set_txhash(sk);
  rt = ip_route_newports(fl4, rt, orig_sport, orig_dport,
             inet->inet_sport, inet->inet_dport, sk);
  /* OK, now commit destination to socket.  */
  sk->sk_gso_type = SKB_GSO_TCPV4;
  sk_setup_caps(sk, &rt->dst);
  if (likely(!tp->repair)) {
    if (!tp->write_seq)
      tp->write_seq = secure_tcp_seq(
        inet->inet_saddr, inet->inet_daddr,
        inet->inet_sport, usin->sin_port);

    tp->tsoffset = secure_tcp_ts_off(
      sock_net(sk), inet->inet_saddr, inet->inet_daddr);
  }
  rt = NULL;

  err = tcp_connect(sk);
}

/* Build a SYN and send it off. */
int tcp_connect(struct sock *sk)
{
  struct tcp_sock *tp = tcp_sk(sk);
  struct sk_buff *buff;
  int err;

  tcp_call_bpf(sk, BPF_SOCK_OPS_TCP_CONNECT_CB, 0, NULL);

  /* Do all connect socket setups that can be done AF independent. */
  tcp_connect_init(sk);

  buff = sk_stream_alloc_skb(sk, 0, sk->sk_allocation, true);

  tcp_init_nondata_skb(buff, tp->write_seq++, TCPHDR_SYN);
  tcp_mstamp_refresh(tp);
  tp->retrans_stamp = tcp_time_stamp(tp);
  tcp_connect_queue_skb(sk, buff);
  tcp_ecn_send_syn(sk, buff);
  /* Insert skb into rb tree, ordered by TCP_SKB_CB(skb)->seq */
  tcp_rbtree_insert(&sk->tcp_rtx_queue, buff);

  /* Send off SYN; include data in Fast Open. */
  err = tp->fastopen_req
    ? tcp_send_syn_data(sk, buff)
    : tcp_transmit_skb(sk, buff, 1, sk->sk_allocation);

  tp->snd_nxt = tp->write_seq;
  tp->pushed_seq = tp->write_seq;
  buff = tcp_send_head(sk);
  if (unlikely(buff)) {
    tp->snd_nxt  = TCP_SKB_CB(buff)->seq;
    tp->pushed_seq  = TCP_SKB_CB(buff)->seq;
  }

  /* Timer for repeating the SYN until an answer. */
  inet_csk_reset_xmit_timer(sk, ICSK_TIME_RETRANS, inet_csk(sk)->icsk_rto, TCP_RTO_MAX);
  return 0;
}

/* Do all connect socket setups that can be done AF independent. */
void tcp_connect_init(struct sock *sk)
{
  const struct dst_entry *dst = __sk_dst_get(sk);
  struct tcp_sock *tp = tcp_sk(sk);
  __u8 rcv_wscale;
  u32 rcv_wnd;

  tp->tcp_header_len = sizeof(struct tcphdr);
  if (sock_net(sk)->ipv4.sysctl_tcp_timestamps)
    tp->tcp_header_len += TCPOLEN_TSTAMP_ALIGNED;

  /* If user gave his TCP_MAXSEG, record it to clamp */
  if (tp->rx_opt.user_mss)
    tp->rx_opt.mss_clamp = tp->rx_opt.user_mss;
  tp->max_window = 0;
  tcp_mtup_init(sk);
  tcp_sync_mss(sk, dst_mtu(dst));

  tcp_ca_dst_init(sk, dst);

  if (!tp->window_clamp)
    tp->window_clamp = dst_metric(dst, RTAX_WINDOW);
  tp->advmss = tcp_mss_clamp(tp, dst_metric_advmss(dst));

  tcp_initialize_rcv_mss(sk);

  /* limit the window selection if the user enforce a smaller rx buffer */
  if (sk->sk_userlocks & SOCK_RCVBUF_LOCK &&
      (tp->window_clamp > tcp_full_space(sk) || tp->window_clamp == 0))
    tp->window_clamp = tcp_full_space(sk);

  rcv_wnd = tcp_rwnd_init_bpf(sk);
  if (rcv_wnd == 0)
    rcv_wnd = dst_metric(dst, RTAX_INITRWND);

  tcp_select_initial_window(sk, tcp_full_space(sk),
          tp->advmss - (tp->rx_opt.ts_recent_stamp ? tp->tcp_header_len - sizeof(struct tcphdr) : 0),
          &tp->rcv_wnd,
          &tp->window_clamp,
          sock_net(sk)->ipv4.sysctl_tcp_window_scaling,
          &rcv_wscale,
          rcv_wnd);

  tp->rx_opt.rcv_wscale = rcv_wscale;
  tp->rcv_ssthresh = tp->rcv_wnd;

  sk->sk_err = 0;
  sock_reset_flag(sk, SOCK_DONE);
  tp->snd_wnd = 0;
  tcp_init_wl(tp, 0);
  tcp_write_queue_purge(sk);
  tp->snd_una = tp->write_seq;
  tp->snd_sml = tp->write_seq;
  tp->snd_up = tp->write_seq;
  tp->snd_nxt = tp->write_seq;

  if (likely(!tp->repair))
    tp->rcv_nxt = 0;
  else
    tp->rcv_tstamp = tcp_jiffies32;
  tp->rcv_wup = tp->rcv_nxt;
  tp->copied_seq = tp->rcv_nxt;

  inet_csk(sk)->icsk_rto = tcp_timeout_init(sk);
  inet_csk(sk)->icsk_retransmits = 0;
  tcp_clear_retrans(tp);
}

void tcp_connect_queue_skb(struct sock *sk, struct sk_buff *skb)
{
  struct tcp_sock *tp = tcp_sk(sk);
  struct tcp_skb_cb *tcb = TCP_SKB_CB(skb);

  tcb->end_seq += skb->len;
  __skb_header_release(skb);
  sk->sk_wmem_queued += skb->truesize;
  sk_mem_charge(sk, skb->truesize);
  WRITE_ONCE(tp->write_seq, tcb->end_seq);
  tp->packets_out += tcp_skb_pcount(skb);
}

long inet_wait_for_connect(struct sock *sk, long timeo, int writebias)
{
  DEFINE_WAIT_FUNC(wait, woken_wake_function);

  add_wait_queue(sk_sleep(sk), &wait);
  sk->sk_write_pending += writebias;

  while ((1 << sk->sk_state) & (TCPF_SYN_SENT | TCPF_SYN_RECV)) {
    timeo = wait_woken(&wait, TASK_INTERRUPTIBLE, timeo);
    if (signal_pending(current) || !timeo)
      break;
  }
  remove_wait_queue(sk_sleep(sk), &wait);
  sk->sk_write_pending -= writebias;
  return timeo;
}
```
* [wait_woken](./linux-proc.md#wait_woken)

## receive

```c
struct net_protocol tcp_protocol = {
  .early_demux          =  tcp_v4_early_demux,
  .early_demux_handler  =  tcp_v4_early_demux,
  .handler              =  tcp_v4_rcv,
  .err_handler          =  tcp_v4_err,
  .no_policy            =  1,
  .netns_ok             =  1,
  .icmp_strict_tag_validation = 1,
}

/* tcp_v4_rcv -> tcp_v4_do_rcv -> tcp_rcv_state_process
 * see process of `TCP_NEW_SYN_RECV` state in tcp_v4_rcv */
int tcp_rcv_state_process(struct sock *sk, struct sk_buff *skb)
{
  struct tcp_sock *tp = tcp_sk(sk);
  struct inet_connection_sock *icsk = inet_csk(sk);
  const struct tcphdr *th = tcp_hdr(skb);
  struct request_sock *req;
  int queued = 0;
  bool acceptable;

  switch (sk->sk_state) {
  case TCP_CLOSE:
      goto discard;

  case TCP_LISTEN:
      if (th->ack)
        return 1;

      if (th->rst)
        goto discard;

      if (th->syn) {
        if (th->fin)
          goto discard;
        /* It is possible that we process SYN packets from backlog,
         * so we need to make sure to disable BH and RCU right there. */
        rcu_read_lock();
        local_bh_disable();
        acceptable = icsk->icsk_af_ops->conn_request(sk, skb) >= 0;
        local_bh_enable();
        rcu_read_unlock();

        if (!acceptable)
          return 1;
        consume_skb(skb);
        return 0;
      }
      goto discard;

  case TCP_SYN_SENT:
      tp->rx_opt.saw_tstamp = 0;
      tcp_mstamp_refresh(tp);
      queued = tcp_rcv_synsent_state_process(sk, skb, th);
      if (queued >= 0)
        return queued;

      /* Do step6 onward by hand. */
      tcp_urg(sk, skb, th);
      __kfree_skb(skb);
      tcp_data_snd_check(sk);
      return 0;
  }

  tcp_mstamp_refresh(tp);
  tp->rx_opt.saw_tstamp = 0;
  req = tp->fastopen_rsk;
  if (req) {
    bool req_stolen;
    if (!tcp_check_req(sk, skb, req, true, &req_stolen))
      goto discard;
  }

  if (!th->ack && !th->rst && !th->syn)
    goto discard;

  /* Step 1 2 3 4 */
  if (!tcp_validate_incoming(sk, skb, th, 0))
    return 0;

  /* Step 5: check the ACK field */
  acceptable = tcp_ack(sk, skb, FLAG_SLOWPATH | FLAG_UPDATE_TS_RECENT | FLAG_NO_CHALLENGE_ACK) > 0;

  if (!acceptable) {
    if (sk->sk_state == TCP_SYN_RECV)
      return 1;  /* send one RST */
    tcp_send_challenge_ack(sk, skb);
    goto discard;
  }
  switch (sk->sk_state) {
  case TCP_SYN_RECV:
      tp->delivered++; /* SYN-ACK delivery isn't tracked in tcp_ack */
      if (!tp->srtt_us)
        tcp_synack_rtt_meas(sk, req);

      /* Once we leave TCP_SYN_RECV, we no longer need req
       * so release it. */
      if (req) {
        inet_csk(sk)->icsk_retransmits = 0;
        reqsk_fastopen_remove(sk, req, false);
        /* Re-arm the timer because data may have been sent out.
        * This is similar to the regular data transmission case
        * when new data has just been ack'ed.
        *
        * (TFO) - we could try to be more aggressive and
        * retransmitting any data sooner based on when they
        * are sent out. */
        tcp_rearm_rto(sk);
      } else {
        tcp_init_transfer(sk, BPF_SOCK_OPS_PASSIVE_ESTABLISHED_CB);
        tp->copied_seq = tp->rcv_nxt;
      }
      tcp_set_state(sk, TCP_ESTABLISHED);
      sk->sk_state_change(sk);

      /* Note, that this wakeup is only for marginal crossed SYN case.
      * Passively open sockets are not waked up, because
      * sk->sk_sleep == NULL and sk->sk_socket == NULL. */
      if (sk->sk_socket)
        sk_wake_async(sk, SOCK_WAKE_IO, POLL_OUT);

      tp->snd_una = TCP_SKB_CB(skb)->ack_seq;
      tp->snd_wnd = ntohs(th->window) << tp->rx_opt.snd_wscale;
      tcp_init_wl(tp, TCP_SKB_CB(skb)->seq);

      if (tp->rx_opt.tstamp_ok)
        tp->advmss -= TCPOLEN_TSTAMP_ALIGNED;

      if (!inet_csk(sk)->icsk_ca_ops->cong_control)
        tcp_update_pacing_rate(sk);

      /* Prevent spurious tcp_cwnd_restart() on first data packet */
      tp->lsndtime = tcp_jiffies32;

      tcp_initialize_rcv_mss(sk);
      tcp_fast_path_on(tp);
      break;

  case TCP_FIN_WAIT1: {
      int tmo;

      /* If we enter the TCP_FIN_WAIT1 state and we are a
      * Fast Open socket and this is the first acceptable
      * ACK we have received, this would have acknowledged
      * our SYNACK so stop the SYNACK timer. */
      if (req) {
        /* We no longer need the request sock. */
        reqsk_fastopen_remove(sk, req, false);
        tcp_rearm_rto(sk);
      }
      if (tp->snd_una != tp->write_seq)
        break;

      tcp_set_state(sk, TCP_FIN_WAIT2);
      sk->sk_shutdown |= SEND_SHUTDOWN;

      sk_dst_confirm(sk);

      if (!sock_flag(sk, SOCK_DEAD)) {
        /* Wake up lingering close() */
        sk->sk_state_change(sk);
        break;
      }

      if (tp->linger2 < 0) {
        tcp_done(sk);
        return 1;
      }
      if (TCP_SKB_CB(skb)->end_seq != TCP_SKB_CB(skb)->seq
        && after(TCP_SKB_CB(skb)->end_seq - th->fin, tp->rcv_nxt))
      {
        /* Receive out of order FIN after close() */
        if (tp->syn_fastopen && th->fin)
          tcp_fastopen_active_disable(sk);
        tcp_done(sk);
        return 1;
      }

      tmo = tcp_fin_time(sk);
      if (tmo > TCP_TIMEWAIT_LEN) {
        inet_csk_reset_keepalive_timer(sk, tmo - TCP_TIMEWAIT_LEN);
      } else if (th->fin || sock_owned_by_user(sk)) {
        /* Bad case. We could lose such FIN otherwise.
        * It is not a big problem, but it looks confusing
        * and not so rare event. We still can lose it now,
        * if it spins in bh_lock_sock(), but it is really
        * marginal case. */
        inet_csk_reset_keepalive_timer(sk, tmo);
      } else {
        tcp_time_wait(sk, TCP_FIN_WAIT2, tmo);
        goto discard;
      }
      break;
  }

  case TCP_CLOSING:
      if (tp->snd_una == tp->write_seq) {
        tcp_time_wait(sk, TCP_TIME_WAIT, 0);
        goto discard;
      }
      break;

  case TCP_LAST_ACK:
      if (tp->snd_una == tp->write_seq) {
        tcp_update_metrics(sk);
        tcp_done(sk);
        goto discard;
      }
      break;
  }

  /* Step 6: check the URG bit */
  tcp_urg(sk, skb, th);

  /* Step 7: process the segment text */
  switch (sk->sk_state) {
  case TCP_CLOSE_WAIT:
  case TCP_CLOSING:
  case TCP_LAST_ACK:
      if (!before(TCP_SKB_CB(skb)->seq, tp->rcv_nxt))
        break;
      /* fall through */
  case TCP_FIN_WAIT1:
  case TCP_FIN_WAIT2:
      /* RFC 793 says to queue data in these states,
      * RFC 1122 says we MUST send a reset.
      * BSD 4.4 also does reset. */
      if (sk->sk_shutdown & RCV_SHUTDOWN) {
        if (TCP_SKB_CB(skb)->end_seq != TCP_SKB_CB(skb)->seq && after(TCP_SKB_CB(skb)->end_seq - th->fin, tp->rcv_nxt)) {
          tcp_reset(sk);
          return 1;
        }
      }
      /* Fall through */
  case TCP_ESTABLISHED:
      tcp_data_queue(sk, skb);
      queued = 1;
      break;
  }

  /* tcp_data could move socket to TIME-WAIT */
  if (sk->sk_state != TCP_CLOSE) {
    tcp_data_snd_check(sk);
    tcp_ack_snd_check(sk);
  }

  if (!queued) {
discard:
    tcp_drop(sk, skb);
  }
  return 0;
}
```

### tcp_conn_request

* [tcp/dccp: lockless listener](https://lwn.net/Articles/659199/)

```c
const struct inet_connection_sock_af_ops ipv4_specific = {
  .queue_xmit        = ip_queue_xmit,
  .send_check        = tcp_v4_send_check,
  .rebuild_header    = inet_sk_rebuild_header,
  .sk_rx_dst_set     = inet_sk_rx_dst_set,
  .conn_request      = tcp_v4_conn_request,
  .syn_recv_sock     = tcp_v4_syn_recv_sock,
  .net_header_len    = sizeof(struct iphdr),
  .setsockopt        = ip_setsockopt,
  .getsockopt        = ip_getsockopt,
  .addr2sockaddr     = inet_csk_addr2sockaddr,
  .sockaddr_len      = sizeof(struct sockaddr_in),
  .mtu_reduced       = tcp_v4_mtu_reduced,
};

struct request_sock_ops tcp_request_sock_ops = {
  .family           =  PF_INET,
  .obj_size         =  sizeof(struct tcp_request_sock),
  .rtx_syn_ack      =  tcp_rtx_synack,
  .send_ack         =  tcp_v4_reqsk_send_ack,
  .destructor       =  tcp_v4_reqsk_destructor,
  .send_reset       =  tcp_v4_send_reset,
  .syn_ack_timeout  =  tcp_syn_ack_timeout,
};

const struct tcp_request_sock_ops tcp_request_sock_ipv4_ops = {
  .mss_clamp        =  TCP_MSS_DEFAULT,
  .req_md5_lookup   =  tcp_v4_md5_lookup,
  .calc_md5_hash    =  tcp_v4_md5_hash_skb,
  .cookie_init_seq  =  cookie_v4_init_sequence,
  .route_req        =  tcp_v4_route_req,
  .init_seq         =  tcp_v4_init_seq,
  .init_ts_off      =  tcp_v4_init_ts_off,
  .send_synack      =  tcp_v4_send_synack,
};

int tcp_v4_conn_request(struct sock *sk, struct sk_buff *skb)
{
  if (skb_rtable(skb)->rt_flags & (RTCF_BROADCAST | RTCF_MULTICAST))
    goto drop;

  return tcp_conn_request(&tcp_request_sock_ops,
        &tcp_request_sock_ipv4_ops, sk, skb);

drop:
  tcp_listendrop(sk);
  return 0;
}

int tcp_conn_request(
  struct request_sock_ops *rsk_ops,
  const struct tcp_request_sock_ops *af_ops,
  struct sock *sk, struct sk_buff *skb)
{
  struct tcp_fastopen_cookie foc = { .len = -1 };
  __u32 isn = TCP_SKB_CB(skb)->tcp_tw_isn;
  struct tcp_options_received tmp_opt;
  struct tcp_sock *tp = tcp_sk(sk);
  struct net *net = sock_net(sk);
  struct sock *fastopen_sk = NULL;
  struct request_sock *req;
  bool want_cookie = false;
  struct dst_entry *dst;
  struct flowi fl;

  /* return reqsk_queue_len(&inet_csk(sk)->icsk_accept_queue) >= sk->sk_max_ack_backlog;
   * icsk_accept_queue: FIFO of established children */
  if ((net->ipv4.sysctl_tcp_syncookies == 2 || inet_csk_reqsk_queue_is_full(sk)) && !isn) {
    want_cookie = tcp_syn_flood_action(sk, skb, rsk_ops->slab_name);
    if (!want_cookie)
      goto drop;
  }

  /* return sk->sk_ack_backlog > sk->sk_max_ack_backlog; */
  if (sk_acceptq_is_full(sk)) {
    goto drop;
  }

  req = inet_reqsk_alloc(rsk_ops, sk, !want_cookie);
  if (!req)
    goto drop;

  tcp_rsk(req)->af_specific = af_ops;
  tcp_rsk(req)->ts_off = 0;

  tcp_clear_options(&tmp_opt);
  tmp_opt.mss_clamp = af_ops->mss_clamp;
  tmp_opt.user_mss  = tp->rx_opt.user_mss;
  tcp_parse_options(sock_net(sk), skb, &tmp_opt, 0, want_cookie ? NULL : &foc);

  if (want_cookie && !tmp_opt.saw_tstamp)
    tcp_clear_options(&tmp_opt);

  if (IS_ENABLED(CONFIG_SMC) && want_cookie)
    tmp_opt.smc_ok = 0;

  tmp_opt.tstamp_ok = tmp_opt.saw_tstamp;
  tcp_openreq_init(req, &tmp_opt, skb, sk);
  inet_rsk(req)->no_srccheck = inet_sk(sk)->transparent;

  /* Note: tcp_v6_init_req() might override ir_iif for link locals */
  inet_rsk(req)->ir_iif = inet_request_bound_dev_if(sk, skb);

  /* tcp_v4_init_seq */
  af_ops->init_req(req, sk, skb);

  if (security_inet_conn_request(sk, skb, req))
    goto drop_and_free;

  if (tmp_opt.tstamp_ok)
    tcp_rsk(req)->ts_off = af_ops->init_ts_off(net, skb);

  dst = af_ops->route_req(sk, &fl, req);
  if (!dst)
    goto drop_and_free;

  if (!want_cookie && !isn) {
    /* Kill the following clause, if you dislike this way. */
    if (!net->ipv4.sysctl_tcp_syncookies
      && (net->ipv4.sysctl_max_syn_backlog - inet_csk_reqsk_queue_len(sk) < (net->ipv4.sysctl_max_syn_backlog >> 2))
      && !tcp_peer_is_proven(req, dst))
    {
      pr_drop_req(req, ntohs(tcp_hdr(skb)->source), rsk_ops->family);
      goto drop_and_release;
    }

    isn = af_ops->init_seq(skb);
  }

  tcp_ecn_create_request(req, skb, sk, dst);

  if (want_cookie) {
    isn = cookie_init_sequence(af_ops, sk, skb, &req->mss);
    req->cookie_ts = tmp_opt.tstamp_ok;
    if (!tmp_opt.tstamp_ok)
      inet_rsk(req)->ecn_ok = 0;
  }

  tcp_rsk(req)->snt_isn = isn;
  tcp_rsk(req)->txhash = net_tx_rndhash();
  tcp_openreq_init_rwin(req, sk, dst);
  sk_rx_queue_set(req_to_sk(req), skb);
  if (!want_cookie) {
    tcp_reqsk_record_syn(sk, req, skb);
    fastopen_sk = tcp_try_fastopen(sk, skb, req, &foc, dst);
  }
  if (fastopen_sk) {
    af_ops->send_synack(fastopen_sk, dst, &fl, req, &foc, TCP_SYNACK_FASTOPEN);
    /* Add the child socket directly into the accept queue */
    if (!inet_csk_reqsk_queue_add(sk, req, fastopen_sk)) {
      reqsk_fastopen_remove(fastopen_sk, req, false);
      bh_unlock_sock(fastopen_sk);
      sock_put(fastopen_sk);
      reqsk_put(req);
      goto drop;
    }
    sk->sk_data_ready(sk); /* sock_def_readable */
    bh_unlock_sock(fastopen_sk);
    sock_put(fastopen_sk);
  } else {
    tcp_rsk(req)->tfo_listener = false;
    if (!want_cookie)
      inet_csk_reqsk_queue_hash_add(sk, req, tcp_timeout_init((struct sock *)req));
    af_ops->send_synack(sk, dst, &fl, req, &foc, !want_cookie ? TCP_SYNACK_NORMAL : TCP_SYNACK_COOKIE);
    if (want_cookie) {
      reqsk_free(req);
      return 0;
    }
  }
  reqsk_put(req);
  return 0;

drop_and_release:
  dst_release(dst);
drop_and_free:
  reqsk_free(req);
drop:
  tcp_listendrop(sk);
  return 0;
}

request_sock *inet_reqsk_alloc(const struct request_sock_ops *ops,
  struct sock *sk_listener,
  bool attach_listener)
{
  struct request_sock *req = reqsk_alloc(ops, sk_listener, attach_listener);

  if (req) {
    struct inet_request_sock *ireq = inet_rsk(req);

    ireq->ireq_opt = NULL;
#if IS_ENABLED(CONFIG_IPV6)
    ireq->pktopts = NULL;
#endif
    atomic64_set(&ireq->ir_cookie, 0);
    /* https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/commit/?id=10feb428a504
     * #define ireq_state   req.__req_common.skc_state
     * #define sk_state     __sk_common.skc_state */
    ireq->ireq_state = TCP_NEW_SYN_RECV;
    write_pnet(&ireq->ireq_net, sock_net(sk_listener));
    ireq->ireq_family = sk_listener->sk_family;
  }

  return req;
}

struct request_sock *reqsk_alloc(const struct request_sock_ops *ops, struct sock *sk_listener,
      bool attach_listener)
{
  struct request_sock *req;

  req = kmem_cache_alloc(ops->slab, GFP_ATOMIC | __GFP_NOWARN);
  if (!req)
    return NULL;
  req->rsk_listener = NULL;
  if (attach_listener) {
    if (unlikely(!refcount_inc_not_zero(&sk_listener->sk_refcnt))) {
      kmem_cache_free(ops->slab, req);
      return NULL;
    }
    req->rsk_listener = sk_listener;
  }
  req->rsk_ops = ops;
  req_to_sk(req)->sk_prot = sk_listener->sk_prot;
  sk_node_init(&req_to_sk(req)->sk_node);
  sk_tx_queue_clear(req_to_sk(req));
  req->saved_syn = NULL;
  refcount_set(&req->rsk_refcnt, 0);

  return req;
}

/* send_synack -> */
int tcp_v4_send_synack(
  const struct sock *sk, struct dst_entry *dst,
  struct flowi *fl,
  struct request_sock *req,
  struct tcp_fastopen_cookie *foc,
  enum tcp_synack_type synack_type)
{

}

/* wakeup `accept` slept at inet_csk_wait_for_connect */
void sock_def_wakeup(struct sock *sk)
{
  struct socket_wq *wq;

  wq = rcu_dereference(sk->sk_wq);
  if (skwq_has_sleeper(wq))
    wake_up_interruptible_all(&wq->wait);
}

#define wake_up_interruptible_all(x) __wake_up(x, TASK_INTERRUPTIBLE, 0, NULL)

void tcp_v4_init_req(struct request_sock *req,
  const struct sock *sk_listener,
  struct sk_buff *skb)
{
  struct inet_request_sock *ireq = inet_rsk(req);
  struct net *net = sock_net(sk_listener);

  sk_rcv_saddr_set(req_to_sk(req), ip_hdr(skb)->daddr);
  sk_daddr_set(req_to_sk(req), ip_hdr(skb)->saddr);
  RCU_INIT_POINTER(ireq->ireq_opt, tcp_v4_save_options(net, skb));
}
```
* [wake_up](./linux-proc.md#wake_up)

### tcp_rcv_synsent_state_process
```c
int tcp_rcv_synsent_state_process(struct sock *sk, struct sk_buff *skb,
  const struct tcphdr *th)
{
  struct inet_connection_sock *icsk = inet_csk(sk);
  struct tcp_sock *tp = tcp_sk(sk);
  struct tcp_fastopen_cookie foc = { .len = -1 };
  int saved_clamp = tp->rx_opt.mss_clamp;
  bool fastopen_fail;

  tcp_parse_options(sock_net(sk), skb, &tp->rx_opt, 0, &foc);
  if (tp->rx_opt.saw_tstamp && tp->rx_opt.rcv_tsecr)
    tp->rx_opt.rcv_tsecr -= tp->tsoffset;

  if (th->ack) {
    /* rfc793:
     * "If the state is SYN-SENT then
     *    first check the ACK bit
     *      If the ACK bit is set
     *    If SEG.ACK =< ISS, or SEG.ACK > SND.NXT, send
     *        a reset (unless the RST bit is set, if so drop
     *        the segment and return)" */
    if (!after(TCP_SKB_CB(skb)->ack_seq, tp->snd_una) || after(TCP_SKB_CB(skb)->ack_seq, tp->snd_nxt))
      goto reset_and_undo;

    if (tp->rx_opt.saw_tstamp && tp->rx_opt.rcv_tsecr &&
        !between(tp->rx_opt.rcv_tsecr, tp->retrans_stamp, tcp_time_stamp(tp)))
    {
      goto reset_and_undo;
    }

    /* Now ACK is acceptable.
     * "If the RST bit is set
     *    If the ACK was acceptable then signal the user "error:
     *    connection reset", drop the segment, enter CLOSED state,
     *    delete TCB, and return." */
    if (th->rst) {
      tcp_reset(sk);
      goto discard;
    }

    if (!th->syn)
      goto discard_and_undo;

    /* rfc793:
     *   "If the SYN bit is on ...
     *    are acceptable then ...
     *    (our SYN has been ACKed), change the connection
     *    state to ESTABLISHED..." */

    tcp_ecn_rcv_synack(tp, th);

    tcp_init_wl(tp, TCP_SKB_CB(skb)->seq);
    tcp_ack(sk, skb, FLAG_SLOWPATH);

    /* Ok.. it's good. Set up sequence numbers and
     * move to established. */
    WRITE_ONCE(tp->rcv_nxt, TCP_SKB_CB(skb)->seq + 1);
    tp->rcv_wup = TCP_SKB_CB(skb)->seq + 1;

    /* RFC1323: The window in SYN & SYN/ACK segments is
     * never scaled. */
    tp->snd_wnd = ntohs(th->window);

    if (!tp->rx_opt.wscale_ok) {
      tp->rx_opt.snd_wscale = tp->rx_opt.rcv_wscale = 0;
      tp->window_clamp = min(tp->window_clamp, 65535U);
    }

    if (tp->rx_opt.saw_tstamp) {
      tp->rx_opt.tstamp_ok     = 1;
      tp->tcp_header_len = sizeof(struct tcphdr) + TCPOLEN_TSTAMP_ALIGNED;
      tp->advmss      -= TCPOLEN_TSTAMP_ALIGNED;
      tcp_store_ts_recent(tp);
    } else {
      tp->tcp_header_len = sizeof(struct tcphdr);
    }

    tcp_sync_mss(sk, icsk->icsk_pmtu_cookie);
    tcp_initialize_rcv_mss(sk);

    /* Remember, tcp_poll() does not lock socket!
     * Change state from SYN-SENT only after copied_seq
     * is initialized. */
    tp->copied_seq = tp->rcv_nxt;

    smc_check_reset_syn(tp);

    tcp_finish_connect(sk, skb);

    fastopen_fail = (tp->syn_fastopen || tp->syn_data) && tcp_rcv_fastopen_synack(sk, skb, &foc);

    if (!sock_flag(sk, SOCK_DEAD)) {
      /* wakup `connect` slept at inet_wait_for_connect */
      sk->sk_state_change(sk);
      sk_wake_async(sk, SOCK_WAKE_IO, POLL_OUT);
    }

    if (fastopen_fail)
      return -1;
    if (sk->sk_write_pending ||
        icsk->icsk_accept_queue.rskq_defer_accept ||
        icsk->icsk_ack.pingpong)
    {
      /* Save one ACK. Data will be ready after
       * several ticks, if write_pending is set.
       *
       * It may be deleted, but with this feature tcpdumps
       * look so _wonderfully_ clever, that I was not able
       * to stand against the temptation 8)     --ANK */
      inet_csk_schedule_ack(sk);
      tcp_enter_quickack_mode(sk, TCP_MAX_QUICKACKS);
      inet_csk_reset_xmit_timer(sk, ICSK_TIME_DACK, TCP_DELACK_MAX, TCP_RTO_MAX);

discard:
      tcp_drop(sk, skb);
      return 0;
    } else {
      tcp_send_ack(sk);
    }
    return -1;
  }

  /* No ACK in the segment */

  if (th->rst) {
    /* rfc793:
     * "If the RST bit is set
     *
     *      Otherwise (no ACK) drop the segment and return." */
    goto discard_and_undo;
  }

  /* PAWS check. */
  if (tp->rx_opt.ts_recent_stamp && tp->rx_opt.saw_tstamp &&
      tcp_paws_reject(&tp->rx_opt, 0))
    goto discard_and_undo;

  if (th->syn) {
    /* We see SYN without ACK. It is attempt of
     * simultaneous connect with crossed SYNs.
     * Particularly, it can be connect to self. */
    tcp_set_state(sk, TCP_SYN_RECV);

    if (tp->rx_opt.saw_tstamp) {
      tp->rx_opt.tstamp_ok = 1;
      tcp_store_ts_recent(tp);
      tp->tcp_header_len = sizeof(struct tcphdr) + TCPOLEN_TSTAMP_ALIGNED;
    } else {
      tp->tcp_header_len = sizeof(struct tcphdr);
    }

    WRITE_ONCE(tp->rcv_nxt, TCP_SKB_CB(skb)->seq + 1);
    tp->copied_seq = tp->rcv_nxt;
    tp->rcv_wup = TCP_SKB_CB(skb)->seq + 1;

    /* RFC1323: The window in SYN & SYN/ACK segments is
     * never scaled. */
    tp->snd_wnd    = ntohs(th->window);
    tp->snd_wl1    = TCP_SKB_CB(skb)->seq;
    tp->max_window = tp->snd_wnd;

    tcp_ecn_rcv_syn(tp, th);

    tcp_mtup_init(sk);
    tcp_sync_mss(sk, icsk->icsk_pmtu_cookie);
    tcp_initialize_rcv_mss(sk);

    tcp_send_synack(sk);
#if 0
    /* Note, we could accept data and URG from this segment.
     * There are no obstacles to make this (except that we must
     * either change tcp_recvmsg() to prevent it from returning data
     * before 3WHS completes per RFC793, or employ TCP Fast Open).
     *
     * However, if we ignore data in ACKless segments sometimes,
     * we have no reasons to accept it sometimes.
     * Also, seems the code doing it in step6 of tcp_rcv_state_process
     * is not flawless. So, discard packet for sanity.
     * Uncomment this return to process the data. */
    return -1;
#else
    goto discard;
#endif
  }
  /* "fifth, if neither of the SYN or RST bits is set then
   * drop the segment and return." */

discard_and_undo:
  tcp_clear_options(&tp->rx_opt);
  tp->rx_opt.mss_clamp = saved_clamp;
  goto discard;

reset_and_undo:
  tcp_clear_options(&tp->rx_opt);
  tp->rx_opt.mss_clamp = saved_clamp;
  return 1;
}

void tcp_finish_connect(struct sock *sk, struct sk_buff *skb)
{
  struct tcp_sock *tp = tcp_sk(sk);
  struct inet_connection_sock *icsk = inet_csk(sk);

  tcp_set_state(sk, TCP_ESTABLISHED);
  icsk->icsk_ack.lrcvtime = tcp_jiffies32;

  if (skb) {
    icsk->icsk_af_ops->sk_rx_dst_set(sk, skb);
    security_inet_conn_established(sk, skb);
    sk_mark_napi_id(sk, skb);
  }

  tcp_init_transfer(sk, BPF_SOCK_OPS_ACTIVE_ESTABLISHED_CB);

  /* Prevent spurious tcp_cwnd_restart() on first data
   * packet. */
  tp->lsndtime = tcp_jiffies32;

  if (sock_flag(sk, SOCK_KEEPOPEN))
    inet_csk_reset_keepalive_timer(sk, keepalive_time_when(tp));

  if (!tp->rx_opt.snd_wscale)
    __tcp_fast_path_on(tp, tp->snd_wnd);
  else
    tp->pred_flags = 0;
}
```

### tcp_check_req

```c
/* Process an incoming packet for SYN_RECV sockets represented as a
 * request_sock. Normally sk is the listener socket but for
 * TFO (TCP Fast Open) it points to the child socket. */
struct sock *tcp_check_req(struct sock *sk, struct sk_buff *skb,
         struct request_sock *req,
         bool fastopen, bool *req_stolen)
{
    struct tcp_options_received tmp_opt;
    struct sock *child;
    const struct tcphdr *th = tcp_hdr(skb);
    __be32 flg = tcp_flag_word(th) & (TCP_FLAG_RST|TCP_FLAG_SYN|TCP_FLAG_ACK);
    bool paws_reject = false;
    bool own_req;

/* 1. parse tcp options */
    tmp_opt.saw_tstamp = 0;
    if (th->doff > (sizeof(struct tcphdr)>>2)) {
        tcp_parse_options(sock_net(sk), skb, &tmp_opt, 0, NULL);

        if (tmp_opt.saw_tstamp) {
            tmp_opt.ts_recent = req->ts_recent;
            if (tmp_opt.rcv_tsecr)
                tmp_opt.rcv_tsecr -= tcp_rsk(req)->ts_off;
            /* We do not store true stamp, but it is not required,
            * it can be estimated (approximately)
            * from another data. */
            tmp_opt.ts_recent_stamp = ktime_get_seconds() - ((TCP_TIMEOUT_INIT/HZ)<<req->num_timeout);
            paws_reject = tcp_paws_reject(&tmp_opt, th->rst);
        }
    }

/* 2. Check for pure retransmitted SYN. */
    if (TCP_SKB_CB(skb)->seq == tcp_rsk(req)->rcv_isn && flg == TCP_FLAG_SYN && !paws_reject) {
        if (!tcp_oow_rate_limited(sock_net(sk), skb, LINUX_MIB_TCPACKSKIPPEDSYNRECV, &tcp_rsk(req)->last_oow_ack_time)
            && !inet_rtx_syn_ack(sk, req)) {
            unsigned long expires = jiffies;

            expires += min(TCP_TIMEOUT_INIT << req->num_timeout,
                    TCP_RTO_MAX);
            if (!fastopen)
                mod_timer_pending(&req->rsk_timer, expires);
            else
                req->rsk_timer.expires = expires;
        }
        return NULL;
    }

    if ((flg & TCP_FLAG_ACK) && !fastopen && (TCP_SKB_CB(skb)->ack_seq != tcp_rsk(req)->snt_isn + 1))
        return sk;

/* 3. Out of window check */
    ret = tcp_in_window(TCP_SKB_CB(skb)->seq, TCP_SKB_CB(skb)->end_seq,
        tcp_rsk(req)->rcv_nxt, tcp_rsk(req)->rcv_nxt + req->rsk_rcv_wnd);
    if (paws_reject || !ret) {
            /* Out of window: send ACK and drop. */
            ret = tcp_oow_rate_limited(sock_net(sk), skb, LINUX_MIB_TCPACKSKIPPEDSYNRECV, &tcp_rsk(req)->last_oow_ack_time);
            if (!(flg & TCP_FLAG_RST) && !ret) {
                req->rsk_ops->send_ack(sk, skb, req); /* tcp_v4_reqsk_send_ack */
            }
            return NULL;
    }

  /* In sequence, PAWS is OK. */

    if (tmp_opt.saw_tstamp && !after(TCP_SKB_CB(skb)->seq, tcp_rsk(req)->rcv_nxt))
        req->ts_recent = tmp_opt.rcv_tsval;

    if (TCP_SKB_CB(skb)->seq == tcp_rsk(req)->rcv_isn) {
        /* Truncate SYN, it is out of window starting at tcp_rsk(req)->rcv_isn + 1. */
        flg &= ~TCP_FLAG_SYN;
    }

    if (flg & (TCP_FLAG_RST|TCP_FLAG_SYN)) {
        goto embryonic_reset;
    }

    if (!(flg & TCP_FLAG_ACK))
        return NULL;

    if (fastopen)
        return sk;

    /* While TCP_DEFER_ACCEPT is active, drop bare ACK. */
    if (req->num_timeout < inet_csk(sk)->icsk_accept_queue.rskq_defer_accept &&
        TCP_SKB_CB(skb)->end_seq == tcp_rsk(req)->rcv_isn + 1)
    {
        inet_rsk(req)->acked = 1;
        return NULL;
    }

    /* OK, ACK is valid, create big socket and
    * feed this segment to it. It will repeat all
    * the tests. THIS SEGMENT MUST MOVE SOCKET TO
    * ESTABLISHED STATE. If it will be dropped after
    * socket is created, wait for troubles. */
    child = inet_csk(sk)->icsk_af_ops->syn_recv_sock(sk, skb, req, NULL, req, &own_req);
    if (!child)
        goto listen_overflow;

    sock_rps_save_rxhash(child, skb);
    tcp_synack_rtt_meas(child, req);
    *req_stolen = !own_req;

    return inet_csk_complete_hashdance(sk, child, req, own_req);

listen_overflow:
    if (!sock_net(sk)->ipv4.sysctl_tcp_abort_on_overflow) {
        inet_rsk(req)->acked = 1;
        return NULL;
    }

embryonic_reset:
    if (!(flg & TCP_FLAG_RST)) {
        req->rsk_ops->send_reset(sk, skb);
    } else if (fastopen) { /* received a valid RST pkt */
        reqsk_fastopen_remove(sk, req, true);
        tcp_reset(sk);
    }
    if (!fastopen) {
        inet_csk_reqsk_queue_drop(sk, req);
    }
    return NULL;
}

const struct inet_connection_sock_af_ops ipv4_specific = {
    .syn_recv_sock = tcp_v4_syn_recv_sock;
}

/* The three way handshake has completed - we got a valid synack -
 * now create the new socket. */
struct sock *tcp_v4_syn_recv_sock(const struct sock *sk, struct sk_buff *skb,
  struct request_sock *req,
  struct dst_entry *dst,
  struct request_sock *req_unhash,
  bool *own_req)
{
  struct inet_request_sock *ireq;
  struct inet_sock *newinet;
  struct tcp_sock *newtp;
  struct sock *newsk;
#ifdef CONFIG_TCP_MD5SIG
  struct tcp_md5sig_key *key;
#endif
  struct ip_options_rcu *inet_opt;

  if (sk_acceptq_is_full(sk))
    goto exit_overflow;

  newsk = tcp_create_openreq_child(sk, req, skb);
  if (!newsk)
    goto exit_nonewsk;

  newsk->sk_gso_type = SKB_GSO_TCPV4;
  inet_sk_rx_dst_set(newsk, skb);

  newtp          = tcp_sk(newsk);
  newinet          = inet_sk(newsk);
  ireq          = inet_rsk(req);
  sk_daddr_set(newsk, ireq->ir_rmt_addr);
  sk_rcv_saddr_set(newsk, ireq->ir_loc_addr);
  newsk->sk_bound_dev_if = ireq->ir_iif;
  newinet->inet_saddr   = ireq->ir_loc_addr;
  inet_opt        = rcu_dereference(ireq->ireq_opt);
  RCU_INIT_POINTER(newinet->inet_opt, inet_opt);
  newinet->mc_index     = inet_iif(skb);
  newinet->mc_ttl        = ip_hdr(skb)->ttl;
  newinet->rcv_tos      = ip_hdr(skb)->tos;
  inet_csk(newsk)->icsk_ext_hdr_len = 0;
  if (inet_opt)
    inet_csk(newsk)->icsk_ext_hdr_len = inet_opt->opt.optlen;
  newinet->inet_id = prandom_u32();

  if (!dst) {
    dst = inet_csk_route_child_sock(sk, newsk, req);
    if (!dst)
      goto put_and_exit;
  } else {
    /* syncookie case : see end of cookie_v4_check() */
  }
  sk_setup_caps(newsk, dst);

  tcp_ca_openreq_child(newsk, dst);

  tcp_sync_mss(newsk, dst_mtu(dst));
  newtp->advmss = tcp_mss_clamp(tcp_sk(sk), dst_metric_advmss(dst));

  tcp_initialize_rcv_mss(newsk);

#ifdef CONFIG_TCP_MD5SIG
  /* Copy over the MD5 key from the original socket */
  key = tcp_md5_do_lookup(sk, (union tcp_md5_addr *)&newinet->inet_daddr,
        AF_INET);
  if (key) {
    /* We're using one, so create a matching key
     * on the newsk structure. If we fail to get
     * memory, then we end up not copying the key
     * across. Shucks. */
    tcp_md5_do_add(newsk, (union tcp_md5_addr *)&newinet->inet_daddr,
             AF_INET, 32, key->key, key->keylen, GFP_ATOMIC);
    sk_nocaps_add(newsk, NETIF_F_GSO_MASK);
  }
#endif

  if (__inet_inherit_port(sk, newsk) < 0)
    goto put_and_exit;
  *own_req = inet_ehash_nolisten(newsk, req_to_sk(req_unhash));
  if (likely(*own_req)) {
    tcp_move_syn(newtp, req);
    ireq->ireq_opt = NULL;
  } else {
    newinet->inet_opt = NULL;
  }
  return newsk;

exit_overflow:

exit_nonewsk:
  dst_release(dst);
exit:
  tcp_listendrop(sk);
  return NULL;
put_and_exit:
  newinet->inet_opt = NULL;
  inet_csk_prepare_forced_close(newsk);
  tcp_done(newsk);
  goto exit;
}

struct sock *tcp_create_openreq_child(const struct sock *sk,
  struct request_sock *req,
  struct sk_buff *skb)
{
  struct sock *newsk = inet_csk_clone_lock(sk, req, GFP_ATOMIC);
  const struct inet_request_sock *ireq = inet_rsk(req);
  struct tcp_request_sock *treq = tcp_rsk(req);
  struct inet_connection_sock *newicsk;
  struct tcp_sock *oldtp, *newtp;
  u32 seq;

  if (!newsk)
    return NULL;

  newicsk = inet_csk(newsk);
  newtp = tcp_sk(newsk);
  oldtp = tcp_sk(sk);

  /* Now setup tcp_sock */
  newtp->pred_flags = 0;

  seq = treq->rcv_isn + 1;
  newtp->rcv_wup = seq;
  newtp->copied_seq = seq;
  WRITE_ONCE(newtp->rcv_nxt, seq);
  newtp->segs_in = 1;

  newtp->snd_sml = newtp->snd_una =
  newtp->snd_nxt = newtp->snd_up = treq->snt_isn + 1;

  INIT_LIST_HEAD(&newtp->tsq_node);
  INIT_LIST_HEAD(&newtp->tsorted_sent_queue);

  tcp_init_wl(newtp, treq->rcv_isn);

  newtp->srtt_us = 0;
  newtp->mdev_us = jiffies_to_usecs(TCP_TIMEOUT_INIT);
  minmax_reset(&newtp->rtt_min, tcp_jiffies32, ~0U);
  newicsk->icsk_rto = TCP_TIMEOUT_INIT;
  newicsk->icsk_ack.lrcvtime = tcp_jiffies32;

  newtp->packets_out = 0;
  newtp->retrans_out = 0;
  newtp->sacked_out = 0;
  newtp->snd_ssthresh = TCP_INFINITE_SSTHRESH;
  newtp->tlp_high_seq = 0;
  newtp->lsndtime = tcp_jiffies32;
  newsk->sk_txhash = treq->txhash;
  newtp->last_oow_ack_time = 0;
  newtp->total_retrans = req->num_retrans;

  /* So many TCP implementations out there (incorrectly) count the
   * initial SYN frame in their delayed-ACK and congestion control
   * algorithms that we must have the following bandaid to talk
   * efficiently to them.  -DaveM */
  newtp->snd_cwnd = TCP_INIT_CWND;
  newtp->snd_cwnd_cnt = 0;

  /* There's a bubble in the pipe until at least the first ACK. */
  newtp->app_limited = ~0U;

  tcp_init_xmit_timers(newsk);
  newtp->write_seq = newtp->pushed_seq = treq->snt_isn + 1;

  newtp->rx_opt.saw_tstamp = 0;

  newtp->rx_opt.dsack = 0;
  newtp->rx_opt.num_sacks = 0;

  newtp->urg_data = 0;

  if (sock_flag(newsk, SOCK_KEEPOPEN))
    inet_csk_reset_keepalive_timer(newsk, keepalive_time_when(newtp));

  newtp->rx_opt.tstamp_ok = ireq->tstamp_ok;
  newtp->rx_opt.sack_ok = ireq->sack_ok;
  newtp->window_clamp = req->rsk_window_clamp;
  newtp->rcv_ssthresh = req->rsk_rcv_wnd;
  newtp->rcv_wnd = req->rsk_rcv_wnd;
  newtp->rx_opt.wscale_ok = ireq->wscale_ok;
  if (newtp->rx_opt.wscale_ok) {
    newtp->rx_opt.snd_wscale = ireq->snd_wscale;
    newtp->rx_opt.rcv_wscale = ireq->rcv_wscale;
  } else {
    newtp->rx_opt.snd_wscale = newtp->rx_opt.rcv_wscale = 0;
    newtp->window_clamp = min(newtp->window_clamp, 65535U);
  }
  newtp->snd_wnd = ntohs(tcp_hdr(skb)->window) << newtp->rx_opt.snd_wscale;
  newtp->max_window = newtp->snd_wnd;

  if (newtp->rx_opt.tstamp_ok) {
    newtp->rx_opt.ts_recent = req->ts_recent;
    newtp->rx_opt.ts_recent_stamp = ktime_get_seconds();
    newtp->tcp_header_len = sizeof(struct tcphdr) + TCPOLEN_TSTAMP_ALIGNED;
  } else {
    newtp->rx_opt.ts_recent_stamp = 0;
    newtp->tcp_header_len = sizeof(struct tcphdr);
  }
  newtp->tsoffset = treq->ts_off;
#ifdef CONFIG_TCP_MD5SIG
  newtp->md5sig_info = NULL;  /*XXX*/
  if (newtp->af_specific->md5_lookup(sk, newsk))
    newtp->tcp_header_len += TCPOLEN_MD5SIG_ALIGNED;
#endif
  if (skb->len >= TCP_MSS_DEFAULT + newtp->tcp_header_len)
    newicsk->icsk_ack.last_seg_size = skb->len - newtp->tcp_header_len;
  newtp->rx_opt.mss_clamp = req->mss;
  tcp_ecn_openreq_child(newtp, req);
  newtp->fastopen_req = NULL;
  newtp->fastopen_rsk = NULL;
  newtp->syn_data_acked = 0;
  newtp->rack.mstamp = 0;
  newtp->rack.advanced = 0;
  newtp->rack.reo_wnd_steps = 1;
  newtp->rack.last_delivered = 0;
  newtp->rack.reo_wnd_persist = 0;
  newtp->rack.dsack_seen = 0;

  return newsk;
}

struct sock *inet_csk_clone_lock(const struct sock *sk,
         const struct request_sock *req,
         const gfp_t priority)
{
  struct sock *newsk = sk_clone_lock(sk, priority);

  if (newsk) {
    struct inet_connection_sock *newicsk = inet_csk(newsk);

    inet_sk_set_state(newsk, TCP_SYN_RECV);
    newicsk->icsk_bind_hash = NULL;

    inet_sk(newsk)->inet_dport = inet_rsk(req)->ir_rmt_port;
    inet_sk(newsk)->inet_num = inet_rsk(req)->ir_num;
    inet_sk(newsk)->inet_sport = htons(inet_rsk(req)->ir_num);

    /* listeners have SOCK_RCU_FREE, not the children */
    sock_reset_flag(newsk, SOCK_RCU_FREE);

    inet_sk(newsk)->mc_list = NULL;

    newsk->sk_mark = inet_rsk(req)->ir_mark;
    atomic64_set(&newsk->sk_cookie, atomic64_read(&inet_rsk(req)->ir_cookie));

    newicsk->icsk_retransmits = 0;
    newicsk->icsk_backoff    = 0;
    newicsk->icsk_probes_out  = 0;

    /* Deinitialize accept_queue to trap illegal accesses. */
    memset(&newicsk->icsk_accept_queue, 0, sizeof(newicsk->icsk_accept_queue));

    security_inet_csk_clone(newsk, req);
  }
  return newsk;
}

sock *inet_csk_complete_hashdance(struct sock *sk, struct sock *child,
  struct request_sock *req, bool own_req)
{
    if (own_req) {
        inet_csk_reqsk_queue_drop(sk, req);
        reqsk_queue_removed(&inet_csk(sk)->icsk_accept_queue, req);
        if (inet_csk_reqsk_queue_add(sk, req, child))
            return child;
    }
    /* Too bad, another child took ownership of the request, undo. */
    bh_unlock_sock(child);
    sock_put(child);
    return NULL;
}
```

# accept


```c
accpet() {
    __sys_accept4() {
        sockfd_lookup_light();
        newsock = sock_alloc();
        newfd = get_unused_fd_flags();
        newfile = sock_alloc_file(sock) {
            alloc_file_pseudo(SOCK_INODE(sock), sock_mnt, dname);
        }

        sock->ops->accept() { /* inet_stream_ops.accept */
            inet_accept() {
                sk1->sk_prot->accept() { /* tcp_prot.accept */
                    inet_csk_accept() {
                        reqsk_queue_empty() {
                            /* wakeup by sk_listener->sk_data_ready() sock_def_readable in TCP_NEW_SYN_RECV */
                            inet_csk_wait_for_connect() {
                                prepare_to_wait_exclusive() {
                                    wq_entry->flags |= WQ_FLAG_EXCLUSIVE;
                                    __add_wait_queue_entry_tail(wq_head, wq_entry);
                                }
                                schedule_timeout();
                                finish_wait() {
                                    list_del_init(&wq_entry->entry);
                                }
                            }
                        }

                        reqsk_queue_remove() {
                            WRITE_ONCE(queue->rskq_accept_head, req->dl_next);
                            sk_acceptq_removed() {
                                --sk->sk_ack_backlog;
                            }
                        }
                    }
                }

                sock_graft(sk2, newsock) {
                    newsock->sk = sk2;
                }
                newsock->state = SS_CONNECTED;
            }
        }
        move_addr_to_user();
        fd_install(newfd, newfile);
    }
}
```

```c
SYSCALL_DEFINE3(accept, int, fd, struct sockaddr __user *, upeer_sockaddr,
    int __user *, upeer_addrlen)
{
  return sys_accept4(fd, upeer_sockaddr, upeer_addrlen, 0);
}

SYSCALL_DEFINE4(accept4, int, fd, struct sockaddr __user *, upeer_sockaddr,
    int __user *, upeer_addrlen, int, flags)
{
  struct socket *sock, *newsock;
  struct file *newfile;
  int err, len, newfd, fput_needed;
  struct sockaddr_storage address;

  /* listen socket */
  sock = sockfd_lookup_light(fd, &err, &fput_needed);
  newsock = sock_alloc();
  newsock->type = sock->type;
  newsock->ops = sock->ops;
  newfd = get_unused_fd_flags(flags);
  newfile = sock_alloc_file(newsock, flags, sock->sk->sk_prot_creator->name);
  err = sock->ops->accept(sock, newsock, sock->file->f_flags, false);
  if (upeer_sockaddr) {
    newsock->ops->getname(newsock, (struct sockaddr *)&address, &len, 2);
    err = move_addr_to_user(&address, len, upeer_sockaddr, upeer_addrlen);
  }
  fd_install(newfd, newfile);

  return newfd;
}

/* inet_stream_ops.accept */
int inet_accept(struct socket *sock, struct socket *newsock, int flags, bool kern)
{
    struct sock *sk1 = sock->sk;
    int err = -EINVAL;
    struct sock *sk2 = sk1->sk_prot->accept(sk1, flags, &err, kern);
    sock_rps_record_flow(sk2);
    sock_graft(sk2, newsock);
    newsock->state = SS_CONNECTED;
}

/* tcp_prot.accept */
struct sock *inet_csk_accept(struct sock *sk, int flags, int *err, bool kern)
{
  struct inet_connection_sock *icsk = inet_csk(sk);
  struct request_sock_queue *queue = &icsk->icsk_accept_queue;
  struct request_sock *req;
  struct sock *newsk;
  int error;

  if (sk->sk_state != TCP_LISTEN)
    goto out_err;

  /* Find already established connection */
  if (reqsk_queue_empty(queue)) {
    long timeo = sock_rcvtimeo(sk, flags & O_NONBLOCK);
    error = inet_csk_wait_for_connect(sk, timeo);
  }
  req = reqsk_queue_remove(queue, sk);
  newsk = req->sk;

  return newsk;
}

void sock_graft(struct sock *sk, struct socket *parent)
{
  write_lock_bh(&sk->sk_callback_lock);
  rcu_assign_pointer(sk->sk_wq, parent->wq);
  parent->sk = sk;
  sk_set_socket(sk, parent);
  sk->sk_uid = SOCK_INODE(parent)->i_uid;
  security_sock_graft(sk, parent);
  write_unlock_bh(&sk->sk_callback_lock);
}

int inet_csk_wait_for_connect(struct sock *sk, long timeo)
{
  struct inet_connection_sock *icsk = inet_csk(sk);
  DEFINE_WAIT(wait);
  int err;

  for (;;) {
    /* waked up by: sk->sk_state_change = sock_def_wakeup; */
    prepare_to_wait_exclusive(sk_sleep(sk), &wait, TASK_INTERRUPTIBLE);
    release_sock(sk);
    if (reqsk_queue_empty(&icsk->icsk_accept_queue))
      timeo = schedule_timeout(timeo);
    sched_annotate_sleep();
    lock_sock(sk);
    err = 0;
    if (!reqsk_queue_empty(&icsk->icsk_accept_queue))
      break;
    err = -EINVAL;
    if (sk->sk_state != TCP_LISTEN)
      break;
    err = sock_intr_errno(timeo);
    if (signal_pending(current))
      break;
    err = -EAGAIN;
    if (!timeo)
      break;
  }
  finish_wait(sk_sleep(sk), &wait);
  return err;
}

wait_queue_head_t *sk_sleep(struct sock *sk)
{
  return &rcu_dereference_raw(sk->sk_wq)->wait;
}

void prepare_to_wait_exclusive(
  struct wait_queue_head *wq_head, struct wait_queue_entry *wq_entry, int state)
{
  unsigned long flags;

  wq_entry->flags |= WQ_FLAG_EXCLUSIVE;
  spin_lock_irqsave(&wq_head->lock, flags);
  if (list_empty(&wq_entry->entry))
    __add_wait_queue_entry_tail(wq_head, wq_entry);
  set_current_state(state);
  spin_unlock_irqrestore(&wq_head->lock, flags);
}

#define DEFINE_WAIT(name) DEFINE_WAIT_FUNC(name, autoremove_wake_function)

#define DEFINE_WAIT_FUNC(name, function)      \
  struct wait_queue_entry name = {            \
    .private  = current,                      \
    .func     = function,                     \
    .entry    = LIST_HEAD_INIT((name).entry), \
  }

int autoremove_wake_function(
  struct wait_queue_entry *wq_entry, unsigned mode, int sync, void *key)
{
  /* try_to_wake_up */
  int ret = default_wake_function(wq_entry, mode, sync, key);

  if (ret)
    list_del_init(&wq_entry->entry);

  return ret;
}

void finish_wait(
  struct wait_queue_head *wq_head, struct wait_queue_entry *wq_entry)
{
  unsigned long flags;

  __set_current_state(TASK_RUNNING);

  if (!list_empty_careful(&wq_entry->entry)) {
    spin_lock_irqsave(&wq_head->lock, flags);
    list_del_init(&wq_entry->entry);
    spin_unlock_irqrestore(&wq_head->lock, flags);
  }
}
```

# accept queue

Overview of Socket Hash Tables:
1. inet_bind_hashbucket (Bind Hash Table)
    * Purpose: Tracks sockets bound to specific local ports (e.g., via bind()).
    * Key: Local port number.
    * Prevents port conflicts (e.g., multiple sockets binding to the same port unless SO_REUSEADDR is set).

2. inet_listen_hashbucket (Listening Hash Table)
    * Key: Local port number.
    * Hash Function: Hashed via inet_lhashfn() (port-based hash).
    * Usage:
        * Looked up during incoming connection requests to find the listening socket.
        * Supports SO_REUSEPORT, allowing multiple listeners on the same port.

3. inet_ehash_bucket (Established Hash Table)
    * Key: 4-tuple (source IP, source port, destination IP, destination port).
    * Usage:
        * Used to match incoming packets to existing connections.
        * Time-wait sockets are also stored here to handle lingering packets after closure.

4. twchain (Time-Wait Hash Table) (Optional)
    * Purpose: In some kernel configurations, time-wait sockets are separated into a dedicated hash table.
    * Structure: Similar to ehash, using hlist_nulls_head.
    * Key: Same 4-tuple as ehash.
    * Usage: Reduces contention in ehash by isolating time-wait entries.

```c
struct inet_hashinfo {
    /* Established Hash Table */
    struct inet_ehash_bucket  *ehash;
    spinlock_t                *ehash_locks;
    unsigned int              ehash_mask;
    unsigned int              ehash_locks_mask;

    /* Bind Hash Table */
    struct kmem_cache               *bind_bucket_cachep;
    struct inet_bind_hashbucket     *bhash;
    unsigned int                    bhash_size;

    /* Listening Hash Table */
    unsigned int                    lhash2_mask;
    struct inet_listen_hashbucket   *lhash2;
};
```

## inet_csk_reqsk_queue_hash_add

```c
/* 1st handshake, invoked by tcp_conn_request, when first SYN arrives to server and fastopen is off
 * add the request_sock into the half-accept queue */
void inet_csk_reqsk_queue_hash_add(
  struct sock *sk, struct request_sock *req, unsigned long timeout)
{
    reqsk_queue_hash_req(req, timeout) {
        timer_setup(&req->rsk_timer, reqsk_timer_handler, TIMER_PINNED);
        mod_timer(&req->rsk_timer, jiffies + timeout);

        inet_ehash_insert(req_to_sk(req), NULL);

        smp_wmb();
        refcount_set(&req->rsk_refcnt, 2 + 1);
    }

    inet_csk_reqsk_queue_added(sk) {
        reqsk_queue_added(&inet_csk(sk)->icsk_accept_queue) {
            atomic_inc(&queue->young);
            atomic_inc(&queue->qlen);
        }
    }
}

bool inet_ehash_insert(struct sock *sk, struct sock *osk)
{
    struct inet_hashinfo *hashinfo = sk->sk_prot->h.hashinfo;
    struct hlist_nulls_head *list;
    struct inet_ehash_bucket *head;
    spinlock_t *lock;
    bool ret = true;

    sk->sk_hash = sk_ehashfn(sk);
    head = inet_ehash_bucket(hashinfo, sk->sk_hash);
    list = &head->chain;
    lock = inet_ehash_lockp(hashinfo, sk->sk_hash);

    spin_lock(lock);
    if (osk) {
        ret = sk_nulls_del_node_init_rcu(osk) {
            bool rc = __sk_nulls_del_node_init_rcu(sk) {
                if (sk_hashed(sk)) {
                    hlist_nulls_del_init_rcu(&sk->sk_nulls_node);
                    return true;
                }
                return false;
            }

            if (rc) {
                /* paranoid for a while -acme */
                WARN_ON(refcount_read(&sk->sk_refcnt) == 1);
                __sock_put(sk);
            }
            return rc;
        }
    }
    if (ret) {
        __sk_nulls_add_node_rcu(sk, list) {
            hlist_nulls_add_head_rcu(&sk->sk_nulls_node, list);
        }
    }
    spin_unlock(lock);
    return ret;
}
```

## inet_ehash_nolisten

```c
/* 3rd handshake, invoked by tcp_v4_syn_recv_sock when thres-way handshake completed at server
 * add socket hashinfo */
bool inet_ehash_nolisten(struct sock *sk, struct sock *osk)
{
    bool ok = inet_ehash_insert(sk, osk);
        --->

    if (ok) {
        sock_prot_inuse_add(sock_net(sk), sk->sk_prot, 1);
    } else {
        percpu_counter_inc(sk->sk_prot->orphan_count);
        inet_sk_set_state(sk, TCP_CLOSE);
        sock_set_flag(sk, SOCK_DEAD);
        inet_csk_destroy_sock(sk);
    }
    return ok;
}
```

## inet_csk_reqsk_queue_drop

```c
/* 3rd handshake, invoked at inet_csk_complete_hashdance
 * remove the request_sock from half-accept quque */
void inet_csk_reqsk_queue_drop_and_put(struct sock *sk, struct request_sock *req)
{
    inet_csk_reqsk_queue_drop(sk, req) {
        /* return true if req was found in the ehash table */
        ret = reqsk_queue_unlink(&inet_csk(sk)->icsk_accept_queue, req) {
            struct inet_hashinfo *hashinfo = req_to_sk(req)->sk_prot->h.hashinfo;
            bool found = false;

            if (sk_hashed(req_to_sk(req))) {
                spinlock_t *lock = inet_ehash_lockp(hashinfo, req->rsk_hash);

                spin_lock(lock);
                found = __sk_nulls_del_node_init_rcu(req_to_sk(req));
                spin_unlock(lock);
            }
            if (timer_pending(&req->rsk_timer) && del_timer_sync(&req->rsk_timer))
                reqsk_put(req);
            return found;
        }
        if (ret) {
            reqsk_queue_removed(&inet_csk(sk)->icsk_accept_queue, req) {
                if (req->num_timeout == 0)
                    atomic_dec(&queue->young);
                atomic_dec(&queue->qlen);
            }
            reqsk_put(req);
        }
    }

    reqsk_put(req);
}
```

## inet_csk_reqsk_queue_add

```c
/* 3rd handshake, invoked by tcp_conn_request, when first SYN arrives to server and fastopen is on
 * add the new connected sock into the accept queue */
sock *inet_csk_reqsk_queue_add(struct sock *sk, struct request_sock *req, struct sock *child)
{
    struct request_sock_queue *queue = &inet_csk(sk)->icsk_accept_queue;

    spin_lock(&queue->rskq_lock);
    if (unlikely(sk->sk_state != TCP_LISTEN)) {
        inet_child_forget(sk, req, child);
        child = NULL;
    } else {
        req->sk = child;
        req->dl_next = NULL;
        if (queue->rskq_accept_head == NULL)
            WRITE_ONCE(queue->rskq_accept_head, req);
        else
            queue->rskq_accept_tail->dl_next = req;
        queue->rskq_accept_tail = req;
        sk_acceptq_added(sk) {
            sk->sk_ack_backlog++;
        }
    }
    spin_unlock(&queue->rskq_lock);
    return child;
}
```

## reqsk_queue_remove

```c
/* invoked by accept, remove the connected soket from accept queue */
request_sock *reqsk_queue_remove(struct request_sock_queue *queue, struct sock *parent)
{
    struct request_sock *req;

    spin_lock_bh(&queue->rskq_lock);
    req = queue->rskq_accept_head;
    if (req) {
        sk_acceptq_removed(parent) {
            sk->sk_ack_backlog--;
        }
        WRITE_ONCE(queue->rskq_accept_head, req->dl_next);
        if (queue->rskq_accept_head == NULL)
            queue->rskq_accept_tail = NULL;
    }
    spin_unlock_bh(&queue->rskq_lock);
    return req;
}
```

## queue_is_full_len

```c
bool sk_acceptq_is_full(const struct sock *sk)
{
    return sk->sk_ack_backlog > sk->sk_max_ack_backlog;
}

int inet_csk_reqsk_queue_is_full(const struct sock *sk)
{
    return inet_csk_reqsk_queue_len(sk) >= sk->sk_max_ack_backlog;
}

int inet_csk_reqsk_queue_len(const struct sock *sk)
{
    return reqsk_queue_len(&inet_csk(sk)->icsk_accept_queue);
}

int reqsk_queue_len(const struct request_sock_queue *queue)
{
    return atomic_read(&queue->qlen);
}

int reqsk_queue_len_young(const struct request_sock_queue *queue)
{
    return atomic_read(&queue->young);
}
```

# shutdown

```c
SYSCALL_DEFINE2(shutdown, int, fd, int, how)
{
  return __sys_shutdown(fd, how);
}

int __sys_shutdown(int fd, int how)
{
  int err, fput_needed;
  struct socket *sock;

  sock = sockfd_lookup_light(fd, &err, &fput_needed);
  if (sock != NULL) {
    err = security_socket_shutdown(sock, how);
    if (!err)
      err = sock->ops->shutdown(sock, how);
    fput_light(sock->file, fput_needed);
  }
  return err;
}

const struct proto_ops inet_stream_ops = {
  .shutdown = inet_shutdown,
};

int inet_shutdown(struct socket *sock, int how)
{
  struct sock *sk = sock->sk;
  int err = 0;

  /* This should really check to make sure
   * the socket is a TCP socket. (WHY AC...) */
  how++; /* maps 0->1 has the advantage of making bit 1 rcvs and
           1->2 bit 2 snds.
           2->3 */
  if ((how & ~SHUTDOWN_MASK) || !how)  /* MAXINT->0 */
    return -EINVAL;

  lock_sock(sk);
  if (sock->state == SS_CONNECTING) {
    if ((1 << sk->sk_state) & (TCPF_SYN_SENT | TCPF_SYN_RECV | TCPF_CLOSE))
      sock->state = SS_DISCONNECTING;
    else
      sock->state = SS_CONNECTED;
  }

  switch (sk->sk_state) {
  case TCP_CLOSE:
      err = -ENOTCONN;
      /* Hack to wake up other listeners, who can poll for
        EPOLLHUP, even on eg. unconnected UDP sockets -- RR */
      /* fall through */
  default:
      sk->sk_shutdown |= how;
      if (sk->sk_prot->shutdown)
        sk->sk_prot->shutdown(sk, how);
      break;

  /* Remaining two branches are temporary solution for missing
   * close() in multithreaded environment. It is _not_ a good idea,
   * but we have no choice until close() is repaired at VFS level. */
  case TCP_LISTEN:
      if (!(how & RCV_SHUTDOWN))
        break;
      /* fall through */
  case TCP_SYN_SENT:
      err = sk->sk_prot->disconnect(sk, O_NONBLOCK);
      sock->state = err ? SS_DISCONNECTING : SS_UNCONNECTED;
      break;
  }

  /* Wake up anyone sleeping in poll. */
  sk->sk_state_change(sk);
  release_sock(sk);
  return err;
}

struct proto tcp_prot = {
  .shutdown   = tcp_shutdown,
  .disconnect = tcp_disconnect
};

/* shutdown the sending side of a connection. Much like close except
 * that we don't receive shut down or sock_set_flag(sk, SOCK_DEAD). */
void tcp_shutdown(struct sock *sk, int how)
{
  if (!(how & SEND_SHUTDOWN))
    return;

  /* If we've already sent a FIN, or it's a closed state, skip this. */
  if ((1 << sk->sk_state) & (TCPF_ESTABLISHED | TCPF_SYN_SENT |TCPF_SYN_RECV | TCPF_CLOSE_WAIT)) {
    /* Clear out any half completed packets.  FIN if needed. */
    if (tcp_close_state(sk))
      tcp_send_fin(sk);
  }
}

int tcp_close_state(struct sock *sk)
{
  int next = (int)new_state[sk->sk_state];
  int ns = next & TCP_STATE_MASK;

  tcp_set_state(sk, ns);

  return next & TCP_ACTION_FIN;
}

const unsigned char new_state[16] = {
  /* current state:       new state:         action:  */
  [0 /* (Invalid) */]   = TCP_CLOSE,
  [TCP_ESTABLISHED]     = TCP_FIN_WAIT1 | TCP_ACTION_FIN,
  [TCP_SYN_SENT]        = TCP_CLOSE,
  [TCP_SYN_RECV]        = TCP_FIN_WAIT1 | TCP_ACTION_FIN,
  [TCP_FIN_WAIT1]       = TCP_FIN_WAIT1,
  [TCP_FIN_WAIT2]       = TCP_FIN_WAIT2,
  [TCP_TIME_WAIT]       = TCP_CLOSE,
  [TCP_CLOSE]           = TCP_CLOSE,
  [TCP_CLOSE_WAIT]      = TCP_LAST_ACK  | TCP_ACTION_FIN,
  [TCP_LAST_ACK]        = TCP_LAST_ACK,
  [TCP_LISTEN]          = TCP_CLOSE,
  [TCP_CLOSING]         = TCP_CLOSING,
  [TCP_NEW_SYN_RECV]    = TCP_CLOSE,  /* should not happen ! */
};
```

```c
__sys_shutdown();
  sock->ops->shutdown();
    inet_shutdown();
      sk->sk_shutdown |= how;
      sk->sk_prot->shutdown();
        tcp_shutdown();
          if (!(how & SEND_SHUTDOWN))
            return;
          if (tcp_close_state())
            tcp_send_fin();
```

# sk_buff

* [kmalloc_slab](./linux-kernel-mem.md#kmalloc)
* [slab_alloc](./linux-kernel-mem.md#slab_alloc)

* [How sk_buffs alloc work](http://vger.kernel.org/~davem/skb_data.html)
* [Management of sk_buffs](https://people.cs.clemson.edu/~westall/853/notes/skbuff.pdf)

<img src='../images/kernel/net-sk_buf.svg' style='max-height:850px'/>

```c
struct sk_buff {
    union {
        struct {
            struct sk_buff    *next;
            struct sk_buff    *prev;
            union {
                struct net_device *dev;
                unsigned long     dev_scratch;
            };
        };
        struct rb_node  rbnode; /* used in netem & tcp stack */
    };

    union {
        struct sock    *sk;
        int            ip_defrag_offset;
    };

    char            cb[48] __aligned(8);

    unsigned int    len,    /* bytes of both linear and non-linear data */
                    data_len; /* bytes of non-linear data */
    __u16           mac_len, hdr_len;

    __be16          protocol;
    __u16           transport_header;
    __u16           network_header;
    __u16           mac_header;

    /* typedef unsigned char *sk_buff_data_t; */
    sk_buff_data_t     tail;
    sk_buff_data_t     end;
    unsigned char     *head, *data;
    unsigned int      truesize;
    refcount_t        users;
};

struct skb_shared_info {
  __u8    __unused, meta_len;
  __u8    nr_frags, tx_flags;

  unsigned short  gso_size; /* generic segmentation size */
  unsigned int    gso_type; /* SKB_GSO_TCPV4 or SKB_GSO_TCPV6 */
  unsigned short  gso_segs;

  struct sk_buff                *frag_list;
  struct skb_shared_hwtstamps   hwtstamps;
  u32                           tskey;

  /* must be last field, see pskb_expand_head() */
  skb_frag_t                    frags[MAX_SKB_FRAGS];
};

typedef struct skb_frag_struct skb_frag_t;

struct skb_frag_struct {
    struct {
        struct page *p;
    } page;

    __u32 page_offset;
    __u32 size;
};

struct proto tcp_prot = {
  .max_header = MAX_TCP_HEADER,
};

#define MAX_TCP_HEADER  L1_CACHE_ALIGN(128 + MAX_HEADER)
#define L1_CACHE_ALIGN(x) __ALIGN_KERNEL(x, L1_CACHE_BYTES)

#define __ALIGN_KERNEL(x, a)  __ALIGN_KERNEL_MASK(x, (typeof(x))(a) - 1)
#define __ALIGN_KERNEL_MASK(x, mask)  (((x) + (mask)) & ~(mask))

/* L1 cache line size */
#define L1_CACHE_SHIFT  (CONFIG_X86_L1_CACHE_SHIFT)
#define L1_CACHE_BYTES  (1 << L1_CACHE_SHIFT)

#if defined(CONFIG_HYPERV_NET)
# define LL_MAX_HEADER 128
#elif defined(CONFIG_WLAN) || IS_ENABLED(CONFIG_AX25)
# if defined(CONFIG_MAC80211_MESH)
#  define LL_MAX_HEADER 128
# else
#  define LL_MAX_HEADER 96
# endif
#else
# define LL_MAX_HEADER 32
#endif

#if !IS_ENABLED(CONFIG_NET_IPIP) && !IS_ENABLED(CONFIG_NET_IPGRE) && \
    !IS_ENABLED(CONFIG_IPV6_SIT) && !IS_ENABLED(CONFIG_IPV6_TUNNEL)
#define MAX_HEADER LL_MAX_HEADER
#else
#define MAX_HEADER (LL_MAX_HEADER + 48)
#endif

struct sk_buff *sk_stream_alloc_skb(struct sock *sk, int size, gfp_t gfp,
            bool force_schedule)
{
  struct sk_buff *skb;

  /* The TCP header must be at least 32-bit aligned.  */
  size = ALIGN(size, 4);

  if (unlikely(tcp_under_memory_pressure(sk)))
    sk_mem_reclaim_partial(sk);

  /* max_header L1_CACHE_ALIGN(128 + MAX_HEADER) */
  skb = alloc_skb_fclone(size + sk->sk_prot->max_header, gfp);
  if (likely(skb)) {
    bool mem_scheduled;

    if (force_schedule) {
      mem_scheduled = true;
      sk_forced_mem_schedule(sk, skb->truesize);
    } else {
      mem_scheduled = sk_wmem_schedule(sk, skb->truesize);
    }
    if (likely(mem_scheduled)) {
      /* reserve spaces for headers(Ethernet, IP and TCP) */
      skb_reserve(skb, sk->sk_prot->max_header) {
        skb->data += len;
        skb->tail += len;
      }
      /* Make sure that we have exactly size bytes
       * available to the caller, no more, no less. */
      skb->reserved_tailroom = skb->end - skb->tail - size;
      INIT_LIST_HEAD(&skb->tcp_tsorted_anchor);
      return skb;
    }
    __kfree_skb(skb);
  } else {
    sk->sk_prot->enter_memory_pressure(sk);
    sk_stream_moderate_sndbuf(sk);
  }
  return NULL;
}

struct sk_buff *alloc_skb_fclone(
  unsigned int size, gfp_t priority)
{
  return __alloc_skb(size, priority, SKB_ALLOC_FCLONE, NUMA_NO_NODE);
}

/* Allocate a new &sk_buff. The returned buffer has no headroom and a
 * tail room of at least size bytes. The object has a reference count
 * of one. The return is the buffer. On a failure the return is %NULL.
 *
 * Buffers may only be allocated from interrupts using a @gfp_mask of
 * %GFP_ATOMIC. */
struct sk_buff *__alloc_skb(unsigned int size, gfp_t gfp_mask, int flags, int node)
{
  struct kmem_cache *cache;
  struct skb_shared_info *shinfo;
  struct sk_buff *skb;
  u8 *data;
  bool pfmemalloc;

  cache = (flags & SKB_ALLOC_FCLONE) ? skbuff_fclone_cache : skbuff_head_cache;

  if (sk_memalloc_socks() && (flags & SKB_ALLOC_RX))
    gfp_mask |= __GFP_MEMALLOC;

  /* Get the HEAD */
  skb = kmem_cache_alloc_node(cache, gfp_mask & ~__GFP_DMA, node);
  if (!skb)
    goto out;
  prefetchw(skb);

  /* We do our best to align skb_shared_info on a separate cache
   * line. It usually works because kmalloc(X > SMP_CACHE_BYTES) gives
   * aligned memory blocks, unless SLUB/SLAB debug is enabled.
   * Both skb->head and skb_shared_info are cache line aligned. */
  size = SKB_DATA_ALIGN(size);
  size += SKB_DATA_ALIGN(sizeof(struct skb_shared_info));
  data = kmalloc_reserve(size, gfp_mask, node, &pfmemalloc);
  if (!data)
    goto nodata;

  /* kmalloc(size) might give us more room than requested.
   * Put skb_shared_info exactly at the end of allocated zone,
   * to allow max possible filling before reallocation. */
  size = SKB_WITH_OVERHEAD(ksize(data));
  prefetchw(data + size);

  /* Only clear those fields we need to clear, not those that we will
   * actually initialise below. Hence, don't put any more fields after
   * the tail pointer in struct sk_buff! */
  memset(skb, 0, offsetof(struct sk_buff, tail));
  /* Account for allocated memory : skb + skb->head */
  skb->truesize = SKB_TRUESIZE(size);
  skb->pfmemalloc = pfmemalloc;
  refcount_set(&skb->users, 1);
  skb->head = data;
  skb->data = data;
  skb_reset_tail_pointer(skb); /* skb->tail = skb->data; */
  skb->end = skb->tail + size;
  skb->mac_header = (typeof(skb->mac_header))~0U;
  skb->transport_header = (typeof(skb->transport_header))~0U;

  /* make sure we initialize shinfo sequentially */
  shinfo = skb_shinfo(skb);
  memset(shinfo, 0, offsetof(struct skb_shared_info, dataref));
  atomic_set(&shinfo->dataref, 1);

  if (flags & SKB_ALLOC_FCLONE) {
    struct sk_buff_fclones *fclones;

    fclones = container_of(skb, struct sk_buff_fclones, skb1);

    skb->fclone = SKB_FCLONE_ORIG;
    refcount_set(&fclones->fclone_ref, 1);

    fclones->skb2.fclone = SKB_FCLONE_CLONE;
  }
out:
  return skb;
nodata:
  kmem_cache_free(cache, skb);
  skb = NULL;
  goto out;
}

/* kmalloc_reserve is a wrapper around kmalloc_node_track_caller that tells
 * the caller if emergency pfmemalloc reserves are being used. If it is and
 * the socket is later found to be SOCK_MEMALLOC then PFMEMALLOC reserves
 * may be used. Otherwise, the packet data may be discarded until enough
 * memory is free */
#define kmalloc_reserve(size, gfp, node, pfmemalloc) \
  __kmalloc_reserve(size, gfp, node, _RET_IP_, pfmemalloc)

void *__kmalloc_reserve(size_t size, gfp_t flags, int node,
            unsigned long ip, bool *pfmemalloc)
{
  void *obj;
  bool ret_pfmemalloc = false;

  /* Try a regular allocation, when that fails and we're not entitled
   * to the reserves, fail. */
  obj = kmalloc_node_track_caller(size, flags | __GFP_NOMEMALLOC | __GFP_NOWARN, node);
  if (obj || !(gfp_pfmemalloc_allowed(flags)))
    goto out;

  /* Try again but now we are using pfmemalloc reserves */
  ret_pfmemalloc = true;
  obj = kmalloc_node_track_caller(size, flags, node);

out:
  if (pfmemalloc)
    *pfmemalloc = ret_pfmemalloc;

  return obj;
}

/* slub.c */
void *__kmalloc_track_caller(size_t size, gfp_t gfpflags, unsigned long caller)
{
  struct kmem_cache *s;
  void *ret;

  if (unlikely(size > KMALLOC_MAX_CACHE_SIZE))
    return kmalloc_large(size, gfpflags);

  s = kmalloc_slab(size, gfpflags);

  if (unlikely(ZERO_OR_NULL_PTR(s)))
    return s;

  ret = slab_alloc(s, gfpflags, caller);

  /* Honor the call site pointer we received. */
  trace_kmalloc(caller, ret, size, s->size, gfpflags);

  return ret;
}
```

## skb_clone
```c

/* Duplicate an &sk_buff. The new one is not owned by a socket. Both
 * copies share the same packet data but not structure. The new
 * buffer has a reference count of 1. If the allocation fails the
 * function returns %NULL otherwise the new buffer is returned.
 *
 * If this function is called from an interrupt gfp_mask() must be
 * %GFP_ATOMIC */
struct sk_buff *skb_clone(struct sk_buff *skb, gfp_t gfp_mask)
{
  struct sk_buff_fclones *fclones =
    container_of(skb, struct sk_buff_fclones, skb1);
  struct sk_buff *n;

  if (skb_orphan_frags(skb, gfp_mask))
    return NULL;

  if (skb->fclone == SKB_FCLONE_ORIG &&
      refcount_read(&fclones->fclone_ref) == 1) {
    n = &fclones->skb2;
    refcount_set(&fclones->fclone_ref, 2);
  } else {
    if (skb_pfmemalloc(skb))
      gfp_mask |= __GFP_MEMALLOC;

    n = kmem_cache_alloc(skbuff_head_cache, gfp_mask);
    if (!n)
      return NULL;

    n->fclone = SKB_FCLONE_UNAVAILABLE;
  }

  return __skb_clone(n, skb);
}

struct sk_buff *__skb_clone(struct sk_buff *n, struct sk_buff *skb)
{
#define C(x) n->x = skb->x

  n->next = n->prev = NULL;
  n->sk = NULL;
  __copy_skb_header(n, skb);

  C(len);
  C(data_len);
  C(mac_len);
  n->hdr_len = skb->nohdr ? skb_headroom(skb) : skb->hdr_len;
  n->cloned = 1;
  n->nohdr = 0;
  n->peeked = 0;
  C(pfmemalloc);
  n->destructor = NULL;
  C(tail);
  C(end);
  C(head);
  C(head_frag);
  C(data);
  C(truesize);
  refcount_set(&n->users, 1);

  atomic_inc(&(skb_shinfo(skb)->dataref));
  skb->cloned = 1;

  return n;
#undef C
}
```

```c
sk_stream_alloc_skb()
  size = ALIGN(size, 4)
  skb = alloc_skb_fclone()
    __alloc_skb()
      struct kmem_cache* cache = (flags & SKB_ALLOC_FCLONE) ? skbuff_fclone_cache : skbuff_head_cache;
      skb = kmem_cache_alloc_node(cache) /* Get the HEAD */
      data = kmalloc_reserve()
        __kmalloc_reserve()
          kmalloc_node_track_caller()
            struct kmem_cache *s = (size > KMALLOC_MAX_CACHE_SIZE)
              ? kmalloc_large(size, gfpflags)
              : kmalloc_slab()
            slab_alloc(s, gfpflags, caller)

      skb->head = data;
      skb->data = data;
      skb->tail = skb->data;
      skb->end = skb->tail + size;
      skb->mac_header = (typeof(skb->mac_header))~0U;
      skb->transport_header = (typeof(skb->transport_header))~0U;

  skb_reserve(skb, sk->sk_prot->max_header)
    skb->data += len;
    skb->tail += len;
```

## kfree_skb_partial

```c
void skb_release_head_state(struct sk_buff *skb)
{
    skb_dst_drop(skb);
    if (skb->destructor) {
        /* skb->destructor = skb_is_tcp_pure_ack(skb) ? __sock_wfree : tcp_wfree; */
        skb->destructor(skb) {
            tcp_wfree(struct sk_buff *skb) {
                struct sock *sk = skb->sk;
                struct tcp_sock *tp = tcp_sk(sk);
                unsigned long flags, nval, oval;
                struct tsq_tasklet *tsq;
                bool empty;

                /* Keep one reference on sk_wmem_alloc.
                 * Will be released by sk_free() from here or tcp_tasklet_func() */
                WARN_ON(refcount_sub_and_test(skb->truesize - 1, &sk->sk_wmem_alloc));

                if (refcount_read(&sk->sk_wmem_alloc) >= SKB_TRUESIZE(1)
                    && this_cpu_ksoftirqd() == current)
                    goto out;

                oval = smp_load_acquire(&sk->sk_tsq_flags);
                do {
                    if (!(oval & TSQF_THROTTLED) || (oval & TSQF_QUEUED))
                        goto out;

                    nval = (oval & ~TSQF_THROTTLED) | TSQF_QUEUED;
                } while (!try_cmpxchg(&sk->sk_tsq_flags, &oval, nval));

                /* queue this socket to tasklet queue */
                tsq = this_cpu_ptr(&tsq_tasklet);
                empty = list_empty(&tsq->head);
                list_add(&tp->tsq_node, &tsq->head);
                if (empty) {
                    tasklet_schedule(&tsq->tasklet) {
                        if (!test_and_set_bit(TASKLET_STATE_SCHED, &t->state)) {
                            __tasklet_schedule(t) {
                                __tasklet_schedule_common(t, &tasklet_vec, TASKLET_SOFTIRQ);
                            }
                        }
                    }
                }
                return;
            out:
                sk_free(sk);
            }
        }
    }
#if IS_ENABLED(CONFIG_NF_CONNTRACK)
    nf_conntrack_put(skb_nfct(skb));
#endif
    skb_ext_put(skb);
}


void tcp_tasklet_func(struct tasklet_struct *t)
{
    /* tsq: tcp small queue */
    struct tsq_tasklet *tsq = from_tasklet(tsq,  t, tasklet);
    LIST_HEAD(list);
    unsigned long flags;
    struct list_head *q, *n;
    struct tcp_sock *tp;
    struct sock *sk;

    local_irq_save(flags);
    list_splice_init(&tsq->head, &list);
    local_irq_restore(flags);

    list_for_each_safe(q, n, &list) {
        tp = list_entry(q, struct tcp_sock, tsq_node);
        list_del(&tp->tsq_node);

        sk = (struct sock *)tp;
        clear_bit(TSQ_QUEUED, &sk->sk_tsq_flags);

        tcp_tsq_handler(sk) {
            if (!sock_owned_by_user(sk)) {
                tcp_tsq_write(sk) {
                    if ((1 << sk->sk_state) &
                        (TCPF_ESTABLISHED | TCPF_FIN_WAIT1 | TCPF_CLOSING |
                            TCPF_CLOSE_WAIT  | TCPF_LAST_ACK)) {
                        struct tcp_sock *tp = tcp_sk(sk);

                        if (tp->lost_out > tp->retrans_out &&
                            tcp_snd_cwnd(tp) > tcp_packets_in_flight(tp)) {
                            tcp_mstamp_refresh(tp);
                            tcp_xmit_retransmit_queue(sk);
                        }

                        tcp_write_xmit(sk, tcp_current_mss(sk), tp->nonagle,
                                0, GFP_ATOMIC);
                    }
                }
            } else if (!test_and_set_bit(TCP_TSQ_DEFERRED, &sk->sk_tsq_flags)) {
                sock_hold(sk);
            }
        }
        sk_free(sk) {

        }
    }
}
```

# write

* [Monitoring and Tuning the Linux Networking Stack: Sending Data](https://blog.packagecloud.io/monitoring-tuning-linux-networking-stack-sending-data/)
* [How TCP output engine works](http://vger.kernel.org/~davem/tcp_output.html)

![](../images/kernel/net-read-write-route-bridge.svg)

---

<img src='../images/kernel/net-filter-3.png' style='max-height:850px'/>

---

<img src='../images/kernel/net-write.svg'/>

## call-graph-write

```c
/* vfs layer */
SYSCALL_DEFINE3(write, fd, buf, count) {
    vfs_write(file, buf, counts) {
        file->f_op->write_iter(kio, iter);
    }
}

/* socket layer */
sock_write_iter() {
    sock_sendmsg() {
        sock_sendmsg_nosec() {
            sock->ops->sendmsg() {
                inet_stream_ops.sendmsg() {
                    sk->sk_prot->sendmsg(); /* tcp_prot.sendmsg */
                }
            }
        }
    }
}

/* tcp layer */
tcp_sendmsg() {
    if (flags & MSG_FASTOPEN) {
        err = tcp_sendmsg_fastopen();
    }

/* 1. wait for connection */
    if (!TCPF_ESTABLISHED) {
        sk_stream_wait_connect();
    }

    mss_now = tcp_send_mss(sk, &size_goal, flags);

    while (msg_data_left(msg)) {
/* 2. alloc skb & insert into sk_write_queue */
        skb = tcp_stream_alloc_skb(); /* alloc new one if null */
        tcp_skb_entail(sk, skb); /* add into sk->sk_write_queue */

/* 3. copy data */
        if (!zc) {                          /* 3.1 copy to sock's sk_frag page_frag */
            struct page_frag *pfrag = sk_page_frag(sk);
            sk_page_frag_refill(sk, pfrag);
            skb_copy_to_page_nocache();
        } else if (zc == MSG_ZEROCOPY) {    /* 3.2 zero copy */
            skb_zerocopy_iter_stream();
        } else if (zc == MSG_SPLICE_PAGES) {/* splice to sk_buff's skb_shared_info */
            skb_splice_from_iter();
        }

/* 4. send data */
        if (skb->len < size_goal || (flags & MSG_OOB) || unlikely(tp->repair))
            continue;

        tcp_write_xmit() {
            while ((skb = tcp_send_head(sk))) {
                max_segs = tcp_tso_segs();
                tso_segs = tcp_init_tso_segs();

                /* 4.1 congestion control */
                tcp_cwnd_test();
                tcp_snd_wnd_test();
                tcp_nagle_test();
                tcp_tso_should_defer();

                /* 4.2 fragment */
                limit = tcp_mss_split_point();
                tso_fragment() {
                    skb_split() {
                        skb_split_inside_header();
                        skb_split_no_header();
                    }
                }

                tcp_transmit_skb() {
                    /* 4.3 build TCP header */
                    /* 4.4 send msg: ipv4_specific.ip_queue_xmit */
                    icsk->icsk_af_ops->queue_xmit(sk, skb, &inet->cork.fl);
                }

                /* 4.5 add into retransmit */
                tcp_event_new_data_sent() {
                    __skb_unlink(skb, &sk->sk_write_queue);
                    tcp_rbtree_insert(&sk->tcp_rtx_queue, skb);
                    tcp_rearm_rto(sk);
                }
            }

            tcp_cwnd_validate() {
                tcp_cwnd_application_limited();
            }
        }
    }
}

/* ip layer */
ipv4_specific.ip_queue_xmit() {
    /* 1. get route table */
    rt = ip_route_output_ports() {
        ip_route_output_key_hash_rcu() {
            fib_lookup();
            __mkroute_output() {
                rt_dst_alloc();
            }
        }
    }
    skb_dst_set_noref(skb, &rt->dst);

    /* 2. build IP header */

    /* 3. netfilter */
    ip_local_out() {
        nf_hook();                          /* NF_INET_LOCAL_OUT */
        dst_output() {
            skb_dst(skb)->output() {
                ip_output() { /* ipv4_dst_ops.output() */
                    NF_HOOK_COND();         /* NF_INET_POST_ROUTING */
                    ip_finish_output() {
                        if (skb->len > mtu)
                            return ip_fragment(sk, ip_finish_output2)
                        ip_finish_output2();
                    }
                }
            }
        }
    }
}

/* mac layer */
ip_finish_output2() {
    exthop = rt_nexthop(rt, ip_hdr(skb)->daddr);
    neigh = ipv4_neigh_lookup_noref(dev, nexthop);
    neigh_output(neigh, skb) {
        neigh->output() {
            neigh_resolve_output() { /* arp_hh_ops.output */
                ret = neigh_event_send() {
                    __skb_queue_tail(&neigh->arp_queue, skb);
                    neigh_probe() {
                        arp_send_dst() {
                            skb = arp_create();
                            arp_xmit(skb); /* arp_rcv */
                        }
                    }
                }
                if (!ret) {
                    dev_queue_xmit(skb);
                }
            }
        }
    }
}


/* dev layer */
dev_queue_xmit() {
    struct net_device *dev = skb->dev;
    struct netdev_queue *txq = netdev_pick_tx(dev, skb);
    struct Qdisc *q = rcu_derefence_bh(txq->qdisc);

    if (static_branch_unlikely(&egress_needed_key)) {
        skb = sch_handle_egress(skb, &rc, dev);
    }
    txq = netdev_core_pick_tx(dev, skb, sb_dev);

    __dev_xmit_skb(skb, q, dev, txq) {
        dev_qdisc_enqueue() {
            q->enqueue(skb, q, &to_free);
        }

        if (qdisc_run_begin(q)) { /* check if qdisc is alreay running */
            __qdisc_run(q) {
                qdisc_restart(q, &pakcets) {
                    skb = dequeue_skb(q, &validate, packets); /* may dequeue bulk sbk */
                    txq = skb_get_tx_queue(dev, skb);
                    sch_direct_xmit(skb, q, dev, txq, root_lock, validate) {
                        dev_hard_start_xmit() {
                            while (skb) {
                                xmit_one() {
                                    netdev_start_xmit() {
                                        net_device_ops->ndo_start_xmit(skb, dev) {
                                            ixgb_xmit_frame();
                                        }
                                    }
                                }
                            }
                        }
                        if (dev_xmit_complete(ret)) {
                            /* Driver sent out skb successfully or skb was consumed */
                            ret = qdisc_qlen(q);
                        } else {
                            /* Driver returned NETDEV_TX_BUSY - requeue skb */
                            ret = dev_requeue_skb(skb, q);
                        }
                    }
                }

                while (qdisc_restart(...)) {
                    if (quota <= 0 || need_resched()) {
                        __netif_reschedule(q) {
                            q->next_sched = NULL;
                            *sd->output_queue_tailp = q;
                            sd->output_queue_tailp = &q->next_sched;

                            raise_softirq_irqoff(NET_TX_SOFTIRQ) { /* return to user space */
                                net_tx_action() { /* NET_TX_SOFTIRQ */
                                    qdisc_run(q) {
                                        if (qdisc_run_begin(q)) {
                                            __qdisc_run(q);
                                            qdisc_run_end(q) {
                                                __clear_bit(__QDISC_STATE2_RUNNING, &qdisc->state2);
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }

            qdisc_run_end(q);
        }
    }
}

/* driver layer */
ixgb_xmit_frame();
```

## vfs layer tx

```c
SYSCALL_DEFINE3(write, unsigned int, fd, const char __user *, buf,
    size_t, count)
{
  struct fd f = fdget_pos(fd);
  loff_t pos = file_pos_read(f.file);
    ret = vfs_write(f.file, buf, count, &pos);
}

ssize_t __vfs_write(struct file *file, const char __user *p, size_t count,
        loff_t *pos)
{
  if (file->f_op->write)
    return file->f_op->write(file, p, count, pos);
  else if (file->f_op->write_iter)
    return new_sync_write(file, p, count, pos);
  else
    return -EINVAL;
}

static ssize_t new_sync_read(
  struct file *filp, char __user *buf, size_t len, loff_t *ppos)
{
  struct iovec iov = { .iov_base = buf, .iov_len = len };
  struct kiocb kiocb;
  struct iov_iter iter;
  ssize_t ret;

  init_sync_kiocb(&kiocb, filp);
  kiocb.ki_pos = *ppos;
  iov_iter_init(&iter, READ, &iov, 1, len);

  /* file->f_op->read_iter(kio, iter); */
  ret = call_read_iter(filp, &kiocb, &iter);
  *ppos = kiocb.ki_pos;
  return ret;
}

const struct file_operations socket_file_ops = {
  .owner          =  THIS_MODULE,
  .llseek         =  no_llseek,
  .read_iter      =  sock_read_iter,
  .write_iter     =  sock_write_iter,
  .poll           =  sock_poll,
  .unlocked_ioctl =  sock_ioctl,
  .mmap           =  sock_mmap,
  .release        =  sock_close,
};

const struct file_operations eventfd_fops = {
  .show_fdinfo    = eventfd_show_fdinfo,
  .release        = eventfd_release,
  .poll           = eventfd_poll,
  .read           = eventfd_read,
  .write          = eventfd_write,
  .llseek         = noop_llseek,
};

ssize_t sock_write_iter(struct kiocb *iocb, struct iov_iter *from)
{
  struct file *file = iocb->ki_filp;
  struct socket *sock = file->private_data;
  struct msghdrg = {.msg_iter = *from, .msg_iocb = iocb};
  ssize_t res;

  res = sock_sendmsg(sock, &msg);
  *from = msg.msg_iter;
  return res;
}
```

## socket layer tx

```c
/* sock_sendmsg -> */
int sock_sendmsg_nosec(struct socket *sock, struct msghdr *msg)
{
  int ret = sock->ops->sendmsg(sock, msg, msg_data_left(msg));
}

/* inet_stream_ops.sendmsg */
int inet_sendmsg(struct socket *sock, struct msghdr *msg, size_t size)
{
  struct sock *sk = sock->sk;
  return sk->sk_prot->sendmsg(sk, msg, size);
}
```

## tcp layer tx

```sh
 0                   1                   2                   3
 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|          Source Port          |       Destination Port        |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                        Sequence Number                        |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                    Acknowledgment Number                      |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|  Data |           |U|A|P|R|S|F|                               |
| Offset| Reserved  |R|C|S|S|Y|I|            Window             |
|       |           |G|K|H|T|N|N|                               |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|           Checksum            |         Urgent Pointer        |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                    Options (if Data Offset > 5)               |
|           kind(1) | lenth(1)  | info(n)                       |
|           0                   |   EOL                         |
|           1                   |   NOP                         |
|           2           4       |   MSS                         |
|           3           3       |   WSCALE/WSOPT                |
|           4           2       |   SACK                        |
|           5           N*8 + 2 |   [,]...[,]                   |
|           8           10      |   TSOPT: TSV(4) TSER(4)       |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
```
* **Source Port** (16 bits)
    * Identifies the sending application/process on the source device.
    * Range: 0 to 65,535.
    * Example: 49152 (ephemeral port, 11000000 00000000).

* **Destination Port** (16 bits)
    * Identifies the receiving application/process on the destination device.
    * Common examples: 80 (HTTP), 443 (HTTPS).
    * Example: 80 (00000000 01010000).

* **Sequence Number** (32 bits)
    * Tracks the order of bytes sent in the TCP stream. For the first segment, it’s the Initial Sequence Number (ISN); subsequent segments indicate the byte offset.
    * Example: 00000000 00000000 00000000 00000001 (1).

* **Acknowledgment Number** (32 bits)
    * Indicates the next byte the receiver expects to receive, acknowledging receipt of prior bytes. Used only if the ACK flag is set.
    * Example: 00000000 00000000 00000000 00000010 (acknowledging up to byte 1).

* **Data Offset** (4 bits)
    * Specifies the length of the TCP header in 32-bit words (4 bytes).
    * Minimum is 5 (20 bytes, no options); maximum is 15 (60 bytes with options).
    * Example: 0101 (5 words = 20 bytes).

* **Reserved** (6 bits)
    * Set to zero for future use.
    * Example: 000000.

* **Flags** (6 bits)
    * Control bits for TCP connection management:
    * URG: Urgent Pointer field is significant.
    * ACK: Acknowledgment field is significant.
    * PSH: Push data to the application immediately.
    * RST: Reset the connection.
    * SYN: Synchronize sequence numbers (connection start).
    * FIN: No more data from sender (connection end).
    * Example: 000010 (ACK set).

* **Window** (16 bits)
    * Specifies the size of the receiver’s buffer (in bytes) for incoming data, used for flow control.
    * Example: 00000000 11111111 (255 bytes).

* **Checksum** (16 bits)
    * Verifies the integrity of the TCP header, data, and a pseudo-header (parts of the IP header).
    * Example: 10101010 11001100 (varies).

* **Urgent Pointer** (16 bits)
    * Indicates the offset from the Sequence Number where urgent data ends (only valid if URG flag is set).
    * Example: 00000000 00000000 (no urgent data).

* **Options** (variable, 0-40 bytes)
    * Optional fields for features like Maximum Segment Size (MSS), Window Scaling, or Timestamps. Present if Data Offset > 5.
    * Padded to align with 32-bit boundaries.
    * Example: MSS option might be 02 04 05 B4 (MSS = 1460 bytes).

### copy data

```c
/* tcp_prot.sendmsg, copy data to skb */
int tcp_sendmsg(struct sock *sk, struct msghdr *msg, size_t size)
{
    struct tcp_sock *tp = tcp_sk(sk);
    struct ubuf_info *uarg = NULL;
    struct sk_buff *skb;
    struct sockcm_cookie sockc;
    int flags, err, copied = 0;
    int mss_now = 0, size_goal, copied_syn = 0;
    bool process_backlog = false;
    bool zc = false;
    long timeo;

    flags = msg->msg_flags;

/* determine zc behavior */
    if ((flags & MSG_ZEROCOPY) && size) {
        if (msg->msg_ubuf) {
            uarg = msg->msg_ubuf;
            net_zcopy_get(uarg);
            zc = sk->sk_route_caps & NETIF_F_SG;
        } else if (sock_flag(sk, SOCK_ZEROCOPY)) {
            skb = tcp_write_queue_tail(sk);
            uarg = msg_zerocopy_realloc(sk, size, skb_zcopy(skb));
            if (!uarg) {
                err = -ENOBUFS;
                goto out_err;
            }
            zc = sk->sk_route_caps & NETIF_F_SG;
            if (!zc)
                uarg->zerocopy = 0;
        }
    } else if (unlikely(msg->msg_flags & MSG_SPLICE_PAGES) && size) {
        if (sk->sk_route_caps & NETIF_F_SG)
            zc = MSG_SPLICE_PAGES;
    }

/* fast open */
    if (unlikely(flags & MSG_FASTOPEN || inet_sk(sk)->defer_connect) && !tp->repair) {
        err = tcp_sendmsg_fastopen(sk, msg, &copied_syn, size);
        if (err == -EINPROGRESS && copied_syn > 0)
            goto out;
        else if (err)
            goto out_err;
    }

    timeo = sock_sndtimeo(sk, flags & MSG_DONTWAIT);

    /* If a gap is detected between sends, mark the socket application-limited. */
    tcp_rate_check_app_limited(sk);

/* 1. wait for connect */
    if (((1 << sk->sk_state) & ~(TCPF_ESTABLISHED | TCPF_CLOSE_WAIT)) && !tcp_passive_fastopen(sk)) {
        err = sk_stream_wait_connect(sk, &timeo);
        if (err != 0)
            goto do_error;
    }

    if (unlikely(tp->repair)) {
        if (tp->repair_queue == TCP_RECV_QUEUE) {
            copied = tcp_send_rcvq(sk, msg, size);
            goto out_nopush;
        }

        err = -EINVAL;
        if (tp->repair_queue == TCP_NO_QUEUE)
            goto out_err;

        /* 'common' sending to sendq */
    }

    sockcm_init(&sockc, sk);
    if (msg->msg_controllen) {
        err = sock_cmsg_send(sk, msg, &sockc);
        if (unlikely(err)) {
            err = -EINVAL;
            goto out_err;
        }
    }

    /* This should be in poll */
    sk_clear_bit(SOCKWQ_ASYNC_NOSPACE, sk);

    /* Ok commence sending. */
    copied = 0;

restart:
    mss_now = tcp_send_mss(sk, &size_goal, flags);

    err = -EPIPE;
    if (sk->sk_err || (sk->sk_shutdown & SEND_SHUTDOWN))
        goto do_error;

    while (msg_data_left(msg) { return iov_iter_count(&msg->msg_iter); }) {
        int copy = 0;

/* 2. get an available skb */
        skb = tcp_write_queue_tail(sk) {
            return skb_peek_tail(&sk->sk_write_queue);
        }
        if (skb)
            copy = size_goal - skb->len;

        if (copy <= 0 || !tcp_skb_can_collapse_to(skb)) { /* !TCP_SKB_CB(skb)->eor */
            bool first_skb;

new_segment:
            ret = sk_stream_memory_free(sk) {
                return __sk_stream_memory_free(sk, 0) {
                    if (READ_ONCE(sk->sk_wmem_queued) >= READ_ONCE(sk->sk_sndbuf))
                        return false;

                    return sk->sk_prot->stream_memory_free
                        ? INDIRECT_CALL_INET_1(sk->sk_prot->stream_memory_free,
                                    tcp_stream_memory_free, sk, wake)
                        : true;
                }
            }
            if (!ret)
                goto wait_for_sndbuf;

            if (unlikely(process_backlog >= 16)) {
                process_backlog = 0;
                if (sk_flush_backlog(sk))
                    goto restart;
            }

            first_skb = tcp_rtx_and_write_queues_empty(sk);
            linear = select_size(first_skb, zc);
            skb = tcp_stream_alloc_skb(sk, sk->sk_allocation, first_skb);
            if (!skb)
                goto wait_for_space;

            process_backlog++;
            skb->decrypted = !!(flags & MSG_SENDPAGE_DECRYPTED);

            tcp_skb_entail(sk, skb);
            copy = size_goal;

            if (tp->repair)
                TCP_SKB_CB(skb)->sacked |= TCPCB_REPAIRED;
        }

        /* Try to append data to the end of skb. */
        if (copy > msg_data_left(msg))
            copy = msg_data_left(msg);

/* 3. copy data */
        if (zc == 0) {
/* 3.1 copy to page_frag */
            bool merge = true;
            int i = skb_shinfo(skb)->nr_frags;
            struct page_frag *pfrag = sk_page_frag(sk) {
                return (sk->sk_use_task_frag)
                    ? &current->task_frag
                    : &sk->sk_frag;
            }

            if (!sk_page_frag_refill(sk, pfrag))
                goto wait_for_space;

            if (!skb_can_coalesce(skb, i, pfrag->page, pfrag->offset)) {
                if (i >= READ_ONCE(net_hotdata.sysctl_max_skb_frags)) {
                    tcp_mark_push(tp, skb);
                    goto new_segment;
                }
                merge = false;
            }

            copy = min_t(int, copy, pfrag->size - pfrag->offset);

            if (unlikely(skb_zcopy_pure(skb) || skb_zcopy_managed(skb))) {
                if (tcp_downgrade_zcopy_pure(sk, skb))
                    goto wait_for_space;
                skb_zcopy_downgrade_managed(skb);
            }

            /* check if the socket can schedule copy bytes of memory from its write buffer. */
            copy = tcp_wmem_schedule(sk, copy) {
                int left;

                if (likely(sk_wmem_schedule(sk, copy)))
                    return copy;

                /* We could be in trouble if we have nothing queued.
                 * Use whatever is left in sk->sk_forward_alloc and tcp_wmem[0]
                 * to guarantee some progress. */
                left = READ_ONCE(sock_net(sk)->ipv4.sysctl_tcp_wmem[0]) - sk->sk_wmem_queued;
                if (left > 0)
                    sk_forced_mem_schedule(sk, min(left, copy));
                return min(copy, sk->sk_forward_alloc);
            }
            if (!copy)
                goto wait_for_space;

            err = skb_copy_to_page_nocache(sk, &msg->msg_iter, skb,
                pfrag->page, pfrag->offset, copy);
            if (err)
                goto do_error;

            if (merge) { /* merge the pfrag into &skb_shinfo(skb)->frags[i-1] */
                skb_frag_size_add(&skb_shinfo(skb)->frags[i - 1], copy) {
                    frag->len += delta;
                }
            } else { /* splice the pfrag into sk */
                skb_fill_page_desc(skb, i, pfrag->page, pfrag->offset, copy) {
                    skb_fill_netmem_desc(skb, i, page_to_netmem(page), off, size) {
                        __skb_fill_netmem_desc(skb, i, netmem, off, size);
                        skb_shinfo(skb)->nr_frags = i + 1;
                    }
                }
                page_ref_inc(pfrag->page);
            }
            pfrag->offset += copy;
        } else if (zc == MSG_ZEROCOPY)  {
/* 3.2 zero copy */
            /* First append to a fragless skb builds initial
                * pure zerocopy skb */
            if (!skb->len)
                skb_shinfo(skb)->flags |= SKBFL_PURE_ZEROCOPY;

            if (!skb_zcopy_pure(skb)) {
                copy = tcp_wmem_schedule(sk, copy);
                if (!copy)
                    goto wait_for_space;
            }

            err = skb_zerocopy_iter_stream(sk, skb, msg, copy, uarg);
            if (err == -EMSGSIZE || err == -EEXIST) {
                tcp_mark_push(tp, skb);
                goto new_segment;
            }
            if (err < 0)
                goto do_error;
            copy = err;
        } else if (zc == MSG_SPLICE_PAGES) {
/* 3.3 splice pages. Splice in data if we can; copy if we can't. */
            if (tcp_downgrade_zcopy_pure(sk, skb))
                goto wait_for_space;
            copy = tcp_wmem_schedule(sk, copy);
            if (!copy)
                goto wait_for_space;

            err = skb_splice_from_iter(skb, &msg->msg_iter, copy,
                            sk->sk_allocation);
            if (err < 0) {
                if (err == -EMSGSIZE) {
                    tcp_mark_push(tp, skb);
                    goto new_segment;
                }
                goto do_error;
            }
            copy = err;

            if (!(flags & MSG_NO_SHARED_FRAGS))
                skb_shinfo(skb)->flags |= SKBFL_SHARED_FRAG;

            sk_wmem_queued_add(sk, copy);
            sk_mem_charge(sk, copy);
        }

        if (!copied)
            TCP_SKB_CB(skb)->tcp_flags &= ~TCPHDR_PSH;

        WRITE_ONCE(tp->write_seq, tp->write_seq + copy);
        TCP_SKB_CB(skb)->end_seq += copy;
        tcp_skb_pcount_set(skb, 0);

        copied += copy;
        if (!msg_data_left(msg)) {
            if (unlikely(flags & MSG_EOR))
                TCP_SKB_CB(skb)->eor = 1;
            goto out;
        }

        if (skb->len < size_goal || (flags & MSG_OOB) || unlikely(tp->repair))
            continue;

/* 4.1 send data
* Three conditions to send data:
* 1. !msg_data_left(msg)
* 2. forced_push(tp)
* 3. skb == tcp_send_head(sk) */

        if (forced_push(tp)) { /* tp->write_seq > tp->pushed_seq + (tp->max_window >> 1)) */
            tcp_mark_push(tp, skb);
            __tcp_push_pending_frames(sk, mss_now, TCP_NAGLE_PUSH);
        } else if (skb == tcp_send_head(sk))
            tcp_push_one(sk, mss_now);

        continue;

wait_for_sndbuf:
        set_bit(SOCK_NOSPACE, &sk->sk_socket->flags);

        if (copied)
            tcp_push(sk, flags & ~MSG_MORE, mss_now, TCP_NAGLE_PUSH, size_goal);

        err = sk_stream_wait_memory(sk, &timeo);
        if (err != 0)
            goto do_error;

        mss_now = tcp_send_mss(sk, &size_goal, flags);
    }

/* 4.2 send data */
out:
    if (copied) {
        tcp_tx_timestamp(sk, sockc.tsflags);
        tcp_push(sk, flags, mss_now, tp->nonagle, size_goal);
    }

out_nopush:
    sock_zerocopy_put(uarg);
    return copied + copied_syn;

do_error:
    skb = tcp_write_queue_tail(sk);
do_fault:
    tcp_remove_empty_skb(sk, skb);

    if (copied + copied_syn)
        goto out;
out_err:
    sock_zerocopy_put_abort(uarg);
    err = sk_stream_error(sk, flags, err);
    /* make sure we wake any epoll edge trigger waiter */
    if (unlikely(tcp_rtx_and_write_queues_empty(sk) && err == -EAGAIN)) {
        sk->sk_write_space(sk);
        tcp_chrono_stop(sk, TCP_CHRONO_SNDBUF_LIMITED);
    }

    return err;
}

int skb_copy_to_page_nocache(
  struct sock *sk, struct iov_iter *from,
  struct sk_buff *skb,
  struct page *page,
  int off, int copy)
{
  int err;

  err = skb_do_copy_data_nocache(sk, skb, from, page_address(page) + off, copy, skb->len);
  if (err)
    return err;

  skb->len            += copy;
  skb->data_len       += copy;
  skb->truesize       += copy;
  sk->sk_wmem_queued  += copy;
  sk_mem_charge(sk, copy);
  return 0;
}

/* __tcp_push_pending_frames | tcp_push_one ->
 * 1. fragment segment
 * 2. congestion control
 * 3. Nagle algorithm
 * 3. slide window
 *
 * TSO: TCP Segmentation Offload */

 /* In order to avoid the overhead associated
  * with a large number of packets on the transmit path,
  * the Linux kernel implements several optimizations:
  * 1. TCP segmentation offload (TSO),
  * 2. UDP fragmentation offload (UFO) and
  * 3. generic segmentation offload (GSO).
  *
  * All of these optimizations allow the IP stack to create packets
  * which are larger than the MTU of the outgoing NIC.
  * For IPv4, packets as large as the IPv4 maximum of 65,536 bytes
  * can be created and queued to the driver queue.
  * In the case of TSO and UFO, the NIC hardware takes responsibility
  * for breaking the single large packet into packets small enough
  * to be transmitted on the physical interface.
  * For NICs without hardware support, GSO performs the same operation
  * in software immediately before queueing to the driver queue. */
bool tcp_write_xmit(struct sock *sk, unsigned int mss_now, int nonagle,
         int push_one, gfp_t gfp)
{
    struct tcp_sock *tp = tcp_sk(sk);
    struct sk_buff *skb;
    unsigned int tso_segs, sent_pkts;
    int cwnd_quota;
    int result;
    bool is_cwnd_limited = false, is_rwnd_limited = false;
    u32 max_segs;

    sent_pkts = 0;

    tcp_mstamp_refresh(tp);
    if (!push_one) {
        result = tcp_mtu_probe(sk); /* Do MTU probing. */
        if (!result) {
            return false;
        } else if (result > 0) {
            sent_pkts = 1;
        }
    }

    /* Return the number of segments we want in the skb we are transmitting. */
    max_segs = tcp_tso_segs(sk, mss_now);

    while ((skb = tcp_send_head(sk))) {
        unsigned int limit;

        if (tcp_pacing_check(sk))
        break;

        tso_segs = tcp_init_tso_segs(skb, mss_now); /* DIV_ROUND_UP(skb->len, mss_now) */

        if (unlikely(tp->repair) && tp->repair_queue == TCP_SEND_QUEUE) {
            /* "skb_mstamp" is used as a start point for the retransmit timer */
            tcp_update_skb_after_send(tp, skb);
            goto repair; /* Skip network transmission */
        }

        /* return how many segments are allowed */
        cwnd_quota = tcp_cwnd_test(tp, skb);
        if (!cwnd_quota) {
            if (push_one == 2)
                cwnd_quota = 1; /* Force out a loss probe pkt. */
            else
                break;
        }

        /* Does at least the first segment of SKB fit into the send window? */
        if (unlikely(!tcp_snd_wnd_test(tp, skb, mss_now))) {
            is_rwnd_limited = true;
            break;
        }

        if (tso_segs == 1) {
            if (unlikely(!tcp_nagle_test(tp, skb, mss_now, (tcp_skb_is_last(sk, skb) ? nonagle : TCP_NAGLE_PUSH))))
                break;
        } else {
            if (!push_one && tcp_tso_should_defer(sk, skb, &is_cwnd_limited, &is_rwnd_limited, max_segs))
                break;
        }

        limit = mss_now;
        if (tso_segs > 1 && !tcp_urg_mode(tp))
            /* Returns the portion of skb which can be sent right away */
            limit = tcp_mss_split_point(sk, skb, mss_now,
                    min_t(unsigned int, cwnd_quota, max_segs),
                    nonagle);

        if (skb->len > limit && unlikely(tso_fragment(sk, TCP_FRAG_IN_WRITE_QUEUE, skb, limit, mss_now, gfp)))
            break;

        if (tcp_small_queue_check(sk, skb, 0))
            break;

        /* Argh, we hit an empty skb(), presumably a thread
        * is sleeping in sendmsg()/sk_stream_wait_memory().
        * We do not want to send a pure-ack packet and have
        * a strange looking rtx queue with empty packet(s). */
        if (TCP_SKB_CB(skb)->end_seq == TCP_SKB_CB(skb)->seq)
            break;

        if (unlikely(tcp_transmit_skb(sk, skb, 1, gfp)))
            break;

    repair:
        /* Advance the send_head.  This one is sent out.
        * This call will increment packets_out. */
        tcp_event_new_data_sent(sk, skb);

        tcp_minshall_update(tp, mss_now, skb);
        sent_pkts += tcp_skb_pcount(skb);

        if (push_one)
            break;
    }

  if (is_rwnd_limited)
    tcp_chrono_start(sk, TCP_CHRONO_RWND_LIMITED);
  else
    tcp_chrono_stop(sk, TCP_CHRONO_RWND_LIMITED);

  is_cwnd_limited |= (tcp_packets_in_flight(tp) >= tp->snd_cwnd);
  if (likely(sent_pkts || is_cwnd_limited))
    tcp_cwnd_validate(sk, is_cwnd_limited);

  if (likely(sent_pkts)) {
    if (tcp_in_cwnd_reduction(sk))
      tp->prr_out += sent_pkts;

    /* Send one loss probe per tail loss episode. */
    if (push_one != 2)
      tcp_schedule_loss_probe(sk, false);
    return false;
  }

  return !tp->packets_out && !tcp_write_queue_empty(sk);
}

int tcp_init_tso_segs(struct sk_buff *skb, unsigned int mss_now)
{
  int tso_segs = tcp_skb_pcount(skb);

  if (!tso_segs || (tso_segs > 1 && tcp_skb_mss(skb) != mss_now)) {
    tcp_set_skb_tso_segs(skb, mss_now);
    tso_segs = tcp_skb_pcount(skb);
  }
  return tso_segs;
}

void tcp_set_skb_tso_segs(struct sk_buff *skb, unsigned int mss_now)
{
  if (skb->len <= mss_now) {
    /* Avoid the costly divide in the normal
     * non-TSO case. */
    tcp_skb_pcount_set(skb, 1);
    TCP_SKB_CB(skb)->tcp_gso_size = 0;
  } else {
    tcp_skb_pcount_set(skb, DIV_ROUND_UP(skb->len, mss_now));
    TCP_SKB_CB(skb)->tcp_gso_size = mss_now;
  }
}
```

### tso_fragment

```c
/* Returns the portion of skb which can be sent right away */
unsigned int tcp_mss_split_point(
  const struct sock *sk,
  const struct sk_buff *skb,
  unsigned int mss_now,
  unsigned int max_segs,
  int nonagle)
{
  const struct tcp_sock *tp = tcp_sk(sk);
  u32 partial, needed, window, max_len;

  window = tcp_wnd_end(tp) - TCP_SKB_CB(skb)->seq;
  max_len = mss_now * max_segs;

  if (likely(max_len <= window && skb != tcp_write_queue_tail(sk)))
    return max_len;

  needed = min(skb->len, window);

  if (max_len <= needed)
    return max_len;

  return needed;
}

/* Trim TSO SKB to LEN bytes, put the remaining data into a new packet
 * which is put after SKB on the list.  It is very much like
 * tcp_fragment() except that it may make several kinds of assumptions
 * in order to speed up the splitting operation.  In particular, we
 * know that all the data is in scatter-gather pages, and that the
 * packet has never been sent out before (and thus is not cloned). */
int tso_fragment(
  struct sock *sk, enum tcp_queue tcp_queue,
  struct sk_buff *skb, unsigned int len,
  unsigned int mss_now, gfp_t gfp)
{
  struct sk_buff *buff;
  int nlen = skb->len - len;
  u8 flags;

  /* All of a TSO frame must be composed of paged data.  */
  if (skb->len != skb->data_len)
    return tcp_fragment(sk, tcp_queue, skb, len, mss_now, gfp);

  buff = sk_stream_alloc_skb(sk, 0, gfp, true);

  sk->sk_wmem_queued += buff->truesize;
  sk_mem_charge(sk, buff->truesize);
  buff->truesize += nlen;
  skb->truesize -= nlen;

  /* Correct the sequence numbers. */
  TCP_SKB_CB(buff)->seq = TCP_SKB_CB(skb)->seq + len;
  TCP_SKB_CB(buff)->end_seq = TCP_SKB_CB(skb)->end_seq;
  TCP_SKB_CB(skb)->end_seq = TCP_SKB_CB(buff)->seq;

  /* PSH and FIN should only be set in the second packet. */
  flags = TCP_SKB_CB(skb)->tcp_flags;
  TCP_SKB_CB(skb)->tcp_flags = flags & ~(TCPHDR_FIN | TCPHDR_PSH);
  TCP_SKB_CB(buff)->tcp_flags = flags;

  /* This packet was never sent out yet, so no SACK bits. */
  TCP_SKB_CB(buff)->sacked = 0;

  tcp_skb_fragment_eor(skb, buff);

  buff->ip_summed = CHECKSUM_PARTIAL;
  skb_split(skb, buff, len);
  tcp_fragment_tstamp(skb, buff);

  /* Fix up tso_factor for both original and new SKB.  */
  tcp_set_skb_tso_segs(skb, mss_now);
  tcp_set_skb_tso_segs(buff, mss_now);

  /* Link BUFF into the send queue. */
  __skb_header_release(buff);
  tcp_insert_write_queue_after(skb, buff, sk, tcp_queue);

  return 0;
}

void skb_split(struct sk_buff *skb, struct sk_buff *skb1, const u32 len)
{
  int pos = skb_headlen(skb); /* skb->len - skb->data_len */

  skb_shinfo(skb1)->tx_flags |= skb_shinfo(skb)->tx_flags & SKBTX_SHARED_FRAG;
  skb_zerocopy_clone(skb1, skb, 0);

  if (len < pos)   /* Split line is inside header. */
    skb_split_inside_header(skb, skb1, len, pos);
  else      /* Second chunk has no header, nothing to copy. */
    skb_split_no_header(skb, skb1, len, pos);
}

void skb_split_inside_header(struct sk_buff *skb,
  struct sk_buff* skb1, const u32 len, const int pos)
{
  int i;

  skb_copy_from_linear_data_offset(skb, len, skb_put(skb1, pos - len),
           pos - len);
  /* And move data appendix as is. */
  for (i = 0; i < skb_shinfo(skb)->nr_frags; i++)
    skb_shinfo(skb1)->frags[i] = skb_shinfo(skb)->frags[i];

  skb_shinfo(skb1)->nr_frags = skb_shinfo(skb)->nr_frags;
  skb_shinfo(skb)->nr_frags  = 0;
  skb1->data_len  = skb->data_len;
  skb1->len       += skb1->data_len;
  skb->data_len   = 0;
  skb->len        = len;
  skb_set_tail_pointer(skb, len); /* skb->data + offset */
}

void skb_split_no_header(
  struct sk_buff *skb, struct sk_buff* skb1,
  const u32 len, int pos)
{
  int i, k = 0;
  const int nfrags = skb_shinfo(skb)->nr_frags;

  skb_shinfo(skb)->nr_frags = 0;
  skb1->len       = skb1->data_len = skb->len - len;
  skb->len        = len;
  skb->data_len   = len - pos;

  for (i = 0; i < nfrags; i++) {
    int size = skb_frag_size(&skb_shinfo(skb)->frags[i]);
    if (pos + size > len) {
      skb_shinfo(skb1)->frags[k] = skb_shinfo(skb)->frags[i];

      if (pos < len) {
        /* Split frag.
         * We have two variants in this case:
         * 1. Move all the frag to the second
         *    part, if it is possible. F.e.
         *    this approach is mandatory for TUX,
         *    where splitting is expensive.
         * 2. Split is accurately. We make this. */
        skb_frag_ref(skb, i);
        skb_shinfo(skb1)->frags[0].page_offset += len - pos;
        skb_frag_size_sub(&skb_shinfo(skb1)->frags[0], len - pos);
        skb_frag_size_set(&skb_shinfo(skb)->frags[i], len - pos);
        skb_shinfo(skb)->nr_frags++;
      }
      k++;
    } else {
      skb_shinfo(skb)->nr_frags++;
    }

    pos += size;
  }
  skb_shinfo(skb1)->nr_frags = k;
}

void tcp_event_new_data_sent(struct sock *sk, struct sk_buff *skb)
{
  struct inet_connection_sock *icsk = inet_csk(sk);
  struct tcp_sock *tp = tcp_sk(sk);
  unsigned int prior_packets = tp->packets_out;

  tp->snd_nxt = TCP_SKB_CB(skb)->end_seq;

  __skb_unlink(skb, &sk->sk_write_queue);
  tcp_rbtree_insert(&sk->tcp_rtx_queue, skb);

  if (tp->highest_sack == NULL)
    tp->highest_sack = skb;

  tp->packets_out += tcp_skb_pcount(skb); /* skb_shinfo(skb)->gso_segs */
  if (!prior_packets || icsk->icsk_pending == ICSK_TIME_LOSS_PROBE)
    tcp_rearm_rto(sk);

  NET_ADD_STATS(sock_net(sk), LINUX_MIB_TCPORIGDATASENT,
          tcp_skb_pcount(skb));
}
```

### tcp_transmit_skb

```c
/* This routine actually transmits TCP packets queued in by
 * tcp_do_sendmsg().  This is used by both the initial
 * transmission and possible later retransmissions.
 * All SKB's seen here are completely headerless.  It is our
 * job to build the TCP header, and pass the packet down to
 * IP so it can do the same plus pass the packet off to the
 * device.
 *
 * We are working here with either a clone of the original
 * SKB, or a fresh unique copy made by the retransmit engine. */
int tcp_transmit_skb(struct sock *sk, struct sk_buff *skb,
  int clone_it, gfp_t gfp_mask, u32 rcv_nxt)
{
    const struct inet_connection_sock *icsk = inet_csk(sk);
    struct inet_sock *inet;
    struct tcp_sock *tp;
    struct tcp_skb_cb *tcb;
    struct tcp_out_options opts;
    unsigned int tcp_options_size, tcp_header_size;
    struct sk_buff *oskb = NULL;
    struct tcp_md5sig_key *md5;
    struct tcphdr *th;
    int err;

    tp = tcp_sk(sk);

    if (clone_it) {
        TCP_SKB_CB(skb)->tx.in_flight = TCP_SKB_CB(skb)->end_seq - tp->snd_una;
        oskb = skb;

        tcp_skb_tsorted_save(oskb) {
        if (unlikely(skb_cloned(oskb)))
            skb = pskb_copy(oskb, gfp_mask);
        else
            skb = skb_clone(oskb, gfp_mask);
        } tcp_skb_tsorted_restore(oskb);

        if (unlikely(!skb))
        return -ENOBUFS;
    }
    skb->skb_mstamp = tp->tcp_mstamp;

    inet = inet_sk(sk);
    tcb = TCP_SKB_CB(skb);
    memset(&opts, 0, sizeof(opts));

    if (unlikely(tcb->tcp_flags & TCPHDR_SYN)) {
        tcp_options_size = tcp_syn_options(sk, skb, &opts, &md5);
    } else {
        tcp_options_size = tcp_established_options(sk, skb, &opts, &md5);
        if (tcp_skb_pcount(skb) > 1)
            tcb->tcp_flags |= TCPHDR_PSH;
    }
    tcp_header_size = tcp_options_size + sizeof(struct tcphdr);

    skb->ooo_okay = sk_wmem_alloc_get(sk) < SKB_TRUESIZE(1);

    skb->pfmemalloc = 0;

    /* skb_push decrements the 'skb->data' pointer */
    skb_push(skb, tcp_header_size);
    skb_reset_transport_header(skb);

    skb_orphan(skb);
    skb->sk = sk;
    skb->destructor = skb_is_tcp_pure_ack(skb) ? __sock_wfree : tcp_wfree;
    skb_set_hash_from_sk(skb, sk);
    refcount_add(skb->truesize, &sk->sk_wmem_alloc);

    skb_set_dst_pending_confirm(skb, sk->sk_dst_pending_confirm);

/* Build TCP header and checksum it. */
    th = (struct tcphdr *)skb->data;
    th->source      = inet->inet_sport;
    th->dest        = inet->inet_dport;
    th->seq         = htonl(tcb->seq);
    th->ack_seq     = htonl(rcv_nxt);
    *(((__be16 *)th) + 6)  = htons(((tcp_header_size >> 2) << 12) | tcb->tcp_flags);

    th->check       = 0;
    th->urg_ptr     = 0;

    /* The urg_mode check is necessary during a below snd_una win probe */
    if (unlikely(tcp_urg_mode(tp) && before(tcb->seq, tp->snd_up))) {
        if (before(tp->snd_up, tcb->seq + 0x10000)) {
            th->urg_ptr = htons(tp->snd_up - tcb->seq);
            th->urg = 1;
        } else if (after(tcb->seq + 0xFFFF, tp->snd_nxt)) {
            th->urg_ptr = htons(0xFFFF);
            th->urg = 1;
        }
    }

    tcp_options_write((__be32 *)(th + 1), tp, &opts);
    skb_shinfo(skb)->gso_type = sk->sk_gso_type;
    if (likely(!(tcb->tcp_flags & TCPHDR_SYN))) {
        th->window = htons(tcp_select_window(sk));
        tcp_ecn_send(sk, skb, th, tcp_header_size);
    } else {
        /* RFC1323: The window in SYN & SYN/ACK segments
        * is never scaled. */
        th->window  = htons(min(tp->rcv_wnd, 65535U));
    }

    icsk->icsk_af_ops->send_check(sk, skb);

    if (likely(tcb->tcp_flags & TCPHDR_ACK))
        tcp_event_ack_sent(sk, tcp_skb_pcount(skb), rcv_nxt);

    if (skb->len != tcp_header_size) {
        tcp_event_data_sent(tp, sk);
        tp->data_segs_out += tcp_skb_pcount(skb);
        tp->bytes_sent += skb->len - tcp_header_size;
        tcp_internal_pacing(sk, skb);
    }

    if (after(tcb->end_seq, tp->snd_nxt) || tcb->seq == tcb->end_seq)
        TCP_ADD_STATS(sock_net(sk), TCP_MIB_OUTSEGS, tcp_skb_pcount(skb));

    tp->segs_out += tcp_skb_pcount(skb);
    /* OK, its time to fill skb_shinfo(skb)->gso_{segs|size} */
    skb_shinfo(skb)->gso_segs = tcp_skb_pcount(skb);
    skb_shinfo(skb)->gso_size = tcp_skb_mss(skb);

    /* Our usage of tstamp should remain private */
    skb->tstamp = 0;

    /* Cleanup our debris for IP stacks */
    memset(skb->cb, 0, max(sizeof(struct inet_skb_parm), sizeof(struct inet6_skb_parm)));

    err = icsk->icsk_af_ops->queue_xmit(sk, skb, &inet->cork.fl);

    if (unlikely(err > 0)) {
        tcp_enter_cwr(sk);
        err = net_xmit_eval(err);
    }
    if (!err && oskb) {
        tcp_update_skb_after_send(tp, oskb);
        tcp_rate_skb_sent(sk, oskb);
    }
    return err;
}
```

## ip layer tx

```sh
 0                   1                   2                   3
 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|Version|  IHL  |Type of Service|          Total Length         |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|         Identification        |Flags|      Fragment Offset    |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|  Time to Live |    Protocol   |         Header Checksum       |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                       Source Address                          |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                    Destination Address                        |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                    Options (if IHL > 5)                       |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
```

* **Version** (4 bits)
    * Indicates the IP version. For IPv4, this is 4.
    * Example: 0100 in binary.

* **IHL (Internet Header Length)** (4 bits)
    * Specifies the length of the header in 32-bit words (4 bytes).
    * Minimum value is 5 (20 bytes, no options); maximum is 15 (60 bytes with options).
    * Example: 0101 (5 words = 20 bytes).

* **Type of Service (ToS)** (8 bits)
    * Historically used for Quality of Service (QoS) to indicate priority or handling (e.g., delay, throughput). Now often reinterpreted as DSCP (6 bits) and ECN (2 bits).
    * Example: 00000000 (default).

* **Total Length** (16 bits)
    * The total size of the IP packet (header + data) in bytes.
    * Range: 20 bytes (header only) to 65,535 bytes (max packet size).
    * Example: 00000000 01000000 (64 bytes).

* **Identification** (16 bits)
    * A unique identifier for the packet, used to reassemble fragments if the packet is split.
    * Example: 00000000 00000001 (ID 1).

* **Flags** (3 bits)
    * Controls fragmentation:
    * Bit 0: Reserved (always 0).
    * Bit 1: DF (Don’t Fragment) - 1 means no fragmentation allowed.
    * Bit 2: MF (More Fragments) - 1 means more fragments follow.
    * Example: 010 (DF set, no more fragments).

* **Fragment Offset** (13 bits)
    * Indicates the position of this fragment in the original packet, measured in 8-byte units.
    * Example: 0000000000000 (first fragment or unfragmented).

* **Time to Live (TTL)** (8 bits)
    * Limits the packet’s lifetime to prevent infinite looping; decremented by each router. Packet is discarded if TTL reaches 0.
    * Range: 0 to 255 hops.
    * Example: 01000000 (64 hops, a common default).

* **Protocol** (8 bits)
    * Identifies the next-level protocol encapsulated in the packet (e.g., TCP = 6, UDP = 17).
    * Example: 00000110 (TCP).
    * See IANA protocol numbers for a full list.

* **Header Checksum** (16 bits)
    * A checksum to verify the integrity of the IP header (not the data). Recalculated at each hop since TTL changes.
    * Example: 10101010 11001100 (varies).

* **Source Address** (32 bits)
    * The IPv4 address of the sender.

    * Example: 192.168.1.100 (11000000 10101000 00000001 01100100).

* **Destination Address** (32 bits)
    * The IPv4 address of the recipient.
    * Example: 8.8.8.8 (00001000 00001000 00001000 00001000).

* **Options** (variable, 0-40 bytes)
    * Optional field for additional features (e.g., source routing). Present only if IHL > 5. Rarely used due to processing overhead.
    * Padded to align with 32-bit boundaries.



```c
const struct inet_connection_sock_af_ops ipv4_specific = {
    .queue_xmit        = ip_queue_xmit,
    .send_check        = tcp_v4_send_check,
    .rebuild_header    = inet_sk_rebuild_header,
    .sk_rx_dst_set     = inet_sk_rx_dst_set,
    .conn_request      = tcp_v4_conn_request,
    .syn_recv_sock     = tcp_v4_syn_recv_sock,
    .net_header_len    = sizeof(struct iphdr),
    .setsockopt        = ip_setsockopt,
    .getsockopt        = ip_getsockopt,
    .addr2sockaddr     = inet_csk_addr2sockaddr,
    .sockaddr_len      = sizeof(struct sockaddr_in),
    .mtu_reduced       = tcp_v4_mtu_reduced,
};

/* 1. select route
 * 2. build IP header
 * 3. send packet */
int ip_queue_xmit(struct sock *sk, struct sk_buff *skb, struct flowi *fl)
{
    struct inet_sock *inet = inet_sk(sk);
    struct net *net = sock_net(sk);
    struct ip_options_rcu *inet_opt;
    struct flowi4 *fl4;
    struct rtable *rt;
    struct iphdr *iph;
    int res;

/* 1. select route */
    inet_opt = rcu_dereference(inet->inet_opt);
    fl4 = &fl->u.ip4;
    rt = skb_rtable(skb);
    if (rt) {
        goto packet_routed;
    }

    /* Make sure we can route this packet. */
    rt = (struct rtable *)__sk_dst_check(sk, 0);
    if (!rt) {
        __be32 daddr;
        /* Use correct destination address if we have options. */
        daddr = inet->inet_daddr;
        rt = ip_route_output_ports(net, fl4, sk,
                        daddr, inet->inet_saddr,
                        inet->inet_dport,
                        inet->inet_sport,
                        sk->sk_protocol,
                        RT_CONN_FLAGS(sk),
                        sk->sk_bound_dev_if);
        if (IS_ERR(rt))
            goto no_route;
        sk_setup_caps(sk, &rt->dst);
    }
    skb_dst_set_noref(skb, &rt->dst);

/* 2. build IP header */
packet_routed:
    /* OK, we know where to send it, allocate and build IP header.
     * skb_push decrements the 'skb->data' pointer */
    skb_push(skb, sizeof(struct iphdr) + (inet_opt ? inet_opt->opt.optlen : 0));
    skb_reset_network_header(skb);
    iph               = ip_hdr(skb);
    *((__be16 *)iph)  = htons((4 << 12) | (5 << 8) | (inet->tos & 0xff));
    iph->frag_off     = (ip_dont_fragment(sk, &rt->dst) && !skb->ignore_df) ? htons(IP_DF) : 0;
    iph->ttl          = ip_select_ttl(inet, &rt->dst);
    iph->protocol     = sk->sk_protocol;
    ip_copy_addrs(iph, fl4);

    /* Transport layer set skb->h.foo itself. */
    if (inet_opt && inet_opt->opt.optlen) {
        iph->ihl += inet_opt->opt.optlen >> 2;
        ip_options_build(skb, &inet_opt->opt, inet->inet_daddr, rt, 0);
    }

    ip_select_ident_segs(net, skb, sk, skb_shinfo(skb)->gso_segs ?: 1);

    /* TODO : should we use skb->sk here instead of sk ? */
    skb->priority = sk->sk_priority;
    skb->mark = sk->sk_mark;

/* 3. send packet */
    res = ip_local_out(net, sk, skb);
}
```

### build ip header

<img src='../images/kernel/net-ip-header.png' style='max-height:850px'/>

### send package

```c
int ip_local_out(
    struct net *net, struct sock *sk, struct sk_buff *skb)
{
    int err;

    err = __ip_local_out(net, sk, skb);
    if (likely(err == 1))
        err = dst_output(net, sk, skb);

    return err;
}

int __ip_local_out(struct net *net, struct sock *sk, struct sk_buff *skb)
{
    struct iphdr *iph = ip_hdr(skb);

    IP_INC_STATS(net, IPSTATS_MIB_OUTREQUESTS);

    iph_set_totlen(iph, skb->len) {
        iph->tot_len = len <= IP_MAX_MTU ? htons(len) : 0;
    }
    ip_send_check(iph) {
        iph->check = 0;
        iph->check = ip_fast_csum((unsigned char *)iph, iph->ihl);
    }

    /* if egress device is enslaved to an L3 master device pass the
    * skb to its handler for processing */
    skb = l3mdev_ip_out(sk, skb);
    if (unlikely(!skb))
        return 0;

    skb->protocol = htons(ETH_P_IP);

    return nf_hook(NFPROTO_IPV4, NF_INET_LOCAL_OUT,
               net, sk, skb, NULL, skb_dst(skb)->dev,
               dst_output);
}

int dst_output(struct net *net, struct sock *sk, struct sk_buff *skb)
{
    return skb_dst(skb)->output(net, sk, skb);
}

/* ipv4_dst_ops.output */
int ip_output(struct net *net, struct sock *sk, struct sk_buff *skb)
{
    struct net_device *dev = skb_dst(skb)->dev;
    skb->dev = dev;
    skb->protocol = htons(ETH_P_IP);

    return NF_HOOK_COND(NFPROTO_IPV4, NF_INET_POST_ROUTING,
        net, sk, skb, NULL, dev,
        ip_finish_output,
        !(IPCB(skb)->flags & IPSKB_REROUTED));
}

int ip_finish_output(struct net *net, struct sock *sk, struct sk_buff *skb)
{
    unsigned int mtu;
    int ret;

    ret = BPF_CGROUP_RUN_PROG_INET_EGRESS(sk, skb);
    if (ret) {
        kfree_skb(skb);
        return ret;
    }

    #if defined(CONFIG_NETFILTER) && defined(CONFIG_XFRM)
    /* Policy lookup after SNAT yielded a new policy */
    if (skb_dst(skb)->xfrm) {
        IPCB(skb)->flags |= IPSKB_REROUTED;
        return dst_output(net, sk, skb);
    }
    #endif
    mtu = ip_skb_dst_mtu(sk, skb);
    if (skb_is_gso(skb)) /* support gso: skb_shinfo(skb)->gso_size */
        return ip_finish_output_gso(net, sk, skb, mtu);

    if (skb->len > mtu || IPCB(skb)->frag_max_size)
        return ip_fragment(net, sk, skb, mtu, ip_finish_output2);

    return ip_finish_output2(net, sk, skb);
}
```

### skb_gso_segment

```c
int ip_finish_output_gso(struct net *net, struct sock *sk,
        struct sk_buff *skb, unsigned int mtu)
{
    netdev_features_t features;
    struct sk_buff *segs;
    int ret = 0;

    /* common case: seglen is <= mtu */
    if (skb_gso_validate_network_len(skb, mtu)) {
        return ip_finish_output2(net, sk, skb);
    }

    features = netif_skb_features(skb);
    BUILD_BUG_ON(sizeof(*IPCB(skb)) > SKB_SGO_CB_OFFSET);
    segs = skb_gso_segment(skb, features & ~NETIF_F_GSO_MASK);
    if (IS_ERR_OR_NULL(segs)) {
        kfree_skb(skb);
        return -ENOMEM;
    }

    consume_skb(skb);

    do {
        struct sk_buff *nskb = segs->next;
        int err;

        segs->next = NULL;
        skb_mark_not_on_list(segs);
        err = ip_fragment(net, sk, segs, mtu, ip_finish_output2);

        if (err && ret == 0) {
            ret = err;
        }
        segs = nskb;
    } while (segs);

    return ret;
}

static inline
struct sk_buff *skb_gso_segment(struct sk_buff *skb, netdev_features_t features)
{
    return __skb_gso_segment(skb, features, true);
}

struct sk_buff *__skb_gso_segment(struct sk_buff *skb,
          netdev_features_t features, bool tx_path)
{
    struct sk_buff *segs;

    if (unlikely(skb_needs_check(skb, tx_path))) {
        int err;

        /* We're going to init ->check field in TCP or UDP header */
        err = skb_cow_head(skb, 0);
        if (err < 0)
            return ERR_PTR(err);
    }

    /* Only report GSO partial support if it will enable us to
    * support segmentation on this frame without needing additional
    * work. */
    if (features & NETIF_F_GSO_PARTIAL) {
        netdev_features_t partial_features = NETIF_F_GSO_ROBUST;
        struct net_device *dev = skb->dev;

        partial_features |= dev->features & dev->gso_partial_features;
        if (!skb_gso_ok(skb, features | partial_features))
        features &= ~NETIF_F_GSO_PARTIAL;
    }

    BUILD_BUG_ON(SKB_SGO_CB_OFFSET + sizeof(*SKB_GSO_CB(skb)) > sizeof(skb->cb));

    SKB_GSO_CB(skb)->mac_offset = skb_headroom(skb);
    SKB_GSO_CB(skb)->encap_level = 0;

    skb_reset_mac_header(skb);
    skb_reset_mac_len(skb);

    segs = skb_mac_gso_segment(skb, features);

    if (unlikely(skb_needs_check(skb, tx_path) && !IS_ERR(segs)))
        skb_warn_bad_offload(skb);

    return segs;
}
```

#### skb_mac_gso_segment

```c
struct sk_buff *skb_mac_gso_segment(struct sk_buff *skb,
            netdev_features_t features)
{
    struct sk_buff *segs = ERR_PTR(-EPROTONOSUPPORT);
    struct packet_offload *ptype;
    int vlan_depth = skb->mac_len;
    __be16 type = skb_network_protocol(skb, &vlan_depth);

    if (unlikely(!type))
        return ERR_PTR(-EINVAL);

    __skb_pull(skb, vlan_depth);

    rcu_read_lock();
    list_for_each_entry_rcu(ptype, &offload_base, list) {
        if (ptype->type == type && ptype->callbacks.gso_segment) {
            segs = ptype->callbacks.gso_segment(skb, features);
            break;
        }
    }
    rcu_read_unlock();

    __skb_push(skb, skb->data - skb_mac_header(skb));

    return segs;
}
```

#### inet_gso_segment

```c
struct sk_buff *inet_gso_segment(struct sk_buff *skb,
    netdev_features_t features)
{
  bool udpfrag = false, fixedid = false, gso_partial, encap;
  struct sk_buff *segs = ERR_PTR(-EINVAL);
  const struct net_offload *ops;
  unsigned int offset = 0;
  struct iphdr *iph;
  int proto, tot_len;
  int nhoff;
  int ihl;
  int id;

  skb_reset_network_header(skb);
  nhoff = skb_network_header(skb) - skb_mac_header(skb);
  if (unlikely(!pskb_may_pull(skb, sizeof(*iph))))
    goto out;

  iph = ip_hdr(skb);
  ihl = iph->ihl * 4;
  if (ihl < sizeof(*iph))
    goto out;

  id = ntohs(iph->id);
  proto = iph->protocol;

  /* Warning: after this point, iph might be no longer valid */
  if (unlikely(!pskb_may_pull(skb, ihl)))
    goto out;
  __skb_pull(skb, ihl);

  encap = SKB_GSO_CB(skb)->encap_level > 0;
  if (encap)
    features &= skb->dev->hw_enc_features;
  SKB_GSO_CB(skb)->encap_level += ihl;

  skb_reset_transport_header(skb);

  segs = ERR_PTR(-EPROTONOSUPPORT);

  if (!skb->encapsulation || encap) {
    udpfrag = !!(skb_shinfo(skb)->gso_type & SKB_GSO_UDP);
    fixedid = !!(skb_shinfo(skb)->gso_type & SKB_GSO_TCP_FIXEDID);

    /* fixed ID is invalid if DF bit is not set */
    if (fixedid && !(ip_hdr(skb)->frag_off & htons(IP_DF)))
      goto out;
  }

  ops = rcu_dereference(inet_offloads[proto]);
  if (likely(ops && ops->callbacks.gso_segment))
    segs = ops->callbacks.gso_segment(skb, features); /* tcp4_gso_segment */

  if (IS_ERR_OR_NULL(segs))
    goto out;

  gso_partial = !!(skb_shinfo(segs)->gso_type & SKB_GSO_PARTIAL);

  skb = segs;
  do {
    iph = (struct iphdr *)(skb_mac_header(skb) + nhoff);
    if (udpfrag) {
      iph->frag_off = htons(offset >> 3);
      if (skb->next)
        iph->frag_off |= htons(IP_MF);
      offset += skb->len - nhoff - ihl;
      tot_len = skb->len - nhoff;
    } else if (skb_is_gso(skb)) {
      if (!fixedid) {
        iph->id = htons(id);
        id += skb_shinfo(skb)->gso_segs;
      }

      if (gso_partial)
        tot_len = skb_shinfo(skb)->gso_size +
            SKB_GSO_CB(skb)->data_offset +
            skb->head - (unsigned char *)iph;
      else
        tot_len = skb->len - nhoff;
    } else {
      if (!fixedid)
        iph->id = htons(id++);
      tot_len = skb->len - nhoff;
    }
    iph->tot_len = htons(tot_len);
    ip_send_check(iph);
    if (encap)
      skb_reset_inner_headers(skb);
    skb->network_header = (u8 *)iph - skb->head;
    skb_reset_mac_len(skb);
  } while ((skb = skb->next));

out:
  return segs;
}
```

#### tcp4_gso_segment

```c
struct sk_buff *tcp4_gso_segment(struct sk_buff *skb,
          netdev_features_t features)
{
    if (!(skb_shinfo(skb)->gso_type & SKB_GSO_TCPV4))
        return ERR_PTR(-EINVAL);

    if (!pskb_may_pull(skb, sizeof(struct tcphdr)))
        return ERR_PTR(-EINVAL);

    if (unlikely(skb->ip_summed != CHECKSUM_PARTIAL)) {
        const struct iphdr *iph = ip_hdr(skb);
        struct tcphdr *th = tcp_hdr(skb);

        /* Set up checksum pseudo header, usually expect stack to
        * have done this already. */

        th->check = 0;
        skb->ip_summed = CHECKSUM_PARTIAL;
        __tcp_v4_send_check(skb, iph->saddr, iph->daddr);
    }

    return tcp_gso_segment(skb, features);
}
```

### ip_fragment

## mac layer tx

```c
/* ip_finish_output -> */
int ip_finish_output2(struct net *net, struct sock *sk, struct sk_buff *skb)
{
    struct dst_entry *dst = skb_dst(skb);
    struct rtable *rt = (struct rtable *)dst;
    struct net_device *dev = dst->dev;
    unsigned int hh_len = LL_RESERVED_SPACE(dev);
    struct neighbour *neigh;
    u32 nexthop;

    nexthop = (__force u32) rt_nexthop(rt, ip_hdr(skb)->daddr);
    neigh = __ipv4_neigh_lookup_noref(dev, nexthop);
    if (unlikely(!neigh))
        neigh = __neigh_create(&arp_tbl, &nexthop, dev, false);
    if (!IS_ERR(neigh)) {
        int res;
        sock_confirm_neigh(skb, neigh);
        res = neigh_output(neigh, skb);
        return res;
    }
}

struct neighbour *__ipv4_neigh_lookup_noref(
  struct net_device *dev, u32 key)
{
    return ___neigh_lookup_noref(&arp_tbl, neigh_key_eq32, arp_hashfn, &key, dev);
}

struct neigh_table arp_tbl = {
    .family       = AF_INET,
    .key_len      = 4,
    .protocol     = cpu_to_be16(ETH_P_IP),
    .hash         = arp_hash,
    .key_eq       = arp_key_eq,
    .constructor  = arp_constructor,
    .proxy_redo   = parp_redo,
    .id           = "arp_cache",

    .gc_interval  = 30 * HZ,
    .gc_thresh1   = 128,
    .gc_thresh2   = 512,
    .gc_thresh3   = 1024,
};
```

### neighbour

```c
struct neighbour *__neigh_create(struct neigh_table *tbl,
  const void *pkey, struct net_device *dev, bool want_ref)
{
    u32 hash_val;
    int key_len = tbl->key_len;
    int error;
    struct neighbour *n1, *rc, *n = neigh_alloc(tbl, dev);
    struct neigh_hash_table *nht;

    memcpy(n->primary_key, pkey, key_len);
    n->dev = dev;
    dev_hold(dev);

    /* Protocol specific setup. */
    if (tbl->constructor && (error = tbl->constructor(n)) < 0) {

    }

    if (atomic_read(&tbl->entries) > (1 << nht->hash_shift))
        nht = neigh_hash_grow(tbl, nht->hash_shift + 1);

    hash_val = tbl->hash(pkey, dev, nht->hash_rnd) >> (32 - nht->hash_shift);

    for (n1 = rcu_dereference_protected(nht->hash_buckets[hash_val], lockdep_is_held(&tbl->lock));
         n1 != NULL;
         n1 = rcu_dereference_protected(n1->next, lockdep_is_held(&tbl->lock))) {
        if (dev == n1->dev && !memcmp(n1->primary_key, pkey, key_len)) {
            if (want_ref)
                neigh_hold(n1);
            rc = n1;
            goto out_tbl_unlock;
        }
    }

    rcu_assign_pointer(n->next,
      rcu_dereference_protected(nht->hash_buckets[hash_val],
        lockdep_is_held(&tbl->lock)));
    rcu_assign_pointer(nht->hash_buckets[hash_val], n);
}

struct neighbour *neigh_alloc(struct neigh_table *tbl, struct net_device *dev)
{
    struct neighbour *n = NULL;
    unsigned long now = jiffies;
    int entries;

    n = kzalloc(tbl->entry_size + dev->neigh_priv_len, GFP_ATOMIC);
    if (!n)
        goto out_entries;

    __skb_queue_head_init(&n->arp_queue);
    rwlock_init(&n->lock);
    seqlock_init(&n->ha_lock);
    n->updated    = n->used = now;
    n->nud_state    = NUD_NONE;
    n->output    = neigh_blackhole;
    seqlock_init(&n->hh.hh_lock);
    n->parms    = neigh_parms_clone(&tbl->parms);
    setup_timer(&n->timer, neigh_timer_handler, (unsigned long)n);

    NEIGH_CACHE_STAT_INC(tbl, allocs);
    n->tbl      = tbl;
    refcount_set(&n->refcnt, 1);
    n->dead      = 1;
}

/* __neigh_create -> arp_tbl.constructor -> */
int arp_constructor(struct neighbour *neigh)
{
  __be32 addr = *(__be32 *)neigh->primary_key;
  struct net_device *dev = neigh->dev;
  struct in_device *in_dev;
  struct neigh_parms *parms;

  neigh->type = inet_addr_type_dev_table(dev_net(dev), dev, addr);

  parms = in_dev->arp_parms;
  __neigh_parms_put(neigh->parms);
  neigh->parms = neigh_parms_clone(parms);

  neigh->ops = &arp_hh_ops;

  neigh->output = neigh->ops->output;
}

const struct neigh_ops arp_hh_ops = {
  .family             = AF_INET,
  .solicit            = arp_solicit,
  .error_report       = arp_error_report,
  .output             = neigh_resolve_output,
  .connected_output   = neigh_resolve_output,
};
```

### neigh_output

```c
int neigh_output(struct neighbour *n, struct sk_buff *skb)
{
    return n->output(n, skb);
}
/* arp_hh_ops.output -> */
int neigh_resolve_output(struct neighbour *neigh, struct sk_buff *skb)
{
    /* send arp packet */
    if (!neigh_event_send(neigh, skb)) {
        /* send data */
        rc = dev_queue_xmit(skb);
    }
}

int neigh_event_send(struct neighbour *neigh, struct sk_buff *skb)
{
  unsigned long now = jiffies;

  if (READ_ONCE(neigh->used) != now)
    WRITE_ONCE(neigh->used, now);
  if (!(neigh->nud_state & (NUD_CONNECTED|NUD_DELAY|NUD_PROBE)))
    return __neigh_event_send(neigh, skb);
  return 0;
}

int __neigh_event_send(struct neighbour *neigh, struct sk_buff *skb)
{
  int rc;
  bool immediate_probe = false;

  write_lock_bh(&neigh->lock);

  rc = 0;
  if (neigh->nud_state & (NUD_CONNECTED | NUD_DELAY | NUD_PROBE))
    goto out_unlock_bh;
  if (neigh->dead)
    goto out_dead;

  if (!(neigh->nud_state & (NUD_STALE | NUD_INCOMPLETE))) {
    if (NEIGH_VAR(neigh->parms, MCAST_PROBES) + NEIGH_VAR(neigh->parms, APP_PROBES)) {
      unsigned long next, now = jiffies;

      atomic_set(&neigh->probes, NEIGH_VAR(neigh->parms, UCAST_PROBES));
      neigh_del_timer(neigh);
      neigh->nud_state  = NUD_INCOMPLETE;
      neigh->updated    = now;
      next = now + max(NEIGH_VAR(neigh->parms, RETRANS_TIME), HZ/2);
      neigh_add_timer(neigh, next);
      immediate_probe = true;
    } else {
      neigh->nud_state = NUD_FAILED;
      neigh->updated = jiffies;
      write_unlock_bh(&neigh->lock);

      kfree_skb(skb);
      return 1;
    }
  } else if (neigh->nud_state & NUD_STALE) {
    neigh_dbg(2, "neigh %p is delayed\n", neigh);
    neigh_del_timer(neigh);
    neigh->nud_state = NUD_DELAY;
    neigh->updated = jiffies;
    neigh_add_timer(neigh, jiffies + NEIGH_VAR(neigh->parms, DELAY_PROBE_TIME));
  }

  if (neigh->nud_state == NUD_INCOMPLETE) {
    if (skb) {
      while (neigh->arp_queue_len_bytes + skb->truesize > NEIGH_VAR(neigh->parms, QUEUE_LEN_BYTES)) {
        struct sk_buff *buff;

        buff = __skb_dequeue(&neigh->arp_queue);
        if (!buff)
          break;
        neigh->arp_queue_len_bytes -= buff->truesize;
        kfree_skb(buff);
        NEIGH_CACHE_STAT_INC(neigh->tbl, unres_discards);
      }
      skb_dst_force(skb);
      /* skb is dequed in arp_rcv -> neigh_update */
      __skb_queue_tail(&neigh->arp_queue, skb);
      neigh->arp_queue_len_bytes += skb->truesize;
    }
    rc = 1;
  }

out_unlock_bh:
  if (immediate_probe)
    neigh_probe(neigh);
  else
    write_unlock(&neigh->lock);
  local_bh_enable();
  return rc;

out_dead:
  if (neigh->nud_state & NUD_STALE)
    goto out_unlock_bh;
  write_unlock_bh(&neigh->lock);
  kfree_skb(skb);
  return 1;
}

void neigh_probe(struct neighbour *neigh)
{
  struct sk_buff *skb = skb_peek_tail(&neigh->arp_queue);

  if (neigh->ops->solicit)
    neigh->ops->solicit(neigh, skb);
}
/* arp_solicit -> */
void arp_send_dst(
  int type, int ptype, __be32 dest_ip,
  struct net_device *dev, __be32 src_ip,
  const unsigned char *dest_hw,
  const unsigned char *src_hw,
  const unsigned char *target_hw,
  struct dst_entry *dst)
{
  struct sk_buff *skb;

  skb = arp_create(type, ptype, dest_ip, dev, src_ip, dest_hw, src_hw, target_hw);

  skb_dst_set(skb, dst_clone(dst));
  arp_xmit(skb);
}
```

## dev layer tx

* [xps: Transmit Packet Steering](https://lwn.net/Articles/412062/)
* [Queueing in the Linux Network Stack](https://www.coverfire.com/articles/queueing-in-the-linux-network-stack/)
* [Controlling Queue Delay - A modern AQM is just one piece of the solution to bufferbloat.](https://queue.acm.org/detail.cfm?id=2209336)
* [Bufferbloat: Dark Buffers in the Internet](http://cacm.acm.org/magazines/2012/1/144810-bufferbloat/fulltext)
* [Linux Advanced Routing and Traffic Control Howto (LARTC) ](http://www.lartc.org/howto/)
* [LWN - TCP Small Queues](http://lwn.net/Articles/507065/)
  * It limits the amount of data that can be queued for transmission by any given socket regardless of where the data is queued, so it shouldn't be fooled by buffers lurking in the queueing, traffic control, or netfilter code. That limit is set by a new sysctl knob found at:
    > /proc/sys/net/ipv4/tcp_limit_output_bytes

* [LWN - Byte Queue Limits](http://lwn.net/Articles/454390/)
  * Byte queue limits work only at the device queue level

* If the NIC driver wakes to pull packets off of the queue for transmission and the queue is empty the hardware will miss a transmission opportunity thereby reducing the throughput of the system. This is referred to as **starvation**.
    * Note that an empty queue when the system does not have anything to transmit is not starvation – this is normal.
* While a large queue is necessary for a busy system to maintain high throughput, it has the downside of allowing for the introduction of a large amount of **latency**.

<img src='../images/kernel/net-queue-displine.png' style='max-height:850px'/>

<img src='../images/kernel/net-dev-pci.svg' style='max-height:850px'/>

```c
struct net_device {
    struct netdev_rx_queue  *_rx;
    struct netdev_queue     *_tx;
};

struct netdev_queue {
    struct net_device   *dev;
    struct Qdisc        *qdisc;
    struct Qdisc        *qdisc_sleeping;

    unsigned long       tx_maxrate;
    unsigned long       trans_timeout;

    /* Subordinate device that the queue has been assigned to */
    struct net_device  *sb_dev;

    /* Time (in jiffies) of last Tx */
    unsigned long       trans_start;

    unsigned long       state;
};

struct Qdisc {
    int (*enqueue)(struct sk_buff *skb,struct Qdisc *sch, struct sk_buff **to_free);
    struct sk_buff* (*dequeue)(struct Qdisc *sch);

    const struct Qdisc_ops  *ops;
    struct qdisc_skb_head   q;
    struct netdev_queue     *dev_queue;
    long                    privdata[];
};

struct qdisc_skb_head {
    struct sk_buff  *head;
    struct sk_buff  *tail;
};

struct Qdisc_ops {
    const struct Qdisc_class_ops  *cl_ops;
    char                        id[IFNAMSIZ];
    int                         priv_size;
    unsigned int                static_flags;

    int (*init)(struct Qdisc *sch, struct nlattr *arg, struct netlink_ext_ack *extack);
    int (*enqueue)(struct sk_buff *skb, struct Qdisc *sch, struct sk_buff **to_free);
    struct sk_buff*  (*dequeue)(struct Qdisc*);
    struct sk_buff*  (*peek)(struct Qdisc*);
};

/* dev_queue_xmit -> */
int __dev_queue_xmit(struct sk_buff *skb, void *accel_priv)
{
    struct net_device *dev = skb->dev;
    struct netdev_queue *txq;
    struct Qdisc *q; /* queueing discipline */

    txq = netdev_pick_tx(dev, skb, accel_priv);
    q = rcu_dereference_bh(txq->qdisc);

    if (q->enqueue) {
        rc = __dev_xmit_skb(skb, q, dev, txq);
        goto out;
    }
}

u16 netdev_pick_tx(struct net_device *dev, struct sk_buff *skb,
         struct net_device *sb_dev)
{
    struct sock *sk = skb->sk;
    int queue_index = sk_tx_queue_get(sk);

    sb_dev = sb_dev ? : dev;

    if (queue_index < 0 || skb->ooo_okay || queue_index >= dev->real_num_tx_queues) {
        /* Transmit Packet steering (XPS), Receive Packet steering(RPS) */
        int new_index = get_xps_queue(dev, sb_dev, skb);

        if (new_index < 0)
            new_index = skb_tx_hash(dev, sb_dev, skb);

        if (queue_index != new_index && sk &&
            sk_fullsock(sk) && rcu_access_pointer(sk->sk_dst_cache)) {

            sk_tx_queue_set(sk, new_index);
        }

        queue_index = new_index;
    }

    return queue_index;
}

# ip addr
1: lo: <LOOPBACK,UP,LOWER_UP> mtu 65536 qdisc noqueue state UNKNOWN group default qlen 1000
    link/loopback 00:00:00:00:00:00 brd 00:00:00:00:00:00
    inet 127.0.0.1/8 scope host lo
       valid_lft forever preferred_lft forever
    inet6 ::1/128 scope host
       valid_lft forever preferred_lft forever
2: eth0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1400 qdisc pfifo_fast state UP group default qlen 1000
    link/ether fa:16:3e:75:99:08 brd ff:ff:ff:ff:ff:ff
    inet 10.173.32.47/21 brd 10.173.39.255 scope global noprefixroute dynamic eth0
       valid_lft 67104sec preferred_lft 67104sec
    inet6 fe80::f816:3eff:fe75:9908/64 scope link
       valid_lft forever preferred_lft forever
```

```c
int __dev_xmit_skb(
  struct sk_buff *skb,
  struct Qdisc *q,
  struct net_device *dev,
  struct netdev_queue *txq)
{
    rc = q->enqueue(skb, q, &to_free) & NET_XMIT_MASK; /* pfifo_enqueue */
    if (qdisc_run_begin(q)) {
        __qdisc_run(q);
    }
}

bool qdisc_run_begin(struct Qdisc *qdisc)
{
    if (qdisc->flags & TCQ_F_NOLOCK) {
        if (!spin_trylock(&qdisc->seqlock))
        return false;
    } else if (qdisc_is_running(qdisc)) {
        return false;
    }
    /* Variant of write_seqcount_begin() telling lockdep a trylock
    * was attempted. */
    raw_write_seqcount_begin(&qdisc->running);
    seqcount_acquire(&qdisc->running.dep_map, 0, 1, _RET_IP_);
    return true;
}

void __qdisc_run(struct Qdisc *q)
{
    int quota = dev_tx_weight; /* 64 */
    int packets;
    while (qdisc_restart(q, &packets)) {
        /* Ordered by possible occurrence: Postpone processing if
        * 1. we've exceeded packet quota
        * 2. another process needs the CPU */
        quota -= packets;
        /* control transmit speed
        * return unlikely(tif_need_resched()); */
        if (quota <= 0 || need_resched()) {
            __netif_schedule(q);
            break;
        }
    }
    qdisc_run_end(q);
}

/* net_tx_action -> qdisc_run -> __qdisc_run -> */
int qdisc_restart(struct Qdisc *q, int *packets)
{
    struct netdev_queue *txq;
    struct net_device *dev;
    spinlock_t *root_lock;
    struct sk_buff *skb;
    bool validate;

    /* Dequeue packet */
    skb = dequeue_skb(q, &validate, packets);
    if (unlikely(!skb))
        return 0;

    root_lock = qdisc_lock(q);
    dev = qdisc_dev(q);
    txq = skb_get_tx_queue(dev, skb);

    return sch_direct_xmit(skb, q, dev, txq, root_lock, validate);
}

int sch_direct_xmit(
  struct sk_buff *skb, struct Qdisc *q,
  struct net_device *dev, struct netdev_queue *txq,
  spinlock_t *root_lock, bool validate)
{
    int ret = NETDEV_TX_BUSY;

    if (likely(skb)) {
        if (!netif_xmit_frozen_or_stopped(txq))
            skb = dev_hard_start_xmit(skb, dev, txq, &ret);
    }

    if (dev_xmit_complete(ret)) {
        /* Driver sent out skb successfully or skb was consumed */
        ret = qdisc_qlen(q);
    } else {
        /* Driver returned NETDEV_TX_BUSY - requeue skb */
        ret = dev_requeue_skb(skb, q);
    }
}

struct sk_buff *dev_hard_start_xmit(
  struct sk_buff *first,
  struct net_device *dev,
  struct netdev_queue *txq,
  int *ret)
{
  struct sk_buff *skb = first;
  int rc = NETDEV_TX_OK;

  while (skb) {
      struct sk_buff *next = skb->next;
      rc = xmit_one(skb, dev, txq, next != NULL);
      skb = next;
      if (netif_xmit_stopped(txq) && skb) {
          rc = NETDEV_TX_BUSY;
          break;
      }
  }
}

int xmit_one(struct sk_buff *skb, struct net_device *dev,
        struct netdev_queue *txq, bool more)
{
  unsigned int len;
  int rc;

  if (!list_empty(&ptype_all) || !list_empty(&dev->ptype_all))
    dev_queue_xmit_nit(skb, dev);

  len = skb->len;
  trace_net_dev_start_xmit(skb, dev);
  rc = netdev_start_xmit(skb, dev, txq, more);
  trace_net_dev_xmit(skb, rc, dev, len);

  return rc;
}

/* netdev_start_xmit -> __netdev_start_xmit */
netdev_tx_t __netdev_start_xmit(
  const struct net_device_ops *ops,
  struct sk_buff *skb,
  struct net_device *dev, bool more)
{
    skb->xmit_more = more ? 1 : 0;
    return ops->ndo_start_xmit(skb, dev);
}
/* .ndo_start_xmit = ixgb_xmit_frame */
netdev_tx_t
ixgb_xmit_frame(struct sk_buff *skb, struct net_device *netdev)
{

}

void __netif_schedule(struct Qdisc *q)
{
  if (!test_and_set_bit(__QDISC_STATE_SCHED, &q->state))
    __netif_reschedule(q);
}

void __netif_reschedule(struct Qdisc *q)
{
  struct softnet_data *sd;
  unsigned long flags;

  local_irq_save(flags);
  sd = this_cpu_ptr(&softnet_data);
  q->next_sched = NULL;
  *sd->output_queue_tailp = q;
  sd->output_queue_tailp = &q->next_sched;
  raise_softirq_irqoff(NET_TX_SOFTIRQ);
  local_irq_restore(flags);
}

/* register soft irq when boot */
open_softirq(NET_TX_SOFTIRQ, net_tx_action); /* snd */
open_softirq(NET_RX_SOFTIRQ, net_rx_action); /* rcv */

void net_tx_action(struct softirq_action *h)
{
  struct softnet_data *sd = this_cpu_ptr(&softnet_data);

  if (sd->output_queue) {
    struct Qdisc *head;
    local_irq_disable();
    head = sd->output_queue;
    sd->output_queue = NULL;
    sd->output_queue_tailp = &sd->output_queue;
    local_irq_enable();

    while (head) {
      struct Qdisc *q = head;
      spinlock_t *root_lock;
      head = head->next_sched;
      qdisc_run(q);
    }
  }
}

void qdisc_run(struct Qdisc *q)
{
  if (qdisc_run_begin(q)) {
    __qdisc_run(q);
    qdisc_run_end(q);
  }
}
```

## driver layer tx

```c
/* internet trasaction gigabit */
/* drivers/net/ethernet/intel/ixgb/ixgb_main.c */
const struct net_device_ops ixgb_netdev_ops = {
  .ndo_open               = ixgb_open,
  .ndo_stop               = ixgb_close,
  .ndo_start_xmit         = ixgb_xmit_frame,
  .ndo_set_rx_mode        = ixgb_set_multi,
  .ndo_validate_addr      = eth_validate_addr,
  .ndo_set_mac_address    = ixgb_set_mac,
  .ndo_change_mtu         = ixgb_change_mtu,
  .ndo_tx_timeout         = ixgb_tx_timeout,
  .ndo_vlan_rx_add_vid    = ixgb_vlan_rx_add_vid,
  .ndo_vlan_rx_kill_vid   = ixgb_vlan_rx_kill_vid,
  .ndo_fix_features       = ixgb_fix_features,
  .ndo_set_features       = ixgb_set_features,
};

netdev_tx_t
ixgb_xmit_frame(struct sk_buff *skb, struct net_device *netdev)
{
  struct ixgb_adapter *adapter = netdev_priv(netdev);

  if (count) {
      ixgb_tx_queue(adapter, count, vlan_id, tx_flags);
      /* Make sure there is space in the ring for the next send. */
      ixgb_maybe_stop_tx(netdev, &adapter->tx_ring, DESC_NEEDED);
  }

  return NETDEV_TX_OK;
}
```

# read

* [Monitoring and Tuning the Linux Networking Stack: Receiving Data](https://blog.packagecloud.io/monitoring-tuning-linux-networking-stack-receiving-data/)
* [Illustrated Guide to Monitoring and Tuning the Linux Networking Stack: Receiving Data](https://blog.packagecloud.io/illustrated-guide-monitoring-tuning-linux-networking-stack-receiving-data/)
* [NAPI description on Linux Foundation](https://wiki.linuxfoundation.org/networking/napi)

| Queue | Type | Scope | Purpose | Handler |
| :-: | :-: | :-: | :-: | :-: |
| Hardware RX Queue | Ring buffer | Per NIC/queue | Buffer raw frames | Driver (NAPI) |
| NAPI Poll Queue | Linked list | Per CPU | Schedule polling | net_rx_action() |
| Input Packet Queue | Linked list | Per CPU | Backlog for RPS/deferred | backlog_poll() |
| Out-of-Order Queue | Red-black tree | Per TCP socket | Reorder TCP segments | tcp_data_queue() |
| Socket Receive Queue | Linked list | Per socket | Buffer data for app | Application (recv()) |
| Socket Backlog Queue | Linked list | Per listen socket | Buffer pending connections | accept() |
| Process Queue | Linked list | Per CPU | Threaded deferred processing | NAPI thread |

![](../images/kernel/net-read-write-route-bridge.svg)

---

<img src='../images/kernel/net-filter-3.png' style='max-height:850px'/>

<img src='../images/kernel/net-read-flow.png' style='max-height:850px'/>

---

<img src='../images/kernel/net-read.svg'/>

## call-graph-read

```c
/* driver layer */
ixgb_intr(int irq, void *data) {
    IXGB_WRITE_REG(&adapter->hw, IMC, ~0);    /* disable irq */
    __napi_schedule() {
        list_add_tail(&napi->poll_list, &sd->poll_list)
        __raise_softirq_irqoff(NET_RX_SOFTIRQ) {
            net_rx_action() {
                LIST_HEAD(list);
                list_splice_init(&sd->poll_list, &list)

                for (list_empty(list)) {
                    napi_poll() {
                        napi->poll() { /* .poll = ixgbe_poll */
                            ixgb_clean_tx_irq(adapter); /* dma_unmap_page */
                            ixgb_clean_rx_irq(adapter, &work_done, budget) {
                                netif_receive_skb(skb);
                                    --->
                            }

                            if (work_done < budget) {
                                napi_complete_done(napi, work_done);
                                if (!test_bit(__IXGB_DOWN, &adapter->flags)) {
                                    ixgb_irq_enable(adapter) {/* re-enable irq */
                                        IXGB_WRITE_REG(&adapter->hw, IMC, val);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

/* mac layer */
netif_receive_skb() {
    __netif_receive_skb_core() {
        /* 0. RPS handling */
        if (rps_needed) {
            enqueue_to_backlog(skb, cpu);
            return ret;
        }

        /* 1. XDP (eXpress Data Path) */
        do_xdp_generic(&skb);

        /* 2. Packet_all Type Delivery (Global and Device-Specific) */
        list_for_each_entry_rcu(ptype, &net_hotdata.ptype_all, list) {
            if (pt_prev)
                ret = deliver_skb(skb, pt_prev, orig_dev);
            pt_prev = ptype;
        }
        list_for_each_entry_rcu(ptype, &skb->dev->ptype_all, list) {
            if (pt_prev)
                ret = deliver_skb(skb, pt_prev, orig_dev);
            pt_prev = ptype;
        }

        /* 3. Ingress Hooks */
        sch_handle_ingress();
        nf_ingress();

        /* 4. Vlan */
        vlan_do_receive(&skb);

        /* 5. RX Handler (e.g., Bridging) */
        rx_handler = rcu_dereference(skb->dev->rx_handler); /* br_handle_frame */
        rx_handler(&skb);


        /* 6. Packet_specific Type Delivery (Global and Device-Specific) */
        type = skb->protocol;
        if (likely(!deliver_exact)) {
            deliver_ptype_list_skb(&ptype_base[ntohs(type) & PTYPE_HASH_MASK]);
        }
        deliver_ptype_list_skb(&orig_dev->ptype_specific);
        deliver_ptype_list_skb(&skb->dev->ptype_specific);
    }
}

/* ip layer */
ip_rcv() {
    NF_HOOK(NF_INET_PRE_ROUTING) {              /* 1. NF_INET_PRE_ROUTING */
        ip_rcv_finish() {
            ip_rcv_finish_core() {
                ip_route_input_noref()          /* 2. Routing Lookup */
                ip_rcv_options(skb, dev);       /* IP Options Handling, Multicast/Broadcast Handling */
            }
            dst_input() {
                skb_dst(skb)->input(skb).ip_local_deliver() {
                    ip_defrag()                 /* 3. defragment */
                    NF_HOOK(NF_INET_LOCAL_IN)   /* 4. NF_INET_LOCAL_IN */
                    ip_local_deliver_finish() {
                        inet_protos[protocol]->handler() {
                            net_protocol->handler()
                        }
                    }
                }
            }
        }
    }
}

/* tcp layer */
tcp_v4_rcv() {
    sk = __inet_lookup_skb() {
        __inet_lookup_established();
        __inet_lookup_listener();
    }

process:
    if (sk->sk_state == TCP_TIME_WAIT)
        goto do_time_wait;

    /* handle half-connection sock */
    if (sk->sk_state == TCP_NEW_SYN_RECV) {
        struct request_sock *req = inet_reqsk(sk);
        sk = req->rsk_listener;

        newsk = tcp_check_req() {
            /* 1. alloc sock and insert into ehash */
            child = inet_csk(sk)->icsk_af_ops->syn_recv_sock() {
                tcp_v4_syn_recv_sock() {
                    if (sk_acceptq_is_full());
                        goto drop;
                    newsk = tcp_create_openreq_child(sk, req, skb) {
                        inet_csk_clone_lock() {
                            struct proto *prot = READ_ONCE(sk->sk_prot);
                            newsk = sk_prot_alloc(prot, priority, sk->sk_family) {
                                slab = prot->slab;
                                newsk = kmem_cache_alloc(slab, priority & ~__GFP_ZERO);
                            }
                            sock_copy(newsk, sk);
                            inet_sk_set_state(newsk, TCP_SYN_RECV) {
                                sk->sk_state = TCP_SYN_RECV;
                            }
                        }
                        tcp_init_xmit_timers();
                    }

                    inet_ehash_nolisten() {
                        inet_ehash_insert() { /* hash insert the new sk and remove the old request_sk */
                            inet_ehash_bucket(hashinfo, sk->sk_hash);
                            __sk_nulls_add_node_rcu() {
                                hlist_nulls_add_head_rcu(&sk->sk_nulls_node, list);
                            }
                            if (osk) {
                                sk_nulls_del_node_init_rcu(osk);
                            }
                        }
                    }
                }
            }

            inet_csk_complete_hashdance(child) {
                /* 2. remove from request queue */
                inet_csk_reqsk_queue_drop() {
                    reqsk_queue_unlink() {
                        __sk_nulls_del_node_init_rcu(sk);
                            hlist_nulls_del_init_rcu(&sk->sk_nulls_node);
                    }
                    reqsk_queue_removed() {
                        --icsk_accept_queue.yong;
                        --icsk_accept_queue.qlen;
                    }
                }
                reqsk_queue_removed();

                /* 3. insert into accept queue */
                inet_csk_reqsk_queue_add() {
                    sk->rskq_accept_tail = reqst_sk;
                    sk_acceptq_added() {
                        icsk_accept_queue->rskq_accept_tail = req;
                        sk_acceptq_added();
                            ++sk->sk_ack_backlog
                    }
                }
            }
        }

        tcp_child_process() {
            if (!sock_owned_by_user(child)) {
                tcp_rcv_state_process(child, skb);
                sk_listener->sk_data_ready() /* sock_def_readable, wakeup accept->inet_csk_wait_for_connect */
            } else {
                __sk_add_backlog(child, skb);
            }
        }
    }

    tcp_filter(sk, skb);
    tcp_v4_fill_cb(skb, iph, th);

    tcp_v4_do_rcv() {
        /* Fast Path */
        if (sk->sk_state == TCP_ESTABLISHED) {
            tcp_rcv_established() {
                trace_tcp_probe(sk, skb);
                tcp_checksum_complete(skb);

                tcp_data_queue(sk, skb) {
                    /* 1. [seq = rcv_next < end_seq < win] Normal  tcp_queue_rcv() */
                    /* 2. [seq < end_seq < rcv_next < win] DSACK    tcp_dsack_set() */
                    /* 3. [rcv_next < win < seq < end_seq] Out of Window tcp_drop() */
                    /* 4. [seq < rcv_next < end_seq < win] Out of Window tcp_ofo_queue() */
                    /* 5. [rcv_next < seq < end_seq < win] Out of Order tcp_data_queue_ofo() */
                }
                tcp_data_snd_check(sk) {
                    tcp_check_space() {
                        if (SOCK_QUEUE_SHRUNK && SOCK_NOSPACE) {
                            sk->sk_write_space(sk); {
                                sock_def_write_space();
                            }
                        }
                    }
                }
                tcp_ack_snd_check(sk);
                return 0;
            }
        }

        /* Slow Path */
        tcp_rcv_state_process() {
            if (sk->sk_state == TCP_LISTEN) {
                icsk->icsk_af_ops->conn_request(); /* ipv4_specific.conn_request */
                    tcp_v4_conn_request() {
                        tcp_conn_request() {
                            /* 1. check reqsk queue and accept queue */
                            if (inet_csk_reqsk_queue_is_full(sk));
                                drop;
                            if (sk_acceptq_is_full(sk));
                                drop;

                            /* 2. allock reqsk and init */
                            inet_reqsk_alloc();
                                sk->sk_state = TCP_NEW_SYN_RECV;

                            af_ops->init_req(req, sk, skb) {
                                tcp_v4_init_req();
                                    sk_rcv_saddr_set();
                                    sk_daddr_set();
                            }

                            /* 3. insert into ehash */
                            if (fastopen) {
                                inet_csk_reqsk_queue_add() {
                                    sk->rskq_accept_tail = reqst_sk;
                                    sk_acceptq_added() {
                                        icsk_accept_queue->rskq_accept_tail = req;
                                        sk_acceptq_added();
                                            ++sk->sk_ack_backlog
                                    }
                                }
                            } else  {
                                inet_csk_reqsk_queue_hash_add() {
                                    reqsk_queue_hash_req() {
                                        timer_setup();
                                        mod_timer(&req->rsk_timer, jiffies + timeout);
                                        inet_ehash_insert() {
                                            inet_ehash_bucket(hashinfo, sk->sk_hash);
                                            __sk_nulls_add_node_rcu();
                                                hlist_nulls_add_head_rcu(&sk->sk_nulls_node, list);
                                            if (osk);
                                                sk_nulls_del_node_init_rcu(osk);
                                        }
                                    }
                                    inet_csk_reqsk_queue_added() {
                                        ++icsk_accept_queue->young;
                                        --icsk_accept_queue->qlen;
                                    }
                                }
                            }

                            /* 4. send synack */
                            af_ops->send_synack() {
                                tcp_v4_send_synack();
                            }
                        }
                    }
                return 0;
            } /* TCP_LISTEN */

            if (sk->sk_state == TCP_SYN_SENT) {
                tcp_rcv_synsent_state_process() {
                    tcp_finish_connect(sk, skb) {
                        tcp_set_state(sk, TCP_ESTABLISHED);
                        tcp_init_transfer() {
                            tcp_init_buffer_space();
                            tcp_init_congestion_control(sk);
                        }
                        inet_csk_reset_keepalive_timer();
                    }

                    sk->sk_state_change() {
                        /* wakup `connect` slept at inet_wait_for_connect */
                        sock_def_wakeup();
                    }
                    tcp_send_ack(sk);
                    tcp_set_state(sk, TCP_ESTABLISHED);
                }

                tcp_data_snd_check(sk);
                return 0;
            } /* TCP_SYN_SENT */

            tcp_check_req(); /* tp->fastopen_rsk */

            /* Step 1 2 3 4 */
            tcp_validate_incoming() {
                /* RFC1323: H1. Apply PAWS check first. */
                /* Step 1: check sequence number */
                /* Step 2: check RST bit */
                /* Step 3: check security and precedence [ignored] */
                /* Step 4: Check for a SYN RFC 5961 4.2 : Send a challenge ack */
            }

            /* step 5: check the ACK field */
            tcp_ack() {
                tcp_clean_rtx_queue();
                tcp_set_xmit_timer();
                tcp_cong_control() {
                    if (tcp_in_cwnd_reduction(sk)) {
                        /* Reduce cwnd if state mandates */
                        tcp_cwnd_reduction(sk, acked_sacked, rs->losses, flag);
                    } else if (tcp_may_raise_cwnd(sk, flag)) {
                        /* Advance cwnd if state allows */
                        tcp_cong_avoid(sk, ack, acked_sacked);
                    }
                    tcp_update_pacing_rate(sk);
                }
                tcp_xmit_recovery();
            }

            if (sk->sk_state == TCP_SYN_RECV) {
                tcp_init_transfer() {
                    tcp_snd_cwnd_set();
                    tcp_init_buffer_space()
                    tcp_init_congestion_control();
                }
                tcp_set_state(sk, TCP_ESTABLISHED);

                sk->sk_state_change() {
                    sock_def_wakeup() {
                        wake_up_interruptible_all() {
                            __wake_up() {
                                __wake_up_common_lock() {
                                    __wake_up_common();
                                }
                            }
                        }
                    }
                }
            } /* TCP_SYN_RECV */

            if (sk->sk_state == TCP_LAST_ACK)
                tcp_done();

            if (sk->sk_state == TCP_FIN_WAIT1) {
                tcp_set_state(sk, TCP_FIN_WAIT2);
                sk->sk_shutdown |= SEND_SHUTDOWN;
                tcp_time_wait(sk, TCP_FIN_WAIT2, tmo);
                sk->sk_state_change(); /* Wake up lingering close() */
                tcp_time_wait(sk, TCP_FIN_WAIT2, tmo);
            }

            if (sk->sk_state == TCP_FIN_WAIT2) {
                if (sk->sk_shutdown & RCV_SHUTDOWN);
                    tcp_reset();
            }

            if (sk->sk_state == TCP_ESTABLISHED) {
                tcp_data_queue();
            }

            if (sk->sk_state == TCP_CLOSING) {
                tcp_time_wait(sk, TCP_TIME_WAIT, 0);
            }

            /* Step 6: check the URG bit */
            tcp_urg(sk, skb, th);
        }
    }

    if (!sock_owned_by_user(sk)) {
        ret = tcp_v4_do_rcv(sk, skb);
    } else if (tcp_add_backlog(sk, skb)) {
        goto discard_and_relse;
    }

do_time_wait:
    if (sk->sk_state == TCP_TIME_WAIT) {
        tcp_v4_fill_cb(skb, iph, th);
        switch (tcp_timewait_state_process(inet_twsk(sk), skb, th)) {
            case TCP_TW_SYN: {
                /* reuse connection transition to SYN_RECV */
                goto process;
            }
            case TCP_TW_ACK: {
                /* send last ack */
                tcp_v4_timewait_ack(sk, skb);
                break;
            }
            case TCP_TW_RST: {
                tcp_v4_send_reset(sk, skb);
                inet_twsk_deschedule_put(inet_twsk(sk));
                goto discard_it;
            }
            case TCP_TW_SUCCESS: {

            }
        }
    }
}

tcp_data_queue() {
    /* 1. ACK [seq = rcv_next < end_seq < win] */
        tcp_queue_rcv();
        tcp_event_data_recv();
            tcp_grow_window();
        tcp_ofo_queue(sk); /* put data from the out_of_order queue into the receive_queue */

        kfree_skb_partial();
        tcp_data_ready() { /* wakeup `sk_wait_data` */
            sk->sk_data_ready();
                sock_def_readable();
        }

    /* 2. DROP [seq < end_seq < rcv_next < win] */
        tcp_dsack_set();
        tcp_enter_quickack_mode();
        inet_csk_schedule_ack(sk);
        tcp_drop();

    /* 3. DROP [rcv_next < win < seq < end_seq] */
        tcp_enter_quickack_mode();
        inet_csk_schedule_ack(sk);
        tcp_drop();

    /* 4. OFO [seq < rcv_next < end_seq < win] */
        tcp_dsack_set();

    /* 5. DACK [rcv_next < seq < end_seq < win] */
        tcp_data_queue_ofo();
}

/* vfs layer */
SYSCALL_DEFINE3(read, fd, buf, count) {
    vfs_read(file, buf, counts);
        file->f_op->read_iter(kio, iter);
}

sock_read_iter() {
    sock_recvmsg() {
        sock->ops->recvmsg() {
            sock_rps_record_flow(); /* RPS, RFS */
            inet_recvmsg() {
                sk->sk_prot->recvmsg() {
                    tcp_recvmsg() {
                        skb_peek_tail(&sk->sk_receive_queue)
                        skb_queue_walk(&sk->sk_receive_queue, skb) {
                            if (sk->sk_shutdown & RCV_SHUTDOWN)
                                break;
                            skb_copy_datagram_msg() /* copy data to user space */
                            tcp_rcv_space_adjust()    /* adjust receive buffer space */

                            sk_wait_data() { /* waiting for data, waked up by `tcp_data_ready` */
                                sk_wait_event() {
                                    wait_woken()
                                        schedule_timeout()
                                }
                                woken_wake_function() {
                                    default_wake_function()
                                        try_to_wake_up()
                                }
                            }
                        }

                        tcp_cleanup_rbuf()

                        /* process backlog */
                        release_sock(sk) {
                            sk_backlog_rcv() {
                                sk->sk_backlog_rcv() {
                                    tcp_v4_do_rcv()
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
```
* [try_to_wake_up](./linux-proc.md#ttwu)

## driver layer rx

<img src='../images/kernel/net-dev-pci.svg' style='max-height:850px'/>

<img src='../images/kernel/net-rx_ring.png' style='max-height:850px'/>

Ring | Buffer
:-: | :-:
<img src='../images/kernel/net-desc-ring-1.png' style='max-height:550px'/> | <img src='../images/kernel/net-desc-ring-2.png' style='max-height:850px'/>

| Aspect | NIC Multiple Queues | Kernel Ring Buffer |
| :-: | :-: | :-: |
| Location | NIC hardware | Kernel memory |
| Scope | Multiple queues per NIC (e.g., 4 RX, 4 TX) | One ring buffer per queue per NIC |
| Purpose | Load balancing across CPU cores | Packet staging and kernel processing |
| Data Structure | Hardware ring buffers of descriptors | Software circular buffer of descriptors |
| Control | NIC firmware/hardware + driver config | Kernel network driver |
| Scalability | Scales with CPU cores via parallel queues | Scales with queue size and batching |
| Interaction | Feeds packets into kernel ring buffers | Receives packets from NIC queues |


* You can interpret a struct pci_dev as a representation of a struct net_device at the PCI level, but with some important nuances.
* The struct pci_dev represents the underlying PCI hardware device (e.g., a network interface card or NIC), while the struct net_device represents the network interface at the kernel’s networking layer.
* They are not the same object—they serve different purposes at different layers.


NAPI Usage:
1. embed `struct napi_struct` in the driver's private structure
2. initialise and register before the net device itself, using `netif_napi_add`, and unregistered after the net device, using `netif_napi_del`
3. changes to driver's interrupt handler
    * disabling interrupts
    * tell the networking subsystem to poll your driver shortly to pick up all available packets by `napi_schedule`
4. create a `poll` method for your driver; it's job is to obtain packets from the network interface and feed them into the kernel
    * The poll() function must return the amount of work done.
    * If and only if the return value is less than the budget, your driver must reenable interrupts and turn off polling by `napi_complete`

```c
struct pci_dev {
    struct device     dev;      /* Generic device interface */
    struct pci_driver *driver;  /* Driver bound to this device */
};

struct device {
    void                      *driver_data;  /* dev_set_drvdata/dev_get_drvdata */

    const char                *init_name;
    struct kobject            kobj;

    unsigned long             state;

    struct list_head          dev_list;
    struct list_head          napi_list;
    struct list_head          unreg_list;
    struct list_head          close_list;
    struct list_head          ptype_all;
    struct list_head          ptype_specific;

    struct device             *parent;
    const struct device_type  *type;
    struct device_driver      *driver;
    struct device_private     *p;

    struct bus_type           *bus;
    void                      *platform_data;
    int                       numa_node;
};

struct device_driver {
    const char          *name;
    struct bus_type     *bus;

    struct module       *owner;
    const char          *mod_name;  /* used for built-in modules */

    bool suppress_bind_attrs;  /* disables bind/unbind via sysfs */
    enum probe_type probe_type;

    const struct of_device_id     *of_match_table;
    const struct acpi_device_id   *acpi_match_table;

    int (*probe) (struct device *dev);
    int (*remove) (struct device *dev);

    void (*coredump) (struct device *dev);

    struct driver_private         *p;
    const struct dev_pm_ops       *pm;
};

struct net_device {
    struct device               dev;
    const struct net_device_ops *netdev_ops;
    const struct ethtool_ops    *ethtool_ops;

    struct netdev_rx_queue      *_rx;
    unsigned int                num_rx_queues;
    unsigned int                real_num_rx_queues;

    struct netdev_queue         *_tx;
    unsigned int                num_tx_queues;
    unsigned int                real_num_tx_queues;

    struct Qdisc                *qdisc;

    struct {
        struct list_head upper;
        struct list_head lower;
    } adj_list;
}

struct ixgb_adapter {
    struct timer_list watchdog_timer;
    unsigned long active_vlans[BITS_TO_LONGS(VLAN_N_VID)];
    u32 bd_number;
    u32 rx_buffer_len;
    u32 part_num;
    u16 link_speed;
    u16 link_duplex;
    struct work_struct tx_timeout_task;

    /* TX */
    struct ixgb_desc_ring tx_ring;
    unsigned int restart_queue;
    unsigned long timeo_start;
    u32 tx_cmd_type;
    u64 hw_csum_tx_good;
    u64 hw_csum_tx_error;
    u32 tx_int_delay;
    u32 tx_timeout_count;
    bool tx_int_delay_enable;
    bool detect_tx_hung;

    /* RX */
    struct ixgb_desc_ring rx_ring;
    u64 hw_csum_rx_error;
    u64 hw_csum_rx_good;
    u32 rx_int_delay;
    bool rx_csum;

    /* OS defined structs */
    struct napi_struct  napi;
    struct net_device   *netdev;
    struct pci_dev      *pdev;

    /* structs defined in ixgb_hw.h */
    struct ixgb_hw hw;
    u16 msg_enable;
    struct ixgb_hw_stats stats;
    u32 alloc_rx_buff_failed;
    bool have_msi;
    unsigned long flags;
};

struct ixgb_desc_ring {
    void            *desc; /* pointer to ring array of ixgb_{rx, tx}_desc */
    dma_addr_t      dma; /* physical address of the descriptor ring */
    unsigned int    size; /* length of descriptor ring in bytes */
    unsigned int    count; /* number of descriptors in the ring */
    unsigned int    next_to_use; /* next descriptor to associate a buffer with */
    unsigned int    next_to_clean; /* next descriptor to check for DD status bit */
    struct ixgb_buffer *buffer_info; /* array of buffer information structs */
};

#ifdef CONFIG_ARCH_DMA_ADDR_T_64BIT
    typedef u64 dma_addr_t;
#else
    typedef u32 dma_addr_t;
#endif

/* ixgb_alloc_rx_buffers maps skb to dma */
struct ixgb_buffer {
    struct sk_buff  *skb;
    dma_addr_t      dma;
    unsigned long   time_stamp;
    u16             length;
    u16             next_to_watch;
    u16             mapped_as_page;
};

struct ixgb_rx_desc {
    __le64        buff_addr;
    __le16        length;
    __le16        reserved;
    u8            status;
    u8            errors;
    __le16        special;
};

struct ixgb_tx_desc {
    __le64        buff_addr;
    __le32        cmd_type_len;
    u8            status;
    u8            popts;
    __le16        vlan;
};

/* (GRO) allows the NIC driver to combine received packets
 * into a single large packet which is then passed to the IP stack */
struct napi_struct {
    struct list_head  poll_list; /* archoed */

    unsigned long     state;
    int               weight;
    unsigned long     gro_bitmask;
    int               (*poll)(struct napi_struct *, int);
    int               poll_owner;
    struct net_device *dev;
    struct gro_list   gro_hash[GRO_HASH_BUCKETS];
    struct sk_buff     *skb;
    struct list_head  rx_list; /* Pending GRO_NORMAL skbs */
    int               rx_count; /* length of rx_list */
    struct hrtimer    timer;
    struct list_head  dev_list;
    struct hlist_node napi_hash_node;
    unsigned int      napi_id;
};
```

```c
MODULE_AUTHOR("Intel Corporation, <linux.nics@intel.com>");
MODULE_DESCRIPTION("Intel(R) PRO/10GbE Network Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);

int ixgb_init_module(void)
{
    return pci_register_driver(&ixgb_driver);
}

struct pci_driver {
    struct list_head  node;
    const char        *name;
    const struct pci_device_id *id_table;
    int  (*probe)(struct pci_dev *dev, const struct pci_device_id *id);
    void (*remove)(struct pci_dev *dev);
    int  (*suspend)(struct pci_dev *dev, pm_message_t state);
    int  (*suspend_late)(struct pci_dev *dev, pm_message_t state);
    int  (*resume_early)(struct pci_dev *dev);
    int  (*resume) (struct pci_dev *dev);
    void (*shutdown) (struct pci_dev *dev);
    int  (*sriov_configure) (struct pci_dev *dev, int num_vfs);
    const struct pci_error_handlers   *err_handler;
    const struct attribute_group      **groups;
    struct device_driver              driver;
    struct pci_dynids                 dynids;
};

struct pci_driver ixgb_driver = {
    .name         = ixgb_driver_name,
    .id_table     = ixgb_pci_tbl,
    .probe        = ixgb_probe,
    .remove       = ixgb_remove,
    .err_handler  = &ixgb_err_handler
};

static struct pci_driver ixgbe_driver = {
    .name      = ixgbe_driver_name,
    .id_table  = ixgbe_pci_tbl,
    .probe     = ixgbe_probe,
    .remove    = ixgbe_remove,
    .driver.pm = pm_sleep_ptr(&ixgbe_pm_ops),
    .shutdown  = ixgbe_shutdown,
    .sriov_configure = ixgbe_pci_sriov_configure,
    .err_handler = &ixgbe_err_handler
};

static const struct pci_device_id ixgbe_pci_tbl[] = {
    {PCI_VDEVICE(INTEL, IXGBE_DEV_ID_82598), board_82598 },
    {PCI_VDEVICE(INTEL, IXGBE_DEV_ID_82598AF_DUAL_PORT), board_82598 },
    {PCI_VDEVICE(INTEL, IXGBE_DEV_ID_82598AF_SINGLE_PORT), board_82598 },
    {PCI_VDEVICE(INTEL, IXGBE_DEV_ID_82598AT), board_82598 },
    {PCI_VDEVICE(INTEL, IXGBE_DEV_ID_82598AT2), board_82598 },
};

enum ixgbe_boards {
    board_82598,
    board_82599,
    board_X540,
    board_X550,
    board_X550EM_x,
    board_x550em_x_fw,
    board_x550em_a,
    board_x550em_a_fw,
    board_e610,
};

module_init(ixgbe_init_module);

/* Device Initialization Routine */
int ixgbe_probe(
  struct pci_dev *pdev, const struct pci_device_id *ent)
{
    struct net_device *netdev = NULL;
    struct ixgb_adapter *adapter;

    netdev = alloc_etherdev(sizeof(struct ixgb_adapter));
    SET_NETDEV_DEV(netdev, &pdev->dev);

    pci_set_drvdata(pdev, netdev); /* pdev->dev->driver_data = netdev */
    adapter = netdev_priv(netdev);
    adapter->netdev = netdev;
    adapter->pdev = pdev;
    adapter->msg_enable = netif_msg_init(debug, DEFAULT_MSG_ENABLE);

    adapter->hw.back = adapter;
    adapter->hw.hw_addr = pci_ioremap_bar(pdev, BAR_0);

    netdev->netdev_ops = &ixgb_netdev_ops;
    ixgb_set_ethtool_ops(netdev);
    netdev->watchdog_timeo = 5 * HZ;

    /* register poll func: ixgbe_poll */
    netif_napi_add(netdev, &adapter->napi, ixgbe_poll, 64);

    strncpy(netdev->name, pci_name(pdev), sizeof(netdev->name) - 1);

    adapter->bd_number = cards_found;
    adapter->link_speed = 0;
    adapter->link_duplex = 0;
}

void netif_napi_add(struct net_device *dev, struct napi_struct *napi,
        int (*poll)(struct napi_struct *, int), int weight)
{
    INIT_LIST_HEAD(&napi->poll_list);
    hrtimer_init(&napi->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL_PINNED);
    napi->timer.function = napi_watchdog;
    init_gro_hash(napi);
    napi->skb = NULL;
    napi->poll = poll;
    napi->weight = weight;
    napi->dev = dev;
    napi->poll_owner = -1;
    set_bit(NAPI_STATE_SCHED, &napi->state);
    set_bit(NAPI_STATE_NPSVC, &napi->state);
    list_add_rcu(&napi->dev_list, &dev->napi_list);
    napi_hash_add(napi);
}
```

```c
/* internet trasaction gigabyte
  drivers/net/ethernet/intel/ixgb/ixgb_main.c */
const struct net_device_ops ixgb_netdev_ops = {
    .ndo_open               = ixgb_open,
    .ndo_stop               = ixgb_close,
    .ndo_start_xmit         = ixgb_xmit_frame,
    .ndo_set_rx_mode        = ixgb_set_multi,
    .ndo_validate_addr      = eth_validate_addr,
    .ndo_set_mac_address    = ixgb_set_mac,
    .ndo_change_mtu         = ixgb_change_mtu,
    .ndo_tx_timeout         = ixgb_tx_timeout,
    .ndo_vlan_rx_add_vid    = ixgb_vlan_rx_add_vid,
    .ndo_vlan_rx_kill_vid   = ixgb_vlan_rx_kill_vid,
    .ndo_fix_features       = ixgb_fix_features,
    .ndo_set_features       = ixgb_set_features,
};

/* activate device: ixgb_open ->
 * register interrupt */
int ixgb_up(struct ixgb_adapter *adapter)
{
    struct net_device *netdev = adapter->netdev;

    err = request_irq(adapter->pdev->irq, ixgb_intr, irq_flags, netdev->name, netdev);
}

irqreturn_t ixgb_intr(int irq, void *data)
{
    struct net_device *netdev = data;
    struct ixgb_adapter *adapter = netdev_priv(netdev);
    struct ixgb_hw *hw = &adapter->hw;
    u32 icr = IXGB_READ_REG(hw, ICR);

    if (unlikely(!icr))
        return IRQ_NONE;  /* Not our interrupt */

    if (unlikely(icr & (IXGB_INT_RXSEQ | IXGB_INT_LSC)))
        if (!test_bit(__IXGB_DOWN, &adapter->flags))
        mod_timer(&adapter->watchdog_timer, jiffies);

    /* check if napi can be scheduled */
    if (napi_schedule_prep(&adapter->napi)) {
        /*Disable interrupts and register for poll.
        The flush of the posted write is intentionally left out.
        Enable in ixgbe_poll if work is done, i.e., work < budget */
        IXGB_WRITE_REG(&adapter->hw, IMC, ~0);
        __napi_schedule(&adapter->napi);
    }
    return IRQ_HANDLED;
}

static inline bool napi_schedule(struct napi_struct *n)
{
    ret = napi_schedule_prep(n) {
        unsigned long new, val = READ_ONCE(n->state);

        do {
            if (unlikely(val & NAPIF_STATE_DISABLE))
                return false;
            new = val | NAPIF_STATE_SCHED;
            new |= (val & NAPIF_STATE_SCHED) / NAPIF_STATE_SCHED * NAPIF_STATE_MISSED;
        } while (!try_cmpxchg(&n->state, &val, new));

        return !(val & NAPIF_STATE_SCHED);
    }
    if (ret) {
        __napi_schedule(n);
        return true;
    }

    return false;
}

/* __napi_schedule - schedule for receive
 * @n: entry to schedule
 *
 * The entry's receive function will be scheduled to run.
 * Consider using __napi_schedule_irqoff() if hard irqs are masked. */
void __napi_schedule(struct napi_struct *n)
{
    unsigned long flags;

    local_irq_save(flags);
    ____napi_schedule(this_cpu_ptr(&softnet_data), n) {
        list_add_tail(&napi->poll_list, &sd->poll_list);
        __raise_softirq_irqoff(NET_RX_SOFTIRQ); /* net_rx_action */
    }
    local_irq_restore(flags);
}

void net_rx_action(struct softirq_action *h)
{
    struct softnet_data *sd = this_cpu_ptr(&softnet_data);
    unsigned long time_limit = jiffies + usecs_to_jiffies(netdev_budget_usecs);
    int budget = netdev_budget;
    LIST_HEAD(list);
    LIST_HEAD(repoll);

    local_irq_disable();
    list_splice_init(&sd->poll_list, &list);
    local_irq_enable();

    for (;;) {
        struct napi_struct *n;

        if (list_empty(&list)) {
            /* Receive Packet Steering */
            if (!sd_has_rps_ipi_waiting(sd) && list_empty(&repoll))
                goto out;
            break;
        }

        n = list_first_entry(&list, struct napi_struct, poll_list);
        budget -= napi_poll(n, &repoll);

        /* If softirq window is exhausted then punt.
         * Allow this to run for 2 jiffies since which will allow
         * an average latency of 1.5/HZ. */
        if (unlikely(budget <= 0 || time_after_eq(jiffies, time_limit))) {
            sd->time_squeeze++;
            break;
        }
    }

    local_irq_disable();

    list_splice_tail_init(&sd->poll_list, &list);
    list_splice_tail(&repoll, &list);
    list_splice(&list, &sd->poll_list);
    if (!list_empty(&sd->poll_list))
        __raise_softirq_irqoff(NET_RX_SOFTIRQ);

    net_rps_action_and_irq_enable(sd);
out:
    __kfree_skb_flush();
}
```

|Queue|Key Functions|Purpose|
|-----|-------------|-------|
|poll_list|`napi_schedule()`, `net_rx_action()`, `napi_complete_done()`|Holds NAPI contexts for polling|
|process_queue|`enqueue_to_backlog()`, `net_rx_action()`, `__netif_receive_skb_core()`|Packets for softIRQ processing|
|input_pkt_queue|`netif_rx()`, `enqueue_to_backlog()`, `net_rx_action()`, `dev_gro_receive()`|Incoming packets before processing|
|completion_queue|`net_tx_action()`, `kfree_skb()`, `napi_gro_complete()`|Processed packets awaiting cleanup|
|xfrm_backlog|`xfrm_input()`, `xfrm_offload()`, `net_rx_action()`|IPsec offload packet queue|
|defer_list|`net_defer_list_add()`, `net_defer_process()`, `smp_call_function_single_async()`|Deferred packets for later processing|

```c
/* Incoming packets are placed on per-CPU queues */
struct softnet_data {
    struct list_head    poll_list;  /* napi_struct list ready for polling */

    struct sk_buff_head process_queue; /*  Queue of packets to be processed in the softIRQ context */

    struct napi_struct  backlog; /* handle skbs in input_pkt_queue */
    struct sk_buff_head input_pkt_queue; /* Queue for incoming packets before they’re processed. */

    struct sk_buff_head xfrm_backlog;    /* Backlog for XFRM (IPsec) offloaded packets */

    /* stats */
    unsigned int        processed;
    unsigned int        time_squeeze;

#ifdef CONFIG_RPS
    struct softnet_data *rps_ipi_list;  /* Linked list of softnet_data structures for inter-CPU packet steering. */
#endif

    unsigned int        received_rps;
    bool                in_net_rx_action; /* Flag indicating if this CPU is currently in the network receive path. */
    bool                in_napi_threaded_poll; /* Flag indicating if NAPI polling is running in a threaded context. */

#ifdef CONFIG_NET_FLOW_LIMIT
    struct sd_flow_limit __rcu *flow_limit;
#endif
    struct Qdisc        *output_queue;
    struct Qdisc        **output_queue_tailp;
    struct sk_buff      *completion_queue; /* Queue of packets that have been processed and are awaiting */

    struct netdev_xmit xmit; /* Structure for tracking transmit-related data (specific fields depend on kernel version). */

#ifdef CONFIG_RPS
    call_single_data_t      csd; /* Structure for sending inter-CPU calls (e.g., IPIs for RPS). */
    struct softnet_data     *rps_ipi_next; /* Pointer to the next CPU’s softnet_data in the RPS chain. */
    unsigned int            cpu;
    unsigned int            input_queue_head; /* Indices for managing the input packet queue, aligned for cache efficiency. */
    unsigned int            input_queue_tail;
#endif


    atomic_t            dropped;
    spinlock_t          defer_lock;
    int                 defer_count;
    int                 defer_ipi_scheduled;
    struct sk_buff      *defer_list;
    call_single_data_t  defer_csd;
};

/* napi_poll -> ixgbe_poll -> */
int ixgbe_poll(struct napi_struct *napi, int budget)
{
    struct ixgb_adapter *adapter = container_of(napi, struct ixgb_adapter, napi);
    int work_done = 0;

    ixgb_clean_tx_irq(adapter);
    ixgb_clean_rx_irq(adapter, &work_done, budget);

    /*If budget not fully consumed, exit the polling mode
        If and only if the return value is less than the budget,
        your driver must reenable interrupts and turn off polling. */
    if (work_done < budget) {
        napi_complete_done(napi, work_done);
        if (!test_bit(__IXGB_DOWN, &adapter->flags))
        /* counterpart to IXGB_WRITE_REG(&adapter->hw, IMC, ~0) */
        ixgb_irq_enable(adapter);
    }

    return work_done;
}

bool napi_complete_done(struct napi_struct *n, int work_done)
{
    unsigned long flags, val, new;

    /* 1) Don't let napi dequeue from the cpu poll list
    *    just in case its running on a different cpu.
    * 2) If we are busy polling, do nothing here, we have
    *    the guarantee we will be called later. */
    if (unlikely(n->state & (NAPIF_STATE_NPSVC | NAPIF_STATE_IN_BUSY_POLL)))
        return false;

    if (n->gro_bitmask) {
        unsigned long timeout = 0;

        if (work_done)
        timeout = n->dev->gro_flush_timeout;

        /* When the NAPI instance uses a timeout and keeps postponing
        * it, we need to bound somehow the time packets are kept in
        * the GRO layer */
        napi_gro_flush(n, !!timeout);
        if (timeout)
            hrtimer_start(&n->timer, ns_to_ktime(timeout), HRTIMER_MODE_REL_PINNED);
    }
    if (unlikely(!list_empty(&n->poll_list))) {
        /* If n->poll_list is not empty, we need to mask irqs */
        local_irq_save(flags);
        list_del_init(&n->poll_list);
        local_irq_restore(flags);
    }

    do {
        val = READ_ONCE(n->state);

        WARN_ON_ONCE(!(val & NAPIF_STATE_SCHED));

        new = val & ~(NAPIF_STATE_MISSED | NAPIF_STATE_SCHED);

        /* If STATE_MISSED was set, leave STATE_SCHED set,
        * because we will call napi->poll() one more time.
        * This C code was suggested by Alexander Duyck to help gcc. */
        new |= (val & NAPIF_STATE_MISSED) / NAPIF_STATE_MISSED * NAPIF_STATE_SCHED;
    } while (cmpxchg(&n->state, val, new) != val);

    if (unlikely(val & NAPIF_STATE_MISSED)) {
        __napi_schedule(n);
        return false;
    }

    return true;
}

bool ixgb_clean_rx_irq(struct ixgb_adapter *adapter, int *work_done, int work_to_do)
{
    struct ixgb_desc_ring *rx_ring = &adapter->rx_ring;
    struct net_device *netdev = adapter->netdev;
    struct pci_dev *pdev = adapter->pdev;
    struct ixgb_rx_desc *rx_desc, *next_rxd;
    struct ixgb_buffer *buffer_info, *next_buffer, *next2_buffer;
    u32 length;
    unsigned int i, j;
    int cleaned_count = 0;
    bool cleaned = false;

    i = rx_ring->next_to_clean;
    rx_desc = IXGB_RX_DESC(*rx_ring, i);
    buffer_info = &rx_ring->buffer_info[i];

    while (rx_desc->status & IXGB_RX_DESC_STATUS_DD) {
        struct sk_buff *skb;
        u8 status;

        if (*work_done >= work_to_do)
            break;

        (*work_done)++;
        rmb();  /* read descriptor and rx_buffer_info after status DD */
        status = rx_desc->status;
        skb = buffer_info->skb;
        buffer_info->skb = NULL;

        prefetch(skb->data - NET_IP_ALIGN);

        if (++i == rx_ring->count)
            i = 0;
        next_rxd = IXGB_RX_DESC(*rx_ring, i);
        prefetch(next_rxd);

        j = i + 1;
        if (j == rx_ring->count)
            j = 0;
        next2_buffer = &rx_ring->buffer_info[j];
        prefetch(next2_buffer);

        next_buffer = &rx_ring->buffer_info[i];

        cleaned = true;
        cleaned_count++;

        dma_unmap_single(&pdev->dev, buffer_info->dma, buffer_info->length, DMA_FROM_DEVICE);
        buffer_info->dma = 0;

        length = le16_to_cpu(rx_desc->length);
        rx_desc->length = 0;

        if (unlikely(!(status & IXGB_RX_DESC_STATUS_EOP))) {
            dev_kfree_skb_irq(skb);
            goto rxdesc_done;
        }

        if (unlikely(rx_desc->errors &
            (IXGB_RX_DESC_ERRORS_CE | IXGB_RX_DESC_ERRORS_SE |
            IXGB_RX_DESC_ERRORS_P | IXGB_RX_DESC_ERRORS_RXE))) {
            dev_kfree_skb_irq(skb);
            goto rxdesc_done;
        }

        ixgb_check_copybreak(&adapter->napi, buffer_info, length, &skb);

        /* Good Receive */
        skb_put(skb, length);

        /* Receive Checksum Offload */
        ixgb_rx_checksum(adapter, rx_desc, skb);

        skb->protocol = eth_type_trans(skb, netdev);
        if (status & IXGB_RX_DESC_STATUS_VP)
            __vlan_hwaccel_put_tag(skb, htons(ETH_P_8021Q), le16_to_cpu(rx_desc->special));

        netif_receive_skb(skb);

    rxdesc_done:
        /* clean up descriptor, might be written over by hw */
        rx_desc->status = 0;

        /* return some buffers to hardware, one at a time is too slow */
        if (unlikely(cleaned_count >= IXGB_RX_BUFFER_WRITE)) {
            ixgb_alloc_rx_buffers(adapter, cleaned_count);
            cleaned_count = 0;
        }

        /* use prefetched values */
        rx_desc = next_rxd;
        buffer_info = next_buffer;
    }

    rx_ring->next_to_clean = i;

    cleaned_count = IXGB_DESC_UNUSED(rx_ring);
    if (cleaned_count)
        ixgb_alloc_rx_buffers(adapter, cleaned_count);

    return cleaned;
}

void ixgb_check_copybreak(struct napi_struct *napi,
    struct ixgb_buffer *buffer_info,
    u32 length, struct sk_buff **skb)
{
    struct sk_buff *new_skb;

    if (length > copybreak)
        return;

    new_skb = napi_alloc_skb(napi, length);
    if (!new_skb)
        return;

    skb_copy_to_linear_data_offset(
        new_skb, /*skb*/
        -NET_IP_ALIGN, /* offset */
        (*skb)->data - NET_IP_ALIGN, /* from */
        length + NET_IP_ALIGN /* len */
    ) {
        memcpy(skb->data + offset, from, len);
    }
    /* save the skb in buffer_info as good */
    buffer_info->skb = *skb;
    *skb = new_skb;
}
```

## mac layer rx

RPS config:
```sh
ethtool -n <interface> # Check RSS indirection table
echo "0xf" > /sys/class/net/<interface>/queues/rx-<N>/rps_cpus # RPS to CPU0-3
echo "2048" > /sys/class/net/<interface>/queues/rx-<N>/rps_flow_cnt # RFS flows
echo "4096" > /proc/sys/net/<interface>/rps_sock_flow_entries # Global RFS table
```

```c
/* netif_receive_skb -> netif_receive_skb_internal ->
 * __netif_receive_skb -> __netif_receive_skb_core */
int netif_receive_skb(struct sk_buff *skb)
{
    int ret;

    net_timestamp_check(READ_ONCE(net_hotdata.tstamp_prequeue), skb);

    if (skb_defer_rx_timestamp(skb))
        return NET_RX_SUCCESS;

    rcu_read_lock();

    /* 0. RPS handling */
    if (static_branch_unlikely(&rps_needed)) {
        struct rps_dev_flow voidflow, *rflow = &voidflow;
        /* RPS is recored at sock_rps_record_flow_hash */
        int cpu = get_rps_cpu(skb->dev, skb, &rflow);

        if (cpu >= 0) {
            ret = enqueue_to_backlog(skb, cpu, &rflow->last_qtail) {
                struct softnet_data *sd;
                unsigned long flags;
                unsigned int qlen;

                sd = &per_cpu(softnet_data, cpu);

                local_irq_save(flags);

                rps_lock(sd);
                if (!netif_running(skb->dev))
                    goto drop;

                qlen = skb_queue_len_lockless(&sd->input_pkt_queue);
                max_backlog = READ_ONCE(net_hotdata.max_backlog); /* sysctl netdev_max_backlog */
                if (unlikely(qlen > max_backlog))
                    goto cpu_backlog_drop;

                backlog_lock_irq_save(sd, &flags);
                qlen = skb_queue_len(&sd->input_pkt_queue);
                if (qlen <= max_backlog && !skb_flow_limit(skb, qlen)) {
                    if (!qlen) {
                        if (!__test_and_set_bit(NAPI_STATE_SCHED, &sd->backlog.state))
                            napi_schedule_rps(sd);
                    }
                    __skb_queue_tail(&sd->input_pkt_queue, skb);
                    tail = rps_input_queue_tail_incr(sd);
                    backlog_unlock_irq_restore(sd, &flags);

                    /* save the tail outside of the critical section */
                    rps_input_queue_tail_save(qtail, tail);
                    return NET_RX_SUCCESS;
                }

            cpu_backlog_drop:
                atomic_inc(&sd->dropped);
            bad_dev:
                dev_core_stats_rx_dropped_inc(skb->dev);
                kfree_skb_reason(skb, reason);
                return NET_RX_DROP;
            }
            rcu_read_unlock();
            return ret;
        }
    }

    ret = __netif_receive_skb(skb) {
        int ret;

        if (sk_memalloc_socks() && skb_pfmemalloc(skb)) {
            unsigned int noreclaim_flag;

            /* PFMEMALLOC skbs are special, they should
            * - be delivered to SOCK_MEMALLOC sockets only
            * - stay away from userspace
            * - have bounded memory usage
            *
            * Use PF_MEMALLOC as this saves us from propagating the allocation
            * context down to all allocation sites. */
            noreclaim_flag = memalloc_noreclaim_save();
            ret = __netif_receive_skb_one_core(skb, true);
            memalloc_noreclaim_restore(noreclaim_flag);
        } else {
            ret = __netif_receive_skb_one_core(skb, false) {
                struct net_device *orig_dev = skb->dev;
                struct packet_type *pt_prev = NULL;
                int ret;

                ret = __netif_receive_skb_core(&skb, pfmemalloc, &pt_prev);
                if (pt_prev)
                    ret = INDIRECT_CALL_INET(pt_prev->func, ipv6_rcv, ip_rcv, skb,
                                skb->dev, pt_prev, orig_dev);
                return ret;
            }
        }

        return ret;
    }
    rcu_read_unlock();
    return ret;
}

int __netif_receive_skb_core(struct sk_buff **pskb, bool pfmemalloc,
            struct packet_type **ppt_prev)
{
    struct packet_type *ptype, *pt_prev;
    rx_handler_func_t *rx_handler;
    struct sk_buff *skb = *pskb;
    struct net_device *orig_dev;
    bool deliver_exact = false;
    int ret = NET_RX_DROP;
    __be16 type;

    net_timestamp_check(!netdev_tstamp_prequeue, skb);

    trace_netif_receive_skb(skb);

    orig_dev = skb->dev;

    skb_reset_network_header(skb);
    if (!skb_transport_header_was_set(skb))
        skb_reset_transport_header(skb);
    skb_reset_mac_len(skb);

    pt_prev = NULL;

another_round:
    skb->skb_iif = skb->dev->ifindex; /* Records the ingress interface index. */

    __this_cpu_inc(softnet_data.processed);

/* 1. XDP (eXpress Data Path) */
    if (static_branch_unlikely(&generic_xdp_needed_key)) {
        int ret2;
        preempt_disable();
        ret2 = do_xdp_generic(rcu_dereference(skb->dev->xdp_prog), skb);
        preempt_enable();

        if (ret2 != XDP_PASS) {
            ret = NET_RX_DROP;
            goto out;
        }
        skb_reset_mac_len(skb);
    }

    if (skb->protocol == cpu_to_be16(ETH_P_8021Q) ||
        skb->protocol == cpu_to_be16(ETH_P_8021AD)) {
        skb = skb_vlan_untag(skb);
        if (unlikely(!skb))
            goto out;
    }

    if (skb_skip_tc_classify(skb))
        goto skip_classify;

    if (pfmemalloc)
        goto skip_taps;

/* 2. Packet_ALL Type Delivery (Global and Device-Specific) */
    list_for_each_entry_rcu(ptype, &net_hotdata.ptype_all, list) {
        if (pt_prev)
            ret = deliver_skb(skb, pt_prev, orig_dev);
        pt_prev = ptype;
    }

    list_for_each_entry_rcu(ptype, &skb->dev->ptype_all, list) {
        if (pt_prev)
            ret = deliver_skb(skb, pt_prev, orig_dev);
        pt_prev = ptype;
    }

/* 3. Ingress Hooks */
skip_taps:
#ifdef CONFIG_NET_INGRESS
    if (static_branch_unlikely(&ingress_needed_key)) {
        skb = sch_handle_ingress(skb, &pt_prev, &ret, orig_dev);
        if (!skb)
            goto out;

        if (nf_ingress(skb, &pt_prev, &ret, orig_dev) < 0)
            goto out;
    }
#endif
    skb_reset_tc(skb);

skip_classify:
    if (pfmemalloc && !skb_pfmemalloc_protocol(skb))
        goto drop;

/* 4. Vlan handle */
    if (skb_vlan_tag_present(skb)) {
        if (pt_prev) {
            ret = deliver_skb(skb, pt_prev, orig_dev);
            pt_prev = NULL;
        }
        if (vlan_do_receive(&skb))
            goto another_round;
        else if (unlikely(!skb))
            goto out;
    }

/* 5. RX Handler (e.g., Bridging) */
    rx_handler = rcu_dereference(skb->dev->rx_handler); /* br_handle_frame */
    if (rx_handler) {
        if (pt_prev) {
            ret = deliver_skb(skb, pt_prev, orig_dev);
            pt_prev = NULL;
        }
        switch (rx_handler(&skb)) {
        case RX_HANDLER_CONSUMED:
            ret = NET_RX_SUCCESS;
            goto out;
        case RX_HANDLER_ANOTHER:
            goto another_round;
        case RX_HANDLER_EXACT:
            deliver_exact = true;
        case RX_HANDLER_PASS:
            break;
        default:
            BUG();
        }
    }

    if (unlikely(skb_vlan_tag_present(skb))) {
        if (skb_vlan_tag_get_id(skb))
            skb->pkt_type = PACKET_OTHERHOST;
        /* Note: we might in the future use prio bits
        * and set skb->priority like in vlan_do_receive()
        * For the time being, just ignore Priority Code Point */
        skb->vlan_tci = 0;
    }

    type = skb->protocol;

/* 6. Packet_specific Type Delivery (Global and Device-Specific) */
    if (likely(!deliver_exact)) {
        deliver_ptype_list_skb(skb, &pt_prev, orig_dev, type,
            &ptype_base[ntohs(type) & PTYPE_HASH_MASK]);
    }

    deliver_ptype_list_skb(skb, &pt_prev, orig_dev, type,
        &orig_dev->ptype_specific) {
            struct packet_type *ptype, *pt_prev = *pt;

        list_for_each_entry_rcu(ptype, ptype_list, list) {
            if (ptype->type != type)
                continue;
            if (pt_prev)
                deliver_skb(skb, pt_prev, orig_dev);
            pt_prev = ptype;
        }
        *pt = pt_prev;
    }

    if (unlikely(skb->dev != orig_dev)) {
        deliver_ptype_list_skb(skb, &pt_prev, orig_dev, type,
                &skb->dev->ptype_specific);
    }

    if (pt_prev) {
        if (unlikely(skb_orphan_frags_rx(skb, GFP_ATOMIC)))
            goto drop;
        *ppt_prev = pt_prev;
    } else {
drop:
        if (!deliver_exact)
            atomic_long_inc(&skb->dev->rx_dropped);
        else
            atomic_long_inc(&skb->dev->rx_nohandler);
        kfree_skb(skb);
        /* Jamal, now you will not able to escape explaining
        * me how you were going to use this. :-) */
        ret = NET_RX_DROP;
  }

out:
    /* The invariant here is that if *ppt_prev is not NULL
    * then skb should also be non-NULL.
    *
    * Apparently *ppt_prev assignment above holds this invariant due to
    * skb dereferencing near it. */
    *pskb = skb;
    return ret;
}

struct packet_type ip_packet_type = {
    .type = cpu_to_be16(ETH_P_IP),
    .func = ip_rcv,
};

struct packet_type arp_packet_type = {
    .type =  cpu_to_be16(ETH_P_ARP),
    .func =  arp_rcv,
};

/* if_ether.h inet_init -> dev_add_pack(&ip_packet_type) */
void dev_add_pack(struct packet_type *pt)
{
    struct list_head *head = ptype_head(pt);
    list_add_rcu(&pt->list, head);
}

struct list_head *ptype_head(const struct packet_type *pt)
{
  if (pt->type == htons(ETH_P_ALL))
      return pt->dev ? &pt->dev->ptype_all : &ptype_all;
  else
      return pt->dev ? &pt->dev->ptype_specific : &ptype_base[ntohs(pt->type) & PTYPE_HASH_MASK];
}
```

### vlan_do_receive

```c
bool vlan_do_receive(struct sk_buff **skbp)
{
    struct sk_buff *skb = *skbp;
    __be16 vlan_proto = skb->vlan_proto;
    u16 vlan_id = skb_vlan_tag_get_id(skb);
    struct net_device *vlan_dev;
    struct vlan_pcpu_stats *rx_stats;

    vlan_dev = vlan_find_dev(skb->dev, vlan_proto, vlan_id);
    if (!vlan_dev)
        return false;

    skb = *skbp = skb_share_check(skb, GFP_ATOMIC);
    if (unlikely(!skb))
        return false;

    if (unlikely(!(vlan_dev->flags & IFF_UP))) {
        kfree_skb(skb);
        *skbp = NULL;
        return false;
    }

    skb->dev = vlan_dev;
    if (unlikely(skb->pkt_type == PACKET_OTHERHOST)) {
        if (ether_addr_equal_64bits(eth_hdr(skb)->h_dest, vlan_dev->dev_addr))
        skb->pkt_type = PACKET_HOST;
    }

    if (!(vlan_dev_priv(vlan_dev)->flags & VLAN_FLAG_REORDER_HDR) &&
        !netif_is_macvlan_port(vlan_dev) &&
        !netif_is_bridge_port(vlan_dev))
    {
        unsigned int offset = skb->data - skb_mac_header(skb);

        skb_push(skb, offset);
        skb = *skbp = vlan_insert_inner_tag(skb, skb->vlan_proto,
                    skb->vlan_tci, skb->mac_len);
        if (!skb)
            return false;
        skb_pull(skb, offset + VLAN_HLEN);
        skb_reset_mac_len(skb);
    }

    skb->priority = vlan_get_ingress_priority(vlan_dev, skb->vlan_tci);
    __vlan_hwaccel_clear_tag(skb);

    rx_stats = this_cpu_ptr(vlan_dev_priv(vlan_dev)->vlan_pcpu_stats);

    u64_stats_update_begin(&rx_stats->syncp);
    rx_stats->rx_packets++;
    rx_stats->rx_bytes += skb->len;
    if (skb->pkt_type == PACKET_MULTICAST)
        rx_stats->rx_multicast++;
    u64_stats_update_end(&rx_stats->syncp);

    return true;
}

struct sk_buff *vlan_insert_inner_tag(struct sk_buff *skb,
  __be16 vlan_proto, u16 vlan_tci, unsigned int mac_len)
{
    int err;

    err = __vlan_insert_inner_tag(skb, vlan_proto, vlan_tci, mac_len);
    if (err) {
        dev_kfree_skb_any(skb);
        return NULL;
    }
    return skb;
}

int __vlan_insert_inner_tag(struct sk_buff *skb,
  __be16 vlan_proto, u16 vlan_tci, unsigned int mac_len)
{
    struct vlan_ethhdr *veth;

    if (skb_cow_head(skb, VLAN_HLEN) < 0)
        return -ENOMEM;

    skb_push(skb, VLAN_HLEN);

    /* Move the mac header sans proto to the beginning of the new header. */
    if (likely(mac_len > ETH_TLEN))
        memmove(skb->data, skb->data + VLAN_HLEN, mac_len - ETH_TLEN);
    skb->mac_header -= VLAN_HLEN;

    veth = (struct vlan_ethhdr *)(skb->data + mac_len - ETH_HLEN);

    /* first, the ethernet type */
    if (likely(mac_len >= ETH_TLEN)) {
        veth->h_vlan_proto = vlan_proto;
    } else {
        veth->h_vlan_encapsulated_proto = skb->protocol;
    }

    /* now, the TCI */
    veth->h_vlan_TCI = htons(vlan_tci);

    return 0;
}
```

### ref_br_handle_frame

[:link: br_handle_frame](#br_handle_frame)

## ip layer rx

```c
/*1. pre nf
  2. route
  2. defragment
  3. in nf */
int ip_rcv(struct sk_buff *skb, struct net_device *dev,
  struct packet_type *pt, struct net_device *orig_dev)
{
    const struct iphdr *iph;
    struct net *net;
    u32 len;

    net = dev_net(dev);
    iph = ip_hdr(skb);
    len = ntohs(iph->tot_len);
    skb->transport_header = skb->network_header + iph->ihl*4;

    return NF_HOOK(NFPROTO_IPV4, NF_INET_PRE_ROUTING,
        net, NULL, skb, dev, NULL, ip_rcv_finish);
}

int ip_rcv_finish(struct net *net, struct sock *sk, struct sk_buff *skb)
{
    struct net_device *dev = skb->dev;
    int ret;

    ret = ip_rcv_finish_core(net, sk, skb, dev, NULL);
    if (ret != NET_RX_DROP) {
        ret = dst_input(skb) {
            return skb_dst(skb)->input(skb);
        }
    }

    return ret;
}

int ip_rcv_finish_core(struct net *net,
    struct sk_buff *skb, struct net_device *dev,
    const struct sk_buff *hint)
{
    const struct iphdr *iph = ip_hdr(skb);
    int err, drop_reason;
    struct rtable *rt;

/* 1. Routing Hint Check
 * Attempts to use a routing hint (from a previous packet) to speed up routing lookup. */
    if (ip_can_use_hint(skb, iph, hint)) {
        drop_reason = ip_route_use_hint(skb, iph->daddr, iph->saddr, ip4h_dscp(iph), dev, hint);
        if (unlikely(drop_reason))
            goto drop_error;
    }

/* 2. Early Demultiplexing
 * Performs early demultiplexing to direct packets to the correct socket before full routing. */

    drop_reason = SKB_DROP_REASON_NOT_SPECIFIED;
    if (READ_ONCE(net->ipv4.sysctl_ip_early_demux)
        && !skb_dst(skb) && !skb->sk && !ip_is_fragment(iph)) {

        switch (iph->protocol) {
        case IPPROTO_TCP:
            if (READ_ONCE(net->ipv4.sysctl_tcp_early_demux)) {
                tcp_v4_early_demux(skb); /* tries to match the packet to a listening socket. */

                /* must reload iph, skb->head might have changed */
                iph = ip_hdr(skb);
            }
            break;
        case IPPROTO_UDP:
            if (READ_ONCE(net->ipv4.sysctl_udp_early_demux)) {
                err = udp_v4_early_demux(skb); /* tries to match the packet to a listening socket. */
                if (unlikely(err))
                    goto drop_error;

                /* must reload iph, skb->head might have changed */
                iph = ip_hdr(skb);
            }
            break;
        }
    }

/* 3. Routing Lookup
 * Sets skb->dst with the route (struct rtable).
 * Determines the packet’s routing path if no destination is set. */
    if (!skb_valid_dst(skb)) {
        drop_reason = ip_route_input_noref(skb, iph->daddr, iph->saddr, ip4h_dscp(iph), dev);
        if (unlikely(drop_reason))
            goto drop_error;
        drop_reason = SKB_DROP_REASON_NOT_SPECIFIED;
    } else {
        struct in_device *in_dev = __in_dev_get_rcu(dev);

        if (in_dev && IN_DEV_ORCONF(in_dev, NOPOLICY))
            IPCB(skb)->flags |= IPSKB_NOPOLICY;
    }

/* 4. Traffic Class Accounting */

#ifdef CONFIG_IP_ROUTE_CLASSID
    if (unlikely(skb_dst(skb)->tclassid)) {
        struct ip_rt_acct *st = this_cpu_ptr(ip_rt_acct);
        u32 idx = skb_dst(skb)->tclassid;
        st[idx&0xFF].o_packets++;
        st[idx&0xFF].o_bytes += skb->len;
        st[(idx>>16)&0xFF].i_packets++;
        st[(idx>>16)&0xFF].i_bytes += skb->len;
    }
#endif

/* 5. IP Options Handling */
    if (iph->ihl > 5 && ip_rcv_options(skb, dev))
        goto drop;

/* 6. Multicast/Broadcast Handling */
    rt = skb_rtable(skb);
    if (rt->rt_type == RTN_MULTICAST) {
        __IP_UPD_PO_STATS(net, IPSTATS_MIB_INMCAST, skb->len);
    } else if (rt->rt_type == RTN_BROADCAST) {
        __IP_UPD_PO_STATS(net, IPSTATS_MIB_INBCAST, skb->len);
    } else if (skb->pkt_type == PACKET_BROADCAST ||
        skb->pkt_type == PACKET_MULTICAST) {
        struct in_device *in_dev = __in_dev_get_rcu(dev);

        /* RFC 1122 3.3.6:
        *
        *   When a host sends a datagram to a link-layer broadcast
        *   address, the IP destination address MUST be a legal IP
        *   broadcast or IP multicast address.
        *
        *   A host SHOULD silently discard a datagram that is received
        *   via a link-layer broadcast (see Section 2.4) but does not
        *   specify an IP multicast or broadcast destination address.
        *
        * This doesn't explicitly say L2 *broadcast*, but broadcast is
        * in a way a form of multicast and the most common use case for
        * this is 802.11 protecting against cross-station spoofing (the
        * so-called "hole-196" attack) so do it for both. */
        if (in_dev && IN_DEV_ORCONF(in_dev, DROP_UNICAST_IN_L2_MULTICAST)) {
            drop_reason = SKB_DROP_REASON_UNICAST_IN_L2_MULTICAST;
            goto drop;
        }
    }

    return NET_RX_SUCCESS;

drop:
    kfree_skb_reason(skb, drop_reason);
    return NET_RX_DROP;

drop_error:
    if (drop_reason == SKB_DROP_REASON_IP_RPFILTER)
        __NET_INC_STATS(net, LINUX_MIB_IPRPFILTER);
    goto drop;
}

/* see at rt_dst_alloc(), rt.dst.input -> */
int ip_local_deliver(struct sk_buff *skb)
{
  struct net *net = dev_net(skb->dev);

  if (ip_is_fragment(ip_hdr(skb))) {
    if (ip_defrag(net, skb, IP_DEFRAG_LOCAL_DELIVER))
      return 0;
  }

  return NF_HOOK(NFPROTO_IPV4, NF_INET_LOCAL_IN,
           net, NULL, skb, skb->dev, NULL,
           ip_local_deliver_finish);
}

int ip_local_deliver_finish(struct net *net, struct sock *sk, struct sk_buff *skb)
{
    __skb_pull(skb, skb_network_header_len(skb));

    int protocol = ip_hdr(skb)->protocol;
    const struct net_protocol *ipprot;

    ipprot = rcu_dereference(inet_protos[protocol]);
    if (ipprot) {
        int ret;
        ret = ipprot->handler(skb);
    }
}
```

## tcp layer rx

```c
struct net_protocol *inet_protos[MAX_INET_PROTOS];

struct net_protocol {
    int   (*early_demux)(struct sk_buff *skb);
    int   (*early_demux_handler)(struct sk_buff *skb);
    int   (*handler)(struct sk_buff *skb);
    void  (*err_handler)(struct sk_buff *skb, u32 info);
};

int inet_add_protocol(const struct net_protocol *prot, unsigned char protocol)
{
    return !cmpxchg((const struct net_protocol **)&inet_protos[protocol],
        NULL, prot) ? 0 : -1;
}

int inet_init(void)
{
    inet_add_protocol(&udp_protocol, IPPROTO_UDP);
    inet_add_protocol(&tcp_protocol, IPPROTO_TCP);
}

struct net_protocol tcp_protocol = {
    .early_demux          =  tcp_v4_early_demux,
    .early_demux_handler  =  tcp_v4_early_demux,
    .handler              =  tcp_v4_rcv,
    .err_handler          =  tcp_v4_err,
    .no_policy            =  1,
    .netns_ok             =  1,
    .icmp_strict_tag_validation = 1,
};

struct net_protocol udp_protocol = {
    .early_demux          =  udp_v4_early_demux,
    .early_demux_handler  =  udp_v4_early_demux,
    .handler              =  udp_rcv,
    .err_handler          =  udp_err,
    .no_policy            =  1,
    .netns_ok             =  1,
};
```

### tcp_v4_rcv-TCP_NEW_SYN_RECV

```c
int tcp_v4_rcv(struct sk_buff *skb)
{
    struct net *net = dev_net(skb->dev);
    int sdif = inet_sdif(skb);
    const struct iphdr *iph;
    const struct tcphdr *th;
    bool refcounted;
    struct sock *sk;
    int ret;

    if (skb->pkt_type != PACKET_HOST)
        goto discard_it;

    th = (const struct tcphdr *)skb->data;
    iph = ip_hdr(skb);

lookup:
    sk = __inet_lookup_skb(
        net->ipv4.tcp_death_row.hashinfo,
        skb, __tcp_hdrlen(th), th->source,
        th->dest, sdif, &refcounted);
    if (!sk)
        goto no_tcp_socket;

    if (sk->sk_state == TCP_TIME_WAIT)
        goto do_time_wait;

    if (sk->sk_state == TCP_NEW_SYN_RECV) {
        struct request_sock *req = inet_reqsk(sk);
        bool req_stolen = false;
        struct sock *newsk;

        sk = req->rsk_listener;
        if (unlikely(tcp_v4_inbound_md5_hash(sk, skb))) {
            sk_drops_add(sk, skb);
            reqsk_put(req);
            goto discard_it;
        }
        if (tcp_checksum_complete(skb)) {
            reqsk_put(req);
            goto csum_error;
        }
        if (unlikely(sk->sk_state != TCP_LISTEN)) {
            nsk = reuseport_migrate_sock(sk, req_to_sk(req), skb);
            if (!nsk) {
                inet_csk_reqsk_queue_drop_and_put(sk, req);
                goto lookup;
            }
            sk = nsk;
            /* reuseport_migrate_sock() has already held one sk_refcnt
            * before returning. */
        } else {
            /* We own a reference on the listener, increase it again
            * as we might lose it too soon. */
            sock_hold(sk);
        }

        refcounted = true;
        newsk = NULL;
        if (!tcp_filter(sk, skb)) {
            th = (const struct tcphdr *)skb->data;
            iph = ip_hdr(skb);
            tcp_v4_fill_cb(skb, iph, th);
            newsk = tcp_check_req(sk, skb, req, false, &req_stolen);
        }

        if (!newsk) {
            reqsk_put(req);
            if (req_stolen) {
                /* Another cpu got exclusive access to req
                * and created a full blown socket.
                * Try to feed this packet to this socket
                * instead of discarding it. */
                tcp_v4_restore_cb(skb);
                sock_put(sk);
                goto lookup;
            }
            goto discard_and_relse;
        }

        if (newsk == sk) {
            reqsk_put(req);
            tcp_v4_restore_cb(skb);
        } else {
            drop_reason = tcp_child_process(sk, nsk, skb);
            if (drop_reason) {
                enum sk_rst_reason rst_reason;

                rst_reason = sk_rst_convert_drop_reason(drop_reason);
                tcp_v4_send_reset(nsk, skb, rst_reason);
                goto discard_and_relse;
            }
            sock_put(sk);
            return 0;
        }
    }

process:
    drop_reason = tcp_inbound_hash(sk, NULL, skb, &iph->saddr, &iph->daddr,
                    AF_INET, dif, sdif);
    if (drop_reason)
        goto discard_and_relse;

    if (tcp_filter(sk, skb))
        goto discard_and_relse;

    th = (const struct tcphdr *)skb->data;
    iph = ip_hdr(skb);
    tcp_v4_fill_cb(skb, iph, th);

    skb->dev = NULL;

    if (sk->sk_state == TCP_LISTEN) {
        ret = tcp_v4_do_rcv(sk, skb);
        goto put_and_return;
    }

    sk_incoming_cpu_update(sk) {
        int cpu = raw_smp_processor_id();

        if (unlikely(READ_ONCE(sk->sk_incoming_cpu) != cpu))
            WRITE_ONCE(sk->sk_incoming_cpu, cpu);
    }

    bh_lock_sock_nested(sk);
    tcp_segs_in(tcp_sk(sk), skb);

    ret = 0;
    if (!sock_owned_by_user(sk)) {
        ret = tcp_v4_do_rcv(sk, skb);
    } else {
        if (tcp_add_backlog(sk, skb, &drop_reason))
            goto discard_and_relse;
    }
    bh_unlock_sock(sk);

put_and_return:
    if (refcounted)
        sock_put(sk);

    return ret;

no_tcp_socket:
    if (!xfrm4_policy_check(NULL, XFRM_POLICY_IN, skb))
        goto discard_it;

    tcp_v4_fill_cb(skb, iph, th);

    if (tcp_checksum_complete(skb)) {
csum_error:
        __TCP_INC_STATS(net, TCP_MIB_CSUMERRORS);
bad_packet:
        __TCP_INC_STATS(net, TCP_MIB_INERRS);
    } else {
        tcp_v4_send_reset(NULL, skb);
    }

discard_it:
  /* Discard frame. */
    kfree_skb(skb);
    return 0;

discard_and_relse:
    sk_drops_add(sk, skb);
    if (refcounted)
        sock_put(sk);
    goto discard_it;

    if (!xfrm4_policy_check(NULL, XFRM_POLICY_IN, skb)) {
        inet_twsk_put(inet_twsk(sk));
        goto discard_it;
    }

    tcp_v4_fill_cb(skb, iph, th);

    if (tcp_checksum_complete(skb)) {
        inet_twsk_put(inet_twsk(sk));
        goto csum_error;
    }

    switch (tcp_timewait_state_process(inet_twsk(sk), skb, th)) {
    case TCP_TW_SYN: {
        struct sock *sk2 = inet_lookup_listener(dev_net(skb->dev),
                &tcp_hashinfo, skb,
                __tcp_hdrlen(th),
                iph->saddr, th->source,
                iph->daddr, th->dest,
                inet_iif(skb),
                sdif);
        if (sk2) {
            inet_twsk_deschedule_put(inet_twsk(sk));
            sk = sk2;
            tcp_v4_restore_cb(skb);
            refcounted = false;
            goto process;
        }
    }
    /* to ACK */
    /* fall through */
    case TCP_TW_ACK:
        tcp_v4_timewait_ack(sk, skb);
        break;
    case TCP_TW_RST:
        tcp_v4_send_reset(sk, skb);
        inet_twsk_deschedule_put(inet_twsk(sk));
        goto discard_it;
    case TCP_TW_SUCCESS:;
    }

    goto discard_it;
}

int tcp_child_process(struct sock *parent, struct sock *child,
          struct sk_buff *skb)
{
    int ret = 0;
    int state = child->sk_state;

    /* record NAPI ID of child */
    sk_mark_napi_id(child, skb);

    tcp_segs_in(tcp_sk(child), skb);
    if (!sock_owned_by_user(child)) {
        ret = tcp_rcv_state_process(child, skb);
        /* Wakeup parent, send SIGIO */
        if (state == TCP_SYN_RECV && child->sk_state != state)
            parent->sk_data_ready(parent);
    } else {
        /* Alas, it is possible again, because we do lookup
        * in main socket hash table and lock on listening
        * socket does not protect us more. */
        __sk_add_backlog(child, skb);
    }

    bh_unlock_sock(child);
    sock_put(child);
    return ret;
}
```

### tcp_v4_do_rcv-TCP_ESTABLISHED

```c
int tcp_v4_do_rcv(struct sock *sk, struct sk_buff *skb)
{
    struct sock *rsk;

    if (sk->sk_state == TCP_ESTABLISHED) { /* Fast path */
        struct dst_entry *dst = sk->sk_rx_dst;

        sock_rps_save_rxhash(sk, skb);
        sk_mark_napi_id(sk, skb);
        if (dst) {
            if (inet_sk(sk)->rx_dst_ifindex != skb->skb_iif || !dst->ops->check(dst, 0)) {
                dst_release(dst);
                sk->sk_rx_dst = NULL;
            }
        }
        tcp_rcv_established(sk, skb);
        return 0;
    }

    if (tcp_checksum_complete(skb))
        goto csum_err;

    if (sk->sk_state == TCP_LISTEN) {
        struct sock *newsk = tcp_v4_cookie_check(sk, skb);

        if (!newsk)
            goto discard;
        if (newsk != sk) {
            if (tcp_child_process(sk, newsk, skb)) {
                rsk = newsk;
                goto reset;
            }
            return 0;
        }
    } else
        sock_rps_save_rxhash(sk, skb);

    if (tcp_rcv_state_process(sk, skb)) {
        rsk = sk;
        goto reset;
    }
    return 0;

reset:
    tcp_v4_send_reset(rsk, skb);
discard:
    kfree_skb(skb);
    /* Be careful here. If this function gets more complicated and
    * gcc suffers from register pressure on the x86, sk (in %ebx)
    * might be destroyed here. This current version compiles correctly,
    * but you have been warned. */
    return 0;

csum_err:
    TCP_INC_STATS(sock_net(sk), TCP_MIB_CSUMERRORS);
    TCP_INC_STATS(sock_net(sk), TCP_MIB_INERRS);
    goto discard;
}

void tcp_rcv_established(struct sock *sk, struct sk_buff *skb)
{
    const struct tcphdr *th = (const struct tcphdr *)skb->data;
    struct tcp_sock *tp = tcp_sk(sk);
    unsigned int len = skb->len;

    /* TCP congestion window tracking */
    trace_tcp_probe(sk, skb);

    tcp_mstamp_refresh(tp);
    if (unlikely(!sk->sk_rx_dst))
        inet_csk(sk)->icsk_af_ops->sk_rx_dst_set(sk, skb);

    tp->rx_opt.saw_tstamp = 0;

    if ((tcp_flag_word(th) & TCP_HP_BITS) == tp->pred_flags &&
        TCP_SKB_CB(skb)->seq == tp->rcv_nxt &&
        !after(TCP_SKB_CB(skb)->ack_seq, tp->snd_nxt))
    {
        int tcp_header_len = tp->tcp_header_len;

        /* Timestamp header prediction: tcp_header_len
        * is automatically equal to th->doff*4 due to pred_flags
        * match. */

        /* Check timestamp */
        if (tcp_header_len == sizeof(struct tcphdr) + TCPOLEN_TSTAMP_ALIGNED) {
            /* No? Slow path! */
            if (!tcp_parse_aligned_timestamp(tp, th))
                goto slow_path;

            /* If PAWS failed, check it more carefully in slow path */
            if ((s32)(tp->rx_opt.rcv_tsval - tp->rx_opt.ts_recent) < 0)
                goto slow_path;

            /* DO NOT update ts_recent here, if checksum fails
            * and timestamp was corrupted part, it will result
            * in a hung connection since we will drop all
            * future packets due to the PAWS test. */
        }

        if (len <= tcp_header_len) {
            /* Bulk data transfer: sender */
            if (len == tcp_header_len) {
                /* Predicted packet is in window by definition.
                * seq == rcv_nxt and rcv_wup <= rcv_nxt.
                * Hence, check seq<=rcv_wup reduces to: */
                if (tcp_header_len == (sizeof(struct tcphdr) + TCPOLEN_TSTAMP_ALIGNED)
                    && tp->rcv_nxt == tp->rcv_wup) {
                    tcp_store_ts_recent(tp);
                }

                /* We know that such packets are checksummed
                * on entry. */
                tcp_ack(sk, skb, 0);
                __kfree_skb(skb);
                tcp_data_snd_check(sk);
                /* When receiving pure ack in fast path, update
                * last ts ecr directly instead of calling
                * tcp_rcv_rtt_measure_ts() */
                tp->rcv_rtt_last_tsecr = tp->rx_opt.rcv_tsecr;
                return;
            } else { /* Header too small */
                TCP_INC_STATS(sock_net(sk), TCP_MIB_INERRS);
                goto discard;
            }
        } else {
            int eaten = 0;
            bool fragstolen = false;

            if (tcp_checksum_complete(skb))
                goto csum_error;

            if ((int)skb->truesize > sk->sk_forward_alloc)
                goto step5;

            /* Predicted packet is in window by definition.
            * seq == rcv_nxt and rcv_wup <= rcv_nxt.
            * Hence, check seq<=rcv_wup reduces to: */
            if (tcp_header_len == (sizeof(struct tcphdr) + TCPOLEN_TSTAMP_ALIGNED)
                && tp->rcv_nxt == tp->rcv_wup) {
                tcp_store_ts_recent(tp);
            }

            tcp_rcv_rtt_measure_ts(sk, skb);

            NET_INC_STATS(sock_net(sk), LINUX_MIB_TCPHPHITS);

            /* Bulk data transfer: receiver */
            tcp_cleanup_skb(skb);
            __skb_pull(skb, tcp_header_len);
            eaten = tcp_queue_rcv(sk, skb, tcp_header_len, &fragstolen);

            tcp_event_data_recv(sk, skb);

            if (TCP_SKB_CB(skb)->ack_seq != tp->snd_una) {
                /* Well, only one small jumplet in fast path... */
                tcp_ack(sk, skb, FLAG_DATA);
                tcp_data_snd_check(sk);
                if (!inet_csk_ack_scheduled(sk))
                    goto no_ack;
            } else {
                tcp_update_wl(tp, TCP_SKB_CB(skb)->seq);
            }

            __tcp_ack_snd_check(sk, 0);
no_ack:
            if (eaten)
                kfree_skb_partial(skb, fragstolen);
            tcp_data_ready(sk);
            return;
        }
    }

slow_path:
    if (len < (th->doff << 2) || tcp_checksum_complete(skb))
        goto csum_error;

    if (!th->ack && !th->rst && !th->syn)
        goto discard;

    /* Standard slow path. */
    if (!tcp_validate_incoming(sk, skb, th, 1))
        return;

step5:
    if (tcp_ack(sk, skb, FLAG_SLOWPATH | FLAG_UPDATE_TS_RECENT) < 0)
        goto discard;

    tcp_rcv_rtt_measure_ts(sk, skb);

    /* Process urgent data. */
    tcp_urg(sk, skb, th);

    /* step 7: process the segment text */
    tcp_data_queue(sk, skb);

    tcp_data_snd_check(sk);
    tcp_ack_snd_check(sk);
    return;

csum_error:
    TCP_INC_STATS(sock_net(sk), TCP_MIB_CSUMERRORS);
    TCP_INC_STATS(sock_net(sk), TCP_MIB_INERRS);

discard:
    tcp_drop(sk, skb);
}
```

### tcp_rcv_state_process

```c
int tcp_rcv_state_process(struct sock *sk, struct sk_buff *skb)
{
    struct tcp_sock *tp = tcp_sk(sk);
    struct inet_connection_sock *icsk = inet_csk(sk);
    const struct tcphdr *th = tcp_hdr(skb);
    struct request_sock *req;
    int queued = 0;
    bool acceptable;

    switch (sk->sk_state) {
    case TCP_CLOSE:
        goto discard;

    case TCP_LISTEN:
        if (th->ack)
            return 1;

        if (th->rst)
            goto discard;

        if (th->syn) {
            if (th->fin)
                goto discard;
            /* It is possible that we process SYN packets from backlog,
             * so we need to make sure to disable BH and RCU right there. */
            rcu_read_lock();
            local_bh_disable();
            acceptable = icsk->icsk_af_ops->conn_request(sk, skb) >= 0;
            local_bh_enable();
            rcu_read_unlock();

            if (!acceptable)
                return 1;
            consume_skb(skb);
            return 0;
        }
        goto discard;

    case TCP_SYN_SENT:
        tp->rx_opt.saw_tstamp = 0;
        tcp_mstamp_refresh(tp);
        queued = tcp_rcv_synsent_state_process(sk, skb, th);
        if (queued >= 0)
            return queued;

        /* Do step6 onward by hand. */
        tcp_urg(sk, skb, th);
        __kfree_skb(skb);
        tcp_data_snd_check(sk);
        return 0;
    }

    tcp_mstamp_refresh(tp);
    tp->rx_opt.saw_tstamp = 0;
    req = tp->fastopen_rsk;
    if (req) {
        bool req_stolen;
        if (!tcp_check_req(sk, skb, req, true, &req_stolen))
            goto discard;
    }

    if (!th->ack && !th->rst && !th->syn)
        goto discard;

    /* Step 1 2 3 4 */
    if (!tcp_validate_incoming(sk, skb, th, 0))
        return 0;

    /* Step 5: check the ACK field */
    acceptable = tcp_ack(sk, skb, FLAG_SLOWPATH | FLAG_UPDATE_TS_RECENT | FLAG_NO_CHALLENGE_ACK) > 0;

    if (!acceptable) {
        if (sk->sk_state == TCP_SYN_RECV)
            return 1;  /* send one RST */
        tcp_send_challenge_ack(sk, skb);
        goto discard;
    }

    switch (sk->sk_state) {
    case TCP_SYN_RECV:
        tp->delivered++; /* SYN-ACK delivery isn't tracked in tcp_ack */
        if (!tp->srtt_us)
            tcp_synack_rtt_meas(sk, req);

        /* Once we leave TCP_SYN_RECV, we no longer need req
        * so release it. */
        if (req) {
            inet_csk(sk)->icsk_retransmits = 0;
            reqsk_fastopen_remove(sk, req, false);
            /* Re-arm the timer because data may have been sent out.
            * This is similar to the regular data transmission case
            * when new data has just been ack'ed.
            *
            * (TFO) - we could try to be more aggressive and
            * retransmitting any data sooner based on when they
            * are sent out. */
            tcp_rearm_rto(sk);
        } else {
            tcp_init_transfer(sk, BPF_SOCK_OPS_PASSIVE_ESTABLISHED_CB);
            tp->copied_seq = tp->rcv_nxt;
        }
        tcp_set_state(sk, TCP_ESTABLISHED);
        sk->sk_state_change(sk);

        /* Note, that this wakeup is only for marginal crossed SYN case.
        * Passively open sockets are not waked up, because
        * sk->sk_sleep == NULL and sk->sk_socket == NULL. */
        if (sk->sk_socket)
            sk_wake_async(sk, SOCK_WAKE_IO, POLL_OUT);

        tp->snd_una = TCP_SKB_CB(skb)->ack_seq;
        tp->snd_wnd = ntohs(th->window) << tp->rx_opt.snd_wscale;
        tcp_init_wl(tp, TCP_SKB_CB(skb)->seq);

        if (tp->rx_opt.tstamp_ok)
            tp->advmss -= TCPOLEN_TSTAMP_ALIGNED;

        if (!inet_csk(sk)->icsk_ca_ops->cong_control)
            tcp_update_pacing_rate(sk);

        /* Prevent spurious tcp_cwnd_restart() on first data packet */
        tp->lsndtime = tcp_jiffies32;

        tcp_initialize_rcv_mss(sk);
        tcp_fast_path_on(tp);
        break;

    case TCP_FIN_WAIT1: {
        int tmo;

        /* If we enter the TCP_FIN_WAIT1 state and we are a
        * Fast Open socket and this is the first acceptable
        * ACK we have received, this would have acknowledged
        * our SYNACK so stop the SYNACK timer. */
        if (req) {
            /* We no longer need the request sock. */
            reqsk_fastopen_remove(sk, req, false);
            tcp_rearm_rto(sk);
        }
        if (tp->snd_una != tp->write_seq)
            break;

        tcp_set_state(sk, TCP_FIN_WAIT2);
        sk->sk_shutdown |= SEND_SHUTDOWN;

        sk_dst_confirm(sk);

        if (!sock_flag(sk, SOCK_DEAD)) {
            /* Wake up lingering close() */
            sk->sk_state_change(sk);
            break;
        }

        if (tp->linger2 < 0) {
            tcp_done(sk);
            return 1;
        }
        if (TCP_SKB_CB(skb)->end_seq != TCP_SKB_CB(skb)->seq
            && after(TCP_SKB_CB(skb)->end_seq - th->fin, tp->rcv_nxt))
        {
            /* Receive out of order FIN after close() */
            if (tp->syn_fastopen && th->fin)
            tcp_fastopen_active_disable(sk);
            tcp_done(sk);
            return 1;
        }

        tmo = tcp_fin_time(sk);
        if (tmo > TCP_TIMEWAIT_LEN) {
            inet_csk_reset_keepalive_timer(sk, tmo - TCP_TIMEWAIT_LEN);
        } else if (th->fin || sock_owned_by_user(sk)) {
            /* Bad case. We could lose such FIN otherwise.
            * It is not a big problem, but it looks confusing
            * and not so rare event. We still can lose it now,
            * if it spins in bh_lock_sock(), but it is really
            * marginal case. */
            inet_csk_reset_keepalive_timer(sk, tmo);
        } else {
            tcp_time_wait(sk, TCP_FIN_WAIT2, tmo);
            goto discard;
        }
        break;
    }

    case TCP_CLOSING:
        if (tp->snd_una == tp->write_seq) {
            tcp_time_wait(sk, TCP_TIME_WAIT, 0);
            goto discard;
        }
        break;

    case TCP_LAST_ACK:
        if (tp->snd_una == tp->write_seq) {
            tcp_update_metrics(sk);
            tcp_done(sk);
            goto discard;
        }
        break;
    }

    /* Step 6: check the URG bit */
    tcp_urg(sk, skb, th);

    /* Step 7: process the segment text */
    switch (sk->sk_state) {
    case TCP_CLOSE_WAIT:
    case TCP_CLOSING:
    case TCP_LAST_ACK:
        if (!before(TCP_SKB_CB(skb)->seq, tp->rcv_nxt))
            break;
        /* fall through */
    case TCP_FIN_WAIT1:
    case TCP_FIN_WAIT2:
        /* RFC 793 says to queue data in these states,
        * RFC 1122 says we MUST send a reset.
        * BSD 4.4 also does reset. */
        if (sk->sk_shutdown & RCV_SHUTDOWN) {
            if (TCP_SKB_CB(skb)->end_seq != TCP_SKB_CB(skb)->seq && after(TCP_SKB_CB(skb)->end_seq - th->fin, tp->rcv_nxt)) {
                tcp_reset(sk);
                return 1;
            }
        }
        /* Fall through */
    case TCP_ESTABLISHED:
        tcp_data_queue(sk, skb);
        queued = 1;
        break;
    }

    /* tcp_data could move socket to TIME-WAIT */
    if (sk->sk_state != TCP_CLOSE) {
        tcp_data_snd_check(sk);
        tcp_ack_snd_check(sk);
    }

    if (!queued) {
discard:
        tcp_drop(sk, skb);
    }

    return 0;
}

bool tcp_validate_incoming(
    struct sock *sk, struct sk_buff *skb,
    const struct tcphdr *th, int syn_inerr)
{
    struct tcp_sock *tp = tcp_sk(sk);
    bool rst_seq_match = false;

    /* RFC1323: H1. Apply PAWS check first. */
    if (tcp_fast_parse_options(sock_net(sk), skb, th, tp)
        && tp->rx_opt.saw_tstamp
        && tcp_paws_discard(sk, skb))
    {
        if (!th->rst) {
            if (!tcp_oow_rate_limited(sock_net(sk), skb,
                          LINUX_MIB_TCPACKSKIPPEDPAWS,
                          &tp->last_oow_ack_time))
                tcp_send_dupack(sk, skb);
            goto discard;
        }
        /* Reset is accepted even if it did not pass PAWS. */
    }

    /* Step 1: check sequence number */
    if (!tcp_sequence(tp, TCP_SKB_CB(skb)->seq, TCP_SKB_CB(skb)->end_seq)) {
        /* RFC793, page 37: "In all states except SYN-SENT, all reset
         * (RST) segments are validated by checking their SEQ-fields."
         * And page 69: "If an incoming segment is not acceptable,
         * an acknowledgment should be sent in reply (unless the RST
         * bit is set, if so drop the segment and return)". */
        if (!th->rst) {
            if (th->syn)
                goto syn_challenge;
            if (!tcp_oow_rate_limited(sock_net(sk), skb,
                          LINUX_MIB_TCPACKSKIPPEDSEQ,
                          &tp->last_oow_ack_time))
                tcp_send_dupack(sk, skb);
        } else if (tcp_reset_check(sk, skb)) {
            tcp_reset(sk);
        }
        goto discard;
    }

    /* Step 2: check RST bit */
    if (th->rst) {
        /* RFC 5961 3.2 (extend to match against (RCV.NXT - 1) after a
         * FIN and SACK too if available):
         * If seq num matches RCV.NXT or (RCV.NXT - 1) after a FIN, or
         * the right-most SACK block,
         * then
         *     RESET the connection
         * else
         *     Send a challenge ACK */
        if (TCP_SKB_CB(skb)->seq == tp->rcv_nxt || tcp_reset_check(sk, skb)) {
            rst_seq_match = true;
        } else if (tcp_is_sack(tp) && tp->rx_opt.num_sacks > 0) {
            struct tcp_sack_block *sp = &tp->selective_acks[0];
            int max_sack = sp[0].end_seq;
            int this_sack;

            for (this_sack = 1; this_sack < tp->rx_opt.num_sacks; ++this_sack) {
                max_sack = after(sp[this_sack].end_seq, max_sack) ? sp[this_sack].end_seq : max_sack;
            }

            if (TCP_SKB_CB(skb)->seq == max_sack)
                rst_seq_match = true;
        }

        if (rst_seq_match)
            tcp_reset(sk);
        else {
            /* Disable TFO if RST is out-of-order
             * and no data has been received
             * for current active TFO socket */
            if (tp->syn_fastopen && !tp->data_segs_in && sk->sk_state == TCP_ESTABLISHED)
                tcp_fastopen_active_disable(sk);
            tcp_send_challenge_ack(sk, skb);
        }
        goto discard;
    }

    /* Step 3: check security and precedence [ignored] */

    /* Step 4: Check for a SYN
     * RFC 5961 4.2 : Send a challenge ack */
    if (th->syn) {
syn_challenge:
        if (syn_inerr)
            TCP_INC_STATS(sock_net(sk), TCP_MIB_INERRS);
        tcp_send_challenge_ack(sk, skb);
        goto discard;
    }

    return true;

discard:
    tcp_drop(sk, skb);
    return false;
}
```

### tcp_data_queue

```c
/* tcp_rcv_established ->
 * Queues:
 * 1. backlog, push back when user is reading
 * 2. sk_receive_queue, push back when user not reading
 * 3. out_of_order_queue */
void tcp_data_queue(struct sock *sk, struct sk_buff *skb)
{
    struct tcp_sock *tp = tcp_sk(sk);
    bool fragstolen = false;

/* 1. Normal: [seq = rcv_next < end_seq < win] */
    if (TCP_SKB_CB(skb)->seq == tp->rcv_nxt) {
        if (tcp_receive_window(tp) == 0) {
            goto out_of_window;
        }

    queue_and_out:
        if (tcp_try_rmem_schedule(sk, skb, skb->truesize)) {
            /* TODO: maybe ratelimit these WIN 0 ACK ? */
            inet_csk(sk)->icsk_ack.pending |= (ICSK_ACK_NOMEM | ICSK_ACK_NOW);
            inet_csk_schedule_ack(sk);
            sk->sk_data_ready(sk);

            if (skb_queue_len(&sk->sk_receive_queue) && skb->len) {
                reason = SKB_DROP_REASON_PROTO_MEM;
                NET_INC_STATS(sock_net(sk), LINUX_MIB_TCPRCVQDROP);
                goto drop;
            }
            sk_forced_mem_schedule(sk, skb->truesize);
        }

        eaten = tcp_queue_rcv(sk, skb, 0, &fragstolen);
        if (skb->len)
            tcp_event_data_recv(sk, skb);
        if (TCP_SKB_CB(skb)->tcp_flags & TCPHDR_FIN)
            tcp_fin(sk);

        /* checks to see if we can put data from the
         * out_of_order queue into the receive_queue */
        if (!RB_EMPTY_ROOT(&tp->out_of_order_queue)) {
            tcp_ofo_queue(sk);

            /* RFC5681. 4.2. SHOULD send immediate ACK, when
             * gap in queue is filled. */
            if (RB_EMPTY_ROOT(&tp->out_of_order_queue))
                inet_csk(sk)->icsk_ack.pending |= ICSK_ACK_NOW;
        }

        if (tp->rx_opt.num_sacks)
            tcp_sack_remove(tp);

        tcp_fast_path_check(sk);

        if (eaten > 0)
            kfree_skb_partial(skb, fragstolen);
        if (!sock_flag(sk, SOCK_DEAD))
            /* wake up user blokced and waiting for data at: sk_wait_event */
            tcp_data_ready(sk);
        return;
    }

/* 2. DSACK: [seq < end_seq < rcv_next < win] */
    if (!after(TCP_SKB_CB(skb)->end_seq, tp->rcv_nxt)) {
        tcp_rcv_spurious_retrans(sk, skb);
        /* A retransmit, 2nd most common case.  Force an immediate ack. */
        tcp_dsack_set(sk, TCP_SKB_CB(skb)->seq, TCP_SKB_CB(skb)->end_seq);

    out_of_window:
        tcp_enter_quickack_mode(sk, TCP_MAX_QUICKACKS);
        inet_csk_schedule_ack(sk);
    drop:
        tcp_drop(sk, skb);
        return;
    }

/* 3. Out of Window: [rcv_next < win < seq < end_seq] */
    if (!before(TCP_SKB_CB(skb)->seq, tp->rcv_nxt + tcp_receive_window(tp)))
        goto out_of_window;

/* 4. Out of Window: [seq < rcv_next < end_seq < win] */
    if (before(TCP_SKB_CB(skb)->seq, tp->rcv_nxt)) {
        /* Partial packet, seq < rcv_next < end_seq */
        tcp_dsack_set(sk, TCP_SKB_CB(skb)->seq, tp->rcv_nxt);

        /* If window is closed, drop tail of packet. But after
        * remembering D-SACK for its head made in previous line. */
        if (!tcp_receive_window(tp)) {
            goto out_of_window;
        }
        goto queue_and_out;
    }

/* 5. Out of order: [rcv_next < seq < end_seq < win] */
    tcp_data_queue_ofo(sk, skb);
}

int tcp_queue_rcv(
    struct sock *sk, struct sk_buff *skb, int hdrlen, bool *fragstolen)
{
    int eaten;
    struct sk_buff *tail = skb_peek_tail(&sk->sk_receive_queue);

    __skb_pull(skb, hdrlen);
    eaten = (tail && tcp_try_coalesce(sk, tail, skb, fragstolen)) ? 1 : 0;
    tcp_rcv_nxt_update(tcp_sk(sk), TCP_SKB_CB(skb)->end_seq);
    if (!eaten) {
        __skb_queue_tail(&sk->sk_receive_queue, skb);
        skb_set_owner_r(skb, sk);
    }
    return eaten;
}

void tcp_data_queue_ofo(struct sock *sk, struct sk_buff *skb)
{
    struct tcp_sock *tp = tcp_sk(sk);
    struct rb_node **p, *parent;
    struct sk_buff *skb1;
    u32 seq, end_seq;
    bool fragstolen;

    tcp_ecn_check_ce(sk, skb);

    if (unlikely(tcp_try_rmem_schedule(sk, skb, skb->truesize))) {
        sk->sk_data_ready(sk); /* sock_def_readable */
        tcp_drop(sk, skb);
        return;
    }

    /* Disable header prediction. */
    tp->pred_flags = 0;
    inet_csk_schedule_ack(sk);

    seq = TCP_SKB_CB(skb)->seq;
    end_seq = TCP_SKB_CB(skb)->end_seq;

    p = &tp->out_of_order_queue.rb_node;
    if (RB_EMPTY_ROOT(&tp->out_of_order_queue)) {
        /* Initial out of order segment, build 1 SACK. */
        if (tcp_is_sack(tp)) {
            tp->rx_opt.num_sacks = 1;
            tp->selective_acks[0].start_seq = seq;
            tp->selective_acks[0].end_seq = end_seq;
        }
        rb_link_node(&skb->rbnode, NULL, p);
        rb_insert_color(&skb->rbnode, &tp->out_of_order_queue);
        tp->ooo_last_skb = skb;
        goto end;
    }

    /* In the typical case, we are adding an skb to the end of the list.
     * Use of ooo_last_skb avoids the O(Log(N)) rbtree lookup. */
    if (tcp_ooo_try_coalesce(sk, tp->ooo_last_skb, skb, &fragstolen)) {
coalesce_done:
        /* For non sack flows, do not grow window to force DUPACK
         * and trigger fast retransmit. */
        if (tcp_is_sack(tp))
            tcp_grow_window(sk, skb);
        kfree_skb_partial(skb, fragstolen);
        skb = NULL;
        goto add_sack;
    }
    /* Can avoid an rbtree lookup if we are adding skb after ooo_last_skb */
    if (!before(seq, TCP_SKB_CB(tp->ooo_last_skb)->end_seq)) {
        parent = &tp->ooo_last_skb->rbnode;
        p = &parent->rb_right;
        goto insert;
    }

    /* Find place to insert this segment. Handle overlaps on the way. */
    parent = NULL;
    while (*p) {
        parent = *p;
        skb1 = rb_to_skb(parent);
        if (before(seq, TCP_SKB_CB(skb1)->seq)) {
            p = &parent->rb_left;
            continue;
        }
        if (before(seq, TCP_SKB_CB(skb1)->end_seq)) {
            if (!after(end_seq, TCP_SKB_CB(skb1)->end_seq)) {
                /* All the bits are present. Drop. */
                tcp_drop(sk, skb);
                skb = NULL;
                tcp_dsack_set(sk, seq, end_seq);
                goto add_sack;
            }
            if (after(seq, TCP_SKB_CB(skb1)->seq)) {
                /* Partial overlap. */
                tcp_dsack_set(sk, seq, TCP_SKB_CB(skb1)->end_seq);
            } else {
                /* skb's seq == skb1's seq and skb covers skb1.
                 * Replace skb1 with skb. */
                rb_replace_node(&skb1->rbnode, &skb->rbnode, &tp->out_of_order_queue);
                tcp_dsack_extend(sk, TCP_SKB_CB(skb1)->seq, TCP_SKB_CB(skb1)->end_seq);
                tcp_drop(sk, skb1);
                goto merge_right;
            }
        } else if (tcp_ooo_try_coalesce(sk, skb1, skb, &fragstolen)) {
            goto coalesce_done;
        }
        p = &parent->rb_right;
    }
insert:
    /* Insert segment into RB tree. */
    rb_link_node(&skb->rbnode, parent, p);
    rb_insert_color(&skb->rbnode, &tp->out_of_order_queue);

merge_right:
    /* Remove other segments covered by skb. */
    while ((skb1 = skb_rb_next(skb)) != NULL) {
        if (!after(end_seq, TCP_SKB_CB(skb1)->seq))
            break;
        if (before(end_seq, TCP_SKB_CB(skb1)->end_seq)) {
            tcp_dsack_extend(sk, TCP_SKB_CB(skb1)->seq, end_seq);
            break;
        }
        rb_erase(&skb1->rbnode, &tp->out_of_order_queue);
        tcp_dsack_extend(sk, TCP_SKB_CB(skb1)->seq, TCP_SKB_CB(skb1)->end_seq);
        tcp_drop(sk, skb1);
    }
    /* If there is no skb after us, we are the last_skb ! */
    if (!skb1)
        tp->ooo_last_skb = skb;

add_sack:
    if (tcp_is_sack(tp))
        tcp_sack_new_ofo_skb(sk, seq, end_seq);
end:
    if (skb) {
        /* For non sack flows, do not grow window to force DUPACK
         * and trigger fast retransmit. */
        if (tcp_is_sack(tp))
            tcp_grow_window(sk, skb);
        skb_condense(skb);
        skb_set_owner_r(skb, sk);
    }
}

/* This one checks to see if we can put data from the
 * out_of_order queue into the receive_queue. */
void tcp_ofo_queue(struct sock *sk)
{
    struct tcp_sock *tp = tcp_sk(sk);
    __u32 dsack_high = tp->rcv_nxt;
    bool fin, fragstolen, eaten;
    struct sk_buff *skb, *tail;
    struct rb_node *p;

    p = rb_first(&tp->out_of_order_queue);
    while (p) {
        skb = rb_to_skb(p);
        if (after(TCP_SKB_CB(skb)->seq, tp->rcv_nxt))
            break;

        if (before(TCP_SKB_CB(skb)->seq, dsack_high)) {
            __u32 dsack = dsack_high;
            if (before(TCP_SKB_CB(skb)->end_seq, dsack_high))
                dsack_high = TCP_SKB_CB(skb)->end_seq;
            tcp_dsack_extend(sk, TCP_SKB_CB(skb)->seq, dsack);
        }
        p = rb_next(p);
        rb_erase(&skb->rbnode, &tp->out_of_order_queue);

        if (unlikely(!after(TCP_SKB_CB(skb)->end_seq, tp->rcv_nxt))) {
            tcp_drop(sk, skb);
            continue;
        }

        tail = skb_peek_tail(&sk->sk_receive_queue);
        eaten = tail && tcp_try_coalesce(sk, tail, skb, &fragstolen);
        tcp_rcv_nxt_update(tp, TCP_SKB_CB(skb)->end_seq);
        fin = TCP_SKB_CB(skb)->tcp_flags & TCPHDR_FIN;
        if (!eaten)
            __skb_queue_tail(&sk->sk_receive_queue, skb);
        else
            kfree_skb_partial(skb, fragstolen);

        if (unlikely(fin)) {
            tcp_fin(sk);
            /* tcp_fin() purges tp->out_of_order_queue,
             * so we must end this loop right now. */
            break;
        }
    }
}
```

## vfs layer rx

```c
SYSCALL_DEFINE3(read, unsigned int, fd, char __user *, buf, size_t, count)
{
    struct fd f = fdget_pos(fd);
    loff_t pos = file_pos_read(f.file);
    ret = vfs_read(f.file, buf, count, &pos);
}

ssize_t __vfs_read(struct file *file, char __user *buf, size_t count,
       loff_t *pos)
{
    if (file->f_op->read)
        return file->f_op->read(file, buf, count, pos);
    else if (file->f_op->read_iter)
        return new_sync_read(file, buf, count, pos);
    else
        return -EINVAL;
}

static ssize_t new_sync_read(
  struct file *filp, char __user *buf, size_t len, loff_t *ppos)
{
    struct iovec iov = { .iov_base = buf, .iov_len = len };
    struct kiocb kiocb;
    struct iov_iter iter;
    ssize_t ret;

    init_sync_kiocb(&kiocb, filp);
    kiocb.ki_pos = *ppos;
    iov_iter_init(&iter, READ, &iov, 1, len);

    /* file->f_op->read_iter(kio, iter); */
    ret = call_read_iter(filp, &kiocb, &iter);
    *ppos = kiocb.ki_pos;
    return ret;
}

const struct file_operations socket_file_ops = {
    .read_iter    = sock_read_iter,
    .write_iter   = sock_write_iter,
};
```

## socket layer rx
```c
ssize_t sock_read_iter(struct kiocb *iocb, struct iov_iter *to)
{
    struct file *file = iocb->ki_filp;
    struct socket *sock = file->private_data;
    struct msghdr msg = {.msg_iter = *to, .msg_iocb = iocb};
    ssize_t res;

    if (file->f_flags & O_NONBLOCK)
        msg.msg_flags = MSG_DONTWAIT;

    res = sock_recvmsg(sock, &msg, msg.msg_flags);
    *to = msg.msg_iter;
    return res;
}

/* sock_recvmsg -> */
int sock_recvmsg_nosec(struct socket *sock, struct msghdr *msg, int flags)
{
    return sock->ops->recvmsg(sock, msg, msg_data_left(msg), flags);
}

/* inet_stream_ops.inet_recvmsg */
int inet_recvmsg(struct socket *sock, struct msghdr *msg, size_t size,
     int flags)
{
    struct sock *sk = sock->sk;
    int addr_len = 0;
    int err;

    if (likely(!(flags & MSG_ERRQUEUE))) {
        sock_rps_record_flow(sk) {
            if (static_branch_unlikely(&rfs_needed)) {
                if (sk->sk_state == TCP_ESTABLISHED) {
                    sock_rps_record_flow_hash(READ_ONCE(sk->sk_rxhash)) { /* Used at get_rps_cpu */
                        struct rps_sock_flow_table *sock_flow_table;

                        if (!hash)
                            return;
                        rcu_read_lock();
                        sock_flow_table = rcu_dereference(net_hotdata.rps_sock_flow_table);
                        if (sock_flow_table) {
                            rps_record_sock_flow(sock_flow_table, hash) {
                                unsigned int index = hash & table->mask;
                                u32 val = hash & ~net_hotdata.rps_cpu_mask;

                                val |= raw_smp_processor_id();
                                if (READ_ONCE(table->ents[index]) != val)
                                    WRITE_ONCE(table->ents[index], val);
                            }
                        }
                        rcu_read_unlock();
                    }
                }
            }
        }
    }

    err = sk->sk_prot->recvmsg(sk, msg, size, flags & MSG_DONTWAIT,
           flags & ~MSG_DONTWAIT, &addr_len);
}

int tcp_recvmsg(struct sock *sk, struct msghdr *msg, size_t len, int nonblock,
    int flags, int *addr_len)
{
    struct tcp_sock *tp = tcp_sk(sk);
    int copied = 0;
    u32 peek_seq;
    u32 *seq;
    unsigned long used;
    int err, inq;
    int target;    /* Read at least this many bytes */
    long timeo;
    struct sk_buff *skb, *last;
    u32 urg_hole = 0;
    struct scm_timestamping tss;
    bool has_tss = false;
    bool has_cmsg;

    if (unlikely(flags & MSG_ERRQUEUE))
        return inet_recv_error(sk, msg, len, addr_len);

    if (sk_can_busy_loop(sk) && skb_queue_empty_lockless(&sk->sk_receive_queue) &&
        (sk->sk_state == TCP_ESTABLISHED))
        sk_busy_loop(sk, nonblock);

    lock_sock(sk);

    err = -ENOTCONN;
    if (sk->sk_state == TCP_LISTEN)
        goto out;

    has_cmsg = tp->recvmsg_inq;
    timeo = sock_rcvtimeo(sk, nonblock);

    /* Urgent data needs to be handled specially. */
    if (flags & MSG_OOB)
        goto recv_urg;

    if (unlikely(tp->repair)) {
        err = -EPERM;
        if (!(flags & MSG_PEEK))
            goto out;

        if (tp->repair_queue == TCP_SEND_QUEUE)
            goto recv_sndq;

        err = -EINVAL;
        if (tp->repair_queue == TCP_NO_QUEUE)
            goto out;

        /* 'common' recv queue MSG_PEEK-ing */
    }

    seq = &tp->copied_seq;
    if (flags & MSG_PEEK) {
        peek_seq = tp->copied_seq;
        seq = &peek_seq;
    }

    target = sock_rcvlowat(sk, flags & MSG_WAITALL, len);

    do {
        u32 offset;

        /* Are we at urgent data? Stop if we have read anything or have SIGURG pending. */
        if (tp->urg_data && tp->urg_seq == *seq) {
            if (copied)
                break;
            if (signal_pending(current)) {
                copied = timeo ? sock_intr_errno(timeo) : -EAGAIN;
                break;
            }
        }

        /* Next get a buffer. */

        last = skb_peek_tail(&sk->sk_receive_queue);
        skb_queue_walk(&sk->sk_receive_queue, skb) {
            last = skb;
            offset = *seq - TCP_SKB_CB(skb)->seq;

            if (offset < skb->len)
                goto found_ok_skb;
            if (TCP_SKB_CB(skb)->tcp_flags & TCPHDR_FIN)
                goto found_fin_ok;
        }

        /* Well, if we have backlog, try to process it now yet. */
        if (copied >= target && !sk->sk_backlog.tail)
            break;

        if (copied) {
            if (sk->sk_err ||
                sk->sk_state == TCP_CLOSE ||
                (sk->sk_shutdown & RCV_SHUTDOWN) ||
                !timeo || signal_pending(current))
                break;
        } else {
            if (sock_flag(sk, SOCK_DONE))
                break;

            if (sk->sk_err) {
                copied = sock_error(sk);
                break;
            }

            if (sk->sk_shutdown & RCV_SHUTDOWN)
                break;

            if (sk->sk_state == TCP_CLOSE) {
                /* This occurs when user tries to read
                * from never connected socket. */
                copied = -ENOTCONN;
                break;
            }

            if (!timeo) {
                copied = -EAGAIN;
                break;
            }

            if (signal_pending(current)) {
                copied = sock_intr_errno(timeo);
                break;
            }
        }

        tcp_cleanup_rbuf(sk, copied);

        if (copied >= target) {
            /* Do not sleep, just process backlog. */
            release_sock(sk);
            lock_sock(sk);
        } else {
            sk_wait_data(sk, &timeo, last);
        }

        if ((flags & MSG_PEEK)
            && (peek_seq - copied - urg_hole != tp->copied_seq))
        {
            peek_seq = tp->copied_seq;
        }

        continue;

    found_ok_skb:
        /* Ok so how much can we use? */
        used = skb->len - offset;
        if (len < used)
            used = len;

        /* Do we have urgent data here? */
        if (tp->urg_data) {
            u32 urg_offset = tp->urg_seq - *seq;
            if (urg_offset < used) {
                if (!urg_offset) {
                    if (!sock_flag(sk, SOCK_URGINLINE)) {
                    ++*seq;
                    urg_hole++;
                    offset++;
                    used--;
                    if (!used)
                        goto skip_copy;
                    }
                } else
                    used = urg_offset;
            }
        }

        if (!(flags & MSG_TRUNC)) {
            err = skb_copy_datagram_msg(skb, offset, msg, used);
            if (err) {
                /* Exception. Bailout! */
                if (!copied)
                    copied = -EFAULT;
                break;
            }
        }

        *seq += used;
        copied += used;
        len -= used;

        tcp_rcv_space_adjust(sk);

    skip_copy:
        if (tp->urg_data && after(tp->copied_seq, tp->urg_seq)) {
            tp->urg_data = 0;
            tcp_fast_path_check(sk);
        }

        if (TCP_SKB_CB(skb)->has_rxtstamp) {
            tcp_update_recv_tstamps(skb, &tss);
            has_tss = true;
            has_cmsg = true;
        }

        if (used + offset < skb->len)
            continue;

        if (TCP_SKB_CB(skb)->tcp_flags & TCPHDR_FIN)
            goto found_fin_ok;
        if (!(flags & MSG_PEEK))
            sk_eat_skb(sk, skb);

        continue;

        found_fin_ok:
        /* Process the FIN. */
        ++*seq;
        if (!(flags & MSG_PEEK))
            sk_eat_skb(sk, skb);
        break;
    } while (len > 0);

    /* Clean up data we have read: This will do ACK frames. */
    tcp_cleanup_rbuf(sk, copied);

    /* process backlog */
    release_sock(sk);

    if (has_cmsg) {
        if (has_tss)
            tcp_recv_timestamp(msg, sk, &tss);
        if (tp->recvmsg_inq) {
            inq = tcp_inq_hint(sk);
            put_cmsg(msg, SOL_TCP, TCP_CM_INQ, sizeof(inq), &inq);
        }
    }

    return copied;

out:
    release_sock(sk);
    return err;

recv_urg:
    err = tcp_recv_urg(sk, msg, len, flags);
    goto out;

recv_sndq:
    err = tcp_peek_sndq(sk, msg, len);
    goto out;
}

/* Clean up the receive buffer for full frames taken by the user,
 * then send an ACK if necessary.  COPIED is the number of bytes
 * tcp_recvmsg has given to the user so far, it speeds up the
 * calculation of whether or not we must ACK for the sake of
 * a window update. */
void tcp_cleanup_rbuf(struct sock *sk, int copied)
{
    struct tcp_sock *tp = tcp_sk(sk);
    bool time_to_ack = false;

    struct sk_buff *skb = skb_peek(&sk->sk_receive_queue);

    if (inet_csk_ack_scheduled(sk)) {
        const struct inet_connection_sock *icsk = inet_csk(sk);
        /* Delayed ACKs frequently hit locked sockets during bulk
            * receive. */
        if (icsk->icsk_ack.blocked ||
            /* Once-per-two-segments ACK was not sent by tcp_input.c */
            tp->rcv_nxt - tp->rcv_wup > icsk->icsk_ack.rcv_mss ||
            /* If this read emptied read buffer, we send ACK, if
            * connection is not bidirectional, user drained
            * receive buffer and there was a small segment
            * in queue. */
            (copied > 0 &&
            ((icsk->icsk_ack.pending & ICSK_ACK_PUSHED2) ||
            ((icsk->icsk_ack.pending & ICSK_ACK_PUSHED) &&
            !icsk->icsk_ack.pingpong)) &&
            !atomic_read(&sk->sk_rmem_alloc)))
        {
        time_to_ack = true;
        }
    }

  /* We send an ACK if we can now advertise a non-zero window
   * which has been raised "significantly".
   *
   * Even if window raised up to infinity, do not send window open ACK
   * in states, where we will not receive more. It is useless. */
    if (copied > 0 && !time_to_ack && !(sk->sk_shutdown & RCV_SHUTDOWN)) {
        __u32 rcv_window_now = tcp_receive_window(tp);

        /* Optimize, __tcp_select_window() is not cheap. */
        if (2*rcv_window_now <= tp->window_clamp) {
            __u32 new_window = __tcp_select_window(sk);

            /* Send ACK now, if this read freed lots of space
            * in our buffer. Certainly, new_window is new window.
            * We can advertise it now, if it is not less than current one.
            * "Lots" means "at least twice" here. */
            if (new_window && new_window >= 2 * rcv_window_now)
                time_to_ack = true;
        }
    }
    if (time_to_ack)
        tcp_send_ack(sk);
}

/* process backlog */
void release_sock(struct sock *sk)
{
    if (sk->sk_backlog.tail)
        __release_sock(sk);

    if (sk->sk_prot->release_cb)
        sk->sk_prot->release_cb(sk);

    sock_release_ownership(sk);
    if (waitqueue_active(&sk->sk_lock.wq))
        wake_up(&sk->sk_lock.wq);
    spin_unlock_bh(&sk->sk_lock.slock);
}

void __release_sock(struct sock *sk)
  __releases(&sk->sk_lock.slock)
  __acquires(&sk->sk_lock.slock)
{
    struct sk_buff *skb, *next;

    while ((skb = sk->sk_backlog.head) != NULL) {
        sk->sk_backlog.head = sk->sk_backlog.tail = NULL;
        do {
            next = skb->next;
            prefetch(next);
            skb->next = NULL;
            sk_backlog_rcv(sk, skb);
            cond_resched();
            skb = next;
        } while (skb != NULL);
    }
}

int __sk_backlog_rcv(struct sock *sk, struct sk_buff *skb)
{
    int ret;
    unsigned int noreclaim_flag;

    noreclaim_flag = memalloc_noreclaim_save();
    ret = sk->sk_backlog_rcv(sk, skb); /* tcp_v4_do_rcv */
    memalloc_noreclaim_restore(noreclaim_flag);

    return ret;
}

int sk_wait_data(struct sock *sk, long *timeo, const struct sk_buff *skb)
{
    DEFINE_WAIT_FUNC(wait, woken_wake_function);
    int rc;

    add_wait_queue(sk_sleep(sk), &wait);
    sk_set_bit(SOCKWQ_ASYNC_WAITDATA, sk);
    rc = sk_wait_event(sk, timeo, skb_peek_tail(&sk->sk_receive_queue) != skb, &wait);
    sk_clear_bit(SOCKWQ_ASYNC_WAITDATA, sk);
    remove_wait_queue(sk_sleep(sk), &wait);
    return rc;
}

#define sk_wait_event(__sk, __timeo, __condition, __wait)    \
  ({  int __rc;            \
    release_sock(__sk);          \
    __rc = __condition;          \
    if (!__rc) {            \
      *(__timeo) = wait_woken(__wait, TASK_INTERRUPTIBLE, *(__timeo));\
    }              \
    sched_annotate_sleep();          \
    lock_sock(__sk);          \
    __rc = __condition;          \
    __rc;              \
  })
```
* [wait_woken](./linux-proc.md#wait_woken)
* [wake_up](./linux-proc.md#wake_up)

# route
<img src='../images/kernel/net-filter.svg' style='max-height:850px'/>

---

<img src='../images/kernel/net-filter-2.svg' style='max-height:850px'/>

---

<img src='../images/kernel/net-filter-3.png' style='max-height:850px'/>

---

<img src='../images/kernel/net-route-fib.svg' style='max-height:850px'/>

---

<img src='../images/kernel/net-route.svg' style='max-height:850px'/>

```c
struct net {
    struct netns_ipv4    ipv4;
};

struct netns_ipv4 {
    struct hlist_head    *fib_table_hash;
}

struct fib_table {
    struct hlist_node    tb_hlist;
    u32                  tb_id;
    int                  tb_num_default;
    struct rcu_head      rcu;
    unsigned long        *tb_data;
    unsigned long        __data[0];
};

struct fib_alias {
    struct hlist_node   fa_list;
    struct fib_info     *fa_info;
    u8                  fa_tos;
    u8                  fa_type;
    u8                  fa_state;
    u8                  fa_slen;
    u32                 tb_id;
    s16                 fa_default;
    struct rcu_head     rcu;
};

struct fib_info {
    struct hlist_node  fib_hash;
    struct hlist_node  fib_lhash;
    struct net         *fib_net;
    int               fib_treeref;
    refcount_t        fib_clntref;
    unsigned int      fib_flags;
    unsigned char     fib_dead;
    unsigned char     fib_protocol;
    unsigned char     fib_scope;
    unsigned char     fib_type;
    __be32            fib_prefsrc;
    u32               fib_tb_id;
    u32               fib_priority;
    struct dst_metrics  *fib_metrics;
    #define fib_mtu fib_metrics->metrics[RTAX_MTU-1]
    #define fib_window fib_metrics->metrics[RTAX_WINDOW-1]
    #define fib_rtt fib_metrics->metrics[RTAX_RTT-1]
    #define fib_advmss fib_metrics->metrics[RTAX_ADVMSS-1]
    int                 fib_nhs;
    struct rcu_head     rcu;
    struct fib_nh       fib_nh[0];
    #define fib_dev       fib_nh[0].nh_dev
};

struct fib_nh {
    struct net_device   *nh_dev;
    struct hlist_node   nh_hash;
    struct fib_info     *nh_parent;
    unsigned int        nh_flags;
    unsigned char       nh_scope;
    int                 nh_weight;
    atomic_t            nh_upper_bound;
    __u32               nh_tclassid;
    int                 nh_oif;
    __be32              nh_gw;
    __be32              nh_saddr;
    int                 nh_saddr_genid;
    struct rtable __rcu * __percpu    *nh_pcpu_rth_output;
    struct rtable __rcu               *nh_rth_input;
    struct fnhe_hash_bucket  __rcu    *nh_exceptions;
    struct lwtunnel_state             *nh_lwtstate;
};

struct fib_result {
    __be32          prefix;
    unsigned char   prefixlen;
    unsigned char   nh_sel;
    unsigned char   type;
    unsigned char   scope;
    u32             tclassid;
    struct fib_info   *fi;
    struct fib_table  *table;
    struct hlist_head *fa_head;
};
```

## route out
```c
struct rtable *ip_route_output_ports(
  struct net *net, struct flowi4 *fl4,
  struct sock *sk,
  __be32 daddr, __be32 saddr,
  __be16 dport, __be16 sport,
  __u8 proto, __u8 tos, int oif)
{
  flowi4_init_output(fl4, oif, sk ? sk->sk_mark : 0, tos,
         RT_SCOPE_UNIVERSE, proto,
         sk ? inet_sk_flowi_flags(sk) : 0,
         daddr, saddr, dport, sport, sock_net_uid(net, sk));
  if (sk)
    security_sk_classify_flow(sk, flowi4_to_flowi(fl4));

  return ip_route_output_flow(net, fl4, sk);
}

struct rtable *ip_route_output_flow(struct net *net, struct flowi4 *flp4,
            const struct sock *sk)
{
  struct rtable *rt = __ip_route_output_key(net, flp4);

  if (IS_ERR(rt))
    return rt;

  if (flp4->flowi4_proto) {
    flp4->flowi4_oif = rt->dst.dev->ifindex;
    rt = (struct rtable *)xfrm_lookup_route(net, &rt->dst,
              flowi4_to_flowi(flp4), sk, 0);
  }

  return rt;
}

struct rtable *__ip_route_output_key(struct net *net,
               struct flowi4 *flp)
{
  return ip_route_output_key_hash(net, flp, NULL);
}

struct rtable *ip_route_output_key_hash(struct net *net, struct flowi4 *fl4,
          const struct sk_buff *skb)
{
  __u8 tos = RT_FL_TOS(fl4);
  struct fib_result res = {
    .type       = RTN_UNSPEC,
    .fi         = NULL,
    .table      = NULL,
    .tclassid   = 0,
  };
  struct rtable *rth;

  fl4->flowi4_iif = LOOPBACK_IFINDEX;
  fl4->flowi4_tos = tos & IPTOS_RT_MASK;
  fl4->flowi4_scope = ((tos & RTO_ONLINK) ? RT_SCOPE_LINK : RT_SCOPE_UNIVERSE);

  rcu_read_lock();
  rth = ip_route_output_key_hash_rcu(net, fl4, &res, skb);
  rcu_read_unlock();

  return rth;
}

struct rtable *ip_route_output_key_hash_rcu(struct net *net, struct flowi4 *fl4,
  struct fib_result *res, const struct sk_buff *skb)
{
  struct net_device *dev_out = NULL;
  int orig_oif = fl4->flowi4_oif;
  unsigned int flags = 0;
  struct rtable *rth;

  err = fib_lookup(net, fl4, res, 0);

make_route:
  rth = __mkroute_output(res, fl4, orig_oif, dev_out, flags);

out:
  return rth;
}

static struct rtable *__mkroute_output(const struct fib_result *res,
               const struct flowi4 *fl4, int orig_oif,
               struct net_device *dev_out,
               unsigned int flags)
{
  struct fib_info *fi = res->fi;
  struct fib_nh_exception *fnhe;
  struct in_device *in_dev;
  u16 type = res->type;
  struct rtable *rth;
  bool do_cache;

  in_dev = __in_dev_get_rcu(dev_out);
  if (!in_dev)
    return ERR_PTR(-EINVAL);

  if (likely(!IN_DEV_ROUTE_LOCALNET(in_dev)))
    if (ipv4_is_loopback(fl4->saddr) &&
        !(dev_out->flags & IFF_LOOPBACK) &&
        !netif_is_l3_master(dev_out))
      return ERR_PTR(-EINVAL);

  if (ipv4_is_lbcast(fl4->daddr))
    type = RTN_BROADCAST;
  else if (ipv4_is_multicast(fl4->daddr))
    type = RTN_MULTICAST;
  else if (ipv4_is_zeronet(fl4->daddr))
    return ERR_PTR(-EINVAL);

  if (dev_out->flags & IFF_LOOPBACK)
    flags |= RTCF_LOCAL;

  do_cache = true;
  if (type == RTN_BROADCAST) {
    flags |= RTCF_BROADCAST | RTCF_LOCAL;
    fi = NULL;
  } else if (type == RTN_MULTICAST) {
    flags |= RTCF_MULTICAST | RTCF_LOCAL;
    if (!ip_check_mc_rcu(in_dev, fl4->daddr, fl4->saddr,
             fl4->flowi4_proto))
      flags &= ~RTCF_LOCAL;
    else
      do_cache = false;
    /* If multicast route do not exist use
     * default one, but do not gateway in this case.
     * Yes, it is hack. */
    if (fi && res->prefixlen < 4)
      fi = NULL;
  } else if ((type == RTN_LOCAL) && (orig_oif != 0) &&
       (orig_oif != dev_out->ifindex)) {
    /* For local routes that require a particular output interface
     * we do not want to cache the result.  Caching the result
     * causes incorrect behaviour when there are multiple source
     * addresses on the interface, the end result being that if the
     * intended recipient is waiting on that interface for the
     * packet he won't receive it because it will be delivered on
     * the loopback interface and the IP_PKTINFO ipi_ifindex will
     * be set to the loopback interface as well. */
    do_cache = false;
  }

  fnhe = NULL;
  do_cache &= fi != NULL;
  if (fi) {
    struct rtable __rcu **prth;
    struct fib_nh *nh = &FIB_RES_NH(*res);

    fnhe = find_exception(nh, fl4->daddr);
    if (!do_cache)
      goto add;
    if (fnhe) {
      prth = &fnhe->fnhe_rth_output;
    } else {
      if (unlikely(fl4->flowi4_flags &
             FLOWI_FLAG_KNOWN_NH &&
             !(nh->nh_gw &&
               nh->nh_scope == RT_SCOPE_LINK))) {
        do_cache = false;
        goto add;
      }
      prth = raw_cpu_ptr(nh->nh_pcpu_rth_output);
    }
    rth = rcu_dereference(*prth);
    if (rt_cache_valid(rth) && dst_hold_safe(&rth->dst))
      return rth;
  }

add:
  rth = rt_dst_alloc(dev_out, flags, type,
         IN_DEV_CONF_GET(in_dev, NOPOLICY),
         IN_DEV_CONF_GET(in_dev, NOXFRM),
         do_cache);
  if (!rth)
    return ERR_PTR(-ENOBUFS);

  rth->rt_iif = orig_oif;

  RT_CACHE_STAT_INC(out_slow_tot);

  if (flags & (RTCF_BROADCAST | RTCF_MULTICAST)) {
    if (flags & RTCF_LOCAL &&
        !(dev_out->flags & IFF_LOOPBACK)) {
      rth->dst.output = ip_mc_output;
      RT_CACHE_STAT_INC(out_slow_mc);
    }
#ifdef CONFIG_IP_MROUTE
    if (type == RTN_MULTICAST) {
      if (IN_DEV_MFORWARD(in_dev) &&
          !ipv4_is_local_multicast(fl4->daddr)) {
        rth->dst.input = ip_mr_input;
        rth->dst.output = ip_mc_output;
      }
    }
#endif
  }

  rt_set_nexthop(rth, fl4->daddr, res, fnhe, fi, type, 0, do_cache);
  lwtunnel_set_redirect(&rth->dst);

  return rth;
}

void rt_set_nexthop(struct rtable *rt, __be32 daddr,
         const struct fib_result *res,
         struct fib_nh_exception *fnhe,
         struct fib_info *fi, u16 type, u32 itag,
         const bool do_cache)
{
  bool cached = false;

  if (fi) {
    struct fib_nh *nh = &FIB_RES_NH(*res);

    if (nh->nh_gw && nh->nh_scope == RT_SCOPE_LINK) {
      rt->rt_gateway = nh->nh_gw;
      rt->rt_uses_gateway = 1;
    }
    dst_init_metrics(&rt->dst, fi->fib_metrics->metrics, true);
    if (fi->fib_metrics != &dst_default_metrics) {
      rt->dst._metrics |= DST_METRICS_REFCOUNTED;
      refcount_inc(&fi->fib_metrics->refcnt);
    }
#ifdef CONFIG_IP_ROUTE_CLASSID
    rt->dst.tclassid = nh->nh_tclassid;
#endif
    rt->dst.lwtstate = lwtstate_get(nh->nh_lwtstate);
    if (unlikely(fnhe))
      cached = rt_bind_exception(rt, fnhe, daddr, do_cache);
    else if (do_cache)
      cached = rt_cache_route(nh, rt);
    if (unlikely(!cached)) {
      /* Routes we intend to cache in nexthop exception or
       * FIB nexthop have the DST_NOCACHE bit clear.
       * However, if we are unsuccessful at storing this
       * route into the cache we really need to set it. */
      if (!rt->rt_gateway)
        rt->rt_gateway = daddr;
      rt_add_uncached_list(rt);
    }
  } else
    rt_add_uncached_list(rt);

#ifdef CONFIG_IP_ROUTE_CLASSID
#ifdef CONFIG_IP_MULTIPLE_TABLES
  set_class_tag(rt, res->tclassid);
#endif
  set_class_tag(rt, itag);
#endif
}
```

<img src='../images/kernel/net-forwarding-table.svg' style='max-height:850px'/>

```c
# Linux Server A
default via 192.168.1.1 dev eth0
192.168.1.0/24 dev eth0 proto kernel scope link src 192.168.1.100 metric 100

# Linux router
192.168.1.0/24 dev eth0 proto kernel scope link src 192.168.1.1
192.168.2.0/24 dev eth1 proto kernel scope link src 192.168.2.1

# Linux Server B
default via 192.168.2.1 dev eth0
192.168.2.0/24 dev eth0 proto kernel scope link src 192.168.2.100 metric 100
```

## route in
```c
int ip_route_input_noref(struct sk_buff *skb, __be32 daddr, __be32 saddr,
       u8 tos, struct net_device *dev)
{
  struct fib_result res;
  int err;

  tos &= IPTOS_RT_MASK;
  rcu_read_lock();
  err = ip_route_input_rcu(skb, daddr, saddr, tos, dev, &res);
  rcu_read_unlock();

  return err;
}

/* called with rcu_read_lock held */
int ip_route_input_rcu(struct sk_buff *skb, __be32 daddr, __be32 saddr,
           u8 tos, struct net_device *dev, struct fib_result *res)
{
  return ip_route_input_slow(skb, daddr, saddr, tos, dev, res);
}

int ip_route_input_slow(struct sk_buff *skb, __be32 daddr, __be32 saddr,
             u8 tos, struct net_device *dev,
             struct fib_result *res)
{
  struct in_device *in_dev = __in_dev_get_rcu(dev);
  struct flow_keys *flkeys = NULL, _flkeys;
  struct net    *net = dev_net(dev);
  struct ip_tunnel_info *tun_info;
  int    err = -EINVAL;
  unsigned int  flags = 0;
  u32    itag = 0;
  struct rtable  *rth;
  struct flowi4  fl4;
  bool do_cache = true;

  /* IP on this device is disabled. */

  if (!in_dev)
    goto out;

  /* Check for the most weird martians, which can be not detected
     by fib_lookup. */

  tun_info = skb_tunnel_info(skb);
  if (tun_info && !(tun_info->mode & IP_TUNNEL_INFO_TX))
    fl4.flowi4_tun_key.tun_id = tun_info->key.tun_id;
  else
    fl4.flowi4_tun_key.tun_id = 0;
  skb_dst_drop(skb);

  if (ipv4_is_multicast(saddr) || ipv4_is_lbcast(saddr))
    goto martian_source;

  res->fi = NULL;
  res->table = NULL;
  if (ipv4_is_lbcast(daddr) || (saddr == 0 && daddr == 0))
    goto brd_input;

  /* Accept zero addresses only to limited broadcast;
   * I even do not know to fix it or not. Waiting for complains :-) */
  if (ipv4_is_zeronet(saddr))
    goto martian_source;

  if (ipv4_is_zeronet(daddr))
    goto martian_destination;

  /* Following code try to avoid calling IN_DEV_NET_ROUTE_LOCALNET(),
   * and call it once if daddr or/and saddr are loopback addresses */
  if (ipv4_is_loopback(daddr)) {
    if (!IN_DEV_NET_ROUTE_LOCALNET(in_dev, net))
      goto martian_destination;
  } else if (ipv4_is_loopback(saddr)) {
    if (!IN_DEV_NET_ROUTE_LOCALNET(in_dev, net))
      goto martian_source;
  }

  /*  Now we are ready to route packet. */
  fl4.flowi4_oif = 0;
  fl4.flowi4_iif = dev->ifindex;
  fl4.flowi4_mark = skb->mark;
  fl4.flowi4_tos = tos;
  fl4.flowi4_scope = RT_SCOPE_UNIVERSE;
  fl4.flowi4_flags = 0;
  fl4.daddr = daddr;
  fl4.saddr = saddr;
  fl4.flowi4_uid = sock_net_uid(net, NULL);

  if (fib4_rules_early_flow_dissect(net, skb, &fl4, &_flkeys)) {
    flkeys = &_flkeys;
  } else {
    fl4.flowi4_proto = 0;
    fl4.fl4_sport = 0;
    fl4.fl4_dport = 0;
  }

  err = fib_lookup(net, &fl4, res, 0);
  if (err != 0) {
    if (!IN_DEV_FORWARD(in_dev))
      err = -EHOSTUNREACH;
    goto no_route;
  }

  if (res->type == RTN_BROADCAST) {
    if (IN_DEV_BFORWARD(in_dev))
      goto make_route;
    /* not do cache if bc_forwarding is enabled */
    if (IPV4_DEVCONF_ALL(net, BC_FORWARDING))
      do_cache = false;
    goto brd_input;
  }

  if (res->type == RTN_LOCAL) {
    err = fib_validate_source(skb, saddr, daddr, tos,
            0, dev, in_dev, &itag);
    if (err < 0)
      goto martian_source;
    goto local_input;
  }

  if (!IN_DEV_FORWARD(in_dev)) {
    err = -EHOSTUNREACH;
    goto no_route;
  }
  if (res->type != RTN_UNICAST)
    goto martian_destination;

make_route:
  err = ip_mkroute_input(skb, res, in_dev, daddr, saddr, tos, flkeys);
out:  return err;

brd_input:
  if (skb->protocol != htons(ETH_P_IP))
    goto e_inval;

  if (!ipv4_is_zeronet(saddr)) {
    err = fib_validate_source(skb, saddr, 0, tos, 0, dev, in_dev, &itag);
    if (err < 0)
      goto martian_source;
  }
  flags |= RTCF_BROADCAST;
  res->type = RTN_BROADCAST;
  RT_CACHE_STAT_INC(in_brd);

local_input:
  do_cache &= res->fi && !itag;
  if (do_cache) {
    rth = rcu_dereference(FIB_RES_NH(*res).nh_rth_input);
    if (rt_cache_valid(rth)) {
      skb_dst_set_noref(skb, &rth->dst);
      err = 0;
      goto out;
    }
  }

  rth = rt_dst_alloc(l3mdev_master_dev_rcu(dev) ? : net->loopback_dev,
         flags | RTCF_LOCAL, res->type,
         IN_DEV_CONF_GET(in_dev, NOPOLICY), false, do_cache);
  if (!rth)
    goto e_nobufs;

  rth->dst.output= ip_rt_bug;
#ifdef CONFIG_IP_ROUTE_CLASSID
  rth->dst.tclassid = itag;
#endif
  rth->rt_is_input = 1;

  RT_CACHE_STAT_INC(in_slow_tot);
  if (res->type == RTN_UNREACHABLE) {
    rth->dst.input= ip_error;
    rth->dst.error= -err;
    rth->rt_flags   &= ~RTCF_LOCAL;
  }

  if (do_cache) {
    struct fib_nh *nh = &FIB_RES_NH(*res);

    rth->dst.lwtstate = lwtstate_get(nh->nh_lwtstate);
    if (lwtunnel_input_redirect(rth->dst.lwtstate)) {
      WARN_ON(rth->dst.input == lwtunnel_input);
      rth->dst.lwtstate->orig_input = rth->dst.input;
      rth->dst.input = lwtunnel_input;
    }

    if (unlikely(!rt_cache_route(nh, rth)))
      rt_add_uncached_list(rth);
  }
  skb_dst_set(skb, &rth->dst);
  err = 0;
  goto out;

no_route:
  RT_CACHE_STAT_INC(in_no_route);
  res->type = RTN_UNREACHABLE;
  res->fi = NULL;
  res->table = NULL;
  goto local_input;

  /*  Do not cache martian addresses: they should be logged (RFC1812) */
martian_destination:
  RT_CACHE_STAT_INC(in_martian_dst);
#ifdef CONFIG_IP_ROUTE_VERBOSE
  if (IN_DEV_LOG_MARTIANS(in_dev))
    net_warn_ratelimited("martian destination %pI4 from %pI4, dev %s\n",
             &daddr, &saddr, dev->name);
#endif

e_inval:
  err = -EINVAL;
  goto out;

e_nobufs:
  err = -ENOBUFS;
  goto out;

martian_source:
  ip_handle_martian_source(dev, in_dev, skb, daddr, saddr);
  goto out;
}

int ip_mkroute_input(struct sk_buff *skb,
          struct fib_result *res,
          struct in_device *in_dev,
          __be32 daddr, __be32 saddr, u32 tos,
          struct flow_keys *hkeys)
{
#ifdef CONFIG_IP_ROUTE_MULTIPATH
  if (res->fi && res->fi->fib_nhs > 1) {
    int h = fib_multipath_hash(res->fi->fib_net, NULL, skb, hkeys);

    fib_select_multipath(res, h);
  }
#endif

  /* create a routing cache entry */
  return __mkroute_input(skb, res, in_dev, daddr, saddr, tos);
}

int __mkroute_input(struct sk_buff *skb,
         const struct fib_result *res,
         struct in_device *in_dev,
         __be32 daddr, __be32 saddr, u32 tos)
{
  struct fib_nh_exception *fnhe;
  struct rtable *rth;
  int err;
  struct in_device *out_dev;
  bool do_cache;
  u32 itag = 0;

  /* get a working reference to the output device */
  out_dev = __in_dev_get_rcu(FIB_RES_DEV(*res));
  if (!out_dev) {
    net_crit_ratelimited("Bug in ip_route_input_slow(). Please report.\n");
    return -EINVAL;
  }

  err = fib_validate_source(skb, saddr, daddr, tos, FIB_RES_OIF(*res),
          in_dev->dev, in_dev, &itag);
  if (err < 0) {
    ip_handle_martian_source(in_dev->dev, in_dev, skb, daddr,
           saddr);

    goto cleanup;
  }

  do_cache = res->fi && !itag;
  if (out_dev == in_dev && err && IN_DEV_TX_REDIRECTS(out_dev) &&
      skb->protocol == htons(ETH_P_IP) &&
      (IN_DEV_SHARED_MEDIA(out_dev) ||
       inet_addr_onlink(out_dev, saddr, FIB_RES_GW(*res))))
    IPCB(skb)->flags |= IPSKB_DOREDIRECT;

  if (skb->protocol != htons(ETH_P_IP)) {
    /* Not IP (i.e. ARP). Do not create route, if it is
     * invalid for proxy arp. DNAT routes are always valid.
     *
     * Proxy arp feature have been extended to allow, ARP
     * replies back to the same interface, to support
     * Private VLAN switch technologies. See arp.c. */
    if (out_dev == in_dev &&
        IN_DEV_PROXY_ARP_PVLAN(in_dev) == 0) {
      err = -EINVAL;
      goto cleanup;
    }
  }

  fnhe = find_exception(&FIB_RES_NH(*res), daddr);
  if (do_cache) {
    if (fnhe)
      rth = rcu_dereference(fnhe->fnhe_rth_input);
    else
      rth = rcu_dereference(FIB_RES_NH(*res).nh_rth_input);
    if (rt_cache_valid(rth)) {
      skb_dst_set_noref(skb, &rth->dst);
      goto out;
    }
  }

  rth = rt_dst_alloc(out_dev->dev, 0, res->type,
         IN_DEV_CONF_GET(in_dev, NOPOLICY),
         IN_DEV_CONF_GET(out_dev, NOXFRM), do_cache);
  if (!rth) {
    err = -ENOBUFS;
    goto cleanup;
  }

  rth->rt_is_input = 1;
  RT_CACHE_STAT_INC(in_slow_tot);

  rth->dst.input = ip_forward;

  rt_set_nexthop(rth, daddr, res, fnhe, res->fi, res->type, itag, do_cache);

  lwtunnel_set_redirect(&rth->dst);
  skb_dst_set(skb, &rth->dst);
out:
  err = 0;
 cleanup:
  return err;
}

void skb_dst_set(struct sk_buff *skb, struct dst_entry *dst)
{
 skb->_skb_refdst = (unsigned long)dst;
}
```

## fib_lookup
```c
/* FIB: Forwarding Information Base*/
int fib_lookup(struct net *net, struct flowi4 *flp,
           struct fib_result *res, unsigned int flags)
{
  struct fib_table *tb;
  int err = -ENETUNREACH;

  flags |= FIB_LOOKUP_NOREF;
  if (net->ipv4.fib_has_custom_rules)
    return __fib_lookup(net, flp, res, flags);

  rcu_read_lock();

  res->tclassid = 0;

  tb = rcu_dereference_rtnl(net->ipv4.fib_main);
  if (tb)
    err = fib_table_lookup(tb, flp, res, flags);

  if (!err)
    goto out;

  tb = rcu_dereference_rtnl(net->ipv4.fib_default);
  if (tb)
    err = fib_table_lookup(tb, flp, res, flags);

out:
  if (err == -EAGAIN)
    err = -ENETUNREACH;

  rcu_read_unlock();

  return err;
}

int fib_table_lookup(struct fib_table *tb, const struct flowi4 *flp,
         struct fib_result *res, int fib_flags)
{
  struct trie *t = (struct trie *) tb->tb_data;
#ifdef CONFIG_IP_FIB_TRIE_STATS
  struct trie_use_stats __percpu *stats = t->stats;
#endif
  const t_key key = ntohl(flp->daddr);
  struct key_vector *n, *pn;
  struct fib_alias *fa;
  unsigned long index;
  t_key cindex;

  pn = t->kv;
  cindex = 0;

  n = get_child_rcu(pn, cindex);
  if (!n) {
    trace_fib_table_lookup(tb->tb_id, flp, NULL, -EAGAIN);
    return -EAGAIN;
  }

#ifdef CONFIG_IP_FIB_TRIE_STATS
  this_cpu_inc(stats->gets);
#endif

  /* Step 1: Travel to the longest prefix match in the trie */
  for (;;) {
    index = get_cindex(key, n);

    /* This bit of code is a bit tricky but it combines multiple
     * checks into a single check.  The prefix consists of the
     * prefix plus zeros for the "bits" in the prefix. The index
     * is the difference between the key and this value.  From
     * this we can actually derive several pieces of data.
     *   if (index >= (1ul << bits))
     *     we have a mismatch in skip bits and failed
     *   else
     *     we know the value is cindex
     *
     * This check is safe even if bits == KEYLENGTH due to the
     * fact that we can only allocate a node with 32 bits if a
     * long is greater than 32 bits. */
    if (index >= (1ul << n->bits))
      break;

    /* we have found a leaf. Prefixes have already been compared */
    if (IS_LEAF(n))
      goto found;

    /* only record pn and cindex if we are going to be chopping
     * bits later.  Otherwise we are just wasting cycles. */
    if (n->slen > n->pos) {
      pn = n;
      cindex = index;
    }

    n = get_child_rcu(n, index);
    if (unlikely(!n))
      goto backtrace;
  }

  /* Step 2: Sort out leaves and begin backtracing for longest prefix */
  for (;;) {
    /* record the pointer where our next node pointer is stored */
    struct key_vector __rcu **cptr = n->tnode;

    /* This test verifies that none of the bits that differ
     * between the key and the prefix exist in the region of
     * the lsb and higher in the prefix. */
    if (unlikely(prefix_mismatch(key, n)) || (n->slen == n->pos))
      goto backtrace;

    /* exit out and process leaf */
    if (unlikely(IS_LEAF(n)))
      break;

    /* Don't bother recording parent info.  Since we are in
     * prefix match mode we will have to come back to wherever
     * we started this traversal anyway */

    while ((n = rcu_dereference(*cptr)) == NULL) {
backtrace:
#ifdef CONFIG_IP_FIB_TRIE_STATS
      if (!n)
        this_cpu_inc(stats->null_node_hit);
#endif
      /* If we are at cindex 0 there are no more bits for
       * us to strip at this level so we must ascend back
       * up one level to see if there are any more bits to
       * be stripped there. */
      while (!cindex) {
        t_key pkey = pn->key;

        /* If we don't have a parent then there is
         * nothing for us to do as we do not have any
         * further nodes to parse. */
        if (IS_TRIE(pn)) {
          trace_fib_table_lookup(tb->tb_id, flp,
                     NULL, -EAGAIN);
          return -EAGAIN;
        }
#ifdef CONFIG_IP_FIB_TRIE_STATS
        this_cpu_inc(stats->backtrack);
#endif
        /* Get Child's index */
        pn = node_parent_rcu(pn);
        cindex = get_index(pkey, pn);
      }

      /* strip the least significant bit from the cindex */
      cindex &= cindex - 1;

      /* grab pointer for next child node */
      cptr = &pn->tnode[cindex];
    }
  }

found:
  /* this line carries forward the xor from earlier in the function */
  index = key ^ n->key;

  /* Step 3: Process the leaf, if that fails fall back to backtracing */
  hlist_for_each_entry_rcu(fa, &n->leaf, fa_list) {
    struct fib_info *fi = fa->fa_info;
    int nhsel, err;

    if ((BITS_PER_LONG > KEYLENGTH) || (fa->fa_slen < KEYLENGTH)) {
      if (index >= (1ul << fa->fa_slen))
        continue;
    }
    if (fa->fa_tos && fa->fa_tos != flp->flowi4_tos)
      continue;
    if (fi->fib_dead)
      continue;
    if (fa->fa_info->fib_scope < flp->flowi4_scope)
      continue;
    fib_alias_accessed(fa);
    err = fib_props[fa->fa_type].error;
    if (unlikely(err < 0)) {
#ifdef CONFIG_IP_FIB_TRIE_STATS
      this_cpu_inc(stats->semantic_match_passed);
#endif
      trace_fib_table_lookup(tb->tb_id, flp, NULL, err);
      return err;
    }
    if (fi->fib_flags & RTNH_F_DEAD)
      continue;
    for (nhsel = 0; nhsel < fi->fib_nhs; nhsel++) {
      const struct fib_nh *nh = &fi->fib_nh[nhsel];
      struct in_device *in_dev = __in_dev_get_rcu(nh->nh_dev);

      if (nh->nh_flags & RTNH_F_DEAD)
        continue;
      if (in_dev &&
          IN_DEV_IGNORE_ROUTES_WITH_LINKDOWN(in_dev) &&
          nh->nh_flags & RTNH_F_LINKDOWN &&
          !(fib_flags & FIB_LOOKUP_IGNORE_LINKSTATE))
        continue;
      if (!(flp->flowi4_flags & FLOWI_FLAG_SKIP_NH_OIF)) {
        if (flp->flowi4_oif &&
            flp->flowi4_oif != nh->nh_oif)
          continue;
      }

      if (!(fib_flags & FIB_LOOKUP_NOREF))
        refcount_inc(&fi->fib_clntref);

      res->prefix = htonl(n->key);
      res->prefixlen = KEYLENGTH - fa->fa_slen;
      res->nh_sel = nhsel;
      res->type = fa->fa_type;
      res->scope = fi->fib_scope;
      res->fi = fi;
      res->table = tb;
      res->fa_head = &n->leaf;
#ifdef CONFIG_IP_FIB_TRIE_STATS
      this_cpu_inc(stats->semantic_match_passed);
#endif
      trace_fib_table_lookup(tb->tb_id, flp, nh, err);

      return err;
    }
  }
#ifdef CONFIG_IP_FIB_TRIE_STATS
  this_cpu_inc(stats->semantic_match_miss);
#endif
  goto backtrace;
}
```

## rt_dst_alloc
```c
struct rtable {
  struct dst_entry  dst;
  int               rt_genid;
  unsigned int      rt_flags;
  __u16             rt_type;
  __u8              rt_is_input;
  __u8              rt_uses_gateway;
  int               rt_iif;
  u8                rt_gw_family;
  union {
    __be32           rt_gw4;
    struct in6_addr  rt_gw6;
  };
  u32 rt_mtu_locked:1,  rt_pmtu:31;
  struct list_head      rt_uncached;
  struct uncached_list  *rt_uncached_list;
};

struct dst_entry {
  struct net_device   *dev;
  struct  dst_ops     *ops;
  unsigned long       _metrics;
  unsigned long       expires;
  struct xfrm_state   *xfrm;
  int (*input)(struct sk_buff *);
  int (*output)(struct net *net, struct sock *sk, struct sk_buff *skb);
};

struct dst_ops ipv4_dst_ops = {
  .family           = AF_INET,
  .check            = ipv4_dst_check,
  .default_advmss   = ipv4_default_advmss,
  .mtu              = ipv4_mtu,
  .cow_metrics      = ipv4_cow_metrics,
  .destroy          = ipv4_dst_destroy,
  .negative_advice  = ipv4_negative_advice,
  .link_failure     = ipv4_link_failure,
  .update_pmtu      = ip_rt_update_pmtu,
  .redirect         = ip_do_redirect,
  .local_out        = __ip_local_out,
  .neigh_lookup     = ipv4_neigh_lookup,
  .confirm_neigh    = ipv4_confirm_neigh,
};

struct rtable *rt_dst_alloc(
  struct net_device *dev, unsigned int flags, u16 type,
  bool nopolicy, bool noxfrm, bool will_cache)
{
  struct rtable *rt;

  /*rt = dst_alloc(&ip6_dst_blackhole_ops
    rt = dst_alloc(&ipv4_dst_blackhole_ops
    rt = dst_alloc(&dn_dst_ops
    rt = dst_alloc(&dn_dst_ops */

  rt = dst_alloc(&ipv4_dst_ops, dev, 1, DST_OBSOLETE_FORCE_CHK,
        (will_cache ? 0 : DST_HOST)
        | (nopolicy ? DST_NOPOLICY : 0)
        | (noxfrm ? DST_NOXFRM : 0));

  if (rt) {
    rt->rt_genid = rt_genid_ipv4(dev_net(dev));
    rt->rt_flags = flags;
    rt->rt_type = type;
    rt->rt_is_input = 0;
    rt->rt_iif = 0;
    rt->rt_pmtu = 0;
    rt->rt_gateway = 0;
    rt->rt_uses_gateway = 0;
    rt->rt_table_id = 0;
    INIT_LIST_HEAD(&rt->rt_uncached);

    rt->dst.output = ip_output;
    if (flags & RTCF_LOCAL)
      rt->dst.input = ip_local_deliver;

    /* rth->dst.input = ip_forward;
      rt->dst.input = ip6_input;
      rt->dst.input = ip6_mc_input;
      rt->dst.input = ip6_forward; */
  }

  return rt;
}

void *dst_alloc(struct dst_ops *ops, struct net_device *dev,
    int initial_ref, int initial_obsolete, unsigned short flags)
{
  struct dst_entry *dst;

  if (ops->gc && dst_entries_get_fast(ops) > ops->gc_thresh) {
    if (ops->gc(ops))
      return NULL;
  }

  dst = kmem_cache_alloc(ops->kmem_cachep, GFP_ATOMIC);
  if (!dst)
    return NULL;

  dst_init(dst, ops, dev, initial_ref, initial_obsolete, flags);

  return dst;
}

void *dst_alloc(struct dst_ops *ops, struct net_device *dev,
    int initial_ref, int initial_obsolete, unsigned short flags)
{
  struct dst_entry *dst;

  if (ops->gc && dst_entries_get_fast(ops) > ops->gc_thresh) {
    if (ops->gc(ops))
      return NULL;
  }

  dst = kmem_cache_alloc(ops->kmem_cachep, GFP_ATOMIC);
  if (!dst)
    return NULL;

  dst_init(dst, ops, dev, initial_ref, initial_obsolete, flags);

  return dst;
}

void dst_init(struct dst_entry *dst, struct dst_ops *ops,
        struct net_device *dev, int initial_ref, int initial_obsolete,
        unsigned short flags)
{
  dst->dev = dev;
  if (dev)
    dev_hold(dev);
  dst->ops = ops;
  dst_init_metrics(dst, dst_default_metrics.metrics, true);
  dst->expires = 0UL;
  dst->xfrm = NULL;
  dst->input = dst_discard;
  dst->output = dst_discard_out;
  dst->error = 0;
  dst->obsolete = initial_obsolete;
  dst->header_len = 0;
  dst->trailer_len = 0;
  dst->tclassid = 0;
  dst->lwtstate = NULL;
  atomic_set(&dst->__refcnt, initial_ref);
  dst->__use = 0;
  dst->lastuse = jiffies;
  dst->flags = flags;
  if (!(flags & DST_NOCOUNT))
    dst_entries_add(ops, 1);
}
```


# bridge

<img src='../images/kernel/net-bridge.svg' style='max-height:850px'/>

[lab](https://github.com/yanfeizhang/coder-kung-fu/tree/main/tests/network/test05)

* [Linux虚拟网络设备之bridge(桥)](https://segmentfault.com/a/1190000009491002)

<img src='../images/kernel/net-filter-3.png' style='max-height:850px'/>

```c
+----------------------------------------------------------------+
|                                                                |
|       +------------------------------------------------+       |
|       |             Network Protocol Stack             |       |
|       +------------------------------------------------+       |
|                         ↑                           ↑          |
|.........................|...........................|..........|
|                         ↓                           ↓          |
|        +------+     +--------+     +-------+    +-------+      |
|        |      |     | .3.101 |     |       |    | .3.102|      |
|        +------+     +--------+     +-------+    +-------+      |
|        | eth0 |<--->|   br0  |<--->| veth0 |    | veth1 |      |
|        +------+     +--------+     +-------+    +-------+      |
|            ↑                           ↑            ↑          |
|            |                           |            |          |
|            |                           +------------+          |
|            |                                                   |
+------------|---------------------------------------------------+
             ↓
     Physical Network
```


```c
+----------------------------------------------------------------+-----------------------------------------+-----------------------------------------+
|                          Host                                  |              VirtualMachine1            |              VirtualMachine2            |
|                                                                |                                         |                                         |
|       +------------------------------------------------+       |       +-------------------------+       |       +-------------------------+       |
|       |             Network Protocol Stack             |       |       |  Network Protocol Stack |       |       |  Network Protocol Stack |       |
|       +------------------------------------------------+       |       +-------------------------+       |       +-------------------------+       |
|                          ↑                                     |                   ↑                     |                    ↑                    |
|..........................|.....................................|...................|.....................|....................|....................|
|                          ↓                                     |                   ↓                     |                    ↓                    |
|                     +--------+                                 |               +-------+                 |                +-------+                |
|                     | .3.101 |                                 |               | .3.102|                 |                | .3.103|                |
|        +------+     +--------+     +-------+                   |               +-------+                 |                +-------+                |
|        | eth0 |<--->|   br0  |<--->|tun/tap|                   |               | eth0  |                 |                | eth0  |                |
|        +------+     +--------+     +-------+                   |               +-------+                 |                +-------+                |
|            ↑             ↑             ↑                       |                   ↑                     |                    ↑新·                    |
|            |             |             +-------------------------------------------+                     |                    |                    |
|            |             ↓                                     |                                         |                    |                    |
|            |         +-------+                                 |                                         |                    |                    |
|            |         |tun/tap|                                 |                                         |                    |                    |
|            |         +-------+                                 |                                         |                    |                    |
|            |             ↑                                     |                                         |                    |                    |
|            |             +-------------------------------------------------------------------------------|--------------------+                    |
|            |                                                   |                                         |                                         |
|            |                                                   |                                         |                                         |
|            |                                                   |                                         |                                         |
+------------|---------------------------------------------------+-----------------------------------------+-----------------------------------------+
             ↓
     Physical Network  (192.168.3.0/24)
```

```c
+----------------------------------------------------------------+-----------------------------------------+-----------------------------------------+
|                          Host                                  |              Container 1                |              Container 2                |
|                                                                |                                         |                                         |
|       +------------------------------------------------+       |       +-------------------------+       |       +-------------------------+       |
|       |             Network Protocol Stack             |       |       |  Network Protocol Stack |       |       |  Network Protocol Stack |       |
|       +------------------------------------------------+       |       +-------------------------+       |       +-------------------------+       |
|            ↑             ↑                                     |                   ↑                     |                    ↑                    |
|............|.............|.....................................|...................|.....................|....................|....................|
|            ↓             ↓                                     |                   ↓                     |                    ↓                    |
|        +------+     +--------+                                 |               +-------+                 |                +-------+                |
|        |.3.101|     |  .9.1  |                                 |               |  .9.2 |                 |                |  .9.3 |                |
|        +------+     +--------+     +-------+                   |               +-------+                 |                +-------+                |
|        | eth0 |     |   br0  |<--->|  veth |                   |               | eth0  |                 |                | eth0  |                |
|        +------+     +--------+     +-------+                   |               +-------+                 |                +-------+                |
|            ↑             ↑             ↑                       |                   ↑                     |                    ↑                    |
|            |             |             +-------------------------------------------+                     |                    |                    |
|            |             ↓                                     |                                         |                    |                    |
|            |         +-------+                                 |                                         |                    |                    |
|            |         |  veth |                                 |                                         |                    |                    |
|            |         +-------+                                 |                                         |                    |                    |
|            |             ↑                                     |                                         |                    |                    |
|            |             +-------------------------------------------------------------------------------|--------------------+                    |
|            |                                                   |                                         |                                         |
|            |                                                   |                                         |                                         |
|            |                                                   |                                         |                                         |
+------------|---------------------------------------------------+-----------------------------------------+-----------------------------------------+
             ↓
     Physical Network  (192.168.3.0/24)
```

```c
struct net_bridge {
    struct list_head      port_list;
    struct net_device     *dev;

    struct rhashtable     fdb_hash_tbl;

    spinlock_t            multicast_lock;
    struct net_bridge_mdb_htable __rcu *mdb;
    struct hlist_head     router_list;

    struct kobject        *ifobj;
    struct hlist_head     fdb_list;
};

struct net_bridge_port {
    struct net_bridge     *br;
    struct net_device     *dev;
    struct list_head      list;

    port_id               port_id;
    struct timer_list     forward_delay_timer;
    struct timer_list     hold_timer;
    struct timer_list     message_age_timer;
    struct kobject        kobj;
    struct rcu_head       rcu;
    struct netpoll        *np;
};

const struct net_device_ops br_netdev_ops = {
    .ndo_open     = br_dev_open,
    .ndo_stop     = br_dev_stop,
    .ndo_init     = br_dev_init,
    .ndo_do_ioctl = br_dev_ioctl,
    .ndo_start_xmit   = br_dev_xmit,
};
```

## br_add_bridge

```c
int br_add_bridge(struct net *net, const char *name)
{
    struct net_device *dev;
    int res;

    dev = alloc_netdev(sizeof(struct net_bridge), name, NET_NAME_UNKNOWN, br_dev_setup);
    dev_net_set(dev, net);
    dev->rtnl_link_ops = &br_link_ops;

    /* making it visible to the network stack and user space (e.g., as eth0). */
    res = register_netdev(dev);

    return res;
}

#define alloc_netdev(sizeof_priv, name, setup) \
  alloc_netdev_mqs(sizeof_priv, name, setup, 1, 1)
```

## br_add_if

```c
int br_dev_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
    struct net_bridge *br = netdev_priv(dev);

    switch (cmd) {
    case SIOCDEVPRIVATE:
        return old_dev_ioctl(dev, rq, cmd);

    case SIOCBRADDIF:
    case SIOCBRDELIF:
        return add_del_if(br, rq->ifr_ifindex, cmd == SIOCBRADDIF);
    }

    return -EOPNOTSUPP;
}

int add_del_if(struct net_bridge *br, int ifindex, int isadd)
{
    struct net *net = dev_net(br->dev);
    struct net_device *dev;
    int ret;

    if (!ns_capable(net->user_ns, CAP_NET_ADMIN))
        return -EPERM;

    dev = __dev_get_by_index(net, ifindex);
    if (dev == NULL)
        return -EINVAL;

    if (isadd)
        ret = br_add_if(br, dev, NULL);
    else
        ret = br_del_if(br, dev);

    return ret;
}

int br_add_if(struct net_bridge *br, struct net_device *dev,
        struct netlink_ext_ack *extack)
{
  struct net_bridge_port *p;
  int err = 0;
  unsigned br_hr, dev_hr;
  bool changed_addr, fdb_synced = false;

  /* No bridging of bridges */
  if (dev->netdev_ops->ndo_start_xmit == br_dev_xmit) {
    NL_SET_ERR_MSG(extack, "Can not enslave a bridge to a bridge");
    return -ELOOP;
  }

  /* Device has master upper dev */
  if (netdev_master_upper_dev_get(dev))
    return -EBUSY;

  /* No bridging devices that dislike that (e.g. wireless) */
  if (dev->priv_flags & IFF_DONT_BRIDGE) {
    NL_SET_ERR_MSG(extack, "Device does not allow enslaving to a bridge");
    return -EOPNOTSUPP;
  }

  p = new_nbp(br, dev);

  call_netdevice_notifiers(NETDEV_JOIN, dev);

  err = dev_set_allmulti(dev, 1);

  err = kobject_init_and_add(&p->kobj, &brport_ktype, &(dev->dev.kobj),
           SYSFS_BRIDGE_PORT_ATTR);
  if (err)
    goto err2;

  err = br_sysfs_addif(p);
  if (err)
    goto err2;

  err = br_netpoll_enable(p);
  if (err)
    goto err3;

  err = netdev_rx_handler_register(dev, br_handle_frame, p);
  if (err)
    goto err4;

  dev->priv_flags |= IFF_BRIDGE_PORT;

  err = netdev_master_upper_dev_link(dev, br->dev, NULL, NULL, extack);
  if (err)
    goto err5;

  err = nbp_switchdev_mark_set(p);
  if (err)
    goto err6;

  dev_disable_lro(dev);

  list_add_rcu(&p->list, &br->port_list);

  nbp_update_port_count(br);
  if (!br_promisc_port(p) && (p->dev->priv_flags & IFF_UNICAST_FLT)) {
    fdb_synced = br_fdb_sync_static(br, p) == 0;
    if (!fdb_synced)
      netdev_err(dev, "failed to sync bridge static fdb addresses to this port\n");
  }

  netdev_update_features(br->dev);

  br_hr = br->dev->needed_headroom;
  dev_hr = netdev_get_fwd_headroom(dev);
  if (br_hr < dev_hr)
    update_headroom(br, dev_hr);
  else
    netdev_set_rx_headroom(dev, br_hr);

  if (br_fdb_insert(br, p, dev->dev_addr, 0))
    netdev_err(dev, "failed insert local address bridge forwarding table\n");

  err = nbp_vlan_init(p);
  if (err) {
    netdev_err(dev, "failed to initialize vlan filtering on this port\n");
    goto err7;
  }

  spin_lock_bh(&br->lock);
  changed_addr = br_stp_recalculate_bridge_id(br);

  if (netif_running(dev) && netif_oper_up(dev) && (br->dev->flags & IFF_UP))
    br_stp_enable_port(p);
  spin_unlock_bh(&br->lock);

  br_ifinfo_notify(RTM_NEWLINK, NULL, p);

  if (changed_addr)
    call_netdevice_notifiers(NETDEV_CHANGEADDR, br->dev);

  br_mtu_auto_adjust(br);
  br_set_gso_limits(br);

  kobject_uevent(&p->kobj, KOBJ_ADD);

  return 0;

err7:
  if (fdb_synced)
    br_fdb_unsync_static(br, p);
  list_del_rcu(&p->list);
  br_fdb_delete_by_port(br, p, 0, 1);
  nbp_update_port_count(br);
err6:
  netdev_upper_dev_unlink(dev, br->dev);
err5:
  dev->priv_flags &= ~IFF_BRIDGE_PORT;
  netdev_rx_handler_unregister(dev);
err4:
  br_netpoll_disable(p);
err3:
  sysfs_remove_link(br->ifobj, p->dev->name);
err2:
  br_multicast_del_port(p);
  kobject_put(&p->kobj);
  dev_set_allmulti(dev, -1);
err1:
  dev_put(dev);
  return err;
}

int netdev_rx_handler_register(struct net_device *dev,
             rx_handler_func_t *rx_handler,
             void *rx_handler_data)
{
  if (netdev_is_rx_handler_busy(dev))
    return -EBUSY;

  if (dev->priv_flags & IFF_NO_RX_HANDLER)
    return -EINVAL;

  /* Note: rx_handler_data must be set before rx_handler */
  rcu_assign_pointer(dev->rx_handler_data, rx_handler_data);
  rcu_assign_pointer(dev->rx_handler, rx_handler);

  return 0;
}
```

## br_handle_frame

```c
rx_handler_result_t br_handle_frame(struct sk_buff **pskb)
{
  struct net_bridge_port *p;
  struct sk_buff *skb = *pskb;
  const unsigned char *dest = eth_hdr(skb)->h_dest;
  br_should_route_hook_t *rhook;

  if (unlikely(skb->pkt_type == PACKET_LOOPBACK))
    return RX_HANDLER_PASS;

  if (!is_valid_ether_addr(eth_hdr(skb)->h_source))
    goto drop;

  skb = skb_share_check(skb, GFP_ATOMIC);
  if (!skb)
    return RX_HANDLER_CONSUMED;

  p = br_port_get_rcu(skb->dev);
  if (p->flags & BR_VLAN_TUNNEL) {
    if (br_handle_ingress_vlan_tunnel(skb, p, nbp_vlan_group_rcu(p)))
      goto drop;
  }

  if (unlikely(is_link_local_ether_addr(dest))) {
    u16 fwd_mask = p->br->group_fwd_mask_required;
    /* See IEEE 802.1D Table 7-10 Reserved addresses
     *
     *    Assignment                Value
     * Bridge Group Address     01-80-C2-00-00-00
     * (MAC Control) 802.3      01-80-C2-00-00-01
     * (Link Aggregation)802.3  01-80-C2-00-00-02
     * 802.1X PAE address       01-80-C2-00-00-03
     *
     * 802.1AB LLDP             01-80-C2-00-00-0E
     *
     * Others reserved for future standardization */

    fwd_mask |= p->group_fwd_mask;
    switch (dest[5]) {
    case 0x00:  /* Bridge Group Address */
      /* If STP is turned off, then must forward to keep loop detection */
      if (p->br->stp_enabled == BR_NO_STP || fwd_mask & (1u << dest[5]))
        goto forward;
      *pskb = skb;
      __br_handle_local_finish(skb);
      return RX_HANDLER_PASS;

    case 0x01:  /* IEEE MAC (Pause) */
      goto drop;

    case 0x0E:  /* 802.1AB LLDP */
      fwd_mask |= p->br->group_fwd_mask;
      if (fwd_mask & (1u << dest[5]))
        goto forward;
      *pskb = skb;
      __br_handle_local_finish(skb);
      return RX_HANDLER_PASS;

    default:
      /* Allow selective forwarding for most other protocols */
      fwd_mask |= p->br->group_fwd_mask;
      if (fwd_mask & (1u << dest[5]))
        goto forward;
    }

    /* The else clause should be hit when nf_hook():
     *   - returns < 0 (drop/error)
     *   - returns = 0 (stolen/nf_queue)
     * Thus return 1 from the okfn() to signal the skb is ok to pass */
    if (NF_HOOK(NFPROTO_BRIDGE, NF_BR_LOCAL_IN, dev_net(skb->dev),
      NULL, skb, skb->dev, NULL, br_handle_local_finish) == 1)
    {
      return RX_HANDLER_PASS;
    } else {
      return RX_HANDLER_CONSUMED;
    }
  }

forward:
  switch (p->state) {
  case BR_STATE_FORWARDING:
    rhook = rcu_dereference(br_should_route_hook);
    if (rhook) {
      if ((*rhook)(skb)) {
        *pskb = skb;
        return RX_HANDLER_PASS;
      }
      dest = eth_hdr(skb)->h_dest;
    }
    /* fall through */
  case BR_STATE_LEARNING:
    if (ether_addr_equal(p->br->dev->dev_addr, dest))
      skb->pkt_type = PACKET_HOST;

    NF_HOOK(NFPROTO_BRIDGE, NF_BR_PRE_ROUTING, dev_net(skb->dev),
      NULL, skb, skb->dev, NULL, br_handle_frame_finish
    );
    break;
  default:
drop:
    kfree_skb(skb);
  }
  return RX_HANDLER_CONSUMED;
}

int br_handle_frame_finish(struct net *net, struct sock *sk, struct sk_buff *skb)
{
  struct net_bridge_port *p = br_port_get_rcu(skb->dev);
  enum br_pkt_type pkt_type = BR_PKT_UNICAST;
  struct net_bridge_fdb_entry *dst = NULL;
  struct net_bridge_mdb_entry *mdst;
  bool local_rcv, mcast_hit = false;
  struct net_bridge *br;
  u16 vid = 0;

  if (!p || p->state == BR_STATE_DISABLED)
    goto drop;

  if (!br_allowed_ingress(p->br, nbp_vlan_group_rcu(p), skb, &vid))
    goto out;

  nbp_switchdev_frame_mark(p, skb);

  /* insert into forwarding database after filtering to avoid spoofing */
  br = p->br;
  if (p->flags & BR_LEARNING)
    br_fdb_update(br, p, eth_hdr(skb)->h_source, vid, false);

  local_rcv = !!(br->dev->flags & IFF_PROMISC);
  if (is_multicast_ether_addr(eth_hdr(skb)->h_dest)) {
    /* by definition the broadcast is also a multicast address */
    if (is_broadcast_ether_addr(eth_hdr(skb)->h_dest)) {
      pkt_type = BR_PKT_BROADCAST;
      local_rcv = true;
    } else {
      pkt_type = BR_PKT_MULTICAST;
      if (br_multicast_rcv(br, p, skb, vid))
        goto drop;
    }
  }

  if (p->state == BR_STATE_LEARNING)
    goto drop;

  BR_INPUT_SKB_CB(skb)->brdev = br->dev;
  BR_INPUT_SKB_CB(skb)->src_port_isolated = !!(p->flags & BR_ISOLATED);

  if (IS_ENABLED(CONFIG_INET) &&
    (skb->protocol == htons(ETH_P_ARP) || skb->protocol == htons(ETH_P_RARP)))
  {
    br_do_proxy_suppress_arp(skb, br, vid, p);
  } else if (IS_ENABLED(CONFIG_IPV6) &&
    skb->protocol == htons(ETH_P_IPV6) &&
    br->neigh_suppress_enabled &&
    pskb_may_pull(skb, sizeof(struct ipv6hdr) +
    sizeof(struct nd_msg)) &&
    ipv6_hdr(skb)->nexthdr == IPPROTO_ICMPV6)
  {
      struct nd_msg *msg, _msg;

      msg = br_is_nd_neigh_msg(skb, &_msg);
      if (msg)
        br_do_suppress_nd(skb, br, vid, p, msg);
  }

  switch (pkt_type) {
  case BR_PKT_MULTICAST:
    mdst = br_mdb_get(br, skb, vid);
    if ((mdst || BR_INPUT_SKB_CB_MROUTERS_ONLY(skb)) && br_multicast_querier_exists(br, eth_hdr(skb)))
    {
      if ((mdst && mdst->host_joined) || br_multicast_is_router(br)) {
        local_rcv = true;
        br->dev->stats.multicast++;
      }
      mcast_hit = true;
    } else {
      local_rcv = true;
      br->dev->stats.multicast++;
    }
    break;
  case BR_PKT_UNICAST:
    dst = br_fdb_find_rcu(br, eth_hdr(skb)->h_dest, vid);
  default:
    break;
  }

  if (dst) {
    unsigned long now = jiffies;

    if (dst->is_local)
      return br_pass_frame_up(skb);

    if (now != dst->used)
      dst->used = now;
    br_forward(dst->dst, skb, local_rcv, false);
  } else {
    if (!mcast_hit)
      br_flood(br, skb, pkt_type, local_rcv, false);
    else
      br_multicast_flood(mdst, skb, local_rcv, false);
  }

  if (local_rcv)
    return br_pass_frame_up(skb);

out:
  return 0;
drop:
  kfree_skb(skb);
  goto out;
}
```

### br_forward
```c
void br_forward(const struct net_bridge_port *to,
    struct sk_buff *skb, bool local_rcv, bool local_orig)
{
  if (unlikely(!to))
    goto out;

  /* redirect to backup link if the destination port is down */
  if (rcu_access_pointer(to->backup_port) && !netif_carrier_ok(to->dev)) {
    struct net_bridge_port *backup_port;

    backup_port = rcu_dereference(to->backup_port);
    if (unlikely(!backup_port))
      goto out;
    to = backup_port;
  }

  if (should_deliver(to, skb)) {
    if (local_rcv)
      deliver_clone(to, skb, local_orig);
    else
      __br_forward(to, skb, local_orig);
    return;
  }

out:
  if (!local_rcv)
    kfree_skb(skb);
}

void __br_forward(const struct net_bridge_port *to,
       struct sk_buff *skb, bool local_orig)
{
  struct net_bridge_vlan_group *vg;
  struct net_device *indev;
  struct net *net;
  int br_hook;

  vg = nbp_vlan_group_rcu(to);
  skb = br_handle_vlan(to->br, to, vg, skb);
  if (!skb)
    return;

  indev = skb->dev;
  skb->dev = to->dev;
  if (!local_orig) {
    if (skb_warn_if_lro(skb)) {
      kfree_skb(skb);
      return;
    }
    br_hook = NF_BR_FORWARD;
    skb_forward_csum(skb);
    net = dev_net(indev);
  } else {
    if (unlikely(netpoll_tx_running(to->br->dev))) {
      skb_push(skb, ETH_HLEN);
      if (!is_skb_forwardable(skb->dev, skb))
        kfree_skb(skb);
      else
        br_netpoll_send_skb(to, skb);
      return;
    }
    br_hook = NF_BR_LOCAL_OUT;
    net = dev_net(skb->dev);
    indev = NULL;
  }

  NF_HOOK(NFPROTO_BRIDGE, br_hook,
    net, NULL, skb, indev, skb->dev,
    br_forward_finish);
}

int br_forward_finish(struct net *net, struct sock *sk, struct sk_buff *skb)
{
  skb->tstamp = 0;
  return NF_HOOK(NFPROTO_BRIDGE, NF_BR_POST_ROUTING,
    net, sk, skb, NULL, skb->dev,
    br_dev_queue_push_xmit);

}

int br_dev_queue_push_xmit(struct net *net, struct sock *sk, struct sk_buff *skb)
{
  skb_push(skb, ETH_HLEN);
  if (!is_skb_forwardable(skb->dev, skb))
    goto drop;

  br_drop_fake_rtable(skb);

  if (skb->ip_summed == CHECKSUM_PARTIAL &&
    (skb->protocol == htons(ETH_P_8021Q) ||
    skb->protocol == htons(ETH_P_8021AD)))
  {
    int depth;

    if (!__vlan_get_protocol(skb, skb->protocol, &depth))
      goto drop;

    skb_set_network_header(skb, depth);
  }

  dev_queue_xmit(skb);

  return 0;

drop:
  kfree_skb(skb);
  return 0;
}
```

### br_pass_frame_up
```c

int br_pass_frame_up(struct sk_buff *skb)
{
  struct net_device *indev, *brdev = BR_INPUT_SKB_CB(skb)->brdev;
  struct net_bridge *br = netdev_priv(brdev);
  struct net_bridge_vlan_group *vg;
  struct pcpu_sw_netstats *brstats = this_cpu_ptr(br->stats);

  vg = br_vlan_group_rcu(br);
  /* Bridge is just like any other port.  Make sure the
   * packet is allowed except in promisc modue when someone
   * may be running packet capture. */
  if (!(brdev->flags & IFF_PROMISC) && !br_allowed_egress(vg, skb)) {
    kfree_skb(skb);
    return NET_RX_DROP;
  }

  indev = skb->dev;
  skb->dev = brdev;
  skb = br_handle_vlan(br, NULL, vg, skb);
  if (!skb)
    return NET_RX_DROP;
  /* update the multicast stats if the packet is IGMP/MLD */
  br_multicast_count(br, NULL, skb, br_multicast_igmp_type(skb), BR_MCAST_DIR_TX);

  return NF_HOOK(NFPROTO_BRIDGE, NF_BR_LOCAL_IN,
    dev_net(indev), NULL, skb, indev, NULL,
    br_netif_receive_skb);
}

int br_netif_receive_skb(struct net *net, struct sock *sk, struct sk_buff *skb)
{
  br_drop_fake_rtable(skb);
  return netif_receive_skb(skb);
}
```

```c
br_add_bridge()
    struct net_device* dev = alloc_netdev(sizeof(struct net_bridge));
    dev_net_set(dev, net);
    dev->rtnl_link_ops = &br_link_ops;
    register_netdev(dev);

br_dev_ioctl()
    br_add_if()
        struct net_bridge_port *p = new_nbp(br, dev);
        br_sysfs_addif(p);
        br_netpoll_enable(p);
        netdev_rx_handler_register(dev, br_handle_frame, p);
            dev->rx_handler_data = p;
          dev->rx_handler = br_handle_frame;
        list_add_rcu(&p->list, &br->port_list);
        br_fdb_insert(br, p, dev->dev_addr, 0);
        nbp_vlan_init(p);

br_handle_frame()
    NF_HOOK(NFPROTO_BRIDGE, NF_BR_PRE_ROUTING,br_handle_frame_finish);
        br_fdb_update(br, p, eth_hdr(skb)->h_source, vid, false)
        struct net_bridge_fdb_entry *dst = br_fdb_find_rcu(br, eth_hdr(skb)->h_dest, vid);

        br_pass_frame_up(skb);
            skb->dev = brdev;
            NF_HOOK(NFPROTO_BRIDGE, NF_BR_LOCAL_IN, br_netif_receive_skb);
                netif_receive_skb(skb);

        br_forward(dst->dst, skb);
            if (should_deliver(to, skb))
                skb->dev = to->dev;
                NF_HOOK(NFPROTO_BRIDGE, NF_BR_LOCAL_OUT br_forward_finish)
                    NF_HOOK(NFPROTO_BRIDGE,NF_BR_POST_ROUTING, br_dev_queue_push_xmit)
                        br_dev_queue_push_xmit()
                            dev_queue_xmit(skb);

        br_flood(br, skb);

        br_multicast_flood(mdst, skb);
```

# veth

* [Linux虚拟网络设备之veth](https://segmentfault.com/a/1190000009251098)

```c
+----------------------------------------------------------------+
|                                                                |
|       +------------------------------------------------+       |
|       |             Network Protocol Stack             |       |
|       +------------------------------------------------+       |
|              ↑               ↑               ↑                 |
|..............|...............|...............|.................|
|              ↓               ↓               ↓                 |
|        +----------+    +-----------+   +-----------+           |
|        |   eth0   |    |   veth0   |   |   veth1   |           |
|        +----------+    +-----------+   +-----------+           |
|192.168.1.11  ↑               ↑               ↑                 |
|              |               +---------------+                 |
|              |         192.168.2.11     192.168.2.1            |
+--------------|-------------------------------------------------+
               ↓
         Physical Network
```

```c
static const struct net_device_ops veth_netdev_ops = {
  .ndo_init            = veth_dev_init,
  .ndo_open            = veth_open,
  .ndo_stop            = veth_close,
  .ndo_start_xmit      = veth_xmit,
  .ndo_get_stats64     = veth_get_stats64,
  .ndo_set_rx_mode     = veth_set_multicast_list,
  .ndo_set_mac_address = eth_mac_addr,
  .ndo_set_rx_headroom  = veth_set_rx_headroom,
  .ndo_bpf              = veth_xdp,
  .ndo_xdp_xmit         = veth_xdp_xmit,
};

static struct rtnl_link_ops veth_link_ops = {
  .kind       = DRV_NAME,
  .priv_size  = sizeof(struct veth_priv),
  .setup      = veth_setup,
  .validate   = veth_validate,
  .newlink    = veth_newlink,
  .dellink    = veth_dellink,
  .policy     = veth_policy,
  .maxtype    = VETH_INFO_MAX,
  .get_link_net  = veth_get_link_net,
};

static __init int veth_init(void)
{
  return rtnl_link_register(&veth_link_ops);
}

static LIST_HEAD(link_ops);
```

## veth_newlink
```c
/* ip link add veth0 type veth peer name veth1 */
int veth_newlink(struct net *src_net, struct net_device *dev,
      struct nlattr *tb[], struct nlattr *data[],
      struct netlink_ext_ack *extack)
{
  int err;
  struct net_device *peer;
  struct veth_priv *priv;
  char ifname[IFNAMSIZ];
  struct nlattr *peer_tb[IFLA_MAX + 1], **tbp;
  unsigned char name_assign_type;
  struct ifinfomsg *ifmp;
  struct net *net;

  /* create and register peer first */
  if (data != NULL && data[VETH_INFO_PEER] != NULL) {
    struct nlattr *nla_peer;

    nla_peer = data[VETH_INFO_PEER];
    ifmp = nla_data(nla_peer);
    err = rtnl_nla_parse_ifla(peer_tb,
            nla_data(nla_peer) + sizeof(struct ifinfomsg),
            nla_len(nla_peer) - sizeof(struct ifinfomsg),
            NULL);
    if (err < 0)
      return err;

    err = veth_validate(peer_tb, NULL, extack);
    if (err < 0)
      return err;

    tbp = peer_tb;
  } else {
    ifmp = NULL;
    tbp = tb;
  }

  if (ifmp && tbp[IFLA_IFNAME]) {
    nla_strlcpy(ifname, tbp[IFLA_IFNAME], IFNAMSIZ);
    name_assign_type = NET_NAME_USER;
  } else {
    snprintf(ifname, IFNAMSIZ, DRV_NAME "%%d");
    name_assign_type = NET_NAME_ENUM;
  }

  net = rtnl_link_get_net(src_net, tbp);

  peer = rtnl_create_link(net, ifname, name_assign_type, &veth_link_ops, tbp);

  if (!ifmp || !tbp[IFLA_ADDRESS])
    eth_hw_addr_random(peer);

  if (ifmp && (dev->ifindex != 0))
    peer->ifindex = ifmp->ifi_index;

  peer->gso_max_size = dev->gso_max_size;
  peer->gso_max_segs = dev->gso_max_segs;

  err = register_netdevice(peer);
  put_net(net);
  net = NULL;
  if (err < 0)
    goto err_register_peer;

  netif_carrier_off(peer);

  err = rtnl_configure_link(peer, ifmp);
  if (err < 0)
    goto err_configure_peer;

  /* register dev last */

  if (tb[IFLA_ADDRESS] == NULL)
    eth_hw_addr_random(dev);

  if (tb[IFLA_IFNAME])
    nla_strlcpy(dev->name, tb[IFLA_IFNAME], IFNAMSIZ);
  else
    snprintf(dev->name, IFNAMSIZ, DRV_NAME "%%d");

  err = register_netdevice(dev);
  if (err < 0)
    goto err_register_dev;

  netif_carrier_off(dev);

  /* tie the deviced together */
  priv = netdev_priv(dev);
  rcu_assign_pointer(priv->peer, peer);

  priv = netdev_priv(peer);
  rcu_assign_pointer(priv->peer, dev);

  return 0;

err_register_dev:
  /* nothing to do */
err_configure_peer:
  unregister_netdevice(peer);
  return err;

err_register_peer:
  free_netdev(peer);
  return err;
}
```

## veth_setup
```c
void veth_setup(struct net_device *dev)
{
  ether_setup(dev);

  dev->priv_flags &= ~IFF_TX_SKB_SHARING;
  dev->priv_flags |= IFF_LIVE_ADDR_CHANGE;
  dev->priv_flags |= IFF_NO_QUEUE;
  dev->priv_flags |= IFF_PHONY_HEADROOM;

  dev->netdev_ops = &veth_netdev_ops;
  dev->ethtool_ops = &veth_ethtool_ops;

  dev->features |= NETIF_F_LLTX;
  dev->features |= VETH_FEATURES;
  dev->vlan_features = dev->features &
           ~(NETIF_F_HW_VLAN_CTAG_TX |
             NETIF_F_HW_VLAN_STAG_TX |
             NETIF_F_HW_VLAN_CTAG_RX |
             NETIF_F_HW_VLAN_STAG_RX);
  dev->needs_free_netdev = true;
  dev->priv_destructor = veth_dev_free;
  dev->max_mtu = ETH_MAX_MTU;

  dev->hw_features = VETH_FEATURES;
  dev->hw_enc_features = VETH_FEATURES;
  dev->mpls_features = NETIF_F_HW_CSUM | NETIF_F_GSO_SOFTWARE;
}
```

## veth_xmit
```c
int dev_hard_start_xmit(struct sk_buff *skb, struct net_device *dev,
   struct netdev_queue *txq)
{
  const struct net_device_ops *ops = dev->netdev_ops;
  ops->ndo_start_xmit(skb, dev);
}

netdev_tx_t veth_xmit(struct sk_buff *skb, struct net_device *dev)
{
  struct veth_priv *rcv_priv, *priv = netdev_priv(dev);
  struct veth_rq *rq = NULL;
  struct net_device *rcv;
  int length = skb->len;
  bool rcv_xdp = false;
  int rxq;

  rcu_read_lock();
  rcv = rcu_dereference(priv->peer);
  if (unlikely(!rcv)) {
    kfree_skb(skb);
    goto drop;
  }

  rcv_priv = netdev_priv(rcv);
  rxq = skb_get_queue_mapping(skb);
  if (rxq < rcv->real_num_rx_queues) {
    rq = &rcv_priv->rq[rxq];
    rcv_xdp = rcu_access_pointer(rq->xdp_prog);
    skb_record_rx_queue(skb, rxq);
  }

  if (likely(veth_forward_skb(rcv, skb, rq, rcv_xdp) == NET_RX_SUCCESS)) {
    struct pcpu_vstats *stats = this_cpu_ptr(dev->vstats);

    u64_stats_update_begin(&stats->syncp);
    stats->bytes += length;
    stats->packets++;
    u64_stats_update_end(&stats->syncp);
  } else {
drop:
    atomic64_inc(&priv->dropped);
  }

  if (rcv_xdp)
    __veth_xdp_flush(rq);

  rcu_read_unlock();

  return NETDEV_TX_OK;
}

int veth_forward_skb(struct net_device *dev, struct sk_buff *skb,
          struct veth_rq *rq, bool xdp)
{
  return __dev_forward_skb(dev, skb)
    ?
    : xdp
      ? veth_xdp_rx(rq, skb)
      : netif_rx(skb);
}

int netif_rx(struct sk_buff *skb)
{
  return netif_rx_internal(skb);
}


static int netif_rx_internal(struct sk_buff *skb)
{
  int ret;

  net_timestamp_check(netdev_tstamp_prequeue, skb);

  trace_netif_rx(skb);
  {
    unsigned int qtail;

    ret = enqueue_to_backlog(skb, get_cpu(), &qtail);
    put_cpu();
  }
  return ret;
}
```

# tun

* [Linux虚拟网络设备之tun/tap](https://segmentfault.com/a/1190000009249039)

```c
+----------------------------------------------------------------+
|                                                                |
|  +--------------------+      +--------------------+            |
|  | User Application A |      | User Application B |<-----+     |
|  +--------------------+      +--------------------+      |     |
|               | 1                    | 5                 |     |
|...............|......................|...................|.....|
|               ↓                      ↓                   |     |
|         +----------+           +----------+              |     |
|         | socket A |           | socket B |              |     |
|         +----------+           +----------+              |     |
|                 | 2               | 6                    |     |
|.................|.................|......................|.....|
|                 ↓                 ↓                      |     |
|             +------------------------+                 4 |     |
|             | Network Protocol Stack |                   |     |
|             +------------------------+                   |     |
|                | 7                 | 3                   |     |
|................|...................|.....................|.....|
|                ↓                   ↓                     |     |
|        +----------------+    +----------------+          |     |
|        |      eth0      |    |      tun0      |          |     |
|        +----------------+    +----------------+          |     |
|    10.32.0.11  |                   |   192.168.3.11      |     |
|                | 8                 +---------------------+     |
|                |                                               |
+----------------|-----------------------------------------------+
                 ↓
         Physical Network
```

# congestion control

```c
[root@VM-16-17-centos ~]# cat /proc/sys/net/ipv4/tcp_congestion_control
cubic
```

<img src='../images/kernel/net-cubic-algorithm.png' style='max-height:850px'/>
<img src='../images/kernel/net-cubic-algorithm-growth.png' style='max-height:850px'/>

* β: Multiplicative decrease factor
* wmax: Window size just before the last reduction
* T: Time elapsed since the last window reduction
* C: A scaling constant
* cwnd: The congestion window at the current time

RFC 8312 indicates the following:
* The unit of all window sizes in this document is segments of the maximum segment size (MSS), and the unit of all times is seconds. (Section 4)
* β SHOULD be set to 0.7 (Section 4.5)
* C SHOULD be set to 0.4 (Section 5)

CUBIC is a less aggressive and more systematic derivative of BIC TCP, in which the window size is a cubic function of time since the last congestion event, with the inflection point set to the window size prior to the event.

Because it is a cubic function, there are two components to window growth. The first is a concave portion where the window size quickly ramps up to the size before the last congestion event. Next is the convex growth where CUBIC probes for more bandwidth, slowly at first then very rapidly.

CUBIC spends a lot of time at a plateau between the concave and convex growth region which allows the network to stabilize before CUBIC begins looking for more bandwidth.

Another major difference between CUBIC and many earlier TCP algorithms is that it does not rely on the cadence of RTTs to increase the window size. CUBIC's window size is dependent only on the last congestion event. With earlier algorithms like TCP New Reno, flows with very short round-trip delay times (RTTs) will receive ACKs faster and therefore have their congestion windows grow faster than other flows with longer RTTs. CUBIC allows for more fairness between flows since the window growth is independent of RTT.


## tcp_ca_state

<img src='../images/kernel/net-ca-state.svg' style='max-height:850px'/>

Linux NewReno/SACK/ECN state machine.

* **Open** Normal state, no dubious events, fast path.
* **Disorder** In all the respects it is "Open", but requires a bit more attention. It is entered when we see some SACKs or dupacks. It is split of "Open" mainly to move some processing from fast path to slow one.
* **CWR** CWND was reduced due to some Congestion Notification event. It can be ECN, ICMP source quench, local device congestion.
* **Recovery** CWND was reduced, we are fast-retransmitting.
* **Loss** CWND was reduced due to RTO timeout or SACK reneging.

tcp_fastretrans_alert() is entered:
- each incoming ACK, if state is not "Open"
- when arrived ACK is unusual, namely:
    * SACK
    * Duplicate ACK.
    * ECN ECE.

 Counting packets in flight is pretty simple.
 * in_flight = packets_out - left_out + retrans_out
    * packets_out is SND.NXT-SND.UNA counted in packets.
    * retrans_out is number of retransmitted segments.
    * left_out is number of segments left network, but not ACKed yet.
    * left_out = sacked_out + lost_out
        * sacked_out: Packets, which arrived to receiver out of order and hence not ACKed. With SACKs this number is simply amount of SACKed data. Even without SACKs it is easy to give pretty reliable estimate of this number, counting duplicate ACKs.
        * lost_out: Packets lost by network. TCP has no explicit "loss notification" feedback from network (for now). It means that this number can be only _guessed_. Actually, it is the heuristics to predict lossage that distinguishes different algorithms.

* F.e. after RTO, when all the queue is considered as lost, lost_out = packets_out and in_flight = retrans_out.
    * Essentially, we have now a few algorithms detecting lost packets.
        * If the receiver supports SACK:
            * RFC6675/3517: It is the conventional algorithm. A packet is considered lost if the number of higher sequence packets SACKed is greater than or equal the DUPACK thoreshold (reordering). This is implemented in tcp_mark_head_lost and tcp_update_scoreboard.
            * RACK (draft-ietf-tcpm-rack-01): it is a newer algorithm (2017-) that checks timing instead of counting DUPACKs. Essentially a packet is considered lost if it's not S/ACKed after RTT + reordering_window, where both metrics are dynamically measured and adjusted. This is implemented in tcp_rack_mark_lost.

        * If the receiver does not support SACK:
            * NewReno (RFC6582): in Recovery we assume that one segment is lost (classic Reno). While we are in Recovery and a partial ACK arrives, we assume that one more packet is lost (NewReno). This heuristics are the same in NewReno and SACK.

* Really tricky (and requiring careful tuning) part of algorithm is hidden in functions **tcp_time_to_recover**() and tcp_xmit_retransmit_queue(). The first determines the moment _when_ we should reduce CWND and, hence, slow down forward transmission. In fact, it determines the moment when we decide that hole is caused by loss, rather than by a reorder.

* **tcp_xmit_retransmit_queue**() decides, _what_ we should retransmit to fill
    holes, caused by lost packets.

* And the most logically complicated part of algorithm is undo heuristics. We detect false retransmits due to both too early fast retransmit (reordering) and underestimated RTO, analyzing timestamps and D-SACKs. When we detect that some segments were retransmitted by mistake and CWND reduction was wrong, we undo window reduction and abort recovery phase. This logic is hidden inside several functions named tcp_try_undo_<something>.

```c
enum tcp_ca_state {
  TCP_CA_Open = 0,
#define TCPF_CA_Open  (1<<TCP_CA_Open)
  TCP_CA_Disorder = 1,
#define TCPF_CA_Disorder (1<<TCP_CA_Disorder)
  TCP_CA_CWR = 2,
#define TCPF_CA_CWR  (1<<TCP_CA_CWR)
  TCP_CA_Recovery = 3,
#define TCPF_CA_Recovery (1<<TCP_CA_Recovery)
  TCP_CA_Loss = 4
#define TCPF_CA_Loss  (1<<TCP_CA_Loss)
};

void tcp_set_ca_state(struct sock *sk, const u8 ca_state)
{
  struct inet_connection_sock *icsk = inet_csk(sk);

  if (icsk->icsk_ca_ops->set_state)
    icsk->icsk_ca_ops->set_state(sk, ca_state);
  icsk->icsk_ca_state = ca_state;
}
```

### TCPF_CA_Recovery

During the Recovery state, the size of the congestion window is reduced by one segment for every new acknowledgment, similar to the CWR state. This window reduction process terminates when the congestion window size equal to ssthresh, which is half of the window size when entering the Recovery state (sshresh = cwnd / 2). The congestion window does not increase during recovery, and the sender retransmits segments marked as lost, or marks forward transmission on new data according to the packet conservation principle. The sender remains in the Recovery state until all data segments being sent when entering the Recovery state are successfully acknowledged (snd_una > high_seq), after which the sender resumes the OPEN state, and the retransmission timeout may interrupt the Recovery state.

```c
void tcp_enter_recovery(struct sock *sk, bool ece_ack)
{
  struct tcp_sock *tp = tcp_sk(sk);
  int mib_idx;

  if (tcp_is_reno(tp))
    mib_idx = LINUX_MIB_TCPRENORECOVERY;
  else
    mib_idx = LINUX_MIB_TCPSACKRECOVERY;

  NET_INC_STATS(sock_net(sk), mib_idx);

  tp->prior_ssthresh = 0;
  tcp_init_undo(tp);

  if (!tcp_in_cwnd_reduction(sk)) {
    if (!ece_ack)
      tp->prior_ssthresh = tcp_current_ssthresh(sk);
    tcp_init_cwnd_reduction(sk);
  }
  tcp_set_ca_state(sk, TCP_CA_Recovery);
}

void tcp_init_cwnd_reduction(struct sock *sk)
{
  struct tcp_sock *tp = tcp_sk(sk);

  tp->high_seq = tp->snd_nxt;
  tp->tlp_high_seq = 0;
  tp->snd_cwnd_cnt = 0;
  tp->prior_cwnd = tp->snd_cwnd;
  tp->prr_delivered = 0;
  tp->prr_out = 0;
  tp->snd_ssthresh = inet_csk(sk)->icsk_ca_ops->ssthresh(sk);
  tcp_ecn_queue_cwr(tp);
}
```

### TCP_CA_CWR

When receiving a congestion notification, the sender does not reduce the congestion window immediately, but reduces the congestion window by one segment for every new ACK until the size of the window is halved. The sender will not have significant retransmissions in the process of reducing the congestion window size.

```c
void tcp_enter_cwr(struct sock *sk)
{
  struct tcp_sock *tp = tcp_sk(sk);

  tp->prior_ssthresh = 0;
  if (inet_csk(sk)->icsk_ca_state < TCP_CA_CWR) {
    tp->undo_marker = 0;
    tcp_init_cwnd_reduction(sk);
    tcp_set_ca_state(sk, TCP_CA_CWR);
  }
}

void tcp_init_cwnd_reduction(struct sock *sk)
{
  struct tcp_sock *tp = tcp_sk(sk);

  tp->high_seq = tp->snd_nxt;
  tp->tlp_high_seq = 0;
  tp->snd_cwnd_cnt = 0;
  tp->prior_cwnd = tp->snd_cwnd;
  tp->prr_delivered = 0;
  tp->prr_out = 0;
  tp->snd_ssthresh = inet_csk(sk)->icsk_ca_ops->ssthresh(sk);
  tcp_ecn_queue_cwr(tp);
}

void tcp_ecn_queue_cwr(struct tcp_sock *tp)
{
  if (tp->ecn_flags & TCP_ECN_OK)
    tp->ecn_flags |= TCP_ECN_QUEUE_CWR;
}

```

### TCP_CA_Loss
```c
void tcp_enter_loss(struct sock *sk)
{
  const struct inet_connection_sock *icsk = inet_csk(sk);
  struct tcp_sock *tp = tcp_sk(sk);
  struct net *net = sock_net(sk);
  bool new_recovery = icsk->icsk_ca_state < TCP_CA_Recovery;

  tcp_timeout_mark_lost(sk);

  /* Reduce ssthresh if it has not yet been made inside this window. */
  if (icsk->icsk_ca_state <= TCP_CA_Disorder ||
      !after(tp->high_seq, tp->snd_una) ||
      (icsk->icsk_ca_state == TCP_CA_Loss && !icsk->icsk_retransmits))
  {
    tp->prior_ssthresh = tcp_current_ssthresh(sk);
    tp->prior_cwnd = tp->snd_cwnd;
    tp->snd_ssthresh = icsk->icsk_ca_ops->ssthresh(sk);
    tcp_ca_event(sk, CA_EVENT_LOSS);
    tcp_init_undo(tp);
  }
  tp->snd_cwnd     = tcp_packets_in_flight(tp) + 1;
  tp->snd_cwnd_cnt   = 0;
  tp->snd_cwnd_stamp = tcp_jiffies32;

  /* Timeout in disordered state after receiving substantial DUPACKs
   * suggests that the degree of reordering is over-estimated. */
  if (icsk->icsk_ca_state <= TCP_CA_Disorder && tp->sacked_out >= net->ipv4.sysctl_tcp_reordering)
    tp->reordering = min_t(unsigned int, tp->reordering, net->ipv4.sysctl_tcp_reordering);
  tcp_set_ca_state(sk, TCP_CA_Loss);
  tp->high_seq = tp->snd_nxt;
  tcp_ecn_queue_cwr(tp);

  /* F-RTO RFC5682 sec 3.1 step 1: retransmit SND.UNA if no previous
   * loss recovery is underway except recurring timeout(s) on
   * the same SND.UNA (sec 3.2). Disable F-RTO on path MTU probing */
  tp->frto = net->ipv4.sysctl_tcp_frto &&
       (new_recovery || icsk->icsk_retransmits) &&
       !inet_csk(sk)->icsk_mtup.probe_size;
}
```

### TCP_CA_Disorder

In this state, the congestion window is not adjusted, but each newly arrived segment triggers the sending of a new data segment. Therefore, TCP senders follow the packet conservation principle, which states that a new packet is only sent after an old packet has left the network.

```c
void tcp_try_keep_open(struct sock *sk)
{
  struct tcp_sock *tp = tcp_sk(sk);
  int state = TCP_CA_Open;

  if (tcp_left_out(tp) || tcp_any_retrans_done(sk))
    state = TCP_CA_Disorder;

  if (inet_csk(sk)->icsk_ca_state != state) {
    tcp_set_ca_state(sk, state);
    tp->high_seq = tp->snd_nxt;
  }
}
```

## ecn
<img src='../images/kernel/net-ecn-ip.png' style='max-height:850px'/>

```c
/* ECE: Echo of Congestion Encountered */
enum {
  INET_ECN_NOT_ECT = 0, /* tos 00 - Non ECN-Capable Transport, Non-ECT */
  INET_ECN_ECT_1 = 1,   /* tos 01 - ECN Capable Transport ECT(1)*/
  INET_ECN_ECT_0 = 2,   /* tos 10 - ECN Capable Transport ECT(0) */
  INET_ECN_CE = 3,      /* tos 11 - Congestion Encountered*/
  INET_ECN_MASK = 3,
};

/* ip support */
int IP_ECN_set_ce(struct iphdr *iph)
{
  u32 check = (__force u32)iph->check;
  u32 ecn = (iph->tos + 1) & INET_ECN_MASK;

  /* After the last operation we have (in binary):
   * INET_ECN_NOT_ECT => 01
   * INET_ECN_ECT_1   => 10
   * INET_ECN_ECT_0   => 11
   * INET_ECN_CE      => 00 */
  if (!(ecn & 2))
    return !ecn;

  /* The following gives us:
   * INET_ECN_ECT_1 => check += htons(0xFFFD)
   * INET_ECN_ECT_0 => check += htons(0xFFFE) */
  check += (__force u16)htons(0xFFFB) + (__force u16)htons(ecn);

  iph->check = (__force __sum16)(check + (check>=0xFFFF));
  iph->tos |= INET_ECN_CE;
  return 1;
}
```

<img src='../images/kernel/net-ecn-tcp.png' style='max-height:850px'/>

```c
/* tcp support */
#define TCPHDR_FIN 0x01
#define TCPHDR_SYN 0x02
#define TCPHDR_RST 0x04
#define TCPHDR_PSH 0x08
#define TCPHDR_ACK 0x10
#define TCPHDR_URG 0x20
#define TCPHDR_ECE 0x40
#define TCPHDR_CWR 0x80

#define TCPHDR_SYN_ECN  (TCPHDR_SYN | TCPHDR_ECE | TCPHDR_CWR)
```

## tcp_congestion_ops
```c
struct tcp_congestion_ops {
  struct list_head  list;
  u32 key;
  u32 flags;

  /* return slow start threshold (required) */
  u32 (*ssthresh)(struct sock *sk);
  /* do new cwnd calculation (required) */
  void (*cong_avoid)(struct sock *sk, u32 ack, u32 acked);
  /* call before changing ca_state (optional) */
  void (*set_state)(struct sock *sk, u8 new_state);
  /* call when cwnd event occurs (optional) */
  void (*cwnd_event)(struct sock *sk, enum tcp_ca_event ev);
  /* call when packets are delivered to update cwnd and pacing rate,
   * after all the ca_state processing. (optional) */
  void (*cong_control)(struct sock *sk, const struct rate_sample *rs);

  char     name[TCP_CA_NAME_MAX];
  struct module   *owner;
};

enum tcp_ca_event {
  CA_EVENT_TX_START,      /* first transmit when no packets in flight */
  CA_EVENT_CWND_RESTART,  /* congestion window restart */
  CA_EVENT_COMPLETE_CWR,  /* end of congestion recovery */
  CA_EVENT_LOSS,          /* loss timeout */
  CA_EVENT_ECN_NO_CE,     /* ECT set, but not CE marked */
  CA_EVENT_ECN_IS_CE,     /* received CE marked IP packet */
};

/* Information about inbound ACK, passed to cong_ops->in_ack_event() */
enum tcp_ca_ack_event_flags {
  CA_ACK_SLOWPATH     = (1 << 0),  /* In slow path processing */
  CA_ACK_WIN_UPDATE   = (1 << 1),  /* ACK updated window */
  CA_ACK_ECE          = (1 << 2),  /* ECE bit is set on ack */
};

struct tcp_congestion_ops cubictcp __read_mostly = {
  .init         = bictcp_init,
  .ssthresh     = bictcp_recalc_ssthresh,
  .cong_avoid   = bictcp_cong_avoid,
  .set_state    = bictcp_state,
  .undo_cwnd    = tcp_reno_undo_cwnd,
  .cwnd_event   = bictcp_cwnd_event,
  .pkts_acked   = bictcp_acked,
  .owner        = THIS_MODULE,
  .name         = "cubic",
};

void bictcp_state(struct sock *sk, u8 new_state)
{
  if (new_state == TCP_CA_Loss) {
    bictcp_reset(inet_csk_ca(sk));
    bictcp_hystart_reset(sk);
  }
}

void tcp_in_ack_event(struct sock *sk, u32 flags)
{
  const struct inet_connection_sock *icsk = inet_csk(sk);

  if (icsk->icsk_ca_ops->in_ack_event)
    icsk->icsk_ca_ops->in_ack_event(sk, flags);
}

struct bictcp {
  u32  cnt;           /* increase cwnd by 1 after ACKs */
  u32  last_max_cwnd; /* last maximum snd_cwnd */
  u32  last_cwnd;     /* the last snd_cwnd */
  u32  last_time;     /* time when updated last_cwnd */
  u32  bic_origin_point;/* origin point of bic function */
  u32  bic_K;         /* time to origin point from the beginning of the current epoch */
  u32  delay_min;     /* min delay (msec << 3) */
  u32  epoch_start;   /* beginning of an epoch */
  u32  ack_cnt;       /* number of acks */
  u32  tcp_cwnd;      /* estimated tcp cwnd */
  u16  unused;
  u8   sample_cnt;    /* number of samples to decide curr_rtt */
  u8   found;         /* the exit point is found? */
  u32  round_start;   /* beginning of each round */
  u32  end_seq;       /* end_seq of the round */
  u32  last_ack;      /* last time when the ACK spacing is close */
  u32  curr_rtt;      /* the minimum rtt of current round */
};
```

### bictcp_init
```c
void bictcp_init(struct sock *sk)
{
  struct bictcp *ca = inet_csk_ca(sk);

  bictcp_reset(ca);

  if (hystart)
    bictcp_hystart_reset(sk);

  if (!hystart && initial_ssthresh)
    tcp_sk(sk)->snd_ssthresh = initial_ssthresh;
}

void bictcp_reset(struct bictcp *ca)
{
  ca->cnt = 0;
  ca->last_max_cwnd = 0;
  ca->last_cwnd = 0;
  ca->last_time = 0;
  ca->bic_origin_point = 0;
  ca->bic_K = 0;
  ca->delay_min = 0;
  ca->epoch_start = 0;
  ca->ack_cnt = 0;
  ca->tcp_cwnd = 0;
  ca->found = 0;
}

void bictcp_hystart_reset(struct sock *sk)
{
  struct tcp_sock *tp = tcp_sk(sk);
  struct bictcp *ca = inet_csk_ca(sk);

  ca->round_start = ca->last_ack = bictcp_clock();
  ca->end_seq = tp->snd_nxt;
  ca->curr_rtt = 0;
  ca->sample_cnt = 0;
}
```

### ssthresh
```c

void tcp_init_cwnd_reduction(struct sock *sk)
{
  tp->snd_ssthresh = inet_csk(sk)->icsk_ca_ops->ssthresh(sk);
}

void tcp_enter_loss(struct sock *sk)
{
  /* Reduce ssthresh if it has not yet been made inside this window. */
  if (icsk->icsk_ca_state <= TCP_CA_Disorder ||
      !after(tp->high_seq, tp->snd_una) ||
      (icsk->icsk_ca_state == TCP_CA_Loss && !icsk->icsk_retransmits))
  {
    tp->snd_ssthresh = icsk->icsk_ca_ops->ssthresh(sk);
    tcp_ca_event(sk, CA_EVENT_LOSS);
    tcp_init_undo(tp);
  }
}

u32 bictcp_recalc_ssthresh(struct sock *sk)
{
  const struct tcp_sock *tp = tcp_sk(sk);
  struct bictcp *ca = inet_csk_ca(sk);

  ca->epoch_start = 0;  /* end of epoch */

  /* Wmax and fast convergence */
  if (tp->snd_cwnd < ca->last_max_cwnd && fast_convergence)
    ca->last_max_cwnd = (tp->snd_cwnd * (BICTCP_BETA_SCALE + beta)) / (2 * BICTCP_BETA_SCALE);
  else
    ca->last_max_cwnd = tp->snd_cwnd;

  return max((tp->snd_cwnd * beta) / BICTCP_BETA_SCALE, 2U);
}
```

### cong_avoid
```c
void bictcp_cong_avoid(struct sock *sk, u32 ack, u32 acked)
{
  struct tcp_sock *tp = tcp_sk(sk);
  struct bictcp *ca = inet_csk_ca(sk);

  if (!tcp_is_cwnd_limited(sk))
    return;

  if (tcp_in_slow_start(tp)) {
    if (hystart && after(ack, ca->end_seq))
      bictcp_hystart_reset(sk);
    acked = tcp_slow_start(tp, acked);
    if (!acked)
      return;
  }
  bictcp_update(ca, tp->snd_cwnd, acked);
  tcp_cong_avoid_ai(tp, ca->cnt, acked);
}

bool tcp_is_cwnd_limited(const struct sock *sk)
{
  const struct tcp_sock *tp = tcp_sk(sk);

  /* If in slow start, ensure cwnd grows to twice what was ACKed. */
  if (tcp_in_slow_start(tp))
    return tp->snd_cwnd < 2 * tp->max_packets_out;

  return tp->is_cwnd_limited;
}

/* implement cubic algorithm */
void bictcp_update(struct bictcp *ca, u32 cwnd, u32 acked)
{
  u32 delta, bic_target, max_cnt;
  u64 offs, t;

  ca->ack_cnt += acked;  /* count the number of ACKed packets */

  if (ca->last_cwnd == cwnd && (s32)(tcp_jiffies32 - ca->last_time) <= HZ / 32)
    return;

  /* The CUBIC function can update ca->cnt at most once per jiffy.
   * On all cwnd reduction events, ca->epoch_start is set to 0,
   * which will force a recalculation of ca->cnt. */
  if (ca->epoch_start && tcp_jiffies32 == ca->last_time)
    goto tcp_friendliness;

  ca->last_cwnd = cwnd;
  ca->last_time = tcp_jiffies32;

  if (ca->epoch_start == 0) {
    ca->epoch_start = tcp_jiffies32;  /* record beginning */
    ca->ack_cnt = acked;      /* start counting */
    ca->tcp_cwnd = cwnd;      /* syn with cubic */

    if (ca->last_max_cwnd <= cwnd) {
      ca->bic_K = 0;
      ca->bic_origin_point = cwnd;
    } else {
      /* Compute new K based on
       * (wmax-cwnd) * (srtt>>3 / HZ) / c * 2^(3*bictcp_HZ) */
      ca->bic_K = cubic_root(cube_factor * (ca->last_max_cwnd - cwnd));
      ca->bic_origin_point = ca->last_max_cwnd;
    }
  }

  /* cubic function - calc*/
  /* calculate c * time^3 / rtt,
   *  while considering overflow in calculation of time^3
   * (so time^3 is done by using 64 bit)
   * and without the support of division of 64bit numbers
   * (so all divisions are done by using 32 bit)
   *  also NOTE the unit of those veriables
   *    time  = (t - K) / 2^bictcp_HZ
   *    c = bic_scale >> 10
   * rtt  = (srtt >> 3) / HZ
   * !!! The following code does not have overflow problems,
   * if the cwnd < 1 million packets !!! */

  t = (s32)(tcp_jiffies32 - ca->epoch_start);
  t += msecs_to_jiffies(ca->delay_min >> 3);
  /* change the unit from HZ to bictcp_HZ */
  t <<= BICTCP_HZ;
  do_div(t, HZ);

  if (t < ca->bic_K)    /* t - K */
    offs = ca->bic_K - t;
  else
    offs = t - ca->bic_K;

  /* c/rtt * (t-K)^3 */
  delta = (cube_rtt_scale * offs * offs * offs) >> (10+3*BICTCP_HZ);
  if (t < ca->bic_K)                            /* below origin*/
    bic_target = ca->bic_origin_point - delta;
  else                                          /* above origin*/
    bic_target = ca->bic_origin_point + delta;

  /* cubic function - calc bictcp_cnt*/
  if (bic_target > cwnd) {
    ca->cnt = cwnd / (bic_target - cwnd);
  } else {
    ca->cnt = 100 * cwnd;              /* very small increment*/
  }

  /* The initial growth of cubic function may be too conservative
   * when the available bandwidth is still unknown. */
  if (ca->last_max_cwnd == 0 && ca->cnt > 20)
    ca->cnt = 20;  /* increase cwnd 5% per RTT */

tcp_friendliness:
  /* TCP Friendly */
  if (tcp_friendliness) {
    u32 scale = beta_scale;

    delta = (cwnd * scale) >> 3;
    while (ca->ack_cnt > delta) {    /* update tcp cwnd */
      ca->ack_cnt -= delta;
      ca->tcp_cwnd++;
    }

    if (ca->tcp_cwnd > cwnd) {  /* if bic is slower than tcp */
      delta = ca->tcp_cwnd - cwnd;
      max_cnt = cwnd / delta;
      if (ca->cnt > max_cnt)
        ca->cnt = max_cnt;
    }
  }

  /* The maximum rate of cwnd increase CUBIC allows is 1 packet per
   * 2 packets ACKed, meaning cwnd grows at 1.5x per RTT. */
  ca->cnt = max(ca->cnt, 2U);
}

/* In theory this is tp->snd_cwnd += 1 / tp->snd_cwnd (or alternative w),
 * for every packet that was ACKed. */
void tcp_cong_avoid_ai(struct tcp_sock *tp, u32 w, u32 acked)
{
  /* If credits accumulated at a higher w, apply them gently now. */
  if (tp->snd_cwnd_cnt >= w) {
    tp->snd_cwnd_cnt = 0;
    tp->snd_cwnd++;
  }

  tp->snd_cwnd_cnt += acked;
  if (tp->snd_cwnd_cnt >= w) {
    u32 delta = tp->snd_cwnd_cnt / w;

    tp->snd_cwnd_cnt -= delta * w;
    tp->snd_cwnd += delta;
  }
  tp->snd_cwnd = min(tp->snd_cwnd, tp->snd_cwnd_clamp);
}
```

```c
sk->sk_prot->init(sk)
  tcp_init_sock(sk);
    tcp_assign_congestion_control(sk);

void tcp_assign_congestion_control(struct sock *sk)
{
  struct net *net = sock_net(sk);
  struct inet_connection_sock *icsk = inet_csk(sk);
  const struct tcp_congestion_ops *ca;

  rcu_read_lock();
  ca = rcu_dereference(net->ipv4.tcp_congestion_control);
  if (unlikely(!try_module_get(ca->owner)))
    ca = &tcp_reno;
  icsk->icsk_ca_ops = ca;
  rcu_read_unlock();

  memset(icsk->icsk_ca_priv, 0, sizeof(icsk->icsk_ca_priv));
  if (ca->flags & TCP_CONG_NEEDS_ECN)
    INET_ECN_xmit(sk);
  else
    INET_ECN_dontxmit(sk);
}


void tcp_reno_cong_avoid(struct sock *sk, u32 ack, u32 acked)
{
  struct tcp_sock *tp = tcp_sk(sk);

  if (!tcp_is_cwnd_limited(sk))
    return;

  /* In "safe" area, increase. */
  if (tcp_in_slow_start(tp)) {
    acked = tcp_slow_start(tp, acked);
    if (!acked)
      return;
  }
  /* In dangerous area, increase slowly. */
  tcp_cong_avoid_ai(tp, tp->snd_cwnd, acked);
}
```

```c
tcp_rcv_state_process();
  if (!inet_csk(sk)->icsk_ca_ops->cong_control)
    tcp_update_pacing_rate(sk);

void tcp_update_pacing_rate(struct sock *sk)
{
  const struct tcp_sock *tp = tcp_sk(sk);
  u64 rate;

  /* set sk_pacing_rate to 200 % of current rate (mss * cwnd / srtt) */
  rate = (u64)tp->mss_cache * ((USEC_PER_SEC / 100) << 3);

  /* current rate is (cwnd * mss) / srtt
   * In Slow Start [1], set sk_pacing_rate to 200 % the current rate.
   * In Congestion Avoidance phase, set it to 120 % the current rate.
   *
   * [1] : Normal Slow Start condition is (tp->snd_cwnd < tp->snd_ssthresh)
   *   If snd_cwnd >= (tp->snd_ssthresh / 2), we are approaching
   *   end of slow start and should slow down. */
  if (tp->snd_cwnd < tp->snd_ssthresh / 2)
    rate *= sock_net(sk)->ipv4.sysctl_tcp_pacing_ss_ratio;
  else
    rate *= sock_net(sk)->ipv4.sysctl_tcp_pacing_ca_ratio;

  rate *= max(tp->snd_cwnd, tp->packets_out);

  if (likely(tp->srtt_us))
    do_div(rate, tp->srtt_us);

  /* WRITE_ONCE() is needed because sch_fq fetches sk_pacing_rate
   * without any lock. We want to make sure compiler wont store
   * intermediate values in this location. */
  WRITE_ONCE(sk->sk_pacing_rate, min_t(u64, rate,
               sk->sk_max_pacing_rate));
}
```

### cwnd_event
```c
void tcp_data_queue(struct sock *sk, struct sk_buff *skb)
{
  if (skb->len)
    tcp_event_data_recv(sk, skb);
}

void tcp_rcv_established(struct sock *sk, struct sk_buff *skb)
{
  tcp_event_data_recv(sk, skb);
}

void tcp_event_data_recv(struct sock *sk, struct sk_buff *skb)
{
  tcp_ecn_check_ce(sk, skb);
}

void tcp_data_queue_ofo(struct sock *sk, struct sk_buff *skb)
{
  tcp_ecn_check_ce(sk, skb);
}

void tcp_ecn_check_ce(struct sock *sk, const struct sk_buff *skb)
{
  if (tcp_sk(sk)->ecn_flags & TCP_ECN_OK)
    __tcp_ecn_check_ce(sk, skb);
}

void __tcp_ecn_check_ce(struct sock *sk, const struct sk_buff *skb)
{
  struct tcp_sock *tp = tcp_sk(sk);

  switch (TCP_SKB_CB(skb)->ip_dsfield & INET_ECN_MASK) {
  case INET_ECN_NOT_ECT:
    /* Funny extension: if ECT is not set on a segment,
     * and we already seen ECT on a previous segment,
     * it is probably a retransmit. */
    if (tp->ecn_flags & TCP_ECN_SEEN)
      tcp_enter_quickack_mode(sk, 2);
    break;

  case INET_ECN_CE:
    if (tcp_ca_needs_ecn(sk))
      tcp_ca_event(sk, CA_EVENT_ECN_IS_CE);

    if (!(tp->ecn_flags & TCP_ECN_DEMAND_CWR)) {
      /* Better not delay acks, sender can have a very low cwnd */
      tcp_enter_quickack_mode(sk, 2);
      tp->ecn_flags |= TCP_ECN_DEMAND_CWR;
    }
    tp->ecn_flags |= TCP_ECN_SEEN;
    break;

  default:
    if (tcp_ca_needs_ecn(sk))
      tcp_ca_event(sk, CA_EVENT_ECN_NO_CE);
    tp->ecn_flags |= TCP_ECN_SEEN;
    break;
  }
}

void tcp_ca_event(struct sock *sk, const enum tcp_ca_event event)
{
  const struct inet_connection_sock *icsk = inet_csk(sk);

  if (icsk->icsk_ca_ops->cwnd_event)
    icsk->icsk_ca_ops->cwnd_event(sk, event);
}

void bictcp_cwnd_event(struct sock *sk, enum tcp_ca_event event)
{
  if (event == CA_EVENT_TX_START) {
    struct bictcp *ca = inet_csk_ca(sk);
    u32 now = tcp_jiffies32;
    s32 delta;

    delta = now - tcp_sk(sk)->lsndtime;

    /* We were application limited (idle) for a while.
     * Shift epoch_start to keep cwnd growth to cubic curve. */
    if (ca->epoch_start && delta > 0) {
      ca->epoch_start += delta;
      if (after(ca->epoch_start, now))
        ca->epoch_start = now;
    }
    return;
  }
}
```

### pkts_acked
```c
int tcp_ack(struct sock *sk, const struct sk_buff *skb, int flag)
{
    tcp_clean_rtx_queue(sk, prior_fack, prior_snd_una, &sack_state);
}
int tcp_clean_rtx_queue(struct sock *sk, u32 prior_fack,
    u32 prior_snd_una,
    struct tcp_sacktag_state *sack)
{
    icsk->icsk_ca_ops->pkts_acked(sk, &sample);
}

/* Track delayed acknowledgment ratio using sliding window
 * ratio = (15*ratio + sample) / 16 */
void bictcp_acked(struct sock *sk, const struct ack_sample *sample)
{
  const struct tcp_sock *tp = tcp_sk(sk);
  struct bictcp *ca = inet_csk_ca(sk);
  u32 delay;

  /* Some calls are for duplicates without timetamps */
  if (sample->rtt_us < 0)
    return;

  /* Discard delay samples right after fast recovery */
  if (ca->epoch_start && (s32)(tcp_jiffies32 - ca->epoch_start) < HZ)
    return;

  delay = (sample->rtt_us << 3) / USEC_PER_MSEC;
  if (delay == 0)
    delay = 1;

  /* first time call or link delay decreases */
  if (ca->delay_min == 0 || ca->delay_min > delay)
    ca->delay_min = delay;

  /* hystart triggers when cwnd is larger than some threshold */
  if (hystart && tcp_in_slow_start(tp) && tp->snd_cwnd >= hystart_low_window)
    hystart_update(sk, delay);
}

void hystart_update(struct sock *sk, u32 delay)
{
  struct tcp_sock *tp = tcp_sk(sk);
  struct bictcp *ca = inet_csk_ca(sk);

  if (ca->found & hystart_detect)
    return;

  if (hystart_detect & HYSTART_ACK_TRAIN) {
    u32 now = bictcp_clock();

    /* first detection parameter - ack-train detection */
    if ((s32)(now - ca->last_ack) <= hystart_ack_delta) {
      ca->last_ack = now;
      if ((s32)(now - ca->round_start) > ca->delay_min >> 4) {
        ca->found |= HYSTART_ACK_TRAIN;
        tp->snd_ssthresh = tp->snd_cwnd;
      }
    }
  }

  if (hystart_detect & HYSTART_DELAY) {
    /* obtain the minimum delay of more than sampling packets */
    if (ca->curr_rtt > delay)
      ca->curr_rtt = delay;
    if (ca->sample_cnt < HYSTART_MIN_SAMPLES) {
      if (ca->curr_rtt == 0 || ca->curr_rtt > delay)
        ca->curr_rtt = delay;

      ca->sample_cnt++;
    } else {
      if (ca->curr_rtt > ca->delay_min + HYSTART_DELAY_THRESH(ca->delay_min >> 3)) {
        ca->found |= HYSTART_DELAY;
        tp->snd_ssthresh = tp->snd_cwnd;
      }
    }
  }
}
```

### hystart
```c
void bictcp_hystart_reset(struct sock *sk)
{
  struct tcp_sock *tp = tcp_sk(sk);
  struct bictcp *ca = inet_csk_ca(sk);

  ca->round_start = ca->last_ack = bictcp_clock();
  ca->end_seq = tp->snd_nxt;
  ca->curr_rtt = 0;
  ca->sample_cnt = 0;
}
```

### set_state
```c
void tcp_set_ca_state(struct sock *sk, const u8 ca_state)
{
  struct inet_connection_sock *icsk = inet_csk(sk);

  if (icsk->icsk_ca_ops->set_state)
    icsk->icsk_ca_ops->set_state(sk, ca_state);
  icsk->icsk_ca_state = ca_state;
}

void bictcp_state(struct sock *sk, u8 new_state)
{
  if (new_state == TCP_CA_Loss) {
    bictcp_reset(inet_csk_ca(sk));
    bictcp_hystart_reset(sk);
  }
}
```

## tcp_ack
```c
/* check the ACK field */
int tcp_ack(struct sock *sk, const struct sk_buff *skb, int flag)
{
  struct inet_connection_sock *icsk = inet_csk(sk);
  struct tcp_sock *tp = tcp_sk(sk);
  struct tcp_sacktag_state sack_state;
  struct rate_sample rs = { .prior_delivered = 0 };
  u32 prior_snd_una = tp->snd_una;
  bool is_sack_reneg = tp->is_sack_reneg;
  u32 ack_seq = TCP_SKB_CB(skb)->seq;
  u32 ack = TCP_SKB_CB(skb)->ack_seq;
  bool is_dupack = false;
  int prior_packets = tp->packets_out;
  u32 delivered = tp->delivered;
  u32 lost = tp->lost;
  int rexmit = REXMIT_NONE; /* Flag to (re)transmit to recover losses */
  u32 prior_fack;

  sack_state.first_sackt = 0;
  sack_state.rate = &rs;

  /* We very likely will need to access rtx queue. */
  prefetch(sk->tcp_rtx_queue.rb_node);

  /* If the ack is older than previous acks
   * then we can probably ignore it.  */
  if (before(ack, prior_snd_una)) {
    /* RFC 5961 5.2 [Blind Data Injection Attack].[Mitigation] */
    if (before(ack, prior_snd_una - tp->max_window)) {
      if (!(flag & FLAG_NO_CHALLENGE_ACK))
        tcp_send_challenge_ack(sk, skb);
      return -1;
    }
    goto old_ack;
  }

  /* If the ack includes data we haven't sent yet, discard
   * this segment (RFC793 Section 3.9). */
  if (after(ack, tp->snd_nxt))
    goto invalid_ack;

  if (after(ack, prior_snd_una)) {
    flag |= FLAG_SND_UNA_ADVANCED;
    icsk->icsk_retransmits = 0;

#if IS_ENABLED(CONFIG_TLS_DEVICE)
    if (static_branch_unlikely(&clean_acked_data_enabled))
      if (icsk->icsk_clean_acked)
        icsk->icsk_clean_acked(sk, ack);
#endif
  }

  prior_fack = tcp_is_sack(tp) ? tcp_highest_sack_seq(tp) : tp->snd_una;
  rs.prior_in_flight = tcp_packets_in_flight(tp);

  /* ts_recent update must be made after we are sure that the packet
   * is in window. */
  if (flag & FLAG_UPDATE_TS_RECENT)
    tcp_replace_ts_recent(tp, TCP_SKB_CB(skb)->seq);

  if (!(flag & FLAG_SLOWPATH) && after(ack, prior_snd_una)) {
    /* Window is constant, pure forward advance.
     * No more checks are required.
     * Note, we use the fact that SND.UNA>=SND.WL2. */
    tcp_update_wl(tp, ack_seq);
    tcp_snd_una_update(tp, ack);
    flag |= FLAG_WIN_UPDATE;

    tcp_in_ack_event(sk, CA_ACK_WIN_UPDATE);

  } else {
    u32 ack_ev_flags = CA_ACK_SLOWPATH;

    if (ack_seq != TCP_SKB_CB(skb)->end_seq)
      flag |= FLAG_DATA;

    flag |= tcp_ack_update_window(sk, skb, ack, ack_seq);

    if (TCP_SKB_CB(skb)->sacked)
      flag |= tcp_sacktag_write_queue(sk, skb, prior_snd_una, &sack_state);

    if (tcp_ecn_rcv_ecn_echo(tp, tcp_hdr(skb))) {
      flag |= FLAG_ECE;
      ack_ev_flags |= CA_ACK_ECE;
    }

    if (flag & FLAG_WIN_UPDATE)
      ack_ev_flags |= CA_ACK_WIN_UPDATE;

    tcp_in_ack_event(sk, ack_ev_flags);
  }

  /* This is a deviation from RFC3168 since it states that:
   * "When the TCP data sender is ready to set the CWR bit after reducing
   * the congestion window, it SHOULD set the CWR bit only on the first
   * new data packet that it transmits."
   * We accept CWR on pure ACKs to be more robust
   * with widely-deployed TCP implementations that do this. */
  tcp_ecn_accept_cwr(sk, skb);

  /* We passed data and got it acked, remove any soft error
   * log. Something worked... */
  sk->sk_err_soft = 0;
  icsk->icsk_probes_out = 0;
  tp->rcv_tstamp = tcp_jiffies32;
  if (!prior_packets)
    goto no_queue;

  /* See if we can take anything off of the retransmit queue. */
  flag |= tcp_clean_rtx_queue(sk, prior_fack, prior_snd_una, &sack_state);

  tcp_rack_update_reo_wnd(sk, &rs);

  if (tp->tlp_high_seq)
    tcp_process_tlp_ack(sk, ack, flag);
  /* If needed, reset TLP/RTO timer; RACK may later override this. */
  if (flag & FLAG_SET_XMIT_TIMER)
    tcp_set_xmit_timer(sk);

  if (tcp_ack_is_dubious(sk, flag)) {
    is_dupack = !(flag & (FLAG_SND_UNA_ADVANCED | FLAG_NOT_DUP));
    tcp_fastretrans_alert(sk, prior_snd_una, is_dupack, &flag, &rexmit);
  }

  if ((flag & FLAG_FORWARD_PROGRESS) || !(flag & FLAG_NOT_DUP))
    sk_dst_confirm(sk);

  delivered = tcp_newly_delivered(sk, delivered, flag);
  lost = tp->lost - lost;      /* freshly marked lost */
  rs.is_ack_delayed = !!(flag & FLAG_ACK_MAYBE_DELAYED);
  tcp_rate_gen(sk, delivered, lost, is_sack_reneg, sack_state.rate);
  tcp_cong_control(sk, ack, delivered, flag, sack_state.rate);
  tcp_xmit_recovery(sk, rexmit);
  return 1;

no_queue:
  /* If data was DSACKed, see if we can undo a cwnd reduction. */
  if (flag & FLAG_DSACKING_ACK) {
    tcp_fastretrans_alert(sk, prior_snd_una, is_dupack, &flag, &rexmit);
    tcp_newly_delivered(sk, delivered, flag);
  }
  /* If this ack opens up a zero window, clear backoff.  It was
   * being used to time the probes, and is probably far higher than
   * it needs to be for normal retransmission. */
  tcp_ack_probe(sk);

  if (tp->tlp_high_seq)
    tcp_process_tlp_ack(sk, ack, flag);
  return 1;

invalid_ack:
  SOCK_DEBUG(sk, "Ack %u after %u:%u\n", ack, tp->snd_una, tp->snd_nxt);
  return -1;

old_ack:
  /* If data was SACKed, tag it and see if we should send more data.
   * If data was DSACKed, see if we can undo a cwnd reduction. */
  if (TCP_SKB_CB(skb)->sacked) {
    flag |= tcp_sacktag_write_queue(sk, skb, prior_snd_una, &sack_state);
    tcp_fastretrans_alert(sk, prior_snd_una, is_dupack, &flag, &rexmit);
    tcp_newly_delivered(sk, delivered, flag);
    tcp_xmit_recovery(sk, rexmit);
  }

  SOCK_DEBUG(sk, "Ack %u before %u:%u\n", ack, tp->snd_una, tp->snd_nxt);
  return 0;
}

int tcp_clean_rtx_queue(struct sock *sk, u32 prior_fack,
  u32 prior_snd_una,
  struct tcp_sacktag_state *sack)
{
  const struct inet_connection_sock *icsk = inet_csk(sk);
  u64 first_ackt, last_ackt;
  struct tcp_sock *tp = tcp_sk(sk);
  u32 prior_sacked = tp->sacked_out;
  u32 reord = tp->snd_nxt; /* lowest acked un-retx un-sacked seq */
  struct sk_buff *skb, *next;
  bool fully_acked = true;
  long sack_rtt_us = -1L;
  long seq_rtt_us = -1L;
  long ca_rtt_us = -1L;
  u32 pkts_acked = 0;
  u32 last_in_flight = 0;
  bool rtt_update;
  int flag = 0;

  first_ackt = 0;

  for (skb = skb_rb_first(&sk->tcp_rtx_queue); skb; skb = next) {
    struct tcp_skb_cb *scb = TCP_SKB_CB(skb);
    const u32 start_seq = scb->seq;
    u8 sacked = scb->sacked;
    u32 acked_pcount;

    tcp_ack_tstamp(sk, skb, prior_snd_una);

    /* Determine how many packets and what bytes were acked, tso and else */
    if (after(scb->end_seq, tp->snd_una)) {
      if (tcp_skb_pcount(skb) == 1 || !after(tp->snd_una, scb->seq))
        break;

      acked_pcount = tcp_tso_acked(sk, skb);
      if (!acked_pcount)
        break;
      fully_acked = false;
    } else {
      acked_pcount = tcp_skb_pcount(skb);
    }

    if (unlikely(sacked & TCPCB_RETRANS)) {
      if (sacked & TCPCB_SACKED_RETRANS)
        tp->retrans_out -= acked_pcount;
      flag |= FLAG_RETRANS_DATA_ACKED;
    } else if (!(sacked & TCPCB_SACKED_ACKED)) {
      last_ackt = skb->skb_mstamp;
      WARN_ON_ONCE(last_ackt == 0);
      if (!first_ackt)
        first_ackt = last_ackt;

      last_in_flight = TCP_SKB_CB(skb)->tx.in_flight;
      if (before(start_seq, reord))
        reord = start_seq;
      if (!after(scb->end_seq, tp->high_seq))
        flag |= FLAG_ORIG_SACK_ACKED;
    }

    if (sacked & TCPCB_SACKED_ACKED) {
      tp->sacked_out -= acked_pcount;
    } else if (tcp_is_sack(tp)) {
      tp->delivered += acked_pcount;
      if (!tcp_skb_spurious_retrans(tp, skb))
        tcp_rack_advance(tp, sacked, scb->end_seq, skb->skb_mstamp);
    }
    if (sacked & TCPCB_LOST)
      tp->lost_out -= acked_pcount;

    tp->packets_out -= acked_pcount;
    pkts_acked += acked_pcount;
    tcp_rate_skb_delivered(sk, skb, sack->rate);

    /* Initial outgoing SYN's get put onto the write_queue
     * just like anything else we transmit.  It is not
     * true data, and if we misinform our callers that
     * this ACK acks real data, we will erroneously exit
     * connection startup slow start one packet too
     * quickly.  This is severely frowned upon behavior. */
    if (likely(!(scb->tcp_flags & TCPHDR_SYN))) {
      flag |= FLAG_DATA_ACKED;
    } else {
      flag |= FLAG_SYN_ACKED;
      tp->retrans_stamp = 0;
    }

    if (!fully_acked)
      break;

    next = skb_rb_next(skb);
    if (unlikely(skb == tp->retransmit_skb_hint))
      tp->retransmit_skb_hint = NULL;
    if (unlikely(skb == tp->lost_skb_hint))
      tp->lost_skb_hint = NULL;
    tcp_highest_sack_replace(sk, skb, next);
    tcp_rtx_queue_unlink_and_free(skb, sk);
  }

  if (!skb)
    tcp_chrono_stop(sk, TCP_CHRONO_BUSY);

  if (likely(between(tp->snd_up, prior_snd_una, tp->snd_una)))
    tp->snd_up = tp->snd_una;

  if (skb && (TCP_SKB_CB(skb)->sacked & TCPCB_SACKED_ACKED))
    flag |= FLAG_SACK_RENEGING;

  if (likely(first_ackt) && !(flag & FLAG_RETRANS_DATA_ACKED)) {
    seq_rtt_us = tcp_stamp_us_delta(tp->tcp_mstamp, first_ackt);
    ca_rtt_us = tcp_stamp_us_delta(tp->tcp_mstamp, last_ackt);

    if (pkts_acked == 1 && last_in_flight < tp->mss_cache &&
        last_in_flight && !prior_sacked && fully_acked &&
        sack->rate->prior_delivered + 1 == tp->delivered &&
        !(flag & (FLAG_CA_ALERT | FLAG_SYN_ACKED)))
    {
      /* Conservatively mark a delayed ACK. It's typically
       * from a lone runt packet over the round trip to
       * a receiver w/o out-of-order or CE events. */
      flag |= FLAG_ACK_MAYBE_DELAYED;
    }
  }
  if (sack->first_sackt) {
    sack_rtt_us = tcp_stamp_us_delta(tp->tcp_mstamp, sack->first_sackt);
    ca_rtt_us = tcp_stamp_us_delta(tp->tcp_mstamp, sack->last_sackt);
  }
  rtt_update = tcp_ack_update_rtt(sk, flag, seq_rtt_us, sack_rtt_us, ca_rtt_us, sack->rate);

  if (flag & FLAG_ACKED) {
    flag |= FLAG_SET_XMIT_TIMER;  /* set TLP or RTO timer */
    if (unlikely(icsk->icsk_mtup.probe_size && !after(tp->mtu_probe.probe_seq_end, tp->snd_una))) {
      tcp_mtup_probe_success(sk);
    }

    if (tcp_is_reno(tp)) {
      tcp_remove_reno_sacks(sk, pkts_acked);

      /* If any of the cumulatively ACKed segments was
       * retransmitted, non-SACK case cannot confirm that
       * progress was due to original transmission due to
       * lack of TCPCB_SACKED_ACKED bits even if some of
       * the packets may have been never retransmitted. */
      if (flag & FLAG_RETRANS_DATA_ACKED)
        flag &= ~FLAG_ORIG_SACK_ACKED;
    } else {
      int delta;

      /* Non-retransmitted hole got filled? That's reordering */
      if (before(reord, prior_fack))
        tcp_check_sack_reordering(sk, reord, 0);

      delta = prior_sacked - tp->sacked_out;
      tp->lost_cnt_hint -= min(tp->lost_cnt_hint, delta);
    }
  } else if (skb && rtt_update && sack_rtt_us >= 0 && sack_rtt_us > tcp_stamp_us_delta(tp->tcp_mstamp, skb->skb_mstamp)) {
    /* Do not re-arm RTO if the sack RTT is measured from data sent
     * after when the head was last (re)transmitted. Otherwise the
     * timeout may continue to extend in loss recovery. */
    flag |= FLAG_SET_XMIT_TIMER;  /* set TLP or RTO timer */
  }

  if (icsk->icsk_ca_ops->pkts_acked) {
    struct ack_sample sample = { .pkts_acked = pkts_acked,
               .rtt_us = sack->rate->rtt_us,
               .in_flight = last_in_flight };

    icsk->icsk_ca_ops->pkts_acked(sk, &sample);
  }

  return flag;
}
```

## tcp_fastretrans_alert
```c
/* Process an event, which can update packets-in-flight not trivially.
 * Main goal of this function is to calculate new estimate for left_out,
 * taking into account both packets sitting in receiver's buffer and
 * packets lost by network.
 *
 * Besides that it updates the congestion state when packet loss or ECN
 * is detected. But it does not reduce the cwnd, it is done by the
 * congestion control later.
 *
 * It does _not_ decide what to send, it is made in function
 * tcp_xmit_retransmit_queue(). */

int tcp_ack(struct sock *sk, const struct sk_buff *skb, int flag)
{
  if (tcp_ack_is_dubious(sk, flag)) {
    is_dupack = !(flag & (FLAG_SND_UNA_ADVANCED | FLAG_NOT_DUP));
    tcp_fastretrans_alert(sk, prior_snd_una, is_dupack, &flag, &rexmit);
  }

  if (flag & FLAG_DSACKING_ACK) {
    tcp_fastretrans_alert(sk, prior_snd_una, is_dupack, &flag, &rexmit);
    tcp_newly_delivered(sk, delivered, flag);
  }

  if (TCP_SKB_CB(skb)->sacked) {
    flag |= tcp_sacktag_write_queue(sk, skb, prior_snd_una, &sack_state);
    tcp_fastretrans_alert(sk, prior_snd_una, is_dupack, &flag, &rexmit);
    tcp_newly_delivered(sk, delivered, flag);
    tcp_xmit_recovery(sk, rexmit);
  }
}

void tcp_fastretrans_alert(struct sock *sk, const u32 prior_snd_una,
          bool is_dupack, int *ack_flag, int *rexmit)
{
  struct inet_connection_sock *icsk = inet_csk(sk);
  struct tcp_sock *tp = tcp_sk(sk);
  int fast_rexmit = 0, flag = *ack_flag;
  bool do_lost = is_dupack || ((flag & FLAG_DATA_SACKED) && tcp_force_fast_retransmit(sk));

  if (!tp->packets_out && tp->sacked_out)
    tp->sacked_out = 0;

  /* Now state machine starts.
   * A. ECE, hence prohibit cwnd undoing, the reduction is required. */
  if (flag & FLAG_ECE)
    tp->prior_ssthresh = 0;

  /* B. In all the states check for reneging SACKs. */
  if (tcp_check_sack_reneging(sk, flag))
    return;

  /* C. Check consistency of the current state. */
  tcp_verify_left_out(tp);

  /* D. Check state exit conditions. State can be terminated
   *    when high_seq is ACKed. */
  if (icsk->icsk_ca_state == TCP_CA_Open) {
    WARN_ON(tp->retrans_out != 0);
    tp->retrans_stamp = 0;
  } else if (!before(tp->snd_una, tp->high_seq)) {
    switch (icsk->icsk_ca_state) {
    case TCP_CA_CWR:
      /* CWR is to be held something *above* high_seq
       * is ACKed for CWR bit to reach receiver. */
      if (tp->snd_una != tp->high_seq) {
        tcp_end_cwnd_reduction(sk);
        tcp_set_ca_state(sk, TCP_CA_Open);
      }
      break;

    case TCP_CA_Recovery:
      if (tcp_is_reno(tp))
        tcp_reset_reno_sack(tp);
      if (tcp_try_undo_recovery(sk))
        return;
      tcp_end_cwnd_reduction(sk);
      break;
    }
  }

  /* E. Process state. */
  switch (icsk->icsk_ca_state) {
  case TCP_CA_Recovery:
    if (!(flag & FLAG_SND_UNA_ADVANCED)) {
      if (tcp_is_reno(tp) && is_dupack)
        tcp_add_reno_sack(sk);
    } else {
      if (tcp_try_undo_partial(sk, prior_snd_una))
        return;
      /* Partial ACK arrived. Force fast retransmit. */
      do_lost = tcp_is_reno(tp) || tcp_force_fast_retransmit(sk);
    }
    if (tcp_try_undo_dsack(sk)) {
      tcp_try_keep_open(sk);
      return;
    }
    tcp_identify_packet_loss(sk, ack_flag);
    break;

  case TCP_CA_Loss:
    tcp_process_loss(sk, flag, is_dupack, rexmit);
    tcp_identify_packet_loss(sk, ack_flag);
    if (!(icsk->icsk_ca_state == TCP_CA_Open || (*ack_flag & FLAG_LOST_RETRANS)))
      return;

    /* Change state if cwnd is undone or retransmits are lost */
    /* fall through */
  default:
    if (tcp_is_reno(tp)) {
      if (flag & FLAG_SND_UNA_ADVANCED)
        tcp_reset_reno_sack(tp);
      if (is_dupack)
        tcp_add_reno_sack(sk);
    }

    if (icsk->icsk_ca_state <= TCP_CA_Disorder)
      tcp_try_undo_dsack(sk);

    tcp_identify_packet_loss(sk, ack_flag);
    if (!tcp_time_to_recover(sk, flag)) { /* tp->sacked_out + 1 > tp->reordering */
      tcp_try_to_open(sk, flag);
      return;
    }

    /* MTU probe failure: don't reduce cwnd */
    if (icsk->icsk_ca_state < TCP_CA_CWR &&
        icsk->icsk_mtup.probe_size &&
        tp->snd_una == tp->mtu_probe.probe_seq_start)
    {
      tcp_mtup_probe_failed(sk);
      /* Restores the reduction we did in tcp_mtup_probe() */
      tp->snd_cwnd++;
      tcp_simple_retransmit(sk);
      return;
    }

    /* Otherwise enter Recovery state */
    tcp_enter_recovery(sk, (flag & FLAG_ECE));
    fast_rexmit = 1;
  }

  if (!tcp_is_rack(sk) && do_lost)
    tcp_update_scoreboard(sk, fast_rexmit);

  *rexmit = REXMIT_LOST;
}

void tcp_end_cwnd_reduction(struct sock *sk)
{
  struct tcp_sock *tp = tcp_sk(sk);

  if (inet_csk(sk)->icsk_ca_ops->cong_control)
    return;

  /* Reset cwnd to ssthresh in CWR or Recovery (unless it's undone) */
  if (tp->snd_ssthresh < TCP_INFINITE_SSTHRESH && (inet_csk(sk)->icsk_ca_state == TCP_CA_CWR || tp->undo_marker)) {
    tp->snd_cwnd = tp->snd_ssthresh;
    tp->snd_cwnd_stamp = tcp_jiffies32;
  }
  tcp_ca_event(sk, CA_EVENT_COMPLETE_CWR);
}

bool tcp_try_undo_recovery(struct sock *sk)
{
  struct tcp_sock *tp = tcp_sk(sk);

  if (tcp_may_undo(tp)) {
    int mib_idx;

    /* Happy end! We did not retransmit anything
     * or our original transmission succeeded. */
    DBGUNDO(sk, inet_csk(sk)->icsk_ca_state == TCP_CA_Loss ? "loss" : "retrans");
    tcp_undo_cwnd_reduction(sk, false);
    if (inet_csk(sk)->icsk_ca_state == TCP_CA_Loss)
      mib_idx = LINUX_MIB_TCPLOSSUNDO;
    else
      mib_idx = LINUX_MIB_TCPFULLUNDO;

    NET_INC_STATS(sock_net(sk), mib_idx);
  } else if (tp->rack.reo_wnd_persist) {
    tp->rack.reo_wnd_persist--;
  }
  if (tp->snd_una == tp->high_seq && tcp_is_reno(tp)) {
    /* Hold old state until something *above* high_seq
     * is ACKed. For Reno it is MUST to prevent false
     * fast retransmits (RFC2582). SACK TCP is safe. */
    if (!tcp_any_retrans_done(sk))
      tp->retrans_stamp = 0;
    return true;
  }
  tcp_set_ca_state(sk, TCP_CA_Open);
  tp->is_sack_reneg = 0;
  return false;
}

void tcp_update_scoreboard(struct sock *sk, int fast_rexmit)
{
  struct tcp_sock *tp = tcp_sk(sk);

  if (tcp_is_sack(tp)) {
    int sacked_upto = tp->sacked_out - tp->reordering;
    if (sacked_upto >= 0)
      tcp_mark_head_lost(sk, sacked_upto, 0);
    else if (fast_rexmit)
      tcp_mark_head_lost(sk, 1, 1);
  }
}

/* Detect loss in event "A" above by marking head of queue up as lost.
 * For non-SACK(Reno) senders, the first "packets" number of segments
 * are considered lost. For RFC3517 SACK, a segment is considered lost if it
 * has at least tp->reordering SACKed seqments above it; "packets" refers to
 * the maximum SACKed segments to pass before reaching this limit. */
void tcp_mark_head_lost(struct sock *sk, int packets, int mark_head)
{
  struct tcp_sock *tp = tcp_sk(sk);
  struct sk_buff *skb;
  int cnt, oldcnt, lost;
  unsigned int mss;
  /* Use SACK to deduce losses of new sequences sent during recovery */
  const u32 loss_high = tcp_is_sack(tp) ?  tp->snd_nxt : tp->high_seq;

  WARN_ON(packets > tp->packets_out);
  skb = tp->lost_skb_hint;
  if (skb) {
    /* Head already handled? */
    if (mark_head && after(TCP_SKB_CB(skb)->seq, tp->snd_una))
      return;
    cnt = tp->lost_cnt_hint;
  } else {
    skb = tcp_rtx_queue_head(sk);
    cnt = 0;
  }

  skb_rbtree_walk_from(skb) {
    /* TODO: do this better */
    /* this is not the most efficient way to do this... */
    tp->lost_skb_hint = skb;
    tp->lost_cnt_hint = cnt;

    if (after(TCP_SKB_CB(skb)->end_seq, loss_high))
      break;

    oldcnt = cnt;
    if (tcp_is_reno(tp) || (TCP_SKB_CB(skb)->sacked & TCPCB_SACKED_ACKED))
      cnt += tcp_skb_pcount(skb);

    if (cnt > packets) {
      if (tcp_is_sack(tp) || (TCP_SKB_CB(skb)->sacked & TCPCB_SACKED_ACKED) || (oldcnt >= packets))
        break;

      mss = tcp_skb_mss(skb);
      /* If needed, chop off the prefix to mark as lost. */
      lost = (packets - oldcnt) * mss;
      if (lost < skb->len &&
          tcp_fragment(sk, TCP_FRAG_IN_RTX_QUEUE, skb, lost, mss, GFP_ATOMIC) < 0)
        break;
      cnt = packets;
    }

    tcp_skb_mark_lost(tp, skb);

    if (mark_head)
      break;
  }

  tcp_verify_left_out(tp);
}

void tcp_skb_mark_lost(struct tcp_sock *tp, struct sk_buff *skb)
{
  if (!(TCP_SKB_CB(skb)->sacked & (TCPCB_LOST|TCPCB_SACKED_ACKED))) {
    tcp_verify_retransmit_hint(tp, skb);

    tp->lost_out += tcp_skb_pcount(skb);
    tcp_sum_lost(tp, skb);
    TCP_SKB_CB(skb)->sacked |= TCPCB_LOST;
  }
}
```

## tcp_cong_control
```c
int tcp_ack(struct sock *sk, const struct sk_buff *skb, int flag)
{
  tcp_cong_control(sk, ack, delivered, flag, sack_state.rate);
}

void tcp_cong_control(struct sock *sk, u32 ack, u32 acked_sacked,
           int flag, const struct rate_sample *rs)
{
  const struct inet_connection_sock *icsk = inet_csk(sk);

  if (icsk->icsk_ca_ops->cong_control) {
    icsk->icsk_ca_ops->cong_control(sk, rs);
    return;
  }

  /* return (TCPF_CA_CWR | TCPF_CA_Recovery) & (1 << inet_csk(sk)->icsk_ca_state) */
  if (tcp_in_cwnd_reduction(sk)) {
    /* Reduce cwnd if state mandates */
    tcp_cwnd_reduction(sk, acked_sacked, flag);
  } else if (tcp_may_raise_cwnd(sk, flag)) {
    /* Advance cwnd if state allows */
    tcp_cong_avoid(sk, ack, acked_sacked);
  }
  tcp_update_pacing_rate(sk);
}

void tcp_cwnd_reduction(struct sock *sk, int newly_acked_sacked, int flag)
{
  struct tcp_sock *tp = tcp_sk(sk);
  int sndcnt = 0;
  int delta = tp->snd_ssthresh - tcp_packets_in_flight(tp);

  if (newly_acked_sacked <= 0 || WARN_ON_ONCE(!tp->prior_cwnd))
    return;

  tp->prr_delivered += newly_acked_sacked;
  if (delta < 0) {
    u64 dividend = (u64)tp->snd_ssthresh * tp->prr_delivered + tp->prior_cwnd - 1;
    sndcnt = div_u64(dividend, tp->prior_cwnd) - tp->prr_out;
  } else if ((flag & FLAG_RETRANS_DATA_ACKED) && !(flag & FLAG_LOST_RETRANS)) {
    sndcnt = min_t(int, delta,
             max_t(int, tp->prr_delivered - tp->prr_out,
             newly_acked_sacked) + 1);
  } else {
    sndcnt = min(delta, newly_acked_sacked);
  }
  /* Force a fast retransmit upon entering fast recovery */
  sndcnt = max(sndcnt, (tp->prr_out ? 0 : 1));
  tp->snd_cwnd = tcp_packets_in_flight(tp) + sndcnt;
}

/* Decide wheather to run the increase function of congestion control. */
bool tcp_may_raise_cwnd(const struct sock *sk, const int flag)
{
  /* If reordering is high then always grow cwnd whenever data is
   * delivered regardless of its ordering. Otherwise stay conservative
   * and only grow cwnd on in-order delivery (RFC5681). A stretched ACK w/
   * new SACK or ECE mark may first advance cwnd here and later reduce
   * cwnd in tcp_fastretrans_alert() based on more states. */
  if (tcp_sk(sk)->reordering > sock_net(sk)->ipv4.sysctl_tcp_reordering)
    return flag & FLAG_FORWARD_PROGRESS;

  return flag & FLAG_DATA_ACKED;
}
```

## tcp_slow_start
```c
tcp_cong_avoid();
    bictcp_cong_avoid();
        tcp_slow_start();
```

```c
bool tcp_in_slow_start(const struct tcp_sock *tp)
{
  return tp->snd_cwnd < tp->snd_ssthresh;
}

u32 tcp_slow_start(struct tcp_sock *tp, u32 acked)
{
  u32 cwnd = min(tp->snd_cwnd + acked, tp->snd_ssthresh);

  acked -= cwnd - tp->snd_cwnd;
  tp->snd_cwnd = min(cwnd, tp->snd_cwnd_clamp);

  /* if cwnd < ssthresh return 0 */
  return acked;
}
```

## tcp_cong_avoid

```c
void tcp_cong_avoid(struct sock *sk, u32 ack, u32 acked)
{
  const struct inet_connection_sock *icsk = inet_csk(sk);

  icsk->icsk_ca_ops->cong_avoid(sk, ack, acked); /* bictcp_cong_avoid */
  tcp_sk(sk)->snd_cwnd_stamp = tcp_jiffies32;
}
```

[:point_right: bictcp_cong_avoid](#cong_avoid)

## tcp_xmit_recovery
```c
/* tcp_cong_control has updated the cwnd already. So if we're in
 * loss recovery then now we do any new sends (for FRTO) or
 * retransmits (for CA_Loss or CA_recovery) that make sense. */
void tcp_xmit_recovery(struct sock *sk, int rexmit)
{
  struct tcp_sock *tp = tcp_sk(sk);

  if (rexmit == REXMIT_NONE)
    return;

  if (unlikely(rexmit == 2)) {
    __tcp_push_pending_frames(sk, tcp_current_mss(sk), TCP_NAGLE_OFF);
    if (after(tp->snd_nxt, tp->high_seq))
      return;
    tp->frto = 0;
  }
  tcp_xmit_retransmit_queue(sk);
}

/* This gets called after a retransmit timeout, and the initially
 * retransmitted data is acknowledged.  It tries to continue
 * resending the rest of the retransmit queue, until either
 * we've sent it all or the congestion window limit is reached. */
void tcp_xmit_retransmit_queue(struct sock *sk)
{
  const struct inet_connection_sock *icsk = inet_csk(sk);
  struct sk_buff *skb, *rtx_head, *hole = NULL;
  struct tcp_sock *tp = tcp_sk(sk);
  u32 max_segs;
  int mib_idx;

  if (!tp->packets_out)
    return;

  rtx_head = tcp_rtx_queue_head(sk);
  skb = tp->retransmit_skb_hint ?: rtx_head;
  max_segs = tcp_tso_segs(sk, tcp_current_mss(sk));

  skb_rbtree_walk_from(skb) {
    __u8 sacked;
    int segs;

    if (tcp_pacing_check(sk))
      break;

    /* we could do better than to assign each time */
    if (!hole)
      tp->retransmit_skb_hint = skb;

    segs = tp->snd_cwnd - tcp_packets_in_flight(tp);
    if (segs <= 0)
      return;
    sacked = TCP_SKB_CB(skb)->sacked;
    /* In case tcp_shift_skb_data() have aggregated large skbs,
     * we need to make sure not sending too bigs TSO packets */
    segs = min_t(int, segs, max_segs);

    if (tp->retrans_out >= tp->lost_out) {
      break;
    } else if (!(sacked & TCPCB_LOST)) {
      if (!hole && !(sacked & (TCPCB_SACKED_RETRANS|TCPCB_SACKED_ACKED)))
        hole = skb;
      continue;

    } else {
      if (icsk->icsk_ca_state != TCP_CA_Loss)
        mib_idx = LINUX_MIB_TCPFASTRETRANS;
      else
        mib_idx = LINUX_MIB_TCPSLOWSTARTRETRANS;
    }

    if (sacked & (TCPCB_SACKED_ACKED|TCPCB_SACKED_RETRANS))
      continue;

    if (tcp_small_queue_check(sk, skb, 1))
      return;

    if (tcp_retransmit_skb(sk, skb, segs))
      return;

    NET_ADD_STATS(sock_net(sk), mib_idx, tcp_skb_pcount(skb));

    if (tcp_in_cwnd_reduction(sk))
      tp->prr_out += tcp_skb_pcount(skb);

    if (skb == rtx_head && icsk->icsk_pending != ICSK_TIME_REO_TIMEOUT)
      inet_csk_reset_xmit_timer(sk, ICSK_TIME_RETRANS, inet_csk(sk)->icsk_rto, TCP_RTO_MAX);
  }
}
```

## tcp_cwnd_test
```c
/* Can at least one segment of SKB be sent right now, according to the
 * congestion window rules?  If so, return how many segments are allowed. */
unsigned int tcp_cwnd_test(const struct tcp_sock *tp,
           const struct sk_buff *skb)
{
  u32 in_flight, cwnd, halfcwnd;

  /* Don't be strict about the congestion window for the final FIN.  */
  if ((TCP_SKB_CB(skb)->tcp_flags & TCPHDR_FIN) && tcp_skb_pcount(skb) == 1)
    return 1;

  in_flight = tcp_packets_in_flight(tp);
  cwnd = tp->snd_cwnd;
  if (in_flight >= cwnd)
    return 0;

  /* For better scheduling, ensure we have at least
   * 2 GSO packets in flight. */
  halfcwnd = max(cwnd >> 1, 1U);
  return min(halfcwnd, cwnd - in_flight);
}

/* Read this equation as:
 *
 *  "Packets sent once on transmission queue" MINUS
 *  "Packets left network, but not honestly ACKed yet" PLUS
 *  "Packets fast retransmitted" */
unsigned int tcp_packets_in_flight(const struct tcp_sock *tp)
{
  return tp->packets_out - tcp_left_out(tp) + tp->retrans_out;
}
```

## tcp_cwnd_validate
```c
void tcp_cwnd_validate(struct sock *sk, bool is_cwnd_limited)
{
  const struct tcp_congestion_ops *ca_ops = inet_csk(sk)->icsk_ca_ops;
  struct tcp_sock *tp = tcp_sk(sk);

  /* Track the maximum number of outstanding packets in each
   * window, and remember whether we were cwnd-limited then. */
  if (!before(tp->snd_una, tp->max_packets_seq) ||
      tp->packets_out > tp->max_packets_out ||
      is_cwnd_limited)
  {
    tp->max_packets_out = tp->packets_out;
    tp->max_packets_seq = tp->snd_nxt;
    tp->is_cwnd_limited = is_cwnd_limited;
  }

  if (tcp_is_cwnd_limited(sk)) {
    /* Network is feed fully. */
    tp->snd_cwnd_used = 0;
    tp->snd_cwnd_stamp = tcp_jiffies32;
  } else {
    /* Network starves. */
    if (tp->packets_out > tp->snd_cwnd_used)
      tp->snd_cwnd_used = tp->packets_out;

    if (sock_net(sk)->ipv4.sysctl_tcp_slow_start_after_idle &&
        (s32)(tcp_jiffies32 - tp->snd_cwnd_stamp) >= inet_csk(sk)->icsk_rto &&
        !ca_ops->cong_control)
      tcp_cwnd_application_limited(sk);

    /* The following conditions together indicate the starvation
     * is caused by insufficient sender buffer:
     * 1) just sent some data (see tcp_write_xmit)
     * 2) not cwnd limited (this else condition)
     * 3) no more data to send (tcp_write_queue_empty())
     * 4) application is hitting buffer limit (SOCK_NOSPACE) */
    if (tcp_write_queue_empty(sk) && sk->sk_socket &&
        test_bit(SOCK_NOSPACE, &sk->sk_socket->flags) &&
        (1 << sk->sk_state) & (TCPF_ESTABLISHED | TCPF_CLOSE_WAIT))
      tcp_chrono_start(sk, TCP_CHRONO_SNDBUF_LIMITED);
  }
}
```

```c
tcp_ack();
    int rexmit = REXMIT_NONE;

    tcp_ack_update_window();
        tp->bytes_acked += ack - tp->snd_una;
        tp->snd_una = ack;

    tcp_ecn_accept_cwr();
    tcp_clean_rtx_queue();
      icsk_ca_ops->pkts_acked();
        bictcp_acked();
            if (hystart && tcp_in_slow_start(tp) && tp->snd_cwnd >= hystart_low_window)
                hystart_update(sk, delay);

    if (tcp_ack_is_dubious(sk, flag))
/* updates the congestion state when packet loss or ECN is detected */
        tcp_fastretrans_alert(&rexmit);
            if (!before(tp->snd_una, tp->high_seq)) {
                switch (icsk->icsk_ca_state) {
                    case TCP_CA_CWR: {
                        tcp_end_cwnd_reduction(sk);
                        tcp_set_ca_state(sk, TCP_CA_Open);
                        break;
                    }
                    case TCP_CA_Recovery: {
                        tcp_try_undo_recovery(sk)
                            tcp_set_ca_state(sk, TCP_CA_Open);
                        tcp_end_cwnd_reduction(sk);
                        break;
                    }
                }
            }

            switch (icsk->icsk_ca_state) {
                case TCP_CA_Recovery:

                case TCP_CA_Loss:
                    tcp_process_loss(sk, flag, is_dupack, rexmit);

                default:
                    if (icsk->icsk_ca_state <= TCP_CA_Disorder)
                        tcp_try_undo_dsack(sk);
                    if (!tcp_time_to_recover(sk, flag)) {/* tp->sacked_out + 1 > tp->reordering */
                        tcp_try_to_open(sk, flag);
                            if (flag & FLAG_ECE)
                                tcp_enter_cwr(sk);
                                    tcp_set_ca_state(sk, TCP_CA_CWR);
                            else
                                tcp_try_keep_open();
                                    tcp_set_ca_state(sk, TCP_CA_Open | TCP_CA_Disorder);
                                    tp->high_seq = tp->snd_nxt;

                        return;
                    }

                    tcp_enter_recovery(sk, (flag & FLAG_ECE));
                        tcp_set_ca_state(sk, TCP_CA_Recovery);
                        tcp_init_cwnd_reduction();
                            snd_ssthresh = max((tp->snd_cwnd * beta) / BICTCP_BETA_SCALE, 2U);
                }

            *rexmit = REXMIT_LOST;

            if (do_lost)
                tcp_update_scoreboard();
                    tcp_mark_head_lost();
                        tcp_skb_mark_lost();
                            tp->lost_out += tcp_skb_pcount(skb);
                            TCP_SKB_CB(skb)->sacked |= TCPCB_LOST;

/* reduce the cwnd */
    tcp_cong_control();
        if (tcp_in_cwnd_reduction(sk)) {
            tcp_cwnd_reduction(sk, acked_sacked, flag);
                tp->snd_cwnd = tcp_packets_in_flight(tp) + sndcnt;
        } else if (tcp_may_raise_cwnd(sk, flag)) {
            tcp_cong_avoid(sk, ack, acked_sacked);
                icsk_ca_ops->cong_avoid();
                    bictcp_cong_avoid();
                        if (tcp_in_slow_start(tp)) {
                            acked = tcp_slow_start(tp, acked);
                                tp->snd_cwnd = min(cwnd, tp->snd_cwnd_clamp);
                            if (!acked) /* 0 if snd_cwnd < snd_ssthresh */
                            return;
                        }

                        bictcp_update(ca, tp->snd_cwnd, acked);
                            /* calculate ca->cnt */

                        /* tp->snd_cwnd += 1 / tp->snd_cwnd (or alternative w), for every packet that was ACKed. */
                        tcp_cong_avoid_ai(tp, ca->cnt, acked);
                            tp->snd_cwnd = min(tp->snd_cwnd, tp->snd_cwnd_clamp);
        }


/* retransmit */
    tcp_xmit_recovery(rexmit);
```

## Refer
* [RFC 8312 - CUBIC for Fast Long-Distance Networks](https://datatracker.ietf.org/doc/html/rfc8312)
* [TCP’s Congestion Control Implementation in Linux Kernel.pdf](https://wiki.aalto.fi/download/attachments/69901948/TCP-CongestionControlFinal.pdf)
* [Cubic with Faster Convergence: An Improved Cubic Fast Convergence Mechanism](https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=&ved=2ahUKEwiwgpvBjOP1AhWUgMYKHVfUAEIQFnoECBcQAw&url=https%3A%2F%2Fwww.atlantis-press.com%2Farticle%2F4642.pdf&usg=AOvVaw1hU80iGNUiyn7QUU43T2rq)


# tcp_timer

[深入探讨TCP中使用的定时器](https://mp.weixin.qq.com/s/VqkffGmZjHu9jMnx-0nAzw)

| Timer | Purpose | Typical Duration | Linux Function | Struct.Member
| :-: | :-: | :-: | :-: | :-: |
| Connect Req | Timeout SYN_RECV requests | RTT-based, backoff | reqsk_timer_handler | request_sock.rsk_timer
| Retransmission | Retransmit lost segments | RTT-based (adaptive) | tcp_write_timer->tcp_retransmit_timer | inet_connection_sock.icsk_retransmit_timer
| Delayed ACK | Batch ACKs | ~40ms (configurable) | tcp_delack_timer | inet_connection_sock.icsk_delack_timer
| Keepalive | Detect dead connections | 2 hours, then 75s intervals | tcp_keepalive_timer | sock.sk_timer
| TIME_WAIT | Handle late packets | 60s (2 * MSL) | tw_timer_handler | inet_timewait_sock.tw_timer
| Zero Window Probe | Check if window opens | Exponential backoff | tcp_probe_timer | inet_connection_sock.icsk_retransmit_timer
| Persist | Overlap with zero window probe | Exponential backoff | tcp_probe_timer | inet_connection_sock.icsk_retransmit_timer
| Fast Retransmit | Quick recovery (implicit) | N/A (duplicate ACK-driven) | tcp_fastretrans_alert |

## timer_lifecycle
### timer_init
```c
/* sk->sk_prot->init(sk) */
int tcp_v4_init_sock(struct sock *sk)
{
  struct inet_connection_sock *icsk = inet_csk(sk);

  tcp_init_sock(sk);

  icsk->icsk_af_ops = &ipv4_specific;

  tcp_sk(sk)->af_specific = &tcp_sock_ipv4_specific;

  return 0;
}

void tcp_init_sock(struct sock *sk)
{
  struct inet_connection_sock *icsk = inet_csk(sk);
  struct tcp_sock *tp = tcp_sk(sk);

  tp->out_of_order_queue = RB_ROOT;
  sk->tcp_rtx_queue = RB_ROOT;
  tcp_init_xmit_timers(sk);
  INIT_LIST_HEAD(&tp->tsq_node);
  INIT_LIST_HEAD(&tp->tsorted_sent_queue);

  icsk->icsk_rto = TCP_TIMEOUT_INIT;
  tp->mdev_us = jiffies_to_usecs(TCP_TIMEOUT_INIT);
  minmax_reset(&tp->rtt_min, tcp_jiffies32, ~0U);

  /* So many TCP implementations out there (incorrectly) count the
   * initial SYN frame in their delayed-ACK and congestion control
   * algorithms that we must have the following bandaid to talk
   * efficiently to them.  -DaveM */
  tp->snd_cwnd = TCP_INIT_CWND;

  /* There's a bubble in the pipe until at least the first ACK. */
  tp->app_limited = ~0U;

  /* See draft-stevens-tcpca-spec-01 for discussion of the
   * initialization of these values. */
  tp->snd_ssthresh = TCP_INFINITE_SSTHRESH;
  tp->snd_cwnd_clamp = ~0;
  tp->mss_cache = TCP_MSS_DEFAULT;

  tp->reordering = sock_net(sk)->ipv4.sysctl_tcp_reordering;
  tcp_assign_congestion_control(sk);
    /* ---> */

  tp->tsoffset = 0;
  tp->rack.reo_wnd_steps = 1;

  sk->sk_state = TCP_CLOSE;

  sk->sk_write_space = sk_stream_write_space;
  sock_set_flag(sk, SOCK_USE_WRITE_QUEUE);

  icsk->icsk_sync_mss = tcp_sync_mss;

  sk->sk_sndbuf = sock_net(sk)->ipv4.sysctl_tcp_wmem[1];
  sk->sk_rcvbuf = sock_net(sk)->ipv4.sysctl_tcp_rmem[1];

  sk_sockets_allocated_inc(sk);
  sk->sk_route_forced_caps = NETIF_F_GSO;
}

void tcp_init_xmit_timers(struct sock *sk)
{
  inet_csk_init_xmit_timers(sk, &tcp_write_timer, &tcp_delack_timer, &tcp_keepalive_timer);
  hrtimer_init(&tcp_sk(sk)->pacing_timer, CLOCK_MONOTONIC, HRTIMER_MODE_ABS_PINNED_SOFT);
  tcp_sk(sk)->pacing_timer.function = tcp_pace_kick;

  hrtimer_init(&tcp_sk(sk)->compressed_ack_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL_PINNED_SOFT);
  tcp_sk(sk)->compressed_ack_timer.function = tcp_compressed_ack_kick;
}

void inet_csk_init_xmit_timers(struct sock *sk,
  void (*retransmit_handler)(struct timer_list *t),
  void (*delack_handler)(struct timer_list *t),
  void (*keepalive_handler)(struct timer_list *t))
{
  struct inet_connection_sock *icsk = inet_csk(sk);

  timer_setup(&icsk->icsk_retransmit_timer, retransmit_handler, 0);
  timer_setup(&icsk->icsk_delack_timer, delack_handler, 0);
  timer_setup(&sk->sk_timer, keepalive_handler, 0);
  icsk->icsk_pending = icsk->icsk_ack.pending = 0;
}
```

### timer_reset
```c
/*
1. tcp_event_new_data_sent
2. case TCP_SYN_RECV
2. case TCP_FIN_WAIT1
4. tcp_rcv_fastopen_synack */
void tcp_rearm_rto(struct sock *sk)
{
  const struct inet_connection_sock *icsk = inet_csk(sk);
  struct tcp_sock *tp = tcp_sk(sk);

  if (tp->fastopen_rsk)
    return;

  if (!tp->packets_out) {
    inet_csk_clear_xmit_timer(sk, ICSK_TIME_RETRANS);
  } else {
    u32 rto = inet_csk(sk)->icsk_rto;
    /* Offset the time elapsed after installing regular RTO */
    if (icsk->icsk_pending == ICSK_TIME_REO_TIMEOUT ||
        icsk->icsk_pending == ICSK_TIME_LOSS_PROBE)
    {
      s64 delta_us = tcp_rto_delta_us(sk);
      /* delta_us may not be positive if the socket is locked
       * when the retrans timer fires and is rescheduled. */
      rto = usecs_to_jiffies(max_t(int, delta_us, 1));
    }
    inet_csk_reset_xmit_timer(sk, ICSK_TIME_RETRANS, rto, TCP_RTO_MAX);
  }
}

void inet_csk_reset_xmit_timer(struct sock *sk, const int what,
               unsigned long when,
               const unsigned long max_when)
{
  struct inet_connection_sock *icsk = inet_csk(sk);

  if (when > max_when) {
    pr_debug("reset_xmit_timer: sk=%p %d when=0x%lx, caller=%p\n",
       sk, what, when, (void *)_THIS_IP_);
    when = max_when;
  }

  if (what == ICSK_TIME_RETRANS || what == ICSK_TIME_PROBE0 ||
      what == ICSK_TIME_EARLY_RETRANS || what == ICSK_TIME_LOSS_PROBE ||
      what == ICSK_TIME_REO_TIMEOUT)
  {
    icsk->icsk_pending = what;
    icsk->icsk_timeout = jiffies + when;
    sk_reset_timer(sk, &icsk->icsk_retransmit_timer, icsk->icsk_timeout);
  } else if (what == ICSK_TIME_DACK) {
    icsk->icsk_ack.pending |= ICSK_ACK_TIMER;
    icsk->icsk_ack.timeout = jiffies + when;
    sk_reset_timer(sk, &icsk->icsk_delack_timer, icsk->icsk_ack.timeout);
  } else {
    pr_debug("inet_csk BUG: unknown timer value\n");
  }
}

void sk_reset_timer(struct sock *sk, struct timer_list* timer,
        unsigned long expires)
{
  if (!mod_timer(timer, expires))
    sock_hold(sk);
}
```

### timer_clear
```c
void inet_csk_clear_xmit_timer(struct sock *sk, const int what)
{
  struct inet_connection_sock *icsk = inet_csk(sk);

  if (what == ICSK_TIME_RETRANS || what == ICSK_TIME_PROBE0) {
    icsk->icsk_pending = 0;
    sk_stop_timer(sk, &icsk->icsk_retransmit_timer);
  } else if (what == ICSK_TIME_DACK) {
    icsk->icsk_ack.blocked = icsk->icsk_ack.pending = 0;
    sk_stop_timer(sk, &icsk->icsk_delack_timer);
  } else {
    pr_debug("inet_csk BUG: unknown timer value\n");
  }
}

void inet_csk_clear_xmit_timers(struct sock *sk)
{
  struct inet_connection_sock *icsk = inet_csk(sk);

  icsk->icsk_pending = icsk->icsk_ack.pending = icsk->icsk_ack.blocked = 0;

  sk_stop_timer(sk, &icsk->icsk_retransmit_timer);
  sk_stop_timer(sk, &icsk->icsk_delack_timer);
  sk_stop_timer(sk, &sk->sk_timer);
}

void sk_stop_timer(struct sock *sk, struct timer_list* timer)
{
  if (del_timer(timer))
    __sock_put(sk);
}
```

## tcp_write_timer
```c
void tcp_write_timer(struct timer_list *t)
{
  struct inet_connection_sock *icsk = from_timer(icsk, t, icsk_retransmit_timer);
  struct sock *sk = &icsk->icsk_inet.sk;

  bh_lock_sock(sk);
  if (!sock_owned_by_user(sk)) {
    tcp_write_timer_handler(sk);
  } else {
    /* delegate our work to tcp_release_cb() */
    if (!test_and_set_bit(TCP_WRITE_TIMER_DEFERRED, &sk->sk_tsq_flags))
      sock_hold(sk);
  }
  bh_unlock_sock(sk);
  sock_put(sk);
}

void tcp_write_timer_handler(struct sock *sk)
{
    struct inet_connection_sock *icsk = inet_csk(sk);
    int event;

    if (((1 << sk->sk_state) & (TCPF_CLOSE | TCPF_LISTEN)) || !icsk->icsk_pending)
        goto out;

    if (time_after(icsk->icsk_timeout, jiffies)) {
        sk_reset_timer(sk, &icsk->icsk_retransmit_timer, icsk->icsk_timeout);
        goto out;
    }

    tcp_mstamp_refresh(tcp_sk(sk));
    event = icsk->icsk_pending;

    switch (event) {
    case ICSK_TIME_REO_TIMEOUT:
        tcp_rack_reo_timeout(sk);
        break;
    case ICSK_TIME_LOSS_PROBE:
        tcp_send_loss_probe(sk);
        break;
    case ICSK_TIME_RETRANS:
        icsk->icsk_pending = 0;
        tcp_retransmit_timer(sk);
        break;
    case ICSK_TIME_PROBE0:
        icsk->icsk_pending = 0;
        tcp_probe_timer(sk);
        break;
    }

out:
  sk_mem_reclaim(sk);
}

void tcp_retransmit_timer(struct sock *sk)
{
  struct tcp_sock *tp = tcp_sk(sk);
  struct net *net = sock_net(sk);
  struct inet_connection_sock *icsk = inet_csk(sk);

  if (tp->fastopen_rsk) {
    tcp_fastopen_synack_timer(sk);
    /* Before we receive ACK to our SYN-ACK don't retransmit
     * anything else (e.g., data or FIN segments). */
    return;
  }

  if (!tp->packets_out || WARN_ON_ONCE(tcp_rtx_queue_empty(sk)))
    return;

  tp->tlp_high_seq = 0;

  if (!tp->snd_wnd && !sock_flag(sk, SOCK_DEAD) && !((1 << sk->sk_state) & (TCPF_SYN_SENT | TCPF_SYN_RECV))) {
    /* Receiver dastardly shrinks window. Our retransmits
     * become zero probes, but we should not timeout this
     * connection. If the socket is an orphan, time it out,
     * we cannot allow such beasts to hang infinitely. */
    struct inet_sock *inet = inet_sk(sk);

    if (tcp_jiffies32 - tp->rcv_tstamp > TCP_RTO_MAX) {
      tcp_write_err(sk);
      goto out;
    }
    tcp_enter_loss(sk);
    tcp_retransmit_skb(sk, tcp_rtx_queue_head(sk), 1);
    __sk_dst_reset(sk);
    goto out_reset_timer;
  }

  /* if too many retries, abandon the retransmit */
  if (tcp_write_timeout(sk))
    goto out;

  if (icsk->icsk_retransmits == 0) {
    int mib_idx = 0;

    if (icsk->icsk_ca_state == TCP_CA_Recovery) {
      if (tcp_is_sack(tp))
        mib_idx = LINUX_MIB_TCPSACKRECOVERYFAIL;
      else
        mib_idx = LINUX_MIB_TCPRENORECOVERYFAIL;
    } else if (icsk->icsk_ca_state == TCP_CA_Loss) {
      mib_idx = LINUX_MIB_TCPLOSSFAILURES;
    } else if ((icsk->icsk_ca_state == TCP_CA_Disorder) ||
         tp->sacked_out) {
      if (tcp_is_sack(tp))
        mib_idx = LINUX_MIB_TCPSACKFAILURES;
      else
        mib_idx = LINUX_MIB_TCPRENOFAILURES;
    }
  }

  tcp_enter_loss(sk);

  if (tcp_retransmit_skb(sk, tcp_rtx_queue_head(sk), 1) > 0) {
    /* Retransmission failed because of local congestion,
     * do not backoff. */
    if (!icsk->icsk_retransmits)
      icsk->icsk_retransmits = 1;
    inet_csk_reset_xmit_timer(sk, ICSK_TIME_RETRANS,
            min(icsk->icsk_rto, TCP_RESOURCE_PROBE_INTERVAL),
            TCP_RTO_MAX);
    goto out;
  }

  /* Increase the timeout each time we retransmit.  Note that
   * we do not increase the rtt estimate.  rto is initialized
   * from rtt, but increases here.  Jacobson (SIGCOMM 88) suggests
   * that doubling rto each time is the least we can get away with.
   * In KA9Q, Karn uses this for the first few times, and then
   * goes to quadratic.  netBSD doubles, but only goes up to *64,
   * and clamps at 1 to 64 sec afterwards.  Note that 120 sec is
   * defined in the protocol as the maximum possible RTT.  I guess
   * we'll have to use something other than TCP to talk to the
   * University of Mars.
   *
   * PAWS allows us longer timeouts and large windows, so once
   * implemented ftp to mars will work nicely. We will have to fix
   * the 120 second clamps though! */
  icsk->icsk_backoff++;
  icsk->icsk_retransmits++;

out_reset_timer:
  /* If stream is thin, use linear timeouts. Since 'icsk_backoff' is
   * used to reset timer, set to 0. Recalculate 'icsk_rto' as this
   * might be increased if the stream oscillates between thin and thick,
   * thus the old value might already be too high compared to the value
   * set by 'tcp_set_rto' in tcp_input.c which resets the rto without
   * backoff. Limit to TCP_THIN_LINEAR_RETRIES before initiating
   * exponential backoff behaviour to avoid continue hammering
   * linear-timeout retransmissions into a black hole */
  if (sk->sk_state == TCP_ESTABLISHED &&
      (tp->thin_lto || net->ipv4.sysctl_tcp_thin_linear_timeouts) &&
      tcp_stream_is_thin(tp) &&
      icsk->icsk_retransmits <= TCP_THIN_LINEAR_RETRIES)
  {
    icsk->icsk_backoff = 0;
    icsk->icsk_rto = min(__tcp_set_rto(tp), TCP_RTO_MAX);
  } else {
    /* Use normal (exponential) backoff */
    icsk->icsk_rto = min(icsk->icsk_rto << 1, TCP_RTO_MAX);
  }
  inet_csk_reset_xmit_timer(sk, ICSK_TIME_RETRANS,
          tcp_clamp_rto_to_user_timeout(sk), TCP_RTO_MAX);
  if (retransmits_timed_out(sk, net->ipv4.sysctl_tcp_retries1 + 1, 0))
    __sk_dst_reset(sk);

out:;
}

int tcp_write_timeout(struct sock *sk)
{
  struct inet_connection_sock *icsk = inet_csk(sk);
  struct tcp_sock *tp = tcp_sk(sk);
  struct net *net = sock_net(sk);
  bool expired = false, do_reset;
  int retry_until;

  if ((1 << sk->sk_state) & (TCPF_SYN_SENT | TCPF_SYN_RECV)) {
    if (icsk->icsk_retransmits) {
      dst_negative_advice(sk);
    } else {
      sk_rethink_txhash(sk);
    }
    retry_until = icsk->icsk_syn_retries ? : net->ipv4.sysctl_tcp_syn_retries;
    expired = icsk->icsk_retransmits >= retry_until;
  } else {
    if (retransmits_timed_out(sk, net->ipv4.sysctl_tcp_retries1, 0)) {
      /* Black hole detection */
      tcp_mtu_probing(icsk, sk);

      dst_negative_advice(sk);
    } else {
      sk_rethink_txhash(sk);
    }

    retry_until = net->ipv4.sysctl_tcp_retries2;
    if (sock_flag(sk, SOCK_DEAD)) {
      const bool alive = icsk->icsk_rto < TCP_RTO_MAX;

      retry_until = tcp_orphan_retries(sk, alive);
      do_reset = alive || !retransmits_timed_out(sk, retry_until, 0);

      if (tcp_out_of_resources(sk, do_reset))
        return 1;
    }
  }

  if (!expired)
    expired = retransmits_timed_out(sk, retry_until, icsk->icsk_user_timeout);

  tcp_fastopen_active_detect_blackhole(sk, expired);

  if (BPF_SOCK_OPS_TEST_FLAG(tp, BPF_SOCK_OPS_RTO_CB_FLAG))
    tcp_call_bpf_3arg(sk, BPF_SOCK_OPS_RTO_CB, icsk->icsk_retransmits, icsk->icsk_rto, (int)expired);

  if (expired) {
    /* Has it gone just too far? */
    tcp_write_err(sk);
    return 1;
  }

  return 0;
}
```

### tcp_retransmit_skb
```c
int tcp_retransmit_skb(struct sock *sk, struct sk_buff *skb, int segs)
{
  struct tcp_sock *tp = tcp_sk(sk);
  int err = __tcp_retransmit_skb(sk, skb, segs);

  if (err == 0) {
    TCP_SKB_CB(skb)->sacked |= TCPCB_RETRANS;
    tp->retrans_out += tcp_skb_pcount(skb);

    /* Save stamp of the first retransmit. */
    if (!tp->retrans_stamp)
      tp->retrans_stamp = tcp_skb_timestamp(skb);

  }

  if (tp->undo_retrans < 0)
    tp->undo_retrans = 0;
  tp->undo_retrans += tcp_skb_pcount(skb);
  return err;
}

int __tcp_retransmit_skb(struct sock *sk, struct sk_buff *skb, int segs)
{
  struct inet_connection_sock *icsk = inet_csk(sk);
  struct tcp_sock *tp = tcp_sk(sk);
  unsigned int cur_mss;
  int diff, len, err;


  /* Inconclusive MTU probe */
  if (icsk->icsk_mtup.probe_size)
    icsk->icsk_mtup.probe_size = 0;

  /* Do not sent more than we queued. 1/4 is reserved for possible
   * copying overhead: fragmentation, tunneling, mangling etc. */
  if (refcount_read(&sk->sk_wmem_alloc) > min_t(u32, sk->sk_wmem_queued + (sk->sk_wmem_queued >> 2), sk->sk_sndbuf))
    return -EAGAIN;

  if (skb_still_in_host_queue(sk, skb))
    return -EBUSY;

  if (before(TCP_SKB_CB(skb)->seq, tp->snd_una)) {
    if (unlikely(before(TCP_SKB_CB(skb)->end_seq, tp->snd_una))) {
      WARN_ON_ONCE(1);
      return -EINVAL;
    }
    if (tcp_trim_head(sk, skb, tp->snd_una - TCP_SKB_CB(skb)->seq))
      return -ENOMEM;
  }

  if (inet_csk(sk)->icsk_af_ops->rebuild_header(sk))
    return -EHOSTUNREACH; /* Routing failure or similar. */

  cur_mss = tcp_current_mss(sk);

  /* If receiver has shrunk his window, and skb is out of
   * new window, do not retransmit it. The exception is the
   * case, when window is shrunk to zero. In this case
   * our retransmit serves as a zero window probe. */
  if (!before(TCP_SKB_CB(skb)->seq, tcp_wnd_end(tp)) && TCP_SKB_CB(skb)->seq != tp->snd_una)
    return -EAGAIN;

  len = cur_mss * segs;
  if (skb->len > len) {
    if (tcp_fragment(sk, TCP_FRAG_IN_RTX_QUEUE, skb, len, cur_mss, GFP_ATOMIC))
      return -ENOMEM; /* We'll try again later. */
  } else {
    if (skb_unclone(skb, GFP_ATOMIC))
      return -ENOMEM;

    diff = tcp_skb_pcount(skb);
    tcp_set_skb_tso_segs(skb, cur_mss);
    diff -= tcp_skb_pcount(skb);
    if (diff)
      tcp_adjust_pcount(sk, skb, diff);
    if (skb->len < cur_mss)
      tcp_retrans_try_collapse(sk, skb, cur_mss);
  }

  /* RFC3168, section 6.1.1.1. ECN (Explicit Congestion Notification) fallback */
  if ((TCP_SKB_CB(skb)->tcp_flags & TCPHDR_SYN_ECN) == TCPHDR_SYN_ECN)
    tcp_ecn_clear_syn(sk, skb);

  /* Update global and local TCP statistics. */
  segs = tcp_skb_pcount(skb);
  TCP_ADD_STATS(sock_net(sk), TCP_MIB_RETRANSSEGS, segs);
  tp->total_retrans += segs;
  tp->bytes_retrans += skb->len;

  /* make sure skb->data is aligned on arches that require it
   * and check if ack-trimming & collapsing extended the headroom
   * beyond what csum_start can cover. */
  if (unlikely((NET_IP_ALIGN && ((unsigned long)skb->data & 3)) || skb_headroom(skb) >= 0xFFFF)) {
    struct sk_buff *nskb;

    tcp_skb_tsorted_save(skb) {
      nskb = __pskb_copy(skb, MAX_TCP_HEADER, GFP_ATOMIC);
      err = nskb ? tcp_transmit_skb(sk, nskb, 0, GFP_ATOMIC) : -ENOBUFS;
    } tcp_skb_tsorted_restore(skb);

    if (!err) {
      tcp_update_skb_after_send(tp, skb);
      tcp_rate_skb_sent(sk, skb);
    }
  } else {
    err = tcp_transmit_skb(sk, skb, 1, GFP_ATOMIC);
  }

  if (BPF_SOCK_OPS_TEST_FLAG(tp, BPF_SOCK_OPS_RETRANS_CB_FLAG))
    tcp_call_bpf_3arg(sk, BPF_SOCK_OPS_RETRANS_CB,
          TCP_SKB_CB(skb)->seq, segs, err);

  if (likely(!err)) {
    TCP_SKB_CB(skb)->sacked |= TCPCB_EVER_RETRANS;
    trace_tcp_retransmit_skb(sk, skb);
  } else if (err != -EBUSY) {
    NET_ADD_STATS(sock_net(sk), LINUX_MIB_TCPRETRANSFAIL, segs);
  }
  return err;
}
```

## tcp_delack_timer
```c
void tcp_delack_timer(struct timer_list *t)
{
  struct inet_connection_sock *icsk = from_timer(icsk, t, icsk_delack_timer);
  struct sock *sk = &icsk->icsk_inet.sk;

  bh_lock_sock(sk);
  if (!sock_owned_by_user(sk)) {
    tcp_delack_timer_handler(sk);
  } else {
    icsk->icsk_ack.blocked = 1;
    /* deleguate our work to tcp_release_cb() */
    if (!test_and_set_bit(TCP_DELACK_TIMER_DEFERRED, &sk->sk_tsq_flags))
      sock_hold(sk);
  }
  bh_unlock_sock(sk);
  sock_put(sk);
}

void tcp_delack_timer_handler(struct sock *sk)
{
  struct inet_connection_sock *icsk = inet_csk(sk);

  sk_mem_reclaim_partial(sk);

  if (((1 << sk->sk_state) & (TCPF_CLOSE | TCPF_LISTEN)) || !(icsk->icsk_ack.pending & ICSK_ACK_TIMER))
    goto out;

  if (time_after(icsk->icsk_ack.timeout, jiffies)) {
    sk_reset_timer(sk, &icsk->icsk_delack_timer, icsk->icsk_ack.timeout);
    goto out;
  }
  icsk->icsk_ack.pending &= ~ICSK_ACK_TIMER;

  if (inet_csk_ack_scheduled(sk)) {
    if (!icsk->icsk_ack.pingpong) {
      /* Delayed ACK missed: inflate ATO. */
      icsk->icsk_ack.ato = min(icsk->icsk_ack.ato << 1, icsk->icsk_rto);
    } else {
      /* Delayed ACK missed: leave pingpong mode and
       * deflate ATO. */
      icsk->icsk_ack.pingpong = 0;
      icsk->icsk_ack.ato      = TCP_ATO_MIN;
    }
    tcp_mstamp_refresh(tcp_sk(sk));
    tcp_send_ack(sk);
  }

out:
  if (tcp_under_memory_pressure(sk))
    sk_mem_reclaim(sk);
}

int inet_csk_ack_scheduled(const struct sock *sk)
{
  return inet_csk(sk)->icsk_ack.pending & ICSK_ACK_SCHED;
}
```

## tcp_keepalive_timer
```c
```

## reqsk_timer_handler
```c
```

## fast retransmit

# tcpdump
## register_prot_hook
```c
/* strace tcpdump -i eth0 */
/* pcap_can_set_rfmon_linux */
socket(PF_PACKET, SOCK_RAW, htons(ETH_P_ALL))

struct net_proto_family *net_families[NPROTO];
/* net/ipv4/af_inet.c */
const struct net_proto_family packet_family_ops = {
  .family =  PF_PACKET,
  .create =  packet_create,
  .owner  =  THIS_MODULE,
};

const struct proto_ops packet_ops = {
  .family =     PF_PACKET,
  .bind =       packet_bind,
  .connect =    sock_no_connect,
  .accept =     sock_no_accept,
  .poll =       packet_poll,
  .ioctl =      packet_ioctl,
  .listen =     sock_no_listen,
  .shutdown =   sock_no_shutdown,
  .setsockopt = packet_setsockopt,
  .getsockopt = packet_getsockopt,
  .sendmsg =    packet_sendmsg,
  .recvmsg =    packet_recvmsg,
  .mmap =       packet_mmap,
  .sendpage =   sock_no_sendpage,
};

struct packet_sock {
  /* struct sock has to be the first member of packet_sock */
  struct sock                sk;
  struct packet_fanout       *fanout;

  struct packet_ring_buffer   rx_ring;
  struct packet_ring_buffer   tx_ring;

  struct completion           skb_completion;
  struct net_device           *cached_dev;
  int      (*xmit)(struct sk_buff *skb);
  struct packet_type          prot_hook; /* packet_rcv */
};

int packet_create(
  struct net *net, struct socket *sock, int protocol, int kern)
{
  struct sock *sk;
  struct packet_sock *po;
  __be16 proto = (__force __be16)protocol; /* weird, but documented */
  int err;

  sock->state = SS_UNCONNECTED;

  err = -ENOBUFS;
  sk = sk_alloc(net, PF_PACKET, GFP_KERNEL, &packet_proto, kern);
  if (sk == NULL)
    goto out;

  sock->ops = &packet_ops;
  if (sock->type == SOCK_PACKET)
    sock->ops = &packet_ops_spkt;

  sock_init_data(sock, sk);

  po = pkt_sk(sk); /* return (struct packet_sock *)sk; */
  init_completion(&po->skb_completion);
  sk->sk_family = PF_PACKET;
  po->num = proto;
  po->xmit = dev_queue_xmit;

  err = packet_alloc_pending(po);
  if (err)
    goto out2;

  packet_cached_dev_reset(po);

  sk->sk_destruct = packet_sock_destruct;
  sk_refcnt_debug_inc(sk);

  /* Attach a protocol block */

  spin_lock_init(&po->bind_lock);
  mutex_init(&po->pg_vec_lock);
  po->rollover = NULL;
  po->prot_hook.func = packet_rcv;

  if (sock->type == SOCK_PACKET)
    po->prot_hook.func = packet_rcv_spkt;

  po->prot_hook.af_packet_priv = sk;

  if (proto) {
    po->prot_hook.type = proto;
    __register_prot_hook(sk);
  }

  sk_add_node_tail_rcu(sk, &net->packet.sklist);

  preempt_disable();
  sock_prot_inuse_add(net, &packet_proto, 1);
  preempt_enable();

  return 0;
out2:
  sk_free(sk);
out:
  return err;
}

void __register_prot_hook(struct sock *sk)
{
  struct packet_sock *po = pkt_sk(sk);

  if (!po->running) {
    if (po->fanout)
      __fanout_link(sk, po);
    else
      dev_add_pack(&po->prot_hook);

    sock_hold(sk);
    po->running = 1;
  }
}

void dev_add_pack(struct packet_type *pt)
{
  struct list_head *head = ptype_head(pt);

  list_add_rcu(&pt->list, head);
}

struct list_head *ptype_head(const struct packet_type *pt)
{
  if (pt->type == htons(ETH_P_ALL))
    return pt->dev ? &pt->dev->ptype_all : &ptype_all;
  else
    return pt->dev ? &pt->dev->ptype_specific :
         &ptype_base[ntohs(pt->type) & PTYPE_HASH_MASK];
}
```

## tcpdump_rcv
```c
/* net/core/dev.c */
int __netif_receive_skb_core(struct sk_buff *skb, bool pfmemalloc)
{
    list_for_each_entry_rcu(ptype, &ptype_all, list) {
        if (!ptype->dev || ptype->dev == skb->dev) {
            if (pt_prev)
                ret = deliver_skb(skb, pt_prev, orig_dev);
            pt_prev = ptype;
        }
    }

    deliver_ptype_list_skb(skb, &pt_prev, orig_dev, type, &orig_dev->ptype_specific);
    if (pt_prev) {
        ret = pt_prev->func(skb, skb->dev, pt_prev, orig_dev);
    }
}

static inline int deliver_skb(struct sk_buff *skb,
                struct packet_type *pt_prev,
                struct net_device *orig_dev)
{
    if (unlikely(skb_orphan_frags_rx(skb, GFP_ATOMIC)))
        return -ENOMEM;
    refcount_inc(&skb->users);
    return pt_prev->func(skb, skb->dev, pt_prev, orig_dev);
}

int packet_rcv(
  struct sk_buff *skb, struct net_device *dev,
  struct packet_type *pt, struct net_device *orig_dev)
{
  struct sock *sk;
  struct sockaddr_ll *sll;
  struct packet_sock *po;
  u8 *skb_head = skb->data;
  int skb_len = skb->len;
  unsigned int snaplen, res;
  bool is_drop_n_account = false;

  if (skb->pkt_type == PACKET_LOOPBACK)
    goto drop;

  sk = pt->af_packet_priv;
  po = pkt_sk(sk);

  if (!net_eq(dev_net(dev), sock_net(sk)))
    goto drop;

  skb->dev = dev;

  if (dev->header_ops) {
    /* The device has an explicit notion of ll header,
     * exported to higher levels.
     *
     * Otherwise, the device hides details of its frame
     * structure, so that corresponding packet head is
     * never delivered to user. */
    if (sk->sk_type != SOCK_DGRAM)
      skb_push(skb, skb->data - skb_mac_header(skb));
    else if (skb->pkt_type == PACKET_OUTGOING) {
      /* Special case: outgoing packets have ll header at head */
      skb_pull(skb, skb_network_offset(skb));
    }
  }

  snaplen = skb->len;

  res = run_filter(skb, sk, snaplen);
  if (!res)
    goto drop_n_restore;
  if (snaplen > res)
    snaplen = res;

  if (atomic_read(&sk->sk_rmem_alloc) >= sk->sk_rcvbuf)
    goto drop_n_acct;

  if (skb_shared(skb)) {
    struct sk_buff *nskb = skb_clone(skb, GFP_ATOMIC);
    if (nskb == NULL)
      goto drop_n_acct;

    if (skb_head != skb->data) {
      skb->data = skb_head;
      skb->len = skb_len;
    }
    consume_skb(skb);
    skb = nskb;
  }

  sock_skb_cb_check_size(sizeof(*PACKET_SKB_CB(skb)) + MAX_ADDR_LEN - 8);

  sll = &PACKET_SKB_CB(skb)->sa.ll;
  sll->sll_hatype = dev->type;
  sll->sll_pkttype = skb->pkt_type;
  if (unlikely(po->origdev))
    sll->sll_ifindex = orig_dev->ifindex;
  else
    sll->sll_ifindex = dev->ifindex;

  sll->sll_halen = dev_parse_header(skb, sll->sll_addr);

  /* sll->sll_family and sll->sll_protocol are set in packet_recvmsg().
   * Use their space for storing the original skb length. */
  PACKET_SKB_CB(skb)->sa.origlen = skb->len;

  if (pskb_trim(skb, snaplen))
    goto drop_n_acct;

  skb_set_owner_r(skb, sk);
  skb->dev = NULL;
  skb_dst_drop(skb);

  /* drop conntrack reference */
  nf_reset(skb);

  spin_lock(&sk->sk_receive_queue.lock);
  po->stats.stats1.tp_packets++;
  sock_skb_set_dropcount(sk, skb);

  __skb_queue_tail(&sk->sk_receive_queue, skb);

  spin_unlock(&sk->sk_receive_queue.lock);
  sk->sk_data_ready(sk); /* sock_def_readable */
  return 0;

drop_n_acct:
  is_drop_n_account = true;
  spin_lock(&sk->sk_receive_queue.lock);
  po->stats.stats1.tp_drops++;
  atomic_inc(&sk->sk_drops);
  spin_unlock(&sk->sk_receive_queue.lock);

drop_n_restore:
  if (skb_head != skb->data && skb_shared(skb)) {
    skb->data = skb_head;
    skb->len = skb_len;
  }
drop:
  if (!is_drop_n_account)
    consume_skb(skb);
  else
    kfree_skb(skb);
  return 0;
}

unsigned int run_filter(
  struct sk_buff *skb, const struct sock *sk, unsigned int res)
{
  struct sk_filter *filter;

  filter = rcu_dereference(sk->sk_filter);
  if (filter != NULL)
    res = bpf_prog_run_clear_cb(filter->prog, skb);

  return res;
}

u32 bpf_prog_run_clear_cb(const struct bpf_prog *prog, struct sk_buff *skb)
{
  u8 *cb_data = bpf_skb_cb(skb);

  if (unlikely(prog->cb_access))
    memset(cb_data, 0, BPF_SKB_CB_LEN);

  return BPF_PROG_RUN(prog, skb);
}

#define BPF_PROG_RUN(filter, ctx)  (*(filter)->bpf_func)(ctx, (filter)->insnsi)
```

## tcpdump_snd
```c
int xmit_one(struct sk_buff *skb, struct net_device *dev,
        struct netdev_queue *txq, bool more)
{
  unsigned int len;
  int rc;

  if (!list_empty(&ptype_all) || !list_empty(&dev->ptype_all))
    dev_queue_xmit_nit(skb, dev);

  len = skb->len;
  trace_net_dev_start_xmit(skb, dev);
  rc = netdev_start_xmit(skb, dev, txq, more);
  trace_net_dev_xmit(skb, rc, dev, len);

  return rc;
}

void dev_queue_xmit_nit(struct sk_buff *skb, struct net_device *dev)
{
  struct packet_type *ptype;
  struct sk_buff *skb2 = NULL;
  struct packet_type *pt_prev = NULL;
  struct list_head *ptype_list = &ptype_all;

  rcu_read_lock();
again:
  list_for_each_entry_rcu(ptype, ptype_list, list) {
    /* Never send packets back to the socket
     * they originated from - MvS (miquels@drinkel.ow.org) */
    if (skb_loop_sk(ptype, skb))
      continue;

    if (pt_prev) {
      deliver_skb(skb2, pt_prev, skb->dev);
      pt_prev = ptype;
      continue;
    }

    /* need to clone skb, done only once */
    skb2 = skb_clone(skb, GFP_ATOMIC);
    if (!skb2)
      goto out_unlock;

    net_timestamp_set(skb2);

    /* skb->nh should be correctly
     * set by sender, so that the second statement is
     * just protection against buggy protocols. */
    skb_reset_mac_header(skb2);

    if (skb_network_header(skb2) < skb2->data || skb_network_header(skb2) > skb_tail_pointer(skb2)) {
      net_crit_ratelimited("protocol %04x is buggy, dev %s\n", ntohs(skb2->protocol), dev->name);
      skb_reset_network_header(skb2);
    }

    skb2->transport_header = skb2->network_header;
    skb2->pkt_type = PACKET_OUTGOING;
    pt_prev = ptype;
  }

  if (ptype_list == &ptype_all) {
    ptype_list = &dev->ptype_all;
    goto again;
  }

out_unlock:
  if (pt_prev) {
    if (!skb_orphan_frags_rx(skb2, GFP_ATOMIC))
      pt_prev->func(skb2, skb->dev, pt_prev, skb->dev);
    else
      kfree_skb(skb2);
  }
  rcu_read_unlock();
}
```

```c
socket();
  sock_create();
    _sock_create();
      sock = sock_alloc();
        inode = new_inode_pseudo(sock_mnt->mnt_sb);
        sock = SOCKET_I(inode);
        inode->i_op = &sockfs_inode_ops;
      pf = net_families[family]; /* get AF */
      pf->create(); /* inet_family_ops.inet_create */
        packet_create()
          struct sock *sk = sk_alloc();
          sock->ops = &packet_ops;
          struct packet_sock *po = pkt_sk(sk); /* return (struct packet_sock *)sk; */
          po->prot_hook.func = packet_rcv;

          __register_prot_hook();
            dev_add_pack(&po->prot_hook);
              list_add_rcu(&pt->list, &ptype_all);

  sock_map_fd();
    sock_alloc_file();
      sock->file = file;
      file->private_data = sock;

/* receive packet */
  __netif_receive_skb_core();
    list_for_each_entry_rcu(ptype, &ptype_all, list);
      deliver_skb();
        pt_prev->func();
          packet_rcv();
            run_filter();
            __skb_queue_tail(&sk->sk_receive_queue, skb);
            sk->sk_data_ready(sk);
              sock_def_readable();
                if (skwq_has_sleeper(wq));
                  wake_up_interruptible_sync_poll();

/* send packet */
xmit_one()
  deliver_skb();
    ->
```

# ACK, SYN, FIN

## tcp_send_ack
```c
void tcp_send_ack(struct sock *sk)
{
  __tcp_send_ack(sk, tcp_sk(sk)->rcv_nxt);
}

void __tcp_send_ack(struct sock *sk, u32 rcv_nxt)
{
  struct sk_buff *buff;

  /* If we have been reset, we may not send again. */
  if (sk->sk_state == TCP_CLOSE)
    return;

  /* We are not putting this on the write queue, so
   * tcp_transmit_skb() will set the ownership to this sock. */
  buff = alloc_skb(MAX_TCP_HEADER, sk_gfp_mask(sk, GFP_ATOMIC | __GFP_NOWARN));
  if (unlikely(!buff)) {
    inet_csk_schedule_ack(sk);
    inet_csk(sk)->icsk_ack.ato = TCP_ATO_MIN;
    inet_csk_reset_xmit_timer(sk, ICSK_TIME_DACK, TCP_DELACK_MAX, TCP_RTO_MAX);
    return;
  }

  /* Reserve space for headers and prepare control bits. */
  skb_reserve(buff, MAX_TCP_HEADER);
  tcp_init_nondata_skb(buff, tcp_acceptable_seq(sk), TCPHDR_ACK);

  /* We do not want pure acks influencing TCP Small Queues or fq/pacing
   * too much.
   * SKB_TRUESIZE(max(1 .. 66, MAX_TCP_HEADER)) is unfortunately ~784 */
  skb_set_tcp_pure_ack(buff);

  /* Send it off, this clears delayed acks for us. */
  tcp_transmit_skb(sk, buff, 0, (__force gfp_t)0, rcv_nxt);
}

/* If SYN/FIN is present, auto increment end seqno. */
void tcp_init_nondata_skb(struct sk_buff *skb, u32 seq, u8 flags)
{
  skb->ip_summed = CHECKSUM_PARTIAL;

  TCP_SKB_CB(skb)->tcp_flags = flags;
  TCP_SKB_CB(skb)->sacked = 0;

  tcp_skb_pcount_set(skb, 1);

  TCP_SKB_CB(skb)->seq = seq;
  if (flags & (TCPHDR_SYN | TCPHDR_FIN))
    seq++;
  TCP_SKB_CB(skb)->end_seq = seq;
}
```

## tcp_send_delayed_ack
```c
tcp_v4_do_rcv();
  tcp_rcv_established();
    __tcp_ack_snd_check();
      tcp_send_delayed_ack();
```

## tcp_send_synack

## tcp_fin
```c
/* Process the FIN bit. This now behaves as it is supposed to work
 *  and the FIN takes effect when it is validly part of sequence
 *  space. Not before when we get holes.
 *
 *  If we are ESTABLISHED, a received fin moves us to CLOSE-WAIT
 *  (and thence onto LAST-ACK and finally, CLOSE, we never enter
 *  TIME-WAIT)
 *
 *  If we are in FINWAIT-1, a received FIN indicates simultaneous
 *  close and we go into CLOSING (and later onto TIME-WAIT)
 *
 *  If we are in FINWAIT-2, a received FIN moves us to TIME-WAIT. */
void tcp_fin(struct sock *sk)
{
    struct tcp_sock *tp = tcp_sk(sk);

    inet_csk_schedule_ack(sk);

    sk->sk_shutdown |= RCV_SHUTDOWN;
    sock_set_flag(sk, SOCK_DONE);

    switch (sk->sk_state) {
    case TCP_SYN_RECV:
    case TCP_ESTABLISHED:
        /* Move to CLOSE_WAIT */
        tcp_set_state(sk, TCP_CLOSE_WAIT);
        inet_csk(sk)->icsk_ack.pingpong = 1;
        break;

    case TCP_CLOSE_WAIT:
    case TCP_CLOSING:
        /* Received a retransmission of the FIN, do nothing.*/
        break;
    case TCP_LAST_ACK:
        /* RFC793: Remain in the LAST-ACK state. */
        break;

    case TCP_FIN_WAIT1:
        /* This case occurs when a simultaneous close
         * happens, we must ack the received FIN and
         * enter the CLOSING state. */
        tcp_send_ack(sk);
        tcp_set_state(sk, TCP_CLOSING);
        break;
    case TCP_FIN_WAIT2:
        /* Received a FIN -- send ACK and enter TIME_WAIT. */
        tcp_send_ack(sk);
        tcp_time_wait(sk, TCP_TIME_WAIT, 0);
        break;
    default:
        /* Only TCP_LISTEN and TCP_CLOSE are left, in these
         * cases we should never reach this piece of code. */
        break;
    }

    /* It _is_ possible, that we have something out-of-order _after_ FIN.
     * Probably, we should reset in this case. For now drop them. */
    skb_rbtree_purge(&tp->out_of_order_queue);
    if (tcp_is_sack(tp))
        tcp_sack_reset(&tp->rx_opt);
    sk_mem_reclaim(sk);

    if (!sock_flag(sk, SOCK_DEAD)) {
        sk->sk_state_change(sk); /* sock_def_wakeup */

        /* Do not send POLL_HUP for half duplex close. */
        if (sk->sk_shutdown == SHUTDOWN_MASK || sk->sk_state == TCP_CLOSE)
            sk_wake_async(sk, SOCK_WAKE_WAITD, POLL_HUP);
        else
            sk_wake_async(sk, SOCK_WAKE_WAITD, POLL_IN);
    }
}

void sk_wake_async(const struct sock *sk, int how, int band)
{
    if (sock_flag(sk, SOCK_FASYNC)) {
        rcu_read_lock();
        sock_wake_async(rcu_dereference(sk->sk_wq), how, band);
        rcu_read_unlock();
    }
}

int sock_wake_async(struct socket_wq *wq, int how, int band)
{
    if (!wq || !wq->fasync_list)
        return -1;

    switch (how) {
    case SOCK_WAKE_WAITD:
        if (test_bit(SOCKWQ_ASYNC_WAITDATA, &wq->flags))
            break;
        goto call_kill;
    case SOCK_WAKE_SPACE:
        if (!test_and_clear_bit(SOCKWQ_ASYNC_NOSPACE, &wq->flags))
            break;
        /* fall through */
    case SOCK_WAKE_IO:
call_kill:
        kill_fasync(&wq->fasync_list, SIGIO, band);
        break;
    case SOCK_WAKE_URG:
        kill_fasync(&wq->fasync_list, SIGURG, band);
    }

    return 0;
}

void kill_fasync(struct fasync_struct **fp, int sig, int band)
{
    /* First a quick test without locking: usually
     * the list is empty. */
    if (*fp) {
        rcu_read_lock();
        kill_fasync_rcu(rcu_dereference(*fp), sig, band);
        rcu_read_unlock();
    }
}

void kill_fasync_rcu(struct fasync_struct *fa, int sig, int band)
{
    while (fa) {
        struct fown_struct *fown;

        if (fa->magic != FASYNC_MAGIC) {
            return;
        }
        read_lock(&fa->fa_lock);
        if (fa->fa_file) {
            fown = &fa->fa_file->f_owner;
            /* Don't send SIGURG to processes which have not set a
               queued signum: SIGURG has its own default signalling
               mechanism. */
            if (!(sig == SIGURG && fown->signum == 0))
                send_sigio(fown, fa->fa_fd, band);
        }
        read_unlock(&fa->fa_lock);
        fa = rcu_dereference(fa->fa_next);
    }
}
```

## tcp_send_fin
```c
void tcp_send_fin(struct sock *sk)
{
    struct sk_buff *skb, *tskb = tcp_write_queue_tail(sk);
    struct tcp_sock *tp = tcp_sk(sk);

    /* Optimization, tack on the FIN if we have one skb in write queue and
     * this skb was not yet sent, or we are under memory pressure.
     * Note: in the latter case, FIN packet will be sent after a timeout,
     * as TCP stack thinks it has already been transmitted. */
    if (!tskb && tcp_under_memory_pressure(sk))
        tskb = skb_rb_last(&sk->tcp_rtx_queue);

    if (tskb) {
coalesce:
        TCP_SKB_CB(tskb)->tcp_flags |= TCPHDR_FIN;
        TCP_SKB_CB(tskb)->end_seq++;
        tp->write_seq++;
        if (tcp_write_queue_empty(sk)) {
            /* This means tskb was already sent.
             * Pretend we included the FIN on previous transmit.
             * We need to set tp->snd_nxt to the value it would have
             * if FIN had been sent. This is because retransmit path
             * does not change tp->snd_nxt. */
            tp->snd_nxt++;
            return;
        }
    } else {
        skb = alloc_skb_fclone(MAX_TCP_HEADER, sk->sk_allocation);
        if (unlikely(!skb)) {
            if (tskb)
                goto coalesce;
            return;
        }
        INIT_LIST_HEAD(&skb->tcp_tsorted_anchor);
        skb_reserve(skb, MAX_TCP_HEADER);
        sk_forced_mem_schedule(sk, skb->truesize);
        /* FIN eats a sequence byte, write_seq advanced by tcp_queue_skb(). */
        tcp_init_nondata_skb(skb, tp->write_seq, TCPHDR_ACK | TCPHDR_FIN);
        tcp_queue_skb(sk, skb);
    }
    __tcp_push_pending_frames(sk, tcp_current_mss(sk), TCP_NAGLE_OFF);
}

void __tcp_push_pending_frames(
    struct sock *sk, unsigned int cur_mss, int nonagle)
{
    /* If we are closed, the bytes will have to remain here.
     * In time closedown will finish, we empty the write queue and
     * all will be happy. */
    if (unlikely(sk->sk_state == TCP_CLOSE))
        return;

    if (tcp_write_xmit(sk, cur_mss, nonagle, 0, sk_gfp_mask(sk, GFP_ATOMIC)))
        tcp_check_probe_timer(sk);
}
```

# epoll

<img src='../images/kernel/net-epoll.svg' style='max-height:850px'/>

---

<img src='../images/kernel/net-epoll-thread-model.svg' style='max-height:850px'/>

```c
typedef union epoll_data {
    void     *ptr;
    int      fd;
    uint32_t u32;
    uint64_t u64;
} epoll_data_t;

struct epoll_event {
    uint32_t     events;    /* Epoll events */
    epoll_data_t data;      /* User data variable */
};

int epoll_create(int size);
int epoll_ctl(int epfd, int op, int fd, struct epoll_event *event);
int epoll_wait(int epfd, struct epoll_event *events, int maxevents, int timeout);

struct file {
    struct list_head  f_ep_links;
    struct list_head  f_tfile_llink;
};

/* List of files with newly added links, where we may need to limit the number
 * of emanating paths. Protected by the epmutex.
 *
 * list_add(&tf.file->f_tfile_llink, &tfile_check_list); */
LIST_HEAD(tfile_check_list);

struct epitem {
    union {
        struct rb_node  rbn;
        struct rcu_head rcu;
    };

    struct list_head    rdllink;
    /* Nested epoll Support
     * support a collision chain in the hash table */
    struct epitem       *next;
    struct epoll_filefd  ffd;
    /* Number of active wait queue attached to poll operations */
    int                 nwait;
    struct eppoll_entry *pwqlist; /* List containing poll wait queues */
    struct eventpoll    *ep;
    struct list_head    fllink;
    struct wakeup_source *ws;
    struct epoll_event  event;
};

struct epoll_filefd {
    struct file *file;
    int         fd;
};

struct eventpoll {
    struct mutex mtx;
    /* Wait queue used by sys_epoll_wait() */
    wait_queue_head_t     wq;

    /* Nested epoll. Wait queue used by file->poll() */
    wait_queue_head_t     poll_wait;

    struct list_head      rdllist;
    struct rb_root_cached rbr;
    struct epitem         *ovflist;
    struct wakeup_source  *ws;
    struct user_struct    *user;
    struct file           *file;
    int                   visited;
    struct list_head      visited_list_link;
    unsigned int          napi_id;
};
```

## call-graph-epoll

```c
/* epoll_create */
epoll_create() {
    do_epoll_create() {
        ep_alloc();
        get_unused_fd_flags();
        anon_inode_getfile();
        fd_install();
    }
}

/* epoll_ctl - EPOLL_CTL_ADD */
ep_insert() {
    struct ep_pqueue epq;
    kmem_cache_alloc(epi_cache);
    init_poll_funcptr(&epq.pt, ep_ptable_queue_proc);
    revents = ep_item_poll(epi, &pt, 1) {
        if (!is_file_epoll(epi->ffd.file)) { /* return f->f_op == &eventpoll_fops; */
            vfs_poll(file,  &epq.pt, 1) {
                file->f_op->poll() {
                    sock_poll(struct file *file, poll_table *wait) {
                        sock->ops->poll(); /* tcp_poll() */
                        /* 1. register observer
                         * pass _qproc NULL if only polling */
                        sock_poll_wait() {
                            if (!poll_does_not_wait(p)) /* return p == NULL || p->_qproc == NULL */
                                poll_wait(filp, &sock->wq->wait, p) {
                                    if (p && p->_qproc && wait_address) {
                                        p->_qproc() {
                                            init_waitqueue_func_entry(&pwq->wait, ep_poll_callback);
                                            add_wait_queue(whead, &pwq->wait);
                                        }
                                    }
                                }
                        }
                        /* 2. poll listen socket */
                        if (state == TCP_LISTEN) {
                            return inet_csk_listen_poll(sk) {
                                return !reqsk_queue_empty(&inet_csk(sk)->icsk_accept_queue)
                                    ? (EPOLLIN | EPOLLRDNORM) : 0;
                            }
                        }

                        /* 3. poll ready events */
                        if (tcp_stream_is_readable(tp, target, sk))
                            mask |= EPOLLIN | EPOLLRDNORM;
                        if (sk_stream_is_writeable(sk))
                            mask |= EPOLLOUT | EPOLLWRNORM;
                        return mask;
                    }
                    eventfd_poll(struct file *file, poll_table *wait) {
                        poll_wait(struct file * filp, wait_queue_head_t * queue, poll_table *pt)
                            pt->_qproc(filep, queue, pt) {
                                /* ep_ptable_queue_proc,
                                 * virqfd_ptable_queue_proc,
                                 * irqfd_ptable_queue_proc,
                                 * memcg_event_ptable_queue_proc */
                            }
                        if (count > 0)
                            events |= EPOLLIN;
                        if (count == ULLONG_MAX)
                            events |= EPOLLERR;
                        if (ULLONG_MAX - 1 > count)
                            events |= EPOLLOUT;

                        return events;
                    }
                }
            }
        } else {
            poll_wait(epi->ffd.file, &ep->poll_wait, pt);
            return ep_scan_ready_list(ep_read_events_proc);
        }
    }

    list_add_tail_rcu(&epi->fllink, &tfile->f_ep_links);
    ep_rbtree_insert(ep, epi);

    if (revents && !ep_is_linked(epi)) {
        wake_up_locked(&ep->wq);
        ep_poll_safewake(&ep->poll_wait);
    }
}

/* epoll_ctl - EPOLL_CTL_DEL */
ep_remove() {
    ep_unregister_pollwait() {
        while ((pwq = *p) != NULL) {
            ep_remove_wait_queue(pwq);
        }
    }
}

/* epoll_ctl - EPOLL_CTL_MOD */
ep_modify();
    poll_table pt;
    init_poll_funcptr(&pt, NULL);
    if (ep_item_poll(epi, &pt, 1)) {
        spin_lock_irq(&ep->wq.lock);
        if (!ep_is_linked(epi)) {
            list_add_tail(&epi->rdllink, &ep->rdllist);
            ep_pm_stay_awake(epi);

            if (waitqueue_active(&ep->wq))
                wake_up_locked(&ep->wq);
            if (waitqueue_active(&ep->poll_wait))
                pwake++;
        }
        spin_unlock_irq(&ep->wq.lock);
    }

/* epoll_wait */
epoll_wait() {
    do_epoll_wait() {
        ep = f.file->private_data;
        ep_poll(ep) {
            eavail = ep_events_available(ep);
            while (1) {
                if (eavail) {
                    res = ep_send_events(ep, events, maxevents);
                    if (res) {
                        if (res > 0)
                            ep_suspend_napi_irqs(ep);
                        return res;
                    }
                }

                eavail = ep_busy_loop(ep, timed_out) {
                    napi_busy_loop();
                }
                if (eavail)
                    continue;

                init_wait(&wait);
                wait.func = ep_autoremove_wake_function;

                eavail = ep_events_available(ep);
                if (!eavail)
                    __add_wait_queue_exclusive(&ep->wq, &wait);

                schedule_hrtimeout_range(to, slack, HRTIMER_MODE_ABS);

                if (!list_empty_careful(&wait.entry)) {
                    __remove_wait_queue(&ep->wq, &wait);
                }
            }

            ep_send_events() {
                /* 1. send ready events to user space */
                for (ep->rdlist) {
                    init_poll_funcptr(&pt, NULL);
                    revents = ep_item_poll(epi, &pt, 1) {
                        if (tcp_stream_is_readable(tp, target, sk))
                            revents |= EPOLLIN | EPOLLRDNORM;
                        if (sk_stream_is_writeable(sk))
                            revents |= EPOLLOUT | EPOLLWRNORM;
                        return revents & epi->event.events;
                    }

                    if (revents) {
                        if (__put_user(revents, &uevent->events));
                            list_add(&epi->rdllink, head);

                        if (epi->event.events & EPOLLONESHOT)
                            epi->event.events &= EP_PRIVATE_BITS;
                        else if (!(epi->event.events & EPOLLET)) {
                            list_add_tail(&epi->rdllink, &ep->rdllist);
                            ep_pm_stay_awake(epi);
                        }
                    }
                }

                ep_done_scan() {
                    /* 2. link overflow events */
                    for (ep->ovflist) {
                        if (!ep_is_linked(epi)) {
                            list_add_tail(&epi->rdllink, &ep->rdllist);
                        }
                    }

                    /* 3. wake up observers */
                    if (!list_empty(&ep->rdllist)) {
                        wake_up_locked(&ep->wq);
                        ep_poll_safewake(&ep->poll_wait);
                            __wake_up_common_lock();
                    }
                }
            }
        }
    }
}

/* wake epoll_wait */
tcp_data_ready() {
    sk->sk_data_ready(sk) {
        sock_def_readable() {
            if (skwq_has_sleeper(wq)) {
                wake_up_interruptible_sync_poll() {
                    __wake_up_common() {

                        list_for_each_entry_safe_from(curr, next, &wq_head->head) {
                            ret = curr->func(curr, mode, wake_flags, key) {
                                ep_poll_callback() {
                                    /* 1. check EPOLLONCESHOT */
                                    if (!(epi->event.events & ~EP_PRIVATE_BITS))
                                        return;
                                    /* 2. check interested events */
                                    if (!(pollflags & epi->event.events))
                                        return;
                                    /* 3. add to epoll ready list */
                                    if (!ep_is_linked(epi))
                                        list_add_tail(&epi->rdllink, &ep->rdllist);
                                    /* 4. wake up */
                                    wake_up_locked(&ep->wq);
                                        __wake_up_common();
                                    ep_poll_safewake(&ep->poll_wait);
                                        wake_up_poll();
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

tcp_rcv_state_process() {
    tcp_data_snd_check() {
        tcp_check_space() {
            if (sock_flag(sk, SOCK_QUEUE_SHRUNK)) {
                /* sock tranfroms from non-writale(SOCK_NOSPACE) to writable(SOCK_QUEUE_SHRUNK) */
                if (sk->sk_socket && test_bit(SOCK_NOSPACE, &sk->sk_socket->flags)) {
                    tcp_new_space(sk) {
                        if (tcp_should_expand_sndbuf(sk)) {
                            tcp_sndbuf_expand(sk);
                            tp->snd_cwnd_stamp = tcp_jiffies32;
                        }

                        sk->sk_write_space(sk); /* sock_def_write_space */
                    }
                }
            }
        }
    }
}
```

## epoll_create

```c
SYSCALL_DEFINE1(epoll_create, int, size)
{
    if (size <= 0)
        return -EINVAL;

    return do_epoll_create(0);
}

int do_epoll_create(int flags)
{
    int error, fd;
    struct eventpoll *ep = NULL;
    struct file *file;

    error = ep_alloc(&ep);

    fd = get_unused_fd_flags(O_RDWR | (flags & O_CLOEXEC));
    file = anon_inode_getfile("[eventpoll]", &eventpoll_fops, ep,
            O_RDWR | (flags & O_CLOEXEC));

    ep->file = file;
    fd_install(fd, file);
    return fd;
}

const struct file_operations eventpoll_fops = {
    .show_fdinfo  = ep_show_fdinfo,
    .release      = ep_eventpoll_release,
    .poll         = ep_eventpoll_poll,
    .llseek       = noop_llseek,
};

int ep_alloc(struct eventpoll **pep)
{
    int error;
    struct user_struct *user;
    struct eventpoll *ep;

    user = get_current_user();
    error = -ENOMEM;
    ep = kzalloc(sizeof(*ep), GFP_KERNEL);
    if (unlikely(!ep))
        goto free_uid;

    mutex_init(&ep->mtx);
    init_waitqueue_head(&ep->wq);
    init_waitqueue_head(&ep->poll_wait);
    INIT_LIST_HEAD(&ep->rdllist);
    ep->rbr = RB_ROOT_CACHED;
    ep->ovflist = EP_UNACTIVE_PTR;
    ep->user = user;

    *pep = ep;

    return 0;

free_uid:
    free_uid(user);
    return error;
}
```

## epoll_ctl

```c
SYSCALL_DEFINE4(epoll_ctl, int, epfd, int, op, int, fd,
    struct epoll_event __user *, event)
{
    int error;
    int full_check = 0;
    struct fd f, tf;
    struct eventpoll *ep;
    struct epitem *epi;
    struct epoll_event epds;
    struct eventpoll *tep = NULL;

    error = -EBADF;
    f = fdget(epfd);
    tf = fdget(fd);

    if (!file_can_poll(tf.file))
        goto error_tgt_fput;

    /* Check if EPOLLWAKEUP is allowed */
    if (ep_op_has_event(op))
        ep_take_care_of_epollwakeup(&epds);

    if (f.file == tf.file || !is_file_epoll(f.file))
        goto error_tgt_fput;

    ep = f.file->private_data;

    mutex_lock_nested(&ep->mtx, 0);
    if (op == EPOLL_CTL_ADD) {
        if (!list_empty(&f.file->f_ep_links) || is_file_epoll(tf.file)) {
            full_check = 1;
            mutex_unlock(&ep->mtx);
            mutex_lock(&epmutex);

            if (is_file_epoll(tf.file)) {
                error = -ELOOP;
                if (ep_loop_check(ep, tf.file) != 0) {
                    clear_tfile_check_list();
                    goto error_tgt_fput;
                }
            } else {
                list_add(&tf.file->f_tfile_llink, &tfile_check_list);
            }

            mutex_lock_nested(&ep->mtx, 0);
            if (is_file_epoll(tf.file)) {
                tep = tf.file->private_data;
                mutex_lock_nested(&tep->mtx, 1);
            }
        }
    }

    epi = ep_find(ep, tf.file, fd);

    switch (op) {
    case EPOLL_CTL_ADD:
        if (!epi) {
            epds.events |= EPOLLERR | EPOLLHUP;
            error = ep_insert(ep, &epds, tf.file, fd, full_check);
        } else
            error = -EEXIST;
            if (full_check)
                clear_tfile_check_list();
        break;

    case EPOLL_CTL_DEL:
        if (epi)
            error = ep_remove(ep, epi);
        else
            error = -ENOENT;
        break;

    case EPOLL_CTL_MOD:
        if (epi) {
            if (!(epi->event.events & EPOLLEXCLUSIVE)) {
                epds.events |= EPOLLERR | EPOLLHUP;
                error = ep_modify(ep, epi, &epds);
            }
        } else
            error = -ENOENT;
        break;
    }

    if (tep != NULL)
        mutex_unlock(&tep->mtx);
    mutex_unlock(&ep->mtx);

error_tgt_fput:
    if (full_check)
        mutex_unlock(&epmutex);

    fdput(tf);
error_fput:
    fdput(f);
error_return:

    return error;
}
```

### ep_insert

```c
struct ep_pqueue {
    poll_table      pt;
    struct epitem   *epi;
};

typedef struct poll_table_struct {
    /* register observer for file wait queue */
    poll_queue_proc _qproc; /* ep_ptable_queue_proc */
    __poll_t        _key;
} poll_table;

/* structures and helpers for f_op->poll implementations */
typedef void (*poll_queue_proc)(struct file *, wait_queue_head_t *, struct poll_table_struct *);

int ep_insert(
  struct eventpoll *ep, const struct epoll_event *event,
  struct file *tfile, int fd, int full_check)
{
    int error, pwake = 0;
    __poll_t revents;
    long user_watches;
    struct epitem *epi;
    struct ep_pqueue epq;

    if (!(epi = kmem_cache_alloc(epi_cache, GFP_KERNEL)))
        return -ENOMEM;

    INIT_LIST_HEAD(&epi->rdllink);
    INIT_LIST_HEAD(&epi->fllink);
    INIT_LIST_HEAD(&epi->pwqlist);
    epi->ep = ep;
    ep_set_ffd(&epi->ffd, tfile, fd);
    epi->event = *event;
    epi->nwait = 0;
    epi->next = EP_UNACTIVE_PTR;
    if (epi->event.events & EPOLLWAKEUP) {
        error = ep_create_wakeup_source(epi);
        if (error)
            goto error_create_wakeup_source;
    } else {
        RCU_INIT_POINTER(epi->ws, NULL);
    }

    epq.epi = epi;
    init_poll_funcptr(&epq.pt, ep_ptable_queue_proc) {
        pt->_qproc  = qproc;
        pt->_key    = ~(__poll_t)0; /* all events enabled */
    }

    revents = ep_item_poll(epi, &epq.pt, 1);
        --->

    if (epi->nwait < 0)
        goto error_unregister;

    /* Add the current item to the list of active epoll hook for this file */
    spin_lock(&tfile->f_lock);
    list_add_tail_rcu(&epi->fllink, &tfile->f_ep_links);
    spin_unlock(&tfile->f_lock);

    ep_rbtree_insert(ep, epi);

    /* now check if we've created too many backpaths */
    if (full_check && reverse_path_check())
        goto error_remove_epi;

    /* We have to drop the new item inside our item list to keep track of it */
    spin_lock_irq(&ep->wq.lock);

    /* Set epoll busy poll NAPI ID from sk. */
    ep_set_busy_poll_napi_id(epi);

    /* If the file is already "ready" we drop it inside the ready list */
    if (revents && !ep_is_linked(epi)) {
        list_add_tail(&epi->rdllink, &ep->rdllist);
        ep_pm_stay_awake(epi);

        /* Notify waiting tasks that events are available */
        if (waitqueue_active(&ep->wq))
            wake_up_locked(&ep->wq);
        if (waitqueue_active(&ep->poll_wait))
            pwake++;
    }

    spin_unlock_irq(&ep->wq.lock);

    atomic_long_inc(&ep->user->epoll_watches);

    /* We have to call this outside the lock */
    if (pwake)
        ep_poll_safewake(&ep->poll_wait);

    return 0;
}
```

### ep_modify

```c
int ep_modify(struct eventpoll *ep, struct epitem *epi,
         const struct epoll_event *event)
{
    int pwake = 0;
    poll_table pt;

    init_poll_funcptr(&pt, NULL);

    epi->event.events = event->events; /* need barrier below */
    epi->event.data = event->data; /* protected by mtx */
    if (epi->event.events & EPOLLWAKEUP) {
        if (!ep_has_wakeup_source(epi))
            ep_create_wakeup_source(epi);
    } else if (ep_has_wakeup_source(epi)) {
        ep_destroy_wakeup_source(epi);
    }

    smp_mb();

    if (ep_item_poll(epi, &pt, 1)) {
        spin_lock_irq(&ep->wq.lock);
        if (!ep_is_linked(epi)) {
            list_add_tail(&epi->rdllink, &ep->rdllist);
            ep_pm_stay_awake(epi);

            /* Notify waiting tasks that events are available */
            if (waitqueue_active(&ep->wq))
                wake_up_locked(&ep->wq);
            if (waitqueue_active(&ep->poll_wait))
                pwake++;
        }
        spin_unlock_irq(&ep->wq.lock);
    }

    /* We have to call this outside the lock */
    if (pwake)
        ep_poll_safewake(&ep->poll_wait);

    return 0;
}
```

### ep_remove

```c
static void ep_remove_safe(struct eventpoll *ep, struct epitem *epi)
{
    struct file *file = epi->ffd.file;
    struct epitems_head *to_free;
    struct hlist_head *head;

    lockdep_assert_irqs_enabled();

    ep_unregister_pollwait(ep, epi) {
        struct eppoll_entry **p = &epi->pwqlist;
        struct eppoll_entry *pwq;

        while ((pwq = *p) != NULL) {
            *p = pwq->next;
            ep_remove_wait_queue(pwq) {
                wait_queue_head_t *whead;

                rcu_read_lock();
                whead = smp_load_acquire(&pwq->whead);
                if (whead) {
                    remove_wait_queue(whead, &pwq->wait) {
                        list_del(&wq_entry->entry);
                    }
                }
                rcu_read_unlock();
            }
            kmem_cache_free(pwq_cache, pwq);
        }
    }

    /* Remove the current item from the list of epoll hooks */
    spin_lock(&file->f_lock);
    if (epi->dying && !force) {
        spin_unlock(&file->f_lock);
        return false;
    }

    to_free = NULL;
    head = file->f_ep;
    if (head->first == &epi->fllink && !epi->fllink.next) {
        /* See eventpoll_release() for details. */
        WRITE_ONCE(file->f_ep, NULL);
        if (!is_file_epoll(file)) {
            struct epitems_head *v;
            v = container_of(head, struct epitems_head, epitems);
            if (!smp_load_acquire(&v->next))
                to_free = v;
        }
    }
    hlist_del_rcu(&epi->fllink);
    spin_unlock(&file->f_lock);
    free_ephead(to_free);

    rb_erase_cached(&epi->rbn, &ep->rbr);

    write_lock_irq(&ep->lock);
    if (ep_is_linked(epi))
        list_del_init(&epi->rdllink);
    write_unlock_irq(&ep->lock);

    wakeup_source_unregister(ep_wakeup_source(epi));

    kfree_rcu(epi, rcu);

    percpu_counter_dec(&ep->user->epoll_watches);
    return ep_refcount_dec_and_test(ep);
}
```

## epoll_wait

```c
SYSCALL_DEFINE4(epoll_wait, int, epfd, struct epoll_event __user *, events,
    int, maxevents, int, timeout)
{
    return do_epoll_wait(epfd, events, maxevents, timeout);
}

int do_epoll_wait(int epfd, struct epoll_event __user *events,
       int maxevents, int timeout)
{
    int error;
    struct fd f;
    struct eventpoll *ep;

    f = fdget(epfd);
    ep = f.file->private_data;
    error = ep_poll(ep, events, maxevents, timeout);

    return error;
}

int ep_poll(
    struct eventpoll *ep, struct epoll_event __user *events,
    int maxevents, long timeout)
{
    int res = 0, eavail, timed_out = 0;
    u64 slack = 0;
    wait_queue_entry_t wait;
    ktime_t expires, *to = NULL;

    if (timeout && (timeout->tv_sec | timeout->tv_nsec)) {
        slack = select_estimate_accuracy(&end_time);
        to = &expires;
        *to = timespec64_to_ktime(end_time);
    } else if (timeout == 0) {
        timed_out = 1;
    }

    eavail = ep_events_available(ep) {
        return !list_empty_careful(&ep->rdllist) ||
            READ_ONCE(ep->ovflist) != EP_UNACTIVE_PTR;
    }

    while (1) {
        if (eavail) {
            res = ep_send_events(ep, events, maxevents);
            if (res) {
                if (res > 0)
                    ep_suspend_napi_irqs(ep);
                return res;
            }
        }

        if (timed_out)
            return 0;

        eavail = ep_busy_loop(ep, timed_out);
            --->
        if (eavail)
            continue;

        if (signal_pending(current))
            return -EINTR;

        init_wait(&wait);
        wait.func = ep_autoremove_wake_function = []() {
            int ret = default_wake_function(wq_entry, mode, sync, key);
            list_del_init_careful(&wq_entry->entry);
            return ret;
        }

        write_lock_irq(&ep->lock);

        __set_current_state(TASK_INTERRUPTIBLE);

        eavail = ep_events_available(ep);
        if (!eavail) {
            /* both add and wake up wait_queue in exclusive mode */
            __add_wait_queue_exclusive(&ep->wq, &wait);
        }

        write_unlock_irq(&ep->lock);

        if (!eavail)
            timed_out = !schedule_hrtimeout_range(to, slack, HRTIMER_MODE_ABS);
        __set_current_state(TASK_RUNNING);

        eavail = 1;

        if (!list_empty_careful(&wait.entry)) {
            write_lock_irq(&ep->lock);

            if (timed_out)
                eavail = list_empty(&wait.entry);
            __remove_wait_queue(&ep->wq, &wait);
            write_unlock_irq(&ep->lock);
        }
    }
}

void ep_busy_loop(struct eventpoll *ep, int nonblock)
{
    unsigned int napi_id = READ_ONCE(ep->napi_id);
    u16 budget = READ_ONCE(ep->busy_poll_budget);
    bool prefer_busy_poll = READ_ONCE(ep->prefer_busy_poll);

    if (!budget)
        budget = BUSY_POLL_BUDGET;

    if (napi_id >= MIN_NAPI_ID && ep_busy_loop_on(ep)) {
        napi_busy_loop(napi_id, nonblock ? NULL : ep_busy_loop_end, ep, prefer_busy_poll, budget) {
            unsigned long start_time = loop_end ? busy_loop_current_time() : 0;
            int (*napi_poll)(struct napi_struct *napi, int budget);
            struct bpf_net_context __bpf_net_ctx, *bpf_net_ctx;
            void *have_poll_lock = NULL;
            struct napi_struct *napi;

            WARN_ON_ONCE(!rcu_read_lock_held());

        restart:
            napi_poll = NULL;

            napi = napi_by_id(napi_id);
            if (!napi)
                return;

            if (!IS_ENABLED(CONFIG_PREEMPT_RT))
                preempt_disable();
            for (;;) {
                int work = 0;

                local_bh_disable();
                bpf_net_ctx = bpf_net_ctx_set(&__bpf_net_ctx);
                if (!napi_poll) {
                    unsigned long val = READ_ONCE(napi->state);

                    /* If multiple threads are competing for this napi,
                    * we avoid dirtying napi->state as much as we can. */
                    if (val & (NAPIF_STATE_DISABLE | NAPIF_STATE_SCHED | NAPIF_STATE_IN_BUSY_POLL)) {
                        if (flags & NAPI_F_PREFER_BUSY_POLL)
                            set_bit(NAPI_STATE_PREFER_BUSY_POLL, &napi->state);
                        goto count;
                    }
                    if (cmpxchg(&napi->state, val, val | NAPIF_STATE_IN_BUSY_POLL | NAPIF_STATE_SCHED) != val) {
                        if (flags & NAPI_F_PREFER_BUSY_POLL)
                            set_bit(NAPI_STATE_PREFER_BUSY_POLL, &napi->state);
                        goto count;
                    }
                    have_poll_lock = netpoll_poll_lock(napi);
                    napi_poll = napi->poll; /* ixgbe_poll, process_backlog */
                }
                work = napi_poll(napi, budget);
                    --->
                trace_napi_poll(napi, work, budget);
                gro_normal_list(napi);
        count:
                if (work > 0)
                    __NET_ADD_STATS(dev_net(napi->dev), LINUX_MIB_BUSYPOLLRXPACKETS, work);
                skb_defer_free_flush(this_cpu_ptr(&softnet_data));
                bpf_net_ctx_clear(bpf_net_ctx);
                local_bh_enable();

                if (!loop_end || loop_end(loop_end_arg, start_time))
                    break;

                if (unlikely(need_resched())) {
                    if (flags & NAPI_F_END_ON_RESCHED)
                        break;
                    if (napi_poll)
                        busy_poll_stop(napi, have_poll_lock, flags, budget);
                    if (!IS_ENABLED(CONFIG_PREEMPT_RT))
                        preempt_enable();
                    rcu_read_unlock();
                    cond_resched();
                    rcu_read_lock();
                    if (loop_end(loop_end_arg, start_time))
                        return;
                    goto restart;
                }
                cpu_relax();
            }
            if (napi_poll)
                busy_poll_stop(napi, have_poll_lock, flags, budget);
            if (!IS_ENABLED(CONFIG_PREEMPT_RT))
                preempt_enable();
        }

        if (ep_events_available(ep))
            return true;

        if (prefer_busy_poll) {
            napi_resume_irqs(napi_id) {
                struct napi_struct *napi;

                rcu_read_lock();
                napi = napi_by_id(napi_id);
                if (napi) {
                    if (napi_get_irq_suspend_timeout(napi)) {
                        local_bh_disable();
                        napi_schedule(napi);
                        local_bh_enable();
                    }
                }
                rcu_read_unlock();
            }
        }
        ep->napi_id = 0;
        return false;
    }
    return false;
}
```

### ep_send_events

```c
int ep_send_events(struct eventpoll *ep,
        struct epoll_event __user *events, int maxevents)
{
    struct epitem *epi, *tmp;
    LIST_HEAD(txlist);
    poll_table pt;
    int res = 0;

    if (fatal_signal_pending(current))
        return -EINTR;

    /* ep_item_poll will add epi into ep wait queue if pt has _pqproc,
    * since epi has already added in ep_insert, we just check ready events
    * here, so set _qproc to NULL */
    init_poll_funcptr(&pt, NULL);

    mutex_lock(&ep->mtx);
    ep_start_scan(ep, &txlist) {
        list_splice_init(&ep->rdllist, txlist);
        WRITE_ONCE(ep->ovflist, NULL);
    }

    list_for_each_entry_safe(epi, tmp, &txlist, rdllink) {
        struct wakeup_source *ws;
        __poll_t revents;

        if (res >= maxevents)
            break;

        ws = ep_wakeup_source(epi);
        if (ws) {
            if (ws->active)
                __pm_stay_awake(ep->ws);
            __pm_relax(ws);
        }

        list_del_init(&epi->rdllink);

        revents = ep_item_poll(epi, &pt, 1);
            --->
        if (!revents)
            continue;

        events = epoll_put_uevent(revents, epi->event.data, events);
        if (!events) {
            list_add(&epi->rdllink, &txlist);
            ep_pm_stay_awake(epi);
            if (!res)
                res = -EFAULT;
            break;
        }
        res++;
        if (epi->event.events & EPOLLONESHOT)
            epi->event.events &= EP_PRIVATE_BITS;
        else if (!(epi->event.events & EPOLLET)) {
            list_add_tail(&epi->rdllink, &ep->rdllist);
            ep_pm_stay_awake(epi);
        }
    }
    ep_done_scan(ep, &txlist);
    mutex_unlock(&ep->mtx);

    return res;
}

/* splice ovflist into rdlist */
void ep_done_scan(struct eventpoll *ep, struct list_head *txlist)
{
    struct epitem *epi, *nepi;

    write_lock_irq(&ep->lock);

    for (nepi = READ_ONCE(ep->ovflist); (epi = nepi) != NULL;
        nepi = epi->next, epi->next = EP_UNACTIVE_PTR)
    {
        if (!ep_is_linked(epi)) {
            list_add(&epi->rdllink, &ep->rdllist);
            ep_pm_stay_awake(epi);
        }
    }

    WRITE_ONCE(ep->ovflist, EP_UNACTIVE_PTR);

    list_splice(txlist, &ep->rdllist);
    __pm_relax(ep->ws);

    if (!list_empty(&ep->rdllist)) {
        if (waitqueue_active(&ep->wq)) {
            wake_up(&ep->wq);
        }
    }

    write_unlock_irq(&ep->lock);
}
```


### ep_item_poll

```c
__poll_t ep_item_poll(const struct epitem *epi, poll_table *pt, int depth)
{
    struct file *file = epi->ffd.file;
    __poll_t res;

    pt->_key = epi->event.events;
    if (!is_file_epoll(file)) /* return f->f_op == &eventpoll_fops; epoll of epoll */
        res = vfs_poll(file, pt);
    else
        res = __ep_eventpoll_poll(file, pt, depth);
    return res & epi->event.events;
}

__poll_t vfs_poll(struct file *file, struct poll_table_struct *pt)
{
    if (unlikely(!file->f_op->poll))
        return DEFAULT_POLLMASK;
    return file->f_op->poll(file, pt); /* sock_poll, eventfd_poll */
}

__poll_t sock_poll(struct file *file, poll_table *wait)
{
    struct socket *sock = file->private_data;
    __poll_t events = poll_requested_events(wait), flag = 0;

    if (!sock->ops->poll)
        return 0;

    if (sk_can_busy_loop(sock->sk)) {
        /* poll once if requested by the syscall */
        if (events & POLL_BUSY_LOOP)
        sk_busy_loop(sock->sk, 1);

        /* if this socket can poll_ll, tell the system call */
        flag = POLL_BUSY_LOOP;
    }

    return sock->ops->poll(file, sock, wait) | flag;
}

/* inet_stream_ops.poll */
__poll_t tcp_poll(struct file *file, struct socket *sock, poll_table *wait)
{
    __poll_t mask;
    struct sock *sk = sock->sk;
    const struct tcp_sock *tp = tcp_sk(sk);
    int state;

    /* 1. register observer */
    sock_poll_wait(file, sock, wait);

    state = inet_sk_state_load(sk);
    if (state == TCP_LISTEN)
        return inet_csk_listen_poll(sk) {
            return !reqsk_queue_empty(&inet_csk(sk)->icsk_accept_queue) ? (EPOLLIN | EPOLLRDNORM) : 0;
        }

    mask = 0;

    if (sk->sk_shutdown == SHUTDOWN_MASK || state == TCP_CLOSE)
        mask |= EPOLLHUP;
    /* there might still be data in the receive buffer (sk->sk_receive_queue)
     * that hasn’t been read yet. Reporting EPOLLIN ensures the application
     * can drain this data before hitting EOF (read returns 0). */
    if (sk->sk_shutdown & RCV_SHUTDOWN)
        mask |= EPOLLIN | EPOLLRDNORM | EPOLLRDHUP;

    /* 2. poll ready events */
    if (state != TCP_SYN_SENT && (state != TCP_SYN_RECV || tp->fastopen_rsk)) {
        int target = sock_rcvlowat(sk, 0, INT_MAX) {
            int v = waitall ? len : min_t(int, READ_ONCE(sk->sk_rcvlowat), len);
            return v ?: 1;
        }

        if (tp->urg_seq == tp->copied_seq &&
            !sock_flag(sk, SOCK_URGINLINE) &&
            tp->urg_data)
            target++;

        ret = tcp_stream_is_readable(tp, target, sk) {
            ret = tcp_epollin_ready(sk, target) {
                const struct tcp_sock *tp = tcp_sk(sk);
                int avail = READ_ONCE(tp->rcv_nxt) - READ_ONCE(tp->copied_seq);

                if (avail <= 0)
                    return false;

                return (avail >= target) || tcp_rmem_pressure(sk) ||
                    (tcp_receive_window(tp) <= inet_csk(sk)->icsk_ack.rcv_mss);
            }
            if (ret)
                return true;
            return sk_is_readable(sk);
        }
        if (ret)
            mask |= EPOLLIN | EPOLLRDNORM;

        if (!(sk->sk_shutdown & SEND_SHUTDOWN)) {
            ret = sk_stream_is_writeable(sk) {
                wspace = sk_stream_wspace(sk) {
                    return READ_ONCE(sk->sk_sndbuf) - READ_ONCE(sk->sk_wmem_queued);
                }
                min = sk_stream_min_wspace(sk) {
                    return READ_ONCE(sk->sk_wmem_queued) >> 1;
                }
                return wspace >= min && __sk_stream_memory_free(sk, wake);
            }

            if (ret) {
                mask |= EPOLLOUT | EPOLLWRNORM;
            } else {  /* send SIGIO later */
                sk_set_bit(SOCKWQ_ASYNC_NOSPACE, sk);
                set_bit(SOCK_NOSPACE, &sk->sk_socket->flags);
                smp_mb__after_atomic();
                if (sk_stream_is_writeable(sk))
                    mask |= EPOLLOUT | EPOLLWRNORM;
            }
        } else {
            /* Reported despite writes failing, likely for compatibility
             * or to signal the shutdown state. */
            mask |= EPOLLOUT | EPOLLWRNORM;
        }

        if (tp->urg_data & TCP_URG_VALID)
            mask |= EPOLLPRI;
    } else if (state == TCP_SYN_SENT && inet_sk(sk)->defer_connect) {
        mask |= EPOLLOUT | EPOLLWRNORM;
    }

    smp_rmb();
    if (sk->sk_err || !skb_queue_empty_lockless(&sk->sk_error_queue))
        mask |= EPOLLERR;

    return mask;
}

void sock_poll_wait(struct file *filp, struct socket *sock, poll_table *p)
{
    if (!poll_does_not_wait(p)) {
        poll_wait(filp, &sock->wq->wait, p) {
            if (p && p->_qproc && wait_address) {
                /* ep_ptable_queue_proc      virqfd_ptable_queue_proc
                 * irqfd_ptable_queue_proc   memcg_event_ptable_queue_proc */
                p->_qproc(filp, wait_address, p);
            }
        }
        smp_mb();
    }
}

/* add our wait queue to the target file wakeup lists.
 *
 * Since different file implementation will put the wait queue head in
 * completely different locations, there is no way we could locate the
 * correct wait_queue_head_t without the use of the callback. */
void ep_ptable_queue_proc(
  struct file *file, wait_queue_head_t *whead, poll_table *pt)
{
    struct epitem *epi = ep_item_from_epqueue(pt);
    struct eppoll_entry *pwq;

    if (epi->nwait >= 0 && (pwq = kmem_cache_alloc(pwq_cache, GFP_KERNEL))) {
        init_waitqueue_func_entry(&pwq->wait, ep_poll_callback);
        pwq->whead = whead;
        pwq->base = epi;
        if (epi->event.events & EPOLLEXCLUSIVE)
            add_wait_queue_exclusive(whead, &pwq->wait) {
                wq_entry->flags |= WQ_FLAG_EXCLUSIVE;
                __add_wait_queue_entry_tail(wq_head, wq_entry);
            }
        else
            add_wait_queue(whead, &pwq->wait);
        list_add_tail(&pwq->llink, &epi->pwqlist);
        epi->nwait++;
    } else {
        /* We have to signal that an error occurred */
        epi->nwait = -1;
    }
}

/* Wait structure used by the poll hooks */
struct eppoll_entry {
    /* List header used to link this structure to the "struct epitem" */
    struct eppoll_entry *next;

    /* The "base" pointer is set to the container "struct epitem" */
    struct epitem *base;

    /* Wait queue item that will be linked to the target file wait
    * queue head. */
    wait_queue_entry_t wait;

    /* The wait queue head that linked the "wait" wait queue item */
    wait_queue_head_t *whead;
};

struct wait_queue_entry {
    unsigned int        flags;
    void                *private;
    wait_queue_func_t   func;
    struct list_head    entry;
};
```

## ep_poll_callback

```c
/* tcp_rcv_established -> tcp_data_queue */
void tcp_data_ready(struct sock *sk)
{
    const struct tcp_sock *tp = tcp_sk(sk);
    int avail = tp->rcv_nxt - tp->copied_seq;

    if (avail < sk->sk_rcvlowat && !tcp_rmem_pressure(sk) && !sock_flag(sk, SOCK_DONE))
        return;

    sk->sk_data_ready(sk); /* sock_def_readable */
}

void sock_def_readable(struct sock *sk)
{
    struct socket_wq *wq;

    rcu_read_lock();
    wq = rcu_dereference(sk->sk_wq);
    if (skwq_has_sleeper(wq))
        wake_up_interruptible_sync_poll(
            &wq->wait, EPOLLIN | EPOLLPRI | EPOLLRDNORM | EPOLLRDBAND);
    sk_wake_async(sk, SOCK_WAKE_WAITD, POLL_IN);
    rcu_read_unlock();
}

#define wake_up_interruptible_sync_poll(x, m)          \
    __wake_up_sync_key((x), TASK_INTERRUPTIBLE, 1, poll_to_key(m))

void __wake_up_sync_key(struct wait_queue_head *wq_head, unsigned int mode,
      int nr_exclusive, void *key)
{
    __wake_up_common_lock(wq_head, mode, nr_exclusive, wake_flags, key);
}

/* This is the callback that is passed to the wait queue wakeup
 * mechanism. It is called by the stored file descriptors when they
 * have events to report. */
int ep_poll_callback(
    wait_queue_entry_t *wait, unsigned mode, int sync, void *key)
{
    int pwake = 0;
    unsigned long flags;
    struct epitem *epi = ep_item_from_wait(wait);
    struct eventpoll *ep = epi->ep;
    __poll_t pollflags = key_to_poll(key);
    int ewake = 0;

    ep_set_busy_poll_napi_id(epi) {
        sock = sock_from_file(epi->ffd.file);
        sk = sock->sk;
        napi_id = READ_ONCE(sk->sk_napi_id);

        if (napi_id < MIN_NAPI_ID || napi_id == ep->napi_id)
            return;

        /* record NAPI ID for use in next busy poll */
        ep->napi_id = napi_id;
    }

    /* If the event mask does not contain any poll(2) event, we consider the
    * descriptor to be disabled. This condition is likely the effect of the
    * EPOLLONESHOT bit that disables the descriptor when an event is received,
    * until the next EPOLL_CTL_MOD will be issued. */
    if (!(epi->event.events & ~EP_PRIVATE_BITS))
        goto out_unlock;

    /* doesn't have events we are intrested */
    if (pollflags && !(pollflags & epi->event.events))
        goto out_unlock;

    /* #define EP_UNACTIVE_PTR ((void *) -1L) */
    if (READ_ONCE(ep->ovflist) != EP_UNACTIVE_PTR) {
        if (chain_epi_lockless(epi))
            ep_pm_stay_awake_rcu(epi);
    } else if (!ep_is_linked(epi)) {
        /* In the usual case, add event to ready list. */
        if (list_add_tail_lockless(&epi->rdllink, &ep->rdllist))
            ep_pm_stay_awake_rcu(epi);
    }

    /* Wake up ( if active ) both the eventpoll wait list and the ->poll()
     * wait list. */
    if (waitqueue_active(&ep->wq)) {
        if ((epi->event.events & EPOLLEXCLUSIVE) && !(pollflags & POLLFREE)) {
            switch (pollflags & EPOLLINOUT_BITS) {
            case EPOLLIN:
                if (epi->event.events & EPOLLIN)
                    ewake = 1;
                break;
            case EPOLLOUT:
                if (epi->event.events & EPOLLOUT)
                    ewake = 1;
                break;
            case 0:
                ewake = 1;
                break;
            }
        }
        /* wake up epoll_wait, exclusive 1 */
        if (sync) {
            wake_up_sync(&ep->wq);
        } else {
            wake_up(&ep->wq);
        }
    }

    if (waitqueue_active(&ep->poll_wait))
        pwake++;

out_unlock:
    spin_unlock_irqrestore(&ep->wq.lock, flags);

    /* Nested epoll Support */
    if (pwake)
        ep_poll_safewake(&ep->poll_wait);

    if (!(epi->event.events & EPOLLEXCLUSIVE))
        ewake = 1;

    if (pollflags & POLLFREE) {
        list_del_init(&wait->entry);
        smp_store_release(&ep_pwq_from_wait(wait)->whead, NULL);
    }

    return ewake;
}

void __wake_up_locked(
  struct wait_queue_head *wq_head, unsigned int mode, int nr)
{
    __wake_up_common(wq_head, mode, nr, 0, NULL, NULL);
}
```

* [__wake_up_common](./linux-proc.md#wake_up)

## sock_def_write_space

```c
/* tcp_rcv_established ->
 * tcp_rcv_state_process -> */
void tcp_data_snd_check(struct sock *sk)
{
    tcp_push_pending_frames(sk);
    tcp_check_space(sk);
}

/* tcp_event_new_data_sent -> */
void tcp_check_space(struct sock *sk)
{
    smp_mb();
    if (sk->sk_socket && test_bit(SOCK_NOSPACE, &sk->sk_socket->flags)) {
        tcp_new_space(sk) {
            struct tcp_sock *tp = tcp_sk(sk);

            if (tcp_should_expand_sndbuf(sk)) {
                tcp_sndbuf_expand(sk);
                tp->snd_cwnd_stamp = tcp_jiffies32;
            }

            sk->sk_write_space(sk) = sock_def_write_space(struct sock *sk) {
                struct socket_wq *wq;

                rcu_read_lock();

                /* Do not wake up a writer until he can make "significant"
                * progress.  --DaveM */
                if ((refcount_read(&sk->sk_wmem_alloc) << 1) <= sk->sk_sndbuf) {
                    wq = rcu_dereference(sk->sk_wq);
                    if (skwq_has_sleeper(wq))
                        wake_up_interruptible_sync_poll(&wq->wait, EPOLLOUT | EPOLLWRNORM | EPOLLWRBAND);

                    /* Should agree with poll, otherwise some programs break */
                    if (sock_writeable(sk)) /* sk->sk_wmem_alloc < (sk->sk_sndbuf >> 1) */
                        sk_wake_async(sk, SOCK_WAKE_SPACE, POLL_OUT);
                }

                rcu_read_unlock();
            }
        }
        if (!test_bit(SOCK_NOSPACE, &sk->sk_socket->flags)) {
            tcp_chrono_stop(sk, TCP_CHRONO_SNDBUF_LIMITED);
        }
    }
}
```

## evetfd

```c
static const struct file_operations eventfd_fops = {
    .release    = eventfd_release,
    .poll       = eventfd_poll,
    .read_iter  = eventfd_read,
    .write      = eventfd_write,
    .llseek     = noop_llseek,
};
```

```c
SYSCALL_DEFINE2(eventfd2, unsigned int, count, int, flags)
{
    return do_eventfd(count, flags);
}

int do_eventfd(unsigned int count, int flags)
{
  struct eventfd_ctx *ctx;
  struct file *file;
  int fd;

  ctx = kmalloc(sizeof(*ctx), GFP_KERNEL);
  if (!ctx)
    return -ENOMEM;

  kref_init(&ctx->kref);
  init_waitqueue_head(&ctx->wqh);
  ctx->count = count;
  ctx->flags = flags;
  ctx->id = ida_simple_get(&eventfd_ida, 0, 0, GFP_KERNEL);

  flags &= EFD_SHARED_FCNTL_FLAGS;
  flags |= O_RDWR;
  fd = get_unused_fd_flags(flags);
  if (fd < 0)
    goto err;

  file = anon_inode_getfile("[eventfd]", &eventfd_fops, ctx, flags);
  if (IS_ERR(file)) {
    put_unused_fd(fd);
    fd = PTR_ERR(file);
    goto err;
  }

  file->f_mode |= FMODE_NOWAIT;
  fd_install(fd, file);
  return fd;
err:
  eventfd_free_ctx(ctx);
  return fd;
}

struct eventfd_ctx {
  struct kref         kref;
  wait_queue_head_t   wqh;
  __u64               count;
  unsigned int        flags;
  int                 id;
};

static ssize_t eventfd_read(struct kiocb *iocb, struct iov_iter *to)
{
  struct file *file = iocb->ki_filp;
  struct eventfd_ctx *ctx = file->private_data;
  __u64 ucnt = 0;
  DECLARE_WAITQUEUE(wait, current); /* default_wake_function */

  if (iov_iter_count(to) < sizeof(ucnt))
    return -EINVAL;

  spin_lock_irq(&ctx->wqh.lock);
  if (!ctx->count) {
    if ((file->f_flags & O_NONBLOCK) || (iocb->ki_flags & IOCB_NOWAIT)) {
      spin_unlock_irq(&ctx->wqh.lock);
      return -EAGAIN;
    }

    __add_wait_queue(&ctx->wqh, &wait);

    for (;;) {
      set_current_state(TASK_INTERRUPTIBLE);
      if (ctx->count)
        break;
      if (signal_pending(current)) {
        __remove_wait_queue(&ctx->wqh, &wait);
        __set_current_state(TASK_RUNNING);
        spin_unlock_irq(&ctx->wqh.lock);
        return -ERESTARTSYS;
      }
      spin_unlock_irq(&ctx->wqh.lock);
      schedule();
      spin_lock_irq(&ctx->wqh.lock);
    }
    __remove_wait_queue(&ctx->wqh, &wait);
    __set_current_state(TASK_RUNNING);
  }
  eventfd_ctx_do_read(ctx, &ucnt);

  if (waitqueue_active(&ctx->wqh))
    wake_up_locked_poll(&ctx->wqh, EPOLLOUT);

  spin_unlock_irq(&ctx->wqh.lock);
  if (unlikely(copy_to_iter(&ucnt, sizeof(ucnt), to) != sizeof(ucnt)))
    return -EFAULT;

  return sizeof(ucnt);
}

void eventfd_ctx_do_read(struct eventfd_ctx *ctx, __u64 *cnt)
{
  lockdep_assert_held(&ctx->wqh.lock);

  *cnt = (ctx->flags & EFD_SEMAPHORE) ? 1 : ctx->count;
  ctx->count -= *cnt;
}

static __poll_t eventfd_poll(struct file *file, poll_table *wait)
{
  struct eventfd_ctx *ctx = file->private_data;
  __poll_t events = 0;
  u64 count;

  poll_wait(file, &ctx->wqh, wait);

  count = READ_ONCE(ctx->count);

  if (count > 0)
    events |= EPOLLIN;
  if (count == ULLONG_MAX)
    events |= EPOLLERR;
  if (ULLONG_MAX - 1 > count)
    events |= EPOLLOUT;

  return events;
}

static ssize_t eventfd_write(struct file *file, const char __user *buf, size_t count,
           loff_t *ppos)
{
  struct eventfd_ctx *ctx = file->private_data;
  ssize_t res;
  __u64 ucnt;
  DECLARE_WAITQUEUE(wait, current);

  if (count < sizeof(ucnt))
    return -EINVAL;
  if (copy_from_user(&ucnt, buf, sizeof(ucnt)))
    return -EFAULT;
  if (ucnt == ULLONG_MAX)
    return -EINVAL;

  spin_lock_irq(&ctx->wqh.lock);
  res = -EAGAIN;
  if (ULLONG_MAX - ctx->count > ucnt)
    res = sizeof(ucnt);
  else if (!(file->f_flags & O_NONBLOCK)) {
    __add_wait_queue(&ctx->wqh, &wait);
    for (res = 0;;) {
      set_current_state(TASK_INTERRUPTIBLE);
      if (ULLONG_MAX - ctx->count > ucnt) {
        res = sizeof(ucnt);
        break;
      }
      if (signal_pending(current)) {
        res = -ERESTARTSYS;
        break;
      }
      spin_unlock_irq(&ctx->wqh.lock);
      schedule();
      spin_lock_irq(&ctx->wqh.lock);
    }
    __remove_wait_queue(&ctx->wqh, &wait);
    __set_current_state(TASK_RUNNING);
  }

  if (likely(res > 0)) {
    ctx->count += ucnt;
    if (waitqueue_active(&ctx->wqh))
      wake_up_locked_poll(&ctx->wqh, EPOLLIN);
  }
  spin_unlock_irq(&ctx->wqh.lock);

  return res;
}
```

## signalfd

Will a User-Registered Signal Handler Be Triggered with signalfd Set?
* The short answer is no, a user-registered signal handler will not be triggered for signals handled by signalfd, provided those signals are blocked as part of the signalfd setup. Here’s why:

When you set up signalfd, you typically block the signals you want it to handle:

```c
sigset_t mask;
sigemptyset(&mask);
sigaddset(&mask, SIGINT);
sigprocmask(SIG_BLOCK, &mask, NULL); // Block SIGINT
int sfd = signalfd(-1, &mask, 0);
```

A blocked signal is not delivered to the process’s signal handler. Instead, it remains queued until it’s handled (e.g., read from the signalfd file descriptor) or unblocked.

```c
static const struct file_operations signalfd_fops = {
    .show_fdinfo    = signalfd_show_fdinfo,
    .release        = signalfd_release,
    .poll           = signalfd_poll,
    .read_iter      = signalfd_read_iter,
    .llseek         = noop_llseek,
};
```

### do_signalfd

```c
static int do_signalfd4(int ufd, sigset_t *mask, int flags)
{
    struct signalfd_ctx *ctx;

    /* Check the SFD_* constants for consistency.  */
    BUILD_BUG_ON(SFD_CLOEXEC != O_CLOEXEC);
    BUILD_BUG_ON(SFD_NONBLOCK != O_NONBLOCK);

    if (flags & ~(SFD_CLOEXEC | SFD_NONBLOCK))
        return -EINVAL;

    sigdelsetmask(mask, sigmask(SIGKILL) | sigmask(SIGSTOP));
    signotset(mask);

    if (ufd == -1) {
        struct file *file;

        ctx = kmalloc(sizeof(*ctx), GFP_KERNEL);
        if (!ctx)
            return -ENOMEM;

        ctx->sigmask = *mask;

        ufd = get_unused_fd_flags(flags & O_CLOEXEC);
        if (ufd < 0) {
            kfree(ctx);
            return ufd;
        }

        file = anon_inode_getfile("[signalfd]", &signalfd_fops, ctx,
                    O_RDWR | (flags & O_NONBLOCK));
        if (IS_ERR(file)) {
            put_unused_fd(ufd);
            kfree(ctx);
            return PTR_ERR(file);
        }
        file->f_mode |= FMODE_NOWAIT;

        fd_install(ufd, file);
    } else {
        CLASS(fd, f)(ufd);
        if (fd_empty(f))
            return -EBADF;
        ctx = fd_file(f)->private_data;
        if (fd_file(f)->f_op != &signalfd_fops)
            return -EINVAL;
        spin_lock_irq(&current->sighand->siglock);
        ctx->sigmask = *mask;
        spin_unlock_irq(&current->sighand->siglock);

        wake_up(&current->sighand->signalfd_wqh);
    }

    return ufd;
}
```

### signalfd_notify

```c
do_send_signal_info() {
    /* wake up signalfd_wqh when signal arrives */
    signalfd_notify(t, sig) {
        if (unlikely(waitqueue_active(&tsk->sighand->signalfd_wqh))) {
            wake_up(&tsk->sighand->signalfd_wqh);
        }
    }
}
```

```c
/* signalfd_notify -> ep_poll_callback -> epoll_wait -> ep_item_poll */
static __poll_t signalfd_poll(struct file *file, poll_table *wait)
{
    struct signalfd_ctx *ctx = file->private_data;
    __poll_t events = 0;

    poll_wait(file, &current->sighand->signalfd_wqh, wait);

    spin_lock_irq(&current->sighand->siglock);
    if (next_signal(&current->pending, &ctx->sigmask) ||
        next_signal(&current->signal->shared_pending, &ctx->sigmask))
        events |= EPOLLIN;
    spin_unlock_irq(&current->sighand->siglock);

    return events;
}
```

### signalfd_read_iter

```c
static ssize_t signalfd_read_iter(struct kiocb *iocb, struct iov_iter *to)
{
    struct file *file = iocb->ki_filp;
    struct signalfd_ctx *ctx = file->private_data;
    size_t count = iov_iter_count(to);
    ssize_t ret, total = 0;
    kernel_siginfo_t info;
    bool nonblock;

    count /= sizeof(struct signalfd_siginfo);
    if (!count)
        return -EINVAL;

    nonblock = file->f_flags & O_NONBLOCK || iocb->ki_flags & IOCB_NOWAIT;
    do {
        ret = signalfd_dequeue(ctx, &info, nonblock) {
            enum pid_type type;
            ssize_t ret;
            DECLARE_WAITQUEUE(wait, current);

            spin_lock_irq(&current->sighand->siglock);
            ret = dequeue_signal(&ctx->sigmask, info, &type);
            switch (ret) {
            case 0:
                if (!nonblock)
                    break;
                ret = -EAGAIN;
                fallthrough;
            default:
                spin_unlock_irq(&current->sighand->siglock);
                return ret;
            }

            /* block IO */

            add_wait_queue(&current->sighand->signalfd_wqh, &wait);

            for (;;) {
                set_current_state(TASK_INTERRUPTIBLE);
                ret = dequeue_signal(&ctx->sigmask, info, &type);
                if (ret != 0)
                    break;
                if (signal_pending(current)) {
                    ret = -ERESTARTSYS;
                    break;
                }
                spin_unlock_irq(&current->sighand->siglock);
                schedule();
                spin_lock_irq(&current->sighand->siglock);
            }
            spin_unlock_irq(&current->sighand->siglock);

            remove_wait_queue(&current->sighand->signalfd_wqh, &wait);
            __set_current_state(TASK_RUNNING);

            return ret;
        }
        if (unlikely(ret <= 0))
            break;

        ret = signalfd_copyinfo(to, &info);
        if (ret < 0)
            break;
        total += ret;
        nonblock = 1;
    } while (--count);

    return total ? total: ret;
}
```

## eventpoll_init

```c
int __init eventpoll_init(void)
{
    struct sysinfo si;

    si_meminfo(&si);

    max_user_watches = (((si.totalram - si.totalhigh) / 25) << PAGE_SHIFT) / EP_ITEM_COST;

    /* Allocates slab cache used to allocate "struct epitem" items */
    epi_cache = kmem_cache_create("eventpoll_epi", sizeof(struct epitem),
        0, SLAB_HWCACHE_ALIGN|SLAB_PANIC|SLAB_ACCOUNT, NULL);

    /* Allocates slab cache used to allocate "struct eppoll_entry" */
    pwq_cache = kmem_cache_create("eventpoll_pwq",
        sizeof(struct eppoll_entry), 0, SLAB_PANIC|SLAB_ACCOUNT, NULL);
    epoll_sysctls_init();

    ephead_cache = kmem_cache_create("ep_head",
        sizeof(struct epitems_head), 0, SLAB_PANIC|SLAB_ACCOUNT, NULL);

    return 0;
}
```

## Reference:
* [Segmentation Offloads](https://www.kernel.org/doc/html/latest/networking/segmentation-offloads.html)
* Epoll is fundamentally broken [:link: Part 1 ](https://idea.popcount.org/2017-02-20-epoll-is-fundamentally-broken-12) [:link: Part 2 ](https://idea.popcount.org/2017-03-20-epoll-is-fundamentally-broken-22)
* The implementation of epoll [:link: Part 1 ](https://idndx.com/the-implementation-of-epoll-1/) [:link: Part 2 ](https://idndx.com/the-implementation-of-epoll-2/) [:link: Part 3 ](https://idndx.com/the-implementation-of-epoll-3/) [:link: Part 4 ](https://idndx.com/the-implementation-of-epoll-4/)

## Q:
1. Alloc 0 sized sk_buff in `sk_stream_alloc_skb` from `tcp_sendmsg_locked` when there is not enough space for the new data?
2. The segementation deails in tcp and ip layers?
3. What's the tcp header and data memory layout?
4. When rcv, how does mutil-core cpu work?
5. How does napi works, What does net card do when cpu polling?
6. Where does net driver put new data in when cpu polling?

## TODO:
1. Trie tree implementaion to in FIB

# select

```c
typedef struct {
    unsigned long fds_bits[__FD_SETSIZE / (8 * sizeof(long))];
} __kernel_fd_set;

typedef __kernel_fd_set     fd_set;

SYSCALL_DEFINE5(select, int, n, fd_set __user *, inp, fd_set __user *, outp,
        fd_set __user *, exp, struct __kernel_old_timeval __user *, tvp)
{
    return kern_select(n, inp, outp, exp, tvp) {
        struct timespec64 end_time, *to = NULL;
        struct __kernel_old_timeval tv;
        int ret;

        if (tvp) {
            if (copy_from_user(&tv, tvp, sizeof(tv)))
                return -EFAULT;

            to = &end_time;
            if (poll_select_set_timeout(to,
                    tv.tv_sec + (tv.tv_usec / USEC_PER_SEC),
                    (tv.tv_usec % USEC_PER_SEC) * NSEC_PER_USEC))
                return -EINVAL;
        }

        ret = core_sys_select(n, inp, outp, exp, to) {
            fd_set_bits {
                unsigned long *in, *out, *ex;
                unsigned long *res_in, *res_out, *res_ex;
            } fds;

            void *bits;
            int ret, max_fds;
            size_t size, alloc_size;
            struct fdtable *fdt;
            /* Allocate small arguments on the stack to save memory and be faster */
            long stack_fds[SELECT_STACK_ALLOC/sizeof(long)];

            ret = -EINVAL;
            if (n < 0)
                goto out_nofds;

            /* max_fds can increase, so grab it once to avoid race */
            rcu_read_lock();
            fdt = files_fdtable(current->files);
            max_fds = fdt->max_fds;
            rcu_read_unlock();
            if (n > max_fds)
                n = max_fds;

            /* We need 6 bitmaps (in/out/ex for both incoming and outgoing),
            * since we used fdset we need to allocate memory in units of
            * long-words. */
            size = FDS_BYTES(n);
            bits = stack_fds;
            if (size > sizeof(stack_fds) / 6) {
                /* Not enough space in on-stack array; must use kmalloc */
                ret = -ENOMEM;
                if (size > (SIZE_MAX / 6))
                    goto out_nofds;

                alloc_size = 6 * size;
                bits = kvmalloc(alloc_size, GFP_KERNEL);
                if (!bits)
                    goto out_nofds;
            }

            fds.in      = bits;
            fds.out     = bits +   size;
            fds.ex      = bits + 2*size;
            fds.res_in  = bits + 3*size;
            fds.res_out = bits + 4*size;
            fds.res_ex  = bits + 5*size;

            if ((ret = get_fd_set(n, inp, fds.in)) ||
                (ret = get_fd_set(n, outp, fds.out)) ||
                (ret = get_fd_set(n, exp, fds.ex)))
                goto out;

            zero_fd_set(n, fds.res_in);
            zero_fd_set(n, fds.res_out);
            zero_fd_set(n, fds.res_ex);

            ret = do_select(n, &fds, end_time);
                --->

            if (ret < 0)
                goto out;
            if (!ret) {
                ret = -ERESTARTNOHAND;
                if (signal_pending(current))
                    goto out;
                ret = 0;
            }

            if (set_fd_set(n, inp, fds.res_in) ||
                set_fd_set(n, outp, fds.res_out) ||
                set_fd_set(n, exp, fds.res_ex))
                ret = -EFAULT;

        out:
            if (bits != stack_fds)
                kvfree(bits);
        out_nofds:
            return ret;
        }

        return poll_select_finish(&end_time, tvp, PT_TIMEVAL, ret) {
            struct timespec64 rts;

            restore_saved_sigmask_unless(ret == -ERESTARTNOHAND);

            if (!p)
                return ret;

            if (current->personality & STICKY_TIMEOUTS)
                goto sticky;

            /* No update for zero timeout */
            if (!end_time->tv_sec && !end_time->tv_nsec)
                return ret;

            ktime_get_ts64(&rts);
            rts = timespec64_sub(*end_time, rts);
            if (rts.tv_sec < 0)
                rts.tv_sec = rts.tv_nsec = 0;


            switch (pt_type) {
            case PT_TIMEVAL:
                {
                    struct __kernel_old_timeval rtv;

                    if (sizeof(rtv) > sizeof(rtv.tv_sec) + sizeof(rtv.tv_usec))
                        memset(&rtv, 0, sizeof(rtv));
                    rtv.tv_sec = rts.tv_sec;
                    rtv.tv_usec = rts.tv_nsec / NSEC_PER_USEC;
                    if (!copy_to_user(p, &rtv, sizeof(rtv)))
                        return ret;
                }
                break;
            case PT_OLD_TIMEVAL:
                {
                    struct old_timeval32 rtv;

                    rtv.tv_sec = rts.tv_sec;
                    rtv.tv_usec = rts.tv_nsec / NSEC_PER_USEC;
                    if (!copy_to_user(p, &rtv, sizeof(rtv)))
                        return ret;
                }
                break;
            case PT_TIMESPEC:
                if (!put_timespec64(&rts, p))
                    return ret;
                break;
            case PT_OLD_TIMESPEC:
                if (!put_old_timespec32(&rts, p))
                    return ret;
                break;
            default:
                BUG();
            }
            /* If an application puts its timeval in read-only memory, we
            * don't want the Linux-specific update to the timeval to
            * cause a fault after the select has completed
            * successfully. However, because we're not updating the
            * timeval, we can't restart the system call. */

        sticky:
            if (ret == -ERESTARTNOHAND)
                ret = -EINTR;
            return ret;
        }
    }
}
```

## do_select

```c
struct poll_wqueues {
    poll_table pt;
    struct poll_table_page *table;
    struct task_struct *polling_task;
    int triggered;
    int error;
    int inline_index;
    struct poll_table_entry inline_entries[N_INLINE_POLL_ENTRIES];
};

typedef struct poll_table_struct {
    poll_queue_proc _qproc;
    __poll_t _key;
} poll_table;

struct poll_table_page {
    struct poll_table_page * next;
    struct poll_table_entry * entry;
    struct poll_table_entry entries[];
};

struct poll_table_entry {
    struct file *filp;
    __poll_t key;
    wait_queue_entry_t wait;
    wait_queue_head_t *wait_address;
};
```

```c
int do_select(int n, fd_set_bits *fds, struct timespec64 *end_time)
{
    ktime_t expire, *to = NULL;

    struct poll_wqueues table;
    poll_table *wait;
    int retval, i, timed_out = 0;
    u64 slack = 0;
    __poll_t busy_flag = net_busy_loop_on() ? POLL_BUSY_LOOP : 0;
    unsigned long busy_start = 0;

    rcu_read_lock();
    retval = max_select_fd(n, fds);
    rcu_read_unlock();

    if (retval < 0)
        return retval;
    n = retval;

    poll_initwait(&table) {
        init_poll_funcptr(&pwq->pt,
            __pollwait = [](struct file *filp, wait_queue_head_t *wait_address, poll_table *p) {
                struct poll_wqueues *pwq = container_of(p, struct poll_wqueues, pt);
                struct poll_table_entry *entry = poll_get_entry(pwq) {
                    struct poll_table_page *table = p->table;
                    if (p->inline_index < N_INLINE_POLL_ENTRIES)
                        return p->inline_entries + p->inline_index++;

                    if (!table || POLL_TABLE_FULL(table)) {
                        struct poll_table_page *new_table;

                        new_table = (struct poll_table_page *) __get_free_page(GFP_KERNEL);
                        if (!new_table) {
                            p->error = -ENOMEM;
                            return NULL;
                        }
                        new_table->entry = new_table->entries;
                        new_table->next = table;
                        p->table = new_table;
                        table = new_table;
                    }

                    return table->entry++;
                }
                if (!entry)
                    return;

                entry->filp = get_file(filp);
                entry->wait_address = wait_address;
                entry->key = p->_key;

                init_waitqueue_func_entry(&entry->wait,
                    pollwake = [](wait_queue_entry_t *wait, unsigned mode, int sync, void *key) {
                        struct poll_table_entry *entry;

                        entry = container_of(wait, struct poll_table_entry, wait);
                        if (key && !(key_to_poll(key) & entry->key))
                            return 0;

                        return __pollwake(wait, mode, sync, key) {
                            struct poll_wqueues *pwq = wait->private;
                            DECLARE_WAITQUEUE(dummy_wait, pwq->polling_task);

                            smp_wmb();
                            pwq->triggered = 1;

                            return default_wake_function(&dummy_wait, mode, sync, key);
                        }
                    }
                );

                entry->wait.private = pwq;
                add_wait_queue(wait_address, &entry->wait);
            }
        );

        pwq->polling_task = current;
        pwq->triggered = 0;
        pwq->error = 0;
        pwq->table = NULL;
        pwq->inline_index = 0;
    }

    wait = &table.pt;
    if (end_time && !end_time->tv_sec && !end_time->tv_nsec) {
        wait->_qproc = NULL;
        timed_out = 1;
    }

    if (end_time && !timed_out)
        slack = select_estimate_accuracy(end_time);

    retval = 0;
    for (;;) {
        unsigned long *rinp, *routp, *rexp, *inp, *outp, *exp;
        bool can_busy_loop = false;

        inp = fds->in; outp = fds->out; exp = fds->ex;
        rinp = fds->res_in; routp = fds->res_out; rexp = fds->res_ex;

        for (i = 0; i < n; ++rinp, ++routp, ++rexp) {
            unsigned long in, out, ex, all_bits, bit = 1, j;
            unsigned long res_in = 0, res_out = 0, res_ex = 0;
            __poll_t mask;

            in = *inp++; out = *outp++; ex = *exp++;
            all_bits = in | out | ex;
            if (all_bits == 0) {
                i += BITS_PER_LONG;
                continue;
            }

            for (j = 0; j < BITS_PER_LONG; ++j, ++i, bit <<= 1) {
                if (i >= n)
                    break;
                if (!(bit & all_bits))
                    continue;

                mask = select_poll_one(i, wait, in, out, bit, busy_flag) {
                    CLASS(fd, f)(fd);

                    if (fd_empty(f))
                        return EPOLLNVAL;

                    wait->_key = POLLEX_SET | ll_flag;
                    if (in & bit)
                        wait->_key |= POLLIN_SET;
                    if (out & bit)
                        wait->_key |= POLLOUT_SET;

                    return vfs_poll(fd_file(f), wait);
                        --->
                }
                if ((mask & POLLIN_SET) && (in & bit)) {
                    res_in |= bit;
                    retval++;
                    wait->_qproc = NULL;
                }
                if ((mask & POLLOUT_SET) && (out & bit)) {
                    res_out |= bit;
                    retval++;
                    wait->_qproc = NULL;
                }
                if ((mask & POLLEX_SET) && (ex & bit)) {
                    res_ex |= bit;
                    retval++;
                    wait->_qproc = NULL;
                }
                /* got something, stop busy polling */
                if (retval) {
                    can_busy_loop = false;
                    busy_flag = 0;

                /* only remember a returned
                 * POLL_BUSY_LOOP if we asked for it */
                } else if (busy_flag & mask)
                    can_busy_loop = true;

            }
            if (res_in)
                *rinp = res_in;
            if (res_out)
                *routp = res_out;
            if (res_ex)
                *rexp = res_ex;
            cond_resched();
        }

        wait->_qproc = NULL;
        if (retval || timed_out || signal_pending(current))
            break;
        if (table.error) {
            retval = table.error;
            break;
        }

        /* only if found POLL_BUSY_LOOP sockets && not out of time */
        if (can_busy_loop && !need_resched()) {
            if (!busy_start) {
                busy_start = busy_loop_current_time();
                continue;
            }
            if (!busy_loop_timeout(busy_start))
                continue;
        }
        busy_flag = 0;

        /* If this is the first loop and we have a timeout
        * given, then we convert to ktime_t and set the to
        * pointer to the expiry value. */
        if (end_time && !to) {
            expire = timespec64_to_ktime(*end_time);
            to = &expire;
        }

        ret = poll_schedule_timeout(&table, TASK_INTERRUPTIBLE, to, slack) {
            int rc = -EINTR;

            set_current_state(state);
            if (!pwq->triggered)
                rc = schedule_hrtimeout_range(expires, slack, HRTIMER_MODE_ABS);
            __set_current_state(TASK_RUNNING);

            /* Prepare for the next iteration.
            *
            * The following smp_store_mb() serves two purposes.  First, it's
            * the counterpart rmb of the wmb in pollwake() such that data
            * written before wake up is always visible after wake up.
            * Second, the full barrier guarantees that triggered clearing
            * doesn't pass event check of the next iteration.  Note that
            * this problem doesn't exist for the first iteration as
            * add_wait_queue() has full barrier semantics. */
            smp_store_mb(pwq->triggered, 0);

            return rc;
        }

        if (!ret)
            timed_out = 1;
    }

    poll_freewait(&table);

    return retval;
}
```

# poll

```c
struct pollfd {
    int fd;
    short events;
    short revents;
};

struct poll_list {
    struct poll_list *next;
    unsigned int len;
    struct pollfd entries[] __counted_by(len);
};
```

```c
SYSCALL_DEFINE3(poll, struct pollfd __user *, ufds, unsigned int, nfds,
        int, timeout_msecs)
{
    struct timespec64 end_time, *to = NULL;
    int ret;

    if (timeout_msecs >= 0) {
        to = &end_time;
        poll_select_set_timeout(to, timeout_msecs / MSEC_PER_SEC,
            NSEC_PER_MSEC * (timeout_msecs % MSEC_PER_SEC));
    }

    ret = do_sys_poll(ufds, nfds, to) {
        struct poll_wqueues table;
        int err = -EFAULT, fdcount;
        /* Allocate small arguments on the stack to save memory and be
        faster - use long to make sure the buffer is aligned properly
        on 64 bit archs to avoid unaligned access */
        long stack_pps[POLL_STACK_ALLOC/sizeof(long)];
        struct poll_list *const head = (struct poll_list *)stack_pps;
        struct poll_list *walk = head;
        unsigned int todo = nfds;
        unsigned int len;

        if (nfds > rlimit(RLIMIT_NOFILE))
            return -EINVAL;

        len = min_t(unsigned int, nfds, N_STACK_PPS);
        for (;;) {
            walk->next = NULL;
            walk->len = len;
            if (!len)
                break;

            if (copy_from_user(walk->entries, ufds + nfds-todo, sizeof(struct pollfd) * walk->len))
                goto out_fds;

            if (walk->len >= todo)
                break;
            todo -= walk->len;

            len = min(todo, POLLFD_PER_PAGE);
            walk = walk->next = kmalloc(struct_size(walk, entries, len), GFP_KERNEL);
            if (!walk) {
                err = -ENOMEM;
                goto out_fds;
            }
        }

        poll_initwait(&table);
            --->
        fdcount = do_poll(head, &table, end_time);
        poll_freewait(&table);

        if (!user_write_access_begin(ufds, nfds * sizeof(*ufds)))
            goto out_fds;

        for (walk = head; walk; walk = walk->next) {
            struct pollfd *fds = walk->entries;
            unsigned int j;

            for (j = walk->len; j; fds++, ufds++, j--)
                unsafe_put_user(fds->revents, &ufds->revents, Efault);
        }
        user_write_access_end();

        err = fdcount;
    out_fds:
        walk = head->next;
        while (walk) {
            struct poll_list *pos = walk;
            walk = walk->next;
            kfree(pos);
        }

        return err;

    Efault:
        user_write_access_end();
        err = -EFAULT;
        goto out_fds;
    }

    if (ret == -ERESTARTNOHAND) {
        struct restart_block *restart_block;

        restart_block = &current->restart_block;
        restart_block->poll.ufds = ufds;
        restart_block->poll.nfds = nfds;

        if (timeout_msecs >= 0) {
            restart_block->poll.tv_sec = end_time.tv_sec;
            restart_block->poll.tv_nsec = end_time.tv_nsec;
            restart_block->poll.has_timeout = 1;
        } else
            restart_block->poll.has_timeout = 0;

        ret = set_restart_fn(restart_block, do_restart_poll);
    }
    return ret;
}
```

## do_poll

```c
int do_poll(struct poll_list *list, struct poll_wqueues *wait,
        struct timespec64 *end_time)
{
    poll_table* pt = &wait->pt;
    ktime_t expire, *to = NULL;
    int timed_out = 0, count = 0;
    u64 slack = 0;
    __poll_t busy_flag = net_busy_loop_on() ? POLL_BUSY_LOOP : 0;
    unsigned long busy_start = 0;

    /* Optimise the no-wait case */
    if (end_time && !end_time->tv_sec && !end_time->tv_nsec) {
        pt->_qproc = NULL;
        timed_out = 1;
    }

    if (end_time && !timed_out)
        slack = select_estimate_accuracy(end_time);

    for (;;) {
        struct poll_list *walk;
        bool can_busy_loop = false;

        for (walk = list; walk != NULL; walk = walk->next) {
            struct pollfd * pfd, * pfd_end;

            pfd = walk->entries;
            pfd_end = pfd + walk->len;
            for (; pfd != pfd_end; pfd++) {
                __poll_t mask;
                /*
                * Fish for events. If we found one, record it
                * and kill poll_table->_qproc, so we don't
                * needlessly register any other waiters after
                * this. They'll get immediately deregistered
                * when we break out and return.
                */
                mask = do_pollfd(pfd, pt, &can_busy_loop, busy_flag) {
                    int fd = pollfd->fd;
                    __poll_t mask, filter;

                    if (fd < 0)
                        return 0;

                    CLASS(fd, f)(fd);
                    if (fd_empty(f))
                        return EPOLLNVAL;

                    /* userland u16 ->events contains POLL... bitmap */
                    filter = demangle_poll(pollfd->events) | EPOLLERR | EPOLLHUP;
                    pwait->_key = filter | busy_flag;
                    mask = vfs_poll(fd_file(f), pwait);
                    if (mask & busy_flag)
                        *can_busy_poll = true;
                    return mask & filter;   /* Mask out unneeded events. */
                }

                pfd->revents = mangle_poll(mask);
                if (mask) {
                    count++;
                    pt->_qproc = NULL;
                    /* found something, stop busy polling */
                    busy_flag = 0;
                    can_busy_loop = false;
                }
            }
        }
        /*
        * All waiters have already been registered, so don't provide
        * a poll_table->_qproc to them on the next loop iteration.
        */
        pt->_qproc = NULL;
        if (!count) {
            count = wait->error;
            if (signal_pending(current))
                count = -ERESTARTNOHAND;
        }
        if (count || timed_out)
            break;

        /* only if found POLL_BUSY_LOOP sockets && not out of time */
        if (can_busy_loop && !need_resched()) {
            if (!busy_start) {
                busy_start = busy_loop_current_time();
                continue;
            }
            if (!busy_loop_timeout(busy_start))
                continue;
        }
        busy_flag = 0;

        /*
        * If this is the first loop and we have a timeout
        * given, then we convert to ktime_t and set the to
        * pointer to the expiry value.
        */
        if (end_time && !to) {
            expire = timespec64_to_ktime(*end_time);
            to = &expire;
        }

        if (!poll_schedule_timeout(wait, TASK_INTERRUPTIBLE, to, slack))
            timed_out = 1;
    }
    return count;
}
```

# inet_init

```c
/* include/linux/init.h */
#define ___define_initcall(fn, id, __sec) \
  static initcall_t __initcall_##fn##id __used \
    __attribute__((__section__(#__sec ".init"))) = fn;
#endif

#define __define_initcall(fn, id) ___define_initcall(fn, id, .initcall##id)

#define fs_initcall(fn)      __define_initcall(fn, 5)

/* net/ipv4/af_inet.c */
fs_initcall(inet_init);

int __init inet_init(void)
{
  struct inet_protosw *q;
  struct list_head *r;
  int rc = -EINVAL;

  sock_skb_cb_check_size(sizeof(struct inet_skb_parm));

  rc = proto_register(&tcp_prot, 1);
  rc = proto_register(&udp_prot, 1);
  rc = proto_register(&raw_prot, 1);
  rc = proto_register(&ping_prot, 1);

  (void)sock_register(&inet_family_ops);

#ifdef CONFIG_SYSCTL
  ip_static_sysctl_init();
#endif

  inet_add_protocol(&tcp_protocol, IPPROTO_TCP);
  inet_add_protocol(&udp_protocol, IPPROTO_UDP);
  inet_add_protocol(&icmp_protocol, IPPROTO_ICMP);
  inet_add_protocol(&igmp_protocol, IPPROTO_IGMP);

  /* Register the socket-side information for inet_create. */
  for (r = &inetsw[0]; r < &inetsw[SOCK_MAX]; ++r)
    INIT_LIST_HEAD(r);

  for (q = inetsw_array; q < &inetsw_array[INETSW_ARRAY_LEN]; ++q)
    inet_register_protosw(q);

  arp_init();

  ip_init();

  /* Setup TCP slab cache for open requests. */
  tcp_init();

  /* Setup UDP memory threshold */
  udp_init();

  /* Add UDP-Lite (RFC 3828) */
  udplite4_register();

  ping_init();

  if (icmp_init() < 0)
    panic("Failed to create the ICMP control socket.\n");

  /* Initialise the multicast router */
#if defined(CONFIG_IP_MROUTE)
  if (ip_mr_init())
    pr_crit("%s: Cannot init ipv4 mroute\n", __func__);
#endif

  if (init_inet_pernet_ops())
    pr_crit("%s: Cannot init ipv4 inet pernet ops\n", __func__);

  /* Initialise per-cpu ipv4 mibs */
  if (init_ipv4_mibs())
    pr_crit("%s: Cannot init ipv4 mibs\n", __func__);

  ipv4_proc_init();

  ipfrag_init();

  dev_add_pack(&ip_packet_type);

  ip_tunnel_core_init();
}

void __init tcp_init(void)
{
  int max_rshare, max_wshare, cnt;
  unsigned long limit;
  unsigned int i;

  percpu_counter_init(&tcp_sockets_allocated, 0, GFP_KERNEL);
  percpu_counter_init(&tcp_orphan_count, 0, GFP_KERNEL);

  inet_hashinfo_init(&tcp_hashinfo);
  inet_hashinfo2_init(
    &tcp_hashinfo,
    "tcp_listen_portaddr_hash",
    thash_entries, 21,  /* one slot per 2 MB*/
    0, 64 * 1024
  );

  tcp_hashinfo.bind_bucket_cachep = kmem_cache_create(
    "tcp_bind_bucket",
    sizeof(struct inet_bind_bucket),
    0,
    SLAB_HWCACHE_ALIGN|SLAB_PANIC,
    NULL
  );

  /* Size and allocate the main established and bind bucket
   * hash tables.
   *
   * The methodology is similar to that of the buffer cache. */
  tcp_hashinfo.ehash = alloc_large_system_hash(
    "TCP established",
    sizeof(struct inet_ehash_bucket),
    thash_entries,
    17, /* one slot per 128 KB of memory */
    0,
    NULL,
    &tcp_hashinfo.ehash_mask,
    0,
    thash_entries ? 0 : 512 * 1024
  );
  for (i = 0; i <= tcp_hashinfo.ehash_mask; i++)
    INIT_HLIST_NULLS_HEAD(&tcp_hashinfo.ehash[i].chain, i);

  if (inet_ehash_locks_alloc(&tcp_hashinfo))
    panic("TCP: failed to alloc ehash_locks");
  tcp_hashinfo.bhash = alloc_large_system_hash(
    "TCP bind",
    sizeof(struct inet_bind_hashbucket),
    tcp_hashinfo.ehash_mask + 1,
    17, /* one slot per 128 KB of memory */
    0,
    &tcp_hashinfo.bhash_size,
    NULL,
    0,
    64 * 1024
  );
  tcp_hashinfo.bhash_size = 1U << tcp_hashinfo.bhash_size;
  for (i = 0; i < tcp_hashinfo.bhash_size; i++) {
    spin_lock_init(&tcp_hashinfo.bhash[i].lock);
    INIT_HLIST_HEAD(&tcp_hashinfo.bhash[i].chain);
  }


  cnt = tcp_hashinfo.ehash_mask + 1;
  sysctl_tcp_max_orphans = cnt / 2;

  tcp_init_mem();
  /* Set per-socket limits to no more than 1/128 the pressure threshold */
  limit = nr_free_buffer_pages() << (PAGE_SHIFT - 7);
  max_wshare = min(4UL*1024*1024, limit);
  max_rshare = min(6UL*1024*1024, limit);

  init_net.ipv4.sysctl_tcp_wmem[0] = SK_MEM_QUANTUM;
  init_net.ipv4.sysctl_tcp_wmem[1] = 16*1024;
  init_net.ipv4.sysctl_tcp_wmem[2] = max(64*1024, max_wshare);

  init_net.ipv4.sysctl_tcp_rmem[0] = SK_MEM_QUANTUM;
  init_net.ipv4.sysctl_tcp_rmem[1] = 131072;
  init_net.ipv4.sysctl_tcp_rmem[2] = max(131072, max_rshare);

  tcp_v4_init();
  tcp_metrics_init();
  tcp_register_congestion_control(&tcp_reno);
  tcp_tasklet_init();
  mptcp_init();
}

struct pernet_operations __net_initdata tcp_sk_ops = {
  .init         = tcp_sk_init,
  .exit         = tcp_sk_exit,
  .exit_batch   = tcp_sk_exit_batch,
};

void __init tcp_v4_init(void)
{
  if (register_pernet_subsys(&tcp_sk_ops))
    panic("Failed to create the TCP control socket.\n");
}

int __net_init tcp_sk_init(struct net *net)
{
  int res, cpu, cnt;

  net->ipv4.tcp_sk = alloc_percpu(struct sock *);
  if (!net->ipv4.tcp_sk)
    return -ENOMEM;

  for_each_possible_cpu(cpu) {
    struct sock *sk;

    res = inet_ctl_sock_create(&sk, PF_INET, SOCK_RAW, IPPROTO_TCP, net);
    if (res)
      goto fail;
    sock_set_flag(sk, SOCK_USE_WRITE_QUEUE);

    /* Please enforce IP_DF and IPID==0 for RST and
     * ACK sent in SYN-RECV and TIME-WAIT state. */
    inet_sk(sk)->pmtudisc = IP_PMTUDISC_DO;

    *per_cpu_ptr(net->ipv4.tcp_sk, cpu) = sk;
  }

  net->ipv4.sysctl_tcp_ecn = 2;
  net->ipv4.sysctl_tcp_ecn_fallback = 1;

  net->ipv4.sysctl_tcp_base_mss = TCP_BASE_MSS;
  net->ipv4.sysctl_tcp_min_snd_mss = TCP_MIN_SND_MSS;
  net->ipv4.sysctl_tcp_probe_threshold = TCP_PROBE_THRESHOLD;
  net->ipv4.sysctl_tcp_probe_interval = TCP_PROBE_INTERVAL;

  net->ipv4.sysctl_tcp_keepalive_time = TCP_KEEPALIVE_TIME;
  net->ipv4.sysctl_tcp_keepalive_probes = TCP_KEEPALIVE_PROBES;
  net->ipv4.sysctl_tcp_keepalive_intvl = TCP_KEEPALIVE_INTVL;

  net->ipv4.sysctl_tcp_syn_retries = TCP_SYN_RETRIES;
  net->ipv4.sysctl_tcp_synack_retries = TCP_SYNACK_RETRIES;
  net->ipv4.sysctl_tcp_syncookies = 1;
  net->ipv4.sysctl_tcp_reordering = TCP_FASTRETRANS_THRESH;
  net->ipv4.sysctl_tcp_retries1 = TCP_RETR1;
  net->ipv4.sysctl_tcp_retries2 = TCP_RETR2;
  net->ipv4.sysctl_tcp_orphan_retries = 0;
  net->ipv4.sysctl_tcp_fin_timeout = TCP_FIN_TIMEOUT;
  net->ipv4.sysctl_tcp_notsent_lowat = UINT_MAX;
  net->ipv4.sysctl_tcp_tw_reuse = 2;

  cnt = tcp_hashinfo.ehash_mask + 1;
  net->ipv4.tcp_death_row.sysctl_max_tw_buckets = (cnt + 1) / 2;
  net->ipv4.tcp_death_row.hashinfo = &tcp_hashinfo;

  net->ipv4.sysctl_max_syn_backlog = max(128, cnt / 256);
  net->ipv4.sysctl_tcp_sack = 1;
  net->ipv4.sysctl_tcp_window_scaling = 1;
  net->ipv4.sysctl_tcp_timestamps = 1;
  net->ipv4.sysctl_tcp_early_retrans = 3;
  net->ipv4.sysctl_tcp_recovery = TCP_RACK_LOSS_DETECTION;
  net->ipv4.sysctl_tcp_slow_start_after_idle = 1; /* By default, RFC2861 behavior.  */
  net->ipv4.sysctl_tcp_retrans_collapse = 1;
  net->ipv4.sysctl_tcp_max_reordering = 300;
  net->ipv4.sysctl_tcp_dsack = 1;
  net->ipv4.sysctl_tcp_app_win = 31;
  net->ipv4.sysctl_tcp_adv_win_scale = 1;
  net->ipv4.sysctl_tcp_frto = 2;
  net->ipv4.sysctl_tcp_moderate_rcvbuf = 1;
  /* This limits the percentage of the congestion window which we
   * will allow a single TSO frame to consume.  Building TSO frames
   * which are too large can cause TCP streams to be bursty. */
  net->ipv4.sysctl_tcp_tso_win_divisor = 3;
  /* Default TSQ limit of four TSO segments */
  net->ipv4.sysctl_tcp_limit_output_bytes = 262144;
  /* rfc5961 challenge ack rate limiting */
  net->ipv4.sysctl_tcp_challenge_ack_limit = 1000;
  net->ipv4.sysctl_tcp_min_tso_segs = 2;
  net->ipv4.sysctl_tcp_min_rtt_wlen = 300;
  net->ipv4.sysctl_tcp_autocorking = 1;
  net->ipv4.sysctl_tcp_invalid_ratelimit = HZ/2;
  net->ipv4.sysctl_tcp_pacing_ss_ratio = 200;
  net->ipv4.sysctl_tcp_pacing_ca_ratio = 120;

  if (net != &init_net) {
    memcpy(net->ipv4.sysctl_tcp_rmem,
           init_net.ipv4.sysctl_tcp_rmem,
           sizeof(init_net.ipv4.sysctl_tcp_rmem));
    memcpy(net->ipv4.sysctl_tcp_wmem,
           init_net.ipv4.sysctl_tcp_wmem,
           sizeof(init_net.ipv4.sysctl_tcp_wmem));
  }
  net->ipv4.sysctl_tcp_comp_sack_delay_ns = NSEC_PER_MSEC;
  net->ipv4.sysctl_tcp_comp_sack_nr = 44;
  net->ipv4.sysctl_tcp_fastopen = TFO_CLIENT_ENABLE;
  spin_lock_init(&net->ipv4.tcp_fastopen_ctx_lock);
  net->ipv4.sysctl_tcp_fastopen_blackhole_timeout = 60 * 60;
  atomic_set(&net->ipv4.tfo_active_disable_times, 0);

  /* Reno is always built in */
  if (!net_eq(net, &init_net) &&
      try_module_get(init_net.ipv4.tcp_congestion_control->owner))
    net->ipv4.tcp_congestion_control = init_net.ipv4.tcp_congestion_control;
  else
    net->ipv4.tcp_congestion_control = &tcp_reno;

  return 0;
fail:
  tcp_sk_exit(net);

  return res;
}
```

```c
inet_init()
    proto_register(&tcp_prot, 1);
    sock_register(&inet_family_ops);
    inet_add_protocol(&tcp_protocol, IPPROTO_TCP);

    for (q = inetsw_array; q < &inetsw_array[INETSW_ARRAY_LEN]; ++q)
        inet_register_protosw(q);

    ip_init();
    arp_init();
    udp_init();
    ping_init();

    tcp_init();
        inet_hashinfo_init(&tcp_hashinfo);
        tcp_init_mem();
        init_net.ipv4.sysctl_tcp_wmem[0] = ;

        tcp_v4_init();

        tcp_metrics_init();
        tcp_register_congestion_control(&tcp_reno);
        mptcp_init();

        tcp_tasklet_init();
            for_each_possible_cpu(i) {
                struct tsq_tasklet *tsq = &per_cpu(tsq_tasklet, i);
                INIT_LIST_HEAD(&tsq->head);
                tasklet_setup(&tsq->tasklet, tcp_tasklet_func);
            }
```

# net_dev_init

<img src='../images/kernel/net-dev-pci.svg' style='max-height:850px'/>


|Device Type|Physical/Virtual|Purpose|Example Use Case|
|-----------|----------------|-------|----------------|
|Ethernet (eth0)|Physical|Wired networking|LAN connection|
|Wireless (wlan0)|Physical|Wireless networking|Wi-Fi access|
|Loopback (lo)|Virtual|Localhost communication|Testing services|
|Bridge (br0)|Virtual|Connect multiple interfaces|Container/VM networking|
|Veth|Virtual|Namespace connectivity|Container-to-bridge link|
|TUN/TAP|Virtual|VPN or VM networking|OpenVPN, QEMU|
|Macvlan|Virtual|Direct LAN IP assignment|Container IPs on network|
|VXLAN|Virtual|Overlay networking|Cloud multi-host setups|
|Bonding (bond0)|Virtual|NIC aggregation|High availability|
|VLAN (eth0.10)|Virtual|Network segmentation|VLAN tagging|

```c
static int __init net_dev_init(void)
{
    int i, rc = -ENOMEM;

    BUG_ON(!dev_boot_phase);

    net_dev_struct_check();

    if (dev_proc_init())
        goto out;

    if (netdev_kobject_init())
        goto out;

    for (i = 0; i < PTYPE_HASH_SIZE; i++)
        INIT_LIST_HEAD(&ptype_base[i]);

    if (register_pernet_subsys(&netdev_net_ops))
        goto out;

    /*    Initialise the packet receive queues. */
    flush_backlogs_fallback = flush_backlogs_alloc() {
        return kmalloc(struct_size_t(struct flush_backlogs, w, nr_cpu_ids), GFP_KERNEL);
    }
    if (!flush_backlogs_fallback)
        goto out;

    for_each_possible_cpu(i) {
        struct softnet_data *sd = &per_cpu(softnet_data, i);

        skb_queue_head_init(&sd->input_pkt_queue);
        skb_queue_head_init(&sd->process_queue);

        skb_queue_head_init(&sd->xfrm_backlog);
        INIT_LIST_HEAD(&sd->poll_list);
        sd->output_queue_tailp = &sd->output_queue;

        INIT_CSD(&sd->csd, rps_trigger_softirq, sd);
        sd->cpu = i;

        INIT_CSD(&sd->defer_csd, trigger_rx_softirq, sd);
        spin_lock_init(&sd->defer_lock);

        init_gro_hash(&sd->backlog);
        sd->backlog.poll = process_backlog;
        sd->backlog.weight = weight_p;
        INIT_LIST_HEAD(&sd->backlog.poll_list);

        if (net_page_pool_create(i))
            goto out;
    }
    if (use_backlog_threads())
        smpboot_register_percpu_thread(&backlog_threads);

    dev_boot_phase = 0;

    if (register_pernet_device(&loopback_net_ops))
        goto out;

    if (register_pernet_device(&default_device_ops))
        goto out;

    open_softirq(NET_TX_SOFTIRQ, net_tx_action);
    open_softirq(NET_RX_SOFTIRQ, net_rx_action) {
        softirq_vec[nr].action = action;
    }

    rc = cpuhp_setup_state_nocalls(CPUHP_NET_DEV_DEAD, "net/dev:dead",
                    NULL, dev_cpu_dead);
    WARN_ON(rc < 0);
    rc = 0;

    /* avoid static key IPIs to isolated CPUs */
    if (housekeeping_enabled(HK_TYPE_MISC))
        net_enable_timestamp();
out:
    if (rc < 0) {
        for_each_possible_cpu(i) {
            struct page_pool *pp_ptr;

            pp_ptr = per_cpu(system_page_pool, i);
            if (!pp_ptr)
                continue;

            xdp_unreg_page_pool(pp_ptr);
            page_pool_destroy(pp_ptr);
            per_cpu(system_page_pool, i) = NULL;
        }
    }

    return rc;
}
```

# ebpf

* [OPPO - ebpf工作原理介绍](https://mp.weixin.qq.com/s/EODZfKRn15HVXZTKh6Vg3Q)

# Tunning

## System-Wide

On Linux, system-wide tunable parameters can be viewed and set using the sysctl(8) command and written to /etc/sysctl.conf. They can also be read and written from the /proc file system, under **/proc/sys/net**.

```sh
# Enabling auto-tuning of the TCP receive buffer:
net.ipv4.tcp_moderate_rcvbuf = 1

# Setting the auto-tuning parameters for the TCP read and write buffers:
# the minimum, default, and maximum number of bytes to use
net.ipv4.tcp_rmem = 4096 87380 16777216
net.ipv4.tcp_wmem = 4096 65536 16777216

# Socket and TCP Buffers
net.core.rmem_max = 16777216
net.core.wmem_max = 16777216

# TCP Backlog
# First backlog queue, for half-open connections:
net.ipv4.tcp_max_syn_backlog = 4096
# Second backlog queue, the listen backlog, for passing connections to accept(2):
net.core.somaxconn = 1024

# TCP Port
net.ipv4.ip_local_port_range = 1024 65535

# Device Backlog
# Increasing the length of the network device backlog queue, per CPU:
net.core.netdev_max_backlog = 10000

# TCP Congestion Control
# Linux supports pluggable congestion-control algorithms. Listing those currently available:
sysctl net.ipv4.tcp_available_congestion_control
net.ipv4.tcp_available_congestion_control = reno cubic

modprobe tcp_htcp
sysctl net.ipv4.tcp_available_congestion_control
net.ipv4.tcp_available_congestion_control = reno cubic htcp

# TCP Options
net.ipv4.tcp_sack = 1
net.ipv4.tcp_fack = 1
net.ipv4.tcp_tw_reuse = 1
net.ipv4.tcp_tw_recycle = 0

# ECN Explicit Congestion Notification
# 0 to disable ECN
# 1 to allow for incoming connections and request ECN on outgoing connections
# 2 to allow for incoming and not request ECN on outgoing.
net.ipv4.tcp_ecn = 1

# Byte Queue Limits
grep . /sys/devices/pci.../net/ens5/queues/tx-0/byte_queue_limits/limit*
/sys/devices/pci.../net/ens5/queues/tx-0/byte_queue_limits/limit:16654
/sys/devices/pci.../net/ens5/queues/tx-0/byte_queue_limits/limit_max:1879048192
/sys/devices/pci.../net/ens5/queues/tx-0/byte_queue_limits/limit_min:0

# Queueing Disciplines
# list the qdiscs on your system using:
man -k tc-
sysctl net.core.default_qdisc
net.core.default_qdisc = fq_codel
```

## Socket Options

Option Name | Description
- | -
SO_SNDBUF, SO_RCVBUF | Send and receive buffer sizes (these can be tuned up to the system limits described earlier; there is also SO_SNDBUFFORCE to override the send limit).
SO_REUSEPORT | Allows multiple processes or threads to bind to the same port, allowing the kernel to distribute load across them for scalability (since Linux 3.9).
SO_MAX_PACING_RATE | Sets the maximum pacing rate, in bytes per second (see tc-fq(8)).
SO_LINGER | Can be used to reduce TIME_WAIT latency.
SO_TXTIME | Request time-based packet transmission, where deadlines can be supplied (since Linux 4.19) [Corbet 18c] (used for UDP pacing [Bruijn 18]).
TCP_NODELAY | Disables Nagle, sending segments as soon as possible. This may improve latency at the cost of higher network utilization (more packets).
TCP_CORK | Pause transmission until full packets can be sent, improving throughput. (There is also a system-wide setting for the kernel to automatically attempt corking: net.ipv4.tcp_autocorking.)
TCP_QUICKACK | Send ACKs immediately (can increase send bandwidth).
TCP_CONGESTION | Congestion control algorithm for the socket.