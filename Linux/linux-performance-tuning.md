# CPU Performance Tunning

# Memory Performance Tunning

# IO Performance Tunning

# Network Performance Tunning

* Three ways handeshake

    Parameter | Note
    --- | ---
    tcp_syn_retries | retry times of SYN
    tcp_max_syn_backlog, somaxconn, backlog | the legnth of half accept queue
    min(backlog, somaxconn) | the length of the accpet queue
    tcp_synack_retries | retry times of the SYN_ACK
    tcp_fastopen |

* Four ways handeshake

    Paramter | Note
    --- | ---
    tcp_orphan_retries | retry times of FIN
    tcp_fin_tiemout | the timeout of FIN_WAIT2
    tcp_max_orphans | the number of orphan connections
    tcp_max_tw_buckets | the numbers of sockets in time_wait state
    tcp_tw_reuse, tcp_timestamps |

* Tcp transfer

    Parameter | Note
    --- | ---
    tcp_window_scaling |
    tcp_wmem |
    tcp_rmem |
    tcp_moderate_rcvbuf | dynamicly adjust rcv buf
    tcp_mem |