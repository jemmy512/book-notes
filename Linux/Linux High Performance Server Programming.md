# 1. TCP/IP Protocos
Four levle protocols:

Layer | Protocols | Functions
-- | --- | ---
Application Layer | ping telnet OSPF DNS(53) HTTP(80) | processing application logic
Transport Layer | TCP  UDP | support end to end communication for two endpoints
Network Lyaer | ICMP IP(0x800) | route and forward data pakage
Link Layer | ARP(0x806)  RARP(0x835) | implements the NIC interface to handle the transmission of data over the physical medium
    
The source IP and destination IP in the IP header are always the same during transmission.

The source Mac and destinaion Mac in the link layer data frame always change during transmission.

# 2. IP 协议协议详解
IP协议特点：
    无状态：通信双方不同步彼此传输数据的状态信息
    无连接：通信双方不长久的维持对方信息
    不可靠：不保证IP数据包准确的到达接收端

# 3. TCP 协议协议详解

字节流：应用程序对数据的发送和接收没有边界限制
UDP: 应用程序没有足够的接收缓冲区，UDP数据将会被截断

连接停留在FIN_WAIT_2状态：客户端执行关闭连接后未等待服务器响应强行退出。此时客户端连接由内核管理，称之为孤儿连接，
    Linux定义了两个变量分别定义系统能接纳的最大孤儿连接数量和孤儿连接在内核中最大的生存时间。
TIME_WAIT:
    可使用SO_REUSEDADDR 解决TIME_WAIT端口占用问题。
带外数据(OOB)
    紧急指针指向最后一个带外数据的下一个字节，一次发送多个带外数据，有效的只是最后一个。TCP接收端读取带外数据存放到
    带外缓冲区。若没有及时接收，带外数据将被普通数据覆盖。SO_OOBINLINE指定可像接受普通数据一样接受带外数据。
    Linux 提供两种机制通知应用程序OOB的到来：
        1. I/O复用产生的异常事件
        2. SIGURG信号
    int sockatmark(int sockfd); // 判断下一个数据是否是OOB数据，是 ？ 1 ： 0
拥塞控制：SWND, RWND, CWND(Congestion Window), IW(Initial Window), SMSS(sender maxium segament size)
    慢启动(slow start)，拥塞避免(congestion avoidance ):
        CWND += min(N, SMSS) N: 此次确认中包含的之前未被确认的字节数
        当CWND超过慢启动门限(slow start threshold size, ssthres)时，TCP阻塞控制进入阻塞避免阶段。
        阻塞避免算法是的CWND按照线型方式增加。
        网络阻塞判断标准：传输超时，接收到重复的报文确认段。
    快速重传(fast retransmit)，快速回复(fast recovery):
