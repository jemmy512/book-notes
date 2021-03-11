# 2 The Transport Layer: TCP, UDP, SCTP
## TCP:
Reliable, full-duplex byte stream, acknowlegments, timeouts, sequence number, RTT estimate, retransmissions.

TCP is not a 100% reliable protocol, it provides reliable delivery of data or reliable notification of failure.

TCP contains algorithms to estimate the round-trip time(RTT) between a client and server dynamically.

TCP sequences the data by associating a sequence number with every byte that it sends.

TCP provides flow control which is called advertised window.

## UDP
The application writes a msg to a UDP socket, which is then encapsulated in a UDP datagram and then further encapsulated as an IP datagram. And sent to its destination.

Each UDP datagram has a length while TCP is a bytes-stream protocol without any record boundaries at all.

* Three-Way Handshake:
    * SYN: tell server/client client/server inital sequence number. Contain an IP header, a TCP header and options.
    * The ACK of each FIN is the sequence number of the FIN plus one.

* TCP Options:
    * MSS(maxmium segment size): fetch this option by TCP_MAXSEG
    * Window scale option: fetch by SO_REVBUF
    * Timestamp option.

* TCP Termination:

* TIME_WAIT State:
    * The duration that this endpoint remains in this state is twice the maxmium segment lifetime, 2MSL.
    * Two reasons for TIME_WAIT:
        1. To implement TCP's full-duplex connection termination reliably
        2. To allow old duplicate segments to expire in the network

* Port Numbers:
    1. The well-know ports: 0 1023
    2. The regisered ports: 1024 49151
    3. The dynamic or private ports: 49152 65535

4 Elementary of TCP Sockets

5 TCP Client and Server Example

# 7 Sockets Options
## 7.5 Generic Socket Options

### SO_KEEPALIVE
* Scenarios:
    1. The peer responds with the expected ACK. The application is not notified (since everything is okay). TCP will send another probe following another two hours of inactivity.
    2. The peer responds with an RST, which tells the local TCP that the peer host has crashed and rebooted. The socket’s pending error is set to `ECONNRESET` and the socket is closed.
    3. There is no response from the peer to the keep-alive probe. Berkeley-derived TCPs send 8 additional probes, 75 seconds apart, trying to elicit a response. TCP will give up if there is no response within 11 minutes and 15 seconds after sending the first probe. If there is no response at all to TCP’s keep-alive probes, the socket’s pending error is set to `ETIMEDOUT` and the socket is closed. But if the socket receives an ICMP error in response to one of the keep-alive probes, the corresponding error (Figures A.15 and A.16) is returned instead (and the socket is still closed). A common ICMP error in this scenario is ‘‘host unreachable,’’ indicating that the peer host is unreachable, in which case, the pending error is set to `EHOSTUNREACH`. This can occur either because of a network failure or because the remote host has crashed and the last-hop router has detected the crash.

Most kernels maintain these parameters on a per-kernel basis, not on a per-socket basis, so changing the inactivity period from 2 hours to 15 minutes, for example, will affect all sockets on the host that enable this option.

The purpose of this option is to detect if the peer **host** crashes or becomes unreachable. If the peer **process** crashes, its TCP will send a FIN across the connection, which we can easily detect with select.

This option is normally used by servers, although clients can also use the option. Servers use the option because they spend most of their time blocked waiting for input across the TCP connection, that is, waiting for a client request. But if the client host’s connection drops, is powered off, or crashes, the server process will never know about it, and the server will continually wait for input that can never arrive. This is called a half-open connection. The keep-alive option will detect these half-open connections and terminate them.

Scenario | Peer process crashes | Peer host crashes | Peer host is unreachable
--- | --- | --- | ---
Our TCP is actively receiving data | Peer TCP will send a FIN, which we will read as a (possibly premature) EOF. | We will stop receiving data. | We will stop receiving data.
Our TCP is actively sending data | Peer TCP sends a FIN, which we can detect immediately using select for readability. If TCP sends another segment, peer TCP responds with an RST. If the application attempts to write to the socket after TCP has received an RST, our socket implementation sends us SIGPIPE. | Our TCP will time out and our socket’s pending error will be set to `ETIMEDOUT`. | Our TCP will time out and our socket’s pending error will be set to `EHOSTUNREACH`.
Connection is idle, keep-alive set | Peer TCP sends a FIN, which we can detect immediately using select for readability. | Nine keep-alive probes are sent after two hours of inactivity and then our socket’s pending error is set to `ETIMEDOUT`. | Nine keep-alive probes are sent after two hours of inactivity and then our socket’s pending error is set to `EHOSTUNREACH`.
Connection is idle, keep-alive not set | Peer TCP sends a FIN, which we can detect immediately using select for readability. | (Nothing) | (Nothing)

### SO_LINGER
* By default, close returns immediately, but if there is any data still remaining in the socket send buffer, the system will try to deliver the data to the peer.

* Three scenarios of SO_LINGER:
    1. If `l_onoff is 0`, the option is turned off. The value of l_linger is ignored and the previously discussed TCP default applies: close returns immediately.
    2. If `l_onoff is nonzero` and `l_linger is zero`, TCP aborts the connection when it is closed (pp. 1019–1020 of TCPv2). That is, TCP discards any data still remaining in the socket send buffer and sends an `RST` to the peer, not the normal four-packet connection termination sequence. This avoids TCP’s `TIME_WAIT` state, but in doing so, leaves open the possibility of another incarnation of this connection being created within 2MSL seconds (Section 2.7) and having old duplicate segments from the just-terminated connection being incorrectly delivered to the new incarnation.
    3. If `l_onoff is nonzero` and `l_linger is nonzero`, then the kernel will linger when the socket is closed (p. 472 of TCPv2). That is, if there is any data still remaining in the socket send buffer, the process is put to sleep until either: (i) all the data is sent and acknowledged by the peer TCP, or (ii) the linger time expires. If the socket has been set to nonblocking (Chapter 16), it will not wait for the close to complete, even if the linger time is nonzero. When using this feature of the SO_LINGER option, it is important for the application to check the return value from close, because if the linger time expires before the remaining data is sent and acknowledged, close returns EWOULDBLOCK and any remaining data in the send buffer is discarded.

 * The client’s close can return before the server reads the remaining data in its socket receive buffer. Therefore, it is possible for the server host to crash before the server application reads this remaining data, and the client application will never know.

 * Default operation of close: it returns immediately. ![](../Images/Unp/7.5-so-linger-1.png)
 * close with SO_LINGER socket option set and l_linger a positive value. ![](../Images/Unp/7.5-so-linger-2.png)
 * close with SO_LINGER socket option set and l_linger a small positive value. ![](../Images/Unp/7.5-so-linger-3.png)
 * A successful return from close, with the SO_LINGER socket option set, only tells us that the data we sent (and our FIN) have been acknowledged by the peer TCP. This does not tell us whether the peer application has read the data. If we do not set the SO_LINGER socket option, we do not know whether the peer TCP has acknowledged the data.
* Using shutdown to know that peer has received our data. ![](../Images/Unp/7.5-so-linger-4.png)
* Another way to know that the peer application has read our data is to use an application-level acknowledgment, or application ACK. ![](../Images/Unp/7.5-so-linger-5.png)

Function | Description
--- | ---
shutdown SHUT_RD | No more receives can be issued on socket; process can still send on socket; socket receive buffer discarded; any further data received is discarded by TCP (Exercise 6.5); no effect on socket send buffer.
shutdown SHUT_WR | No more sends can be issued on socket; process can still receive on socket; contents of socket send buffer sent to other end, followed by normal TCP connection termination (FIN); no effect on socket receive buffer.
close l_onoff = 0 (default) | No more receives or sends can be issued on socket; contents of socket send buffer sent to other end. If descriptor reference count becomes 0: normal TCP connection termination (FIN) sent following data in send buffer and socket receive buffer discarded.
close l_onoff = 1 l_linger = 0 | No more receives or sends can be issued on socket. If descriptor reference count becomes 0: RST sent to other end; connection state set to CLOSED (no TIME_WAIT state); socket send buffer and socket receive buffer discarded.
close l_onoff = 1 l_linger != 0 | No more receives or sends can be issued on socket; contents of socket send buffer sent to other end. If descriptor reference count becomes 0: normal TCP connection termination (FIN) sent following data in send buffer; socket receive buffer discarded; and if linger time expires before connection CLOSED, close returns `EWOULDBLOCK`.