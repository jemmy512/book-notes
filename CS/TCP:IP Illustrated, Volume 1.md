# 1 Introduction

### 1.1 Architecture Priciples
#### 1.1.1 Packets, Connections, and Datagrams

In ***statistical multiplexing***, traffic is mixed together based on the arrival statistics or timing pattern of the traffic.

Alternative techniques, such as time-division multiplexing ***(TDM)*** and ***static multiplexing***, typically reserve a certain amount of time or other resources for data on each connection.


Note that while circuits are straightforwardly implemented using TDM techniques, ***virtual circuits (VCs)*** that exhibit many of the behaviors of circuits but do not depend on physical circuit switches can be implemented atop connection-oriented packets.

The per-flow state is established prior to the exchange of data on a VC using a signaling protocol that supports connection establishment, clearing, and status information. Such networks are consequently called ***connection-oriented***.

#### 1.1.2 The End-to-End Argument and Fate Sharing

The function in question can completely and correctly be implemented only with the knowledge and help of the application standing at the end points of the com- munication system. 

Fate sharing suggests placing all the necessary state to maintain an active communication association (e.g., virtual connection) at the same location with the communicating endpoints.

#### 1.1.3 Error Control and Flow Control

### 1.2 Design and Implementation

# 12 TCP: The Transmisssion Control Protocol(Premilinary)

### 12.2 Introduction to TCP

#### 12.2.1 The TCP Service Model

TCP provides a connection-oriented, reliable, byte stream service.

#### 12.2.2 Reliability in TCP

If a segment arrives with an invalid checksum, TCP discards it without sending any acknowledgment for the discarded packet.

When TCP sends a group of segments, it normally sets a single retransmission timer, waiting for the other end to acknowledge reception. TCP does not set a dif- ferent retransmission timer for every segment. Rather, it sets a timer when it sends a window of data and updates the timeout as ACKs arrive.

This acknowledgment may not be sent immediately but is nor- mally delayed a fraction of a second. The ACKs used by TCP are cumulative in the sense that an ACK indicating byte number N implies that all bytes up to number N (but not including it) have already been received successfully. 

This provides some robustness against ACK loss—if an ACK is lost, it is very likely that a subsequent ACK is sufficient to ACK the previous segments.

Once a connection is established, every TCP segment that contains data flowing in one direction of the connection also includes an ACK for segments flowing in the opposite direction.

### 12.3 Tcp Header and Encapsulation

```C++
    0                             15 16                            31
    -----------------------------------------------------------------
        Source Port(16)             |    Destination Port(16)
    -----------------------------------------------------------------
                            Sequence Number(32)
    -----------------------------------------------------------------
                            Acknowledgment Number(32)
    -----------------------------------------------------------------
     Header  Reserv  C E U A P R S F|
     Length   (4)    W C R C S S Y I|       Window Size(16)
       (4)           R E G K H T N N|
    -----------------------------------------------------------------
            Checksum(16)            |       Urgent Pointer(16)
    -----------------------------------------------------------------
                                Options
    kind(1) | lenth(1)  | info(n)
    0                   |   EOL
    1                   |   NOP
    2           4       |   MSS
    3           3       |   WSCALE
    4           2       |   SACK
    5           N*8 + 2 |   [,]...[,]
    8           10      |   TSOPT
    -----------------------------------------------------------------
```

 The combination of an IP address and a port number is sometimes called an ***endpoint or socket*** in the TCP literature. 

 The ***Sequence Number*** field identifies the byte in the stream of data from the sending TCP to the receiving TCP that the first byte of data in the containing segment represents. 

 The ***ACK*** Number contains the next sequence number that the sender of the acknowledgment expects to receive.

 The sequence number of the first byte of data sent on this direction of the connection is the ISN plus 1 because the SYN bit field consumes one sequence number.

 As we shall see later, consuming a sequence number also implies reliable delivery using retransmission. Thus, SYNs and application bytes (and FINs, which we will see later) are reliably delivered. ACKs, which do not consume sequence numbers, are not.

 This ***urgent pointer*** is a positive offset that must be added to the Sequence Number field of the segment to yield the sequence number of the last byte of urgent data. 

 To be more efficient, multiple packets must be injected into the network before an ACK is received. This approach is more efficient but also more complex. A typical approach to managing the complexity is to use sliding windows, whereby packets are marked with sequence numbers, and the window size bounds the number of such packets. When the window size var- ies based on either feedback from the receiver or other signals (such as dropped packets), both flow control and congestion control can be achieved.

 # 13 Connection Management

### 13.2 Tcp Connection Establishment and Termination
#### 13.2.1 Tcp alf-close 
#### 13.2.2 Simultaneous Open and Close

A simultaneous open requires the exchange of four segments, one more than the normal three-way handshake.

With a simultaneous close the same number of segments are exchanged as in the normal close. The only real difference is that the segment sequence is interleaved instead of sequential.

#### 13.2.3 Initial Sequence Number
[RFC0793] specifies that the ISN should be viewed as a 32-bit counter that increments by 1 every 4μs. The purpose of doing this is to arrange for the sequence numbers for segments on one connection to not overlap with sequence numbers on a another (new) identical connection. 

#### 13.2.6 Connections and Translator
By implementing a portion of the TCP state machine in a NAT (see, for example, Sections 3.5.2.1 and 3.5.2.2 of [RFC6146]), the connection can be tracked, including the current states, sequence numbers in each direction, and corresponding ACK numbers. Such state tracking is typical for NAT implementations.

### 13.3 Tcp Options
Note that the MSS option is not a negotiation between one TCP and its peer; it is a limit. When one TCP gives its MSS option to the other, it is indicating its unwillingness to accept any segments larger than that size for the duration of the connection.

#### 13.3.2 Selective Ackownledgment(SACK)
Once this has taken place, the TCP receiving out-of-sequence data may provide a SACK option that describes the out-of-sequence data to help its peer perform retransmis- sions more efficiently. SACK information contained in a SACK option consists of a range of sequence numbers representing data blocks the receiver has successfully received. Each range is called a SACK block and is represented by a pair of 32-bit sequence numbers. Thus, a SACK option containing n SACK blocks is (8n + 2) bytes long. Two bytes are used to hold the kind and length of the SACK option.

#### 13.3.3 WSCALE/WSOPT
1. This option can appear only in a SYN segment. 
2. The 1-byte shift count is between 0 and 14 (inclusive). A shift count of 0 indicates no scaling. The maximum scale value of 14 provides for a maximum window of 1,073,725,440 bytes (65,535 × 214), close to 1,073,741,823 (230 −1), effectively 1GB. TCP then maintains the “real” window size internally as a 32-bit value.
3. To enable window scaling, both ends must send the option in their SYN segments. 

#### 13.3.4 Timestamp Option and Protection Against Wrapped Sequence Number
The main reason for wishing to calculate a good estimate of the connection’s RTT is to set the retransmission timeout.

The PAWS algorithm does not require any form of time synchronization between the sender and the receiver. All the receiver needs is for the timestamp values to be monotonically increasing, and to increase by at least 1 per window of data.

#### 13.3.5 User Timeout(UTO) Option
The UTO value (USER_TIMEOUT) specifies the amount of time a TCP sender is willing to wait for an ACK of outstanding data before con- cluding that the remote end has failed.

The UTO option allows one TCP to signal its USER_TIMEOUT value to its connection peer.

NAT devices could also interpret such information to help set their connection activity timers.
```C++
USER_TIMEOUT = min(U_LIMIT, max(ADV_UTO, REMOTE_UTO, L_LIMIT))
```
#### 13.3.6 Authentication Option

### 13.4 Path MTU Discovery with TCP

### 13.5 Tcp State Transition
#### 13.5.2 TIME_WAIT(2 MSL)
The final ACK is resent not because the TCP retransmits ACKs (they do not consume sequence numbers and are not retransmitted by TCP), but because the other side will retransmit its FIN (which does consume a sequence number). 

Any delayed segments that arrive for a connection while it is in the 2MSL wait state are dis- carded. 

### 13.5.3 Quiet Time Concept
[RFC0793] states that TCP should wait an amount of time equal to the MSL before creating any new connections after a reboot or crash. This is called the ***quiet time***. Few implementations abide by this because most hosts take longer than the MSL to reboot after a crash.

#### 13.5.4 FIN_WAIT_2 State
One end of the connection can remain in this state forever. The other end is still in the CLOSE_WAIT state and can remain there forever, until the application decides to issue its close.

Solution: If the application that does the active close does a complete close, not a half-close indicating that it expects to receive data, a timer is set. If the connection is idle when the timer expires, TCP moves the connection into the CLOSED state.

#### 13.5.5 Simultaneous Open and Close Transitions

![Img: tcp state](../Images/tcp-state.png)

### 13.6 RESET Segments
