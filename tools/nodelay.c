/* libnodelay.so -- LD_PRELOAD shim that sets TCP_NODELAY on every TCP socket a
 * process creates (socket) or accepts (accept, accept4), and re-arms
 * TCP_QUICKACK after every recv/recvfrom/read AND every send/sendto/sendmsg/
 * write/writev on a TCP socket.
 *
 * The two halves of one deadlock. NODELAY stops THIS process holding a small
 * write until the peer acknowledges the previous one; QUICKACK stops this
 * process delaying its acknowledgment of the PEER's small write, which is the
 * half we can fix when the peer is the simulator binary. Linux clears
 * QUICKACK on its own, hence re-arming on every read.
 *
 * Re-arming on READ alone does not work, and that is why the first version of
 * this shim measured "no change" (18.5 Hz). Linux re-enters delayed-ACK mode
 * whenever the process SENDS shortly after it received (tcp_event_data_sent,
 * the "pingpong" heuristic), and a bridge reply is exactly that, so an arm on
 * the read is undone by the reply that follows it. QUICKACK is therefore
 * re-armed after every send/write syscall as well. It has to be the syscall:
 * re-arming from Python after sio.emit() changes nothing (19.4 Hz), because
 * gevent-websocket queues the frame and writes it from another greenlet, so
 * that re-arm still lands before the write.
 *
 * Measured 2026-09-12 on the development laptop (i7-12700H, RTX 4070, HUD at
 * 144 fps), tools/sim_rate_probe.py on loopback, lo MTU 65536, same session:
 *   no options                            19.3 Hz   (median 50.9 ms)
 *   Python-level QUICKACK after emit      19.4 Hz
 *   this shim preloaded, no flags        101.6 Hz   (median 8.8 ms, max 25)
 * Real bridge, ros2 lidar topic, 25 s each, same session, Linux simulator only:
 *   tcp_nodelay:=false                    18.6 Hz   (median 52.7 ms)
 *   tcp_nodelay:=true (this shim)         77.3 Hz   (median 12.8 ms, max 25)
 * Cross-check of the mechanism: lo MTU 1500 and no options gives 107 Hz, because
 * the simulator's ~13 KB telemetry is then full-size segments that Nagle sends
 * at once instead of one sub-MSS segment it holds for our ACK. The earlier
 * reading that "the simulator's frame period is the limit" on this laptop was
 * wrong; the socket deadlock was the limit here too, and on a second machine
 * over a LAN cable the same simulator ran the loop at 63.6 Hz.
 *
 * Why: the simulator emits telemetry only in reply to the bridge's message,
 * over a websocket on loopback. Each websocket message is written as small
 * pieces; Nagle's algorithm holds a small write until the previous one is
 * acknowledged, and the receiver's delayed ACK holds that acknowledgment for
 * up to 40 ms. Two such waits per round trip cap the loop at 10-20 Hz on any
 * machine, which is exactly what every rate measurement here showed and what
 * another team traced to this cause. TCP_NODELAY sends small writes at once.
 *
 * NODELAY_CAP_HZ=<hz> paces the loop from the same place. The simulator emits
 * telemetry only in reply, so spacing the bridge's replies at least 1/hz apart
 * caps the whole loop at hz: before every send on a TCP socket the shim sleeps
 * until the previous send on that socket is 1/hz old, scheduled against the
 * previous target (not the previous wake-up) so jitter does not accumulate
 * into drift. It blocks only the bridge's gevent thread, which has nothing else
 * to do between messages; the ROS spin is another thread. The organizers say
 * the evaluation machine runs 40-50 Hz, and with the deadlock gone this laptop
 * runs 77-85, so the cap is how the stack is tuned at the evaluation rate
 * without a slower machine. 0 or unset = no cap. Exposed as loop_hz_cap:= on
 * bridge.launch.py / race.launch.py, and as -e NODELAY_CAP_HZ on docker run.
 *
 * Applied from bridge.launch.py (tcp_nodelay:=true) as the bridge process's
 * environment, so the devkit package stays unmodified. It can also be put in
 * front of the simulator binary for a local test of the other direction.
 *
 *   gcc -shared -fPIC -O2 -o tools/libnodelay.so tools/nodelay.c -ldl
 *   NODELAY_VERBOSE=1 to log each socket it touches and the cap in force.
 */
#define _GNU_SOURCE
#include <dlfcn.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/uio.h>
#include <time.h>
#include <unistd.h>

/* ---- NODELAY_CAP_HZ: pace sends on each TCP socket to at most cap Hz ---- */
#define CAP_MAX_FD 4096
static double cap_period = 0.0;            /* seconds between sends; 0 = no cap */
static int cap_parsed = 0;
static struct timespec cap_next[CAP_MAX_FD];   /* earliest time the next send may go */

static void cap_init(void)
{
    if (cap_parsed) return;
    cap_parsed = 1;
    const char *e = getenv("NODELAY_CAP_HZ");
    double hz = e ? atof(e) : 0.0;
    if (hz > 0.0) cap_period = 1.0 / hz;
    if (getenv("NODELAY_VERBOSE"))
        fprintf(stderr, "[libnodelay] NODELAY_CAP_HZ=%s -> %s\n", e ? e : "(unset)",
                cap_period > 0.0 ? "capped" : "no cap");
}

static void cap_reset(int fd)
{
    if (fd >= 0 && fd < CAP_MAX_FD) cap_next[fd].tv_sec = cap_next[fd].tv_nsec = 0;
}

static void pace(int fd)
{
    cap_init();
    if (cap_period <= 0.0 || fd < 0 || fd >= CAP_MAX_FD) return;
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    struct timespec *nx = &cap_next[fd];
    if (nx->tv_sec || nx->tv_nsec) {
        double wait = (nx->tv_sec - now.tv_sec) + (nx->tv_nsec - now.tv_nsec) * 1e-9;
        if (wait > 0.0) {
            struct timespec ts = { (time_t)wait, (long)((wait - (double)(time_t)wait) * 1e9) };
            while (nanosleep(&ts, &ts) == -1) { /* EINTR: finish the remainder */ }
            clock_gettime(CLOCK_MONOTONIC, &now);
        } else if (wait < -cap_period) {
            /* the loop was slower than the cap for a while: no credit is banked */
            *nx = now;
        }
    } else {
        *nx = now;
    }
    /* next allowed send: previous target + period, not wake-up + period */
    double t = nx->tv_sec + nx->tv_nsec * 1e-9 + cap_period;
    nx->tv_sec = (time_t)t;
    nx->tv_nsec = (long)((t - (double)(time_t)t) * 1e9);
}

static int is_tcp(int fd)
{
    int type = 0, dom = 0;
    socklen_t len = sizeof(type);
    if (getsockopt(fd, SOL_SOCKET, SO_TYPE, &type, &len) != 0 || type != SOCK_STREAM) return 0;
    len = sizeof(dom);
    return getsockopt(fd, SOL_SOCKET, SO_DOMAIN, &dom, &len) == 0 && (dom == AF_INET || dom == AF_INET6);
}

static void quickack(int fd)
{
    int one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_QUICKACK, &one, sizeof(one));
}

static void nodelay(int fd)
{
    if (fd < 0 || !is_tcp(fd)) return;
    cap_reset(fd);
    int one = 1;
    if (setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one)) == 0 && getenv("NODELAY_VERBOSE"))
        fprintf(stderr, "[libnodelay] TCP_NODELAY on fd %d\n", fd);
    quickack(fd);
}

int socket(int domain, int type, int protocol)
{
    static int (*real)(int, int, int) = NULL;
    if (!real) real = dlsym(RTLD_NEXT, "socket");
    int fd = real(domain, type, protocol);
    nodelay(fd);
    return fd;
}

int accept(int s, struct sockaddr *addr, socklen_t *addrlen)
{
    static int (*real)(int, struct sockaddr *, socklen_t *) = NULL;
    if (!real) real = dlsym(RTLD_NEXT, "accept");
    int fd = real(s, addr, addrlen);
    nodelay(fd);
    return fd;
}

int accept4(int s, struct sockaddr *addr, socklen_t *addrlen, int flags)
{
    static int (*real)(int, struct sockaddr *, socklen_t *, int) = NULL;
    if (!real) real = dlsym(RTLD_NEXT, "accept4");
    int fd = real(s, addr, addrlen, flags);
    nodelay(fd);
    return fd;
}

ssize_t recv(int fd, void *buf, size_t len, int flags)
{
    static ssize_t (*real)(int, void *, size_t, int) = NULL;
    if (!real) real = dlsym(RTLD_NEXT, "recv");
    ssize_t n = real(fd, buf, len, flags);
    if (n > 0 && is_tcp(fd)) quickack(fd);
    return n;
}

ssize_t recvfrom(int fd, void *buf, size_t len, int flags, struct sockaddr *src, socklen_t *srclen)
{
    static ssize_t (*real)(int, void *, size_t, int, struct sockaddr *, socklen_t *) = NULL;
    if (!real) real = dlsym(RTLD_NEXT, "recvfrom");
    ssize_t n = real(fd, buf, len, flags, src, srclen);
    if (n > 0 && is_tcp(fd)) quickack(fd);
    return n;
}

ssize_t read(int fd, void *buf, size_t count)
{
    static ssize_t (*real)(int, void *, size_t) = NULL;
    if (!real) real = dlsym(RTLD_NEXT, "read");
    ssize_t n = real(fd, buf, count);
    if (n > 0 && fd > 2 && is_tcp(fd)) quickack(fd);
    return n;
}

/* Send side: the re-arm that actually matters (see the header comment). */

ssize_t send(int fd, const void *buf, size_t len, int flags)
{
    static ssize_t (*real)(int, const void *, size_t, int) = NULL;
    if (!real) real = dlsym(RTLD_NEXT, "send");
    if (is_tcp(fd)) pace(fd);
    ssize_t n = real(fd, buf, len, flags);
    if (n > 0 && is_tcp(fd)) quickack(fd);
    return n;
}

ssize_t sendto(int fd, const void *buf, size_t len, int flags, const struct sockaddr *dst, socklen_t dstlen)
{
    static ssize_t (*real)(int, const void *, size_t, int, const struct sockaddr *, socklen_t) = NULL;
    if (!real) real = dlsym(RTLD_NEXT, "sendto");
    if (is_tcp(fd)) pace(fd);
    ssize_t n = real(fd, buf, len, flags, dst, dstlen);
    if (n > 0 && is_tcp(fd)) quickack(fd);
    return n;
}

ssize_t sendmsg(int fd, const struct msghdr *msg, int flags)
{
    static ssize_t (*real)(int, const struct msghdr *, int) = NULL;
    if (!real) real = dlsym(RTLD_NEXT, "sendmsg");
    if (is_tcp(fd)) pace(fd);
    ssize_t n = real(fd, msg, flags);
    if (n > 0 && is_tcp(fd)) quickack(fd);
    return n;
}

ssize_t write(int fd, const void *buf, size_t count)
{
    static ssize_t (*real)(int, const void *, size_t) = NULL;
    if (!real) real = dlsym(RTLD_NEXT, "write");
    if (fd > 2 && is_tcp(fd)) pace(fd);
    ssize_t n = real(fd, buf, count);
    if (n > 0 && fd > 2 && is_tcp(fd)) quickack(fd);
    return n;
}

ssize_t writev(int fd, const struct iovec *iov, int iovcnt)
{
    static ssize_t (*real)(int, const struct iovec *, int) = NULL;
    if (!real) real = dlsym(RTLD_NEXT, "writev");
    if (fd > 2 && is_tcp(fd)) pace(fd);
    ssize_t n = real(fd, iov, iovcnt);
    if (n > 0 && fd > 2 && is_tcp(fd)) quickack(fd);
    return n;
}
