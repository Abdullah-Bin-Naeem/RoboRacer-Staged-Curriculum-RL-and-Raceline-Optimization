/* libnodelay.so -- LD_PRELOAD shim that sets TCP_NODELAY on every TCP socket a
 * process creates (socket) or accepts (accept, accept4), and re-arms
 * TCP_QUICKACK after every recv/recvfrom/read on a TCP socket.
 *
 * The two halves of one deadlock. NODELAY stops THIS process holding a small
 * write until the peer acknowledges the previous one; QUICKACK stops this
 * process delaying its acknowledgment of the PEER's small write, which is the
 * half we can fix when the peer is the simulator binary. Linux clears
 * QUICKACK on its own, hence re-arming on every read.
 *
 * Measured on the development laptop: neither changes the rate (18.5 Hz either
 * way), because there the simulator's frame period is the limit. On a fast
 * machine the frame is short and the socket deadlock is what caps the loop,
 * as another team measured (10 Hz at 60 fps); that is the case these are for.
 *
 * Why: the simulator emits telemetry only in reply to the bridge's message,
 * over a websocket on loopback. Each websocket message is written as small
 * pieces; Nagle's algorithm holds a small write until the previous one is
 * acknowledged, and the receiver's delayed ACK holds that acknowledgment for
 * up to 40 ms. Two such waits per round trip cap the loop at 10-20 Hz on any
 * machine, which is exactly what every rate measurement here showed and what
 * another team traced to this cause. TCP_NODELAY sends small writes at once.
 *
 * Applied from bridge.launch.py (tcp_nodelay:=true) as the bridge process's
 * environment, so the devkit package stays unmodified. It can also be put in
 * front of the simulator binary for a local test of the other direction.
 *
 *   gcc -shared -fPIC -O2 -o tools/libnodelay.so tools/nodelay.c -ldl
 *   NODELAY_VERBOSE=1 to log each socket it touches.
 */
#define _GNU_SOURCE
#include <dlfcn.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

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
