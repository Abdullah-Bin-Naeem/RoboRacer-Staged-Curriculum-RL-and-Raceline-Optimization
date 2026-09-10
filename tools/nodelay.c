/* libnodelay.so -- LD_PRELOAD shim that sets TCP_NODELAY on every TCP socket a
 * process creates (socket) or accepts (accept, accept4).
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

static void nodelay(int fd)
{
    if (fd < 0) return;
    int type = 0, dom = 0;
    socklen_t len = sizeof(type);
    if (getsockopt(fd, SOL_SOCKET, SO_TYPE, &type, &len) != 0 || type != SOCK_STREAM) return;
    len = sizeof(dom);
    if (getsockopt(fd, SOL_SOCKET, SO_DOMAIN, &dom, &len) != 0 || (dom != AF_INET && dom != AF_INET6)) return;
    int one = 1;
    if (setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one)) == 0 && getenv("NODELAY_VERBOSE"))
        fprintf(stderr, "[libnodelay] TCP_NODELAY on fd %d\n", fd);
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
