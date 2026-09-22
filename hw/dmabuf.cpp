#include "dmabuf.h"

#include <cstdio>
#include <cstdlib>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/dma-heap.h>
#include <linux/dma-buf.h>

namespace hw {
namespace {

int xioctl(int fd, unsigned long req, void *arg)
{
    int r;
    do { r = ioctl(fd, req, arg); } while (r < 0 && errno == EINTR);
    return r;
}

} // namespace

bool dmabuf_alloc(DmaBuf &b, size_t len, const char *heap_path)
{
    dmabuf_free(b);
    int heap = open(heap_path, O_RDWR);
    if (heap < 0) {
        fprintf(stderr, "[E] dmabuf: open %s: %s\n", heap_path, strerror(errno));
        return false;
    }
    struct dma_heap_allocation_data ad = {};
    ad.len = len;
    ad.fd_flags = O_RDWR | O_CLOEXEC;
    int r = xioctl(heap, DMA_HEAP_IOCTL_ALLOC, &ad);
    close(heap);
    if (r < 0) {
        fprintf(stderr, "[E] dmabuf: DMA_HEAP_IOCTL_ALLOC(%zu): %s\n",
                len, strerror(errno));
        return false;
    }
    b.fd = ad.fd;
    b.size = len;
    b.va = mmap(nullptr, len, PROT_READ | PROT_WRITE, MAP_SHARED, b.fd, 0);
    if (b.va == MAP_FAILED) {
        fprintf(stderr, "[E] dmabuf: mmap: %s\n", strerror(errno));
        b.va = nullptr;
        close(b.fd);
        b.fd = -1;
        b.size = 0;
        return false;
    }
    return true;
}

void dmabuf_free(DmaBuf &b)
{
    if (b.va) munmap(b.va, b.size);
    if (b.fd >= 0) close(b.fd);
    b.va = nullptr;
    b.fd = -1;
    b.size = 0;
}

bool dmabuf_sync_begin(DmaBuf &b)
{
    if (b.fd < 0) return false;
    struct dma_buf_sync s = {};
    s.flags = DMA_BUF_SYNC_RW | DMA_BUF_SYNC_START;
    return xioctl(b.fd, DMA_BUF_IOCTL_SYNC, &s) == 0;
}

bool dmabuf_sync_end(DmaBuf &b)
{
    if (b.fd < 0) return false;
    struct dma_buf_sync s = {};
    s.flags = DMA_BUF_SYNC_RW | DMA_BUF_SYNC_END;
    return xioctl(b.fd, DMA_BUF_IOCTL_SYNC, &s) == 0;
}

} // namespace hw
