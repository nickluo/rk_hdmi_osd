/*
 * hw::DmaBuf - minimal dma-heap allocation helper.
 *
 * Same allocation pattern as Compositor's internal scratches
 * (compositor.cpp ensure_y8_scratch_), factored out for callers that need
 * their own dmabuf - e.g. the bridge's no-camera bench mode gray frame.
 * Cached heap: CPU writes must be bracketed with dmabuf_sync(RW) /
 * dmabuf_sync_end(), the RGA driver does not maintain caches for us.
 */
#pragma once

#include <cstddef>

namespace hw {

struct DmaBuf {
    int    fd = -1;
    void  *va = nullptr;
    size_t size = 0;

    bool valid() const { return fd >= 0 && va != nullptr; }
};

/* allocate + mmap `len` bytes on the given dma-heap */
bool dmabuf_alloc(DmaBuf &b, size_t len,
                  const char *heap_path = "/dev/dma_heap/system");
void dmabuf_free(DmaBuf &b);

/* DMA_BUF_IOCTL_SYNC bracket around a CPU write burst */
bool dmabuf_sync_begin(DmaBuf &b);
bool dmabuf_sync_end(DmaBuf &b);

} // namespace hw
