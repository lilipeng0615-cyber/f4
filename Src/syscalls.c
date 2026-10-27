#include <errno.h>
#include <sys/types.h>
#include <unistd.h>

/*
 * This firmware uses SEGGER RTT directly for logging and has no file I/O.
 * Keep unsupported newlib calls explicit instead of using libnosys warning
 * stubs. These weak definitions preserve the libnosys failure behavior and
 * can be replaced by a UART/RTT/filesystem implementation when needed.
 */
__attribute__((weak)) int _close(int fd)
{
    (void)fd;
    errno = ENOSYS;
    return -1;
}

__attribute__((weak)) off_t _lseek(int fd, off_t offset, int whence)
{
    (void)fd;
    (void)offset;
    (void)whence;
    errno = ENOSYS;
    return (off_t)-1;
}

__attribute__((weak)) ssize_t _read(int fd, void *buffer, size_t length)
{
    (void)fd;
    (void)buffer;
    (void)length;
    errno = ENOSYS;
    return -1;
}

__attribute__((weak)) ssize_t _write(int fd, const void *buffer, size_t length)
{
    (void)fd;
    (void)buffer;
    (void)length;
    errno = ENOSYS;
    return -1;
}
