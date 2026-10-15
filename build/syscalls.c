/* 最小 syscall 桩：让 newlib 的 printf 能工作（仅诊断固件使用） */
#include <sys/stat.h>
#include <errno.h>
#include <reent.h>
#include <sys/time.h>

extern int  _write_impl(int fd, const char *buf, int len);

int _write(int fd, char *ptr, int len)
{
    (void)fd;
    return _write_impl(1, ptr, len);
}

int _read(int fd, char *ptr, int len)   { (void)fd; (void)ptr; (void)len; return 0; }
int _close(int fd)                      { (void)fd; return -1; }
int _fstat(int fd, struct stat *st)     { (void)fd; st->st_mode = S_IFCHR; return 0; }
int _isatty(int fd)                     { (void)fd; return 1; }
int _lseek(int fd, int off, int dir)    { (void)fd; (void)off; (void)dir; return 0; }
int _getpid(void)                       { return 1; }
int _kill(int pid, int sig)             { (void)pid; (void)sig; errno = EINVAL; return -1; }
void _exit(int code)                    { (void)code; for(;;); }

int _gettimeofday(struct timeval *tp, void *tz)
{
    (void)tz;
    if(tp){ tp->tv_sec = 0; tp->tv_usec = 0; }
    return 0;
}

/* newlib 自己会提供 _xxx_r 包装（内部调用下面的 _xxx），
 * 所以这里【不要】再定义 _close_r / _write_r 等，否则多重定义冲突。 */

/* 堆起点：放在 bss 之后 */
extern char _end;
void *_sbrk(int incr)
{
    static char *heap = 0;
    char *prev;
    if(heap == 0) heap = &_end;
    prev = heap;
    heap += incr;
    return prev;
}


