#ifndef THREAD_DEFS_HPP

#define MOCAP_THREAD_HZ 300
#define MOCAP_THREAD_TOUT 2
#define MOCAP_THREAD_TYPE FIFO
#define MOCAP_THREAD_PRI 99

#define VPE_THREAD_HZ 40
#define VPE_THREAD_TOUT 2
#define VPE_THREAD_TYPE FIFO
#define VPE_THREAD_PRI 98

#define READ_THREAD_HZ 40
#define READ_THREAD_TOUT 2
#define READ_THREAD_TYPE FIFO
#define READ_THREAD_PRI 30

#define WRITE_THREAD_HZ 4
#define WRITE_THREAD_TOUT 2
#define WRITE_THREAD_TYPE OTHER
#define WRITE_THREAD_PRI 0

#define PRINTF_THREAD_HZ 20
#define PRINTF_THREAD_TOUT 2
#define PRINTF_THREAD_TYPE OTHER
#define PRINTF_THREAD_PRI 0

#define SYS_THREAD_HZ 1
#define SYS_THREAD_TOUT 2
#define SYS_THREAD_TYPE FIFO
#define SYS_THREAD_PRI 80

#endif // !THREAD_DEFS_HPP