#ifndef _FS_CVT_H
#define _FS_CVT_H

//#define USE_FATFS

#ifdef USE_FATFS



#define O_RDONLY	FA_READ
#define O_WRONLY	FA_WRITE
#define O_RDWR		FA_READ|FA_WRITE
#define O_CREAT	    FA_OPEN_ALWAYS
#define O_TRUNC     FA_CREATE_ALWAYS 





#else

#define O_RDONLY	0x01
#define O_WRONLY	0x02
#define O_RDWR		0x04
#define O_CREAT	    0x08
#define O_TRUNC     0x10

//#define FIL  void




#endif //USE_FATFS




typedef unsigned int mm_segment_t;
typedef unsigned int loff_t;
struct file;
struct file_operations {
	int (*read) (struct file *, char  *, unsigned int, loff_t *);
	int (*write) (struct file *, char  *, unsigned int, loff_t *);
};


struct file
{
#ifdef USE_FATFS    
    FIL fil;
#endif    
    struct file_operations op;
    struct file_operations	*f_op;
	loff_t			f_pos;    
};
int filp_write(struct file *file, char  *buff, unsigned int size, loff_t *loff);
int filp_read(struct file *file, char  *buff, unsigned int size, loff_t *loff);
struct file *filp_open(const char *filename, int flags, int mode);
int filp_close(struct file *filp, int id);
int filp_seek(struct file *filp,unsigned int offset);

#define KERNEL_DS	((mm_segment_t) { 0UL })
#define USER_DS		((mm_segment_t) { -0x40000000000UL })


#define get_fs()  (current->addr_limit)
#define get_ds()  (KERNEL_DS)
#define set_fs(x) (current->addr_limit = (x))




#endif
