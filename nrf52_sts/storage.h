#ifndef _STORAGE_H_
#define _STORAGE_H_


#ifdef __cplusplus
extern "C" {
#endif


int storage_is_mounted(void);
int storage_check(void);

int storage_get_space_in_kilobytes(unsigned long *kbtotal, unsigned long *kbfree);

int storage_dir_open(void);
int storage_dir_read(char *name, unsigned long *siz);
int storage_dir_close(void);

int storage_file_get_size(const char *name, unsigned long *siz);
int storage_file_open_for_write(const char *name);
int storage_file_write(const void *data, unsigned int len);
int storage_file_open_for_read(const char *name, unsigned long pos);
int storage_file_read(void *data, unsigned int len, unsigned int *rlen);
int storage_file_close(void);
int storage_file_delete(const char *name);


#ifdef __cplusplus
}
#endif

#endif /* _STORAGE_H_ */
