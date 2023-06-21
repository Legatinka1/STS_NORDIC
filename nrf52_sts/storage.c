#include "storage.h"

#include "Internal/GL_SDC.h"
#include "ff.h"
#include <string.h>

static FATFS storage_fs;
static DIR storage_dir;
static FILINFO storage_fno;
static FIL storage_file;
static int storage_mounted = 0;
static int storage_dir_opened = 0;
static int storage_file_opened_for_write = 0;
static int storage_file_opened_for_read = 0;

int storage_is_mounted(void)
{
    return storage_mounted;
}

int storage_check(void)
{
    if (storage_mounted)
        return 1;
    storage_mounted = f_mount(&storage_fs, "", 1) ? 0 : 1;
    if (!storage_mounted)
    {
        gl_sdc_reinit();
        storage_mounted = f_mount(&storage_fs, "", 1) ? 0 : 1;
    }
    if (!storage_mounted)
    {
        storage_dir_opened = 0;
        storage_file_opened_for_write = 0;
        storage_file_opened_for_read = 0;
    }
    return storage_mounted;
}

int storage_get_space_in_kilobytes(unsigned long *kbtotal, unsigned long *kbfree)
{
    FATFS *fs;
    unsigned long fre_clust;
    if (kbtotal)
        *kbtotal = 0;
    if (kbfree)
        *kbfree = 0;
    if (!(storage_dir_opened || storage_file_opened_for_write || storage_file_opened_for_read))
        storage_mounted = 0;
    if (!storage_check())
        return 0;
    if (f_getfree("", &fre_clust, &fs))
    {
        storage_mounted = 0;
        return 0;
    }
    if (kbtotal)
        *kbtotal = ((fs->n_fatent - 2) * fs->csize) >> 1;
    if (kbfree)
        *kbfree = (fre_clust * fs->csize) >> 1;
    return 1;
}

int storage_dir_close(void)
{
    if (!storage_dir_opened)
        return 1;
    storage_dir_opened = 0;
    if (!f_closedir(&storage_dir))
        return 1;
    storage_mounted = 0;
    return 0;
}

int storage_dir_open(void)
{
    if (!storage_dir_close())
        return 0;
    if (!storage_check())
        return 0;
    if (f_opendir(&storage_dir, ""))
        storage_mounted = 0;
    else
        storage_dir_opened = 1;
    return storage_dir_opened;
}

int storage_dir_read(char *name, unsigned long *siz)
{
    if (name)
        *name = 0;
    if (siz)
        *siz = 0;
    if (!storage_dir_opened)
        return 0;
    if (f_readdir(&storage_dir, &storage_fno))
    {
        storage_mounted = 0;
        storage_dir_opened = 0;
    }
    if (name)
        strcpy(name, storage_fno.fname);
    if (siz)
        *siz = storage_fno.fsize;
    if (!(*name))
        storage_dir_close();
    return storage_dir_opened;
}

int storage_file_get_size(const char *name, unsigned long *siz)
{
    FRESULT fr;
    FILINFO fno;
    if (siz)
        *siz = 0;
    if (!(name ? *name : 0))
        return 0;
    if (!storage_check())
        return 0;
    fr = f_stat(name, &fno);
    if (fr == FR_NO_FILE)
        return 0;
    if (fr)
    {
        storage_mounted = 0;
        return 0;
    }
    if (siz)
        *siz = fno.fsize;
    return 1;
}

int storage_file_close(void)
{
    if ((!storage_file_opened_for_write) && (!storage_file_opened_for_read))
        return 1;
    storage_file_opened_for_write = 0;
    storage_file_opened_for_read = 0;
    if (!f_close(&storage_file))
        return 1;
    storage_mounted = 0;
    return 0;
}

int storage_file_open_for_write(const char *name)
{
    FRESULT fr;
    if (!storage_file_close())
        return 0;
    if (!(name ? *name : 0))
        return 0;
    if (!storage_check())
        return 0;
    fr = f_open(&storage_file, name, FA_WRITE | FA_CREATE_ALWAYS);
    if (fr == FR_DENIED)
        return 0;
    if (fr)
        storage_mounted = 0;
    else
        storage_file_opened_for_write = 1;
    return storage_file_opened_for_write;
}

int storage_file_open_for_read(const char *name, unsigned long pos)
{
    FRESULT fr;
    if (!storage_file_close())
        return 0;
    if (!(name ? *name : 0))
        return 0;
    if (!storage_check())
        return 0;
    fr = f_open(&storage_file, name, FA_READ);
    if (fr == FR_NO_FILE)
        return 0;
    if (fr)
    {
        storage_mounted = 0;
        return 0;
    }
    if (!pos)
    {
        storage_file_opened_for_read = 1;
        return 1;
    }
    if (f_lseek(&storage_file, pos))
        storage_mounted = 0;
    else
        storage_file_opened_for_read = 1;
    return storage_file_opened_for_read;
}

int storage_file_write(const void *data, unsigned int len)
{
    unsigned int bytes_written;
    if (!storage_file_opened_for_write)
        return 0;
    if (!data)
        return 0;
    if (!len)
        return 1;
    if (!f_write(&storage_file, data, len, &bytes_written))
        return 1;
    storage_mounted = 0;
    storage_file_opened_for_write = 0;
    return 0;
}

int storage_file_read(void *data, unsigned int len, unsigned int *rlen)
{
    unsigned int bytes_read;
    unsigned int *br = rlen ? rlen : &bytes_read;
    *br = 0;
    if (!storage_file_opened_for_read)
        return 0;
    if (!data)
        return 0;
    if (!len)
        return 1;
    if (!f_read(&storage_file, data, len, br))
        return 1;
    storage_mounted = 0;
    storage_file_opened_for_read = 0;
    return 0;
}

int storage_file_delete(const char *name)
{
    FRESULT fr;
    if (!storage_file_close())
        return 0;
    if (!(name ? *name : 0))
        return 0;
    if (!storage_check())
        return 0;
    if ((*name) != '*')
    {
        fr = f_unlink(name);
        if ((fr == FR_INVALID_NAME) || (fr == FR_DENIED))
            return 0;
    }
    else
    {
        DIR dir;
        FILINFO fno;
        if (!storage_dir_close())
            return 0;
        fr = f_opendir(&dir, "");
        if (!fr)
        {
            fr = f_readdir(&dir, &fno);
            while ((!fr) && fno.fname[0])
            {
                if(!(fno.fattrib & AM_DIR))
                    f_unlink(fno.fname);
                fr = f_readdir(&dir, &fno);
            }
            fr = f_closedir(&dir);
        }
    }
    if (fr)
    {
        storage_mounted = 0;
        return 0;
    }
    return 1;
}

