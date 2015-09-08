#include "fs_cvt.h"
#include "usbh_config.h"
#include "memory.h"
#include "string.h"
#include "RT2870STA.c"


int filp_write(struct file *file, char  *buff, unsigned int size, loff_t *loff);
int filp_read(struct file *file, char  *buff, unsigned int size, loff_t *loff);


struct file *filp_open(const char *filename, int flags, int mode)
{
    struct file *file;
    
    file = kcalloc(1,sizeof(struct file),0);
    if(file == NULL)
    {
        USBH_DBG("filp_open kcalloc file Failed\r\n");
        return NULL;
    }

    file->f_pos = 0;
    file->op.read = filp_read;
    file->op.write= filp_write;
    file->f_op = &file->op;
    
#ifdef USE_FATFS 
    if(f_open(&file->fil, filename, flags) != FR_OK)
    {
        USBH_DBG("filp_open filp_open %s Failed\r\n",filename);        
        kfree(file);
        return NULL;
    }
    
    return file;
#else
    return file;
#endif
}

int filp_close(struct file *filp, int id)
{   
#ifdef USE_FATFS  
    f_close(&filp->fil);
#endif
    kfree(filp);
    return 0;
}

int filp_seek(struct file *filp,unsigned int offset)
{
#ifdef USE_FATFS  
    f_lseek(&filp->fil,offset);
#endif  
    return 0;
}


int filp_read(struct file *file, char  *buff, unsigned int size, loff_t *loff)
{
    unsigned int numOfReadBytes;    
    if(!file)
    {
        USBH_DBG("filp_read Failed file is NULL\r\n");
        return 0;
    }
        
#ifdef USE_FATFS   
    FRESULT res;   
    
    res = f_read(&file, buff, size, (void *)&numOfReadBytes);

    if((numOfReadBytes == 0) || (res != FR_OK)) /*EOF or Error*/
    {
        USBH_DBG("filp_read Failed numOfReadBytes:%d\r\n",numOfReadBytes);        
        return 0;
    }

    return numOfReadBytes;
#else
    numOfReadBytes = sizeof(RT2870STA_dat) - file->f_pos;
    numOfReadBytes = (size < numOfReadBytes)?size:numOfReadBytes;
    USBH_TRACE("filp_read NOT_USE_FATFS,return data in const data numOfReadBytes:%d\r\n",numOfReadBytes);

    if(numOfReadBytes)
    {
        memcpy(buff,RT2870STA_dat + file->f_pos,numOfReadBytes);
        file->f_pos += numOfReadBytes;
    }
    
    return numOfReadBytes;
#endif    
}


int filp_write(struct file *file, char  *buff, unsigned int size, loff_t *loff)
{
    if(!file)
    {
        USBH_DBG("filp_write Failed file is NULL\r\n");
        return 0;
    }
    
#ifdef USE_FATFS      
    UINT bytesWritten;
    FRESULT res;
    
    res= f_write (&file->fil, buff, size, (void *)&bytesWritten);   

    if((bytesWritten == 0) || (res != FR_OK)) /*EOF or Error*/
    {
        USBH_DBG("filp_write Failed bytesWritten:%d\r\n",bytesWritten);
        return 0;
    }
    return bytesWritten;

#else
    USBH_TRACE("filp_write not permitted,NOT_USE_FATFS\r\n");

    return size;
#endif      
    
    
}









