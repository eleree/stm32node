#include "fatfs.h"

extern FATFS SDFatFS;    /* File system object for SD logical drive */
FIL MyFile;			/* Pointer to the file object */
uint8_t wtext[] = "This is STM32 working with FatFs"; /* File write buffer */
uint8_t rtext[100];
void FileTest(void)
{
    FRESULT res;
    uint32_t byteswritten, bytesread;                     /* File write/read counts */
  

    if(f_open(&MyFile, "0:/STM32.TXT", FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
    {
        /* 'STM32.TXT' file Open for write Error */
        Error_Handler();
    }
  else
  {
            res = f_write(&MyFile, wtext, sizeof(wtext), (void *)&byteswritten);
            if((byteswritten == 0) || (res != FR_OK))
            {
                /* 'STM32.TXT' file Write or EOF Error */
                Error_Handler();
            }
            else
            {
                f_close(&MyFile);

                /*##-7- Open the text file object with read access ###############*/
                if(f_open(&MyFile, "0:/STM32.TXT", FA_READ) != FR_OK)
                {
                    /* 'STM32.TXT' file Open for read Error */
                    Error_Handler();
                }
                else
                {
                    res = f_read(&MyFile, rtext, sizeof(rtext), (UINT*)&bytesread);

                    if((bytesread == 0) || (res != FR_OK))
                    {
                        /* 'STM32.TXT' file Read or EOF Error */
                        Error_Handler();
                    }
                    else
                    {
                        f_close(&MyFile);

                        /*##-10- Compare read data with the expected data ############*/
                        if((bytesread != byteswritten))
                        {
                            /* Read data is different from the expected data */
                            Error_Handler();
                        }
                        else
                        {
                            /* Success of the demo: no error occurrence */
                        }
                    }
                }
            }
  }
}


void FatFsTestTask(void const * argument)
{
  /* init code for FATFS */
  MX_FATFS_Init();

  /* USER CODE BEGIN 5 */
	FRESULT ret =f_mount(&SDFatFS, "0:/", 0);
	if(ret!= FR_OK)
	{
		while(1);
	}
        
	FileTest();
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */ 
}


void FatFsTest(void)
{
  osThreadDef(defaultTask, FatFsTestTask, osPriorityNormal, 0, 256);
  osThreadCreate(osThread(defaultTask), NULL);
}
