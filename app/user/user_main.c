#include "c_stdio.h"
#include "vfs.h"

void stm32node_init(void)
{
#ifdef BUILD_FATFS
    if (!vfs_mount("/SD1", 0)) {
        // Failed to mount -- try reformat
				dbg_printf("Formatting file system. Please wait...\n");
        if (!vfs_format()) {
            dbg_printf( "\n*** ERROR ***: unable to format. FS might be compromised.\n" );
            dbg_printf( "It is advised to re-flash the NodeMCU image.\n" );
        }
        // Note that fs_format leaves the file system mounted
    }
    // test_spiffs();
#endif
		
}
