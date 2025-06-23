//#include "py32f003x8.h"
#include "option_bytes.h"


void SetOptionBytes(void)
{
    // Unlock FLASH and Option Bytes
    HAL_FLASH_Unlock();
    HAL_FLASH_OB_Unlock();

    FLASH_OBProgramInitTypeDef OBInit;
    OBInit.OptionType     = OPTIONBYTE_USER | OPTIONBYTE_RDP;
    OBInit.RDPLevel       = OB_RDP_LEVEL_0;       // 0xAA
    OBInit.USERType       = OB_USER_ALL;
    //OBInit.USERConfig     = OB_WDG_HW | OB_STOP_RST | OB_STDBY_RST;
		OBInit.USERConfig 		= OB_BOR_LEVEL_1p7_1p8 | OB_RESET_MODE_RESET | OB_IWDG_SW | OB_WWDG_SW | OB_BOOT1_SYSTEM;
		

	
    if (HAL_FLASH_OBProgram(&OBInit) != HAL_OK) {
        // Error handling
    }

    // Launch option byte loading
    if (HAL_FLASH_OB_Launch() != HAL_OK) {
        // Error handling
    }

    HAL_FLASH_OB_Lock();
    HAL_FLASH_Lock();
}
