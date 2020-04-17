// ./../../../wiced_btsdk/dev-kit/baselib/20819A1/make/scripts/wiced-gen-lib-installer.pl
// args: 
// output: C:/Users/A62584/mtb_2_0_Mesh_Workshop/BlinkLEDandPrint_Key/template/empty_wiced_bt/build/CYW920819EVB-02/generated/lib_installer.c
#include <stdint.h>
typedef struct tag_PATCH_TABLE_ENTRY_t {
	uint32_t breakout;
	uint32_t replacement;
} PATCH_TABLE_ENTRY_t;
void patch_autoInstall(uint32_t old_address, uint32_t new_address);
void patch_autoReplace(uint32_t breakout_address, uint32_t replacement);
void patch_autoReplaceData(uint32_t breakout_address, uint32_t replacement);
void install_libs(void);

void install_libs(void)
{
}
