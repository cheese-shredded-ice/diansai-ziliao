#include "zf_common_headfile.h"
void Mykey_Init(void)
{
    key_init(10);
    key_clear_all_state();
}
