@echo off

md Output

del Output/*

copy simple_peripheral_cc2640r2lp_oad_offchip_app\FlashROM\simple_peripheral_cc2640r2lp_oad_offchip_app_FlashROM_oad.bin Output\application_oad.bin

copy simple_peripheral_cc2640r2lp_oad_offchip_app\FlashROM\simple_peripheral_cc2640r2lp_oad_offchip_app_FlashROM_oad_merged.bin Output\application_stack.bin

bin2hex.py Output\application_stack.bin Output\application_stack.hex

hexmerge.py Output\application_stack.hex bim_oad_offchip_cc2640r2lp_app\FlashOnly\bim_oad_offchip_cc2640r2lp_app.hex --output=Output\merge.hex --overlap=replace

del Output\application_stack.bin Output\application_stack.hex