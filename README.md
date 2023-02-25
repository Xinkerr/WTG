# WTG

**soc:** CC2640R2F

**IDE:** TI CSS 11.1.0

**SDK:** simplelink_cc2640r2_sdk_1.50.00.58

## NOTE

1."CC2640R2_LAUNCHXL.h" move to simplelink_cc2640r2_sdk_1_50_00_58\source\ti\blestack\boards\CC2640R2_LAUNCHXL

2.merge.hex = bim.hex + application + blestack.hex

3.If you want to run hex_merge.bat, you need install python and IntelHex.

install IntelHex:

python.exe -m pip install -i https://pypi.tuna.tsinghua.edu.cn/simple IntelHex

Add python's Scripts directory to PATH.



## V1.04

1.广播间隔改为850ms

2.广播报文增加传感器数据和电池电压

3.增加10分钟任务，采集电池电压和更新广播报文数据