# TUINO LORA

The base code is added from https://github.com/gimasi/TUINO_ONE/tree/master/tuino_libs/gmx/gmx_lr/tuino_lora 
Modifications done on payload content.

From lines 172-174:
// Transmit Data - as HEX String
gmxLR_TXData("0A");
// possible data: 0A=10m, 14=20m, B2=50m, 64=100m, 96=150m, C8=200m, FF=other
