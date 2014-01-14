thu muc dsPIC_Zigbee_R&D_by_Lab411: su dung de nap cho router-emboard(dspic33) cua khu cham soc lan va canh bao chay rung. router - emboard nay ket noi voi bo nhung IDEA.

1. Mot so doan code can chu y:
- doan code lien quan toi viec chon mang nao de gia nhap:
 tim o cho: case NLME_NETWORK_DISCOVERY_confirm:
- doan code lien quan toi viec gan lai dia chi mang:
 tim o cho: case NLME_PERMIT_JOINING_confirm:
- doan code lien quan toi viec xu ly du lieu nhan ve:
 tim o cho: case APSDE_DATA_indication:
- doan code lien quan toi viec truyen lenh toi cac node mang khac:
 tim o cho: case NO_PRIMITIVE:
- doan code xu ly noi tai ben trong cam bien:
 + hardware init.
 + xu ly ngat timer.
 + xu ly ngat nhan UART.
 + xu ly ngat ADC.
 + xu ly ngat ngoai.
