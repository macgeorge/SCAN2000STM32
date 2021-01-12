# SCAN2000-20 ScanCard for DMM6500 using an STM32 microcontrller
Please see the [project page](https://christidis.info/index.php/personal-projects/keithley-dmm-scanner-card-with-ssr) for more details

The repository includes
1. The schematics of the Scanner Card
2. The Bill of Materials
3. The Gerber files to build a Scanner Card
4. The STM32G070 code as well as the executable

The scanner card has been tested with a Keithley DMM6500 multimeter. It is also compatible with the Keithley 2000series multimeter. Jumper links can be used to change the identity of the scanner card (current configuration: 2000-20).

The solid state relays used are 160V capable (recommended 160V, absolute maximum 200V). Please do not measure voltages higher than that. Creepage between the HV circuit and the LV (control circuit) is adequate for 300V. 

![ScanCardImage](https://raw.githubusercontent.com/macgeorge/SCAN2000STM32/master/5%20-%20Misc/2000SCAN-3DPCB%20top.jpg)
