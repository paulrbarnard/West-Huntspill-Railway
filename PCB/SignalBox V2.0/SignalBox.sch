EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 2
Title "West Huntspill Railway Signalbox Module"
Date "2021-07-30"
Rev "V2.2"
Comp ""
Comment1 ""
Comment2 "creativecommons.org/licenses/by/4.0/"
Comment3 "License: CC BY 4.0"
Comment4 "Author: Paul Barnard"
$EndDescr
$Comp
L power:+5V #PWR04
U 1 1 609422C1
P 2250 3600
F 0 "#PWR04" H 2250 3450 50  0001 C CNN
F 1 "+5V" H 2265 3773 50  0000 C CNN
F 2 "" H 2250 3600 50  0001 C CNN
F 3 "" H 2250 3600 50  0001 C CNN
	1    2250 3600
	1    0    0    -1  
$EndComp
$Comp
L Signalbox:820R R3
U 1 1 6094B87B
P 6700 4500
F 0 "R3" H 6770 4546 50  0000 L CNN
F 1 "820R" H 6770 4455 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 6630 4500 50  0001 C CNN
F 3 "https://docs.rs-online.com/8878/0900766b8157adeb.pdf" H 6700 4500 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/through-hole-fixed-resistors/7077669/" H 6700 4500 50  0001 C CNN "Source"
	1    6700 4500
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR011
U 1 1 6094D159
P 6700 4200
F 0 "#PWR011" H 6700 4050 50  0001 C CNN
F 1 "+5V" H 6715 4373 50  0000 C CNN
F 2 "" H 6700 4200 50  0001 C CNN
F 3 "" H 6700 4200 50  0001 C CNN
	1    6700 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	2950 4450 3300 4450
Wire Wire Line
	6700 4350 6700 4200
$Comp
L Signalbox:DIPSwitch J2
U 1 1 6091E2FE
P 3500 7350
F 0 "J2" H 3472 7232 50  0000 R CNN
F 1 "Master/Slave" H 3472 7323 50  0000 R CNN
F 2 "Button_Switch_THT:SW_DIP_SPSTx01_Slide_6.7x4.1mm_W7.62mm_P2.54mm_LowProfile" H 3500 7350 50  0001 C CNN
F 3 "https://docs.rs-online.com/383f/0900766b80dbe1c6.pdf" H 3500 7350 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/dip-sip-switches/6821058/" H 3500 7350 50  0001 C CNN "Source"
	1    3500 7350
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR09
U 1 1 60926FF5
P 3200 7500
F 0 "#PWR09" H 3200 7250 50  0001 C CNN
F 1 "GND" H 3205 7327 50  0000 C CNN
F 2 "" H 3200 7500 50  0001 C CNN
F 3 "" H 3200 7500 50  0001 C CNN
	1    3200 7500
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 7500 3200 7350
Wire Wire Line
	3200 7350 3300 7350
$Comp
L Signalbox:10K R1
U 1 1 60929019
P 3100 6600
F 0 "R1" H 3170 6646 50  0000 L CNN
F 1 "10K" H 3170 6555 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 3030 6600 50  0001 C CNN
F 3 "https://docs.rs-online.com/76ca/0900766b8157ae01.pdf" H 3100 6600 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/through-hole-fixed-resistors/7077745/" H 3100 6600 50  0001 C CNN "Source"
	1    3100 6600
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR08
U 1 1 6092A0CE
P 3100 6350
F 0 "#PWR08" H 3100 6200 50  0001 C CNN
F 1 "+5V" H 3115 6523 50  0000 C CNN
F 2 "" H 3100 6350 50  0001 C CNN
F 3 "" H 3100 6350 50  0001 C CNN
	1    3100 6350
	1    0    0    -1  
$EndComp
Wire Wire Line
	3100 6450 3100 6350
$Comp
L power:GND #PWR022
U 1 1 6091FDFC
P 4400 4450
F 0 "#PWR022" H 4400 4200 50  0001 C CNN
F 1 "GND" H 4405 4277 50  0000 C CNN
F 2 "" H 4400 4450 50  0001 C CNN
F 3 "" H 4400 4450 50  0001 C CNN
	1    4400 4450
	1    0    0    -1  
$EndComp
NoConn ~ 2350 3750
$Sheet
S 1300 1150 1000 700 
U 6092E635
F0 "5V Supply" 50
F1 "5V Supply.sch" 50
F2 "6V-24V" I L 1300 1500 50 
F3 "5V" O R 2300 1500 50 
F4 "GND" O R 2300 1700 50 
$EndSheet
$Comp
L power:+5V #PWR06
U 1 1 6093E746
P 2650 1350
F 0 "#PWR06" H 2650 1200 50  0001 C CNN
F 1 "+5V" H 2665 1523 50  0000 C CNN
F 2 "" H 2650 1350 50  0001 C CNN
F 3 "" H 2650 1350 50  0001 C CNN
	1    2650 1350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR07
U 1 1 609CBFAD
P 2650 1850
F 0 "#PWR07" H 2650 1600 50  0001 C CNN
F 1 "GND" H 2655 1677 50  0000 C CNN
F 2 "" H 2650 1850 50  0001 C CNN
F 3 "" H 2650 1850 50  0001 C CNN
	1    2650 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	2300 1700 2650 1700
Wire Wire Line
	2650 1700 2650 1850
$Comp
L Pauls~Parts:RS_Pro_2_Way_PCB_terminal_block_2.54mm J1
U 1 1 609B7C4E
P 750 3300
F 0 "J1" H 668 2975 50  0000 C CNN
F 1 "Power Connector" H 668 3066 50  0000 C CNN
F 2 "Pauls Parts:2WayScrewTerminal_2.54" H 750 3300 50  0001 C CNN
F 3 "https://docs.rs-online.com/90f4/0900766b8157c7e9.pdf" H 750 3300 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/pcb-terminal-blocks/7901098/" H 750 3300 50  0001 C CNN "Source"
	1    750  3300
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR03
U 1 1 609B994E
P 1450 3400
F 0 "#PWR03" H 1450 3150 50  0001 C CNN
F 1 "GND" H 1455 3227 50  0000 C CNN
F 2 "" H 1450 3400 50  0001 C CNN
F 3 "" H 1450 3400 50  0001 C CNN
	1    1450 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	950  3200 1200 3200
Wire Wire Line
	950  3300 1450 3300
Wire Wire Line
	1450 3300 1450 3400
$Comp
L Signalbox:470R R6
U 1 1 609DBD63
P 3250 5100
F 0 "R6" H 3320 5146 50  0000 L CNN
F 1 "470R" H 3320 5055 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 3180 5100 50  0001 C CNN
F 3 "https://docs.rs-online.com/b9c3/0900766b8157ade8.pdf" H 3250 5100 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/through-hole-fixed-resistors/7077647/" H 3250 5100 50  0001 C CNN "Source"
	1    3250 5100
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR015
U 1 1 609DD407
P 3250 4850
F 0 "#PWR015" H 3250 4700 50  0001 C CNN
F 1 "+5V" H 3265 5023 50  0000 C CNN
F 2 "" H 3250 4850 50  0001 C CNN
F 3 "" H 3250 4850 50  0001 C CNN
	1    3250 4850
	1    0    0    -1  
$EndComp
$Comp
L Signalbox:RS_Pro_50_Way_PCB_header J4
U 1 1 60938214
P 7450 3700
F 0 "J4" H 7500 5017 50  0000 C CNN
F 1 "Conn_02x25_Odd_Even" H 7500 4926 50  0000 C CNN
F 2 "Connector_IDC:IDC-Header_2x25_P2.54mm_Vertical" H 7450 3700 50  0001 C CNN
F 3 "https://docs.rs-online.com/fea6/0900766b815867b9.pdf" H 7450 3700 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/pcb-headers/6257353/" H 7450 3700 50  0001 C CNN "Source"
	1    7450 3700
	1    0    0    -1  
$EndComp
$Comp
L Signalbox:470R R37
U 1 1 609493FD
P 8700 1100
F 0 "R37" H 8770 1146 50  0000 L CNN
F 1 "470R" H 8770 1055 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 8630 1100 50  0001 C CNN
F 3 "https://docs.rs-online.com/b9c3/0900766b8157ade8.pdf" H 8700 1100 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/through-hole-fixed-resistors/7077647/" H 8700 1100 50  0001 C CNN "Source"
	1    8700 1100
	1    0    0    -1  
$EndComp
$Comp
L Signalbox:1K2 R38
U 1 1 6094A0AE
P 8750 5600
F 0 "R38" H 8820 5646 50  0000 L CNN
F 1 "1K2" H 8820 5555 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 8680 5600 50  0001 C CNN
F 3 "https://docs.rs-online.com/ce35/0900766b8157adf1.pdf" H 8750 5600 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/through-hole-fixed-resistors/7077678/" H 8750 5600 50  0001 C CNN "Source"
	1    8750 5600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR055
U 1 1 6094A5A4
P 8750 5800
F 0 "#PWR055" H 8750 5550 50  0001 C CNN
F 1 "GND" H 8755 5627 50  0000 C CNN
F 2 "" H 8750 5800 50  0001 C CNN
F 3 "" H 8750 5800 50  0001 C CNN
	1    8750 5800
	1    0    0    -1  
$EndComp
Text Notes 7350 5150 0    50   ~ 0
Diodes\n+     -
$Comp
L Signalbox:470R R41
U 1 1 609F5052
P 9050 1100
F 0 "R41" H 9120 1146 50  0000 L CNN
F 1 "470R" H 9120 1055 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 8980 1100 50  0001 C CNN
F 3 "https://docs.rs-online.com/b9c3/0900766b8157ade8.pdf" H 9050 1100 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/through-hole-fixed-resistors/7077647/" H 9050 1100 50  0001 C CNN "Source"
	1    9050 1100
	1    0    0    -1  
$EndComp
$Comp
L Signalbox:470R R45
U 1 1 609F5498
P 9400 1100
F 0 "R45" H 9470 1146 50  0000 L CNN
F 1 "470R" H 9470 1055 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 9330 1100 50  0001 C CNN
F 3 "https://docs.rs-online.com/b9c3/0900766b8157ade8.pdf" H 9400 1100 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/through-hole-fixed-resistors/7077647/" H 9400 1100 50  0001 C CNN "Source"
	1    9400 1100
	1    0    0    -1  
$EndComp
$Comp
L Signalbox:470R R49
U 1 1 609F596D
P 9750 1100
F 0 "R49" H 9820 1146 50  0000 L CNN
F 1 "470R" H 9820 1055 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 9680 1100 50  0001 C CNN
F 3 "https://docs.rs-online.com/b9c3/0900766b8157ade8.pdf" H 9750 1100 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/through-hole-fixed-resistors/7077647/" H 9750 1100 50  0001 C CNN "Source"
	1    9750 1100
	1    0    0    -1  
$EndComp
$Comp
L Signalbox:470R R39
U 1 1 609F5FC9
P 8900 1750
F 0 "R39" H 8970 1796 50  0000 L CNN
F 1 "470R" H 8970 1705 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 8830 1750 50  0001 C CNN
F 3 "https://docs.rs-online.com/b9c3/0900766b8157ade8.pdf" H 8900 1750 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/through-hole-fixed-resistors/7077647/" H 8900 1750 50  0001 C CNN "Source"
	1    8900 1750
	1    0    0    -1  
$EndComp
$Comp
L Signalbox:470R R43
U 1 1 609F6DD9
P 9250 1750
F 0 "R43" H 9320 1796 50  0000 L CNN
F 1 "470R" H 9320 1705 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 9180 1750 50  0001 C CNN
F 3 "https://docs.rs-online.com/b9c3/0900766b8157ade8.pdf" H 9250 1750 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/through-hole-fixed-resistors/7077647/" H 9250 1750 50  0001 C CNN "Source"
	1    9250 1750
	1    0    0    -1  
$EndComp
$Comp
L Signalbox:470R R47
U 1 1 609F7100
P 9600 1750
F 0 "R47" H 9670 1796 50  0000 L CNN
F 1 "470R" H 9670 1705 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 9530 1750 50  0001 C CNN
F 3 "https://docs.rs-online.com/b9c3/0900766b8157ade8.pdf" H 9600 1750 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/through-hole-fixed-resistors/7077647/" H 9600 1750 50  0001 C CNN "Source"
	1    9600 1750
	1    0    0    -1  
$EndComp
$Comp
L Signalbox:470R R52
U 1 1 609F7647
P 10000 1750
F 0 "R52" H 10070 1796 50  0000 L CNN
F 1 "470R" H 10070 1705 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 9930 1750 50  0001 C CNN
F 3 "https://docs.rs-online.com/b9c3/0900766b8157ade8.pdf" H 10000 1750 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/through-hole-fixed-resistors/7077647/" H 10000 1750 50  0001 C CNN "Source"
	1    10000 1750
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR054
U 1 1 609F7F90
P 8700 850
F 0 "#PWR054" H 8700 700 50  0001 C CNN
F 1 "+5V" H 8715 1023 50  0000 C CNN
F 2 "" H 8700 850 50  0001 C CNN
F 3 "" H 8700 850 50  0001 C CNN
	1    8700 850 
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR058
U 1 1 609F85D6
P 9050 850
F 0 "#PWR058" H 9050 700 50  0001 C CNN
F 1 "+5V" H 9065 1023 50  0000 C CNN
F 2 "" H 9050 850 50  0001 C CNN
F 3 "" H 9050 850 50  0001 C CNN
	1    9050 850 
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR062
U 1 1 609F8AB9
P 9400 850
F 0 "#PWR062" H 9400 700 50  0001 C CNN
F 1 "+5V" H 9415 1023 50  0000 C CNN
F 2 "" H 9400 850 50  0001 C CNN
F 3 "" H 9400 850 50  0001 C CNN
	1    9400 850 
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR066
U 1 1 609F8FF6
P 9750 850
F 0 "#PWR066" H 9750 700 50  0001 C CNN
F 1 "+5V" H 9765 1023 50  0000 C CNN
F 2 "" H 9750 850 50  0001 C CNN
F 3 "" H 9750 850 50  0001 C CNN
	1    9750 850 
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR056
U 1 1 609F95D4
P 8900 1500
F 0 "#PWR056" H 8900 1350 50  0001 C CNN
F 1 "+5V" H 8915 1673 50  0000 C CNN
F 2 "" H 8900 1500 50  0001 C CNN
F 3 "" H 8900 1500 50  0001 C CNN
	1    8900 1500
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR060
U 1 1 609FB255
P 9250 1500
F 0 "#PWR060" H 9250 1350 50  0001 C CNN
F 1 "+5V" H 9265 1673 50  0000 C CNN
F 2 "" H 9250 1500 50  0001 C CNN
F 3 "" H 9250 1500 50  0001 C CNN
	1    9250 1500
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR064
U 1 1 609FB6E5
P 9600 1500
F 0 "#PWR064" H 9600 1350 50  0001 C CNN
F 1 "+5V" H 9615 1673 50  0000 C CNN
F 2 "" H 9600 1500 50  0001 C CNN
F 3 "" H 9600 1500 50  0001 C CNN
	1    9600 1500
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR069
U 1 1 609FBC45
P 10000 1500
F 0 "#PWR069" H 10000 1350 50  0001 C CNN
F 1 "+5V" H 10015 1673 50  0000 C CNN
F 2 "" H 10000 1500 50  0001 C CNN
F 3 "" H 10000 1500 50  0001 C CNN
	1    10000 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	8700 950  8700 850 
Wire Wire Line
	9050 950  9050 850 
Wire Wire Line
	9400 950  9400 850 
Wire Wire Line
	9750 950  9750 850 
Wire Wire Line
	8900 1600 8900 1500
Wire Wire Line
	9250 1600 9250 1500
Wire Wire Line
	9600 1600 9600 1500
Wire Wire Line
	10000 1600 10000 1500
Text GLabel 7850 4800 2    50   Input ~ 0
O7
Text GLabel 7850 4500 2    50   Input ~ 0
O6
Text GLabel 7850 4200 2    50   Input ~ 0
O5
Text GLabel 7850 3900 2    50   Input ~ 0
O4
Text GLabel 7850 3600 2    50   Input ~ 0
O3
Text GLabel 7850 3300 2    50   Input ~ 0
O2
Text GLabel 7850 3000 2    50   Input ~ 0
O1
Text GLabel 7850 2700 2    50   Input ~ 0
O0
$Comp
L Signalbox:1K2 R42
U 1 1 60A8F370
P 9100 5600
F 0 "R42" H 9170 5646 50  0000 L CNN
F 1 "1K2" H 9170 5555 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 9030 5600 50  0001 C CNN
F 3 "https://docs.rs-online.com/ce35/0900766b8157adf1.pdf" H 9100 5600 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/through-hole-fixed-resistors/7077678/" H 9100 5600 50  0001 C CNN "Source"
	1    9100 5600
	1    0    0    -1  
$EndComp
$Comp
L Signalbox:1K2 R46
U 1 1 60A8F7B1
P 9450 5600
F 0 "R46" H 9520 5646 50  0000 L CNN
F 1 "1K2" H 9520 5555 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 9380 5600 50  0001 C CNN
F 3 "https://docs.rs-online.com/ce35/0900766b8157adf1.pdf" H 9450 5600 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/through-hole-fixed-resistors/7077678/" H 9450 5600 50  0001 C CNN "Source"
	1    9450 5600
	1    0    0    -1  
$EndComp
$Comp
L Signalbox:1K2 R50
U 1 1 60A8FB75
P 9800 5600
F 0 "R50" H 9870 5646 50  0000 L CNN
F 1 "1K2" H 9870 5555 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 9730 5600 50  0001 C CNN
F 3 "https://docs.rs-online.com/ce35/0900766b8157adf1.pdf" H 9800 5600 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/through-hole-fixed-resistors/7077678/" H 9800 5600 50  0001 C CNN "Source"
	1    9800 5600
	1    0    0    -1  
$EndComp
$Comp
L Signalbox:1K2 R51
U 1 1 60A8FFAC
P 9950 6100
F 0 "R51" H 10020 6146 50  0000 L CNN
F 1 "1K2" H 10020 6055 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 9880 6100 50  0001 C CNN
F 3 "https://docs.rs-online.com/ce35/0900766b8157adf1.pdf" H 9950 6100 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/through-hole-fixed-resistors/7077678/" H 9950 6100 50  0001 C CNN "Source"
	1    9950 6100
	1    0    0    -1  
$EndComp
$Comp
L Signalbox:1K2 R48
U 1 1 60A909B0
P 9600 6100
F 0 "R48" H 9670 6146 50  0000 L CNN
F 1 "1K2" H 9670 6055 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 9530 6100 50  0001 C CNN
F 3 "https://docs.rs-online.com/ce35/0900766b8157adf1.pdf" H 9600 6100 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/through-hole-fixed-resistors/7077678/" H 9600 6100 50  0001 C CNN "Source"
	1    9600 6100
	1    0    0    -1  
$EndComp
$Comp
L Signalbox:1K2 R44
U 1 1 60A90F34
P 9250 6100
F 0 "R44" H 9320 6146 50  0000 L CNN
F 1 "1K2" H 9320 6055 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 9180 6100 50  0001 C CNN
F 3 "https://docs.rs-online.com/ce35/0900766b8157adf1.pdf" H 9250 6100 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/through-hole-fixed-resistors/7077678/" H 9250 6100 50  0001 C CNN "Source"
	1    9250 6100
	1    0    0    -1  
$EndComp
$Comp
L Signalbox:1K2 R40
U 1 1 60A91590
P 8900 6100
F 0 "R40" H 8970 6146 50  0000 L CNN
F 1 "1K2" H 8970 6055 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 8830 6100 50  0001 C CNN
F 3 "https://docs.rs-online.com/ce35/0900766b8157adf1.pdf" H 8900 6100 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/through-hole-fixed-resistors/7077678/" H 8900 6100 50  0001 C CNN "Source"
	1    8900 6100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR059
U 1 1 60A97027
P 9100 5800
F 0 "#PWR059" H 9100 5550 50  0001 C CNN
F 1 "GND" H 9105 5627 50  0000 C CNN
F 2 "" H 9100 5800 50  0001 C CNN
F 3 "" H 9100 5800 50  0001 C CNN
	1    9100 5800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR063
U 1 1 60A9736A
P 9450 5800
F 0 "#PWR063" H 9450 5550 50  0001 C CNN
F 1 "GND" H 9455 5627 50  0000 C CNN
F 2 "" H 9450 5800 50  0001 C CNN
F 3 "" H 9450 5800 50  0001 C CNN
	1    9450 5800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR067
U 1 1 60A976FE
P 9800 5800
F 0 "#PWR067" H 9800 5550 50  0001 C CNN
F 1 "GND" H 9805 5627 50  0000 C CNN
F 2 "" H 9800 5800 50  0001 C CNN
F 3 "" H 9800 5800 50  0001 C CNN
	1    9800 5800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR068
U 1 1 60A9791F
P 9950 6300
F 0 "#PWR068" H 9950 6050 50  0001 C CNN
F 1 "GND" H 9955 6127 50  0000 C CNN
F 2 "" H 9950 6300 50  0001 C CNN
F 3 "" H 9950 6300 50  0001 C CNN
	1    9950 6300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR065
U 1 1 60A97D4C
P 9600 6300
F 0 "#PWR065" H 9600 6050 50  0001 C CNN
F 1 "GND" H 9605 6127 50  0000 C CNN
F 2 "" H 9600 6300 50  0001 C CNN
F 3 "" H 9600 6300 50  0001 C CNN
	1    9600 6300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR061
U 1 1 60A98107
P 9250 6300
F 0 "#PWR061" H 9250 6050 50  0001 C CNN
F 1 "GND" H 9255 6127 50  0000 C CNN
F 2 "" H 9250 6300 50  0001 C CNN
F 3 "" H 9250 6300 50  0001 C CNN
	1    9250 6300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR057
U 1 1 60A98574
P 8900 6300
F 0 "#PWR057" H 8900 6050 50  0001 C CNN
F 1 "GND" H 8905 6127 50  0000 C CNN
F 2 "" H 8900 6300 50  0001 C CNN
F 3 "" H 8900 6300 50  0001 C CNN
	1    8900 6300
	1    0    0    -1  
$EndComp
Wire Wire Line
	8750 5750 8750 5800
Wire Wire Line
	9100 5750 9100 5800
Wire Wire Line
	9450 5750 9450 5800
Wire Wire Line
	9800 5750 9800 5800
Wire Wire Line
	8900 6250 8900 6300
Wire Wire Line
	9250 6250 9250 6300
Wire Wire Line
	9600 6250 9600 6300
Wire Wire Line
	9950 6250 9950 6300
Text GLabel 7150 2600 0    50   Input ~ 0
SG0
Text GLabel 7150 2900 0    50   Input ~ 0
SG1
Text GLabel 7150 3200 0    50   Input ~ 0
SG2
Text GLabel 7150 3500 0    50   Input ~ 0
SG3
Text GLabel 7150 3800 0    50   Input ~ 0
SG4
Text GLabel 7150 4100 0    50   Input ~ 0
SG5
Text GLabel 7150 4400 0    50   Input ~ 0
SG6
Text GLabel 7150 4700 0    50   Input ~ 0
SG7
Wire Wire Line
	7150 2600 7250 2600
Wire Wire Line
	7150 2900 7250 2900
Wire Wire Line
	7150 3200 7250 3200
Wire Wire Line
	7150 3500 7250 3500
Wire Wire Line
	7150 3800 7250 3800
Wire Wire Line
	7150 4100 7250 4100
Wire Wire Line
	7150 4400 7250 4400
Wire Wire Line
	7150 4700 7250 4700
Text GLabel 7850 2500 2    50   Input ~ 0
SG0
Text GLabel 7850 2800 2    50   Input ~ 0
SG1
Text GLabel 7850 3100 2    50   Input ~ 0
SG2
Text GLabel 7850 3400 2    50   Input ~ 0
SG3
Text GLabel 7850 3700 2    50   Input ~ 0
SG4
Text GLabel 7850 4000 2    50   Input ~ 0
SG5
Text GLabel 7850 4300 2    50   Input ~ 0
SG6
Text GLabel 7850 4600 2    50   Input ~ 0
SG7
Wire Wire Line
	7750 2500 7850 2500
Wire Wire Line
	7750 2700 7850 2700
Wire Wire Line
	7750 2800 7850 2800
Wire Wire Line
	7750 3000 7850 3000
Wire Wire Line
	7750 3100 7850 3100
Wire Wire Line
	7750 3300 7850 3300
Wire Wire Line
	7750 3400 7850 3400
Wire Wire Line
	7750 3600 7850 3600
Wire Wire Line
	7750 3700 7850 3700
Wire Wire Line
	7750 3900 7850 3900
Wire Wire Line
	7750 4000 7850 4000
Wire Wire Line
	7750 4200 7850 4200
Wire Wire Line
	7750 4300 7850 4300
Wire Wire Line
	7750 4500 7850 4500
Wire Wire Line
	7750 4600 7850 4600
Wire Wire Line
	7750 4800 7850 4800
$Comp
L Signalbox:1K2 R2
U 1 1 60B9E314
P 3300 1050
F 0 "R2" H 3370 1096 50  0000 L CNN
F 1 "1K2" H 3370 1005 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 3230 1050 50  0001 C CNN
F 3 "https://docs.rs-online.com/ce35/0900766b8157adf1.pdf" H 3300 1050 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/through-hole-fixed-resistors/7077678/" H 3300 1050 50  0001 C CNN "Source"
	1    3300 1050
	1    0    0    -1  
$EndComp
$Comp
L Signalbox:1K2 R5
U 1 1 60B9E31A
P 3650 1050
F 0 "R5" H 3720 1096 50  0000 L CNN
F 1 "1K2" H 3720 1005 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 3580 1050 50  0001 C CNN
F 3 "https://docs.rs-online.com/ce35/0900766b8157adf1.pdf" H 3650 1050 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/through-hole-fixed-resistors/7077678/" H 3650 1050 50  0001 C CNN "Source"
	1    3650 1050
	1    0    0    -1  
$EndComp
$Comp
L Signalbox:1K2 R9
U 1 1 60B9E320
P 4000 1050
F 0 "R9" H 4070 1096 50  0000 L CNN
F 1 "1K2" H 4070 1005 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 3930 1050 50  0001 C CNN
F 3 "https://docs.rs-online.com/ce35/0900766b8157adf1.pdf" H 4000 1050 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/through-hole-fixed-resistors/7077678/" H 4000 1050 50  0001 C CNN "Source"
	1    4000 1050
	1    0    0    -1  
$EndComp
$Comp
L Signalbox:1K2 R11
U 1 1 60B9E326
P 4350 1050
F 0 "R11" H 4420 1096 50  0000 L CNN
F 1 "1K2" H 4420 1005 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 4280 1050 50  0001 C CNN
F 3 "https://docs.rs-online.com/ce35/0900766b8157adf1.pdf" H 4350 1050 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/through-hole-fixed-resistors/7077678/" H 4350 1050 50  0001 C CNN "Source"
	1    4350 1050
	1    0    0    -1  
$EndComp
$Comp
L Signalbox:1K2 R4
U 1 1 60B9E32C
P 3500 1700
F 0 "R4" H 3570 1746 50  0000 L CNN
F 1 "1K2" H 3570 1655 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 3430 1700 50  0001 C CNN
F 3 "https://docs.rs-online.com/ce35/0900766b8157adf1.pdf" H 3500 1700 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/through-hole-fixed-resistors/7077678/" H 3500 1700 50  0001 C CNN "Source"
	1    3500 1700
	1    0    0    -1  
$EndComp
$Comp
L Signalbox:1K2 R8
U 1 1 60B9E332
P 3850 1700
F 0 "R8" H 3920 1746 50  0000 L CNN
F 1 "1K2" H 3920 1655 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 3780 1700 50  0001 C CNN
F 3 "https://docs.rs-online.com/ce35/0900766b8157adf1.pdf" H 3850 1700 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/through-hole-fixed-resistors/7077678/" H 3850 1700 50  0001 C CNN "Source"
	1    3850 1700
	1    0    0    -1  
$EndComp
$Comp
L Signalbox:1K2 R10
U 1 1 60B9E338
P 4200 1700
F 0 "R10" H 4270 1746 50  0000 L CNN
F 1 "1K2" H 4270 1655 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 4130 1700 50  0001 C CNN
F 3 "https://docs.rs-online.com/ce35/0900766b8157adf1.pdf" H 4200 1700 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/through-hole-fixed-resistors/7077678/" H 4200 1700 50  0001 C CNN "Source"
	1    4200 1700
	1    0    0    -1  
$EndComp
$Comp
L Signalbox:1K2 R12
U 1 1 60B9E33E
P 4550 1700
F 0 "R12" H 4620 1746 50  0000 L CNN
F 1 "1K2" H 4620 1655 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 4480 1700 50  0001 C CNN
F 3 "https://docs.rs-online.com/ce35/0900766b8157adf1.pdf" H 4550 1700 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/through-hole-fixed-resistors/7077678/" H 4550 1700 50  0001 C CNN "Source"
	1    4550 1700
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR010
U 1 1 60B9E344
P 3300 800
F 0 "#PWR010" H 3300 650 50  0001 C CNN
F 1 "+5V" H 3315 973 50  0000 C CNN
F 2 "" H 3300 800 50  0001 C CNN
F 3 "" H 3300 800 50  0001 C CNN
	1    3300 800 
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR014
U 1 1 60B9E34A
P 3650 800
F 0 "#PWR014" H 3650 650 50  0001 C CNN
F 1 "+5V" H 3665 973 50  0000 C CNN
F 2 "" H 3650 800 50  0001 C CNN
F 3 "" H 3650 800 50  0001 C CNN
	1    3650 800 
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR019
U 1 1 60B9E350
P 4000 800
F 0 "#PWR019" H 4000 650 50  0001 C CNN
F 1 "+5V" H 4015 973 50  0000 C CNN
F 2 "" H 4000 800 50  0001 C CNN
F 3 "" H 4000 800 50  0001 C CNN
	1    4000 800 
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR021
U 1 1 60B9E356
P 4350 800
F 0 "#PWR021" H 4350 650 50  0001 C CNN
F 1 "+5V" H 4365 973 50  0000 C CNN
F 2 "" H 4350 800 50  0001 C CNN
F 3 "" H 4350 800 50  0001 C CNN
	1    4350 800 
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR013
U 1 1 60B9E35C
P 3500 1450
F 0 "#PWR013" H 3500 1300 50  0001 C CNN
F 1 "+5V" H 3515 1623 50  0000 C CNN
F 2 "" H 3500 1450 50  0001 C CNN
F 3 "" H 3500 1450 50  0001 C CNN
	1    3500 1450
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR018
U 1 1 60B9E362
P 3850 1450
F 0 "#PWR018" H 3850 1300 50  0001 C CNN
F 1 "+5V" H 3865 1623 50  0000 C CNN
F 2 "" H 3850 1450 50  0001 C CNN
F 3 "" H 3850 1450 50  0001 C CNN
	1    3850 1450
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR020
U 1 1 60B9E368
P 4200 1450
F 0 "#PWR020" H 4200 1300 50  0001 C CNN
F 1 "+5V" H 4215 1623 50  0000 C CNN
F 2 "" H 4200 1450 50  0001 C CNN
F 3 "" H 4200 1450 50  0001 C CNN
	1    4200 1450
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR024
U 1 1 60B9E36E
P 4550 1450
F 0 "#PWR024" H 4550 1300 50  0001 C CNN
F 1 "+5V" H 4565 1623 50  0000 C CNN
F 2 "" H 4550 1450 50  0001 C CNN
F 3 "" H 4550 1450 50  0001 C CNN
	1    4550 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3300 900  3300 800 
Wire Wire Line
	3650 900  3650 800 
Wire Wire Line
	4000 900  4000 800 
Wire Wire Line
	4350 900  4350 800 
Wire Wire Line
	3500 1550 3500 1450
Wire Wire Line
	3850 1550 3850 1450
Wire Wire Line
	4200 1550 4200 1450
Wire Wire Line
	4550 1550 4550 1450
Text GLabel 4550 2000 3    50   Output ~ 0
F8
Text GLabel 4350 1350 3    50   Output ~ 0
F9
Text GLabel 4200 2000 3    50   Output ~ 0
FA
Text GLabel 3850 2000 3    50   Output ~ 0
FC
Text GLabel 3500 2000 3    50   Output ~ 0
FE
Text GLabel 4000 1350 3    50   Output ~ 0
FB
Text GLabel 3650 1350 3    50   Output ~ 0
FD
Text GLabel 3300 1350 3    50   Output ~ 0
FF
Text GLabel 7150 2700 0    50   Input ~ 0
F0
Text GLabel 7150 3000 0    50   Input ~ 0
F1
Text GLabel 7150 3300 0    50   Input ~ 0
F2
Text GLabel 7150 3600 0    50   Input ~ 0
F3
Text GLabel 7150 3900 0    50   Input ~ 0
F4
Text GLabel 7150 4200 0    50   Input ~ 0
F5
Text GLabel 7150 4500 0    50   Input ~ 0
F6
Text GLabel 7150 4800 0    50   Input ~ 0
F7
Wire Wire Line
	3300 1350 3300 1200
Wire Wire Line
	3650 1350 3650 1200
Wire Wire Line
	4000 1350 4000 1200
Wire Wire Line
	4350 1350 4350 1200
Wire Wire Line
	3500 2000 3500 1850
Wire Wire Line
	3850 2000 3850 1850
Wire Wire Line
	4200 2000 4200 1850
Wire Wire Line
	4550 2000 4550 1850
Wire Wire Line
	7150 2700 7250 2700
Wire Wire Line
	7150 3000 7250 3000
Wire Wire Line
	7150 3300 7250 3300
Wire Wire Line
	7150 3600 7250 3600
Wire Wire Line
	7150 3900 7250 3900
Wire Wire Line
	7150 4200 7250 4200
Wire Wire Line
	7150 4500 7250 4500
Wire Wire Line
	7150 4800 7250 4800
$Comp
L Signalbox:1K2 R13
U 1 1 60A037FC
P 4750 1050
F 0 "R13" H 4820 1096 50  0000 L CNN
F 1 "1K2" H 4820 1005 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 4680 1050 50  0001 C CNN
F 3 "https://docs.rs-online.com/ce35/0900766b8157adf1.pdf" H 4750 1050 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/through-hole-fixed-resistors/7077678/" H 4750 1050 50  0001 C CNN "Source"
	1    4750 1050
	1    0    0    -1  
$EndComp
$Comp
L Signalbox:1K2 R15
U 1 1 60A03802
P 5100 1050
F 0 "R15" H 5170 1096 50  0000 L CNN
F 1 "1K2" H 5170 1005 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 5030 1050 50  0001 C CNN
F 3 "https://docs.rs-online.com/ce35/0900766b8157adf1.pdf" H 5100 1050 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/through-hole-fixed-resistors/7077678/" H 5100 1050 50  0001 C CNN "Source"
	1    5100 1050
	1    0    0    -1  
$EndComp
$Comp
L Signalbox:1K2 R17
U 1 1 60A03808
P 5450 1050
F 0 "R17" H 5520 1096 50  0000 L CNN
F 1 "1K2" H 5520 1005 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 5380 1050 50  0001 C CNN
F 3 "https://docs.rs-online.com/ce35/0900766b8157adf1.pdf" H 5450 1050 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/through-hole-fixed-resistors/7077678/" H 5450 1050 50  0001 C CNN "Source"
	1    5450 1050
	1    0    0    -1  
$EndComp
$Comp
L Signalbox:1K2 R19
U 1 1 60A0380E
P 5800 1050
F 0 "R19" H 5870 1096 50  0000 L CNN
F 1 "1K2" H 5870 1005 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 5730 1050 50  0001 C CNN
F 3 "https://docs.rs-online.com/ce35/0900766b8157adf1.pdf" H 5800 1050 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/through-hole-fixed-resistors/7077678/" H 5800 1050 50  0001 C CNN "Source"
	1    5800 1050
	1    0    0    -1  
$EndComp
$Comp
L Signalbox:1K2 R14
U 1 1 60A03814
P 4950 1700
F 0 "R14" H 5020 1746 50  0000 L CNN
F 1 "1K2" H 5020 1655 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 4880 1700 50  0001 C CNN
F 3 "https://docs.rs-online.com/ce35/0900766b8157adf1.pdf" H 4950 1700 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/through-hole-fixed-resistors/7077678/" H 4950 1700 50  0001 C CNN "Source"
	1    4950 1700
	1    0    0    -1  
$EndComp
$Comp
L Signalbox:1K2 R16
U 1 1 60A0381A
P 5300 1700
F 0 "R16" H 5370 1746 50  0000 L CNN
F 1 "1K2" H 5370 1655 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 5230 1700 50  0001 C CNN
F 3 "https://docs.rs-online.com/ce35/0900766b8157adf1.pdf" H 5300 1700 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/through-hole-fixed-resistors/7077678/" H 5300 1700 50  0001 C CNN "Source"
	1    5300 1700
	1    0    0    -1  
$EndComp
$Comp
L Signalbox:1K2 R18
U 1 1 60A03820
P 5650 1700
F 0 "R18" H 5720 1746 50  0000 L CNN
F 1 "1K2" H 5720 1655 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 5580 1700 50  0001 C CNN
F 3 "https://docs.rs-online.com/ce35/0900766b8157adf1.pdf" H 5650 1700 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/through-hole-fixed-resistors/7077678/" H 5650 1700 50  0001 C CNN "Source"
	1    5650 1700
	1    0    0    -1  
$EndComp
$Comp
L Signalbox:1K2 R20
U 1 1 60A03826
P 6000 1700
F 0 "R20" H 6070 1746 50  0000 L CNN
F 1 "1K2" H 6070 1655 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 5930 1700 50  0001 C CNN
F 3 "https://docs.rs-online.com/ce35/0900766b8157adf1.pdf" H 6000 1700 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/through-hole-fixed-resistors/7077678/" H 6000 1700 50  0001 C CNN "Source"
	1    6000 1700
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR026
U 1 1 60A0382C
P 4750 800
F 0 "#PWR026" H 4750 650 50  0001 C CNN
F 1 "+5V" H 4765 973 50  0000 C CNN
F 2 "" H 4750 800 50  0001 C CNN
F 3 "" H 4750 800 50  0001 C CNN
	1    4750 800 
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR028
U 1 1 60A03832
P 5100 800
F 0 "#PWR028" H 5100 650 50  0001 C CNN
F 1 "+5V" H 5115 973 50  0000 C CNN
F 2 "" H 5100 800 50  0001 C CNN
F 3 "" H 5100 800 50  0001 C CNN
	1    5100 800 
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR032
U 1 1 60A03838
P 5450 800
F 0 "#PWR032" H 5450 650 50  0001 C CNN
F 1 "+5V" H 5465 973 50  0000 C CNN
F 2 "" H 5450 800 50  0001 C CNN
F 3 "" H 5450 800 50  0001 C CNN
	1    5450 800 
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR036
U 1 1 60A0383E
P 5800 800
F 0 "#PWR036" H 5800 650 50  0001 C CNN
F 1 "+5V" H 5815 973 50  0000 C CNN
F 2 "" H 5800 800 50  0001 C CNN
F 3 "" H 5800 800 50  0001 C CNN
	1    5800 800 
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR027
U 1 1 60A03844
P 4950 1450
F 0 "#PWR027" H 4950 1300 50  0001 C CNN
F 1 "+5V" H 4965 1623 50  0000 C CNN
F 2 "" H 4950 1450 50  0001 C CNN
F 3 "" H 4950 1450 50  0001 C CNN
	1    4950 1450
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR031
U 1 1 60A0384A
P 5300 1450
F 0 "#PWR031" H 5300 1300 50  0001 C CNN
F 1 "+5V" H 5315 1623 50  0000 C CNN
F 2 "" H 5300 1450 50  0001 C CNN
F 3 "" H 5300 1450 50  0001 C CNN
	1    5300 1450
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR035
U 1 1 60A03850
P 5650 1450
F 0 "#PWR035" H 5650 1300 50  0001 C CNN
F 1 "+5V" H 5665 1623 50  0000 C CNN
F 2 "" H 5650 1450 50  0001 C CNN
F 3 "" H 5650 1450 50  0001 C CNN
	1    5650 1450
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR037
U 1 1 60A03856
P 6000 1450
F 0 "#PWR037" H 6000 1300 50  0001 C CNN
F 1 "+5V" H 6015 1623 50  0000 C CNN
F 2 "" H 6000 1450 50  0001 C CNN
F 3 "" H 6000 1450 50  0001 C CNN
	1    6000 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	4750 900  4750 800 
Wire Wire Line
	5100 900  5100 800 
Wire Wire Line
	5450 900  5450 800 
Wire Wire Line
	5800 900  5800 800 
Wire Wire Line
	4950 1550 4950 1450
Wire Wire Line
	5300 1550 5300 1450
Wire Wire Line
	5650 1550 5650 1450
Wire Wire Line
	6000 1550 6000 1450
Text GLabel 6000 2000 3    50   Output ~ 0
F0
Text GLabel 5800 1350 3    50   Output ~ 0
F1
Text GLabel 5650 2000 3    50   Output ~ 0
F2
Text GLabel 5300 2000 3    50   Output ~ 0
F4
Text GLabel 4950 2000 3    50   Output ~ 0
F6
Text GLabel 5450 1350 3    50   Output ~ 0
F3
Text GLabel 5100 1350 3    50   Output ~ 0
F5
Text GLabel 4750 1350 3    50   Output ~ 0
F7
Wire Wire Line
	4750 1350 4750 1200
Wire Wire Line
	5100 1350 5100 1200
Wire Wire Line
	5450 1350 5450 1200
Wire Wire Line
	5800 1350 5800 1200
Wire Wire Line
	4950 2000 4950 1850
Wire Wire Line
	5300 2000 5300 1850
Wire Wire Line
	5650 2000 5650 1850
Wire Wire Line
	6000 2000 6000 1850
Text GLabel 10000 2050 3    50   Output ~ 0
G0
Text GLabel 9750 1400 3    50   Output ~ 0
G1
Text GLabel 9600 2050 3    50   Output ~ 0
G2
Text GLabel 9400 1400 3    50   Output ~ 0
G3
Text GLabel 9250 2050 3    50   Output ~ 0
G4
Text GLabel 9050 1400 3    50   Output ~ 0
G5
Text GLabel 8900 2050 3    50   Output ~ 0
G6
Text GLabel 8700 1400 3    50   Output ~ 0
G7
$Comp
L Signalbox:470R R21
U 1 1 60AA7715
P 7200 1100
F 0 "R21" H 7270 1146 50  0000 L CNN
F 1 "470R" H 7270 1055 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 7130 1100 50  0001 C CNN
F 3 "https://docs.rs-online.com/b9c3/0900766b8157ade8.pdf" H 7200 1100 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/through-hole-fixed-resistors/7077647/" H 7200 1100 50  0001 C CNN "Source"
	1    7200 1100
	1    0    0    -1  
$EndComp
$Comp
L Signalbox:470R R25
U 1 1 60AA771B
P 7550 1100
F 0 "R25" H 7620 1146 50  0000 L CNN
F 1 "470R" H 7620 1055 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 7480 1100 50  0001 C CNN
F 3 "https://docs.rs-online.com/b9c3/0900766b8157ade8.pdf" H 7550 1100 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/through-hole-fixed-resistors/7077647/" H 7550 1100 50  0001 C CNN "Source"
	1    7550 1100
	1    0    0    -1  
$EndComp
$Comp
L Signalbox:470R R29
U 1 1 60AA7721
P 7900 1100
F 0 "R29" H 7970 1146 50  0000 L CNN
F 1 "470R" H 7970 1055 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 7830 1100 50  0001 C CNN
F 3 "https://docs.rs-online.com/b9c3/0900766b8157ade8.pdf" H 7900 1100 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/through-hole-fixed-resistors/7077647/" H 7900 1100 50  0001 C CNN "Source"
	1    7900 1100
	1    0    0    -1  
$EndComp
$Comp
L Signalbox:470R R33
U 1 1 60AA7727
P 8250 1100
F 0 "R33" H 8320 1146 50  0000 L CNN
F 1 "470R" H 8320 1055 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 8180 1100 50  0001 C CNN
F 3 "https://docs.rs-online.com/b9c3/0900766b8157ade8.pdf" H 8250 1100 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/through-hole-fixed-resistors/7077647/" H 8250 1100 50  0001 C CNN "Source"
	1    8250 1100
	1    0    0    -1  
$EndComp
$Comp
L Signalbox:470R R23
U 1 1 60AA772D
P 7400 1750
F 0 "R23" H 7470 1796 50  0000 L CNN
F 1 "470R" H 7470 1705 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 7330 1750 50  0001 C CNN
F 3 "https://docs.rs-online.com/b9c3/0900766b8157ade8.pdf" H 7400 1750 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/through-hole-fixed-resistors/7077647/" H 7400 1750 50  0001 C CNN "Source"
	1    7400 1750
	1    0    0    -1  
$EndComp
$Comp
L Signalbox:470R R27
U 1 1 60AA7733
P 7750 1750
F 0 "R27" H 7820 1796 50  0000 L CNN
F 1 "470R" H 7820 1705 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 7680 1750 50  0001 C CNN
F 3 "https://docs.rs-online.com/b9c3/0900766b8157ade8.pdf" H 7750 1750 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/through-hole-fixed-resistors/7077647/" H 7750 1750 50  0001 C CNN "Source"
	1    7750 1750
	1    0    0    -1  
$EndComp
$Comp
L Signalbox:470R R31
U 1 1 60AA7739
P 8100 1750
F 0 "R31" H 8170 1796 50  0000 L CNN
F 1 "470R" H 8170 1705 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 8030 1750 50  0001 C CNN
F 3 "https://docs.rs-online.com/b9c3/0900766b8157ade8.pdf" H 8100 1750 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/through-hole-fixed-resistors/7077647/" H 8100 1750 50  0001 C CNN "Source"
	1    8100 1750
	1    0    0    -1  
$EndComp
$Comp
L Signalbox:470R R35
U 1 1 60AA773F
P 8500 1750
F 0 "R35" H 8570 1796 50  0000 L CNN
F 1 "470R" H 8570 1705 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 8430 1750 50  0001 C CNN
F 3 "https://docs.rs-online.com/b9c3/0900766b8157ade8.pdf" H 8500 1750 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/through-hole-fixed-resistors/7077647/" H 8500 1750 50  0001 C CNN "Source"
	1    8500 1750
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR038
U 1 1 60AA7745
P 7200 850
F 0 "#PWR038" H 7200 700 50  0001 C CNN
F 1 "+5V" H 7215 1023 50  0000 C CNN
F 2 "" H 7200 850 50  0001 C CNN
F 3 "" H 7200 850 50  0001 C CNN
	1    7200 850 
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR042
U 1 1 60AA774B
P 7550 850
F 0 "#PWR042" H 7550 700 50  0001 C CNN
F 1 "+5V" H 7565 1023 50  0000 C CNN
F 2 "" H 7550 850 50  0001 C CNN
F 3 "" H 7550 850 50  0001 C CNN
	1    7550 850 
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR046
U 1 1 60AA7751
P 7900 850
F 0 "#PWR046" H 7900 700 50  0001 C CNN
F 1 "+5V" H 7915 1023 50  0000 C CNN
F 2 "" H 7900 850 50  0001 C CNN
F 3 "" H 7900 850 50  0001 C CNN
	1    7900 850 
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR050
U 1 1 60AA7757
P 8250 850
F 0 "#PWR050" H 8250 700 50  0001 C CNN
F 1 "+5V" H 8265 1023 50  0000 C CNN
F 2 "" H 8250 850 50  0001 C CNN
F 3 "" H 8250 850 50  0001 C CNN
	1    8250 850 
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR040
U 1 1 60AA775D
P 7400 1500
F 0 "#PWR040" H 7400 1350 50  0001 C CNN
F 1 "+5V" H 7415 1673 50  0000 C CNN
F 2 "" H 7400 1500 50  0001 C CNN
F 3 "" H 7400 1500 50  0001 C CNN
	1    7400 1500
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR044
U 1 1 60AA7763
P 7750 1500
F 0 "#PWR044" H 7750 1350 50  0001 C CNN
F 1 "+5V" H 7765 1673 50  0000 C CNN
F 2 "" H 7750 1500 50  0001 C CNN
F 3 "" H 7750 1500 50  0001 C CNN
	1    7750 1500
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR048
U 1 1 60AA7769
P 8100 1500
F 0 "#PWR048" H 8100 1350 50  0001 C CNN
F 1 "+5V" H 8115 1673 50  0000 C CNN
F 2 "" H 8100 1500 50  0001 C CNN
F 3 "" H 8100 1500 50  0001 C CNN
	1    8100 1500
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR052
U 1 1 60AA776F
P 8500 1500
F 0 "#PWR052" H 8500 1350 50  0001 C CNN
F 1 "+5V" H 8515 1673 50  0000 C CNN
F 2 "" H 8500 1500 50  0001 C CNN
F 3 "" H 8500 1500 50  0001 C CNN
	1    8500 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	7200 950  7200 850 
Wire Wire Line
	7550 950  7550 850 
Wire Wire Line
	7900 950  7900 850 
Wire Wire Line
	8250 950  8250 850 
Wire Wire Line
	7400 1600 7400 1500
Wire Wire Line
	7750 1600 7750 1500
Wire Wire Line
	8100 1600 8100 1500
Wire Wire Line
	8500 1600 8500 1500
Text GLabel 8500 2050 3    50   Output ~ 0
G8
Text GLabel 8250 1400 3    50   Output ~ 0
G9
Text GLabel 8100 2050 3    50   Output ~ 0
GA
Text GLabel 7900 1400 3    50   Output ~ 0
GB
Text GLabel 7750 2050 3    50   Output ~ 0
GC
Text GLabel 7550 1400 3    50   Output ~ 0
GD
Text GLabel 7400 2050 3    50   Output ~ 0
GE
Text GLabel 7200 1400 3    50   Output ~ 0
GF
Text GLabel 7150 2500 0    50   Input ~ 0
G0
Text GLabel 7150 2800 0    50   Input ~ 0
G1
Text GLabel 7150 3100 0    50   Input ~ 0
G2
Text GLabel 7150 3400 0    50   Input ~ 0
G3
Text GLabel 7150 3700 0    50   Input ~ 0
G4
Text GLabel 7150 4000 0    50   Input ~ 0
G5
Text GLabel 7150 4300 0    50   Input ~ 0
G6
Text GLabel 7150 4600 0    50   Input ~ 0
G7
Wire Wire Line
	7150 2500 7250 2500
Wire Wire Line
	7150 2800 7250 2800
Wire Wire Line
	7150 3100 7250 3100
Wire Wire Line
	7150 3400 7250 3400
Wire Wire Line
	7150 3700 7250 3700
Wire Wire Line
	7150 4000 7250 4000
Wire Wire Line
	7150 4300 7250 4300
Wire Wire Line
	7150 4600 7250 4600
Wire Wire Line
	7200 1250 7200 1400
Wire Wire Line
	7550 1250 7550 1400
Wire Wire Line
	7900 1250 7900 1400
Wire Wire Line
	8250 1250 8250 1400
Wire Wire Line
	8700 1250 8700 1400
Wire Wire Line
	9050 1250 9050 1400
Wire Wire Line
	9400 1250 9400 1400
Wire Wire Line
	9750 1250 9750 1400
Wire Wire Line
	7400 1900 7400 2050
Wire Wire Line
	7750 1900 7750 2050
Wire Wire Line
	8100 1900 8100 2050
Wire Wire Line
	8500 1900 8500 2050
Wire Wire Line
	8900 1900 8900 2050
Wire Wire Line
	9250 1900 9250 2050
Wire Wire Line
	9600 1900 9600 2050
Wire Wire Line
	10000 1900 10000 2050
Text GLabel 7850 2600 2    50   Output ~ 0
R0
Text GLabel 7850 2900 2    50   Output ~ 0
R1
Text GLabel 7850 3200 2    50   Output ~ 0
R2
Text GLabel 7850 3500 2    50   Output ~ 0
R3
Text GLabel 7850 3800 2    50   Output ~ 0
R4
Text GLabel 7850 4100 2    50   Output ~ 0
R5
Text GLabel 7850 4400 2    50   Output ~ 0
R6
Text GLabel 7850 4700 2    50   Output ~ 0
R7
Wire Wire Line
	7750 2600 7850 2600
Wire Wire Line
	7750 2900 7850 2900
Wire Wire Line
	7750 3200 7850 3200
Wire Wire Line
	7750 3500 7850 3500
Wire Wire Line
	7750 3800 7850 3800
Wire Wire Line
	7750 4100 7850 4100
Wire Wire Line
	7750 4400 7850 4400
Wire Wire Line
	7750 4700 7850 4700
$Comp
L Signalbox:1K2 R22
U 1 1 60D3724B
P 7350 5600
F 0 "R22" H 7420 5646 50  0000 L CNN
F 1 "1K2" H 7420 5555 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 7280 5600 50  0001 C CNN
F 3 "https://docs.rs-online.com/ce35/0900766b8157adf1.pdf" H 7350 5600 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/through-hole-fixed-resistors/7077678/" H 7350 5600 50  0001 C CNN "Source"
	1    7350 5600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR039
U 1 1 60D37251
P 7350 5800
F 0 "#PWR039" H 7350 5550 50  0001 C CNN
F 1 "GND" H 7355 5627 50  0000 C CNN
F 2 "" H 7350 5800 50  0001 C CNN
F 3 "" H 7350 5800 50  0001 C CNN
	1    7350 5800
	1    0    0    -1  
$EndComp
$Comp
L Signalbox:1K2 R26
U 1 1 60D37257
P 7700 5600
F 0 "R26" H 7770 5646 50  0000 L CNN
F 1 "1K2" H 7770 5555 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 7630 5600 50  0001 C CNN
F 3 "https://docs.rs-online.com/ce35/0900766b8157adf1.pdf" H 7700 5600 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/through-hole-fixed-resistors/7077678/" H 7700 5600 50  0001 C CNN "Source"
	1    7700 5600
	1    0    0    -1  
$EndComp
$Comp
L Signalbox:1K2 R30
U 1 1 60D3725D
P 8050 5600
F 0 "R30" H 8120 5646 50  0000 L CNN
F 1 "1K2" H 8120 5555 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 7980 5600 50  0001 C CNN
F 3 "https://docs.rs-online.com/ce35/0900766b8157adf1.pdf" H 8050 5600 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/through-hole-fixed-resistors/7077678/" H 8050 5600 50  0001 C CNN "Source"
	1    8050 5600
	1    0    0    -1  
$EndComp
$Comp
L Signalbox:1K2 R34
U 1 1 60D37263
P 8400 5600
F 0 "R34" H 8470 5646 50  0000 L CNN
F 1 "1K2" H 8470 5555 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 8330 5600 50  0001 C CNN
F 3 "https://docs.rs-online.com/ce35/0900766b8157adf1.pdf" H 8400 5600 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/through-hole-fixed-resistors/7077678/" H 8400 5600 50  0001 C CNN "Source"
	1    8400 5600
	1    0    0    -1  
$EndComp
$Comp
L Signalbox:1K2 R36
U 1 1 60D37269
P 8550 6100
F 0 "R36" H 8620 6146 50  0000 L CNN
F 1 "1K2" H 8620 6055 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 8480 6100 50  0001 C CNN
F 3 "https://docs.rs-online.com/ce35/0900766b8157adf1.pdf" H 8550 6100 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/through-hole-fixed-resistors/7077678/" H 8550 6100 50  0001 C CNN "Source"
	1    8550 6100
	1    0    0    -1  
$EndComp
$Comp
L Signalbox:1K2 R32
U 1 1 60D3726F
P 8200 6100
F 0 "R32" H 8270 6146 50  0000 L CNN
F 1 "1K2" H 8270 6055 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 8130 6100 50  0001 C CNN
F 3 "https://docs.rs-online.com/ce35/0900766b8157adf1.pdf" H 8200 6100 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/through-hole-fixed-resistors/7077678/" H 8200 6100 50  0001 C CNN "Source"
	1    8200 6100
	1    0    0    -1  
$EndComp
$Comp
L Signalbox:1K2 R28
U 1 1 60D37275
P 7850 6100
F 0 "R28" H 7920 6146 50  0000 L CNN
F 1 "1K2" H 7920 6055 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 7780 6100 50  0001 C CNN
F 3 "https://docs.rs-online.com/ce35/0900766b8157adf1.pdf" H 7850 6100 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/through-hole-fixed-resistors/7077678/" H 7850 6100 50  0001 C CNN "Source"
	1    7850 6100
	1    0    0    -1  
$EndComp
$Comp
L Signalbox:1K2 R24
U 1 1 60D3727B
P 7500 6100
F 0 "R24" H 7570 6146 50  0000 L CNN
F 1 "1K2" H 7570 6055 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 7430 6100 50  0001 C CNN
F 3 "https://docs.rs-online.com/ce35/0900766b8157adf1.pdf" H 7500 6100 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/through-hole-fixed-resistors/7077678/" H 7500 6100 50  0001 C CNN "Source"
	1    7500 6100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR043
U 1 1 60D37281
P 7700 5800
F 0 "#PWR043" H 7700 5550 50  0001 C CNN
F 1 "GND" H 7705 5627 50  0000 C CNN
F 2 "" H 7700 5800 50  0001 C CNN
F 3 "" H 7700 5800 50  0001 C CNN
	1    7700 5800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR047
U 1 1 60D37287
P 8050 5800
F 0 "#PWR047" H 8050 5550 50  0001 C CNN
F 1 "GND" H 8055 5627 50  0000 C CNN
F 2 "" H 8050 5800 50  0001 C CNN
F 3 "" H 8050 5800 50  0001 C CNN
	1    8050 5800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR051
U 1 1 60D3728D
P 8400 5800
F 0 "#PWR051" H 8400 5550 50  0001 C CNN
F 1 "GND" H 8405 5627 50  0000 C CNN
F 2 "" H 8400 5800 50  0001 C CNN
F 3 "" H 8400 5800 50  0001 C CNN
	1    8400 5800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR053
U 1 1 60D37293
P 8550 6300
F 0 "#PWR053" H 8550 6050 50  0001 C CNN
F 1 "GND" H 8555 6127 50  0000 C CNN
F 2 "" H 8550 6300 50  0001 C CNN
F 3 "" H 8550 6300 50  0001 C CNN
	1    8550 6300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR049
U 1 1 60D37299
P 8200 6300
F 0 "#PWR049" H 8200 6050 50  0001 C CNN
F 1 "GND" H 8205 6127 50  0000 C CNN
F 2 "" H 8200 6300 50  0001 C CNN
F 3 "" H 8200 6300 50  0001 C CNN
	1    8200 6300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR045
U 1 1 60D3729F
P 7850 6300
F 0 "#PWR045" H 7850 6050 50  0001 C CNN
F 1 "GND" H 7855 6127 50  0000 C CNN
F 2 "" H 7850 6300 50  0001 C CNN
F 3 "" H 7850 6300 50  0001 C CNN
	1    7850 6300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR041
U 1 1 60D372A5
P 7500 6300
F 0 "#PWR041" H 7500 6050 50  0001 C CNN
F 1 "GND" H 7505 6127 50  0000 C CNN
F 2 "" H 7500 6300 50  0001 C CNN
F 3 "" H 7500 6300 50  0001 C CNN
	1    7500 6300
	1    0    0    -1  
$EndComp
Wire Wire Line
	7350 5750 7350 5800
Wire Wire Line
	7700 5750 7700 5800
Wire Wire Line
	8050 5750 8050 5800
Wire Wire Line
	8400 5750 8400 5800
Wire Wire Line
	7500 6250 7500 6300
Wire Wire Line
	7850 6250 7850 6300
Wire Wire Line
	8200 6250 8200 6300
Wire Wire Line
	8550 6250 8550 6300
Text GLabel 9950 5900 1    50   Input ~ 0
R0
Text GLabel 9600 5900 1    50   Input ~ 0
R2
Text GLabel 9250 5900 1    50   Input ~ 0
R4
Text GLabel 8900 5900 1    50   Input ~ 0
R6
Text GLabel 8550 5900 1    50   Input ~ 0
R8
Text GLabel 8200 5900 1    50   Input ~ 0
RA
Text GLabel 7850 5900 1    50   Input ~ 0
RC
Text GLabel 7500 5900 1    50   Input ~ 0
RE
Text GLabel 7350 5400 1    50   Input ~ 0
RF
Text GLabel 7700 5400 1    50   Input ~ 0
RD
Text GLabel 8050 5400 1    50   Input ~ 0
RB
Text GLabel 8750 5400 1    50   Input ~ 0
R7
Text GLabel 9100 5400 1    50   Input ~ 0
R5
Text GLabel 9450 5400 1    50   Input ~ 0
R3
Text GLabel 9800 5400 1    50   Input ~ 0
R1
Text GLabel 8400 5400 1    50   Input ~ 0
R9
$Comp
L Signalbox:RS_Pro_50_Way_PCB_header J5
U 1 1 60E4A07E
P 9700 3700
F 0 "J5" H 9750 5017 50  0000 C CNN
F 1 "Conn_02x25_Odd_Even" H 9750 4926 50  0000 C CNN
F 2 "Connector_IDC:IDC-Header_2x25_P2.54mm_Vertical" H 9700 3700 50  0001 C CNN
F 3 "https://docs.rs-online.com/fea6/0900766b815867b9.pdf" H 9700 3700 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/pcb-headers/6257353/" H 9700 3700 50  0001 C CNN "Source"
	1    9700 3700
	1    0    0    -1  
$EndComp
Text Notes 9600 5150 0    50   ~ 0
Diodes\n+     -
Text GLabel 10100 4800 2    50   Input ~ 0
OF
Text GLabel 10100 4500 2    50   Input ~ 0
OE
Text GLabel 10100 4200 2    50   Input ~ 0
OD
Text GLabel 10100 3900 2    50   Input ~ 0
OC
Text GLabel 10100 3600 2    50   Input ~ 0
OB
Text GLabel 10100 3300 2    50   Input ~ 0
OA
Text GLabel 10100 3000 2    50   Input ~ 0
O9
Text GLabel 10100 2700 2    50   Input ~ 0
O8
Text GLabel 9400 2600 0    50   Input ~ 0
SG8
Text GLabel 9400 2900 0    50   Input ~ 0
SG9
Text GLabel 9400 3200 0    50   Input ~ 0
SGA
Text GLabel 9400 3500 0    50   Input ~ 0
SGB
Text GLabel 9400 3800 0    50   Input ~ 0
SGC
Text GLabel 9400 4100 0    50   Input ~ 0
SGD
Text GLabel 9400 4400 0    50   Input ~ 0
SGE
Text GLabel 9400 4700 0    50   Input ~ 0
SGF
Wire Wire Line
	9400 2600 9500 2600
Wire Wire Line
	9400 2900 9500 2900
Wire Wire Line
	9400 3200 9500 3200
Wire Wire Line
	9400 3500 9500 3500
Wire Wire Line
	9400 3800 9500 3800
Wire Wire Line
	9400 4100 9500 4100
Wire Wire Line
	9400 4400 9500 4400
Wire Wire Line
	9400 4700 9500 4700
Text GLabel 10100 2500 2    50   Input ~ 0
SG8
Text GLabel 10100 2800 2    50   Input ~ 0
SG9
Text GLabel 10100 3100 2    50   Input ~ 0
SGA
Text GLabel 10100 3400 2    50   Input ~ 0
SGB
Text GLabel 10100 3700 2    50   Input ~ 0
SGC
Text GLabel 10100 4000 2    50   Input ~ 0
SGD
Text GLabel 10100 4300 2    50   Input ~ 0
SGE
Text GLabel 10100 4600 2    50   Input ~ 0
SGF
Wire Wire Line
	10000 2500 10100 2500
Wire Wire Line
	10000 2700 10100 2700
Wire Wire Line
	10000 2800 10100 2800
Wire Wire Line
	10000 3000 10100 3000
Wire Wire Line
	10000 3100 10100 3100
Wire Wire Line
	10000 3300 10100 3300
Wire Wire Line
	10000 3400 10100 3400
Wire Wire Line
	10000 3600 10100 3600
Wire Wire Line
	10000 3700 10100 3700
Wire Wire Line
	10000 3900 10100 3900
Wire Wire Line
	10000 4000 10100 4000
Wire Wire Line
	10000 4200 10100 4200
Wire Wire Line
	10000 4300 10100 4300
Wire Wire Line
	10000 4500 10100 4500
Wire Wire Line
	10000 4600 10100 4600
Wire Wire Line
	10000 4800 10100 4800
Text GLabel 9400 2700 0    50   Input ~ 0
F8
Text GLabel 9400 3000 0    50   Input ~ 0
F9
Text GLabel 9400 3300 0    50   Input ~ 0
FA
Text GLabel 9400 3600 0    50   Input ~ 0
FB
Text GLabel 9400 3900 0    50   Input ~ 0
FC
Text GLabel 9400 4200 0    50   Input ~ 0
FD
Text GLabel 9400 4500 0    50   Input ~ 0
FE
Text GLabel 9400 4800 0    50   Input ~ 0
FF
Wire Wire Line
	9400 2700 9500 2700
Wire Wire Line
	9400 3000 9500 3000
Wire Wire Line
	9400 3300 9500 3300
Wire Wire Line
	9400 3600 9500 3600
Wire Wire Line
	9400 3900 9500 3900
Wire Wire Line
	9400 4200 9500 4200
Wire Wire Line
	9400 4500 9500 4500
Wire Wire Line
	9400 4800 9500 4800
Text GLabel 9400 2500 0    50   Input ~ 0
G8
Text GLabel 9400 2800 0    50   Input ~ 0
G9
Text GLabel 9400 3100 0    50   Input ~ 0
GA
Text GLabel 9400 3400 0    50   Input ~ 0
GB
Text GLabel 9400 3700 0    50   Input ~ 0
GC
Text GLabel 9400 4000 0    50   Input ~ 0
GD
Text GLabel 9400 4300 0    50   Input ~ 0
GE
Text GLabel 9400 4600 0    50   Input ~ 0
GF
Wire Wire Line
	9400 2500 9500 2500
Wire Wire Line
	9400 2800 9500 2800
Wire Wire Line
	9400 3100 9500 3100
Wire Wire Line
	9400 3400 9500 3400
Wire Wire Line
	9400 3700 9500 3700
Wire Wire Line
	9400 4000 9500 4000
Wire Wire Line
	9400 4300 9500 4300
Wire Wire Line
	9400 4600 9500 4600
Text GLabel 10100 2600 2    50   Output ~ 0
R8
Text GLabel 10100 2900 2    50   Output ~ 0
R9
Text GLabel 10100 3200 2    50   Output ~ 0
RA
Text GLabel 10100 3500 2    50   Output ~ 0
RB
Text GLabel 10100 3800 2    50   Output ~ 0
RC
Text GLabel 10100 4100 2    50   Output ~ 0
RD
Text GLabel 10100 4400 2    50   Output ~ 0
RE
Text GLabel 10100 4700 2    50   Output ~ 0
RF
Wire Wire Line
	10000 2600 10100 2600
Wire Wire Line
	10000 2900 10100 2900
Wire Wire Line
	10000 3200 10100 3200
Wire Wire Line
	10000 3500 10100 3500
Wire Wire Line
	10000 3800 10100 3800
Wire Wire Line
	10000 4100 10100 4100
Wire Wire Line
	10000 4400 10100 4400
Wire Wire Line
	10000 4700 10100 4700
Wire Wire Line
	5900 4350 6100 4350
Wire Wire Line
	5900 4250 6100 4250
Wire Wire Line
	5900 4150 6100 4150
Wire Wire Line
	5900 4050 6100 4050
Wire Wire Line
	5900 3950 6100 3950
Wire Wire Line
	5900 3850 6100 3850
Wire Wire Line
	5900 3750 6100 3750
Wire Wire Line
	5900 3650 6100 3650
Wire Wire Line
	5900 3450 6100 3450
Wire Wire Line
	5900 3350 6100 3350
Wire Wire Line
	5900 3250 6100 3250
Wire Wire Line
	5900 3150 6100 3150
Wire Wire Line
	5900 3050 6100 3050
Wire Wire Line
	5900 2950 6100 2950
Wire Wire Line
	5900 2850 6100 2850
Wire Wire Line
	5900 2750 6100 2750
Text GLabel 6100 4350 2    50   Output ~ 0
SG7
Text GLabel 6100 4250 2    50   Output ~ 0
SG6
Text GLabel 6100 4150 2    50   Output ~ 0
SG5
Text GLabel 6100 4050 2    50   Output ~ 0
SG4
Text GLabel 6100 3950 2    50   Output ~ 0
SG3
Text GLabel 6100 3850 2    50   Output ~ 0
SG2
Text GLabel 6100 3750 2    50   Output ~ 0
SG1
Text GLabel 6100 3650 2    50   Output ~ 0
SG0
Text GLabel 6100 3450 2    50   Output ~ 0
O7
Text GLabel 6100 3350 2    50   Output ~ 0
O6
Text GLabel 6100 3250 2    50   Output ~ 0
O5
Text GLabel 6100 3150 2    50   Output ~ 0
O4
Text GLabel 6100 3050 2    50   Output ~ 0
O3
Text GLabel 6100 2950 2    50   Output ~ 0
O2
Text GLabel 6100 2850 2    50   Output ~ 0
O1
Text GLabel 6100 2750 2    50   Output ~ 0
O0
NoConn ~ 4500 3450
NoConn ~ 4500 3350
Wire Wire Line
	4500 4350 4400 4350
Wire Wire Line
	4500 4250 4400 4250
Wire Wire Line
	4400 4150 4500 4150
$Comp
L power:GND #PWR030
U 1 1 60946005
P 5200 4750
F 0 "#PWR030" H 5200 4500 50  0001 C CNN
F 1 "GND" H 5205 4577 50  0000 C CNN
F 2 "" H 5200 4750 50  0001 C CNN
F 3 "" H 5200 4750 50  0001 C CNN
	1    5200 4750
	1    0    0    -1  
$EndComp
$Comp
L Signalbox:MCP23017_SP U2
U 1 1 609050FE
P 5200 3550
F 0 "U2" H 4800 4600 50  0000 C CNN
F 1 "MCP23017_SP" H 4800 4500 50  0000 C CNN
F 2 "Package_DIP:DIP-28_W7.62mm" H 5400 2550 50  0001 L CNN
F 3 "https://docs.rs-online.com/3aed/0900766b8137eed4.pdf" H 5400 2450 50  0001 L CNN
F 4 "https://uk.rs-online.com/web/p/io-expanders/0403806/" H 5200 3550 50  0001 C CNN "Source"
	1    5200 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	4400 4150 4400 4250
Wire Wire Line
	6200 6950 6400 6950
Wire Wire Line
	6200 6850 6400 6850
Wire Wire Line
	6200 6750 6400 6750
Wire Wire Line
	6200 6650 6400 6650
Wire Wire Line
	6200 6550 6400 6550
Wire Wire Line
	6200 6450 6400 6450
Wire Wire Line
	6200 6350 6400 6350
Wire Wire Line
	6200 6250 6400 6250
Wire Wire Line
	6200 6050 6400 6050
Wire Wire Line
	6200 5950 6400 5950
Wire Wire Line
	6200 5850 6400 5850
Wire Wire Line
	6200 5750 6400 5750
Wire Wire Line
	6200 5650 6400 5650
Wire Wire Line
	6200 5550 6400 5550
Wire Wire Line
	6200 5450 6400 5450
Wire Wire Line
	6200 5350 6400 5350
Text GLabel 6400 6950 2    50   Output ~ 0
SGF
Text GLabel 6400 6850 2    50   Output ~ 0
SGE
Text GLabel 6400 6750 2    50   Output ~ 0
SGD
Text GLabel 6400 6650 2    50   Output ~ 0
SGC
Text GLabel 6400 6550 2    50   Output ~ 0
SGB
Text GLabel 6400 6450 2    50   Output ~ 0
SGA
Text GLabel 6400 6350 2    50   Output ~ 0
SG9
Text GLabel 6400 6250 2    50   Output ~ 0
SG8
Text GLabel 6400 6050 2    50   Output ~ 0
OF
Text GLabel 6400 5950 2    50   Output ~ 0
OE
Text GLabel 6400 5850 2    50   Output ~ 0
OD
Text GLabel 6400 5750 2    50   Output ~ 0
OC
Text GLabel 6400 5650 2    50   Output ~ 0
OB
Text GLabel 6400 5550 2    50   Output ~ 0
OA
Text GLabel 6400 5450 2    50   Output ~ 0
O9
Text GLabel 6400 5350 2    50   Output ~ 0
O8
NoConn ~ 4800 6050
NoConn ~ 4800 5950
$Comp
L power:GND #PWR034
U 1 1 60EBFE3A
P 5500 7350
F 0 "#PWR034" H 5500 7100 50  0001 C CNN
F 1 "GND" H 5505 7177 50  0000 C CNN
F 2 "" H 5500 7350 50  0001 C CNN
F 3 "" H 5500 7350 50  0001 C CNN
	1    5500 7350
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR033
U 1 1 60EBFE40
P 5500 5000
F 0 "#PWR033" H 5500 4850 50  0001 C CNN
F 1 "+5V" H 5515 5173 50  0000 C CNN
F 2 "" H 5500 5000 50  0001 C CNN
F 3 "" H 5500 5000 50  0001 C CNN
	1    5500 5000
	1    0    0    -1  
$EndComp
$Comp
L Signalbox:MCP23017_SP U3
U 1 1 60EBFE46
P 5500 6150
F 0 "U3" H 5100 7200 50  0000 C CNN
F 1 "MCP23017_SP" H 5100 7100 50  0000 C CNN
F 2 "Package_DIP:DIP-28_W7.62mm" H 5700 5150 50  0001 L CNN
F 3 "https://docs.rs-online.com/3aed/0900766b8137eed4.pdf" H 5700 5050 50  0001 L CNN
F 4 "https://uk.rs-online.com/web/p/io-expanders/0403806/" H 5500 6150 50  0001 C CNN "Source"
	1    5500 6150
	1    0    0    -1  
$EndComp
Wire Wire Line
	5500 5000 5500 5050
Wire Wire Line
	5500 7350 5500 7250
Wire Wire Line
	5200 4750 5200 4650
Connection ~ 4400 4250
Wire Wire Line
	4400 4250 4400 4350
Connection ~ 4400 4350
Wire Wire Line
	4400 4350 4400 4450
Wire Wire Line
	4050 5350 4800 5350
Wire Wire Line
	4150 2850 4500 2850
Wire Wire Line
	4150 2850 4150 4900
Wire Wire Line
	4050 2750 4500 2750
Wire Wire Line
	4050 2750 4050 4800
$Comp
L power:GND #PWR025
U 1 1 60FE9EF2
P 4600 7100
F 0 "#PWR025" H 4600 6850 50  0001 C CNN
F 1 "GND" H 4605 6927 50  0000 C CNN
F 2 "" H 4600 7100 50  0001 C CNN
F 3 "" H 4600 7100 50  0001 C CNN
	1    4600 7100
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR023
U 1 1 60FEA80A
P 4400 6850
F 0 "#PWR023" H 4400 6700 50  0001 C CNN
F 1 "+5V" H 4415 7023 50  0000 C CNN
F 2 "" H 4400 6850 50  0001 C CNN
F 3 "" H 4400 6850 50  0001 C CNN
	1    4400 6850
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 7100 4600 6850
Wire Wire Line
	4600 6750 4800 6750
Wire Wire Line
	4600 6850 4800 6850
Connection ~ 4600 6850
Wire Wire Line
	4600 6850 4600 6750
Wire Wire Line
	4400 6950 4400 6850
Wire Wire Line
	4400 6950 4800 6950
Wire Wire Line
	7350 5450 7350 5400
Wire Wire Line
	7700 5450 7700 5400
Wire Wire Line
	8050 5450 8050 5400
Wire Wire Line
	8400 5450 8400 5400
Wire Wire Line
	8750 5450 8750 5400
Wire Wire Line
	9100 5450 9100 5400
Wire Wire Line
	9450 5450 9450 5400
Wire Wire Line
	9800 5450 9800 5400
Wire Wire Line
	7500 5950 7500 5900
Wire Wire Line
	7850 5950 7850 5900
Wire Wire Line
	8200 5950 8200 5900
Wire Wire Line
	8550 5950 8550 5900
Wire Wire Line
	8900 5950 8900 5900
Wire Wire Line
	9250 5950 9250 5900
Wire Wire Line
	9600 5950 9600 5900
Wire Wire Line
	9950 5950 9950 5900
Text GLabel 3300 4450 2    50   Input ~ 0
WiFi
Text GLabel 7850 4900 2    50   Output ~ 0
WiFi
Wire Wire Line
	7250 4900 6700 4900
Wire Wire Line
	6700 4650 6700 4900
Wire Wire Line
	7850 4900 7750 4900
Text GLabel 3300 4250 2    50   Input ~ 0
Active
Text GLabel 10100 4900 2    50   Output ~ 0
Active
Wire Wire Line
	10100 4900 10000 4900
$Comp
L Signalbox:330R R53
U 1 1 61213B8C
P 8900 4500
F 0 "R53" H 8970 4546 50  0000 L CNN
F 1 "330R" H 8970 4455 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 8830 4500 50  0001 C CNN
F 3 "https://docs.rs-online.com/8434/0900766b8157ade3.pdf" H 8900 4500 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/through-hole-fixed-resistors/7077622/" H 8900 4500 50  0001 C CNN "Source"
	1    8900 4500
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0101
U 1 1 61213B92
P 8900 4200
F 0 "#PWR0101" H 8900 4050 50  0001 C CNN
F 1 "+5V" H 8915 4373 50  0000 C CNN
F 2 "" H 8900 4200 50  0001 C CNN
F 3 "" H 8900 4200 50  0001 C CNN
	1    8900 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	8900 4350 8900 4200
Wire Wire Line
	8900 4650 8900 4900
Wire Wire Line
	9500 4900 8900 4900
Text Notes 10400 2550 0    50   ~ 0
Green
Text Notes 10400 2650 0    50   ~ 0
Red
Text Notes 10400 2750 0    50   ~ 0
Yellow
Text Notes 10400 2850 0    50   ~ 0
Green
Text Notes 10400 2950 0    50   ~ 0
Red
Text Notes 10400 3050 0    50   ~ 0
Yellow
Text Notes 10400 3150 0    50   ~ 0
Green
Text Notes 10400 3250 0    50   ~ 0
Red
Text Notes 10400 3350 0    50   ~ 0
Yellow
Text Notes 10400 3450 0    50   ~ 0
Green
Text Notes 10400 3550 0    50   ~ 0
Red
Text Notes 10400 3650 0    50   ~ 0
Yellow
Text Notes 10400 3750 0    50   ~ 0
Green
Text Notes 10400 3850 0    50   ~ 0
Red
Text Notes 10400 3950 0    50   ~ 0
Yellow
Text Notes 10400 4050 0    50   ~ 0
Green
Text Notes 10400 4150 0    50   ~ 0
Red
Text Notes 10400 4250 0    50   ~ 0
Yellow
Text Notes 10400 4350 0    50   ~ 0
Green
Text Notes 10400 4450 0    50   ~ 0
Red
Text Notes 10400 4550 0    50   ~ 0
Yellow
Text Notes 10400 4650 0    50   ~ 0
Green
Text Notes 10400 4750 0    50   ~ 0
Red
Text Notes 10400 4850 0    50   ~ 0
Yellow
Text Notes 10400 4950 0    50   ~ 0
Green
Text Notes 8150 4950 0    50   ~ 0
Yellow
Text Notes 8150 2550 0    50   ~ 0
Green
Text Notes 8150 2650 0    50   ~ 0
Red
Text Notes 8150 2750 0    50   ~ 0
Yellow
Text Notes 8150 2850 0    50   ~ 0
Green
Text Notes 8150 2950 0    50   ~ 0
Red
Text Notes 8150 3050 0    50   ~ 0
Yellow
Text Notes 8150 3150 0    50   ~ 0
Green
Text Notes 8150 3250 0    50   ~ 0
Red
Text Notes 8150 3350 0    50   ~ 0
Yellow
Text Notes 8150 3450 0    50   ~ 0
Green
Text Notes 8150 3550 0    50   ~ 0
Red
Text Notes 8150 3650 0    50   ~ 0
Yellow
Text Notes 8150 3750 0    50   ~ 0
Green
Text Notes 8150 3850 0    50   ~ 0
Red
Text Notes 8150 3950 0    50   ~ 0
Yellow
Text Notes 8150 4050 0    50   ~ 0
Green
Text Notes 8150 4150 0    50   ~ 0
Red
Text Notes 8150 4250 0    50   ~ 0
Yellow
Text Notes 8150 4350 0    50   ~ 0
Green
Text Notes 8150 4450 0    50   ~ 0
Red
Text Notes 8150 4550 0    50   ~ 0
Yellow
Text Notes 8150 4650 0    50   ~ 0
Green
Text Notes 8150 4750 0    50   ~ 0
Red
Text Notes 8150 4850 0    50   ~ 0
Yellow
Wire Wire Line
	3300 4250 2950 4250
Wire Wire Line
	2250 3750 2250 3600
Wire Wire Line
	2300 1500 2650 1500
Wire Wire Line
	2650 1500 2650 1350
Wire Wire Line
	2350 6800 2250 6800
Connection ~ 2250 6800
Wire Wire Line
	2250 7350 3200 7350
Wire Wire Line
	2250 6800 2250 7350
Connection ~ 3200 7350
$Comp
L power:+5V #PWR0102
U 1 1 615DB802
P 5200 2350
F 0 "#PWR0102" H 5200 2200 50  0001 C CNN
F 1 "+5V" H 5215 2523 50  0000 C CNN
F 2 "" H 5200 2350 50  0001 C CNN
F 3 "" H 5200 2350 50  0001 C CNN
	1    5200 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	5200 2350 5200 2450
Wire Wire Line
	1300 1500 1200 1500
Wire Wire Line
	1200 1500 1200 3200
NoConn ~ 1750 4050
Wire Wire Line
	2250 6250 2250 6800
Wire Wire Line
	2350 6250 2350 6800
Wire Wire Line
	2450 6250 2450 6800
Wire Wire Line
	2450 6800 2350 6800
Connection ~ 2350 6800
$Comp
L ESP32_DevKit_V4:ESP32_DevKit_V4 U1
U 1 1 60914B0D
P 2350 5150
F 0 "U1" H 1950 6600 50  0000 R CNN
F 1 "ESP32_DevKit_V4" H 2100 6500 50  0000 R CNN
F 2 "ESP32_DevKit_V4:esp32_devkit_v4" H 1900 6500 50  0001 C CNN
F 3 "https://docs.espressif.com/projects/esp-idf/en/latest/esp32/hw-reference/esp32/get-started-devkitc.html#" H 1900 6500 50  0001 C CNN
F 4 "https://www.amazon.co.uk/Dorhea-ESP32-DevKitC-Development-ESP32-WROOM-32U-Amplifier/dp/B08T5RBCXS/ref=sr_1_10?dchild=1&keywords=esp32+devkitc+v4&qid=1621968791&sr=8-10" H 2350 5150 50  0001 C CNN "Source"
	1    2350 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3250 4850 3250 4950
Text GLabel 1500 4250 0    50   BiDi ~ 0
SDA
Text GLabel 1500 4350 0    50   Output ~ 0
SCL
Wire Wire Line
	1500 4250 1750 4250
Wire Wire Line
	1500 4350 1750 4350
Text GLabel 3850 4800 0    50   BiDi ~ 0
SDA
Text GLabel 3850 4900 0    50   Input ~ 0
SCL
Wire Wire Line
	3850 4900 4150 4900
Wire Wire Line
	3850 4800 4050 4800
Connection ~ 4050 4800
Wire Wire Line
	4050 4800 4050 5350
Wire Wire Line
	3100 6750 3100 7250
Wire Wire Line
	3100 7250 3300 7250
Connection ~ 4150 4900
Wire Wire Line
	4150 4900 4150 5450
Wire Wire Line
	4150 5450 4800 5450
Wire Wire Line
	3800 6350 3900 6350
Connection ~ 3800 6350
Wire Wire Line
	3800 6150 3800 6350
Wire Wire Line
	3800 5700 3800 5850
$Comp
L power:+5V #PWR017
U 1 1 609DE9D6
P 3800 5700
F 0 "#PWR017" H 3800 5550 50  0001 C CNN
F 1 "+5V" H 3815 5873 50  0000 C CNN
F 2 "" H 3800 5700 50  0001 C CNN
F 3 "" H 3800 5700 50  0001 C CNN
	1    3800 5700
	1    0    0    -1  
$EndComp
$Comp
L Signalbox:470R R7
U 1 1 609DC7A6
P 3800 6000
F 0 "R7" H 3870 6046 50  0000 L CNN
F 1 "470R" H 3870 5955 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 3730 6000 50  0001 C CNN
F 3 "https://docs.rs-online.com/b9c3/0900766b8157ade8.pdf" H 3800 6000 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/through-hole-fixed-resistors/7077647/" H 3800 6000 50  0001 C CNN "Source"
	1    3800 6000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3450 6450 3900 6450
Wire Wire Line
	3450 6750 3450 6450
$Comp
L power:GND #PWR012
U 1 1 60980A90
P 3450 6750
F 0 "#PWR012" H 3450 6500 50  0001 C CNN
F 1 "GND" H 3455 6577 50  0000 C CNN
F 2 "" H 3450 6750 50  0001 C CNN
F 3 "" H 3450 6750 50  0001 C CNN
	1    3450 6750
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 6350 3800 6350
Wire Wire Line
	3750 6650 3900 6650
Wire Wire Line
	3750 6750 3750 6650
Text GLabel 3350 5550 2    50   Output ~ 0
Stop
Wire Wire Line
	3800 6550 3900 6550
Text GLabel 3800 6550 0    50   Input ~ 0
Stop
$Comp
L Pauls~Parts:RS_Pro_4_Way_PCB_terminal_block_2.54mm J3
U 1 1 6092CDE3
P 4100 6450
F 0 "J3" H 4180 6442 50  0000 L CNN
F 1 "Buttons" H 4180 6351 50  0000 L CNN
F 2 "Pauls Parts:4WayScrewTerminal_2.54mm" H 4100 6450 50  0001 C CNN
F 3 "https://docs.rs-online.com/02e6/0900766b8157c7ed.pdf" H 4100 6450 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/pcb-terminal-blocks/7901102/" H 4100 6450 50  0001 C CNN "Source"
	1    4100 6450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR016
U 1 1 609528E5
P 3750 6750
F 0 "#PWR016" H 3750 6500 50  0001 C CNN
F 1 "GND" H 3755 6577 50  0000 C CNN
F 2 "" H 3750 6750 50  0001 C CNN
F 3 "" H 3750 6750 50  0001 C CNN
	1    3750 6750
	1    0    0    -1  
$EndComp
Wire Wire Line
	2950 5650 3600 5650
Wire Wire Line
	3600 5650 3600 6350
Wire Wire Line
	2950 5550 3250 5550
Wire Wire Line
	3250 5250 3250 5550
Connection ~ 3250 5550
Wire Wire Line
	3250 5550 3350 5550
Wire Wire Line
	3100 7250 1350 7250
Connection ~ 3100 7250
Wire Wire Line
	1750 5250 1350 5250
Wire Wire Line
	1350 5250 1350 7250
NoConn ~ 1750 4450
NoConn ~ 1750 4650
NoConn ~ 1750 4750
NoConn ~ 1750 4850
NoConn ~ 1750 5050
NoConn ~ 1750 5150
NoConn ~ 1750 5350
NoConn ~ 1750 5450
NoConn ~ 1750 5650
NoConn ~ 2950 5950
NoConn ~ 2950 5850
NoConn ~ 2950 5450
NoConn ~ 2950 5350
NoConn ~ 2950 5250
NoConn ~ 2950 5150
NoConn ~ 2950 5050
NoConn ~ 2950 4950
NoConn ~ 2950 4850
NoConn ~ 2950 4750
NoConn ~ 2950 4650
NoConn ~ 2950 4550
NoConn ~ 2950 4350
NoConn ~ 2950 4150
NoConn ~ 2950 4050
NoConn ~ 13100 1700
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 60B1D86B
P 3000 1350
F 0 "#FLG0101" H 3000 1425 50  0001 C CNN
F 1 "PWR_FLAG" H 3000 1523 50  0000 C CNN
F 2 "" H 3000 1350 50  0001 C CNN
F 3 "~" H 3000 1350 50  0001 C CNN
	1    3000 1350
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG0102
U 1 1 60B1EEA2
P 1450 2950
F 0 "#FLG0102" H 1450 3025 50  0001 C CNN
F 1 "PWR_FLAG" H 1450 3123 50  0000 C CNN
F 2 "" H 1450 2950 50  0001 C CNN
F 3 "~" H 1450 2950 50  0001 C CNN
	1    1450 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	1450 3300 1450 2950
Connection ~ 1450 3300
Wire Wire Line
	2650 1500 3000 1500
Wire Wire Line
	3000 1500 3000 1350
Connection ~ 2650 1500
Wire Wire Line
	2950 5750 4250 5750
Wire Wire Line
	4450 5750 4450 6250
Wire Wire Line
	4450 6250 4800 6250
Wire Wire Line
	4500 3650 4250 3650
Wire Wire Line
	4250 3650 4250 5750
Connection ~ 4250 5750
Wire Wire Line
	4250 5750 4450 5750
$EndSCHEMATC
