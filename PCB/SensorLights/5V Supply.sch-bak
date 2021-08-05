EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 2
Title "West Huntspill Railway Signalbox Module"
Date "2021-05-16"
Rev "V2.1"
Comp ""
Comment1 ""
Comment2 "creativecommons.org/licenses/by/4.0/"
Comment3 "License: CC BY 4.0"
Comment4 "Author: Paul Barnard"
$EndDescr
$Comp
L Device:CP C2
U 1 1 60939DF4
P 4400 3700
F 0 "C2" H 4518 3746 50  0000 L CNN
F 1 "100uF" H 4518 3655 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D5.0mm_P2.00mm" H 4438 3550 50  0001 C CNN
F 3 "https://docs.rs-online.com/bcd8/0900766b814894c5.pdf" H 4400 3700 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/aluminium-capacitors/3150962/" H 4400 3700 50  0001 C CNN "Source"
	1    4400 3700
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 60939DFA
P 6250 3200
F 0 "C3" V 5998 3200 50  0000 C CNN
F 1 "10nF" V 6089 3200 50  0000 C CNN
F 2 "Capacitor_THT:C_Rect_L10.0mm_W4.0mm_P7.50mm_MKS4" H 6288 3050 50  0001 C CNN
F 3 "https://docs.rs-online.com/9793/0900766b814700cb.pdf" H 6250 3200 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/polypropylene-film-capacitors/8961512/" V 6250 3200 50  0001 C CNN "Source"
	1    6250 3200
	0    1    1    0   
$EndComp
$Comp
L Device:CP C4
U 1 1 60939E00
P 7300 3750
F 0 "C4" H 7418 3796 50  0000 L CNN
F 1 "68uF" H 7418 3705 50  0000 L CNN
F 2 "Capacitor_THT:C_Radial_D4.0mm_H7.0mm_P1.50mm" H 7338 3600 50  0001 C CNN
F 3 "https://docs.rs-online.com/f424/0900766b80becf93.pdf" H 7300 3750 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/aluminium-capacitors/0572312/" H 7300 3750 50  0001 C CNN "Source"
	1    7300 3750
	1    0    0    -1  
$EndComp
$Comp
L Device:L L1
U 1 1 60939E06
P 6900 3400
F 0 "L1" V 7090 3400 50  0000 C CNN
F 1 "47uH" V 6999 3400 50  0000 C CNN
F 2 "Inductor_THT:L_Toroid_Vertical_L14.7mm_W8.6mm_P5.58mm_Pulse_KM-1" H 6900 3400 50  0001 C CNN
F 3 "https://docs.rs-online.com/24ef/0900766b81505716.pdf" H 6900 3400 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/leaded-inductors/1048418/" V 6900 3400 50  0001 C CNN "Source"
	1    6900 3400
	0    -1   -1   0   
$EndComp
$Comp
L Device:D_Schottky D5
U 1 1 60939E0C
P 6600 3750
F 0 "D5" V 6554 3830 50  0000 L CNN
F 1 "1N5819RL" V 6645 3830 50  0000 L CNN
F 2 "Diode_THT:D_DO-41_SOD81_P7.62mm_Horizontal" H 6600 3750 50  0001 C CNN
F 3 "https://docs.rs-online.com/6961/0900766b80da3e32.pdf" H 6600 3750 50  0001 C CNN
F 4 "https://uk.rs-online.com/web/p/schottky-diodes-rectifiers/6870754/" V 6600 3750 50  0001 C CNN "Source"
	1    6600 3750
	0    1    1    0   
$EndComp
Wire Wire Line
	6600 3200 6400 3200
Wire Wire Line
	6100 3200 5900 3200
NoConn ~ 4900 3200
Wire Wire Line
	6600 3200 6600 3400
Wire Wire Line
	5900 3400 6600 3400
Connection ~ 6600 3400
Wire Wire Line
	6600 3400 6750 3400
Wire Wire Line
	7050 3400 7300 3400
Connection ~ 7300 3400
Wire Wire Line
	7300 3000 7300 3400
Wire Wire Line
	6600 3600 6600 3400
Wire Wire Line
	7300 3600 7300 3400
Text HLabel 3550 3000 0    50   UnSpc ~ 0
6V-24V
Text HLabel 8200 3400 2    50   UnSpc ~ 0
5V
Wire Wire Line
	7300 3400 8200 3400
Text HLabel 8200 4100 2    50   UnSpc ~ 0
GND
Wire Wire Line
	7300 4100 8200 4100
Wire Wire Line
	7300 3900 7300 4100
Wire Wire Line
	6600 3900 6600 4100
Connection ~ 7300 4100
Wire Wire Line
	5400 4100 6600 4100
Wire Wire Line
	5400 3600 5400 4100
Connection ~ 6600 4100
Wire Wire Line
	4400 4100 5400 4100
Wire Wire Line
	4400 3850 4400 4100
Connection ~ 5400 4100
Wire Wire Line
	3550 3000 4400 3000
Wire Wire Line
	4400 3550 4400 3000
Wire Wire Line
	7300 3000 5900 3000
Wire Wire Line
	4400 3000 4900 3000
Connection ~ 4400 3000
Wire Wire Line
	6600 4100 7300 4100
NoConn ~ 4500 1850
$Comp
L Regulator_Switching:LM2675N-5 U3
U 1 1 60DCF0DC
P 5400 3200
F 0 "U3" H 5400 3667 50  0000 C CNN
F 1 "LM2675N-5" H 5400 3576 50  0000 C CNN
F 2 "Package_DIP:DIP-8_W7.62mm" H 5450 2850 50  0001 L CIN
F 3 "http://www.ti.com/lit/ds/symlink/lm2675.pdf" H 5400 3200 50  0001 C CNN
F 4 "https://uk.farnell.com/texas-instruments/lm2675n-5-0-nopb/ic-dc-dc-converter-1a/dp/3122905?st=lm2675n-5" H 5400 3200 50  0001 C CNN "Source"
	1    5400 3200
	1    0    0    -1  
$EndComp
$EndSCHEMATC
