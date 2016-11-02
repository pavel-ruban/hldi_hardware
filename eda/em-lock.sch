EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:stm32
LIBS:switches
LIBS:powerint
LIBS:Oscillators
LIBS:ab2_usb
LIBS:video
LIBS:ttl_ieee
LIBS:transf
LIBS:supertex
LIBS:stm8
LIBS:silabs
LIBS:sensors
LIBS:rfcom
LIBS:relays
LIBS:references
LIBS:pspice
LIBS:onsemi
LIBS:nxp_armmcu
LIBS:nordicsemi
LIBS:msp430
LIBS:motor_drivers
LIBS:microchip_pic32mcu
LIBS:microchip_pic18mcu
LIBS:microchip_pic16mcu
LIBS:microchip_pic12mcu
LIBS:microchip_pic10mcu
LIBS:microchip_dspic33dsc
LIBS:maxim
LIBS:logo
LIBS:ir
LIBS:hc11
LIBS:graphic
LIBS:gennum
LIBS:ftdi
LIBS:elec-unifil
LIBS:diode
LIBS:dc-dc
LIBS:cmos_ieee
LIBS:brooktre
LIBS:analog_devices
LIBS:actel
LIBS:ac-dc
LIBS:Zilog
LIBS:Xicor
LIBS:Power_Management
LIBS:Lattice
LIBS:ESD_Protection
LIBS:Altera
LIBS:74xgxx
LIBS:ab2_7segment
LIBS:ab2_audio
LIBS:ab2_buffer
LIBS:ab2_capacitor
LIBS:ab2_connectivity
LIBS:ab2_dac
LIBS:ab2_diode
LIBS:ab2_gpio_expansion
LIBS:ab2_header
LIBS:ab2_idc
LIBS:ab2_inductor
LIBS:ab2_input_devices
LIBS:ab2_jumper
LIBS:ab2_lcd
LIBS:ab2_led
LIBS:ab2_memory
LIBS:ab2_opamp
LIBS:ab2_pot
LIBS:ab2_power
LIBS:ab2_regulator
LIBS:ab2_relay
LIBS:ab2_resistor
LIBS:ab2_sensor
LIBS:ab2_stepper
LIBS:ab2_supply
LIBS:ab2_terminal_block
LIBS:ab2_test
LIBS:ab2_transistor
LIBS:ab2_uC
LIBS:ab2_xtal
LIBS:SparkFun
LIBS:I20
LIBS:em-lock-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 2
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L LED_RCBG D1
U 1 1 57BF0C43
P 2300 1600
F 0 "D1" V 2350 2050 50  0000 R CNN
F 1 "LED_RCBG" V 2250 2300 50  0000 R CNN
F 2 "I20:LED-RGB-5MM_Common_Cathode" H 2300 1550 50  0001 C CNN
F 3 "" H 2300 1550 50  0000 C CNN
	1    2300 1600
	0    1    1    0   
$EndComp
$Comp
L GND #PWR01
U 1 1 57BF0D6B
P 2300 1300
F 0 "#PWR01" H 2300 1050 50  0001 C CNN
F 1 "GND" H 2305 1127 50  0000 C CNN
F 2 "" H 2300 1300 50  0000 C CNN
F 3 "" H 2300 1300 50  0000 C CNN
	1    2300 1300
	-1   0    0    1   
$EndComp
$Comp
L MMBT3906 Q2
U 1 1 57BF0F6F
P 2400 3300
F 0 "Q2" V 2350 3100 50  0000 L CNN
F 1 "MMBT3906" V 2650 3100 50  0000 L CNN
F 2 "I20:SOT-23" V 2750 3100 50  0000 L CIN
F 3 "" H 2400 3300 50  0000 L CNN
	1    2400 3300
	-1   0    0    -1  
$EndComp
$Comp
L MMBT3906 Q1
U 1 1 57BF0FDE
P 2200 2850
F 0 "Q1" V 2150 2650 50  0000 L CNN
F 1 "MMBT3906" V 2450 2650 50  0000 L CNN
F 2 "I20:SOT-23" V 2550 2650 50  0000 L CIN
F 3 "" H 2200 2850 50  0000 L CNN
	1    2200 2850
	-1   0    0    -1  
$EndComp
$Comp
L R R10
U 1 1 57BF1730
P 2500 2100
F 0 "R10" V 2500 2050 50  0000 L CNN
F 1 "82R" V 2600 2000 50  0000 L CNN
F 2 "I20:R_0805" V 2430 2100 50  0001 C CNN
F 3 "" H 2500 2100 50  0000 C CNN
	1    2500 2100
	-1   0    0    1   
$EndComp
$Comp
L R R9
U 1 1 57BF17B4
P 2300 2100
F 0 "R9" V 2300 2050 50  0000 L CNN
F 1 "15R" V 2400 2000 50  0000 L CNN
F 2 "I20:R_0805" V 2230 2100 50  0001 C CNN
F 3 "" H 2300 2100 50  0000 C CNN
	1    2300 2100
	-1   0    0    1   
$EndComp
$Comp
L R R6
U 1 1 57BF1859
P 2100 2100
F 0 "R6" V 2100 2050 50  0000 L CNN
F 1 "10R" V 2200 2000 50  0000 L CNN
F 2 "I20:R_0805" V 2030 2100 50  0001 C CNN
F 3 "" H 2100 2100 50  0000 C CNN
	1    2100 2100
	-1   0    0    1   
$EndComp
$Comp
L VDD #PWR02
U 1 1 57BF1F35
P 2300 4100
F 0 "#PWR02" H 2300 3950 50  0001 C CNN
F 1 "VDD" H 2317 4273 50  0000 C CNN
F 2 "" H 2300 4100 50  0000 C CNN
F 3 "" H 2300 4100 50  0000 C CNN
	1    2300 4100
	-1   0    0    1   
$EndComp
$Comp
L R R12
U 1 1 57BF244F
P 2950 3300
F 0 "R12" V 2743 3300 50  0000 C CNN
F 1 "2k" V 2834 3300 50  0000 C CNN
F 2 "I20:R_0805" V 2880 3300 50  0001 C CNN
F 3 "" H 2950 3300 50  0000 C CNN
	1    2950 3300
	0    -1   -1   0   
$EndComp
$Comp
L R R11
U 1 1 57BF251B
P 2950 2850
F 0 "R11" V 2743 2850 50  0000 C CNN
F 1 "2k" V 2834 2850 50  0000 C CNN
F 2 "I20:R_0805" V 2880 2850 50  0001 C CNN
F 3 "" H 2950 2850 50  0000 C CNN
	1    2950 2850
	0    -1   -1   0   
$EndComp
$Comp
L R R3
U 1 1 57BF341E
P 1850 2100
F 0 "R3" V 1850 2050 50  0000 L CNN
F 1 "82R" V 1950 2000 50  0000 L CNN
F 2 "I20:R_0805" V 1780 2100 50  0001 C CNN
F 3 "" H 1850 2100 50  0000 C CNN
	1    1850 2100
	-1   0    0    1   
$EndComp
$Comp
L R R2
U 1 1 57BF3424
P 1650 2100
F 0 "R2" V 1650 2050 50  0000 L CNN
F 1 "15R" V 1750 2000 50  0000 L CNN
F 2 "I20:R_0805" V 1580 2100 50  0001 C CNN
F 3 "" H 1650 2100 50  0000 C CNN
	1    1650 2100
	-1   0    0    1   
$EndComp
$Comp
L R R1
U 1 1 57BF342A
P 1450 2100
F 0 "R1" V 1450 2050 50  0000 L CNN
F 1 "10R" V 1550 2000 50  0000 L CNN
F 2 "I20:R_0805" V 1380 2100 50  0001 C CNN
F 3 "" H 1450 2100 50  0000 C CNN
	1    1450 2100
	-1   0    0    1   
$EndComp
$Comp
L CONN_02X04 P6
U 1 1 57BFAC84
P 8750 5950
F 0 "P6" H 8750 6350 50  0000 C CNN
F 1 "ESP8266_CONN" H 8700 6250 50  0000 C CNN
F 2 "I20:ESP8266_CONN" H 8750 4750 50  0001 C CNN
F 3 "" H 8750 4750 50  0000 C CNN
	1    8750 5950
	1    0    0    -1  
$EndComp
$Comp
L VDD #PWR03
U 1 1 57BFC190
P 9450 5800
F 0 "#PWR03" H 9450 5650 50  0001 C CNN
F 1 "VDD" V 9450 6000 50  0000 C CNN
F 2 "" H 9450 5800 50  0000 C CNN
F 3 "" H 9450 5800 50  0000 C CNN
	1    9450 5800
	0    1    1    0   
$EndComp
$Comp
L C C8
U 1 1 57BFC498
P 9350 5600
F 0 "C8" H 9450 5700 50  0000 L CNN
F 1 "10uF" H 9400 5500 50  0000 L CNN
F 2 "I20:C_0805" H 9388 5450 50  0001 C CNN
F 3 "" H 9350 5600 50  0000 C CNN
	1    9350 5600
	1    0    0    -1  
$EndComp
$Comp
L C C7
U 1 1 57BFCAA7
P 9100 5600
F 0 "C7" H 9150 5700 50  0000 L CNN
F 1 "100nF" H 9100 5500 50  0000 L CNN
F 2 "I20:C_0805" H 9138 5450 50  0001 C CNN
F 3 "" H 9100 5600 50  0000 C CNN
	1    9100 5600
	1    0    0    -1  
$EndComp
$Comp
L Q_NMOS_DGS Q4
U 1 1 57BF2360
P 8950 3800
F 0 "Q4" H 8800 3950 50  0000 L CNN
F 1 "Q_NMOS_DGS" V 9200 3500 50  0000 L CNN
F 2 "I20:SOT-23" H 9150 3900 50  0001 C CNN
F 3 "" H 8950 3800 50  0000 C CNN
	1    8950 3800
	-1   0    0    -1  
$EndComp
$Comp
L R R24
U 1 1 57BF2660
P 9700 3800
F 0 "R24" V 9493 3800 50  0000 C CNN
F 1 "240" V 9584 3800 50  0000 C CNN
F 2 "I20:R_0805" V 9630 3800 50  0001 C CNN
F 3 "" H 9700 3800 50  0000 C CNN
	1    9700 3800
	0    1    1    0   
$EndComp
$Comp
L GND #PWR04
U 1 1 57BF3222
P 8850 4450
F 0 "#PWR04" H 8850 4200 50  0001 C CNN
F 1 "GND" H 8855 4277 50  0000 C CNN
F 2 "" H 8850 4450 50  0000 C CNN
F 3 "" H 8850 4450 50  0000 C CNN
	1    8850 4450
	1    0    0    -1  
$EndComp
$Comp
L ZENER D3
U 1 1 57BF338B
P 9150 4050
F 0 "D3" V 9104 4129 50  0000 L CNN
F 1 "ZENER-MOSFET-GATE" V 9500 4050 50  0000 L CNN
F 2 "I20:MiniMELF_Standard" H 9150 4050 50  0001 C CNN
F 3 "" H 9150 4050 50  0000 C CNN
	1    9150 4050
	0    1    1    0   
$EndComp
$Comp
L R R23
U 1 1 57BF37C0
P 9500 4100
F 0 "R23" H 9570 4146 50  0000 L CNN
F 1 "10k" H 9570 4055 50  0000 L CNN
F 2 "I20:R_0805" V 9430 4100 50  0001 C CNN
F 3 "" H 9500 4100 50  0000 C CNN
	1    9500 4100
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X04 P7
U 1 1 57BF438E
P 9150 3100
F 0 "P7" H 9227 3141 50  0000 L CNN
F 1 "LOCK_CONN" H 9227 3050 50  0000 L CNN
F 2 "I20:PhoenixContact_MSTBA-G_04x5.08mm_Angled" H 9150 3100 50  0001 C CNN
F 3 "" H 9150 3100 50  0000 C CNN
	1    9150 3100
	1    0    0    -1  
$EndComp
$Comp
L D D2
U 1 1 57BF47C2
P 8650 3350
F 0 "D2" H 8500 3250 50  0000 L CNN
F 1 "M7" H 8700 3250 50  0000 L CNN
F 2 "I20:SMA_Standard" H 8650 3350 50  0001 C CNN
F 3 "" H 8650 3350 50  0000 C CNN
	1    8650 3350
	0    1    1    0   
$EndComp
Text GLabel 8550 3150 0    60   Input ~ 0
EM_LOCK
$Comp
L VDD #PWR05
U 1 1 57BFAAAC
P 4800 3250
F 0 "#PWR05" H 4800 3100 50  0001 C CNN
F 1 "VDD" H 4800 3400 50  0000 C CNN
F 2 "" H 4800 3250 50  0000 C CNN
F 3 "" H 4800 3250 50  0000 C CNN
	1    4800 3250
	1    0    0    -1  
$EndComp
$Comp
L R R15
U 1 1 57BFAAB2
P 4800 3500
F 0 "R15" H 4870 3546 50  0000 L CNN
F 1 "10k" H 4870 3455 50  0000 L CNN
F 2 "I20:R_0805" V 4730 3500 50  0001 C CNN
F 3 "" H 4800 3500 50  0000 C CNN
	1    4800 3500
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X02 P5
U 1 1 57BFAABE
P 5000 3950
F 0 "P5" H 5078 3991 50  0000 L CNN
F 1 "BTN_CALL" H 4900 3800 50  0000 L CNN
F 2 "I20:JST_XH_B02B-XH-A_02x2.50mm_Straight" H 5000 3950 50  0001 C CNN
F 3 "" H 5000 3950 50  0000 C CNN
	1    5000 3950
	1    0    0    -1  
$EndComp
$Comp
L 74LVC2G14 U1
U 2 1 57C00A6E
P 5750 3700
F 0 "U1" H 5725 3967 50  0000 C CNN
F 1 "74LVC2G14" H 5725 3876 50  0000 C CNN
F 2 "I20:SOT-23-6" H 5900 3500 50  0001 C CNN
F 3 "" H 5750 3700 50  0000 C CNN
	2    5750 3700
	1    0    0    -1  
$EndComp
$Sheet
S 8100 800  2100 1400
U 5818E6EE
F0 "stm32f103c8" 60
F1 "stm32.sch" 60
F2 "PD0/RCC_OSC_IN" I L 8100 1050 60 
F3 "PD1/RCC_OSC_OUT" I L 8100 1150 60 
F4 "NRST" I L 8100 1250 60 
F5 "PB2/BOOT1" I L 8100 1350 60 
F6 "BOOT0" I L 8100 1450 60 
F7 "VBAT" I L 8100 1550 60 
F8 "PIN_RED" I L 8100 1650 60 
F9 "PIN_GREEN" I L 8100 1750 60 
F10 "PIN_BLUE" I L 8100 1850 60 
F11 "ESP_RESET" I L 8100 1950 60 
F12 "UART_INT_TX" I L 8100 2050 60 
F13 "UART_INT_RX" I L 8100 2150 60 
F14 "BTN_OPEN" I R 10200 850 60 
F15 "BTN_CALL" I R 10200 950 60 
F16 "LOCK_OPEN" I R 10200 1050 60 
F17 "UART_EXT_TX" I R 10200 1150 60 
F18 "UART_EXT_RX" I R 10200 1250 60 
F19 "SWDIO" I R 10200 1350 60 
F20 "SWCLK" I R 10200 1450 60 
F21 "PB13_SPI2_SCK" I R 10200 1550 60 
F22 "PB14_SPI2_MISO" I R 10200 1650 60 
F23 "PB15_SPI2_MOSI" I R 10200 1850 60 
F24 "PB8_RC522A_IRQ" I R 10200 1750 60 
F25 "PB9_RC522B_IRQ" I R 10200 1950 60 
F26 "PB7_RC522_RST" I R 10200 2050 60 
F27 "PB5_RC522A_CS" I L 8100 850 60 
F28 "PB6_RC522B_CS" I L 8100 950 60 
$EndSheet
Text HLabel 3100 3750 2    60   Input ~ 0
PIN_RED
Text HLabel 3100 3300 2    60   Input ~ 0
PIN_GREEN
Text HLabel 3100 2850 2    60   Input ~ 0
PIN_BLUE
Text HLabel 9450 5900 2    60   Input ~ 0
ESP_RESET
Text HLabel 8450 5800 0    60   Input ~ 0
UART_INT_TX
Text HLabel 10000 3800 2    60   Input ~ 0
LOCK_OPEN
Text HLabel 8550 2950 0    60   Input ~ 0
UART_EXT_TX
Text HLabel 8550 3050 0    60   Input ~ 0
UART_EXT_RX
Text HLabel 6000 3700 2    60   Input ~ 0
BTN_CALL
Text Notes 3600 650  2    60   ~ 0
RGB LED connector/placement
Text Notes 10250 5100 0    60   ~ 0
ESP8266 connector
Text Notes 10250 2650 0    60   ~ 0
Em-lock connector
Text Notes 10500 650  0    60   ~ 0
CPU + periph
Text Notes 6350 2650 0    60   ~ 0
Call button
Text Notes 6350 650  0    60   ~ 0
Open button
$Comp
L R R13
U 1 1 57BF21A1
P 2950 3750
F 0 "R13" V 2743 3750 50  0000 C CNN
F 1 "2k" V 2834 3750 50  0000 C CNN
F 2 "I20:R_0805" V 2880 3750 50  0001 C CNN
F 3 "" H 2950 3750 50  0000 C CNN
	1    2950 3750
	0    -1   -1   0   
$EndComp
$Comp
L MMBT3906 Q3
U 1 1 57BF0EF1
P 2600 3750
F 0 "Q3" V 2550 3550 50  0000 L CNN
F 1 "MMBT3906" V 2850 3550 50  0000 L CNN
F 2 "I20:SOT-23" V 2950 3550 50  0000 L CIN
F 3 "" H 2600 3750 50  0000 L CNN
	1    2600 3750
	-1   0    0    -1  
$EndComp
$Comp
L CONN_01X04 P1
U 1 1 57BF398B
P 750 1600
F 0 "P1" H 800 1350 50  0000 R CNN
F 1 "EXT_LED_CONN" H 600 1850 50  0000 R CNN
F 2 "I20:JST_XH_B04B-XH-A_04x2.50mm_Straight" H 750 1600 50  0001 C CNN
F 3 "" H 750 1600 50  0000 C CNN
	1    750  1600
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR06
U 1 1 582803A4
P 1150 1750
F 0 "#PWR06" H 1150 1500 50  0001 C CNN
F 1 "GND" H 1150 1550 50  0000 C CNN
F 2 "" H 1150 1750 50  0000 C CNN
F 3 "" H 1150 1750 50  0000 C CNN
	1    1150 1750
	1    0    0    -1  
$EndComp
$Comp
L R R22
U 1 1 5828371B
P 9250 6000
F 0 "R22" V 9250 5950 50  0000 C CNN
F 1 "0" V 9250 6050 50  0000 C CNN
F 2 "I20:R_0805" V 9180 6000 50  0001 C CNN
F 3 "" H 9250 6000 50  0000 C CNN
	1    9250 6000
	0    1    1    0   
$EndComp
$Comp
L CONN_01X08 P2
U 1 1 5829A13D
P 1450 6350
F 0 "P2" H 1527 6391 50  0000 L CNN
F 1 "RC522A" H 1527 6300 50  0000 L CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x08" H 1450 6350 50  0001 C CNN
F 3 "" H 1450 6350 50  0000 C CNN
	1    1450 6350
	-1   0    0    1   
$EndComp
$Comp
L VDD #PWR07
U 1 1 5829C375
P 2250 6000
F 0 "#PWR07" H 2250 5850 50  0001 C CNN
F 1 "VDD" V 2250 6200 50  0000 C CNN
F 2 "" H 2250 6000 50  0000 C CNN
F 3 "" H 2250 6000 50  0000 C CNN
	1    2250 6000
	0    1    1    0   
$EndComp
$Comp
L C C2
U 1 1 5829DC43
P 2200 5800
F 0 "C2" H 2250 5700 50  0000 C CNN
F 1 "10uF" H 2050 5700 50  0000 C CNN
F 2 "I20:C_0805" H 2238 5650 50  0001 C CNN
F 3 "" H 2200 5800 50  0000 C CNN
	1    2200 5800
	-1   0    0    1   
$EndComp
$Comp
L C C1
U 1 1 5829DD7E
P 1950 5800
F 0 "C1" H 2000 5700 50  0000 C CNN
F 1 "100nF" H 2200 5700 50  0000 C CNN
F 2 "I20:C_0805" H 1988 5650 50  0001 C CNN
F 3 "" H 1950 5800 50  0000 C CNN
	1    1950 5800
	-1   0    0    1   
$EndComp
Text HLabel 2250 6700 2    60   Input ~ 0
PB5_RC522A_CS
Text HLabel 2250 6600 2    60   Input ~ 0
PB13_SPI2_SCK
Text HLabel 2250 6400 2    60   Input ~ 0
PB14_SPI2_MISO
Text HLabel 2250 6500 2    60   Input ~ 0
PB15_SPI2_MOSI
Text HLabel 2250 6300 2    60   Input ~ 0
PB8_RC522A_IRQ
Text HLabel 2250 6100 2    60   Input ~ 0
PB7_RC522_RST
$Comp
L R R7
U 1 1 582D04E5
P 2100 6900
F 0 "R7" V 2100 6850 50  0000 L CNN
F 1 "4.7k" V 2050 7050 50  0000 L CNN
F 2 "I20:R_0805" V 2030 6900 50  0001 C CNN
F 3 "" H 2100 6900 50  0000 C CNN
	1    2100 6900
	-1   0    0    1   
$EndComp
$Comp
L R R5
U 1 1 582D5886
P 2000 6900
F 0 "R5" V 2000 6850 50  0000 L CNN
F 1 "4.7k" V 1950 7050 50  0000 L CNN
F 2 "I20:R_0805" V 1930 6900 50  0001 C CNN
F 3 "" H 2000 6900 50  0000 C CNN
	1    2000 6900
	-1   0    0    1   
$EndComp
$Comp
L R R4
U 1 1 582D5FEB
P 1900 6900
F 0 "R4" V 1900 6850 50  0000 L CNN
F 1 "4.7k" V 1850 7050 50  0000 L CNN
F 2 "I20:R_0805" V 1830 6900 50  0001 C CNN
F 3 "" H 1900 6900 50  0000 C CNN
	1    1900 6900
	-1   0    0    1   
$EndComp
$Comp
L GNDD #PWR08
U 1 1 582EA563
P 1650 6200
F 0 "#PWR08" H 1650 5950 50  0001 C CNN
F 1 "GNDD" V 1655 6072 50  0000 R CNN
F 2 "" H 1650 6200 50  0000 C CNN
F 3 "" H 1650 6200 50  0000 C CNN
	1    1650 6200
	0    -1   -1   0   
$EndComp
$Comp
L R R8
U 1 1 582F5135
P 2200 6900
F 0 "R8" V 2200 6850 50  0000 L CNN
F 1 "4.7k" V 2150 7050 50  0000 L CNN
F 2 "I20:R_0805" V 2130 6900 50  0001 C CNN
F 3 "" H 2200 6900 50  0000 C CNN
	1    2200 6900
	-1   0    0    1   
$EndComp
$Comp
L GNDD #PWR09
U 1 1 582F6F93
P 2100 7300
F 0 "#PWR09" H 2100 7050 50  0001 C CNN
F 1 "GNDD" V 2100 7050 50  0000 C CNN
F 2 "" H 2100 7300 50  0000 C CNN
F 3 "" H 2100 7300 50  0000 C CNN
	1    2100 7300
	1    0    0    -1  
$EndComp
$Comp
L VDD #PWR010
U 1 1 582F7FA8
P 2200 7300
F 0 "#PWR010" H 2200 7150 50  0001 C CNN
F 1 "VDD" V 2200 7500 50  0000 C CNN
F 2 "" H 2200 7300 50  0000 C CNN
F 3 "" H 2200 7300 50  0000 C CNN
	1    2200 7300
	-1   0    0    1   
$EndComp
$Comp
L VDD #PWR011
U 1 1 582F87A4
P 2000 7300
F 0 "#PWR011" H 2000 7150 50  0001 C CNN
F 1 "VDD" V 2000 7500 50  0000 C CNN
F 2 "" H 2000 7300 50  0000 C CNN
F 3 "" H 2000 7300 50  0000 C CNN
	1    2000 7300
	-1   0    0    1   
$EndComp
$Comp
L VDD #PWR012
U 1 1 582F88B7
P 1900 7300
F 0 "#PWR012" H 1900 7150 50  0001 C CNN
F 1 "VDD" V 1900 7500 50  0000 C CNN
F 2 "" H 1900 7300 50  0000 C CNN
F 3 "" H 1900 7300 50  0000 C CNN
	1    1900 7300
	-1   0    0    1   
$EndComp
$Comp
L GNDD #PWR013
U 1 1 582FC419
P 2200 5650
F 0 "#PWR013" H 2200 5400 50  0001 C CNN
F 1 "GNDD" V 2200 5400 50  0000 C CNN
F 2 "" H 2200 5650 50  0000 C CNN
F 3 "" H 2200 5650 50  0000 C CNN
	1    2200 5650
	-1   0    0    1   
$EndComp
$Comp
L GNDD #PWR014
U 1 1 582FC718
P 1950 5650
F 0 "#PWR014" H 1950 5400 50  0001 C CNN
F 1 "GNDD" V 1950 5400 50  0000 C CNN
F 2 "" H 1950 5650 50  0000 C CNN
F 3 "" H 1950 5650 50  0000 C CNN
	1    1950 5650
	-1   0    0    1   
$EndComp
$Comp
L R R17
U 1 1 5830E800
P 5100 3700
F 0 "R17" V 5200 3650 50  0000 L CNN
F 1 "0" V 5200 3750 50  0000 L CNN
F 2 "I20:R_0805" V 5030 3700 50  0001 C CNN
F 3 "" H 5100 3700 50  0000 C CNN
	1    5100 3700
	0    1    -1   0   
$EndComp
$Comp
L GNDD #PWR015
U 1 1 58310A6A
P 5400 3250
F 0 "#PWR015" H 5400 3000 50  0001 C CNN
F 1 "GNDD" H 5400 3100 50  0000 C CNN
F 2 "" H 5400 3250 50  0000 C CNN
F 3 "" H 5400 3250 50  0000 C CNN
	1    5400 3250
	-1   0    0    1   
$EndComp
$Comp
L GNDD #PWR016
U 1 1 58311616
P 4800 4150
F 0 "#PWR016" H 4800 3900 50  0001 C CNN
F 1 "GNDD" H 4800 4000 50  0000 C CNN
F 2 "" H 4800 4150 50  0000 C CNN
F 3 "" H 4800 4150 50  0000 C CNN
	1    4800 4150
	1    0    0    -1  
$EndComp
$Comp
L C_Small C5
U 1 1 58313AF8
P 5400 3450
F 0 "C5" H 5250 3550 50  0000 L CNN
F 1 "0" H 5450 3550 50  0000 L CNN
F 2 "I20:C_0805" H 5400 3450 50  0001 C CNN
F 3 "" H 5400 3450 50  0000 C CNN
	1    5400 3450
	1    0    0    -1  
$EndComp
$Comp
L VDD #PWR017
U 1 1 5831B08E
P 4800 1050
F 0 "#PWR017" H 4800 900 50  0001 C CNN
F 1 "VDD" H 4800 1200 50  0000 C CNN
F 2 "" H 4800 1050 50  0000 C CNN
F 3 "" H 4800 1050 50  0000 C CNN
	1    4800 1050
	1    0    0    -1  
$EndComp
$Comp
L R R14
U 1 1 5831B094
P 4800 1300
F 0 "R14" H 4870 1346 50  0000 L CNN
F 1 "10k" H 4870 1255 50  0000 L CNN
F 2 "I20:R_0805" V 4730 1300 50  0001 C CNN
F 3 "" H 4800 1300 50  0000 C CNN
	1    4800 1300
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X02 P4
U 1 1 5831B09A
P 5000 1750
F 0 "P4" H 5078 1791 50  0000 L CNN
F 1 "BTN_OPEN" H 4900 1600 50  0000 L CNN
F 2 "I20:JST_XH_B02B-XH-A_02x2.50mm_Straight" H 5000 1750 50  0001 C CNN
F 3 "" H 5000 1750 50  0000 C CNN
	1    5000 1750
	1    0    0    -1  
$EndComp
$Comp
L 74LVC2G14 U1
U 1 1 5831B0A0
P 5750 1500
F 0 "U1" H 5725 1767 50  0000 C CNN
F 1 "74LVC2G14" H 5725 1676 50  0000 C CNN
F 2 "I20:SOT-23-6" H 5900 1300 50  0001 C CNN
F 3 "" H 5750 1500 50  0000 C CNN
	1    5750 1500
	1    0    0    -1  
$EndComp
$Comp
L R R16
U 1 1 5831B0AC
P 5100 1500
F 0 "R16" V 5200 1450 50  0000 L CNN
F 1 "0" V 5200 1650 50  0000 L CNN
F 2 "I20:R_0805" V 5030 1500 50  0001 C CNN
F 3 "" H 5100 1500 50  0000 C CNN
	1    5100 1500
	0    1    -1   0   
$EndComp
$Comp
L GNDD #PWR018
U 1 1 5831B0B3
P 5400 1050
F 0 "#PWR018" H 5400 800 50  0001 C CNN
F 1 "GNDD" H 5400 900 50  0000 C CNN
F 2 "" H 5400 1050 50  0000 C CNN
F 3 "" H 5400 1050 50  0000 C CNN
	1    5400 1050
	-1   0    0    1   
$EndComp
$Comp
L GNDD #PWR019
U 1 1 5831B0B9
P 4800 1950
F 0 "#PWR019" H 4800 1700 50  0001 C CNN
F 1 "GNDD" H 4800 1800 50  0000 C CNN
F 2 "" H 4800 1950 50  0000 C CNN
F 3 "" H 4800 1950 50  0000 C CNN
	1    4800 1950
	1    0    0    -1  
$EndComp
$Comp
L C_Small C4
U 1 1 5831B0BF
P 5400 1250
F 0 "C4" H 5250 1350 50  0000 L CNN
F 1 "0" H 5450 1350 50  0000 L CNN
F 2 "I20:C_0805" H 5400 1250 50  0001 C CNN
F 3 "" H 5400 1250 50  0000 C CNN
	1    5400 1250
	1    0    0    -1  
$EndComp
Text HLabel 6000 1500 2    60   Input ~ 0
BTN_OPEN
$Comp
L VDD #PWR020
U 1 1 5835543A
P 5500 5950
F 0 "#PWR020" H 5500 5800 50  0001 C CNN
F 1 "VDD" V 5500 6150 50  0000 C CNN
F 2 "" H 5500 5950 50  0000 C CNN
F 3 "" H 5500 5950 50  0000 C CNN
	1    5500 5950
	0    1    1    0   
$EndComp
$Comp
L C C6
U 1 1 58355440
P 5450 5750
F 0 "C6" H 5500 5650 50  0000 C CNN
F 1 "10uF" H 5300 5650 50  0000 C CNN
F 2 "I20:C_0805" H 5488 5600 50  0001 C CNN
F 3 "" H 5450 5750 50  0000 C CNN
	1    5450 5750
	-1   0    0    1   
$EndComp
$Comp
L C C3
U 1 1 58355446
P 5200 5750
F 0 "C3" H 5250 5650 50  0000 C CNN
F 1 "100nF" V 5350 5450 50  0000 C CNN
F 2 "I20:C_0805" H 5238 5600 50  0001 C CNN
F 3 "" H 5200 5750 50  0000 C CNN
	1    5200 5750
	-1   0    0    1   
$EndComp
Text HLabel 5500 6550 2    60   Input ~ 0
PB13_SPI2_SCK
Text HLabel 5500 6350 2    60   Input ~ 0
PB14_SPI2_MISO
Text HLabel 5500 6450 2    60   Input ~ 0
PB15_SPI2_MOSI
Text HLabel 5500 6250 2    60   Input ~ 0
PB9_RC522B_IRQ
Text HLabel 5500 6050 2    60   Input ~ 0
PB7_RC522_RST
$Comp
L R R20
U 1 1 58355452
P 5350 6850
F 0 "R20" V 5350 6800 50  0000 L CNN
F 1 "4.7k" V 5300 7000 50  0000 L CNN
F 2 "I20:R_0805" V 5280 6850 50  0001 C CNN
F 3 "" H 5350 6850 50  0000 C CNN
	1    5350 6850
	-1   0    0    1   
$EndComp
$Comp
L R R19
U 1 1 58355458
P 5250 6850
F 0 "R19" V 5250 6800 50  0000 L CNN
F 1 "4.7k" V 5200 7000 50  0000 L CNN
F 2 "I20:R_0805" V 5180 6850 50  0001 C CNN
F 3 "" H 5250 6850 50  0000 C CNN
	1    5250 6850
	-1   0    0    1   
$EndComp
$Comp
L R R18
U 1 1 5835545E
P 5150 6850
F 0 "R18" V 5150 6800 50  0000 L CNN
F 1 "4.7k" V 5100 7000 50  0000 L CNN
F 2 "I20:R_0805" V 5080 6850 50  0001 C CNN
F 3 "" H 5150 6850 50  0000 C CNN
	1    5150 6850
	-1   0    0    1   
$EndComp
$Comp
L GNDD #PWR021
U 1 1 58355464
P 4900 6150
F 0 "#PWR021" H 4900 5900 50  0001 C CNN
F 1 "GNDD" V 4905 6022 50  0000 R CNN
F 2 "" H 4900 6150 50  0000 C CNN
F 3 "" H 4900 6150 50  0000 C CNN
	1    4900 6150
	0    -1   -1   0   
$EndComp
$Comp
L R R21
U 1 1 5835546A
P 5450 6850
F 0 "R21" V 5450 6800 50  0000 L CNN
F 1 "4.7k" V 5400 7000 50  0000 L CNN
F 2 "I20:R_0805" V 5380 6850 50  0001 C CNN
F 3 "" H 5450 6850 50  0000 C CNN
	1    5450 6850
	-1   0    0    1   
$EndComp
$Comp
L GNDD #PWR022
U 1 1 58355470
P 5350 7250
F 0 "#PWR022" H 5350 7000 50  0001 C CNN
F 1 "GNDD" V 5350 7000 50  0000 C CNN
F 2 "" H 5350 7250 50  0000 C CNN
F 3 "" H 5350 7250 50  0000 C CNN
	1    5350 7250
	1    0    0    -1  
$EndComp
$Comp
L VDD #PWR023
U 1 1 58355476
P 5450 7250
F 0 "#PWR023" H 5450 7100 50  0001 C CNN
F 1 "VDD" V 5450 7450 50  0000 C CNN
F 2 "" H 5450 7250 50  0000 C CNN
F 3 "" H 5450 7250 50  0000 C CNN
	1    5450 7250
	-1   0    0    1   
$EndComp
$Comp
L VDD #PWR024
U 1 1 5835547C
P 5250 7250
F 0 "#PWR024" H 5250 7100 50  0001 C CNN
F 1 "VDD" V 5250 7450 50  0000 C CNN
F 2 "" H 5250 7250 50  0000 C CNN
F 3 "" H 5250 7250 50  0000 C CNN
	1    5250 7250
	-1   0    0    1   
$EndComp
$Comp
L VDD #PWR025
U 1 1 58355482
P 5150 7250
F 0 "#PWR025" H 5150 7100 50  0001 C CNN
F 1 "VDD" V 5150 7450 50  0000 C CNN
F 2 "" H 5150 7250 50  0000 C CNN
F 3 "" H 5150 7250 50  0000 C CNN
	1    5150 7250
	-1   0    0    1   
$EndComp
$Comp
L GNDD #PWR026
U 1 1 58355488
P 5450 5600
F 0 "#PWR026" H 5450 5350 50  0001 C CNN
F 1 "GNDD" V 5450 5350 50  0000 C CNN
F 2 "" H 5450 5600 50  0000 C CNN
F 3 "" H 5450 5600 50  0000 C CNN
	1    5450 5600
	-1   0    0    1   
$EndComp
$Comp
L GNDD #PWR027
U 1 1 5835548E
P 5200 5600
F 0 "#PWR027" H 5200 5350 50  0001 C CNN
F 1 "GNDD" V 5200 5350 50  0000 C CNN
F 2 "" H 5200 5600 50  0000 C CNN
F 3 "" H 5200 5600 50  0000 C CNN
	1    5200 5600
	-1   0    0    1   
$EndComp
Wire Wire Line
	4800 3250 4800 3350
Wire Wire Line
	4800 4000 4800 4150
Connection ~ 4800 3700
Wire Wire Line
	4800 3650 4800 3900
Wire Wire Line
	8550 3150 8950 3150
Connection ~ 9150 4300
Wire Wire Line
	9150 4250 9150 4300
Wire Wire Line
	9500 4300 9500 4250
Connection ~ 8850 4300
Wire Wire Line
	8850 4300 9500 4300
Connection ~ 9150 3800
Wire Wire Line
	9150 3850 9150 3800
Wire Wire Line
	8850 4000 8850 4450
Wire Wire Line
	9150 3800 9550 3800
Wire Wire Line
	1450 2550 1450 2250
Connection ~ 2300 2450
Wire Wire Line
	1650 2450 2300 2450
Wire Wire Line
	1650 2250 1650 2450
Connection ~ 2500 2350
Wire Wire Line
	1850 2350 2500 2350
Wire Wire Line
	1850 2350 1850 2250
Wire Wire Line
	1850 1950 1850 1450
Wire Wire Line
	1650 1950 1650 1550
Wire Wire Line
	1450 1950 1450 1650
Wire Wire Line
	2800 2850 2400 2850
Wire Wire Line
	2800 3300 2600 3300
Wire Wire Line
	2800 3750 2800 3750
Wire Wire Line
	2100 4000 2100 3050
Connection ~ 2300 4000
Wire Wire Line
	2300 3500 2300 4100
Wire Wire Line
	2100 4000 2500 4000
Wire Wire Line
	2500 4000 2500 3950
Wire Wire Line
	2500 2250 2500 3550
Wire Wire Line
	2300 2250 2300 3100
Wire Wire Line
	2500 1950 2500 1900
Wire Wire Line
	2300 1900 2300 1950
Wire Wire Line
	2100 1950 2100 1900
Wire Wire Line
	2100 2250 2100 2650
Wire Wire Line
	2300 1300 2300 1300
Wire Wire Line
	9500 3950 9500 3800
Connection ~ 9500 3800
Wire Wire Line
	8500 5800 8450 5800
Wire Wire Line
	9850 3800 10000 3800
Wire Notes Line
	475  4925 6970 4925
Wire Wire Line
	9000 5900 9450 5900
Wire Wire Line
	1650 6600 2250 6600
Wire Wire Line
	2250 6500 1650 6500
Wire Wire Line
	1650 6400 2250 6400
Wire Wire Line
	1650 6300 2250 6300
Wire Wire Line
	1650 6100 2250 6100
Wire Wire Line
	1650 6700 2250 6700
Wire Wire Line
	2000 6750 2000 6400
Connection ~ 2000 6400
Wire Wire Line
	2200 6750 2200 6100
Connection ~ 2200 6100
Wire Wire Line
	2100 6750 2100 6300
Connection ~ 2100 6300
Wire Wire Line
	1900 6750 1900 6700
Connection ~ 1900 6700
Wire Wire Line
	1900 7300 1900 7050
Wire Wire Line
	2000 7300 2000 7050
Wire Wire Line
	2100 7300 2100 7050
Wire Wire Line
	2200 7300 2200 7050
Wire Wire Line
	1650 6000 2250 6000
Wire Wire Line
	2200 6000 2200 5950
Connection ~ 2200 6000
Wire Wire Line
	1950 5950 1950 6000
Connection ~ 1950 6000
Wire Wire Line
	8650 3200 8650 3150
Connection ~ 8650 3150
Wire Wire Line
	8950 3250 8850 3250
Wire Wire Line
	8850 3250 8850 3600
Wire Wire Line
	8650 3500 9000 3500
Connection ~ 8850 3500
Wire Wire Line
	8550 3050 8950 3050
Wire Wire Line
	8550 2950 8950 2950
Wire Wire Line
	4800 3700 4950 3700
Wire Wire Line
	5250 3700 5450 3700
Wire Wire Line
	5400 3550 5400 3700
Connection ~ 5400 3700
Wire Wire Line
	5400 3250 5400 3350
Wire Wire Line
	4800 1050 4800 1150
Wire Wire Line
	4800 1800 4800 1950
Connection ~ 4800 1500
Wire Wire Line
	4800 1450 4800 1700
Wire Wire Line
	4800 1500 4950 1500
Wire Wire Line
	5250 1500 5450 1500
Wire Wire Line
	5400 1350 5400 1500
Connection ~ 5400 1500
Wire Wire Line
	5400 1050 5400 1150
Wire Wire Line
	1450 1650 950  1650
Wire Wire Line
	1650 1550 950  1550
Wire Wire Line
	1850 1450 950  1450
Wire Wire Line
	1150 1750 950  1750
Wire Wire Line
	1450 2550 2100 2550
Connection ~ 2100 2550
Wire Notes Line
	3150 7755 3150 7795
Wire Notes Line
	3150 5390 3150 5375
Wire Notes Line
	6970 6530 6970 475 
Wire Notes Line
	6970 2500 11220 2500
Wire Wire Line
	4900 6550 5500 6550
Wire Wire Line
	5500 6450 4900 6450
Wire Wire Line
	4900 6350 5500 6350
Wire Wire Line
	4900 6250 5500 6250
Wire Wire Line
	4900 6050 5500 6050
Wire Wire Line
	4900 6650 5500 6650
Wire Wire Line
	5250 6700 5250 6350
Connection ~ 5250 6350
Wire Wire Line
	5450 6700 5450 6050
Connection ~ 5450 6050
Wire Wire Line
	5350 6700 5350 6250
Connection ~ 5350 6250
Wire Wire Line
	5150 6700 5150 6650
Connection ~ 5150 6650
Wire Wire Line
	5150 7250 5150 7000
Wire Wire Line
	5250 7250 5250 7000
Wire Wire Line
	5350 7250 5350 7000
Wire Wire Line
	5450 7250 5450 7000
Wire Wire Line
	4900 5950 5500 5950
Wire Wire Line
	5450 5950 5450 5900
Connection ~ 5450 5950
Wire Wire Line
	5200 5900 5200 5950
Connection ~ 5200 5950
Wire Notes Line
	6978 4925 11219 4925
Wire Notes Line
	3750 494  3750 7794
Wire Notes Line
	3750 484  3750 473 
Wire Notes Line
	6969 2500 3751 2500
Text Notes 6600 5100 0    60   ~ 0
RCCB
Text Notes 3400 5100 0    60   ~ 0
RCCA
$Comp
L GNDD #PWR028
U 1 1 583814D2
P 9350 5450
F 0 "#PWR028" H 9350 5200 50  0001 C CNN
F 1 "GNDD" H 9350 5300 50  0000 C CNN
F 2 "" H 9350 5450 50  0000 C CNN
F 3 "" H 9350 5450 50  0000 C CNN
	1    9350 5450
	-1   0    0    1   
$EndComp
$Comp
L GNDD #PWR029
U 1 1 58381774
P 8450 6100
F 0 "#PWR029" H 8450 5850 50  0001 C CNN
F 1 "GNDD" V 8450 5850 50  0000 C CNN
F 2 "" H 8450 6100 50  0000 C CNN
F 3 "" H 8450 6100 50  0000 C CNN
	1    8450 6100
	0    1    1    0   
$EndComp
Wire Wire Line
	9000 6100 9450 6100
Wire Wire Line
	8450 6100 8500 6100
Wire Wire Line
	9000 5800 9450 5800
Connection ~ 9100 5800
$Comp
L GNDD #PWR030
U 1 1 583AE5EE
P 9100 5450
F 0 "#PWR030" H 9100 5200 50  0001 C CNN
F 1 "GNDD" H 9100 5300 50  0000 C CNN
F 2 "" H 9100 5450 50  0000 C CNN
F 3 "" H 9100 5450 50  0000 C CNN
	1    9100 5450
	-1   0    0    1   
$EndComp
Wire Wire Line
	9100 5800 9100 5750
Connection ~ 9350 5800
Wire Wire Line
	9350 5750 9350 5800
$Comp
L VDD #PWR031
U 1 1 583B6020
P 9450 6000
F 0 "#PWR031" H 9450 5850 50  0001 C CNN
F 1 "VDD" V 9450 6200 50  0000 C CNN
F 2 "" H 9450 6000 50  0000 C CNN
F 3 "" H 9450 6000 50  0000 C CNN
	1    9450 6000
	0    1    1    0   
$EndComp
Text HLabel 5500 6650 2    60   Input ~ 0
PB6_RC522B_CS
Text GLabel 9000 3500 2    60   Input ~ 0
LOCK_GND
$Comp
L CONN_01X10 P3
U 1 1 583D3498
P 4700 6200
F 0 "P3" H 4700 5500 50  0000 C CNN
F 1 "RC522B" H 4700 5600 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_2x05" H 4700 6200 50  0001 C CNN
F 3 "" H 4700 6200 50  0000 C CNN
	1    4700 6200
	-1   0    0    1   
$EndComp
NoConn ~ 4900 5850
NoConn ~ 4900 5750
Wire Wire Line
	9000 6000 9100 6000
Wire Wire Line
	9400 6000 9450 6000
Text HLabel 9450 6100 2    60   Input ~ 0
UART_INT_RX
$EndSCHEMATC
