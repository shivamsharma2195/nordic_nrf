# nordic_nrf
Codes of nordic nrf controllers


"e" in last of pca code (i.e. pca10056e) refers to extended means code can be used for subchips of nrf of same group (i.e. nrf52840 family)


nrf52832 - pca10040
nrf52810 - pca10040e
nrf52840 - pca10056


twi_pin_change_nrf52810 ---- shift file in sdk inside example->peripherals --- nrf52810 --- change twi pins
ble_app_uart optimization_adc_2k --- shift file in example->ble_peripherals --- nrf52840 --- uart in ble peripheral with adc mcp3912
ble_app_uart optimization_auto_off_disable --- shift file in example->ble_peripherals --- nrf52840 --- auto off of device after 5 mins of search by ble is disabled
ble_app_uart optimization  --- shift file in example->ble_peripherals --- nrf52840 --- uart in ble peripherals

we used nrf nordic sdk17.0.2 in segger embedded studio for these codes
