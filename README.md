# esphome-yeelight-led-screen-light-bar

Esphome custom firmware for some Yeelight Screen Lights YLTD001.


## Supported devices

| Name                                             | Model                     | Model no.   | Specs                                                            |
| ------------------------------------------------ | ------------------------- | ----------- | ---------------------------------------------------------------- |
| Yeelight LED Screen Light Bar YLTD001            | yeelink.light.UNKNOWN     | YLTD001     | DC 5V, 10W (80x0.2W/Led module+40x0.2W/Led module), 2700K-6500K  |



## Features

### yeelink.light.UNKNOWN

- Light (CCWW)
  - Brightness
  - Color temperature (2700K-6500K)
- Ambient light  (RGB)
  - Brightness
  - Color

## GPIOs

### yeelink.light.UNKNOWN

| Name                | Label  | ESP8266 GPIO |
| ------------------- | ------ | ------------ |
| Warm white PWM      | W      | GPIO5        |
| Cold white PWM      | C      | GPIO12       |
| RGB PWM             | RGB    | GPIO13       |
| Power supply GPIO   |        | GPIO4        |
| SDA                 |        | GPIO(2?)     |
| SCL                 |        | GPIO(14?)    |


```
                    ESP8266NV1.0

?     - o                              o   - ?


?     - o                              o   - ?
?     - o                              o   - TX
GPIO0 - o                              o   - RX
?     - o                              o   - ? 

          o   o   o   o   o   o   o   o
          |   |   |   |   |   |   |   |
         GND VCC SLC SDA RGB  C  EN   W  
```


## Disassembly

### YLTD001

<a href="" target="_blank">
<img src="" width="18%">
</a>