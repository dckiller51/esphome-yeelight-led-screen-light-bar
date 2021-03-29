# version beta

# esphome-yeelight-led-screen-light-bar

Esphome custom firmware for some Yeelight Screen Lights YLTD001.

- Todo list:
1) Fix the rgb flickering problem
2) Implement 2.4GHz remote control to control (http://wiki.telink-semi.cn/wiki/chip-series/TLSR836x-Series/ )


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

<a href="https://github.com/dckiller51/esphome-yeelight-led-screen-light-bar/blob/main/images/01.jpg" target="_blank">
<img src="https://github.com/dckiller51/esphome-yeelight-led-screen-light-bar/blob/main/images/thumbnails/01.jpg" width="18%">
</a>

<a href="https://github.com/dckiller51/esphome-yeelight-led-screen-light-bar/blob/main/images/02.jpg" target="_blank">
<img src="https://github.com/dckiller51/esphome-yeelight-led-screen-light-bar/blob/main/images/thumbnails/02.jpg" width="18%">
</a>

<a href="https://github.com/dckiller51/esphome-yeelight-led-screen-light-bar/blob/main/images/03.jpg" target="_blank">
<img src="https://github.com/dckiller51/esphome-yeelight-led-screen-light-bar/blob/main/images/thumbnails/03.jpg" width="18%">
</a>

<a href="https://github.com/dckiller51/esphome-yeelight-led-screen-light-bar/blob/main/images/04.jpg" target="_blank">
<img src="https://github.com/dckiller51/esphome-yeelight-led-screen-light-bar/blob/main/images/thumbnails/04.jpg" width="18%">
</a>

<a href="https://github.com/dckiller51/esphome-yeelight-led-screen-light-bar/blob/main/images/05.jpg" target="_blank">
<img src="https://github.com/dckiller51/esphome-yeelight-led-screen-light-bar/blob/main/images/thumbnails/05.jpg" width="18%">
</a>

<a href="https://github.com/dckiller51/esphome-yeelight-led-screen-light-bar/blob/main/images/06.jpg" target="_blank">
<img src="https://github.com/dckiller51/esphome-yeelight-led-screen-light-bar/blob/main/images/thumbnails/06.jpg" width="18%">
</a>

<a href="https://github.com/dckiller51/esphome-yeelight-led-screen-light-bar/blob/main/images/connexion.jpg" target="_blank">
<img src="https://github.com/dckiller51/esphome-yeelight-led-screen-light-bar/blob/main/images/thumbnails/connexion.jpg" width="18%">
</a>

<a href="https://github.com/dckiller51/esphome-yeelight-led-screen-light-bar/blob/main/images/esphome01.jpg" target="_blank">
<img src="https://github.com/dckiller51/esphome-yeelight-led-screen-light-bar/blob/main/images/thumbnails/esphome01.jpg" width="18%">
</a>

<a href="https://github.com/dckiller51/esphome-yeelight-led-screen-light-bar/blob/main/images/esphome02.jpg" target="_blank">
<img src="https://github.com/dckiller51/esphome-yeelight-led-screen-light-bar/blob/main/images/thumbnails/esphome02.jpg" width="18%">
</a>