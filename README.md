# TCC - Nome TBD

Neste repositorio estarei adionando os codigos relacionado ao meu tcc.

### Setup (SERÁ APAGADO POSTERIORMENTE)
Setup comando usados (usando Windows):

Seguindo tutorial do BLINK e seu README
(Abrir terminal ESP-32 da extensão do VsCode)
PS D:\Documentos\Tcc\blink> idf.py set-target esp32h2

PS D:\Documentos\Tcc\blink> idf.py menuconfig
Por algum motivo J e K é o equivalente a seta up e seta down para mecher no menu

PS D:\Documentos\Tcc\blink> idf.py -p PORT flash monitor

## Descrição

### O que é ZigBee?

ZigBee é um protocolo sem fio eficiente em termos de energia, desenvolvido especificamente para dispositivos de casas inteligentes e IoT (Internet das Coisas). O protocolo ZigBee é amplamente considerado uma alternativa ao Wi-Fi e ao Bluetooth no setor de IoT — especialmente para dispositivos de baixo consumo de energia que não exigem grande largura de banda. Com o ZigBee, você pode conectar dispositivos inteligentes como lâmpadas, tomadas, fechaduras, termostatos de radiador e interruptores na sua rede doméstica inteligente.

### Por que ZigBee?

ZigBee é a chave para a comunicação independente de fabricantes no ambiente de casa inteligente. Com o ZigBee, é possível usar dispositivos de diversas marcas em uma rede compartilhada. Isso oferece máxima flexibilidade e segurança para o futuro ao escolher seus dispositivos inteligentes. Graças ao alto nível de interoperabilidade, dispositivos de diferentes fabricantes também conseguem se comunicar entre si.

### Como o ZigBee funciona?

Os dispositivos ZigBee funcionam com base em um hub central ZigBee (gateway), que é conectado à internet e controla todos os dispositivos. Os próprios dispositivos se comunicam por meio de uma chamada rede em malha (mesh). Isso significa que cada dispositivo não apenas recebe sinais, mas também os repassa para outros dispositivos. Isso cria uma rede estável e de longo alcance.

Além disso, o coordenador — ou seja, seu aplicativo ZigBee — está conectado ao gateway. Isso permite que você controle seus dispositivos inteligentes de praticamente qualquer lugar.

### ESP32 e ZigBee

O ESP32 é um microcontrolador versátil que pode suportar ZigBee por meio do ESP Zigbee SDK, um framework de desenvolvimento fornecido pela Espressif. Esse SDK permite que os desenvolvedores criem produtos compatíveis com ZigBee com facilidade, oferecendo APIs simplificadas, utilitários e documentação. Alguns modelos do ESP32, como o ESP32-C6, também possuem capacidades de rádio ZigBee integradas.

### Iniciando ZigbeeMQTT

```
# Após ter o repositório instalado, abra o powershell e redirecione para a pasta do zigbeemqtt
D:\Documentos\Tcc\zigbee2mqtt> pnpm start

# Se tudo der certo esse será o output:
[2025-04-20 23:11:58] info:     z2m: Started frontend on port 8080
[2025-04-20 23:11:58] info:     z2m: Zigbee2MQTT started!       
```

## Referências:

https://www.watt24.com/en/guide/everything-you-need-to-know-about-zigbee/

https://www.vesternet.com/en-global/pages/what-is-zigbee

https://docs.espressif.com/projects/esp-zigbee-sdk/en/latest/esp32/introduction.html#esp-zigbee-sdk

https://docs.espressif.com/projects/esp-idf/en/stable/esp32h2/get-started/index.html

https://www.zigbee2mqtt.io/guide/installation/05_windows.html

https://cedalo.com/blog/how-to-install-mosquitto-mqtt-broker-on-windows/

https://www.zigbee2mqtt.io/guide/installation/20_zigbee2mqtt-fails-to-start_crashes-runtime.html

https://github.com/espressif/esp-zigbee-sdk/tree/main
