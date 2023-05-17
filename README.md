# Quadx-Mega: Quadricoptero com Arduino (Quadcopter with Arduino)
QuadX-Mega é um quadricoptero desenvolvido com um microcontrolador Arduino Mega Pro mini. O objetivo desse repositório é apresentar de forma simples e direta o hardware e software necessários para a construção de um drone caseiro. A descrição desse projeto tem como base o trabalho de Joop Brokking e totalmente sem fins lucrativos.

Esse projeto destinado a pessoas que gostam de programação, eletrônica, drones, hobbistas e entusiasta de aeromodelos e aviação. Em caso de dúvidas podem entrar em contato direto pelo GitHub ou através do e-mail: robertribeiro100@gmail.com. 

Site do Joop Brokking: http://www.brokking.net/.

| ![image](https://github.com/ribeirorobert/quadx-mega/blob/main/Images/IMG_4583.jpg) | 
|:--:| 
| **Fig. 1** *QuadX-Mega* |

<!-- ![image](https://github.com/ribeirorobert/quadx-mega/edit/feature/altitude-hold/Images/IMG_4565.jpg) -->

# Hardware
A seguir é listado os componentes e peças utilizadas para a construção do quadricoptero. Entretanto, fica a critério e responsabilidade de quem está construindo escolher os itens que irão compor o projeto. Ainda, reforço que nem todos esses componentes são os melhores e mais indicados do mercado, não sendo objetivo construir um drone com melhor preço, com voos a longas distâncias tão pouco a análise da capacidade de carga e duração de voo.

* 1x Frame F450.
* 4x Motor Brushless ReadyToSky 2212 920KV.
* 4x ESC de 30A SimonK.
* 4x Hélice 10x4.5 (2cw e 2 ccw).
* 1x Bateria de Lipo 3S ou 4S com capacidade maior ou igual a 3000mAh. 
* 1x Carregador de bateria Lipo.
* 1x Indicador e alarme de bateria (opcional)
* 1x Radio controle com no mínimo 6 canais. 
* 1x Arduino Mega Pro mini.
* 1x Módulo MPU6050 -> Acelerômetro e Giroscópio 
* 1x Módulo HMC5883L -> Magnetômetro (opcional)
* 1x Módulo MS5611 -> Sensor de pressão barométrica (opcional)
* 1x Módulo GPS NEO 6MV2 (opcional)
* 1x Módulo de comunicação RF Xbee, Lora, NRF24L01 ou APC220 (opcional)
* 1x Buzzer 5V
* 1x LED vermelho
* 1x LED verde
* 1x LED amarelo
* 1x Balanceador de hélice
* 1x Cinta para fixar bateria (opcional)

Observações: 
1. Os motores utilizados possuem furação de fixação em conformidade com o frame. Se optar por utilizar outro conjunto, verifique se a fixação dos motores é coerente com a do frame. 
2. KV: constante de velocidade (para cada 1V de tensão de alimentação há um incremento teórico da velocidade de 920 rpm).
3. Os ESC's utilizados suportam baterias de 2S a 4S.
4. A escolha da hélice tem relação com o tipo de drone que se deseja construir. Algumas são para aplicações onde a prioridade é velocidade. Por outro lado há hélices que são para aplicações onde o peso é o mais importante. Hélices do modelo 10x4.5 são muito comuns e estão no meio termo entre velocidade-peso.
5. O indicador de alarme da bateria é muito útil principalmente para quem está aprendendo a controlar drones/aeromodelos e aqueles que ainda não possuem telemetria integrado. É emitido um alarme sonoro quando a tensão da bateria chega a tensão de limiar pré-definida. O QuadX-Mega possui uma lógica para emitir alarme quando a tensão da bateria atingir 10.7V. Em geral, é definido uma tensão de descarga mínima para cada célula da bateria para garantir sua vida útil. Descargas profundas reduzem a vida útil da bateria. Para o QuadX-Mega a tensão de descarga mínima por célula é de 3,5V com acréscimo de 0,2V como margem de segurança. Em situações em que o quadricoptero esteja a uma distância considerável do operador é aconselhável ter um pouco mais de energia para ganrantir o retorno de forma segura.
6. Em geral um rádio de apenas 4 canais (Roll-CH1, Pitch-CH2, Yaw-CH4 e Throttle-CH3) é suficiente para controlar um drone simples. Entretando, quanto maior o número de canais mais configurações e comandos podem ser adicionados.
7. A escolha do Arduino Mega Pro mini é em função da quantidade de I/Os e tamanho. Outros microcontroladores como Arduino UNO, Arduino Nano, ESP32 ou STM32F103 poderiam ser utilizados tomando os devidos cuidados e modificações pertinentes a cada sistema.
8. O módulo MPU6050 foi utilizado devido a sua qualidade. Existem diversos modelos de acelerômetros disponível no mercado e alguns com mais graus de liberdade (DOFs) como de exemplo o MPU9250 que é um módulo com 9DOFs (acelerômetro, giroscópio e bússola).
9. A PCI (Placa de Circuito Impresso) projetada permite a utilização de diversos sensores e módulos. São eles: MPU6050, MPU9250, BMP180 ou BMP085, HMC5883L, MS5611, GPS NEO-6MV2 e módulos para telemetria via comunicação serial.

| ![image](https://github.com/ribeirorobert/quadx-mega/blob/main/Images/IMG_4565.jpg)   | 
|:--:| 
| **Fig. 2** *Eletrônica do quadricoptero* |

| ![image](https://github.com/ribeirorobert/quadx-mega/blob/main/Images/IMG_4566.jpg)   | 
|:--:| 
| **Fig. 3** *Eletrônica do quadricoptero* |

O esquemático, lista de componentes e o arquivo Gerber da PCI estão na [pasta PCI](https://github.com/ribeirorobert/quadx-mega/tree/main/PCI) do repositório. 

# Arquivos .STL (STL Files)
Os arquivos .STL utilizados no projeto podem ser acessados diretamente na pasta [STL_files](https://github.com/ribeirorobert/quadx-mega/tree/main/STL_files) do repositório. São eles:
* Skid do trem de pouso.
* Suporte para bateria.
* Case principal.

Observações: 
Foram utilizados peças de fabricação em impressora 3D, mas a confecção desses itens não é obrigatória. Assim, fica a critério do hobbista posicionar certos itens. 

1. Na maioria dos casos o frame do F450 acompanha o skid do trem de pouso principal. Porém, o drone fica um pouco alto (na minha opnião) e em muitas situações fica instável no pouso e decolagem. E como resultado muitas hélices acabam quebrando nessas situações. 
2. A cinta para fixar a bateria é uma opção alternativa. 
3. A case principal foi projetada com objetivo de ser a menor e mais leve possível garantindo a proteção dos componentes da PCI.  

# Software
O código foi desenvolvido em C++ no VScode. Para realizar o download click no botão "<>Code" em verde -> Download ZIP. Ou se preferir basta usar o comando de clone através do git: git clone https://github.com/ribeirorobert/quadx-mega.git que será criado uma cópia do repositório.

O código tem tamanho relativamente grande. Alguns rotinas e funções importantes serão descritas a seguir.

### Defines
Os defines são utilizados para deixar o código mais limpo e organizado além de possibilidar uma maior dinâmica do que está ou não habilitado (flags). Algumas constantes foram definidas com base no datasheet e dados técnicos dos sensores, por outro lado algumas contantes foram obtidas em referência ao trabalho de Joop Brokking ou de forma empírica. Ao mudar alguma constante ou flag tenha certeza do que está fazendo.

As flags a seguir têm função de: habilitar a função de telemetria (se utilizada), habilitar a portal serial do Arduino (para debug), habilitar a rotina com diversas telas de debug, habilitar funções do sensor BMP180 ou MS5611, habilitar a rotina de Fail Safe se implementada, respectivamente. 

```
#define TELEMETRY_ENABLE    0
#define SERIAL_ENABLE       0
#define PRINTF_ENABLE       0
#define BMP180_ENABLE       0
#define MS5611_ENABLE       1
#define FAIL_SAFE           0
```

Já as flags a seguir tem função de habilitar o debug de verificação dos sensores na inicialização do quadricoptero, debug da EEPROM, debug da calibração da IMU, debug da calibração dos ESC's.

```
#define DEBUG_STATUS        0
#define DEBUG_EEPROM        0
#define DEBUG_MPU           0
#define DEBUG_ESC           0
```

### Variáveis
As variáveis em sua maioria são para armazenar os dados dos sensores, controle PID dos ângulo de rolagem (Roll), arfagem (Pitch) e guinada (Yaw), tensão da bateria e status do quadricoptero de modo geral.

### Setup
O setup é função responsável por definir quais pinos do microcontrolador serão definidos como entrada ou saída, inicializar as portas de comunicação Serial e I2C, configurar os pinos de PWM (Pulse Width Modulation) dos ESC's, verificar se os sensores habilitados estão respondendo corretamente e atualizar os timers utilizados no código.

### Loop
É a função principal do código e todas as demais rotinas serão chaveadas de maneira sequencial. É importante destacar que o loop "roda" a uma taxa de 250 Hz (4ms). Esse valor foi definido em conformidade com a frequência de operação dos ESC's. Assim, todas as demais rotinas de controle e leitura dos sensores estão sendo executadas nessa velocidade. Em alguns casos expecíficos como de exemplo da aquisição dos dados do sensor de pressão barométrica foram criados artifícios para escalonar essa taxa. As funções executadas dentro do loop são descritas a seguir.

1. acc_gyro_values(): Essa rotina é responsável por aquisitar os bytes do registradores do acelerômetro e giroscópio do sensor MPU6050 e ainda subtrair dos valores de calibração.

2. euler_angles(): É responsável por filtrar os dados obtidos na função anterior e computar os ângulos de Roll e Pitch através das Equações de Euler. 

Para mais informações visite: 
- https://www.youtube.com/watch?v=pQ24NtnaLl8 
- https://en.wikipedia.org/wiki/Euler_angles
- https://www.youtube.com/watch?v=5IkPWZjUQlw&t=17s
- https://www.youtube.com/watch?v=nCPEJTUYch8&t=616s
- https://www.youtube.com/watch?v=j-kE0AMEWy4&t=9s

3. read_radio_commands(): Essa função é responsável por interpretar alguns comandos combinados do rádio.
* Ligar/Desligar os motores

O status dos motores é divido em três categorias, são elas: LOCK, IDLE e FULL. No status de LOCK os motores permanecem desligados. Já no status de IDLE os motores são acionados em rotação mínima e por fim em FULL os motores podem assumir rotação entre minimo e máximo de acordo com a posição do stick de throttle do radio controle. O objetivo dessa implementação é evitar de modo não intencional os motores sejam acionados. Lembrando que motores brushless operam em rotações elevadas e com a hélice conectada tornam-se perigosos.

| ![image](https://github.com/ribeirorobert/quadx-mega/blob/main/Images/IMG_4580_1.jpg)   | 
|:--:| 
| **Fig. 4** *Comando para ligar os motores* |

Seguindo a indicação das setas mostrado na Figura acima, uma primeira tentativa o status dos motores em LOCK muda para IDLE e um segunda tentativa os motores mudam para o status de FULL, ou seja, aceleração desbroqueada. Para desligar basta seguir a instrução da Figura 5.

| ![image](https://github.com/ribeirorobert/quadx-mega/blob/main/Images/IMG_4580_2.jpg)   | 
|:--:| 
| **Fig. 5** *Comando para desligar os motores* |

* Calibrar IMU -> Giroscópio

Da mesma forma para calibrar o giroscópio é preciso executar a instrução indicada em vermelho duas vezes. Caso queira cancelar a calibração, basta executar a indicação em azul (Figura 10).

| ![image](https://github.com/ribeirorobert/quadx-mega/blob/main/Images/IMG_4580_3.jpg)   | 
|:--:| 
| **Fig. 6** *Comando calibrar o giroscópio* |

* Calibrar IMU -> Acelerômetro

Para calibrar o acelerômetro é preciso executar a instrução indicada em vermelho duas vezes. Caso queira cancelar a calibração, basta executar a indicação em azul (Figura 10).

| ![image](https://github.com/ribeirorobert/quadx-mega/blob/main/Images/IMG_4580_4.jpg)   | 
|:--:| 
| **Fig. 8** *Comando para calibrar o acelerômetro* |

* Calibrar os ESC's

Para calibrar os ESC's é preciso executar a instrução indicada em vermelho duas vezes (Figura 9). Caso queira cancelar a calibração, basta executar a indicação em azul (Figura 10). Porém, a calibração dos ESC's possui um terceira etapa da seguinte forma: 

- Desconecte a bateria e mova o stick do throttle para a posição máxima. 
- Conecte a bateria novamente e aguarde o beep dos motores.
- Mova o stick do throttle para a posição mínima e aguarde o beep dos motores novamente. 
    
Tudo executado corretamente basta realizar o comando para sair da calibração (Figura 10). Se esse comando não for executado ao fim da calibração, toda vez que o drone for ligado a rotina de calibração dos ESC's será executada primeiro.

| ![image](https://github.com/ribeirorobert/quadx-mega/blob/main/Images/IMG_4580_5.jpg)   | 
|:--:| 
| **Fig. 9** *Comando para calibrar os ESC's* |

| ![image](https://github.com/ribeirorobert/quadx-mega/blob/main/Images/IMG_4580_6.jpg)   | 
|:--:| 
| **Fig. 10** *Comando para cancelar* |

* Habilitar Altitude Hold (em desenvolvimento)

Quando habilitada a rotina de Altitude Hold possibilita com o que o quadricoptero permanace em uma altitude fixa usando como referência a pressão barométrica. Ao mover o stick de throttle essa referência de altitude é atualizada até que o stick retorne a posição central. 

Para mais informações da implementação visite: 
- https://www.youtube.com/watch?v=2BLb6qUKikI&t=702s

| ![image](https://github.com/ribeirorobert/quadx-mega/blob/main/Images/IMG_4580_11.jpg)   | 
|:--:| 
| **Fig. 11** *Hibilitar o Altitude Hold* |

* Ligar/Desligar LEDs do frame

| ![image](https://github.com/ribeirorobert/quadx-mega/blob/main/Images/IMG_4580_12.jpg)   | 
|:--:| 
| **Fig. 12** *Acionar os LEDs do frame* |

Observações: Algumas rotinas possuem um timeout, ou seja, ao selecionar uma calibração mas não confirmar, após um intervalo de tempo de 2 minutos a instrução será cancelada. Ainda, para habilitar a calibração do acelerômetro, giroscópio e ESC's o status dos motores deve necessarimente ser LOCK, ou seja, só é possível acionar essas funções com os motores desligados. Há um display de 7 segmentos para auxiliar de maneira visual com as instruções. Em caso de erro de calibração uma mensagem será exibida na tela e bloqueado para voo. Nessa situação desconecte a bateria e realize o procedimento novamente. 

IMPORTANTE: sempre que for calibrar a IMU (Inertial Measurement Unit) certifique-se que o quadricoptero está sobre uma superfície plana, do contrário, o mesmo pode voar com um pouco de inclinação ou apresentar a mensagem de erro na etapa de calibração. Os valores de calibração da IMU são armazenados na EEPROM. Se o quadricoptero não sofrer queda ou impacto não há necessidade de qualibrar a IMU e os ESC's sempre que for ligado.

4. execute_pid_controllers(): Essa função é responsável por executar o controle dos três ângulos (Roll, Pitch e Yaw). Foi utilizado o famoso controlador PID (Proporcional-Integral-Derivativo), porém não será o foco aqui descrever como esse controlador funciona e como projetar os ganhos Kp, Ki e Kd, respectivamente. Cada ângulo possui uma referência que são as respectivas posições dos stick de cada movimento. A referência dos stick são comparadas com os Ângulos de Euler calculados para gerar o sinal de erro, esse por último sendo o objeto de rastreamento e minimização do algoritimo PID. Foi implementando uma rotina para criar uma zona morta (dead band) do sinal de referência dos sticks com intuito de manter nulo o sinal de referência para oscilações mínimas do seu movimento. Essa função "retorna" três sinais de controle que serão utilizados para aumentar/reduzir a rotação dos motores de modo a transformar essas rotações em uma postura/movimento (Attitude) do quadricoptero. 

5. read_battery_voltage(): Essa função é responsável pela leitura da tensão da bateria e filtragem.
6. show_quad_status(): Essa rotina é responsável por mostrar as mensagem e valor da tensão na bateria no display de 7 segmentos.
7. check_alarms(): Essa função é encarregada de acionar os LED's de notificação, buzzer para nível crítico de bateria e LED's do frame.

8. esc_pwm_signals(): Função responsável por atribuir as respectivas velocidades de rotação a cada motor de acordo com a configuração de "X" do quadricoptero. Existem diversas configurações para distribuir o comando de throttle e sinal de controle PID(Roll, Pitch, Yaw) para os motores, cada um com particularidades, vantagens e desvantagens. Cada aplicação terá um ou mais conjunto de configurações recomendáveis. Nesse projeto a distribuição utilizada é dada por:

```
esc1 = throttle - pidPitch + pidRoll - pidYaw); //PWM for ESC 1, front-right - CCW
esc2 = throttle + pidPitch + pidRoll + pidYaw); //PWM for ESC 2, rear-right  - CW
esc3 = throttle + pidPitch - pidRoll - pidYaw); //PWM for ESC 3, rear-left   - CCW
esc4 = throttle - pidPitch - pidRoll + pidYaw); //PWM for ESC 4, front-left  - CW
 ```

9. print_info(): Rotina exclusiva para debug de variáveis. 

IMPORTANTE: manter a flag PRINTF_ENABLE 0 quando for voar com o quadricoptero.

10. ISR(PCINT0_vect): Essa rotina não é executada no Loop, mas é muito importante pois é responsável pela leitura dos sinais PPM (Pulse Position Modulation) do rádio controle. A vantagem em utilizar o modo PPM é a redução da quantidade de fios necessários para aquisitar os pulsos de cada canal. Alguns rádios possuem apenas a opção de PWM, assim, cada canal fornece de forma independente um pulso com largura de 1000-2000us (4º link). O rádio utilizado no projeto é o FlySky FS-I6X, com 6 canais por padrão mas pode ser atualizado para trabalhar com 10 canais juntamente com um recepetor para 10 canais. No modo PPM é possível utilizar 8 canais com o receptor original.

Para mais informações visite: 
- https://www.youtube.com/watch?v=lxE4K7ghST0&list=PLDnffNsiQx6PM4rRNhtB-s_rc2s0CCEmk
- https://www.youtube.com/watch?v=63JmO4Mc8NM
- https://www.youtube.com/watch?v=IsxJD7lS0Go
- http://electronoobs.com/eng_robotica_tut9_1.php

# Montagem (The build)
Para mais detalhes da montagem acesse a pasta de [Imagens](https://github.com/ribeirorobert/quadx-mega/tree/main/Images) no repositório.

# Trabalhos Futuros (Future Works)
* Altitude Hold
* GPS Hold and Route Track
* Back-to-Home
* Telemetry

# Referências (References)
[1] Brokking.net. Link: http://www.brokking.net/

[2] Youtube Joop Brokking. Link: https://www.youtube.com/@Joop_Brokking
