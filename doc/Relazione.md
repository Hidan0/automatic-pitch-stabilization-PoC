# Relazione

## Modulo 'servo'

Nel seguente capitolo verranno illustrate le scelte implementative per la realizzazione
del modulo che implementa un semplice driver per il motore servo SG90.

Il motore servo SG90[^1] è un motore standard leggero e di piccole dimensioni. Il servo
può ruotare di circa 180 gradi (90 gradi per ogni direzione) e utilizza segnali di
modulazione di larghezza di impulso (PWM) per determinare la posizione dell'albero. Un
impulso di $1.5ms$ posiziona l'albero al centro, un impulso di $1ms$ lo posiziona
all'estrema sinistra (-90 gradi), mentre un impulso di $2ms$ lo posiziona all'estrema
destra (90 gradi).

### Generare una PWM con l'ESP32-C3

Per generare il segnale di larghezza di impulso con l'ESP32-C3 è stata utilizzata la
periferica LEDC[^2] (_LED control_). Questa periferica è progettata principalmente per
controllare l'intensità dei LED, ma può anche essere configurata per altri scopi, come
ad esempio la generazione di una PWM.

Il modulo LEDC consente di generare fino a 6 segnali PWM indipendenti utilizzando 4
timer, con una risoluzione massima di 14 bit[^3]. A differenza del modello superiore
ESP32, l'ESP32-C3 supporta solamente l'output a bassa velocità (_low speed channel_),
che viene gestito tramite software[^4].

La creazione delle strutture dati necessarie per l'impostazione di un canale LEDC
richiede due informazioni importanti: la frequenza della PWM e la risoluzione. La prima
è facilmente determinabile, poiché il servo SG90 funziona con un periodo di $20ms$, da
cui si ottiene che la frequenza è $\frac{1}{20ms}=50Hz$.

La scelta dei bit di risoluzione richiede una considerazione aggiuntiva, dato che la
frequenza del timer e i bit della risoluzione sono interdipendenti e legata alla
frequenza del clock, che nel caso dell'ESP32-C3 è di $80MHz$. Più alta è la frequenza
della PWM, più bassa è la risoluzione (e viceversa). Dato che non è possibile generare
un'onda più veloce di quella consentita dal clock, si dovrebbe puntare a mantenere
$f_{PWM}*2^{DUTY\_RES} < 80MHz$[^5]. La frequenza utilizzata è $50Hz$, quindi è
possibile utilizzare al massimo 14 bit di risoluzione. Il minimo è stato determinato con
tramite un processo di _trial and error_, grazie ai messaggi di errore dell'API, ed è
pari a 9 bit. Il valore finale scelto è 11 bit, poiché un valore maggiore di 11 non
garantiva una precisione aggiuntiva nella conversione.

```rust
...
let timer_config = TimerConfig {
    frequency: Hertz(50),
    resolution: RESOLUTION,
};
let timer_driver = LedcTimerDriver::new(timer, &timer_config).unwrap();
let driver = LedcDriver::new(channel, timer_driver, gpio).unwrap();
...
```

## Riferimenti

[^1]: [Servo motor SG90](http://www.ee.ic.ac.uk/pcheung/teaching/DE1_EE/stores/sg90_datasheet.pdf)
[^2]: Espressif docs, [LED Control (LEDC)](https://docs.espressif.com/projects/esp-idf/en/v5.2.2/esp32c3/api-reference/peripherals/ledc.html)
[^3]: [LEDC features](https://www.espressif.com/sites/default/files/documentation/esp32-c3_technical_reference_manual_en.pdf#ledpwm)
[^4]: [LEDC High and Low Speed Mode](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/ledc.html#ledc-high-and-low-speed-mode)
[^5]: [ESP32 Basics: Generating a PWM Signal on the ESP32](https://lastminuteengineers.com/esp32-pwm-tutorial/)
