# Servo driver

## Obiettivo

Lo scopo della libreria è quello di implementare un semplice driver per il motore servo SG90, data l'assenza di *crate* pubbliche.

## Implementazione

### Materiale utilizzato

- ESP32C3

- Servo SG90

### Implementazione

A differenza di `mpu6050-driver`, il driver non è stato realizzato utilizzando la libreria `embedded-hal`, ma esclusivamente l'astrazione hardware di `esp_idf_hal`.

#### PWM

Il processo di creazione del controllore PWM è semplice e diretto, due sono le strutture dati necessarie da creare: 

- `LedcTimerDriver`: la configurazione del timer;

- `LedcDriver`: il driver effettivo.



`LedcTimerDriver` necessita di scegliere uno dei quattro *timer* ([LED PWM Controller](https://www.espressif.com/sites/default/files/documentation/esp32-c3_technical_reference_manual_en.pdf#ledpwm)) di sistema e di configurarlo:

```rust
...
let timer_driver = LedcTimerDriver::<'d>::new(
    timer,
    &TimerConfig::default()
        .frequency(50.Hz())
        .resolution(Resolution::Bits9),
)?;
...
```

Due sono i parametri fondamentali: la *frequenza* e la *duty resolution*. Impostare la frequenza del timer è un'operazione diretta dato che è riportata all'interno del data sheet del SG90 ed è pari a: $\frac{1}{20ms}=50Hz$.

La scelta della *duty resolution* richiede una considerazione aggiuntiva dato che, come scritto all'interno della documentazione del'ESP32C3, la frequenza e la *duty resolution* sono interdipendenti.

> "The frequency and the duty resolution are interdependent. The higher the PWM frequency, the lower the duty resolution which is available, and vice versa."

Il valore di 9 bits (`Resolution::Bits9`) è stato scelto tramite *trial and error*, grazie ai messaggi di errore dell'API di LEDC, che è progettata per segnalare un errore quando si tenta di impostare una frequenza e una risoluzione di lavoro che superano l'intervallo dell'hardware del LEDC. specificando se abbassare o aumentare tali valori.



Il passo finale è quello di creare l'effettivo driver (`LedcDriver`) che necessita di selezionare il *canale*, il *gpio* su cui erogare la PWM e infine il driver del timer configurato precedentemente.

```rust
...
let driver = LedcDriver::new(channel, timer_driver, gpio)?;
...
```

#### Calcolo dell'angolo

Il servomotore può ruotare di circa $180°$, con una larghezza dell'impulso di $500us$ per $-90°$ e $2500us$ per $+90°$.

Il primo step è, dato un angolo $0° \leq \alpha \leq 180°$, di mapparlo all'interno dell'intervallo $500...2500$:

$$
\alpha_{us} = (\frac{\alpha}{180}*(2500-500))+500
$$

Il secondo step è quello di mappare l'angolo in microsecondi all'interno dell'intervallo $0...2^9-1$, ovvero la *duty resolution* (`Resolution::Bits9`)

$$
duty = \alpha_{us} * (2^9-1) / 20000 = \alpha_{us} * 511 / 20000
$$

Tramite queste due formule è possible scrivere la funzione che, dato l'angolo, faccia muovere il servomotore in quella (circa) posizione:

```rust
pub fn write_angle(&mut self, angle: u32) -> Result<()> {
    let angle_us = ((angle as f32 / MAX_ANGLE) * (MAX_DUTY_US - MIN_DUTY_US)) + MIN_DUTY_US;

    let duty: u32 = (angle_us * self.max_duty as f32 / FREQ) as u32;
    self.driver.set_duty(duty)?; // imposto il duty
    Ok(())
}
```



Le formule per il processo inverso sono piuttosto semplici da ottenere:

$$
\alpha_{us} = 20000*duty/(2^9-1)=20000*duty/511
$$

$$
\alpha = \frac{\alpha_{us}-500}{2500-500}*180
$$

E la relativa funzione:

```rust
pub fn read_angle(&mut self) -> u32 {
    let duty = self.driver.get_duty();
    let angle_us = (duty as f32 * FREQ) / self.max_duty as f32;

    ((angle_us - MIN_DUTY_US) / (MAX_DUTY_US - MIN_DUTY_US) * MAX_ANGLE) as u32
}
```

In questo caso occorre considerare l'errore dato dalle conversioni da `u32` a `f32` e viceversa, infatti il valore che si ottiene da `read_angle` è diverso dal parametro che si passa a `write_angle` come si può notare dal seguente log.

```log
...
I (336) servo_driver: Writing angle 0
I (336) servo_driver: Reading angle 0

I (356) servo_driver: Writing angle 1
I (356) servo_driver: Reading angle 0

I (376) servo_driver: Writing angle 2
I (376) servo_driver: Reading angle 0

I (396) servo_driver: Writing angle 3
I (396) servo_driver: Reading angle 0

I (416) servo_driver: Writing angle 4
I (416) servo_driver: Reading angle 0

I (436) servo_driver: Writing angle 5
I (436) servo_driver: Reading angle 4

I (456) servo_driver: Writing angle 6
I (456) servo_driver: Reading angle 4

I (476) servo_driver: Writing angle 7
I (476) servo_driver: Reading angle 4

I (496) servo_driver: Writing angle 8
I (496) servo_driver: Reading angle 7

I (516) servo_driver: Writing angle 9
I (516) servo_driver: Reading angle 7
...
```
