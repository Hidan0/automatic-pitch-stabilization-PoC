# "Driver" MPU6050

## Intro

Per stabilizzare l'_aereo_ è necessario conoscere il suo orientamento nello spazio.
Per far questo è sufficiente conoscere le velocità di rotazione durante il _rollio_ (**roll**), il _beccheggio_ (**pitch**) e l'_imbardata_ (**yaw**).

Un sensore in grado di registrare questi tassi di rotazione è il **giroscopio**, nel mio caso è comodamente integrato nell'MPU6050.

## Obiettivo

Lo scopo del progetto è quello di scrivere un semplice driver per l'MPU6050 per misurare il tasso di rotazione (in gradi/secondo) durante rollio, beccheggio e imbardata, in modo da
comprendere bene il funzionamento di un sensore attraverso la lettura del data-sheet e della sua implementazione.

## Realizzazione

### Materiale utilizzato

- [MPU6050 data sheet](https://arduino.ua/docs/RM-MPU-6000A.pdf)
- ESP32 38 pins
- MPU6050

### Implementazione

Il driver è stato realizzato usando la libreria [embedded-hal](https://github.com/rust-embedded/embedded-hal), che funge da base per la creazione di un ecosistema omogeneo per i sistemi embedded su Rust.

#### I2C

La comunicazione _I2C_ è effettuata tramite la combinazione dei trait `i2c::WriteRead` e `i2c::Write` del'_hal_.

```rust
#[derive(Debug)]
pub struct Mpu6050<I2C> {
    i2c: I2C,
    ...
}

impl<I2C, E> Mpu6050<I2C>
where
    I2C: i2c::WriteRead<Error = E> + i2c::Write<Error = E>,
{
    ...
}
```

Per semplificarne l'utilizzo ho realizzato i seguenti metodi:

```rust
fn write_byte(&mut self, register: u8, byte: u8) -> Result<(), E> {
    self.i2c.write(DEVICE_ADDR, &[register, byte])
}

fn read_2c_word(&mut self, register: u8) -> Result<i16, E> {
    let mut buffer = [0, 0];
    self.i2c.write_read(DEVICE_ADDR, &[register], &mut buffer)?;

    let high: u16 = buffer[0] as u16;
    let low: u16 = buffer[1] as u16;

    let word = (high << 8) | low;

    Ok(word as i16)
}
```

`write_byte` e `read_2c_word` sono due metodi helper rispettivamente per la scrittura di un byte su un registro e per la lettura di una parola (2 byte) interpretata come complemento a due.

#### Gestione del driver

La procedura generale per la gestione del driver è piuttosto semplice:

1. identificare nel data sheet del sensore il suo indirizzo I2C di base;
2. identificare nella mappa dei registri la funzionalità di cui si è interessati;
3. una volta identificata la funzionalità desiderata basterà costruire il byte secondo le indicazioni della sezione ed inviarlo presso il registro a cui fa riferimento.

Consideriamo ad esempio l'istruzione per impostare il filtro passa basso e analizziamo le costanti utilizzate:

```rust
/// Mpu6050 device address
const DEVICE_ADDR: u8 = 0x68;
/// Digital Low Pass Filter address
const DLPF_ADDR: u8 = 0x1A;
/// DLPF configuration of 10Hz bandwidth filter for Gyroscope
const DLPF_CFG_GYR_5: u8 = 0x05;

...
self.write_byte(DLPF_ADDR, DLPF_CFG_GYR_5)?;
...
```

- `DEVICE_ADDR`, all'interno del metodo helper `write_byte`, rappresenta l'indirizzo I2C dell'MPU6050;
- `DLPF_ADDR`, rappresenta l'indirizzo del registro 26, utilizzato per impostare il filto DLPF sia per il giroscopio che per l'accellerometro;
- `DLPF_CFG_GYR_5`, è il byte da scriver sul registro 26 ed è costruito seguendo le indicazioni della sezione sul data sheet:
  |Bit7|Bit6|Bit5 - Bit4 - Bit3|Bit2 - Bit1 - Bit0|
  |-|-|-|-|
  |-|-|`EXT_SYNC_SET[2:0]`|`DLPF_CFG[2:0]`|

  In questo caso `DLPF_CFG` è il parametro di interesse e viene impostato secondo la tabella appropriata. Per impostare il filtro a $10Hz$ il valore del parametro deve essere pari a `5`.
  Non essendo interessati a `EXT_SYNC_SET` e dato che i bit 6 e 7 non sono utilizzati, il valore del byte finale è `0x05`.
  |Bit7|Bit6|Bit5|Bit4|Bit3|Bit2|Bit1|Bit0|
  |-|-|-|-|-|-|-|-|
  |0|0|0|0|0|**1**|**0**|**1**|

#### Calibrazione

```rust
fn calibrate<D: DelayMs<u8>>(&mut self, delay: &mut D) -> Result<(), E> {
    for _ in 0..MAX_CALIBRATIONS {
        let (r, p, y) = self.raw_gyro()?;
        self.rate_cal_roll += r;
        self.rate_cal_pitch += p;
        self.rate_cal_yaw += y;
        delay.delay_ms(1);
    }

    self.rate_cal_roll /= MAX_CALIBRATIONS as f32;
    self.rate_cal_pitch /= MAX_CALIBRATIONS as f32;
    self.rate_cal_yaw /= MAX_CALIBRATIONS as f32;

    Ok(())
}
```

Per calibrare un giroscopio, si determina un valore di riferimento per il tasso di rotazione quando il sensore è **fermo**, che idealmente dovrebbe essere zero. Poiché le misure del giroscopio tendono a fluttuare a causa di piccole vibrazioni ambientali, si calcola la media di un gran numero di valori. Il valore medio viene poi sottratto da tutte le future misurazioni per compensare le fluttuazioni e ottenere una misura più accurata.

```rust
pub fn gyro(&mut self) -> Result<(f32, f32, f32), E> {
    let (r, p, y) = self.raw_gyro()?;
    Ok((
        r - self.rate_cal_roll,
        p - self.rate_cal_pitch,
        y - self.rate_cal_yaw,
    ))
}
```
