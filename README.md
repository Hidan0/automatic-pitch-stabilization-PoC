# PoC - Stabilizzazione automatica del beccheggio

Autore: Riccardo Monilia

Il progetto consiste nello sviluppo di un sistema in grado di rilevare l'assetto di un
aereo di carta, in particolare l'angolo di beccheggio, e di regolare di conseguenza
l'inclinazione della superficie di controllo (elevatore), compensando eventuali
deviazioni rispetto alla posizione desiderata.

Il sistema è basato su un ESP32-C3 che gestisce i dati provenienti da un sensore
IMU MPU6050 per rilevare l'orientamento del velivolo e controlla il servomotore
responsabile della regolazione della superficie di volo. Il software implementato sul
SoC è progettato per interpretare i dati del sensore e applicare le correzioni
necessarie per mantenere l'aereo in equilibrio.

**Componenti elettronici**:

- ESP32-C3
- IMU MPU6050
- SG90
- Breadboard da 830 contatti
- Led verde e rosso
- Cavi e jumper

**Software**:

- Linguaggio di programmazione: Rust
- Ecosistema [esp-rs](https://github.com/esp-rs) per Rust
- Librerie:
  - [mpu6050](https://crates.io/crates/mpu6050)
  - [derive_more](https://crates.io/crates/derive_more)

[Link a repository](https://github.com/Hidan0/automatic-pitch-stabilization-PoC/)

Licenza: Apache-2.0 e MIT

Data _indicativa_ di presentazione: giugno 2025
