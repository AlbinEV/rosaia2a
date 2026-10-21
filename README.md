# ROSAIA — movimentazione di due bracci

Albin Bajrami

Versione Ubuntu: firmware Bluno e controllo ROS 2 di due motori con encoder
e lettura delle correnti. Senza CAD, camera o training. Questa versione
essenziale usa comandi ROS da terminale, non la dashboard web.

## Requisiti e installazione

Ubuntu 24.04, ROS 2 Jazzy, Bluno/ATmega328P, shield Makerfabs H-Bridge,
due motori con encoder e alimentazione adatta al banco.

Seguire [la guida di installazione](docs/INSTALLAZIONE.md) per ROS,
dipendenze, accesso USB, firmware e compilazione.

## Avvio

Nella cartella del progetto:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch bluno_motor_bridge bluno_motor_bridge.launch.py
```

In un secondo terminale caricare gli stessi due `setup.bash`.
Leggere gli encoder con `ros2 topic echo /encoder_counts` e lo stato completo
con `ros2 topic echo /motor_state`.

Solo con banco libero e sotto supervisione, un piccolo movimento per asse:

```bash
ros2 topic pub --once /motor_step_command std_msgs/msg/Int16MultiArray '{data: [0, -5, 150]}'
# Attendere la fine prima del secondo comando.
ros2 topic pub --once /motor_step_command std_msgs/msg/Int16MultiArray '{data: [1, -5, 200]}'
```

Formato: `[asse, conteggi con segno, PWM]`. Asse 0 → M2, asse 1 → M1.
Il segno seleziona il verso motore: verificare il verso sul proprio banco.
I PWM provengono dal banco precedente, non sono valori universali.
Il bridge accetta al massimo 10 conteggi per comando. Questo è uno
spostamento relativo, non una chiusura completa calibrata.

STOP:

```bash
ros2 topic pub --once /motor_command std_msgs/msg/Int16MultiArray '{data: [0, 0]}'
```

Prima di scollegare, inviare STOP, chiudere il bridge con Ctrl+C e spegnere
l'alimentazione motori. Non avviare altri controllori contemporaneamente.
Non azzerare encoder fuori dalla HOME meccanica e non disabilitare watchdog
o limiti. La corrente convertita dagli ADC è una stima da calibrare sul banco.
