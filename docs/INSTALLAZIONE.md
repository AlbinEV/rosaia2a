# Installazione — Ubuntu 24.04 / ROS 2 Jazzy

## 1. Installare ROS

Usare Ubuntu **24.04**. Impostare locale UTF-8 e repository ROS seguendo la
[guida ufficiale Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html),
sezioni “Set locale” e “Enable required repositories”. Non usare istruzioni
per altre distribuzioni ROS o Ubuntu.

Dopo aver configurato il repository ufficiale:

```bash
sudo apt update
sudo apt install ros-jazzy-ros-base ros-dev-tools git python3-serial python3-pytest
source /opt/ros/jazzy/setup.bash
```

`ros-base` è sufficiente: RealSense e OpenCV non servono per questi motori.

## 2. Scaricare il codice e installare le dipendenze

```bash
git clone --branch ubuntu https://github.com/AlbinEV/rosaia2a.git
cd rosaia2a
sudo rosdep init
rosdep update
rosdep install --from-paths ros2_ws/src --ignore-src --rosdistro jazzy -y
```

Eseguire `sudo rosdep init` solo se rosdep non è già inizializzato.
Le dipendenze sono dichiarate nel `package.xml`; non serve un ambiente pip.

## 3. Preparare board e porta USB

Caricare `firmware/bluno_motor_controller/bluno_motor_controller.ino` con
Arduino IDE, selezionando il profilo Uno per la Bluno/ATmega328P.
Durante il caricamento scollegare l'alimentazione dei motori.

```bash
sudo usermod -aG dialout "$USER"
```

Uscire dalla sessione e rientrare dopo aver aggiunto il gruppo. Collegare
la board e individuare la porta:

```bash
ls -l /dev/serial/by-id/
```

Impostare `serial_port` in
`ros2_ws/src/bluno_motor_bridge/config/controller.yaml` con il percorso reale.
Non assumere che coincida con quello del PC originale. Chiudere il monitor
seriale Arduino prima di avviare ROS.

Cablaggio del firmware: asse 0 su M2, encoder D2/D3, corrente A0;
asse 1 su M1, encoder D11/D12, corrente A1. Massa comune e livelli elettrici
compatibili sono necessari; verificare il banco prima di alimentare i motori.

## 4. Compilare e verificare

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --symlink-install --packages-select bluno_motor_bridge
source install/setup.bash
colcon test --packages-select bluno_motor_bridge
colcon test-result --verbose
cd ..
```

Per avvio e movimenti tornare al README. Caricare i due `setup.bash` in ogni
nuovo terminale. I test non muovono i motori e non sostituiscono la verifica
fisica di encoder, alimentazione, tendini e STOP.
