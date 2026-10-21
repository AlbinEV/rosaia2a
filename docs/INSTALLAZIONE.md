# Installazione — Ubuntu 24.04 / ROS 2 Jazzy

Eseguire i blocchi nell'ordine indicato, in un terminale Ubuntu.
Serve **Ubuntu 24.04**, connessione Internet e un utente con accesso `sudo`.
L'installazione non richiede di collegare o alimentare i motori.
Se ROS 2 Jazzy è già installato, passare al punto 3.

## 1. Preparare Ubuntu e il repository ROS

Verificare che la versione indicata sia `24.04`:

```bash
lsb_release -ds
```

Configurare lingua UTF-8 e repository:

```bash
sudo apt update
sudo apt install -y locales software-properties-common curl
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo add-apt-repository -y universe

rosaia_ros_source_version=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | awk -F '"' '/"tag_name"/ {print $4; exit}')
test -n "$rosaia_ros_source_version" && curl -fL -o /tmp/rosaia-ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${rosaia_ros_source_version}/ros2-apt-source_${rosaia_ros_source_version}.noble_all.deb" && sudo dpkg -i /tmp/rosaia-ros2-apt-source.deb
```

Se un comando segnala un errore, fermarsi e risolverlo prima di continuare.

## 2. Installare ROS 2 Jazzy — versione minima

```bash
sudo apt update
sudo apt upgrade -y
sudo apt install -y ros-jazzy-ros-base ros-dev-tools git python3-serial python3-pytest
source /opt/ros/jazzy/setup.bash
ros2 --help
```

Se viene richiesto un riavvio, farlo prima di continuare e aprire un nuovo
terminale. `ros-base` evita RViz, demo e strumenti grafici non necessari:
RealSense e OpenCV non servono per questi motori.

Procedura basata sulla [guida ufficiale ROS 2 Jazzy per Ubuntu](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).

## 3. Scaricare il codice e installare le dipendenze

```bash
source /opt/ros/jazzy/setup.bash
sudo apt install -y ros-dev-tools git python3-serial python3-pytest
cd "$HOME"
git clone --branch ubuntu https://github.com/AlbinEV/rosaia2a.git
cd rosaia2a
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update
rosdep install --from-paths ros2_ws/src --ignore-src --rosdistro jazzy -y
```

Il codice è ora in `~/rosaia2a`. Non ripetere `git clone` se la cartella esiste
già. Le dipendenze sono dichiarate nel `package.xml`; non serve pip.

## 4. Preparare board e porta USB

Se la board ha già il firmware del progetto, **non ricaricarlo**.
Altrimenti caricare `firmware/bluno_motor_controller/bluno_motor_controller.ino`
con Arduino IDE, selezionando il profilo Uno per la Bluno/ATmega328P.
Durante il caricamento scollegare l'alimentazione dei motori.

```bash
sudo usermod -aG dialout "$USER"
```

Uscire dalla sessione e rientrare dopo aver aggiunto il gruppo. Collegare
la board e individuare la porta:

```bash
ls -l /dev/serial/by-id/
```

Aprire la configurazione:

```bash
cd "$HOME/rosaia2a"
nano ros2_ws/src/bluno_motor_bridge/config/controller.yaml
```

Impostare `serial_port` con il percorso reale mostrato dal comando precedente.
Salvare con Ctrl+O, Invio e uscire con Ctrl+X.
Non assumere che coincida con quello del PC originale. Chiudere il monitor
seriale Arduino prima di avviare ROS.

Cablaggio del firmware: asse 0 su M2, encoder D2/D3, corrente A0;
asse 1 su M1, encoder D11/D12, corrente A1. Massa comune e livelli elettrici
compatibili sono necessari; verificare il banco prima di alimentare i motori.

## 5. Compilare e verificare

```bash
source /opt/ros/jazzy/setup.bash
cd "$HOME/rosaia2a/ros2_ws"
colcon build --symlink-install --packages-select bluno_motor_bridge
source install/setup.bash
colcon test --packages-select bluno_motor_bridge
colcon test-result --verbose
cd ..
```

I test non muovono i motori.

## 6. Avviare

```bash
source /opt/ros/jazzy/setup.bash
source "$HOME/rosaia2a/ros2_ws/install/setup.bash"
ros2 launch bluno_motor_bridge bluno_motor_bridge.launch.py
```

Lasciare aperto questo terminale. Il solo avvio del bridge non avvia movimenti.
Per i comandi di movimento e STOP seguire il [README](../README.md).
Caricare i due `setup.bash` anche in ogni nuovo terminale di comando.

Se `ros-dev-tools` segnala conflitti di dipendenze, verificare in
`/etc/apt/sources.list.d/ubuntu.sources` la presenza di `noble-updates` e
`noble-backports`, come indicato nella guida ufficiale. Non rimuovere pacchetti
ROS per aggirare l'errore.
