# Avvio ROSAIA su Ubuntu 24.04

Eseguire dalla cartella del progetto:

```bash
bash 01_prepara.sh --test
bash 02_avvia.sh
```

Il primo script compila il workspace e, con `--test`, esegue i test.
Richiede ROS 2 Jazzy e il pacchetto `librealsense2-dev`; non installa il sistema
operativo e non carica automaticamente il firmware.

Il secondo avvia bridge, controllore encoder, tracker C++ e dashboard:
http://127.0.0.1:8765. L'avvio diagnostico funziona anche senza tutti i marker
visibili. Calibrazione e demo mantengono il controllo di visibilita' completo.

Per arrestare: `bash 02_avvia.sh stop`.
Per consultare dispositivi e nodi: `bash 02_avvia.sh status`.
Per abilitare le azioni motore della dashboard, arrestare prima lo stack e
avviare `bash 02_avvia.sh motion`. L'avvio non esegue una traiettoria.

La configurazione attuale e i modelli sono specifici del setup verificato:
Bluno, due motori primari, RealSense D435 e marker DICT_4X4_50 da 30 mm.
Prima di riutilizzare i modelli controllare home, riferimenti camera e meccanica.
Questi script costituiscono l'avvio locale; la preparazione della USB e delle
dipendenze su un altro computer e' un passaggio separato.
