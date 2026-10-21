# Windows — stato del porting

Autore e manutentore: Albin Bajrami.

Il ramo `windows` è il punto di partenza del porting, non una release Windows
funzionante. La versione operativa attuale è Ubuntu, documentata nel README.
Non eseguire gli script Bash con l'aspettativa che funzionino su Windows nativo.

## Obiettivo

Due script di preparazione e avvio, backend locale, stessa interfaccia web
OPEN/CLOSE, telemetria encoder/correnti e camera. Nessun comando motore deve
essere inviato durante installazione o controllo delle dipendenze.

## Parti da adattare

- Avvio/arresto: sostituire Bash, `/opt/ros`, `/proc`, `os.getuid` e gestione
  POSIX dei gruppi di processi in `rosaia_workflow.py` e `start_final_ui.py`.
- Seriale: individuare la porta COM della board invece del percorso
  `/dev/serial/by-id`; mantenere protocollo firmware e gestione dei riavvii.
- Backend: decidere e implementare il runtime Windows; il backend attuale
  `simple_open_close.py` dipende da ROS 2, non basta installare pyserial.
- Visione: adattare e verificare acquisizione RealSense e pipeline ArUco;
  la sola anteprima RGB non sostituisce la ricostruzione RGB-D.
- Dipendenze e avvio: aggiungere installer/launcher Windows solo dopo aver
  definito il runtime e provato le dipendenze su Windows.

HTML/JavaScript e codec seriale Python sono candidati al riuso. Non è ancora
stata verificata l'intera catena su Windows.

## Verifiche necessarie prima del rilascio

1. Preparazione su un PC Windows pulito e individuazione USB/COM/camera.
2. Telemetria e camera senza alimentazione dei motori.
3. HOME esplicita, STOP, timeout, scollegamento USB e riavvio della board.
4. Movimenti supervisionati con nominali e PWM verificati per quel banco.
5. Salvataggio dei segnali e arresto completo senza processi residui.

Non disattivare watchdog, controllo dei dati obsoleti o protezioni di corrente
per far funzionare il porting. I test su Ubuntu non certificano Windows.
