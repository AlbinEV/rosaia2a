# Riordino del 9 ottobre 2026

Prima delle modifiche sono stati creati due checkpoint locali:

- workspace `rosaia-lego`, repository autonomo: `d8af06b`;
- repository annidato `CAD-IROS`, branch del braccio avvolgente: `2287d87`.

Nessun push o modifica a hardware e robot. La cartella RoboSoft rimane
separata e non e' stata spostata. I dati e le cache ignorati da Git non
sono stati cancellati o spostati: un commit non e' un backup di quei dati.

## Percorsi

- CAD attivo: `CAD-IROS/03_BRACCIO_AVVOLGENTE_1U/CAD/braccio_avvolgente.scad`.
- Stampa corrente: `CAD-IROS/STAMPA`, collegamento alla stampa del CAD attivo.
- Vecchi modelli: `CAD-IROS/ARCHIVIO_MODELLI/01_PROTOTIPO_IROS` e `02_BRACCIO_PETALI_V7`.
- Le precedenti cartelle CAD 01/02 alla radice restano alias, non copie.
- Hardware: `docs/hardware/CONTROL_BOARD_BLUNO_MAKERFABS_ROS2_JAZZY.md`;
  il vecchio percorso alla radice e' un collegamento compatibile.
- Disegni di prova 2:1, provini e log ritirati: archivio della versione attiva.
- Firmware, ROS, scripts, config e data restano ai loro percorsi operativi.

Inventario con hash prima/dopo in
`CAD-IROS/03_BRACCIO_AVVOLGENTE_1U/ARCHIVIO/08_Riordino_2026-10-09/inventario_riordino.json`.
Nessun file materiale e' stato eliminato.

La variante successiva ai checkpoint usa numeri romani I–X e appoggi
dorsali per limitare l'iperestensione a riposo. La numerazione e la
geometria attuali non vanno confuse con gli archivi o la prova 2:1.
Le verifiche geometriche e la stima di peso proprio non sono prove di
resistenza, fatica o compensazione della gravita' lungo tutta la corsa.
