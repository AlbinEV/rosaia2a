# Demo semplice OPEN/CLOSE

## Versione finale consegnata

Avvio: `bash 03_demo_finale.sh`. Guida operativa completa: `GUIDA_DEMO.html`.
La pagina principale su porta 8767 mostra OPEN, CLOSE, STOP e intensita' PWM
50..120% dei valori base [150,200] in contrazione, [120,140] in rilascio.
La regolazione e' permessa solo a motori fermi. `/simple/pose` usa i passi
firmware alternati e i nominali salvati, non il continuo sperimentale.
I controlli di esplorazione restano su `/manual`, non nella pagina finale.
Ultima prova integrata al 100%: CLOSE [83,136], poi OPEN [3,0], senza errori.
I motori sono stati lasciati fermi vicino alla HOME.
La registrazione conserva comandi, percentuale PWM, encoder e forma reale.

Un wrap del timer MCU a 32 bit e' distinto da un riavvio mediante sequenza
consecutiva e continuita' degli encoder. Il riferimento della sessione
corrente e' stato ripristinato dal log verificato senza azzerare gli encoder
in posizione contratta. Dopo un vero reset resta richiesta la conferma HOME.

## Comando continuo: implementato ma NON abilitato

Il prototipo `/simple/smooth` pubblica comandi PWM ai due assi insieme,
con rampa, feedback di velocita' encoder e rallentamento terminale. Non
usa il modello neurale e non promette controllo di forza o sincronizzazione
precisa. I test di apertura hanno raggiunto circa [-6,8] e [-7,0].

La chiusura non e' validata: in due prove e' intervenuta la soglia di corrente
stimata host (2.8 A), anche prima del secondo nominale. Le prove sono state
interrotte con entrambi gli output a zero. Non aumentare/disattivare la
soglia per far passare il test. La scala dei sensori di corrente necessita
verifica: un intervento della stima non dimostra da solo la corrente reale.
Un tentativo di compensare il rilassamento oltre il nominale e' stato
rimosso dopo l'intervento della protezione.

`CONTINUOUS_ENABLED=False` disabilita il pulsante e rifiuta richieste dirette
al relativo endpoint. I comandi manuali +/- e i nominali salvati restano
disponibili. Prima di riabilitarlo verificare alimentazione, corrente
misurata e tensione/trasmissione dei tendini, poi ripetere la validazione.
Evidenza: registrazioni `simple_demo_20261020_205452` e
`simple_demo_20261020_205545` in `data/sessions/`.

## Prove nominali a due assi completate

Con il nuovo riferimento encoder confermato dall'autore, le chiusure
salvate sono q0=84, q1=139 (raw [84,-139]); HOME e' [0,0]. Due cicli
HOME/CLOSE sono stati eseguiti con `python3 scripts/test_saved_closures.py`:

- HOME 1 [0,0], CLOSE 1 [84,136].
- HOME 2 [0,0], CLOSE 2 [80,140].

Si tratta di posizioni encoder approssimate, non di precisione garantita
o di verifica della distanza 10 cm. I passi dei due motori sono alternati,
non simultanei. Il movimento rimane a passi, non ancora fluidificato.
Il rilascio dell'asse 1 a PWM 120 si era fermato a q=68; con PWM 140
entrambi i ritorni sono stati completati. I PWM attuali sono contrazione
[150,200] e rilascio [120,140].
Evidenza: `data/workflows/saved_closures_20261020_204414/result.json` e
registrazione automatica `data/sessions/simple_demo_20261020_204403/`.

## Controlli manuali attuali (+/−)

La pagina ora presenta un pannello per ciascun asse fisico, con + per
contrarre e − per rilasciare. Ogni clic invia un solo passo firmware di
1..10 conteggi nominali, modificabili separatamente nei due campi Step
(valore iniziale 5). Il massimo 10 e' imposto dal bridge firmware attuale,
non dal limite della posizione totale. L'encoder effettivo puo' superare il
passo per inerzia.
Durante un movimento tutti i nuovi comandi sono bloccati: nessuna coda,
nessuna ripetizione continua tenendo premuto il pulsante. STOP resta attivo.
Si muove solo il motore cliccato. PWM contrazione: asse 0=150, asse 1=200;
rilascio asse 0=120, asse 1=140. Il secondo asse e' esplorativo: l'associazione visiva e la
contrazione non sono ancora verificate. Non usare i suoi conteggi come prova
di variazione di forma.

Il limite esplorativo software q=120 e il blocco di rilascio a q=0 sono stati
rimossi su richiesta dell'autore. Non vi e' un finecorsa assoluto software
per il jog: e' responsabilita' dell'operatore evitare sovratensione dei
tendini, avvolgimenti inversi o finecorsa fisici. Restano l'arresto su
escursione inattesa per singolo clic, corrente stimata, perdita camera/
telemetria o reset, timeout e protezioni firmware. Non e' un intervallo di
prestazioni validato. Se si sente solo ronzio o il
cavo e' gia' teso senza movimento, premere STOP e controllare la meccanica.
La posizione operativa cresce positivamente per entrambi gli assi; il raw
dell'asse 1 e' negativo durante la contrazione.

Salva OPEN attuale / Salva CLOSE attuale registra la posizione misurata
del solo motore selezionato in `data/control_tests/jog_nominals.json`.
Non causa movimento e non attiva automaticamente un replay a due bracci.
I precedenti controlli manuali a quota assoluta sono nascosti nella pagina,
ma i loro endpoint rimangono disponibili con i vecchi limiti.

Con lo stack camera/bridge/dashboard gia' attivo, HOME confermata e
controllore di velocita' spento:

```bash
bash scripts/rosaia.sh simple-demo
```

Aprire http://127.0.0.1:8767. Un pulsante alterna CLOSE e OPEN, con STOP
sempre separato. CLOSE usa asse fisico 0, encoder circa 20; OPEN circa 0,
con tolleranza 4 conteggi. PWM contrazione 150, rilascio 120, passi firmware
di massimo 5 conteggi. Sono presenti timeout, limiti locali, controllo di
telemetria/camera e arresto su reset della board. Nessun azzeramento automatico.

Agisce soltanto sul braccio verificato, etichettato ARM 2 dalla camera.
Il secondo motore resta fermo. NON e' una demo sincronizzata a due bracci e
NON raggiunge la distanza di 10 cm tra ID 4 e 7. Non usa una rete neurale.
Le precedenti reti hanno riferimento encoder/associazione non validi per
questo controllo e non vengono usate per scegliere i movimenti.

## Posizione encoder e nominali

Il campo Encoder target accetta conteggi interi tra 0 e 30 sull'asse 0.
Invia esegue un avvicinamento grossolano al target (tolleranza 4 conteggi,
senza correzioni ripetute dopo il primo attraversamento). La posizione
encoder effettiva viene mostrata accanto ai valori OPEN e CLOSE.

Dopo aver inviato e verificato una posizione, Salva OPEN oppure Salva CLOSE
memorizza il valore del campo, senza muovere il robot. Il valore deve essere
entro 4 conteggi dalla lettura attuale; OPEN deve essere minore di CLOSE.
I nominali sono salvati atomicamente in
`data/control_tests/simple_demo_nominals.json`, ricaricati al riavvio e
annotati nel registro segnali. Ricontrollarli se si cambia HOME o tensione
dei tendini. Il primo avvio usa OPEN=0 e CLOSE=20 finche' l'operatore non salva.

Encoder, correnti, PWM e forma osservata vengono registrati automaticamente
in `data/sessions/simple_demo_*/signals.jsonl`, con timestamp host. Il file
contiene eventi reali e comandi, non un video. La pagina mantiene la camera
e la ricostruzione gia' disponibili nel dashboard sulla porta 8765.

Il programma richiede HOME gia' confermata, telemetria e camera attive.
Il jog non impone piu' un limite assoluto di posizione all'avvio. Riavviare la sola
interfaccia conserva la posizione, senza azzeramenti o ritorni automatici.
Non recupera automaticamente una posizione ignota. Un reset della board richiede
di ristabilire la HOME prima di riavviarlo. Per terminare: STOP, poi Ctrl+C
nel terminale; spegnere l'alimentazione motori prima di lasciare il banco.
