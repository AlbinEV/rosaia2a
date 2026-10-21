# Dashboard: curvatura e zona punta

Aprire http://127.0.0.1:8765 dopo `bash 02_avvia.sh`.

La ricostruzione mostra una curva interpolata attraverso base e tre marker
misurati. La curva e' una rappresentazione grafica, non una misura continua
di tutta la superficie. Sono visibili forma misurata, previsione del modello
e zona obiettivo della punta.

Ogni braccio ammette due target: modulo della curvatura in m^-1, oppure centro e
raggio di una zona circolare del marker finale, in millimetri nella vista
raddrizzata sull'home. Per scegliere la zona con un clic, selezionare prima il
braccio nel pannello del modello e fare clic sulla ricostruzione. Premere
Compute inference per proiettare il bersaglio. Questo non aziona i motori.

Il modello MLP rappresenta encoder -> forma. La ricerca inversa del target
produce un encoder obiettivo; il controllore di posizione calcola separatamente
velocita' e accelerazione. La dashboard distingue velocita' pianificata,
riferimento del controllore e velocita' misurata. Le correnti visualizzate
sono stime host da ACS712, dipendenti dalla calibrazione del sensore.

La zona punta viene verificata sui target encoder interi del modello.
Una zona non raggiungibile e' segnalata; l'esecuzione individuale della
proiezione viene rifiutata. Una previsione raggiungibile e' comunque una
previsione del modello, non una dimostrazione di raggiungibilita' hardware.

## Vista home e dimostrazioni registrate (20/10/2026)

La curvatura nell'interfaccia e nell'API /api/curvature e' ora un modulo
non negativo. La ricerca inversa confronta il modulo della curvatura del
modello, conservando il segno internamente per distinguere i bracci speculari.

Il disegno usa basi nominali distanti 0.40 m. Non e' una misura del distacco
fisico. Impostare l'home acquisisce i tre punti di ciascun braccio e azzera
gli encoder a motori fermi. Una rotazione rigida porta soltanto la direzione
base–primo marker dell'home verso il basso. Gli altri marker conservano
la forma misurata; non vengono allineati né sottratti gli scostamenti.
La geometria originale in current.points non viene modificata;
current.display_points contiene esclusivamente la vista raddrizzata.
La calibrazione della vista dura per la sessione del server; dopo un riavvio
impostare nuovamente l'home. Un nuovo azzeramento richiede di rivalidare la
compatibilita' dei modelli con lo zero encoder adottato.

Le zone punta dell'interfaccia usano il frame home_display; la proiezione
trasforma anche i punti previsti dal modello nello stesso frame. Le richieste
API prive di coordinate_frame mantengono il piano grezzo della base.

Registra segnali demo acquisisce stati bracci e segnali motore reali dal
browser, fino a cinque minuti. Termina e salva conserva i campioni nello
storage locale dello stesso browser; Scarica demo JSON esporta i campioni.
Replay registrazione riproduce soltanto la visualizzazione: non comanda i
motori, nasconde l'immagine camera live e disabilita i pulsanti di movimento.
La rete animata e' etichettata come illustrativa, non come inferenza online.
Torna live ripristina i dati correnti. Questa funzione non costituisce una
routine di replay fisico degli encoder e non sostituisce il training.

## Training pilota

Il workflow dispone di `calibrate --quick`: intervallo 0..40 conteggi,
punti training 0,10,20,30,40 e validazione 0,5,15,25,35,40; dwell almeno 1 s.
Le sessioni vengono controllate prima del training e i modelli non vengono
automaticamente marcati come approvati per controllo online.

Il 20/10/2026 questo training non e' stato eseguito: la prova di ritorno del
primo asse supera ancora il piccolo spostamento richiesto e il secondo asse
deve essere verificato con l'alimentazione ripristinata. Prima di acquisire
nuovi sweep servono un ritorno controllato, home meccanica confermata,
camera e basi ferme e il controllore di velocita' sospeso durante gli sweep
firmware locali, come richiesto da run_quasistatic_sweep.sh.

I modelli storici caricati producono, nel controllo statico attuale, uno
scarto minimo della punta di circa 47..54 mm rispetto ai punti osservati.
Non considerarli una calibrazione del montaggio attuale.
