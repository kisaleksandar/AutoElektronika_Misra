# AutoelektronikaProjekat

## Uvod

Ovaj projekat ima za zadatak da simulira merenje nivoa goriva u automobilu. Kao okruženje koristi se VisualStudio2019. Zadatak ovog projekta osim same funkcionalnosti je bila i implementacija Misra standarda prilikom pisanja koda.

## Ideja i zadaci: 
  1. Podaci o trenutnom stanju nivoa goriva u automobilu se dobijaju svaki sekund sa kanala 0 serijske komunikacije izraženi kao vrednost otpornosti u opsegu od 0 do 10K.
  2. Uzimati poslednjih 5 pristiglih vrednosti i računati prosek.
  3. Na osnovu unetih vrednosti MINFUEL(nešto više od 0) i MAXFUEL(nešto manje od 10K) izvršiti kalibraciju. Ove vrednosti šalju se preko kanala 1 serijske komunikacije. MINFUEL        označava vrednost otpornosti koji odgovara praznom rezervoaru, a MAXFUEL vrednost otpornosti koji odgovara punom rezervoaru automobila. Na osnovu trenutne vrednosti                otpornosti koja je pristigla i ovih parametara, izračunava se trenutni nivo goriva u procentima. 
                                              FORMULA: 100 * (trenutna_otpornost - MINFUEL) / (MAXFUEL - MINFUEL)
  4. Ukoliko stigne poruka sa kanala 1 oblika PPvrednost, vrednost predstavlja kolika je potrošnja goriva u litrima na 100km (npr. PP8 bi označavalo 8l na 100km vožnje). Na      osnovu ove informacije moguće je proračunati autonomiju vozila, odnosno koliko još km se automobil može kretati sa trenutnom količinom goriva u rezervoaru. Za ove potrebe          uveden je #define PUN_REZERVOAR 40 koji označava da pun rezervoar automobila iznosi 40l, kako bi se preostala kilometraža izračunala. 
                                              FORMULA: nivo_goriva_procenti*PUN_REZERVOAR/POTROSNJA
  5. Komande START i STOP imaju za zadatak da izmere kolika količina goriva u procentima je potrošena u vremenskom razmaku između slanja te dve naredbe. START i STOP komanda se   realizuju preko LED bar-a pritiskom na odgovarajući taster. LED bar periferija kada se pokrene, dobijaju se 4 stupca (prvi stubac je krajnji sa leve strane). Ukoliko se na LED baru pritisne prva dioda od gore u četvrtom stupcu aktivira se komanda START i zasvetleće četvrta dioda od dole u drugom stupcu koja označava da je ovo merenje aktivno. Zatim je potrebno isključiti taj taster START (dioda za indikaciju da je merenje aktivno i dalje svetli). Zatim je potrebno poslati neku manju vrednost otpornosti (simulacija da je vremenom goriva sve manje), nakon čega pritisnuti taster drugi po redu od gore (ispod START tastera) u četvrtom stupcu, odnosno STOP taster. Tada se gasi dioda za indikaciju aktivnog merenja, pa isključiti i taster STOP. Prilikom pritiska START i STOP tastera, u tim trenucima zapamtile su se vrednosti nivoa goriva u procentima, tako da se podatak o potrošenoj količini goriva u procentima dobija kao njihova razlika.
  6. Prilikom računanja nivoa goriva u procentima, ukoliko ta vrednost iznosi manje od 10% pali se prva dioda od dole u prvom stupcu LED bar-a. U suprotnom ona ne svetli.
  7. Potrebno je preko kanala 1 slati nivo goriva u procentima svakih 1s.
  8. Ukoliko se pritisne prva dioda od dole u trećem stupcu (i samo je ona aktivna) na 7seg displeju se ispisuje skroz sa leve strane trenutni nivo goriva u procentima (rezervisane 3 cifre za to), a na kraju (poslednjih 5 cifara) se ispisuje trenutna pristigla otpornost. Kada nijedan taster nije aktivan u tom stupcu, 7seg displej je isključen. Ukoliko se pritisne druga dioda (taster) od dole u trećem stupcu tada se na 7seg displeju na levoj strani ispisuje koliko još km automobil može da se kreće sa trenutnom količinom goriva, a na samom kraju 7seg displeja ispisuje se rezultat poslednjeg merenja START-STOP komande opisane u tački 5. Brzina osvežavanja displeja je 100ms.
  

## Periferije

Periferije koje je potrebno koristiti su LED_bar, 7seg displej i AdvUniCom softver za simulaciju serijske komunikacije.
Prilikom pokretanja LED_bars_plus.exe navesti RRrr kao argument da bi se dobio led bar sa 2 izlazna i 2 ulazna stupca crvene boje.
Prilikom pokretanja Seg7_Mux.exe navesti kao argument broj 9, kako bi se dobio 7-seg displej sa 9 cifara.
Što se tiče serijske komunikacije, potrebno je otvoriti i kanal 0 i kanal 1. Kanal 0 se automatski otvara pokretanjem AdvUniCom.exe, a kanal 1 otvoriti dodavanjem broja jedan kao argument: AdvUniCom.exe 1

## Kratak pregled taskova

Glavni .c fajl ovog projekta je main_application.c

### void SerialSend_Task0(void* pvParameters)
S obzirom da vrednost trenutne otpornosti treba dobiti svaki sekund sa kanala 0 serijske komunikacije, od strane FreeRTOS-a je to omogućeno tako što se u ovom tasku svakih 1s šalje karakter 'a' preko kanala 0. Kada se pokrene AdvUniCom.exe potrebno je označiti opciju AUTO, odnosno svaki put kad stigne karakter 'a', da se pošalje naredba oblika Rvrednost. (npr R7000. ) Vrednost koja se pošalje je zapravo vrednost trenutne otpornosti (npr. 7000). Povremeno je potrebno ručno u AdvUniCom softveru menjati ovu vrednost kako bi se simulirala promena nivoa goriva u automobilu.

### void SerialReceive_Task0(void* pvParameters)
Ovaj task ima za zadatak da obradi podatke koji stižu sa kanala 0 serijske komunikacije. Podatak koji stiže je u vidu poruke formata Rvrednost. (npr. R7000.). To je podatak o trenutnoj vrednosti otpornosti. Karakteri se smeštaju u niz iz kog se izvlači vrednost (7000) i smešta se u red, kako bi ostali taskovi taj podatak imali na raspolaganju za dalje računanje. Ovaj task "čeka" semafor koji će da pošalje interrupt rutina svaki put kada neki karakter stigne na kanal 0.

### void SerialReceive_Task1(void* pvParameters)
Ovaj task ima za zadatak da obradi podatke koji stižu sa kanala 1 serijske komunikacije. Naredbe koje stižu su formata \00naredba\0d. Primeri su: \00MINFUEL10\0d, \00MAXFUEL9000\0d, \00PP8\0d. Ovaj task će iz ovih  naredbi izvući vrednost za MINFUEL (u ovom primeru 10), vrednost za MAXFUEL (u ovom primeru 9000), vrednost za potrosnju u litrima na 100km (u ovom primeru 8). Te vrednosti smestiće u globalne promenljive koje se koriste u drugim taskovima. Task takođe kao i prethodni čeka odgovarajući semafor da bi se odblokirao i izvršio. Taj semafor daje interrupt rutina svaki put kada pristigne karakter na kanal 1.

### void SerialSend_Task1(void* pvParameters)
Ovaj task ima za zadatak da šalje trenutnu vrednost goriva u procentima na kanal 1 serijske komunikacije svakih 1s. Task šalje samo pod uslovom da su parametri MINFUEL i MAXFUEL uneti, odnosno poslati prethodno sa kanala 1. Ovde bi elegantnije rešenje bilo koristiti for petlju za slanje redom karaktera ovog str stringa, međutim iz nepoznatog razloga je na taj način stizao samo prvi karakter stringa str na kanal 1. Da bi se to rešilo uveden je brojač, kako bi se osiguralo da svaki naredni put šalje naredni karakter iz stringa str.

### void nivo_goriva_u_procentima(void* pvParameters)
Ovaj task ima za zadatak da preračuna trenutni nivo goriva u procentima i izračuna koliko još km moze automobil da se kreće sa trenutnom količinom goriva. Ukoliko je nivo goriva u procentima manji od 10% pali diodu (tačka 6. Ideja i zadaci). Računanje je omogućeno tek kada su svi parametri MINFUEL, MAXFUEL i PP (potrošnja u litrima) uneti, odnosno pristigli sa kanala 1 serijske komunikacije, u suprotnom nema smisla računati. Task preuzima iz reda vrednost otpornosti i računa na osnovu formula potrebne informacije.

### void merenje_proseka_nivoa_goriva(void* pvParameters)
Ovaj task računa prosek poslednjih 5 vrednosti pristiglih otpornosti koje preuzima iz reda i ukoliko je potrebno ispisuje prosek na terminal. (Otkomentarisati printf naredbu)

### void led_bar_tsk(void* pvParameters)
Task koji na osnovu pritisnutog tastera ispisuje na 7seg displej informacije, brzina osvezavanja 100ms. Pritiskom na jedan taster ispisuje nivo goriva u procentima i otpornost. (tačka 8. Ideja i zadaci). Pritiskom na drugi taster ispisuje koliko jos kilometara moze da se krece i ispisuje i računa koliki je rezultat START-STOP naredbe (tačka 5. Ideja i zadaci). 

### void main_demo(void)
U ovoj funkciji se vrši inicijalizacija svih periferija koje se koriste, kreiraju se taskovi, semafori i red, definiše se interrupt za serijsku komunikaciju i poziva vTaskStartScheduler() funkcija koja aktivira planer za raspoređivanje taskova.

## Predlog simulacije celog sistema
Prvo otvoriti sve periferije na način opisan gore. Pokrenuti program. U prozoru AdvUniCom softvera kanala 0, upisati da kada stigne karakter 'a', kanal 0 šalje naredbu formata npr R7800. (tačka na kraju označava kraj naredbe i obavezna je, slovo R je početak naredbe). Čekirati Auto i ok1. Na taj način je trenutna otpornost koja se šalje npr. 7800 oma i šalje se svaki sekund ka FreeRTOS-u. Promenom te vrednosti, simulira se pristizanje različitih vrednosti otpornosti. U kanalu 1 serijske komunikacije poslati naredbe za MINFUEL, MAXFUEL i PP(potrošnju). Na primer: \00MINFUEL10\0d , zatim \00MAXFUEL9000\0d, zatim \00PP8\0d Početak poruke označava hex 0x00, kraj poruke CR (carriage return) koji je u hex formatu 0x0d. Na taj način su uneti parametri, na osnovu kojih je moguće i ima smisla dalje računati. MINFUEL tada ima vrednost 10, MAXFUEL je 9000 i potrošnja je 8l na 100km. Tada u prozoru kanala 1 se pojavljuje nivo goriva u procentima. Uneti na primer vrednost otpornosti 70, odnosno poslati R70. Tada će zasvetleti u LED_bar-u odgovarajuća dioda, jer je za tu vrednost otpornosti nivo goriva u procentima manji od 10%. Kada se vrati stara vrednost npr. 7800 oma, tada se ta dioda isključuje. Zatim se može simulirati rad START-STOP merenja. Pritisnuti taster START i potom isključiti. (Svetli odgovarajuća dioda za indikaciju da je merenje aktivno). Detalji koji taster na LED_bar-u pripada kojoj funkcionalnosti dato je u listi Ideje i zadaci. Zatim promeniti vrednost otpornosti koja se šalje, smanjiti je, jer se vremenom nivo goriva smanjuje. Ako se slalo npr. R7800. poslati sada npr. R6000. i zatim pritisnuti taster STOP i potom isključiti ga (simulacija pritiskanja tastera). Merenje je završeno i vrednost potrošene količine goriva u procentima može se prikazati na displeju pritiskom na taster drugi od dole u trećem stupcu ovog LED_bar-a. Na displeju se tada ispiše i koliko jos km može da se vozi sa trenutnom količinom goriva koja pristiže sa kanala 0 serijske komunikacije. Ukoliko se vrednost otpornosti promeni menja se i vrednost koliko još km može da se vozi i na displeju se ažurira vrednost (displej se osvežava svakih 100ms). Ukoliko se taj taster isključi (drugi od dole u trećem stupcu), displej se isključuje. Zatim pritisnuti taster prvi od dole u trećem stupcu kako bi se na displeju prikazala vrednost nivoa goriva u procentima (levi deo displeja) i trenutna otpornost koja pristiže sa serijske (desni deo displeja). Potom isključiti taj taster i displej se gasi.

