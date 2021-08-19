/* Standard includes. */
#include <stdio.h>
#include <conio.h>
#include <string.h>
#include <stdlib.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "extint.h"

/* Hardware simulator utility functions */
#include "HW_access.h"

/* SERIAL SIMULATOR CHANNEL TO USE */
#define COM_CH (0)
#define COM_CH1 (1)
#define COM_CH2 (2) 

	/* TASK PRIORITIES */
#define OBRADA_TASK_PRI ( tskIDLE_PRIORITY + (UBaseType_t)1 )
#define	TASK_SERIAL_SEND_PRI		( tskIDLE_PRIORITY + (UBaseType_t)3 )
#define TASK_SERIAl_REC_PRI			( tskIDLE_PRIORITY + (UBaseType_t)4 )
#define	SERVICE_TASK_PRI		( tskIDLE_PRIORITY + (UBaseType_t)2 )       


/* TASKS: FORWARD DECLARATIONS */
static void led_bar_tsk(const void* pvParameters); //ocitavanje sa led bara
static void LED_bar_Task1(const void* pvParameters);
static void LED_bar_Task2(const void* pvParameters);//generisanje signala(blinkanje diode)
static void SerialSend_Task(const void* pvParameters); //ispis na serijsku 
static void SerialReceive_Task(void* pvParameters); //prijem komandi sa serijske
static void Primio_kanal_0(const void* pvParameters); //prijem sa senzora 1
static void Primio_kanal_1(const void* pvParameters); //prijem sa senzora 2 
static void Seg7_ispis_task(void* pvParameters); //ispisivanje trazenih informacija na 7-segmentni displej
static void Serijska_stanje_task(void* pvParameters); //redovni ispis stanja sistema na serijsku 
void main_demo(void);
/* TIMER FUNCTIONS*/
static void ispis_tajmer_callback(TimerHandle_t Tmh); 
static void TimerCallback(TimerHandle_t Tmh2);

/* Globalne promjenljive za generalnu upotrebu */
#define R_BUF_SIZE (32)




/* 7-SEG NUMBER DATABASE - ALL HEX DIGITS */
static const uint8_t hexnum[] = { 0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F };

/* GLOBAL OS-HANDLES */
static SemaphoreHandle_t LED_INT_BinarySemaphore;
static SemaphoreHandle_t TBE_BS_0, TBE_BS_1, TBE_BS_2;
static SemaphoreHandle_t RXC_BS_0, RXC_BS_1, RXC_BS_2; 
static SemaphoreHandle_t seg7_ispis;
static SemaphoreHandle_t serijska_stanje; 

static TimerHandle_t per_TimerHandle;
static TimerHandle_t ispis_podaci_tajmer;


static QueueHandle_t seg7_auto_queue;
static QueueHandle_t serijska_ispis_queue;
static QueueHandle_t serijska_ispis_duzina;
static QueueHandle_t queue_senzor1;
static QueueHandle_t queue_senzor2;
static QueueHandle_t queue_kalibracija1;
static QueueHandle_t queue_kalibracija2;
static QueueHandle_t queue_kalibracija3;
static QueueHandle_t queue_kalibracija4;
static QueueHandle_t serijska_prijem_niz;
static QueueHandle_t serijska_prijem_duzina;
static QueueHandle_t stanje_sistema;


static void led_bar_tsk(const void* pvParameters) //ocitati prekidace i reci da li je ukljuceno ili iskljuceno
{

	uint8_t d;
	uint8_t start_local = 0;
	for (;;)
	{
		if (xSemaphoreTake(LED_INT_BinarySemaphore, portMAX_DELAY) != pdTRUE) {
			printf("Greska\n");
		}

		if (xQueueReceive(stanje_sistema, &start_local, pdMS_TO_TICKS(20)) != pdTRUE) {
			printf("Greska\n");
		}


		if (get_LED_BAR(0, &d) != 0) {
			printf("Greska\n");
		} //ocitaj stanje prvog stubca led bara

		if ((d & (uint8_t)0x01) != (uint8_t)0) { //provjeri da li je pritisnut prvi prekidac na led baru, ako jeste, ukljuci sistem, ako nije, iskljucen sistem
			if (set_LED_BAR(1, 0x01) != 0) {
				printf("Greska\n");
			}
			start_local = 1;
		}
		else {
			if (set_LED_BAR(1, 0x00) != 0) {
				printf("Greska\n");
			}
			start_local = 0;
		}

		if (xQueueSend(stanje_sistema, &start_local, 0U) != pdTRUE) {
			printf("Greska\n");
		}

		if (xQueueSend(seg7_auto_queue, &start_local, 0U) != pdTRUE) {
			printf("Greska\n");
		}


	}
}
static void SerialSend_Task(const void* pvParameters)
{ 
	uint8_t t_point = 0;
	uint8_t r[60];
	uint8_t duzina_niza_ispis = 0;

	for (;;)
	{
		if (xSemaphoreTake(TBE_BS_2, portMAX_DELAY) != pdTRUE) {   
			printf("1\n");
		}
		// sacekaj da TX registar bude prazan
 
		if (xQueueReceive(serijska_ispis_queue, &r, pdMS_TO_TICKS(20)) != pdTRUE) {
			printf("2\n");
		}

		//pogledaj da li ima novih vrijednosti (ako nema za 20ms radi dalje)
		if (xQueueReceive(serijska_ispis_duzina, &duzina_niza_ispis, pdMS_TO_TICKS(20)) != pdTRUE) {
			printf("3\n");
		}

		//pogledaj ima li sta novo (ako nema za 20ms radi dalje)

		if (t_point < duzina_niza_ispis) { //dok nije ispisan posljednji karakter salji slovo po slovo 
			if (send_serial_character(COM_CH2, r[t_point++]) != 0) {
				printf("Slanje karaktera neispravno\n");
			}
			

		}
		else { //kada se ispise posljednji karakter, onda resetuj sistem i daj semafor da je ispis rijeci zavrsen
			t_point = 0;
			duzina_niza_ispis = 0;
		}
	}
}

//Void LED_bar_Task1 se koristi za blinkanje diodoma, u zavisnosti od vrednosti KALIBRACIJE 1 tj. zone detekcije 
static void LED_bar_Task1(const void* pvParameters) {
	double kalibracija3_local = 0;// kalibracija1 <=> kalibracija3 , kalibracija 2 <=> kalibracija4
								   // u ove dve promenljive smestamo kalibrisane vrednosti sa senzora
	uint8_t i_local = 0; // pomocna promenljiva pomocu koje ogranicava slucaj kada nijedan uslov nije ispunjen tj. ZONA DETEKCIJE                                        
						 // da nam se na pocetku to ispise samo jednom na terminalu, a ne nonstop da se ispisuje
	for (;;) {

		if (xQueueReceive(queue_kalibracija3, &kalibracija3_local, pdMS_TO_TICKS(20)) != 0) { // smestanje kalibrasane vrednosti sa senzora 1 
			printf("kali3\n");
		}

		if (kalibracija3_local > (double)50 && kalibracija3_local <= (double)100) {
			printf("LEVI SENZOR: DALEKA DETEKCIJA\n");// Generisemo signal frekvencije 1Hz, polovinu periode ce svetleti gornje
			if (set_LED_BAR(2, 0xF0) != 0) {// cetiri diode treceg stubca, a zatim pola periode ce biti ugasene
				printf("Greska\n");
			}
			vTaskDelay(pdMS_TO_TICKS(500));              // time dobijamo blinkanje frekvencijom 1Hz, ukoliko se objekat detektovan
			if (set_LED_BAR(2, 0x00) != 0) {                        // senzorom 1 nalazi u zoni UDALJENA_DETEKCIJA 
				printf("Greska\n");
			}
			vTaskDelay(pdMS_TO_TICKS(500));
			i_local = 0;
		}

		else if (kalibracija3_local > (double)0 && kalibracija3_local <= (double)50) {
			printf("LEVI SENZOR: BLISKA DETEKCIJA\n");// Generisemo signal frekvencije 2Hz, polovinu periode ce svetleti gornje
			if (set_LED_BAR(2, 0xF0) != 0) { // cetiri diode treceg stubca, a zatim pola periode ce biti ugasene
				printf("Greska\n");
			}
			vTaskDelay(pdMS_TO_TICKS(250));                // time dobijamo blinkanje frekvencijom 2Hz, ukoliko se objekat detektovan
			if (set_LED_BAR(2, 0x00) != 0) {                          // senzorom 1 nalazi u zoni BLISKA_DETEKCIJA 
				printf("Greska\n");
			}
			vTaskDelay(pdMS_TO_TICKS(250));
			i_local = 0;
		}

		else if (kalibracija3_local < (double)0) {
			printf("LEVI SENZOR: KONTAKT DETEKCIJA\n");// Generisemo signal frekvencije 2Hz, celu periodu se svetleti gornje
			if (set_LED_BAR(2, 0xF0) != 0) {                           // cetiri diode treceg stubca, ukoliko se objekat detektovan senzorom 1
				printf("Greska\n");
			}
			vTaskDelay(pdMS_TO_TICKS(500));                 // nalazi u zoni KONTAKT_DETEKCIJA
			if (set_LED_BAR(2, 0x00) != 0) {
				printf("Greska\n");
			}
			i_local = 0;
		}
		else if (i_local == (uint8_t)0) {
			printf("LEVI SENZOR: NEMA DETEKCIJE\n");
			i_local = 1;
		}
		

	}

}

static void LED_bar_Task2(const void* pvParameters) {
	double kalibracija4_local = 0;
	uint8_t i_local = 0; // pomocna promenljiva pomocu koje ogranicava slucaj kada nijedan uslov nije ispunjen tj. ZONA DETEKCIJE                                        
						 // da nam se na pocetku to ispise samo jednom na terminalu, a ne nonstop da se ispisuje

	for (;;) {
		if (xQueueReceive(queue_kalibracija4, &kalibracija4_local, pdMS_TO_TICKS(20)) != 0) {
			printf("kali4\n");
		}
		
		



		if (kalibracija4_local > (double)50 && kalibracija4_local <= (double)100) {
			printf("DESNI SENZOR: DALEKA DETEKCIJA\n");      // Generisemo signal frekvencije 1Hz, polovinu periode ce svetleti gornje
			if (set_LED_BAR(3, 0xF0) != 0) {                          // cetiri diode treceg stubca, a zatim pola periode ce biti ugasene             
				printf("Greska\n");
			}
			vTaskDelay(pdMS_TO_TICKS(500));                // time dobijamo blinkanje frekvencijom 1Hz, ukoliko se objekat detektovan
			if (set_LED_BAR(3, 0x00) != 0) { // senzorom 2 u zoni DALEKA_DETEKCIJA
				printf("Greska\n");
			}
			vTaskDelay(pdMS_TO_TICKS(500));
			i_local = 0;
		}

		else if (kalibracija4_local > (double)0 && kalibracija4_local <= (double)50) {
			printf("DESNI SENZOR: BLISKA DETEKCIJA\n");	   // Generisemo signal frekvencije 2Hz, polovinu periode ce svetleti gornje
			if (set_LED_BAR(3, 0xF0) != 0) {                          // cetiri diode treceg stubca, a zatim pola periode ce biti ugasene
				printf("Greska\n");
			}
			vTaskDelay(pdMS_TO_TICKS(250));                // time dobijamo blinkanje frekvencijom 2Hz, ukoliko se objekat detektovan
			if (set_LED_BAR(3, 0x00) != 0) {                          // senzorom 2 u zoni BLISKA_DETEKCIJA
				printf("Greska\n");
			}
			vTaskDelay(pdMS_TO_TICKS(250));
			i_local = 0;
		}

		else if (kalibracija4_local < (double)0) {                       // Generisemo signal frekvencije 2Hz, celu periodu se svetleti gornje
			printf("DESNI SENZOR: KONTAKT DETEKCIJA\n");
			if (set_LED_BAR(3, 0xF0) != 0) {                          // cetiri diode treceg stubca, ukoliko se objekat detektovan senzorom 2
				printf("Greska\n");
			}
			vTaskDelay(pdMS_TO_TICKS(500));                // nalazi u zoni KONTAKT_DETEKCIJA
			if (set_LED_BAR(3, 0x00) != 0) {
				printf("Greska\n");
			}
			i_local = 0;
		}
		else if (i_local == (uint8_t)0) {
			printf("DESNI SENZOR: NEMA DETEKCIJE\n");
			i_local = 1;
		}
		
		
	}
}

static void Primio_kanal_0(const void* pvParameters) //prijem sa kanala 0 (senzor 1)
{

	double senzor1 = 0;   // promeljiva u koju smestamo vrednosti primljene sa kanala 0, ali konverovane u float
	uint8_t cc = 0;   // prvo se primljeno sa kanala nula smesta u ovu promenljivu
	uint8_t br_karaktera = 0; //sluzi nam da se pomeramo kroz niz rastojanje_kanal0[6], ako je stigao karakter za kraj poruke, resetuje se na 0
	double kalibracija1_local = 0;  // promenljiva u koju smestamo kalibrisanu vrednost primljenu sa kanala 0
	uint8_t rastojanje_kanal0[6] = { 0 }; // rastajoanje sa senzora 1
	double min = 20, max = 100; // minimalne i maksimalne vrednosti, potrebne za kalibraciju

	for (;;) {
		if (xSemaphoreTake(RXC_BS_0, portMAX_DELAY) != pdTRUE) {
			printf("6\n");
		}

		// uzima semafor
		if (get_serial_character(COM_CH, &cc) != 0) {
			printf("Greska\n");
		}

			
		

		// smesta karaktere primljene sa kanala 0 u promenljivu cc
		//printf("primio kanal 0 %u\n", (unsigned)cc);
		if (cc == (uint8_t)0x0d) {  // ako je u promeljivu cc stigao karakter 0x0d, to je signal za kraj poruke i vrednost dobijena sa kanala 0 se moze obradjivati
			senzor1 = atof(rastojanje_kanal0); // pomocu funkcije atof, pretvaramo string u float
			br_karaktera = 0; // resetujemo broj karaktera na 0, da bi mogli uspesno da obradjuemo naredne poruke
			if (xQueueSend(queue_senzor1, &senzor1, 0U) != pdTRUE) {
				printf("7\n");
			}

			// vrednosti sa kanala 0, konvertovane u float, smestano u red queue_senzor1
			kalibracija1_local = (double)100 * (senzor1 - min) / (max - min);  // racunamo kalibraciju, i smestamo je u promenljivu kalibracija1
			if (xQueueSend(queue_kalibracija1, &kalibracija1_local, 0U) != pdTRUE) {
				printf("8\n");
			}

			// vrednost kalibracije 1 saljemo u queue_kalibracija1, ovaj queue kasnije receivujemo u Serijska_stanje_task da bi mogli na serijskoj da ispisujemo na serijskoj trenutno stanje kalibracije1
			if (xQueueSend(queue_kalibracija3, &kalibracija1_local, 0U) != pdTRUE) {
				printf("9\n");
			}

			// ovaj queue receivujemo u tasku LED_bar_Task, koji nam sluzi za generisanje signala(blinkanje dioda odredjenom frekvecijom)
		}
		else {
			rastojanje_kanal0[br_karaktera++] = cc; // redom iz cc smestamo karakter po karakter u niz rastojanje_kanala dok ne stigne 0x0d 
		}


	}
}

static void Primio_kanal_1(const void* pvParameters) //POTUPNO IDENTICNA PRICA KAO TASK Primio_kanal_0!!! Samo ovde sa kanala 1, simuliramo senzor 2!!!
{
	double senzor2 = 0;
	uint8_t cc = 0;
	double kalibracija2_local = 0;
	double min = 20, max = 100;
	uint8_t br_karaktera = 0;
	uint8_t rastojanje_kanal1[6] = { 0 };

	for (;;) {
		if (xSemaphoreTake(RXC_BS_1, portMAX_DELAY) != pdTRUE) {
			printf("10\n");
		}


		if (get_serial_character(COM_CH1, &cc) != 0) {
			printf("Greska\n");
		}
		

		
		if (cc == (uint8_t)0x0d) {
			senzor2 = atof(rastojanje_kanal1);
			if (xQueueSend(queue_senzor2, &senzor2, 0U) != pdTRUE) {
				printf("11\n");
			}

			br_karaktera = 0;
			kalibracija2_local = (double)100 * (senzor2 - min) / (max - min);
			if (xQueueSend(queue_kalibracija2, &kalibracija2_local, 0U) != pdTRUE) {
				printf("12\n");
			}

			if (xQueueSend(queue_kalibracija4, &kalibracija2_local, 0U) != pdTRUE) {
				printf("13\n");
			}


		}
		else {
			rastojanje_kanal1[br_karaktera++] = cc;
		}

	}
}

static void SerialReceive_Task(void* pvParameters) //kanal 2, prima komandnu rijec koja se zavrsava karakterom 13(0x0d)
{
	uint8_t r_point = 0;  //za pomeranje kroz niz r_buffer
	uint8_t r_buffer[12]; //smestamo primljeno komandu
	uint8_t start_local = 0; //treba nam za START/STOP 
	uint8_t startovanje = 0; // pomocna promeljiva, mislim da nije potrebna, al nesto kao da bez nje nije htelo lepo da radi NEMAM OBJASNJENJE
	uint8_t cc = 0;  // ovde se direktno iz kanala smesta primljemo iz kanala0, i karakter po karakter posme smesta u r_buffer
	uint8_t duzina_primljene_rijeci = 0; //duzinu primljene reci smestamo ovde, da kasnije mozemo proveriti koja je rec stigla

	for (;;)
	{
		if (xSemaphoreTake(RXC_BS_2, portMAX_DELAY) != pdTRUE) {
			printf("14\n");
		}

		// ceka na serijski prijemni interapt

		if (get_serial_character(COM_CH2, &cc) != 0) {
			printf("Greska\n");
		}
		

		//ucitava primljeni karakter u promenjivu cc        

		if (xQueueReceive(stanje_sistema, &startovanje, pdMS_TO_TICKS(20)) != pdTRUE) {
			printf("15\n");
		}

		// ovo nam treba zbog START/STOP

		start_local = startovanje;

		if (cc == (uint8_t)0x0d) // oznaciti kraj poruke i ako je kraj, preko reda poslati informacije o poruci i restartovati ovaj taks
		{
			duzina_primljene_rijeci = r_point;
			if (xQueueSend(serijska_prijem_niz, &r_buffer, 0U) != pdTRUE) {
				printf("16\n");
			}

			// saljemo niz u queue koji kasnije koristimo za ispis
			if (xQueueSend(serijska_prijem_duzina, &r_point, 0U) != pdTRUE) {
				printf("17\n");
			}

			// saljemo duzinu tog niza, isto nam treba za ispis
			r_point = 0; // reset na nulu, da bi mogli uspesno ponovo da obradjujemo primljene poruke
		}
		else if (r_point < (uint8_t)R_BUF_SIZE)// pamti karaktere prije FF 
		{
			r_buffer[r_point++] = cc; // sve iz cc smestamo u niz r_buffer
		}
		//ovim if-om proveravamo da li je stigla prava rec, takodje proveravamo da li sistem vec upaljen,
		//ako jeste da se ne pali opet(apsurd), zato u uslovu stoji da je start==0, jer sistem treba da je prvo logicno UGASEN da bi se 
		//upalio
		if ((duzina_primljene_rijeci == (uint8_t)sizeof("START") - (uint8_t)1) && (strncmp(r_buffer, ("START"), duzina_primljene_rijeci) == (uint8_t)0) && start_local == (uint8_t)0) {

			if (set_LED_BAR(1, 0x01) != 0) {// palimo indikacionu diodu, drugi stubac, prva od dole.
				printf("Greska\n");
			}
			printf("Dobro uneseno START \n");//ispisujemo na terminal da je korisnik uneo dobru komandu
			start_local = 1; // palimo sistem
		}

		//Slicna prica i sa ovim else if, samo se ovde proverava stop, i proveramo da li je sistem upaljen da bi ga gasili

		else if ((duzina_primljene_rijeci == (uint8_t)sizeof("STOP") - (uint8_t)1) && (strncmp(r_buffer, ("STOP"), duzina_primljene_rijeci) == (uint8_t)0) && start_local == (uint8_t)1) {

			if (set_LED_BAR(1, 0x00) != 0) {//gasimo indikacionu diodu
				printf("Greska\n");
			}
			printf("Dobro uneseno STOP \n");//obavestavamo korisnika da je ukucao dobru komandu
			start_local = 0; // gasimo sistem

		}

if (xQueueSend(stanje_sistema, &start_local, 0U) != pdTRUE) {
	printf("18\n");
}

// saljemo u queue stanje_sistema koja je komanda aktivirana, ovaj queue sluzi za task Serijska_stanje_task
if (xQueueSend(seg7_auto_queue, &start_local, 0U) != pdTRUE) {
	printf("19\n");
}

// saljemo u queue seg7_auto_queue koja je komanda aktivirana, ovaj queue sluzi za Seg7_ispis(nisam siguran da je to bas ime taska)
//razlog zasto imam dva reda, za istu stvar, je sinhronizacija izmedju taskova koja se poremeti ako koristim isti queue.
	}
}



static void Seg7_ispis_task(void* pvParameters) { // TASK ZA ISPIS NA SEG7 DISPLEJU
										   // NA SEG7 DISPLEJU ISPISUJEMO, NA PRVOM SEGMETU 0(AKO JE SISTEM UGASEN), ILI 1 (AKO JE SISTEM AKTIVIRAN)
											// SLEDECA TRI SEGMENTA, ISPISUJEMO VREDNOST SA SENZORA 1(NEKALIBRISANU)
											// SLEDECA TRI SEGMENTA, ISPISUJEMO VREDNOST SA SENZORA 2

	double senzor1_local = 0, senzor2_local = 0;  // promenljive za smestanje vrednost sa senzora
	uint8_t start_local = 0; // promeljiva za smestanje vrednosti start/stop

	for (;;) {
		if (xSemaphoreTake(seg7_ispis, portMAX_DELAY) != pdTRUE) {
			printf("20\n");
		}

		// ceka semafor, osvezava se displej svakih 200ms

		if (xQueueReceive(seg7_auto_queue, &start_local, pdMS_TO_TICKS(20)) != pdTRUE) {
			printf("seg7_1\n");
		}
		

		// start/stop komanda se risivuje 
		if (xQueueReceive(queue_senzor1, &senzor1_local, pdMS_TO_TICKS(20)) != pdTRUE) {
			printf("seg7_2\n");
		}


		// vrednost sa senzora 1 se risivuje
		if (xQueueReceive(queue_senzor2, &senzor2_local, pdMS_TO_TICKS(20)) != pdTRUE ) {
			printf("seg7_3\n");
		}


		// vrednost sa senzora 2 se risivuje

		if (start_local == (uint8_t)1) { //na prvu cifru ispisuje 1 ako je rezim rada start, a 0 ako je stop
			if (select_7seg_digit(0) != 0) {
				printf("Problem\n");
			}
			if (set_7seg_digit(hexnum[1]) != 0) {
				printf("Problem\n");
			}

			if (select_7seg_digit(1) != 0) {
			printf("Problem\n");
			}

			if (set_7seg_digit(hexnum[(uint8_t)senzor1_local / (uint8_t)100]) != 0) {
			printf("Problem\n");
			}
			if (select_7seg_digit(2) != 0) {
			printf("Problem\n");
			}
			if (set_7seg_digit(hexnum[((uint8_t)senzor1_local / (uint8_t)10) % (uint8_t)10]) != 0) {
			printf("Problem\n");
			}
			if (select_7seg_digit(3) != 0) {
			printf("Problem\n");
			}
			if (set_7seg_digit(hexnum[(uint8_t)senzor1_local % (uint8_t)10]) != 0) {
			printf("Problem\n");
			}
			if (select_7seg_digit(4) != 0) {
				printf("Problem\n");
			}
			if (set_7seg_digit(hexnum[(uint8_t)senzor2_local / (uint8_t)100]) != 0) {
			printf("Problem\n");
			}
			if (select_7seg_digit(5) != 0) {
				printf("Problem\n");
			}
			if (set_7seg_digit(hexnum[((uint8_t)senzor2_local / (uint8_t)10) % (uint8_t)10]) != 0) {
				printf("Problem\n");
			}
			if (select_7seg_digit(6) != 0) {
				printf("Problem\n");
			}
			if (set_7seg_digit(hexnum[(uint8_t)senzor2_local % (uint8_t)10]) != 0) {
			printf("Problem\n");
			}

		}

		else {
			if (select_7seg_digit(0) != 0) {
				printf("Problem\n");
			}

			if (set_7seg_digit(hexnum[0]) != 0){
			printf("Problem\n");
			}
			if (select_7seg_digit(1) != 0){
				printf("Problem\n");
			}
			if (set_7seg_digit(hexnum[0]) != 0) {
				printf("Problem\n");
			}

			if (select_7seg_digit(2) != 0) {
			printf("Problem\n");
			}

			if (set_7seg_digit(hexnum[0]) != 0) {
			printf("Problem\n");
			}

			if (select_7seg_digit(3) != 0) {
			printf("Problem\n");
			}

			if (set_7seg_digit(hexnum[0]) != 0) {
			printf("Problem\n");
			}

			if (select_7seg_digit(4) != 0) {
			printf("Problem\n");
			}

			if (set_7seg_digit(hexnum[0]) != 0) {
			printf("Problem\n");
			}

			if (select_7seg_digit(5) != 0) {
			printf("Problem\n");
			}

			if (set_7seg_digit(hexnum[0]) != 0) {
			printf("Problem\n");
			}

			if (select_7seg_digit(6) != 0) {
			printf("Problem\n");
			}

			if (set_7seg_digit(hexnum[0]) != 0) {
			printf("Problem\n");
			}
			
			
		}

	}

	
}

// OVAJ POSLEDNJI TASK MOZDA I NIJE POTREBAN, U NJEMU FAKTICKI PRIPREMAMO PORUKU I SALJEMO JE PREKO QUEUE-A TASKU KOJI CE JE ISPISATI GDE TREBA
// ALI I SA OVIM TASKOM SISTEM RADI KO DOXA
static void Serijska_stanje_task(void* pvParameters) { /*formiramo niz za redovan ispis stanja sistema i saljemo pomocu reda poruku i duzinu poruke
												tasku za ispis na serijsku*/
	uint8_t pomocni_niz[60] = { 0 }; // POMOCNI NIZ U KOJI SMESTAMO KOMPLETNU PORUKU ZA SLANJE NA SERIJSKU KANALA 2
	uint8_t duzina_niza_ispis = 0; // POMOCNA PROMENLJIVA POMOCU KOJE SALJEMO DUZINU TOG NIZA
	uint8_t start_local = 0; // START/STOP
	double kalibracija1_local = 0, kalibracija2_local = 0;  // PROMELJIVE U KOJE CEMO DA RISIVUJEMO VREDNOSTI KALIBRACIJE

	for (;;) {
		if (xSemaphoreTake(serijska_stanje, portMAX_DELAY) != pdTRUE) {
			printf("24\n");
		}

		// UZIMA SEMAFOR SVAKIH 5 SEKUNDI 

		if (xQueueReceive(queue_kalibracija1, &kalibracija1_local, pdMS_TO_TICKS(20)) != pdTRUE) {
			printf("kali1\n");
		}
	
		// RISIVUJE VREDNOST KALIBRACIJE 1
		if (xQueueReceive(queue_kalibracija2, &kalibracija2_local, pdMS_TO_TICKS(20)) != pdTRUE) {
			printf("kali2\n");
		}
		

		// RISIVUJE VREDNOST KALIBRACIJE 2
		if (xQueueReceive(stanje_sistema, &start_local, pdMS_TO_TICKS(20)) != pdTRUE) { // RISIVUJE KOMANDU START/STOP
			printf("stanje\n");
		}
		strcpy(pomocni_niz, "Stanje: "); // PRVO U NIZ pomocni_niz SMESTAMO(KOPIRAMO) "Stanje:" 
		duzina_niza_ispis = (uint8_t)sizeof("Stanje: ") - (uint8_t)1; // OVDE U PROMENLJIVU DUZINA_NIZA_ISPIS SMESTAMO KOLIKO JE TRENUTNO DUGACAK NIZ POMOCNI_NIZ

		if (start_local == (uint8_t)1) { //AKO JE AKTIVAN START NA Stanje: nadovezi START
			strcat(pomocni_niz, "START");//POMOCU FUNKCIJE strcat mi cemo "START" da 'priljubimo' uz Stanje: i to ce izgledati na terminalu Stanje:START
			duzina_niza_ispis += (uint8_t)sizeof("START") - (uint8_t)1; //u duzinu_niza_ispis sumiramo i broj karaktera komande START
		}
		else {
			strcat(pomocni_niz, "STOP"); // SLICNO KAO IZNAD, SAMO JE U PITANJU STOP
			duzina_niza_ispis += (uint8_t)sizeof("STOP") - (uint8_t)1;
		}

		if (start_local == (uint8_t)1) {// SLUZI NAM ZA PRIKAZIVANJE KALIBRACIJE

			strcat(pomocni_niz, ", K1:"); //U POMOCNI NIZ PRILJUBIMO OVO
			duzina_niza_ispis += (uint8_t)sizeof(", K1:") - (uint8_t)1; // SUMIRAMO U DUZINU
			pomocni_niz[duzina_niza_ispis++] = (uint8_t)kalibracija1_local / (uint8_t)100 + (uint8_t)'0'; // I ONDA SE REDOM POMERAMO PO POMOCNOM NIZU I SMESTAMO CIFRU PO CIFRU KALIRBACIJE 1
			pomocni_niz[duzina_niza_ispis++] = (((uint8_t)kalibracija1_local / (uint8_t)10) % (uint8_t)10) + (uint8_t)'0';
			pomocni_niz[duzina_niza_ispis++] = (uint8_t)kalibracija1_local % (uint8_t)10 + (uint8_t)'0';


			strcat(pomocni_niz, ", K2:"); // ISTA PRICA KAO ZA K1
			duzina_niza_ispis += (uint8_t)sizeof(", K2:") - (uint8_t)1;
			pomocni_niz[duzina_niza_ispis++] = (uint8_t)kalibracija2_local / (uint8_t)100 + (uint8_t)'0';
			pomocni_niz[duzina_niza_ispis++] = (((uint8_t)kalibracija2_local / (uint8_t)10) % (uint8_t)10) + (uint8_t)'0';
			pomocni_niz[duzina_niza_ispis++] = (uint8_t)kalibracija2_local % (uint8_t)10 + (uint8_t)'0';

		}

		if (xQueueSend(serijska_ispis_queue, &pomocni_niz, 0U) != pdTRUE) {
			printf("28\n");
		}

		if (xQueueSend(serijska_ispis_duzina, &duzina_niza_ispis, 0U) != pdTRUE) {
			printf("29\n");
		}

		if (send_serial_character(COM_CH2, 13) != 0) {
			printf("Greska\n");
		}


	}
}

/* OPC - ON INPUT CHANGE - INTERRUPT HANDLER */      // POGLEDATI NA FREERTOS.ORG
static uint32_t OnLED_ChangeInterrupt(void)
{
	BaseType_t higher_priority_task_woken = pdFALSE;

	if (xSemaphoreGiveFromISR(LED_INT_BinarySemaphore, &higher_priority_task_woken) != pdTRUE) {

		printf("Greska\n");
	}

	portYIELD_FROM_ISR((uint32_t)higher_priority_task_woken);
}


/* TBE - TRANSMISSION BUFFER EMPTY - INTERRUPT HANDLER */
static uint32_t prvProcessTBEInterrupt(void)
{
	BaseType_t higher_priority_task_woken = pdFALSE;

	if (get_TBE_status(0) != 0) {
		if (xSemaphoreGiveFromISR(TBE_BS_0, &higher_priority_task_woken) != pdTRUE) {

			printf("Greska\n");
		}
	}
	if (get_TBE_status(1) != 0) {
		if (xSemaphoreGiveFromISR(TBE_BS_1, &higher_priority_task_woken) != pdTRUE) {

			printf("Greska\n");
		}
	}
	if (get_TBE_status(2) != 0) {
		if (xSemaphoreGiveFromISR(TBE_BS_2, &higher_priority_task_woken) != pdTRUE) {

			printf("Greska\n");
		}
	}
	
	portYIELD_FROM_ISR((uint32_t)higher_priority_task_woken);
}


/* RXC - RECEPTION COMPLETE - INTERRUPT HANDLER */
static uint32_t prvProcessRXCInterrupt(void)
{
	BaseType_t higher_priority_task_woken = pdFALSE;

	if (get_RXC_status(0) != 0) {
		if (xSemaphoreGiveFromISR(RXC_BS_0, &higher_priority_task_woken) != pdTRUE) {

			printf("Greska\n");
		}
	}
	
	if (get_RXC_status(1) != 0) {
		if (xSemaphoreGiveFromISR(RXC_BS_1, &higher_priority_task_woken) != pdTRUE) {

			printf("Greska\n");
		}
	}

	if (get_RXC_status(2) != 0) {
		if (xSemaphoreGiveFromISR(RXC_BS_2, &higher_priority_task_woken) != pdTRUE) {

			printf("Greska\n");
		}
	}


	portYIELD_FROM_ISR((uint32_t)higher_priority_task_woken);
}


/* PERIODIC TIMER CALLBACK */
static void TimerCallback(TimerHandle_t per_TimerHandle)
{

	if (xSemaphoreGive(seg7_ispis) != pdTRUE) {
		printf("30\n");
	}
	
	//osvezavanje displeja svakim 80ms

}


static void ispis_tajmer_callback(TimerHandle_t ispis_podaci_tajmer)
{

	if (xSemaphoreGive(serijska_stanje) != pdTRUE) {
		printf("31\n");
	}
	
	    // ispis na terminalu kanala 2 svakih 5000ms

}

/* MAIN - SYSTEM STARTUP POINT */
void main_demo(void)
{
	/* Inicijalizacija periferija */
	if (init_7seg_comm() != 0) {
		
		printf("Neuspesna inicijalizacija\n");
	}

	if (init_LED_comm() != 0) {

		printf("Neuspesna inicijalizacija\n");
	}

	if (init_serial_uplink(COM_CH) != 0) {

		printf("Neuspesna inicijalizacija\n");
	}
	
	if (init_serial_downlink(COM_CH) != 0) {

		printf("Neuspesna inicijalizacija\n");
	}

	if (init_serial_uplink(COM_CH1) != 0) {

		printf("Neuspesna inicijalizacija\n");
	}

	if (init_serial_downlink(COM_CH1) != 0) {

		printf("Neuspesna inicijalizacija\n");
	}

	if (init_serial_uplink(COM_CH2) != 0) {

		printf("Neuspesna inicijalizacija\n");
	}

	if (init_serial_downlink(COM_CH2) != 0) {

		printf("Neuspesna inicijalizacija\n");
	}
	

	/* ON INPUT CHANGE INTERRUPT HANDLER */
	vPortSetInterruptHandler(portINTERRUPT_SRL_OIC, OnLED_ChangeInterrupt);

	/* Create LED interrapt semaphore */
	LED_INT_BinarySemaphore = xSemaphoreCreateBinary();
	if (LED_INT_BinarySemaphore == NULL) {
		printf("Greska prilikom kreiranja");
	}
	

	/* create a timer task */
	per_TimerHandle = xTimerCreate("Timer", pdMS_TO_TICKS(80), pdTRUE, NULL, TimerCallback);

	if (per_TimerHandle != pdPASS) {
		printf("Greska prilikom kreiranja\n");
	}
	if (xTimerStart(per_TimerHandle, 0) != pdPASS) {
		printf("Greska prilikom kreiranja\n");
	}
	
	
	
	ispis_podaci_tajmer = xTimerCreate("Timer2", pdMS_TO_TICKS(5000), pdTRUE, NULL, ispis_tajmer_callback);

	if (ispis_podaci_tajmer != pdPASS) {
		printf("Greska prilikom kreiranja\n");
	}
	if (xTimerStart(ispis_podaci_tajmer, 0) != pdPASS) {
		printf("Greska prilikom kreiranja\n");
	}
	
	
	/* SERIAL TRANSMITTER TASK */
	BaseType_t status1;
	status1 = xTaskCreate(SerialSend_Task, "STx", configMINIMAL_STACK_SIZE, NULL, TASK_SERIAL_SEND_PRI, NULL);
	if (status1 != pdPASS) {
		printf("Greska prilikom kreiranja\n");
	}
	/* SERIAL RECEIVER TASK */
	BaseType_t status2;
	status2 = xTaskCreate(SerialReceive_Task, "SRx", configMINIMAL_STACK_SIZE, NULL, TASK_SERIAl_REC_PRI, NULL);
	if (status2 != pdPASS) {
		printf("Greska prilikom kreiranja");
	}
	/* Create TBE semaphore - serial transmit comm */
	TBE_BS_0 = xSemaphoreCreateBinary();
	if (TBE_BS_0 == NULL)
	{
		printf("Greska prilikom kreiranja\n");
	}
	TBE_BS_1 = xSemaphoreCreateBinary();
	if (TBE_BS_1 == NULL)
	{
		printf("Greska prilikom kreiranja\n");
	}
	TBE_BS_2 = xSemaphoreCreateBinary();
	if (TBE_BS_2 == NULL)
	{
		printf("Greska prilikom kreiranja\n");
	}

	/* Create RXC semaphore - serial transmit comm */
	RXC_BS_0 = xSemaphoreCreateBinary();
	if (RXC_BS_0 == NULL)
	{
		printf("Greska prilikom kreiranja\n");
	}
	RXC_BS_1 = xSemaphoreCreateBinary();
	if (RXC_BS_1 == NULL)
	{
		printf("Greska prilikom kreiranja\n");
	}
	RXC_BS_2 = xSemaphoreCreateBinary();
	if (RXC_BS_2 == NULL)
	{
		printf("Greska prilikom kreiranja\n");
	}

	/* Ostali semafori */
	seg7_ispis = xSemaphoreCreateBinary(); // KREIRAM SEMAFOR KOJI SE POSLE TAJMEROM GIVUJE SVAKIM 200mS TASKU SEG7 DISPLEJ, OSVEZAVAMO DISPEL SVAKIH 200mS
	if (seg7_ispis == NULL)
	{
		printf("Greska prilikom kreiranja\n");
	}
	serijska_stanje = xSemaphoreCreateBinary(); // SEMAFOR KOJI SE GIVUJE SVAKIH 5 SEKUNDI ZA ISPIS STANJA NA KANALU 2
	if (serijska_stanje == NULL)
	{
		printf("Greska prilikom kreiranja\n");
	}
	/* SERIAL TRANSMISSION INTERRUPT HANDLER */
	vPortSetInterruptHandler(portINTERRUPT_SRL_TBE, prvProcessTBEInterrupt);

	/* SERIAL RECEPTION INTERRUPT HANDLER */
	vPortSetInterruptHandler(portINTERRUPT_SRL_RXC, prvProcessRXCInterrupt);

	/* Kreiranje redova za komunikaciju izmedju taskova */

	seg7_auto_queue = xQueueCreate(2, sizeof(uint8_t)); // SMESTAMO KOMANDU START/STOP
	if (seg7_auto_queue == NULL)
	{
		printf("Greska prilikom kreiranja\n");
	}
	serijska_ispis_queue = xQueueCreate(3, sizeof(uint8_t[60])); //red za skladistenje poruke za ispis
	if (serijska_ispis_queue == NULL)
	{
		printf("Greska prilikom kreiranja\n");
	}
	serijska_ispis_duzina = xQueueCreate(3, sizeof(uint8_t)); //red za skladistenje duzine rijeci
	if (serijska_ispis_duzina == NULL)
	{
		printf("Greska prilikom kreiranja\n");
	}
	
	serijska_prijem_niz = xQueueCreate(3, sizeof(uint8_t[12])); //red za skladistenje primljene rijeci (komande)
	if (serijska_prijem_niz == NULL)
	{
		printf("Greska prilikom kreiranja\n");
	}
	
	serijska_prijem_duzina = xQueueCreate(3, sizeof(uint8_t)); //red za skladistenje duzine primljene rijeci
	if (serijska_prijem_duzina == NULL)
	{
		printf("Greska prilikom kreiranja\n");
	}

	/*QUEUE-OVI ZA SKLADISTANJE VREDNOSTI SA SENZORA I KALIBRACIJU
	RAZLOG ZASTO IMAM PO DVA QUEUE-A ZA KALIBRACIJE JE U SINHORONIZACIJI TASKOVA*/
	queue_senzor1 = xQueueCreate(2, sizeof(double)); //red za primanje vrijednosti sa senzora 1
	if (queue_senzor1 == NULL)
	{
		printf("Greska prilikom kreiranja\n");
	}
	
	queue_senzor2 = xQueueCreate(2, sizeof(double)); //red za primanje vrijednosti sa senzora 2
	if (queue_senzor2 == NULL)
	{
		printf("Greska prilikom kreiranja\n");
	}
	
	queue_kalibracija1 = xQueueCreate(2, sizeof(double));//red za primenje kalibrisane vrednosti sa senzora 1
	if (queue_kalibracija1 == NULL)
	{
		printf("Greska prilikom kreiranja\n");
	}
	
	queue_kalibracija2 = xQueueCreate(2, sizeof(double));//red za primenje kalibrisane vrednosti sa senzora 2
	if (queue_kalibracija2 == NULL)
	{
		printf("Greska prilikom kreiranja\n");
	}
	
	queue_kalibracija3 = xQueueCreate(2, sizeof(double));//red za primenje kalibrisane vrednosti sa senzora 1
	if (queue_kalibracija3 == NULL)
	{
		printf("Greska prilikom kreiranja\n");
	}
	
	queue_kalibracija4 = xQueueCreate(2, sizeof(double));////red za primenje kalibrisane vrednosti sa senzora 2
	if (queue_kalibracija4 == NULL)
	{
		printf("Greska prilikom kreiranja\n");
	}

	/*QUEUE U KOJI SE SMEŠTAJU START/STOP KOMANDE*/
	stanje_sistema = xQueueCreate(1, sizeof(uint8_t));//PRIMA 0 ILI 1 U ZAVISNOSTI DA LI SISTEM AKTIVAN (1) ILI UGAŠEN (0)
	if (stanje_sistema == NULL)
	{
		printf("Greska prilikom kreiranja\n");
	}
	// KREIRAMO TASKOVE
	BaseType_t status;
	status = xTaskCreate(led_bar_tsk, "ST", configMINIMAL_STACK_SIZE, NULL, (UBaseType_t)SERVICE_TASK_PRI, NULL);// TASK ZA PROVERU DA LI JE PRITISNUT PREKIDAČ
	if (status != pdPASS) {
		printf("Greska prilikom kreiranja\n");
	}
	status = xTaskCreate(Primio_kanal_0, "kanal0", configMINIMAL_STACK_SIZE, NULL, (UBaseType_t)TASK_SERIAl_REC_PRI, NULL);//TASK ZA PRIJEM SA KANALA 0
	if (status != pdPASS) {
		printf("Greska prilikom kreiranja\n");
	}
	status = xTaskCreate(Primio_kanal_1, "kanal1", configMINIMAL_STACK_SIZE, NULL, (UBaseType_t)TASK_SERIAl_REC_PRI, NULL);//TASK ZA PRIJEM SA KANALA 1
	if (status != pdPASS) {
		printf("Greska prilikom kreiranja\n");
	}
	status = xTaskCreate(Seg7_ispis_task, "Seg_7", configMINIMAL_STACK_SIZE, NULL, (UBaseType_t)SERVICE_TASK_PRI, NULL);//TASK ZA ISPIS NA SEG7 DISPLEJ
	if (status != pdPASS) {
		printf("Greska prilikom kreiranja\n");
	}
	status = xTaskCreate(Serijska_stanje_task, "Stanje", configMINIMAL_STACK_SIZE, NULL, (UBaseType_t)OBRADA_TASK_PRI, NULL);// TASK ZA ISPIS NA KANAL 2
	if (status != pdPASS) {
		printf("Greska prilikom kreiranja\n");
	}
	status = xTaskCreate(LED_bar_Task1, "LEtsk1", configMINIMAL_STACK_SIZE, NULL, (UBaseType_t)OBRADA_TASK_PRI, NULL);  //TASK ZA BLINKANJE DIODA(GENERISANJE SIGNALA)
	if (status != pdPASS) {
		printf("Greska prilikom kreiranja\n");
	}
	status = xTaskCreate(LED_bar_Task2, "LEtsk2", configMINIMAL_STACK_SIZE, NULL, (UBaseType_t)OBRADA_TASK_PRI, NULL);
	if (status != pdPASS) {
		printf("Greska prilikom kreiranja\n");
	}

	
	vTaskStartScheduler(); // OVAJ RADI KAD SVI OSTALI NISU AKTIVNI


	//for (;;);
}

