#include "mbed.h"
#include "rtos.h"
#include "EventQueue.h"
#include "stm32f429i_discovery_gyroscope.h"
#include "stm32f429i_discovery_ts.h"
#include "stm32f429i_discovery.h"
#include "hal/dma_api.h"
#include "LCD_DISCO_F429ZI.h"
#include "Timer.h"

#define PERIODE_ECH 2       // [msec]
#define DUREE_ACQUI 10000    // [msec]
#define BUFFSIZE (DUREE_ACQUI/PERIODE_ECH)
// ------------------------------
// Variables et constantes pour l'oscillo
#define NBRE_PIXEL_X 240
#define NBRE_DIVISIONS 10
int msec_par_division = 1000;
int msec_par_pixel = msec_par_division/NBRE_DIVISIONS;
float data_sous_ech[NBRE_PIXEL_X];

// ------------------------------
// Thread
Thread thread_Acqui; //(osPriorityHigh7);
Thread thread_Oscillo;
Thread thread_Pulsation;
Thread thread_Blink;
// ------------------------------
// Enumérés
typedef enum {
    SIG_ACQ_READY = 0x01,
}tSigEvt;

Thread thread;
Thread thread2;
Thread thread3;
Thread thread4(osPriorityLow);
Thread thread_Compute;
Thread thread_Display;

AnalogIn   HearthSensor(PA_5);
DigitalOut led1(LED1);
DigitalOut led2(LED2);
LCD_DISCO_F429ZI lcd;
Timer t;

float data_buffer1[BUFFSIZE];
float data_buffer2[BUFFSIZE];

volatile bool running = true;

bool debug_printf = false;
Mutex mtxAcquisition;
Mutex mtxCompute;
Mutex mtxDisplay;

Mutex mtx1;
Mutex mtx2;
Mutex mtx3;

float FrequenceCardiaque;
Mutex mtx_FrequenceCardiaque;


// -----------------------------------
Mutex stdio_mutex;

void notify(const char* name) {
    stdio_mutex.lock();
    printf("%s\n\r", name);
    stdio_mutex.unlock();
}

// -----------------------------------
// Blink function toggles the led in a long running loop
void blink(DigitalOut *led) {
    while (running) {
        *led = !*led;
        Thread::wait(1000);
    }
}

void blinkFast(DigitalOut *led) {
    while (running) {
        *led = !*led;
        Thread::wait(100);
    }
}

// =============================================

// ________________________________
void Acquisition()
{
    char buff[50];
    while (running)
    {
        long i=0;

        if (debug_printf)
        {
            sprintf(buff, "Taille du buffer : %d", BUFFSIZE);
            notify(buff);

            notify("La tache d'acquisition commence ...");
        }
        while(i<BUFFSIZE)
        {
            data_buffer1[i] = HearthSensor.read();
            i++;
            Thread::wait(PERIODE_ECH);
        }
        int start_time = t.read_us();
        memcpy((char*)data_buffer2, (char*)data_buffer1, sizeof(data_buffer2));

        if (debug_printf)
        {
            sprintf(buff, "Duree de la copie : %d usec...", t.read_us()-start_time);
            notify(buff);

            i=0;
            while (i<BUFFSIZE)
            {
                sprintf(buff, "%f;%f", data_buffer1[i], data_buffer2[i]);
                notify(buff);
                i++;
            }

            notify("La tache d'acquisition est terminee ...");
        }
        // Fin de l'acquisition
        thread_Oscillo.signal_set(SIG_ACQ_READY);
    }
}

// ________________________________
void Oscillo()
{
    char strbuff[50];
    while (running)
    {
        // Attend le signal de fin d'acquisition
        Thread::signal_wait(SIG_ACQ_READY);

        // Sous-échantillonnage
        // Entrées :
        //      - Tableau d'échantillons : data_buffer2
        //      - Nombre de secondes par divisions
        //      - Nombre de pixel par division
        // Sortie :
        //      - Tableau de 240 éléments sous-échantillonnage de data_buffer2
        int dureeTotalObservation = msec_par_division * NBRE_DIVISIONS;  // msec
        float dureeParPixel = dureeTotalObservation / NBRE_PIXEL_X;      // msec
        int un_sur_n = dureeParPixel / PERIODE_ECH;

        sprintf(strbuff, "un sur n : %d %f %d", dureeTotalObservation, dureeParPixel, un_sur_n);
        notify(strbuff);

        int j=0;
        for (int i=0; i<BUFFSIZE; i++)
        {
            // récupère un échantillon tout les "n" dans le tableau d'acquisition brut
            if (i % un_sur_n == 0)
            {
                data_sous_ech[j++] = data_buffer2[i];
            }
        }

        // Affichage du résultat
        if (debug_printf)
        {
            for (int i=0; i<NBRE_PIXEL_X; i++)
            {
                sprintf(strbuff, "%d;%f", i, data_sous_ech[i]);
                notify(strbuff);
            }
        }

        // Affichage de l'oscillogramme
#define OFFSET_AFFICHAGE_Y 30  // Pixel
#define AMPLITUDE_MAX_Y 330  // Pixel
#define COULEUR_TRACE_OSCILLO LCD_COLOR_BLUE
#define BACKGROUND_COLOR LCD_COLOR_WHITE
        for (int i=0 ; i<NBRE_PIXEL_X ; i++)
        {
            //lcd.DrawPixel(i, OFFSET_AFFICHAGE_Y + data_sous_ech[i]*100, COULEUR_TRACE_OSCILLO);
            // Trace une ligne continue avec le dernier point
            if (i!=0) {
                // Efface les derniers points
                lcd.SetTextColor(BACKGROUND_COLOR);
                lcd.DrawLine(i, OFFSET_AFFICHAGE_Y+AMPLITUDE_MAX_Y,
                             i, OFFSET_AFFICHAGE_Y-AMPLITUDE_MAX_Y);

                lcd.SetTextColor(COULEUR_TRACE_OSCILLO);
                lcd.DrawLine(i-1, OFFSET_AFFICHAGE_Y + data_sous_ech[i-1]*250,
                             i,   OFFSET_AFFICHAGE_Y + data_sous_ech[i]*250);
            }
        }
    }
}

// ________________________________
/*void Acquisition()
{
    while (running)
    {
        Thread::signal_wait(0x3);
        notify("La tache d'acquisition commence ...");
        Thread::wait(1000); // simule une tâche qui prend un peu de temps
        notify("La tache d'acquisition est terminee ...");
        // Fin de l'acquisition
        thread_Compute.signal_set(0x1);
    }
}
*/
// ________________________________
void Compute()
{
    int start_time;
    char buff[100];

    while(running)
    {
        Thread::signal_wait(0x1);
        // Attend que la tâche d'acquisition soit terminée avant de commencer les calculs
        start_time = t.read_ms();
        // ________________________________
        // Simule le calcul à réaliser
        notify("La tache de calcul commence ...");
        Thread::wait(800);  // simule le calcul FFT qui prend un peu de temps
        FrequenceCardiaque = FrequenceCardiaque + 0.2;  // Simule un résultat
        sprintf(buff, "La tache de calcul est terminee et a dure %d msec...", t.read_ms()-start_time);
        notify(buff);
        // ________________________________ Fin de la simulation du calcul
        thread_Display.signal_set(0x2);
    }
}

// ________________________________
void Display()
{
    int start_time;
    char buff[100];

    BSP_LCD_SetFont(&Font8);

    while(running)
    {
        Thread::signal_wait(0x2);
        start_time = t.read_ms();
        notify("La tache d'affichage commence ...");
        Thread::wait(2000);  // simule le calcul qui prend un peu de temps
        sprintf(buff, "Frequence cardiaque = %f BPM", FrequenceCardiaque);
        notify(buff);

        lcd.DisplayStringAt(0, LINE(1), (uint8_t *)buff, CENTER_MODE);

        sprintf(buff, "La tache d'affichage est terminee et a dure %d msec...", t.read_ms()-start_time);
        notify(buff);
        thread_Acqui.signal_set(0x3);
    }
}

// ________________________________
int main()
{
    t.start();

    running = 1;

    thread_Acqui.start(callback(Acquisition));
    thread_Oscillo.start(callback(Oscillo));
    //thread_Compute.start(callback(Compute));
    //thread_Display.start(callback(Display));
    thread_Blink.start(callback(blink, &led1));

    thread_Acqui.signal_set(0x3);

    while(true)
    {
        Thread::wait(500);
    }
    return 0;
}
// =============================================

