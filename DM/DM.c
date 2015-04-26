/*
    Disquetes Musicais

    Este módulo movimenta a cabeça das unidades de disquete e chaveia
    o relê conforme comandos recebidos da serial.
    
    Re-escrita do Moppy.ino by Sammy1Am

    (C) 2014-2015, Daniel Quadros

    ----------------------------------------------------------------------------
     "THE BEER-WARE LICENSE" (Revision 42):
     <dqsoft.blogspot@gmail.com> wrote this file.  As long as you retain this 
     notice you can do whatever you want with this stuff. If we meet some day, 
     and you think this stuff is worth it, you can buy me a beer in return.
        Daniel Quadros
    ----------------------------------------------------------------------------

*/

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

// Bits correspondentes aos pinos de E/S
#define DDR_RELE    DDRD
#define PORT_RELE   PORTD
#define BIT_RELE   _BV(PD5)

#define DDR_LED     DDRD
#define PORT_LED    PORTD
#define BIT_LED    _BV(PD4)

#define DIR_A  _BV(PB7)
#define DIR_B  _BV(PB6)
#define DIR_C  _BV(PB5)
#define STEP_A _BV(PB4)
#define STEP_B _BV(PB3)
#define STEP_C _BV(PB2)
#define TRK0_A _BV(PB1)
#define TRK0_B _BV(PB0)
#define TRK0_C _BV(PD6)

// Controle do rele
#define TEMPO_RELE  50; // tempo em ms para pulsar o rele
static volatile uint8_t cntRele;

// Controle do LED
#define TEMPO_LED   100; // tempo em ms para pulsar o LED
static volatile uint8_t cntLED;

// Estrutura para controle de uma unidade de disquete
#define UP      0
#define DOWN    1
typedef struct
{
    uint16_t ddrDir;
    uint16_t portDir;
    uint8_t bitDir;
    uint16_t ddrStep;
    uint16_t portStep;
    uint8_t bitStep;
    uint16_t ddrTrk0;
    uint16_t portTrk0;
    uint16_t pinTrk0;
    uint8_t bitTrk0;
    uint8_t trkAtual;
    uint16_t periodo;
    uint16_t cnt;
    uint8_t  dir;
} DISQUETE;

#define N_DISQ  3
#define MAX_TRK 79

static volatile DISQUETE unidade[N_DISQ] =
{
    {
        (uint16_t) &DDRB, (uint16_t) &PORTB, DIR_A,
        (uint16_t) &DDRB, (uint16_t) &PORTB, STEP_A,
        (uint16_t) &DDRB, (uint16_t) &PORTB, (uint16_t) &PINB, TRK0_A,
        0, 0, 0, UP
    },
    {
        (uint16_t) &DDRB, (uint16_t) &PORTB, DIR_B,
        (uint16_t) &DDRB, (uint16_t) &PORTB, STEP_B,
        (uint16_t) &DDRB, (uint16_t) &PORTB, (uint16_t) &PINB, TRK0_B,
        0, 0, 0, UP
    },
    {
        (uint16_t) &DDRB, (uint16_t) &PORTB, DIR_C,
        (uint16_t) &DDRB, (uint16_t) &PORTB, STEP_C,
        (uint16_t) &DDRD, (uint16_t) &PORTD, (uint16_t) &PIND, TRK0_C,
        0, 0, 0, UP
    },
};

#define DDR_STEP(i) ( (volatile uint8_t *)(unidade[i].ddrStep))
#define PORT_STEP(i) ( (volatile uint8_t *)(unidade[i].portStep))
#define DDR_DIR(i) ( (volatile uint8_t *)(unidade[i].ddrDir))
#define PORT_DIR(i) ( (volatile uint8_t *)(unidade[i].portDir))
#define DDR_TRK0(i) ( (volatile uint8_t *)(unidade[i].ddrTrk0))
#define PORT_TRK0(i) ( (volatile uint8_t *)(unidade[i].portTrk0))
#define PIN_TRK0(i) ( (volatile uint8_t *)(unidade[i].pinTrk0))

// Fila de recepção da serial
#define TO_RX  100      // timeout em ms para receber comando completo
#define T_FILA 16
static volatile uint8_t filaRx[T_FILA];
static volatile uint8_t poe, tira;
static volatile uint8_t toRx = 0;

// Rotinas locais
static void initHw (void);
static void reset_all (void);
static int le_espera_serial (uint8_t to);
static int le_serial (void);
static void piscaLED (void);


// Programa Principal
int main (void)
{
    int c;
    
    // Inicia o hardware
    initHw ();

    // Inicia a fila da serial
    poe = tira = 0;
    
    // Dá um click no rele para mostrar que está vivo
    PORT_RELE |= BIT_RELE;
    cntRele = TEMPO_RELE;
    
    // Coloca todas as unidades na trilha zero
    reset_all ();
    
    // Permite interrupções
    sei ();
    
    // Avisa que está pronto
    piscaLED();
    
    // Loop infinito
    for (;;)
    {
        c = le_espera_serial(0);
        UDR = (uint8_t) c;      // eco (para debug)
        if (c == 100)
        {
            // Reset
            piscaLED ();        // comando reconhecido
            reset_all ();
            cli ();
            poe = tira = 0;     // limpa a fila
            sei ();
        }
        else if (c == 101)
        {
            // Percussão
            piscaLED ();        // comando reconhecido
            cli ();
            if (cntRele == 0)
            {
                PORT_RELE |= BIT_RELE;
                cntRele = TEMPO_RELE;
            }
            sei ();
        }
        else if ((c > 0) && (--c < N_DISQ))
        {
            // Muda periodo da unidade c
            int aux;
            uint16_t periodo;
            aux = le_espera_serial(TO_RX);
            if (aux == -1)
                continue;
            periodo = ((uint16_t) aux) << 8;
            aux = le_espera_serial(TO_RX);
            if (aux == -1)
                continue;
            periodo |= (uint16_t) aux;
            if (periodo != 0xFFFF)
            {
                piscaLED ();    // comando reconhecido
                cli ();
                unidade[c].periodo = periodo;
                unidade[c].cnt = periodo;
                sei ();
            }
        }
    }
}

// Iniciação do hardware
static void initHw ()
{
    int i;
    
    // Configura os pinos do rele
    DDR_RELE |= BIT_RELE;
    PORT_RELE &= ~BIT_RELE;
    
    // Configura os pinos do LED
    DDR_LED |= BIT_LED;
    PORT_LED &= ~BIT_LED;
    
    // Configura os pinos de controle das unidades de disquete
    for (i = 0; i < N_DISQ; i++)
    {
        *DDR_STEP(i) |= unidade[i].bitStep;     // STEP é saida
        *PORT_STEP(i) |= unidade[i].bitStep;    // (repouso alto)
        *DDR_DIR(i) |= unidade[i].bitDir;       // DIR é saída
        *PORT_DIR(i) |= unidade[i].bitDir;      // (repouso alto)
        *DDR_TRK0(i) &= ~unidade[i].bitTrk0;    // TRK0 é entrada
        *PORT_TRK0(i) &= ~unidade[i].bitTrk0;   // (sem pullup)
    }
    
    // Configura o timer 1
    TCCR1A = 0;                     // OC1A/OC1B desconectados
    TCCR1B = _BV(WGM13);            // Modo 8: phase and frequency correct pwm
                                    // Timer parado
    ICR1 = 320;                     // Período = 2*(320/16000000) = 40uSeg
    TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
    TCCR1B |= _BV(CS10);            // Usar clkIO/1, inicia o timer
    TIMSK = _BV(TOIE1);             // Interromper no overflow
        
    // Configura o timer 0
    TCCR0A = 0;                         // Modo normal: overflow a cada 256 contagens
    TCCR0B = _BV(CS01) | _BV(CS00);     // Usar clkIO/64: int a cada 64*256/16000 ms
                                        //   = 1,024 ms
    TIMSK |= _BV(TOIE0);                // Interromper no overflow
    
    // Programa a USART para 9600 8N1
    UCSRA = _BV(U2X);		            // para maior resolução do baud rate
    UCSRB = _BV(RXEN) | _BV(TXEN) | _BV(RXCIE);     // liga recepção e transmissão
                                                    //   com interrupção de Rx
    UCSRC = _BV(UCSZ1) | _BV(UCSZ0) ;               // 8bit, 1 stop, sem paridade
    UBRRH = 0;
    UBRRL = (F_CPU / (8 * 9600UL)) - 1;             // 9600 bps
}


// Coloca todas as unidades na trilha zero
static void reset_all ()
{
    uint8_t oldSREG;
    int i, trk;
    
    oldSREG = SREG;				// salva estado da interrupção
    cli();                      // desliga interrupções
    for (i = 0; i < N_DISQ; i++)
    {
        unidade[i].periodo = unidade[i].cnt = 0;   // parado
        *PORT_DIR(i) |= unidade[i].bitDir;         // voltando
        for (trk = MAX_TRK; 
             (trk > 0) && ((*PIN_TRK0(i) & unidade[i].bitTrk0) != 0);
             trk--)
        {
            *PORT_STEP(i) &= ~unidade[i].bitStep;
            _delay_us (10);
            *PORT_STEP(i) |= unidade[i].bitStep;
            _delay_us (10);
        }
        unidade[i].trkAtual = 0;
        unidade[i].dir = UP;
    }
    SREG = oldSREG;             // restaura interrupções
}

// Aguarda receber um caracter e o retorna
// to = timeout em ms (0 = sem timeout)
static int le_espera_serial (uint8_t to)
{
    int c;
    
    toRx = to;
    for (;;)
    {
        c = le_serial();
        if (c != -1)
            return c;
        if ((to != 0) && (toRx == 0))
            return -1;  // timeout
    }
}

// Pega o próximo caracter da fila de recepção da serial
// Retorna -1 se fila vazia
static int le_serial ()
{
    int c;
    cli ();
    if (poe == tira)
        c = -1;   // fila vazia
    else
    {
        c = filaRx[tira];
        if (++tira == T_FILA)
            tira = 0;
    }
    sei ();
    return c;
}


// Tratamento da interrupção do timer 1
ISR (TIMER1_OVF_vect)
{
    int i;
    volatile DISQUETE *disq;
    
    for (i = 0, disq = unidade; i < N_DISQ; i++, disq++)
    {
        if ((disq->periodo != 0) && (--(disq->cnt) == 0))
        {
            // muda de direção se chegou ao fim
            if (disq->trkAtual == (2*MAX_TRK-1))
            {
                *((volatile uint8_t *)(disq->portDir)) |= disq->bitDir;
                disq->dir = DOWN;
            }
            else if (disq->trkAtual == 0)
            {
                *((volatile uint8_t *)(disq->portDir))  &= ~disq->bitDir;
                disq->dir = UP;
            }
            *((volatile uint8_t *)(disq->portStep))  ^= disq->bitStep;
            disq->cnt = disq->periodo;
            if (disq->dir == UP)
                disq->trkAtual++;
            else
                disq->trkAtual--;
        }
    }
}

// Pisca LED
static void piscaLED (void)
{
    cli ();
    if (cntLED == 0)
    {
        PORT_LED |= BIT_LED;
        cntLED = TEMPO_LED;
    }
    sei ();
}

// Tratamento da interrupção do timer 0
ISR (TIMER0_OVF_vect)
{
    // Volta rele para o repouso após um clique
    if ((cntRele != 0) && (--cntRele == 0))
        PORT_RELE &= ~BIT_RELE;
    
    // Apaga LED após certo tempo
    if ((cntLED != 0) && (--cntLED == 0))
        PORT_LED &= ~BIT_LED;
    
    // Trata timeout na recepção
    if (toRx != 0)
        toRx--;
}

// Interrupção de recepção da USART
ISR(USART_RX_vect)
{
    uint8_t prox;
    
    filaRx[poe] = UDR;      // pega o caracter
    prox = poe + 1;         // avança o ponteiro de entrada
    if (prox == T_FILA)
        prox = 0;
    if (prox != tira)       // não atualizar se fila cheia
        poe = prox;
}
