// Teste do firmware da placa de controle dos diquetes musicais

#include <stdio.h>
#include <strings.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/time.h>

// Periodo das notas  (C3 a B3 em milisegundos)
int notas[] =
{
    7645, 6811, 6068, 5727, 5103, 4546, 4050, 0
};

// Teste de tecla digitada
// http://www.flipcode.com/archives/_kbhit_for_Linux.shtml
int kbhit (void)
{
    static int STDIN = 0;
    struct timeval timeout;
    fd_set rdset;

    FD_ZERO(&rdset);
    FD_SET(STDIN, &rdset);
    timeout.tv_sec  = 0;
    timeout.tv_usec = 0;

    return select(STDIN + 1, &rdset, NULL, NULL, &timeout);
}


void main (void)
{
    int fd;	// handle para a serial
    int i, j;
    struct termios tty;
    int cflag = CREAD | CLOCAL | HUPCL | B9600 | CS8;

    // comandos para o firmware
    unsigned char cmdReset[] = { 100 };
    unsigned char cmdClick[] = { 101 };
    unsigned char cmdNota[3];

    printf ("Ola, disquetes\n");

    // Abre e configura a serial
    fd = open ("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1)
    {
        printf ("Erro ao abrir a serial\n");
        return;
    }
    bzero(&tty, sizeof(tty));
    tty.c_iflag &= ~(IXON | IXOFF);
    tty.c_cflag = cflag;
    tty.c_lflag &= ~(ICANON | ECHO);
    tty.c_iflag &= ~(ICRNL | ISTRIP);
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 0;
    tcflush (fd, TCIFLUSH);
    if (tcsetattr (fd, TCSANOW, &tty) != 0)
    {
        printf ("Erro ao configurar a serial\n");
    }
    sleep(1);

    // Move as cabecas para a trilha 0
    write (fd, cmdReset, sizeof(cmdReset));

    // Vamos repetir ate digitar ENTER no console
    while (!kbhit())
    {
        // Tres cliques no rele
        for (i = 0; i < 3; i++)
        {
            write (fd, cmdClick, sizeof(cmdClick));
            sleep(1);
        }

        // Tocar a sequencia de notas nas unidades
        for (i = 1; i <= 3; i++)
        {
           if (kbhit())
             break;
           cmdNota[0] = i;
           for (j = 0; j < 8; j++)
           {
               int t = notas[j]/80;  // meio periodo em unidades de 40ms
               if (kbhit())
                 break;
               cmdNota[1] = (unsigned char) (t >> 8);
               cmdNota[2] = (unsigned char) t;
               write (fd, cmdNota, sizeof(cmdNota));
               sleep(1);
           }
        }
        sleep(5);
    }

    // Volta as cabecas para a trilha 0 e libera a serial
    write (fd, cmdReset, sizeof(cmdReset));
    close (fd);
}

