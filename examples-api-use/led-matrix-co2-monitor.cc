#include "led-matrix.h"
#include "graphics.h"
#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <time.h>
#include <signal.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

using namespace rgb_matrix;

volatile bool interrupt_received = false;
static void InterruptHandler(int signo)
{
  interrupt_received = true;
}

#define DISP_TYPE 1     //0 = 128x64, Addr-Type 3; 1 = 2x64x64;
#define PATH_TO_FONTS "/home/pi/rpi-rgb-led-matrix/fonts/"
#define BUF_SIZE 20
#define BUF_NUM 10

struct Data
{
    char buf[BUF_NUM][BUF_SIZE] = { "J# 126", "Tür rechts",
                                    "P#64321", "SpeedCold",
                                    "CO2: 1023 ppm", "", "13:25:33",
                                    "22,3 °C", "E13-2", "20,4V" };
} Data;

int serial_port;
int co2 = 0;

static void co2thread_cleanup(void *arg)
{
    close(serial_port);
}

static void *co2thread_thread(void *arg)
{
    int i = 0, n, degC = 0, undok = 0;
    struct termios tty;
    unsigned char msgReadCO2[] = { 0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79 };
    unsigned char read_buf[16];

    pthread_cleanup_push(co2thread_cleanup, NULL);

    //open serial port to CO2 sensor MH-Z19B:
    // https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
    serial_port = open("/dev/ttyUSB0", O_RDWR);
    if (serial_port < 0) 
    {
        printf("Error %i from open: %s\n", errno, strerror(errno));
    }
    if(tcgetattr(serial_port, &tty) != 0) 
    {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }    
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines
    tty.c_lflag &= ~ICANON;     //-> non-canonical mode
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_lflag &= ~ECHO;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;
    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) 
    {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }

    while (!interrupt_received) 
    {
        i++;

        //CO2-Messwert abfragen:
        write(serial_port, msgReadCO2, sizeof(msgReadCO2));
        n = read(serial_port, &read_buf, sizeof(read_buf));
        if (n == 9 && read_buf[0] == 0xFF && read_buf[1] == 0x86)
        {
            co2 = read_buf[2] * 256 + read_buf[3];
            degC = read_buf[4] - 40;
            undok = read_buf[6] * 256 + read_buf[7];
        }

        snprintf(Data.buf[4], BUF_SIZE, "CO2: %4d ppm   ", co2);
        snprintf(Data.buf[7], BUF_SIZE, "%3d°C   ", degC);
        snprintf(Data.buf[9], BUF_SIZE, "%5d    ", undok);
        sleep(10);
    }

    pthread_cleanup_pop(1);
    pthread_exit((void *)pthread_self());
}

static void WriteText(Canvas *canvas,
    int x,
    int y,
    rgb_matrix::Font &text_font,
    char *text,
    Color *outline_color,
    Color *bg_color,
    Color *color,
    int letter_spacing = 0)
{
    rgb_matrix::Font *outline_font = NULL;
    if (outline_color) {
        outline_font = text_font.CreateOutlineFont();
    }
    if (outline_font) {
        // The outline font, we need to write with a negative (-2) text-spacing,
        // as we want to have the same letter pitch as the regular text that we then write on top.
        rgb_matrix::DrawText(canvas,
            *outline_font,
            x - 1,
            y + text_font.baseline(),
            *outline_color,
            bg_color,
            text,
            letter_spacing - 2);
    }

    // regular text+ background:
    rgb_matrix::DrawText(canvas,
        text_font,
        x,
        y + text_font.baseline(),
        *color,
        outline_font ? NULL : bg_color,
        text,
        letter_spacing);
}

int main(int argc, char *argv[])
{
    pthread_t pthr;
    time_t mytime;
    struct tm *timeinfo;

    signal(SIGTERM, InterruptHandler);
    signal(SIGINT, InterruptHandler);

    if (pthread_create(&pthr, NULL, &co2thread_thread, &Data) != 0)
    {
        fprintf(stderr, "Konnte co2thread-Thread nicht erzeugen\n");
        exit(EXIT_FAILURE);
    }

    RGBMatrix::Options matrix_options;
    rgb_matrix::RuntimeOptions runtime_opt;

    matrix_options.hardware_mapping = "regular";
    matrix_options.brightness = 75;          //erlaubt 1..100
    matrix_options.row_address_type = 3;      //!!
    matrix_options.chain_length = 1;
    matrix_options.rows = 64;
    matrix_options.cols = 128;
    matrix_options.parallel = 1;
    matrix_options.pwm_bits = 10;        //(default: 11)
    if(DISP_TYPE == 1)   //2x64x64
    {
        matrix_options.row_address_type = 0;
        matrix_options.chain_length = 2;
        matrix_options.cols = 64;
    }
    //    runtime_opt.daemon = 0;
    //    runtime_opt.drop_privileges = 1;
    //    runtime_opt.gpio_slowdown = 1;  //(default: 1)

        RGBMatrix *canvas = rgb_matrix::CreateMatrixFromOptions(matrix_options, runtime_opt);
    if (canvas == NULL)
        return 1;
    //    canvas->SetBrightness(brightness);

        char bdf_font_file[256];

    //Laden Font klein:
    sprintf(bdf_font_file, "%s6x10.bdf", PATH_TO_FONTS);
    rgb_matrix::Font fontKlein;
    if (!fontKlein.LoadFont(bdf_font_file)) {
        fprintf(stderr, "Couldn't load font '%s'\n", bdf_font_file);
    }
    //Laden Font Mittel:
    sprintf(bdf_font_file, "%s8x13.bdf", PATH_TO_FONTS);
    rgb_matrix::Font fontMittel;
    if (!fontMittel.LoadFont(bdf_font_file)) {
        fprintf(stderr, "Couldn't load font '%s'\n", bdf_font_file);
    }
    //Laden Font Groß:
    sprintf(bdf_font_file, "%s8x13B.bdf", PATH_TO_FONTS);
    rgb_matrix::Font fontGross;
    if (!fontGross.LoadFont(bdf_font_file)) {
        fprintf(stderr, "Couldn't load font '%s'\n", bdf_font_file);
    }

    Color colRot(224, 0, 0);
    Color colRotD(42, 12, 12);
    Color colGruen(0, 192, 0);
    Color colBlau(0, 0, 224);
    Color colGelb(175, 175, 0);
    Color colTuerkis(0, 175, 175);
    Color colUViolett(175, 0, 175);

    Color bg_color(0, 8, 0);
    Color bg_col_Schwarz(0, 0, 0);
    Color outline_color(0, 40, 0);
    int x = 0;
    int y = 0;
    char text[256];

    while (!interrupt_received) 
    {
        mytime = time(NULL);
        timeinfo = localtime(&mytime);

        //Datum:
        x = 0; y = 0;
        strftime(text, BUF_SIZE, "%a %b %e %Y", timeinfo);
        WriteText(canvas, x+5, y, fontMittel, text, NULL, &bg_col_Schwarz, &colGelb);

        //Uhrzeit:
        snprintf(text, BUF_SIZE, "%02d:%02d:%02d", timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
        WriteText(canvas, x + 32, y + 16, fontGross, text, NULL, &bg_col_Schwarz, &colGelb);

        //CO2:
        strncpy(text, Data.buf[4], BUF_SIZE);
        Color *pColor = co2 >= 2500 ? &colRot : co2 >= 2000 ? &colUViolett : co2 >= 1000 ? &colGelb : &colGruen;
        WriteText(canvas, x+2, y + 32, fontGross, text, NULL, &bg_col_Schwarz, pColor);

        //Uhrzeit/Schweißzeit:
//        strncpy(text, Data.buf[6], BUF_SIZE);
//        snprintf(text, BUF_SIZE, "%02d:%02d:%02d", timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
//        WriteText(canvas, x + 78, y + 33, fontKlein, text, NULL, &bg_col_Schwarz, &colTuerkis);

        //Temperatur:
        strncpy(text, Data.buf[7], BUF_SIZE);
        WriteText(canvas, x+2, y + 51, fontGross, text, NULL, &bg_col_Schwarz, &colGruen);

        //Fehler:
        strncpy(text, Data.buf[8], BUF_SIZE);
//        WriteText(canvas, x + 43, y + 51, fontGross, text, NULL, &bg_col_Schwarz, &colRotD);

        //undok. Wert:
        strncpy(text, Data.buf[9], BUF_SIZE);
        WriteText(canvas, x + 79, y + 51, fontMittel, text, NULL, &bg_col_Schwarz, &colRotD);

        usleep(200000);    //Refresh-Rate
    }

    // Finished. Shut down the RGB matrix.
    canvas->Clear();
    delete canvas;
//    pthread_join(pthr, NULL);
    return 0;
}

