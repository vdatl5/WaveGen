/*
 *  Title: Wave Generator
 *  File : main.c
 *  Target : ATMEGA328PU
 */

#define SCL_CLOCK 100000L  // i2c speed
#define F_CPU 16000000UL  // CPU speed
#define READ 1
#define WRITE 0
#define ADDR 0x48  // temp sensor address
#define BUFFER_SIZE 20  // uart buffer size
#define _ASSERT_ENABLE_


#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <stdlib.h>
#include <float.h>
#include <avr/pgmspace.h>
#include <inttypes.h>
#include <util/delay.h>
#include <util/twi.h>
#include <stdio.h>

#include "ASF/mega/utils/compiler.h"
#include "./ring_buffer.h"
#include "config/conf_uart.h"
#include <util/setbaud.h>


// wave types
enum waveTypes{SINEWAVE = 1, SQUAREWAVE = 2 , TRIWAVE = 3, SAWWAVE = 4,
RSAWWAVE = 5 };


// wave struct
typedef struct {
    float amplitude;
    float offset;
    int frequency;
    int wave_type;
}Wave;

//  Look up tables for the different waves
uint8_t  sine_wave[256] = {
    0x80, 0x83, 0x86, 0x89, 0x8C, 0x90, 0x93, 0x96,
    0x99, 0x9C, 0x9F, 0xA2, 0xA5, 0xA8, 0xAB, 0xAE,
    0xB1, 0xB3, 0xB6, 0xB9, 0xBC, 0xBF, 0xC1, 0xC4,
    0xC7, 0xC9, 0xCC, 0xCE, 0xD1, 0xD3, 0xD5, 0xD8,
    0xDA, 0xDC, 0xDE, 0xE0, 0xE2, 0xE4, 0xE6, 0xE8,
    0xEA, 0xEB, 0xED, 0xEF, 0xF0, 0xF1, 0xF3, 0xF4,
    0xF5, 0xF6, 0xF8, 0xF9, 0xFA, 0xFA, 0xFB, 0xFC,
    0xFD, 0xFD, 0xFE, 0xFE, 0xFE, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFE, 0xFE, 0xFD,
    0xFD, 0xFC, 0xFB, 0xFA, 0xFA, 0xF9, 0xF8, 0xF6,
    0xF5, 0xF4, 0xF3, 0xF1, 0xF0, 0xEF, 0xED, 0xEB,
    0xEA, 0xE8, 0xE6, 0xE4, 0xE2, 0xE0, 0xDE, 0xDC,
    0xDA, 0xD8, 0xD5, 0xD3, 0xD1, 0xCE, 0xCC, 0xC9,
    0xC7, 0xC4, 0xC1, 0xBF, 0xBC, 0xB9, 0xB6, 0xB3,
    0xB1, 0xAE, 0xAB, 0xA8, 0xA5, 0xA2, 0x9F, 0x9C,
    0x99, 0x96, 0x93, 0x90, 0x8C, 0x89, 0x86, 0x83,
    0x80, 0x7D, 0x7A, 0x77, 0x74, 0x70, 0x6D, 0x6A,
    0x67, 0x64, 0x61, 0x5E, 0x5B, 0x58, 0x55, 0x52,
    0x4F, 0x4D, 0x4A, 0x47, 0x44, 0x41, 0x3F, 0x3C,
    0x39, 0x37, 0x34, 0x32, 0x2F, 0x2D, 0x2B, 0x28,
    0x26, 0x24, 0x22, 0x20, 0x1E, 0x1C, 0x1A, 0x18,
    0x16, 0x15, 0x13, 0x11, 0x10, 0x0F, 0x0D, 0x0C,
    0x0B, 0x0A, 0x08, 0x07, 0x06, 0x06, 0x05, 0x04,
    0x03, 0x03, 0x02, 0x02, 0x02, 0x01, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x02, 0x02, 0x02, 0x03,
    0x03, 0x04, 0x05, 0x06, 0x06, 0x07, 0x08, 0x0A,
    0x0B, 0x0C, 0x0D, 0x0F, 0x10, 0x11, 0x13, 0x15,
    0x16, 0x18, 0x1A, 0x1C, 0x1E, 0x20, 0x22, 0x24,
    0x26, 0x28, 0x2B, 0x2D, 0x2F, 0x32, 0x34, 0x37,
    0x39, 0x3C, 0x3F, 0x41, 0x44, 0x47, 0x4A, 0x4D,
    0x4F, 0x52, 0x55, 0x58, 0x5B, 0x5E, 0x61, 0x64,
    0x67, 0x6A, 0x6D, 0x70, 0x74, 0x77, 0x7A, 0x7D
};

uint8_t  square_wave[256] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    125, 0, 0, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0, 0, 125,
};

uint8_t triangle[256] =
    {1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25,
    27, 29, 31, 33, 35, 37, 39, 41, 43, 45, 47, 49,
    51, 53, 55, 57, 59, 61, 63, 65, 67, 69, 71, 73,
    75, 77, 79, 81, 83, 85, 87, 89, 91, 93, 95, 97,
    99, 101, 103, 105, 107, 109, 111, 113, 115, 117,
    119, 121, 123, 125, 127, 129, 131, 133, 135, 137,
    139, 141, 143, 145, 147, 149, 151, 153, 155, 157,
    159, 161, 163, 165, 167, 169, 171, 173, 175, 177,
    179, 181, 183, 185, 187, 189, 191, 193, 195, 197,
    199, 201, 203, 205, 207, 209, 211, 213, 215, 217,
    219, 221, 223, 225, 227, 229, 231, 233, 235, 237,
    239, 241, 243, 245, 247, 249, 251, 253, 255, 255,
    253, 251, 249, 247, 245, 243, 241, 239, 237, 235,
    233, 231, 229, 227, 225, 223, 221, 219, 217, 215,
    213, 211, 209, 207, 205, 203, 201, 199, 197, 195,
    193, 191, 189, 187, 185, 183, 181, 179, 177, 175,
    173, 171, 169, 167, 165, 163, 161, 159, 157, 155,
    153, 151, 149, 147, 145, 143, 141, 139, 137, 135,
    133, 131, 129, 127, 125, 123, 121, 119, 117, 115,
    113, 111, 109, 107, 105, 103, 101, 99, 97, 95, 93,
    91, 89, 87, 85, 83, 81, 79, 77, 75, 73, 71, 69, 67,
    65, 63, 61, 59, 57, 55, 53, 51, 49, 47, 45, 43, 41,
    39, 37, 35, 33, 31, 29, 27, 25, 23, 21, 19, 17, 15,
     13, 11, 9, 7, 5, 3, 1};

uint8_t reverse_sawtooth[256] = {
    191, 125, 64, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,
    14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24,
    25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35,
    36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46,
    47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57,
    58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68,
    69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79,
    80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90,
    91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101,
    102, 103, 104, 105, 106, 107, 108, 109, 110,
    111, 112, 113, 114, 115, 116, 117, 118, 119,
    120, 121, 122, 123, 124, 125, 126, 127, 128,
    129, 130, 131, 132, 133, 134, 135, 136, 137,
    138, 139, 140, 141, 142, 143, 144, 145, 146,
    147, 148, 149, 150, 151, 152, 153, 154, 155,
    156, 157, 158, 159, 160, 161, 162, 163, 164,
    165, 166, 167, 168, 169, 170, 171, 172, 173,
    174, 175, 176, 177, 178, 179, 180, 181, 182,
    183, 184, 185, 186, 187, 188, 189, 190, 191,
    192, 193, 194, 195, 196, 197, 198, 199, 200,
    201, 202, 203, 204, 205, 206, 207, 208, 209,
    210, 211, 212, 213, 214, 215, 216, 217, 218,
    219, 220, 221, 222, 223, 224, 225, 226, 227,
    228, 229, 230, 231, 232, 233, 234, 235, 236,
    237, 238, 239, 240, 241, 242, 243, 244, 245,
    246, 247, 248, 249, 250, 251, 252, 253, 254, 255};

uint8_t sawtooth[256] = {
    255, 254, 253, 252, 251, 250, 249, 248, 247,
    246, 245, 244, 243, 242, 241, 240, 239, 238,
    237, 236, 235, 234, 233, 232, 231, 230, 229,
    228, 227, 226, 225, 224, 223, 222, 221, 220,
    219, 218, 217, 216, 215, 214, 213, 212, 211,
    210, 209, 208, 207, 206, 205, 204, 203, 202,
    201, 200, 199, 198, 197, 196, 195, 194, 193,
    192, 191, 190, 189, 188, 187, 186, 185, 184,
    183, 182, 181, 180, 179, 178, 177, 176, 175,
    174, 173, 172, 171, 170, 169, 168, 167, 166,
    165, 164, 163, 162, 161, 160, 159, 158, 157,
    156, 155, 154, 153, 152, 151, 150, 149, 148,
    147, 146, 145, 144, 143, 142, 141, 140, 139,
    138, 137, 136, 135, 134, 133, 132, 131, 130,
    129, 128, 127, 126, 125, 124, 123, 122, 121,
    120, 119, 118, 117, 116, 115, 114, 113, 112,
    111, 110, 109, 108, 107, 106, 105, 104, 103,
    102, 101, 100, 99, 98, 97, 96, 95, 94, 93,
    92, 91, 90, 89, 88, 87, 86, 85, 84, 83, 82,
    81, 80, 79, 78, 77, 76, 75, 74, 73, 72, 71,
    70, 69, 68, 67, 66, 65, 64, 63, 62, 61, 60,
    59, 58, 57, 56, 55, 54, 53, 52, 51, 50, 49,
    48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38,
    37, 36, 35, 34, 33, 32, 31, 30, 29, 28, 27,
    26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16,
    15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4,
    3, 64, 125, 191
};


//  the lookup table that is used to wave1
uint8_t current_wave[256] = {0};
//  lookup table used for wave 2
uint8_t current_2_wave[256] = {0};

//  UART buffers
uint8_t out_buffer[BUFFER_SIZE];
uint8_t in_buffer[BUFFER_SIZE];

// ack and err characters
const char ack[] = "ACK\n";
const char err[] = "ERR\n";


//  UART ring buffers
struct ring_buffer ring_buffer_out;
struct ring_buffer ring_buffer_in;

//  initiate the structs for the waves
Wave waveOne = {1.5, 0, 100, SINEWAVE};
Wave waveTwo = {1.5, 0, 200, SINEWAVE};

int scale = 15;  //  size of the wave scale where
volatile uint8_t temp_c;  // temp representation of port c
volatile uint8_t temp_b;  //  temp representation of port B

//  wave one (W1) variables
volatile uint8_t wave_one_index = 0;  //  W1 look up table index
volatile int repeat_count_1 = 0;  //  W1 times to repeat the look up table value
volatile int current_count_1 = 0;  //  no. of times repeated so far
volatile int sample_count_1 = 0;
volatile uint8_t wave_out_1 = 0;  //  value to be written to ports
volatile int inverse_repeat_1;  //  indexes to skip (1/repeatcout)
volatile int threshold_rep_1 = 0;  //  times to repeat index (out of 15)
volatile int threshold_skip_1 = 0;  //  times to skip index (out of 15)


//  wave two (W2) variables
volatile uint8_t wave_two_index = 0;
volatile uint16_t tempD = 0;
volatile uint8_t wave_out_2 = 0;
volatile int repeat_count_2 = 0;
volatile int current_count_2 = 0;
volatile int sample_count_2 = 0;
volatile int inverse_repeat_2;
volatile int threshold_rep_2 = 0;
volatile int threshold_skip_2 = 0;

//  general wave variables
int sampling_frequency = 44500;  //  sampling rate
int threshold_freq_int;  //  freq of lookup table traversal @ sampling rate
float threshold_freq_float;  //  (float) freq of lookup table traversal @ sr

//  temp sensor variables
volatile  one_second_counter = 0;  //  if one second has past since temp output
int temperature_msb = 0;  //  value of temp reading
int temp_display = 1;  //  if to display the temp value
volatile int one_second_interrup = 0;  //  temp sensor time counter

//  serial variables
uint8_t recieved_byte;  //  byte received over uart
char recieved_string[12] = {0};  //  stores instructions recieved
int recieved_string_index = 0;  //  current index in the received_string buffer
int format_error = 0;  //  send ERR string
int send_ack = 0;  //  send ACK string
int continuee = 0;  //  continue processing the waves (enable interrupt)


//  function defines
void I2cInit(void);
void GetTemp(unsigned char addr);
static void UartInit(void);
static inline void UartPutChar(uint8_t data);
static inline uint8_t UartGetChar(void);
static inline bool UartCharWaiting(void);
int StartSend(void);
void StopSend(void);
int ControlSend(int read, unsigned char addr);
int DataSend(int data);
unsigned char DataGet(unsigned char last);
void InterruptInit(void);
void WaveInit(void);
void ClearReceiveBuffer(void);
void SendReply(void);
void PopulateWaveTable(float Ampl, float offset,
                        int frequency, int waveType, int WaveNo);



 /**
 * \brief Main
 * \param Null
 * \retval Null
 */
int main(void) {
    uint8_t cnt;

    //  initialize uart
    cli();
    UartInit();
    WaveInit();
    InterruptInit();
    I2cInit();
    sei();  //  enable global interrupts

    while (true) {
            //  serial reading code

            SendReply();

            while (UartCharWaiting() == true) {
                TIMSK0 &= ~(1 << OCIE0A);  // disable the interrupt
                TIMSK1 &= ~(1 << OCIE1A);  // disable the interrupt
                //  if there are chars in receive buffer
                recieved_byte = UartGetChar();

                if (recieved_byte == '\0') {
                    //  ignore the null terminator
                    //  UartPutChar('0');
                    continue;
                }

                //  send the instruction back
                //  UartPutChar(recieved_byte);
                recieved_string[recieved_string_index] = recieved_byte;
                recieved_string_index++;

                if (recieved_byte == '!') {
                    //  check for the right format, else send err back
                    //  get the value from the string and convert it in to int
                    char value_received[6] = {0};
                    value_received[0] = recieved_string[4];
                    value_received[1] = recieved_string[5];
                    value_received[2] = recieved_string[6];
                    value_received[3] = recieved_string[7];
                    value_received[4] = recieved_string[8];
                    value_received[5] = '\0';

                    //  convert the value to int
                    char *ptr;
                    int value_int;
                    value_int = (int) strtol(value_received, &ptr, 10);

                    //  interpvalue_int the value as a floating point
                    char *pointer;
                    float value_float = (double) strtod(value_received,
                                                                     &pointer);

                    //  change amplitude
                     if (recieved_string[0] == 'A' &&
                          recieved_string[1] == 'M' ) {
                        if (value_float >= 0 && value_float <= 10) {
                            if (recieved_string[2] == '1') {
                                //  change amplitude for first wave
                                waveOne.amplitude = value_float;
                            } else if (recieved_string[2] == '2') {
                                //  change amplitude for the second wave
                                waveTwo.amplitude = value_float;
                            } else {
                                //  error
                                format_error = 1;
                                continue;
                            }
                        } else {
                            //  error
                            format_error = 1;
                            continue;
                        }
                        //  send ack
                        send_ack = 1;
                        continue;
                     } else if (recieved_string[0] == 'O' &&
                                 recieved_string[1] == 'F' ) {
                        //  put + or - in the string
                        if (value_float >= -10 && value_float <= 10) {
                            if (recieved_string[2] == '1') {
                                //  change offset for first wave
                                waveOne.offset = value_float;
                            } else if (recieved_string[2] == '2') {
                                //  change offset for the second wave
                                waveTwo.offset = value_float;
                            } else {
                                //  error
                                format_error = 1;
                                continue;
                            }
                        } else {
                            //  error
                            format_error = 1;
                            continue;
                        }
                        //  send ack
                        send_ack = 1;
                        continue;
                     } else if (recieved_string[0] == 'F' &&
                                 recieved_string[1] == 'R' ) {
                        if (value_int >= 1 && value_int <= 10000) {
                            if (recieved_string[2] == '1') {
                                //  change frequency for first wave
                                waveOne.frequency = value_int;
                            } else if (recieved_string[2] == '2') {
                                //  change frequency for the second wave
                                waveTwo.frequency = value_int;
                            } else {
                                //  error
                                format_error = 1;
                                continue;
                            }
                        } else {
                                //  error
                                format_error = 1;
                                continue;
                        }
                        //  send ack
                        send_ack = 1;
                        continue;
                     } else if (recieved_string[0] == 'W' &&
                                recieved_string[1] == 'A' ) {
                        if (value_int > 0 && value_int <= 5) {
                            if (recieved_string[2] == '1') {
                                //  for the first wave type
                                waveOne.wave_type = value_int;
                            } else if (recieved_string[2] == '2') {
                                //  for the second wave type
                                waveTwo.wave_type = value_int;
                            } else {
                                //  error
                                format_error = 1;
                                continue;
                            }
                        } else {
                            format_error = 1;
                            continue;
                        }
                        //  reaches here only if everything is fine - ack
                        send_ack = 1;
                        continue;
                    } else if (recieved_string[0] == 'C' &&
                    recieved_string[1] == 'O' && recieved_string[2] == 'N' &&
                    recieved_string[3] == 'T' && recieved_string[4] == 'I' &&
                    recieved_string[5] == 'N' && recieved_string[6] == 'U' &&
                    recieved_string[7] == 'E' &&  recieved_string[8] == 'E') {
                        //  if we can continue the interrupts
                        send_ack = 1;
                        continuee = 1;
                    } else {
                        //  send error
                        format_error = 1;
                        continue;
                    }
                }
                _delay_ms(10);
            }

            if (one_second_interrup == 1 && temp_display == 1) {
                //  get the temperature and display
                one_second_interrup = 0;
                GetTemp(ADDR);
                _delay_ms(10);
            }
    }
}

/**
 * \brief Called when sending via UART is available
 *
 * Adapted from example AVR code (AFS license)
 */
ISR(UART0_DATA_EMPTY_IRQ) {
    //  if there is data in the ring buffer, fetch it and send it
    if (!ring_buffer_is_empty(&ring_buffer_out)) {
        UDR0 = ring_buffer_get(&ring_buffer_out);
    } else {
        //  no more data to send, turn off data ready interrupt
        UCSR0B &= ~(1  <<  UDRIE0);
    }
}

/**
 * \brief Data RX interrupt handler
 *
 * Adapted from example AVR code (AFS license)
 */
ISR(UART0_RX_IRQ) {
    ring_buffer_put(&ring_buffer_in, UDR0);
}

/**
 * \brief Initialize UART
 *
 * Adapted from example AVR code (AFS license)
 */
static void UartInit(void) {
#if defined UBRR0H
    //  get the values from the setbaud tool
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;
#else
#error "Device is not supported by the driver"
#endif

#if USE_2X
    UCSR0A |= (1  <<  U2X0);
#endif

    //  enable RX and TX and set interrupts on rx complete
    UCSR0B = (1  <<  RXEN0) | (1  <<  TXEN0) | (1  <<  RXCIE0);

    //  8-bit, 1 stop bit, no parity, asynchronous UART
    UCSR0C = (1  <<  UCSZ01) | (1  <<  UCSZ00) | (0  <<  USBS0) |
            (0  <<  UPM01) | (0  <<  UPM00) | (0  <<  UMSEL01) |
            (0  <<  UMSEL00);

    //  initialize the in and out buffer for the UART
    ring_buffer_out = ring_buffer_init(out_buffer, BUFFER_SIZE);
    ring_buffer_in = ring_buffer_init(in_buffer, BUFFER_SIZE);
}

/**
 * \brief Function for putting a char in the UART buffer
 *
 * \param data the data to add to the UART buffer and send
 *    Adapted from example AVR code (AFS license)
 */
static inline void UartPutChar(uint8_t data) {
    //  Disable interrupts to get exclusive access to ring_buffer_out.
    cli();
    if (ring_buffer_is_empty(&ring_buffer_out)) {
        //  First data in buffer, enable data ready interrupt
        UCSR0B |=  (1  <<  UDRIE0);
    }
    //  Put data in buffer
    ring_buffer_put(&ring_buffer_out, data);

    //  Re-enable interrupts
    sei();
}

/**
 * \brief Function for getting a char from the UART receive buffer
 *    Adapted from example AVR code (AFS license)
 * \retval Next data byte in receive buffer
 */
static inline uint8_t UartGetChar(void) {
    return ring_buffer_get(&ring_buffer_in);
}


/**
 * \brief Function to check if we have a char waiting in the UART receive buffer
 *    Adapted from example AVR code (AFS license)
 * \retval true if data is waiting
 * \retval false if no data is waiting
 */
static inline bool UartCharWaiting(void) {
    return !ring_buffer_is_empty(&ring_buffer_in);
}



/**
 * \brief Function populating the respective lookup tables
 * \param amplitude of wave, the offset, frequency, wavetype and waveno
 * \retval Null
 */
void PopulateWaveTable(float Ampl, float offset,
                      int frequency, int waveType, int WaveNo) {
    uint8_t *pointer;  //  base wave pointer
    //  set the pointer to the base wave
    switch (waveType) {
        case SINEWAVE:
            pointer =  sine_wave;
            break;
        case SQUAREWAVE:
            pointer = square_wave;
            break;
        case TRIWAVE:
            pointer = triangle;
            break;
        case SAWWAVE:
            pointer = sawtooth;
            break;
        case RSAWWAVE:
            pointer = reverse_sawtooth;
            break;
    }

    //  reset the indexes
    wave_one_index = 0;
    wave_two_index = 0;

    if (WaveNo == 1) {  //  if wave 1
        int value;  //  value to place in current buffer
        float trueRepeatCount = 0;  //  number of times to repeat
        float trueSkipCount = 0;  //  number of times to skip

        //  frequency adjustment code for wave 1
        //  for repeat mode
        repeat_count_1 = threshold_freq_int/frequency;  //  integer division
        trueRepeatCount = threshold_freq_float/frequency;  //  float division
        threshold_rep_1 = scale*(1-(trueRepeatCount-repeat_count_1));
        //  for skip mode
        inverse_repeat_1 = frequency/threshold_freq_int;
        trueSkipCount = frequency/threshold_freq_float;
        threshold_skip_1 = scale*(1-(trueSkipCount - inverse_repeat_1));

        //  populate the current wave
        for (int i = 0; i < 256; i++) {
            //  offset and amplitude to output value
            value = (Ampl/3)*pointer[i] +(127*(3-Ampl)/3)- ((offset/3)*127);

            //  write to the current wave buffer
            if (value > 255) {
                current_wave[i] = 255;
            } else if (value < 0) {
                current_wave[i] =  0;
            } else {
                current_wave[i] = value;
            }
        }
    } else if (WaveNo = 2) {
        //  code for wave 2

        //  frequency adjustment code for wave
        float trueRepeatCount_2 = 0;  //  number of times to repeat
        float trueSkipCount_2 = 0;  //  number of times to skip

        //  times to repeat in repeat mode
        repeat_count_2 = threshold_freq_int/frequency;
        trueRepeatCount_2 = threshold_freq_float/frequency;
        threshold_rep_2 = scale*(1-(trueRepeatCount_2 - repeat_count_2));

        //  times to skip in skip mode
        inverse_repeat_2 = frequency/threshold_freq_int;
        trueSkipCount_2 = frequency/threshold_freq_float;
        threshold_skip_2 = scale*(1-(trueSkipCount_2 - inverse_repeat_2));
        int value;

        //  change the amplitude and the offset
        for (int i = 0; i < 256; i++) {
            //  offset and amplitude to output value
            value = (Ampl/3)*pointer[i] +(127*(3-Ampl)/3)- ((offset/3)*127);

            //  write to the current wave buffer
            if (value > 255) {
                current_2_wave[i] = 255;
            } else if (value < 0) {
                current_2_wave[i] =  0;
            } else {
                current_2_wave[i] = value;
            }
        }
    }
}




/**
 * \brief One second interrupt - signal to read & transmit temp
 *
 * \param None
 *
 */
ISR(TIMER1_COMPA_vect) {
    one_second_interrup = 1;
}

/**
 * \brief Send the I2C Start Bit
 * \param Null
 * \retval Return 1 on failure, 3 on time out, 0 on success
 */
int StartSend(void) {
    //  send start bit
    TWCR = (1  <<  TWINT) | (1  <<  TWSTA) | (1  <<  TWEN);

    int counter = 0;
    //  wait for iterupt flag to be set
    while (!(TWCR & (1  <<  TWINT))) {
        counter++;
        if (counter > 10000) {
            //  took too long - return 3
            return 3;
        }
    }

    //  Check that start was sent successfully
    if ((TWSR & 0xF8) != TW_START && (TWSR & 0xF8) != TW_REP_START) {
        //  printf("Error Start Condition\n");
        return 1;
    }

    return 0;
}

/**
 * \brief Stops I2C
 * \param Null
 * \retval Null
 */
void StopSend(void) {
    TWCR = (1  <<  TWINT) | (1  <<  TWEN) | (1  <<  TWSTO);
}

/**
 * \brief Sends control byte
 * \param Read -1 read mode, 0 write mode, address to read
 * \retval Return 1 on failure, 3 on time out, 0 on success
 */
int ControlSend(int read, unsigned char addr) {
    addr = (addr  <<  1);

    int controlByte = addr + read;

    //  send
    TWDR = controlByte;
    TWCR = (1  <<  TWINT) | (1  <<  TWEN);

    int counter = 0;
    //  Wait for ACK or NACK response
    while (!(TWCR & (1  <<  TWINT))) {
        counter++;
        if (counter > 10000) {
            //  took too long - time out
            return 3;
        }
    }
    //  if not ACK then error
    if ((TWSR & 0xF8) != TW_MT_SLA_ACK && (TWSR & 0xF8) != TW_MR_SLA_ACK) {
        return 1;
    }

    return 0;
}

/**
 * \brief Sends data via I2C
 * \param Data to send
 * \retval Return 1 on failure, 3 on time out, 0 on success
 */
int DataSend(int data) {
    //  send through data
    TWDR = data;
    TWCR = (1  <<  TWINT) | (1  <<  TWEN);

    int counter;
    //  Wait for ACK or NACK
    while (!(TWCR & (1  <<  TWINT))) {
        counter++;
        if (counter > 10000) {
            return 3;
        }
    }

    //  if not ACK then error
    if ((TWSR & 0xF8) != TW_MT_DATA_ACK) {
        return 1;
    }

    return 0;
}


/**
 * \brief Get the temperature sensor data.
 * \param last is 1 if want the last byte, sends NACK
 * \retval Data read, 0 on timeout
 */
unsigned char DataGet(unsigned char last) {
    register unsigned char data = 0;

    if (last) {
        //  send NACK after receiving data
        TWCR &= (~(1  <<  TWEA));
        } else {
        TWCR |= (1  <<  TWEA);
    }

    TWCR |= (1  <<  TWINT);

    int counter = 0;
    //  Wait for data to be received
    while (!(TWCR & (1  <<  TWINT))) {
        counter++;
        if (counter > 10000) {
            return 0;
        }
    }

    if ((TWSR & 0xF8) == TW_MR_DATA_ACK || (TWSR & 0xF8) == TW_MR_DATA_NACK) {
        data = TWDR;
    }

    return data;
}



/**
 * \brief Read the temperature from the temp sensor
 * \param Address to communicate to
 * \retval Null
 */
void GetTemp(unsigned char addr) {
    int temp = 0;
    if (StartSend() != 3) {
            ControlSend(WRITE, addr);
            DataSend(0x51);
            StopSend();
            StartSend();
            ControlSend(WRITE, addr);
            DataSend(0xAA);
            StartSend();
            ControlSend(READ, addr);
            temperature_msb = DataGet(0);  //  msb of temperature
            int sigValue = temperature_msb/10;
            UartPutChar(sigValue+'0');
            UartPutChar((temperature_msb-(sigValue*10))+'0');
            UartPutChar('\n');
    }
}

/**
 * \brief Initializes the TWI clock
 * \param Null
 * \retval Null
 */
void I2cInit(void) {
    TWSR = 0;
    TWBR = ((F_CPU / SCL_CLOCK) - 16) / 2;
}


/**
 * \brief Initializes the timer interrupts
 * \param Null
 * \retval Null
 */
void InterruptInit(void) {
    //  sets the pre-scaler as 1
    TCCR1B |=  (1 << WGM12)|(1 << CS12)|(0 << CS11) |(1 << CS10);
    //  interrupt settings
    TCNT1 = 0;  //  init the counter
    OCR1A = 15000;  //  initialize compare register
    TIMSK1 |= (1  <<  OCIE1A);  //  enable the output compare interrupt


    //  interrupt setting for the sampling frequency
    TCCR0B |= (0  <<  CS02)|(1 << CS01) |(0 << CS00);  //  8 prescaller
    TCCR0A |= (1 << WGM01);
    TCNT0 = 0;
    OCR0A = 44;
    TIMSK0 |= (1 << OCIE0A);  //  enable the interrup
}


/**
 * \brief Initialize the waves
 * \param Null
 * \retval Null
 */
void WaveInit(void) {
    //  set the threshold frequency
    threshold_freq_int = 174;
    threshold_freq_float = 174.5;
    //  set the output ports for wave 1;
    DDRD |= (1 << DDD2| 1 << DDD3 | 1 << DDD4 | 1 << DDD5
    | 1 << DDD6 | 1 << DDD7);
    DDRC |= (1  <<  DDC0 | 1  <<  DDC1 | 1 <<  DDC2 | 1 <<  DDC3);

    DDRB |= (1 << DDB0 |1  <<  DDB1 | 1 << DDB2 | 1 << DDB3
    | 1 << DDB4 | 1 << DDB5);



    //  populate wave 1 lookup table
    PopulateWaveTable(waveOne.amplitude, waveOne.offset, waveOne.frequency,
    waveOne.wave_type, 1);

    //  populate wave 2 lookup table
    PopulateWaveTable(waveTwo.amplitude, waveTwo.offset, waveTwo.frequency,
    waveTwo.wave_type, 2);
}

/**
 * \brief Clears the recieve buffer
 * \param Null
 * \retval Null
 */
void ClearReceiveBuffer(void) {
        for (int cnt = 0; cnt < strlen(recieved_string); cnt++) {
            //  clear receive buffer
            recieved_string[cnt] = 0;
        }
        recieved_string_index = 0;
}


/**
 * \brief Sends ack or err
 * \param Null
 * \retval Null
 */
void SendReply(void) {
    if (format_error == 1) {  //  send err and clear buffer
        format_error = 0;
        for (int cnt = 0; cnt < strlen(err); cnt++) {  //  send "ERR\n" back
            UartPutChar(err[cnt]);
        }
        ClearReceiveBuffer();
    }

    if (send_ack == 1) {
        //  send ack and clear buffer, update lookup tables

        //  for high frequency waves, increase the sampling rate
        if (waveOne.frequency >= 6000 || waveTwo.frequency >= 6000) {
            OCR0A = 39;
            threshold_freq_int = 196;
            threshold_freq_float = 196.0;
            temp_display = 0;
            } else {
            OCR0A = 44;
            threshold_freq_int = 174;
            threshold_freq_float = 174.5;
            temp_display = 1;
        }


        // populate the new waves
        PopulateWaveTable(waveOne.amplitude, waveOne.offset,
        waveOne.frequency, waveOne.wave_type, 1);

        PopulateWaveTable(waveTwo.amplitude, waveTwo.offset,
        waveTwo.frequency, waveTwo.wave_type, 2);

        send_ack = 0;
        for (int cnt = 0; cnt < strlen(ack); cnt++) {  //  send "ACK\n" back
            UartPutChar(ack[cnt]);
        }
        ClearReceiveBuffer();
    }

    if (continuee == 1) {
        //  restart interrupts
        for (int cnt = 0; cnt < strlen(recieved_string); cnt++) {
            // clear buffer
            recieved_string[cnt] = 0;
        }
        TIMSK0 |= (1 << OCIE0A);
        TIMSK1 |= (1 << OCIE1A);
        recieved_string_index = 0;
        continuee = 0;
    }
}






/**
 * \brief 45kHz Sampling Rate Interrupt to output wave
 * \param Null
 * \retval Null
 */
ISR(TIMER0_COMPA_vect) {
    current_count_1++;  //  increment repeat count
    one_second_counter++;

    // wave 1 freq code
    if (repeat_count_1 != 0) {  //  if we are in repeat mode
        if (sample_count_1 <= threshold_rep_1) {
            //  if we are repeating base number of times.
            if (current_count_1 >= repeat_count_1) {
                wave_one_index++;
                sample_count_1++;
                current_count_1 = 0;
            }
        } else {
            //  if we are repeating base+1 times
            if (current_count_1 >= repeat_count_1+1) {
                wave_one_index++;
                sample_count_1++;
                current_count_1 = 0;
            }
        }
        if (sample_count_1 == scale) {
            //  reset scale
            sample_count_1 = 0;
        }
    } else if (inverse_repeat_1 != 0) {
        //  if we are in skip mode
        if (sample_count_1 <= threshold_skip_1) {
            //  skipping base number of times
            wave_one_index = wave_one_index + inverse_repeat_1;
            sample_count_1++;

        } else {
            //  skipping base + 1 number of times
            wave_one_index = wave_one_index + inverse_repeat_1 + 1;
            sample_count_1++;
        }
        if (sample_count_1 == scale) {
            //  reset scale
            sample_count_1 = 0;
        }
    }

    //  wave two frequency code
    current_count_2++;
    if (repeat_count_2 != 0) {  //  if we are in repeat mode
        if (sample_count_2 <= threshold_rep_2) {
            //  if we are repeating base number of times.
            if (current_count_2 >= repeat_count_2) {
                wave_two_index++;
                sample_count_2++;
                current_count_2 = 0;
            }
        } else {
            //  if we are repeating base+1 times
            if (current_count_2 >= repeat_count_2+1) {
                wave_two_index++;
                sample_count_2++;
                current_count_2 = 0;
            }
        }
        if (sample_count_2 == scale) {
            sample_count_2 = 0;
        }
    } else if (inverse_repeat_2 != 0) {
        //  if we are in skip mode
        if (sample_count_2 <= threshold_skip_2 || inverse_repeat_2 > 40) {
            //  skipping base number of times
            wave_two_index = wave_two_index + inverse_repeat_2;
            sample_count_2++;

        } else {
            //  skipping base + 1 number of times
            wave_two_index = wave_two_index + inverse_repeat_2 + 1;
            sample_count_2++;
        }
        if (sample_count_2 == scale) {
            //  reset scale
            sample_count_2 = 0;
        }
    }
    // get the wave 1 value
    wave_out_1 = current_wave[wave_one_index];
    // write value to ports
    temp_b = PORTB;
    temp_b &= 0b11000000;
    temp_b |= (wave_out_1 & 0b11111100)>>2;

    temp_c = PORTC;
    temp_c &= 0b11110011;  //  apply bit mas
    temp_c |= (wave_out_1 & 0b00000011) << 2;

    //  get wave 2 value
    wave_out_2 = current_2_wave[wave_two_index];

    //  write value to ports
    tempD = PORTD;
    tempD &= 0b00000011;
    tempD |= (wave_out_2 & 0b11111100);

    temp_c &= 0b11111100;
    temp_c |= (wave_out_2 & 0b00000011);

    PORTB = temp_b;
    PORTC = temp_c;
    PORTD = tempD;
}






























