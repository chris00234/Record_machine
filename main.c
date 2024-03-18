/**
 * main.c
 * project6 - Tone Player / Recorder
 */
#include <stdint.h>
#include <stdio.h>
//#include <stdbool.h>
#include "tm4c123gh6pm.h" //import header files for intializations



#define RS 0x04
#define RW 0x08
#define E  0x10
#define RCGC2_GPIOA 0x01
#define RCGC2_GPIOC 0x04
#define RCGC2_GPIOE 0x10
#define RCGC2_GPIOF 0x20
#define RCGC2_GPIOB 0x00000002

#define false 0
#define true 1

//GPIO port C and E address with offset to communicate with keypad
#define GPIO_PORTC_DIR_R (*((volatile unsigned long *)0x40006400)) //o
#define GPIO_PORTC_DEN_R (*((volatile unsigned long *)0x4000651C))//o
#define GPIO_PORTC_PUR   (*((volatile unsigned long *)0x40006510))//o
#define GPIO_PORTE_DIR_R (*((volatile unsigned long *)0x40024400))//o
#define GPIO_PORTE_DEN_R (*((volatile unsigned long *)0x4002451C))//o
#define GPIO_PORTE_PDR   (*((volatile unsigned long *)0x40024514)) // Pull-Down Register for Port E

//port C and E data
#define GPIO_PORTE_DATA (*((volatile unsigned long *)(0x40024000 + 0x3FC)))
#define GPIO_PORTC_DATA (*((volatile unsigned long *)(0x40006000 + 0x3FC)))

// Correct base address for I2C3
#define I2C3_BASE_ADDRESS 0x40060000



//-------------------------------------------------------------------------------
// Define system clock frequency
#define SYSTEM_CLOCK 16000000UL

#define SYSCTL_RCGCGPIO_PORTB (1U << 1)
#define SYSCTL_RCGCPWM_PWM0 (1U << 0)
//---------------------------------------------------------- ---------------------



#define MAX_NOTES 64 // Adjust based on EEPROM size and requirements
uint8_t recordedNotes[MAX_NOTES];
uint8_t noteCount = 0;

//mapping from keypad
char key_map[4][4] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
};
const uint32_t toneFreq[8] = {
    261.6, //C
    293.7, //D
    329.6, // E
    349.2, // F
    392.0, // G
    440.0, // A
    493.9, // B
    523.2 //C#
};
typedef enum {PlayMode, RecordMode, PlaybackMode} PlayerState; //state definitions
PlayerState currentState = PlayMode;
long A = 0; //one input

char numberStr[16]; //store max 16bits

void delayMs(int n){
    int i, j;
    for(i = 0 ; i < n; i++)
        for(j = 0; j < 3180; j++) {}
}
void keypad_init(void)
{
    // Enable clock to GPIO Port C and Port E
    SYSCTL_RCGCGPIO_R |= 0x14; // 0001 0100
    // Configure Port E pins as output for keypad rows
    GPIO_PORTE_DIR_R |= 0x0F; // PE0-3 as output to make 1 bit

    GPIO_PORTE_DEN_R |= 0x0F; // Enable digital function

    // Configure Port C pins as input with pull-up for keypad columns
    GPIO_PORTC_DIR_R &= ~0xF0; // PC4-7 as input to make 0 bit
    GPIO_PORTC_PUR |= 0xF0;  // Enable pull-up resistors on PC4-7
    GPIO_PORTC_DEN_R |= 0xF0;  // Enable digital function
}

char keypad_getKey(void){
    int row, col, i;
    char key = ' ';
    for (row = 0; row < 4; row++) {
        GPIO_PORTE_DATA = ~(1 << row); // one row per time ex) row = 1, 1111 1101 because of pullup register config
        for (col = 0; col < 4; col++) {
            if (!(GPIO_PORTC_DATA & (1 << (col + 4)))) { // check if the column is pressed, since row already specified. make that bit low
                GPIO_PORTE_DATA = 0x0F;
                key = key_map[row][col];
                while ((!GPIO_PORTC_DATA & (1 << (col + 4)))); // Wait for key release

                return key;
                //small delay for debouncing
                 // Transmit the pressed key
            }
        }
        GPIO_PORTE_DATA = 0x0F; // set it back to what it was
    }
    return ' '; // when No key pressed
}

void initSpeaker(){

    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_PORTB; //enable clock for Port B6
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_PWM0; //enable clock for PWM0
    // Enable PB6 for PWM functionality
    GPIO_PORTB_AFSEL_R |= (1<<6); // enable alternate function on PB6

    GPIO_PORTB_PCTL_R |= 0x04000000; // configure PB6 as PWM output

    GPIO_PORTB_DEN_R |= (1<<6); // enable digital on PB6

    // configure PWM module 0, generator 0 for down-count mode
    PWM0_0_CTL_R = 0x00000000; // disable Generator 0 during setup
    PWM0_0_GENA_R = 0x0000008C; // action to be taken on the zero and load
    PWM0_0_LOAD_R = 16000; //16Mhz / 16 * 1000
    PWM0_0_CMPA_R = 16000 - 16000 * 0.6; // start with 60% duty cycle
    PWM0_0_CTL_R |= 0x00000001; // enable Generator 0
    PWM0_ENABLE_R |= 0x00000001; //enable PWM output on the corresponding PWM0 pin
}

void playTone(uint32_t frequency) {
    uint32_t loadValue;

    // Calculate the load value based on the desired frequency
    loadValue = (SYSTEM_CLOCK / frequency) - 1;
    PWM0_0_LOAD_R = loadValue; // Set the load value for the PWM timer
    PWM0_0_CMPB_R = loadValue / 2; // 50% duty cycle
    PWM0_ENABLE_R |= 0x01; // Enable PWM output
}

void stopTone(void) {
    PWM0_ENABLE_R &= ~0x01; // Disable PWM output to stop the tone
    //inf loop
}
//--------------------------------------------------------------------

void I2C1_Init(void) {
    SYSCTL_RCGCI2C_R = (1 << 1); // Enable I2C1
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0;

    while((I2C1_MCS_R & (1 << 0)) != 0);

// Enable alternate function on PA6 (SCL) and PA7 (SDA)
    GPIO_PORTA_AFSEL_R |= (1 << 6) | (1 << 7);

    // Enable open drain on PA7 (SDA)
    GPIO_PORTA_ODR_R |= (1 << 7);

    // Enable digital function on PA6 and PA7
    GPIO_PORTA_DEN_R |= (1 << 6) | (1 << 7);

    // Configure PA6 and PA7 for I2C use
    GPIO_PORTA_PCTL_R &= ~0xFF000000; // Clear PMC6 and PMC7 fields
    GPIO_PORTA_PCTL_R |= (3 << 24) | (3 << 28); // Set PMC6 and PMC7 fields to 3 for I2C1 SCL and SDA (Note: Correction here)


    I2C1_MCR_R = (1 << 4); // Master function enable
    I2C1_MTPR_R = 0x07; // SCL clock speed of 100kbps assuming 16MHz system clock
//    setSlaveAddress(0x50);
//    setRW(0);
//    writeByte(0x00, (1 << 0) | (1 << 1)); // set as start
//    writeByte(0x00, (1 << 0) | (1 << 2)); // set as stop
    //delay function here
}


#define EEPROM_I2C_ADDRESS 0xA0


void writeByte(uint8_t dataByte, uint8_t addr) {

    //set write mode
    I2C1_MSA_R  = 0xA0;
    //High address 0x00000000
    I2C1_MDR_R = 0x00000000;
    //Start condition 0b00000011
    I2C1_MCS_R = 0b00000011;
    //Wait for busy  bit
    while (I2C1_MCS_R & I2C_MCS_BUSY);

    //Low address 0xaddress ex. 0x1 0x2 0x3 etc
    I2C1_MDR_R = addr;
    //Run condition ob00000001
    I2C1_MCS_R = 0b00000001;

    //wait for busy
    while (I2C1_MCS_R & I2C_MCS_BUSY);
    //data in reg
    I2C1_MDR_R = dataByte;
    //wait for busy
    while (I2C1_MCS_R & I2C_MCS_BUSY);
    //Stop condition 0b00000101
    I2C1_MCS_R = 0b00000101;
    //delay(10000)
    delayMs(50);

}

uint8_t readByte(uint8_t addr) {
    //set write mode
    I2C1_MSA_R = 0xA0;
    //High address 0x00000000
    I2C1_MDR_R = 0x00000000;
    while (I2C1_MCS_R & I2C_MCS_BUSY);

    //Start condition 0b00000011
    I2C1_MCS_R = 0b00000011;
    //Wait for busy  bit
    while (I2C1_MCS_R & I2C_MCS_BUSY);
    //Low address 0xaddress ex. 0x1 0x2 0x3 etc
    I2C1_MDR_R = addr;
    //Run condition ob00000001
    I2C1_MCS_R = 0b00000001;
    while (I2C1_MCS_R & I2C_MCS_BUSY);

    //slave address set but LSB 1 0xA1
    I2C1_MSA_R = 0xA1;
    //Start 0b0000011
    I2C1_MCS_R = 0b00000011;
    while (I2C1_MCS_R & I2C_MCS_BUSY);

    //Read from MDR
    uint8_t data = I2C1_MDR_R;
    while (I2C1_MCS_R & I2C_MCS_BUSY);

    //Stop
    I2C1_MCS_R = 0b00000101;




    // If no error, return the data received from the slave device
    return data;
}




// RecordNote updates to use EEPROM for storage
void RecordNote(uint8_t note) {
    if (noteCount < MAX_NOTES) {
        writeByte(note, noteCount);
        noteCount++;
    }
}

// PlaybackNotes updates to use EEPROM for retrieving notes
void PlaybackNotes(void) {
    uint8_t i;
    for (i = 0; i < noteCount; i++) {
        uint8_t note = readByte(i);
        if (note >= 0 && note < 8) {
            playTone(toneFreq[note]);
            delayMs(80); // Play each note for 1 second
            stopTone();

        }
    }
    noteCount = 0;
}



void EEPROM_Init(void) {
    SYSCTL_RCGCEEPROM_R |= 1; // Enable clock to EEPROM
    while (EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING); // Wait for EEPROM to be ready
}


void LED_Init(void) {
    SYSCTL_RCGCGPIO_R |= 0x20; // Enable clock to GPIOF for LEDs
    GPIO_PORTF_DIR_R |= 0x0E; // PF1, PF2, PF3 as output
    GPIO_PORTF_DEN_R |= 0x0E; // Enable digital function
}

void SetLED(uint8_t mode) {
    GPIO_PORTF_DATA_R &= ~0x0E; // Turn off all LEDs
    switch (mode) {
        case PlayMode:
            GPIO_PORTF_DATA_R |= 0x02; // Turn on LED for Play Mode
            break;
        case RecordMode:
            GPIO_PORTF_DATA_R |= 0x04; // Turn on LED for Record Mode
            break;
        case PlaybackMode:
            GPIO_PORTF_DATA_R |= 0x08; // Turn on LED for Playback Mode
            break;
    }
}


int main(void) {
    // Initialize peripherals
    I2C1_Init(); // Initialize I2C for external EEPROM
    EEPROM_Init(); // If still needed for other reasons

    LED_Init();
    keypad_init();
    initSpeaker();
    PlayerState currentState = PlayMode;
    SetLED(currentState);

    while (1) {
        char key = keypad_getKey();
        switch (currentState) {
            case PlayMode:
                if (key >= '1' && key <= '8') {
                    playTone(toneFreq[key - '1']);
                }
                else if (key == 'A') {
                    currentState = PlayMode;
                }
                else if (key == 'B') {
                    currentState = RecordMode;
                    noteCount = 0; // Reset recorded notes
                }
                else if (key == 'D') {
                    currentState = PlaybackMode;
                    SetLED(currentState);
                    PlaybackNotes();
                }
                else{
                    stopTone(); // Stop playing the tone
                }
                break;
            case RecordMode:
                if (key >= '1' && key <= '8') {
                    playTone(toneFreq[key - '1']);
                    RecordNote(key - '1');
                }
                else if (key == 'A') {
                    currentState = PlayMode;
                }
                else if (key == 'B') {
                    currentState = RecordMode;
                    //noteCount = 0; // Reset recorded notes
                }
                else if (key == 'D') {
                    currentState = PlaybackMode;
                    SetLED(currentState);
                    PlaybackNotes();
                }
                else{
                    stopTone(); // Stop playing the tone
                }

                break;
            case PlaybackMode:

                if (key == 'A') {
                    currentState = PlayMode;
                }
                else if (key == 'B') {
                    currentState = RecordMode;
                    //noteCount = 0; // Reset recorded notes
                }
                else if (key == 'D') {
                    currentState = PlaybackMode;
                    PlaybackNotes();
                }
                // Playback is triggered once when entering the mode
                break;
        }
        SetLED(currentState);
    }
}




