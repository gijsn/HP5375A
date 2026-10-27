#include <Arduino.h>
#include <digitalWriteFast.h>
#include <pins_arduino.h>

// put the counter to EXT to make the keyboard work

#define OUTPUT_OC 0x0
/* Operation codes */
enum _opcodes {
    NORMALIZE = 0,
    MODULE = 2,      // works
    PLUG_IN = 3,
    SQRT_X = 5,
    SELF_CHECK = 6,  // works
    CALIBRATE = 7,
    DISP = 8,        // works
    TEN_X = 9,
    ADD = 10,
    SUBTRACT = 11,
    ONE_DIV_X = 12,
    LOAD = 13,
    DIVIDE = 14,
    MULTIPLY = 15,
    SWAP_SQS_X = 16,
    SWAP_INT_X = 17,
    MOVE_SQS_X_Y = 18,
    MOVE_INT_X_Y = 19,
    TWO_X = 20,
    SWAP_X_Y = 21,   // works
    CLEAR_X = 22,    // works
    MOVE_X_Y = 23,
    SWAP_PI_X = 24,
    SWAP_EXT_X = 25,
    MOVE_PI_X_Y = 26,
    MOVE_EXT_X_Y = 27,
    DIVIDE_X_10 = 28,
    SWAP_Z_X = 29,
    CLEAR_XYZ = 30,  // works
    MOVE_Z_X_Y = 31,

};

enum _digits {
    N1 = 1,
    N2 = 2,
    N3,
    N4,
    N5,
    N6,
    N7,
    N8,
    N9
};

// exponents and their pre-fix codes
enum _exponents {
    PREFIX = 1,
    G = 9,
    M = 6,
    K = 3,
    n = -9,
    m = -3,
    T = 12,
    f = -15,
    p = -12,
    ZERO = 0,
    DP = 15,

};

// Decimal point codes, numbered 1-13 from right to left
// Lamp number 10 is always lit
// enum _dp {
//   POS1
// }

// all logic works on falling edge!

#define X 10
enum _pins {
    _LOAD_E_KB = 22,  // pin 1, next load number on KD-lines is exponent
    // used in commands SWAP_EXT_X and MOVE_EXT_X_Y
    // these pins are purposefully set to A0-A3 as they are the first 4 pins of port-F, which is used to send the 13 digit number to the counter.
    _SQ1 = A0,       // pin 2, BCD line 1, b register (KB) to x register (Counter)
    _SQ2 = A1,       // pin 3, BCD line 2, b register (KB) to x register (Counter)
    _SQ4 = A2,       // pin 4, BCD line 4, b register (KB) to x register (Counter)
    _SQ8 = A3,       // pin 5, BCD line 8, b register (KB) to x register (Counter)
    KB_SIGN_E = 32,  // pin 6, polarity of exponent being loded from keyboard
    // Used in LOAD commands input is -8421 BCD and -8421 binary for exponent
    _KD1 = 23,       // pin 7, BCD line 1 from KB
    _KD2 = 25,       // pin 8, BCD line 2 from KB
    _KD4 = 27,       // pin 9, BCD line 4 from KB
    _KD8 = 29,       // pin 10, BCD line 8 from KB
    _DP_LATCH = 31,  // pin 11, decimal point latch, set low during LOAD and fixes DP at that point. remains low for subsequent characters. High when exponent is loaded
    // Control decimal point lamps
    DP1 = 33,          // pin 12, decimal point 1
    _DP2 = 35,         // pin 13, decimal point 2
    _DP4 = 37,
    DP8 = 39,
    // Program codes. Code must be held from EXT_START to when EXT_STEP_COMPLETE is received
    _EXT_PROG_A = 45,  // pin 16, program code A, LSB
    _EXT_PROG_B = 47,  // pin 17, program code B
    _EXT_PROG_C = 49,  // pin 19, program code C
    _EXT_PROG_D = 51,  // pin 20, program code D
    _EXT_PROG_E = 53,  // pin 21, program code E, MSB

    REMOTE_AB = X,     // pin 18, select module input A or B, high for A, low for B
    _EXT_START = 52,   // pin 22, EXT_START, low to start operation, pulse-synchronous with EXT_CLK, EXT_STEP_COMPLETE goes low when operation is complete
    _EXT_RESET = 50,   // pin 23, EXT_RESET, set low by mainframe when manual reset
    // TODO: understand this one
    _EXT_SHIFT13 = 48,       // pin 24, used for shifting storageword, 16us long and synchronous with EXT_CLK
    EXT_STEP_COMPL = 19,     // pin 25, counter is ready to accept commands again.
    _SW_RST = X,             // pin 26, active low, reset the counter from keyboard
    EXT_CLK = 18,            // pin 27, Clock to be used, 1MHz with 50% duty cycle
    _EXT_STORE = X,          // pin 30, low by counter when executing SWAP_EXT_X or MOVE_EXT_X_Y,
    _PRESET_SIGN_X_NEG = X,  // pin 31, negative pulse will set the sign of the x-register mantissa to a negative value
    // used in operations SWAP_EXT_X and MOVE_EXT_X_Y, SHIFT_13_EXT pulse train
    EXT_S1 = A8,           // pin 32
    EXT_S2 = A9,
    EXT_S4 = A10,
    EXT_S8 = A11,
    SIGN_X = X,            // pin 36, polarity of the x-register mantissa, high for positive, low for negative
    _DEFEAT_HOLD = 40,     // pin 38, defeat holdoff-restart timer associated with any display step
    _EXT_SW = 42,          // pin 39, EXT is enabled
    _PI_ERROR_DEFEAT = X,  // pin 41, error defeat of plug in module
    _DIRTY_ZEROS = X       // pin 42, low on this line indicates that x-register is 0
};

typedef struct {
    uint8_t number;
    uint8_t mode;
} IOpins_t;

#define NO_PINS 38
IOpins_t pin[NO_PINS] = {
    {_LOAD_E_KB, OUTPUT},
    {_SQ1, OUTPUT_OC},
    {_SQ2, OUTPUT_OC},
    {_SQ4, OUTPUT_OC},
    {_SQ8, OUTPUT_OC},
    {KB_SIGN_E, OUTPUT},
    {_KD1, OUTPUT_OC},  // open collector
    {_KD2, OUTPUT_OC},  // open collector
    {_KD4, OUTPUT_OC},  // open collector
    {_KD8, OUTPUT_OC},  // open collector
    {_DP_LATCH, OUTPUT},
    {DP1, OUTPUT_OC},   // open collector, temp fix
    {_DP2, OUTPUT_OC},  // open collector, temp fix
    {_DP4, OUTPUT_OC},  // open collector, temp fix
    {DP8, OUTPUT_OC},   // open collector, temp fix
    {_EXT_PROG_A, OUTPUT},
    {_EXT_PROG_B, OUTPUT},
    {_EXT_PROG_C, OUTPUT},
    {_EXT_PROG_D, OUTPUT},
    {_EXT_PROG_E, OUTPUT},
    {REMOTE_AB, OUTPUT},
    {_EXT_START, OUTPUT},
    {_EXT_RESET, OUTPUT},
    {_EXT_SHIFT13, INPUT},
    {EXT_STEP_COMPL, INPUT},
    {_SW_RST, OUTPUT},  // open collector
    {EXT_CLK, INPUT},
    {_EXT_STORE, INPUT},
    {_PRESET_SIGN_X_NEG, OUTPUT},
    {EXT_S1, INPUT},
    {EXT_S2, INPUT},
    {EXT_S4, INPUT},
    {EXT_S8, INPUT},
    {SIGN_X, INPUT},
    {_DEFEAT_HOLD, OUTPUT},
    {_EXT_SW, INPUT},
    {_PI_ERROR_DEFEAT, INPUT},
    {_DIRTY_ZEROS, INPUT}};

bool opcode_ext(uint8_t opcode);
void set_opcode(uint8_t opcode);
void prepare_ext_value(char* input);
void set_keyboard_digit(uint8_t digit, bool sign);
void set_keyboard_exponent(uint8_t exponent, bool sign);
void print_vals(uint8_t val1[], uint8_t recv1[]);
void wait_for_input();

bool external_module_enabled();
bool mainframe_reset();
void step(uint8_t opcode);

uint8_t ext_value[13] = {1, 0, 2, 0, 3, 0, 4, 0, 5, 6, 9, 12, 5};
// length needs to be 1 longer for correct loading, is corrected later
uint8_t ext_recv[14] = {0};

void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);
    Serial.println("St");

    for (int i = 0; i < NO_PINS; i++) {
        pinMode(pin[i].number, pin[i].mode);
        if (pin[i].mode >= OUTPUT) {
            digitalWrite(pin[i].number, HIGH);
        }
    }
    // set full K register (EXT_S1-EXT_S8) to input
    // DDRK = 0x0;
    digitalWriteFast(_LOAD_E_KB, HIGH);
    digitalWriteFast(KB_SIGN_E, LOW);
    digitalWriteFast(_PRESET_SIGN_X_NEG, HIGH);
    // this messes up the dot on all operations
    // digitalWriteFast(DP1, LOW);
    // digitalWriteFast(_DP2, HIGH);
    // digitalWriteFast(_DP4, HIGH);
    // digitalWriteFast(DP8, LOW);
    // digitalWriteFast(_DP_LATCH, HIGH);
    for (int i = 0; i < 13; i++) {
        ext_value[i] = ~ext_value[i];
    }
    int i = 0;
    while (true) {
        // does not work?
        // step(CLEAR_X);
        // delay(10);
        // step(DISP);
        // wait_for_input();
        // // pinModeFast(DP1, OUTPUT);
        // // pinModeFast(_DP2, OUTPUT);
        // // pinModeFast(_DP4, OUTPUT);
        // // pinModeFast(DP8, OUTPUT);
        // digitalWriteFast(_LOAD_E_KB, LOW);
        // set_keyboard_digit(2, false);
        // digitalWriteFast(_DEFEAT_HOLD, LOW);
        // delay(10);
        // // digitalWriteFast(DP1, HIGH);
        // // digitalWriteFast(_DP2, HIGH);
        // // digitalWriteFast(_DP4, LOW);
        // // digitalWriteFast(DP8, HIGH);
        // step(LOAD);
        // wait_for_input();
        // digitalWriteFast(_DEFEAT_HOLD, HIGH);
        // continue;

        // pinModeFast(DP1, INPUT);
        // pinModeFast(_DP2, INPUT);
        // pinModeFast(_DP4, INPUT);
        // pinModeFast(DP8, INPUT);
        /*
                // set_keyboard_digit(4, true);
                digitalWriteFast(_LOAD_E_KB, LOW);
                digitalWriteFast(KB_SIGN_E, LOW);
                // digitalWriteFast(_DEFEAT_HOLD, LOW);
                // step(LOAD);
                // step(DISP);
                // wait_for_input();
                step(LOAD);
                // step(DISP);
                wait_for_input();

                set_keyboard_digit(2, true);
                step(LOAD);
                // step(DISP);
                wait_for_input();

                digitalWriteFast(_LOAD_E_KB, LOW);
                digitalWriteFast(KB_SIGN_E, LOW);
                step(LOAD);
                // step(DISP);
                wait_for_input();

                set_keyboard_digit(3, true);
                step(LOAD);
                // step(DISP);
                wait_for_input();
        */

        // works, but does not allow setting of Y?
        Serial.println("CLEAR_XYZ");
        step(CLEAR_XYZ);

        // works
        // step(MODULE);

        // works
        // step(SWAP_X_Y);

        // works with parser
        // seems decimal point is 1 position to the left in real life
        Serial.println("prepare_ext_value -1.35e0 & display");
        prepare_ext_value("-1.2345678912e1");
        step(SWAP_EXT_X);
        step(DISP);
        wait_for_input();

        // does not work
        // prepare_ext_value("7.00e1");
        // step(MOVE_EXT_X_Y);
        // step(DISP);
        // wait_for_input();
        // digitalWriteFast(REMOTE_AB, LOW);
        // step(MODULE);
        // step(DISP);
        // wait_for_input();

        // works, but negated
        Serial.println("prepare_ext_value 70k & display");
        prepare_ext_value("7.00e3");
        step(SWAP_EXT_X);
        step(DISP);
        wait_for_input();

        // step(SWAP_INT_X);
        // step(DISP);
        // wait_for_input();

        // step(SWAP_INT_X);
        // step(DISP);
        // wait_for_input();

        // does not work
        // step(SWAP_INT_X);
        // step(DISP);
        // wait_for_input();

        // does not work
        // step(SWAP_INT_X);
        // step(DISP);
        // wait_for_input();

        // this works
        // Serial.println("TEN_X");
        // step(TEN_X);
        // step(DISP);
        // wait_for_input();

        // step(SWAP_X_Y);
        // step(DISP);
        // wait_for_input();

        // this does not work?
        //  Serial.println("DIVIDE_TEN_X");
        //  step(DIVIDE_X_10);
        //  step(DISP);
        //  wait_for_input();

        // this works
        // Serial.println("ADD");
        // step(ONE_DIV_X);
        // step(DISP);
        // wait_for_input();

        // this does something?
        // Serial.println("DIVIDE");
        // step(DIVIDE);
        // step(DISP);

        // this results in error
        // step(SQRT_X);
        // step(DISP);
        // wait_for_input();

        // does not work
        // set_keyboard_digit(2, true);
        // step(LOAD);

        // does not work
        // set_keyboard_exponent(1, false);
        // step(LOAD);
        // digitalWriteFast(_DEFEAT_HOLD, LOW);
        // step(DISP);

        // does not work

        // set_keyboard_exponent(3, true);
        // step(LOAD);

        // works
        step(DISP);
        wait_for_input();
        // does not produce the correct result
        // step(SQRT_X);

        Serial.print("ext: ");
        Serial.print(external_module_enabled());
        Serial.print(", rst: ");
        Serial.println(mainframe_reset());
        Serial.println("-----------------------");
        delay(1000);
        i++;
        i %= 16;
    }
}

bool opcode_ext(uint8_t opcode) {
    switch (opcode) {
        case SWAP_EXT_X:
        case MOVE_EXT_X_Y:
            return true;
    }
    return false;
}

void set_opcode(uint8_t opcode) {
    // digital write fast needs a constant expression to compile
    // logic is inverted
    if ((opcode & 1)) {
        digitalWriteFast(_EXT_PROG_A, LOW)
    } else {
        digitalWriteFast(_EXT_PROG_A, HIGH);
    }
    if (((opcode >> 1) & 1)) {
        digitalWriteFast(_EXT_PROG_B, LOW);
    } else {
        digitalWriteFast(_EXT_PROG_B, HIGH);
    }
    if (((opcode >> 2) & 1)) {
        digitalWriteFast(_EXT_PROG_C, LOW);
    } else {
        digitalWriteFast(_EXT_PROG_C, HIGH);
    }
    if (((opcode >> 3) & 1)) {
        digitalWriteFast(_EXT_PROG_D, LOW);
    } else {
        digitalWriteFast(_EXT_PROG_D, HIGH);
    }
    if (((opcode >> 4) & 1)) {
        digitalWriteFast(_EXT_PROG_E, LOW);
    } else {
        digitalWriteFast(_EXT_PROG_E, HIGH);
    }
    Serial.print("opcode: ");
    Serial.print(!((opcode >> 4) & 1));
    Serial.print(!((opcode >> 3) & 1));
    Serial.print(!((opcode >> 2) & 1));
    Serial.print(!((opcode >> 1) & 1));
    Serial.print(!(opcode & 1));
    Serial.println();
}

void set_keyboard_exponent(uint8_t digit, bool sign) {
    pinModeFast(_KD1, OUTPUT);
    pinModeFast(_KD2, OUTPUT);
    pinModeFast(_KD4, OUTPUT);
    pinModeFast(_KD8, OUTPUT);
    digitalWriteFast(_LOAD_E_KB, LOW);
    delay(100);
    if (sign) {
        digitalWriteFast(KB_SIGN_E, LOW);
    } else {
        digitalWriteFast(KB_SIGN_E, HIGH);
    }

    if ((digit & 1)) {
        digitalWriteFast(_KD1, LOW)
    } else {
        digitalWriteFast(_KD1, HIGH);
    }
    if (((digit >> 1) & 1)) {
        digitalWriteFast(_KD2, LOW);
    } else {
        digitalWriteFast(_KD2, HIGH);
    }
    if (((digit >> 2) & 1)) {
        digitalWriteFast(_KD4, LOW);
    } else {
        digitalWriteFast(_KD4, HIGH);
    }
    if (((digit >> 3) & 1)) {
        digitalWriteFast(_KD8, LOW);
    } else {
        digitalWriteFast(_KD8, HIGH);
    }

    Serial.print("exponent: ");
    Serial.print(((digit >> 3) & 1));
    Serial.print(((digit >> 2) & 1));
    Serial.print(((digit >> 1) & 1));
    Serial.print((digit & 1));
    Serial.println();
}

void set_negative() {
    digitalWriteFast(_PRESET_SIGN_X_NEG, HIGH);
    step(LOAD);
    step(LOAD);
}

void set_keyboard_digit(uint8_t digit, bool sign) {
    if ((digit & 1)) {
        digitalWriteFast(_KD1, LOW)
    } else {
        digitalWriteFast(_KD1, HIGH);
    }
    if (((digit >> 1) & 1)) {
        digitalWriteFast(_KD2, LOW);
    } else {
        digitalWriteFast(_KD2, HIGH);
    }
    if (((digit >> 2) & 1)) {
        digitalWriteFast(_KD4, LOW);
    } else {
        digitalWriteFast(_KD4, HIGH);
    }
    if (((digit >> 3) & 1)) {
        digitalWriteFast(_KD8, LOW);
    } else {
        digitalWriteFast(_KD8, HIGH);
    }
    Serial.print("digit: ");
    Serial.print(!((digit >> 3) & 1));
    Serial.print(!((digit >> 2) & 1));
    Serial.print(!((digit >> 1) & 1));
    Serial.print(!(digit & 1));
    Serial.println();
}
// accepts input in the form of "-?d+.d+e-?d", like -5.4e-3
void prepare_ext_value(char* input) {
    uint8_t len_input = strlen(input);
    char* cpy_input = (char*)malloc(sizeof(char) * len_input);
    strcpy(cpy_input, input);
    uint8_t exponent = 0;
    uint8_t tmp_ext_value[13] = {0};
    // check sign of mantissa
    if (*cpy_input == '-') {
        // decimal sign is negative
        cpy_input++;
        tmp_ext_value[12] &= ~1;
    } else {
        // decimal sign is positive
        tmp_ext_value[12] |= 1;
    }
    char* decimal = strtok(cpy_input, ".");
    uint8_t len_decimal = strlen(decimal);
    if (len_decimal == 0) {
        tmp_ext_value[10] = 0;
        tmp_ext_value[9] = 0;
    } else {
        tmp_ext_value[10] = atoi(decimal) / 10;
        tmp_ext_value[9] = atoi(decimal) % 10;
        exponent += max(len_decimal - 2, 0);
    }
    // capture fraction
    char* fraction = strtok(NULL, "e");
    unsigned long val = strtoul(fraction, NULL, 10);
    // reverse number to correct for mantissa behavior (10s are bigger in value than 100s behind .)
    unsigned long rev_val = 0;
    while (val > 0) {
        rev_val = rev_val * 10 + val % 10;
        val = val / 10;
    }
    tmp_ext_value[0] = rev_val / 100000000;
    rev_val %= 100000000;
    tmp_ext_value[1] = rev_val / 10000000;
    rev_val %= 10000000;
    tmp_ext_value[2] = rev_val / 1000000;
    rev_val %= 1000000;
    tmp_ext_value[3] = rev_val / 100000;
    rev_val %= 100000;
    tmp_ext_value[4] = rev_val / 10000;
    rev_val %= 10000;
    tmp_ext_value[5] = rev_val / 1000;
    rev_val %= 1000;
    tmp_ext_value[6] = rev_val / 100;
    rev_val %= 100;
    tmp_ext_value[7] = rev_val / 10;
    tmp_ext_value[8] = rev_val % 10;

    // remaining characters are exponent
    char* sign_exp = strtok(NULL, "");
    if (*sign_exp == '-') {
        // exponent sign is negative
        tmp_ext_value[12] &= ~4;
    } else {
        // exponent sign is positive
        tmp_ext_value[12] |= 4;
    }

    exponent += atoi(sign_exp++);
    tmp_ext_value[11] = exponent % 16;
    tmp_ext_value[12] |= (exponent / 16) << 4;

    char tmp[100];
    sprintf(tmp, "%i %i %i %i %i %i %i %i %i %i %i %i %i", tmp_ext_value[0], tmp_ext_value[1], tmp_ext_value[2], tmp_ext_value[3], tmp_ext_value[4], tmp_ext_value[5], tmp_ext_value[6], tmp_ext_value[7], tmp_ext_value[8], tmp_ext_value[9], tmp_ext_value[10], tmp_ext_value[11], tmp_ext_value[12]);
    Serial.print("Prepare val: ");
    Serial.println(tmp);
    Serial.println("           LSB                 .  ");
    // negate value to make -8421 BCD representation
    for (int i = 0; i < 13; i++) {
        tmp_ext_value[i] = ~tmp_ext_value[i];
    }
    memcpy(ext_value, tmp_ext_value, 13);
}

void print_vals(uint8_t val1[], uint8_t recv1[]) {
    char tmp[100];
    sprintf(tmp, "%i %i %i %i %i %i %i %i %i %i %i %i %i", (uint8_t)~val1[0], (uint8_t)~val1[1], (uint8_t)~val1[2], (uint8_t)~val1[3], (uint8_t)~val1[4], (uint8_t)~val1[5], (uint8_t)~val1[6], (uint8_t)~val1[7], (uint8_t)~val1[8], (uint8_t)~val1[9], (uint8_t)~val1[10], (uint8_t)~val1[11], (uint8_t)~val1[12]);
    Serial.print("sent: ");
    Serial.println(tmp);
    sprintf(tmp, "%i %i %i %i %i %i %i %i %i %i %i %i %i", recv1[0], recv1[1], recv1[2], recv1[3], recv1[4], recv1[5], recv1[6], recv1[7], recv1[8], recv1[9], recv1[10], recv1[11], recv1[12]);
    Serial.print("recv: ");
    Serial.println(tmp);
}

void step(uint8_t opcode) {
    set_opcode(opcode);
    bool ext = opcode_ext(opcode);
    uint8_t ctr = 0;
    // {1, 2, 3, 4, 5, 6, 7, 8, 9, 1, 1, 1, 1, 1};
    pinModeFast(_SQ1, OUTPUT);
    pinModeFast(_SQ2, OUTPUT);
    pinModeFast(_SQ4, OUTPUT);
    pinModeFast(_SQ8, OUTPUT);

    cli();
    // catch rising edge of clock
    while (digitalReadFast(EXT_CLK)) {
    }
    while (!digitalReadFast(EXT_CLK)) {
    }
    // sync with 1 MHz clock, we need to wait 10 instructions (at 16 MHz) to go with the next clock pulse
    __asm__("nop\n\t");
    __asm__("nop\n\t");
    __asm__("nop\n\t");
    __asm__("nop\n\t");
    __asm__("nop\n\t");
    __asm__("nop\n\t");
    __asm__("nop\n\t");
    __asm__("nop\n\t");
    __asm__("nop\n\t");
    __asm__("nop\n\t");
    digitalWriteFast(_EXT_START, LOW);
    // wait for 8 additonal nops (digitalWriteFast is 2 instructions) to start the pulses
    __asm__("nop\n\t");
    __asm__("nop\n\t");
    __asm__("nop\n\t");
    __asm__("nop\n\t");
    __asm__("nop\n\t");
    __asm__("nop\n\t");
    __asm__("nop\n\t");
    __asm__("nop\n\t");
    // TODO: implement 13 pulses for external loading
    digitalWriteFast(_EXT_START, HIGH);
    // catch rising edge of ext step complete
    if (ext) {
        // set the first 4 pins of PORTF to the SQ-values in 1 instruction
        PORTF = ext_value[ctr++];
        // await for first falling edge of _EXT_SHIFT13, wait while it is high
        while (((PINL) & (2UL))) {
        }
        // with -OFast, we get all instructions duplicated 13 times to skip the while loop
        // The following takes 10 instructions
        while (ctr < 13) {
            ext_recv[ctr] = ((PINK));
            PORTF = ext_value[ctr++];
            __asm__("nop\n\t");
            __asm__("nop\n\t");
            __asm__("nop\n\t");
            __asm__("nop\n\t");
            __asm__("nop\n\t");
            __asm__("nop\n\t");
            __asm__("nop\n\t");
            __asm__("nop\n\t");
            __asm__("nop\n\t");
        }
        ext_recv[ctr] = ((PINK));
        PORTF = 0xF;
        pinModeFast(_SQ1, OUTPUT_OC);
        pinModeFast(_SQ2, OUTPUT_OC);
        pinModeFast(_SQ4, OUTPUT_OC);
        pinModeFast(_SQ8, OUTPUT_OC);
    }
    while (!((PINC) & (4UL))) {
    }
    sei();
    if (ext) {
        for (int i = 0; i < 13; i++) {
            ext_recv[i] = ext_recv[i + 1] & 0xF;
        }
        print_vals(ext_value, ext_recv);
    }
    delay(10);
    //  reset state
    //  cannot yet access val & recv here without impacting performance during transmit and read
    digitalWriteFast(KB_SIGN_E, LOW);
    digitalWriteFast(_LOAD_E_KB, HIGH);

    pinModeFast(_KD1, OUTPUT_OC);
    pinModeFast(_KD2, OUTPUT_OC);
    pinModeFast(_KD4, OUTPUT_OC);
    pinModeFast(_KD8, OUTPUT_OC);

    digitalWriteFast(_EXT_PROG_A, HIGH);
    digitalWriteFast(_EXT_PROG_B, HIGH);
    digitalWriteFast(_EXT_PROG_C, HIGH);
    digitalWriteFast(_EXT_PROG_D, HIGH);
    digitalWriteFast(_EXT_PROG_E, HIGH);
}
void loop() {
    // put your main code here, to run repeatedly:
}

bool external_module_enabled() {
    if (digitalRead(_EXT_SW) == LOW) {
        return true;
    }
    return false;
}

bool mainframe_reset() {
    if (digitalReadFast(_EXT_RESET) == LOW) {
        return true;
    }
    return false;
}

void select_module_input(bool A) {
    if (A) {
        digitalWriteFast(REMOTE_AB, HIGH);
    }
}
void wait_for_input() {
    while (Serial.available() == 0) {
        delay(1);
    }
    while (Serial.available() > 0) {
        Serial.read();
    }
}
