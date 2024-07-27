#include <Arduino.h>
#include <digitalWriteFast.h>
#include <pins_arduino.h>

#define OUTPUT_OC 0x3
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

#define X 10
enum _pins {
    LOAD_E_KB = 22,
    SQ1 = A0,
    SQ2 = A1,
    SQ4 = A2,
    SQ8 = A3,
    KB_SIGN_E = 32,
    KD1 = 23,
    KD2 = 25,
    KD4 = 27,
    KD8 = 29,
    DP_LATCH = 31,
    DP1 = 33,
    DP2 = 35,
    DP4 = 37,
    DP8 = 39,
    EXT_PROG_A = 45,
    EXT_PROG_B = 47,
    EXT_PROG_C = 49,
    EXT_PROG_D = 51,
    EXT_PROG_E = 53,
    REMOTE_AB = X,
    EXT_START = 52,
    EXT_RESET = 50,
    EXT_SHIFT13 = 48,
    EXT_STEP_COMPL = 19,
    SW_RST = X,
    EXT_CLK = 18,
    EXT_STORE = X,
    PRESET_SIGNX_NEG = X,
    EXT_S1 = A8,
    EXT_S2 = A9,
    EXT_S4 = A10,
    EXT_S8 = A11,
    SIGNX = X,
    DEFEAT_HOLD = 40,
    EXT_SW = 42,
    PI_ERROR_DEFEAT = X,
    DIRTY_ZEROS = X
};

typedef struct {
    uint8_t number;
    uint8_t mode;
} IOpins_t;

#define NO_PINS 38
IOpins_t pin[NO_PINS] = {
    {LOAD_E_KB, OUTPUT},
    {SQ1, OUTPUT},
    {SQ2, OUTPUT},
    {SQ4, OUTPUT},
    {SQ8, OUTPUT},
    {KB_SIGN_E, OUTPUT},
    {KD1, OUTPUT},     // open collector
    {KD2, OUTPUT},     // open collector
    {KD4, OUTPUT},     // open collector
    {KD8, OUTPUT},     // open collector
    {DP_LATCH, OUTPUT},
    {DP1, INPUT},      // open collector, temp fix
    {DP2, INPUT},      // open collector, temp fix
    {DP4, INPUT},      // open collector, temp fix
    {DP8, INPUT},      // open collector, temp fix
    {EXT_PROG_A, OUTPUT},
    {EXT_PROG_B, OUTPUT},
    {EXT_PROG_C, OUTPUT},
    {EXT_PROG_D, OUTPUT},
    {EXT_PROG_E, OUTPUT},
    {REMOTE_AB, OUTPUT},
    {EXT_START, OUTPUT},
    {EXT_RESET, OUTPUT},
    {EXT_SHIFT13, INPUT},
    {EXT_STEP_COMPL, INPUT},
    {SW_RST, OUTPUT},  // open collector
    {EXT_CLK, INPUT},
    {EXT_STORE, INPUT},
    {PRESET_SIGNX_NEG, OUTPUT},
    {EXT_S1, INPUT},
    {EXT_S2, INPUT},
    {EXT_S4, INPUT},
    {EXT_S8, INPUT},
    {SIGNX, INPUT},
    {DEFEAT_HOLD, OUTPUT},
    {EXT_SW, INPUT},
    {PI_ERROR_DEFEAT, INPUT},
    {DIRTY_ZEROS, INPUT}};

bool opcode_ext(uint8_t opcode);
void set_opcode(uint8_t opcode);
void prepare_ext_value(char *input);
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
    Serial.println("Starting HP 5375A keyboard");

    for (int i = 0; i < NO_PINS; i++) {
        pinMode(pin[i].number, pin[i].mode);
        if (pin[i].mode == OUTPUT) {
            digitalWrite(pin[i].number, HIGH);
        }
    }
    DDRK = 0x0;
    digitalWriteFast(LOAD_E_KB, HIGH);
    digitalWriteFast(KB_SIGN_E, LOW);
    digitalWriteFast(PRESET_SIGNX_NEG, HIGH);
    // this messes up the dot on all operations
    // digitalWriteFast(DP1, LOW);
    // digitalWriteFast(DP2, HIGH);
    // digitalWriteFast(DP4, HIGH);
    // digitalWriteFast(DP8, LOW);
    // digitalWriteFast(DP_LATCH, HIGH);
    for (int i = 0; i < 13; i++) {
        ext_value[i] = ~ext_value[i];
    }
    int i = 0;
    while (true) {
        // does not work?
        step(CLEAR_X);

        // works
        // step(CLEAR_XYZ);

        // works
        // step(MODULE);

        // works
        // step(SWAP_X_Y);

        // works with parser
        // seems decimal point is 1 position to the left in real life
        prepare_ext_value("12.65e1");
        step(SWAP_EXT_X);
        step(DISP);
        wait_for_input();

        step(SWAP_X_Y);
        step(DISP);
        wait_for_input();

        prepare_ext_value("12.65e1");
        step(SWAP_EXT_X);
        step(DISP);
        wait_for_input();

        // this does not work
        step(DIVIDE);
        step(DISP);
        wait_for_input();

        // does not work
        // step(SQRT);
        // step(DISP);

        // does not work
        // set_keyboard_digit(2, true);
        // step(LOAD);

        // does not work
        // set_keyboard_exponent(1, false);
        // step(LOAD);
        // digitalWriteFast(DEFEAT_HOLD, LOW);
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
        case SWAP_INT_X:
        case MOVE_INT_X_Y:
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
        digitalWriteFast(EXT_PROG_A, LOW)
    } else {
        digitalWriteFast(EXT_PROG_A, HIGH);
    }
    if (((opcode >> 1) & 1)) {
        digitalWriteFast(EXT_PROG_B, LOW);
    } else {
        digitalWriteFast(EXT_PROG_B, HIGH);
    }
    if (((opcode >> 2) & 1)) {
        digitalWriteFast(EXT_PROG_C, LOW);
    } else {
        digitalWriteFast(EXT_PROG_C, HIGH);
    }
    if (((opcode >> 3) & 1)) {
        digitalWriteFast(EXT_PROG_D, LOW);
    } else {
        digitalWriteFast(EXT_PROG_D, HIGH);
    }
    if (((opcode >> 4) & 1)) {
        digitalWriteFast(EXT_PROG_E, LOW);
    } else {
        digitalWriteFast(EXT_PROG_E, HIGH);
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
    digitalWriteFast(LOAD_E_KB, LOW);
    delay(100);
    if (sign) {
        digitalWriteFast(KB_SIGN_E, LOW);
    } else {
        digitalWriteFast(KB_SIGN_E, HIGH);
    }
    if ((digit & 1)) {
        digitalWriteFast(KD1, LOW)
    } else {
        digitalWriteFast(KD1, HIGH);
    }
    if (((digit >> 1) & 1)) {
        digitalWriteFast(KD2, LOW);
    } else {
        digitalWriteFast(KD2, HIGH);
    }
    if (((digit >> 2) & 1)) {
        digitalWriteFast(KD4, LOW);
    } else {
        digitalWriteFast(KD4, HIGH);
    }
    if (((digit >> 3) & 1)) {
        digitalWriteFast(KD8, LOW);
    } else {
        digitalWriteFast(KD8, HIGH);
    }
    Serial.print("exponent: ");
    Serial.print(((digit >> 3) & 1));
    Serial.print(((digit >> 2) & 1));
    Serial.print(((digit >> 1) & 1));
    Serial.print((digit & 1));
    Serial.println();
}

void set_negative() {
    digitalWriteFast(PRESET_SIGNX_NEG, HIGH);
    step(LOAD);
    step(LOAD);
}

void set_keyboard_digit(uint8_t digit, bool sign) {
    if ((digit & 1)) {
        digitalWriteFast(KD1, LOW)
    } else {
        digitalWriteFast(KD1, HIGH);
    }
    if (((digit >> 1) & 1)) {
        digitalWriteFast(KD2, LOW);
    } else {
        digitalWriteFast(KD2, HIGH);
    }
    if (((digit >> 2) & 1)) {
        digitalWriteFast(KD4, LOW);
    } else {
        digitalWriteFast(KD4, HIGH);
    }
    if (((digit >> 3) & 1)) {
        digitalWriteFast(KD8, LOW);
    } else {
        digitalWriteFast(KD8, HIGH);
    }
    Serial.print("digit: ");
    Serial.print(!((digit >> 3) & 1));
    Serial.print(!((digit >> 2) & 1));
    Serial.print(!((digit >> 1) & 1));
    Serial.print(!(digit & 1));
    Serial.println();
}
// accepts input in the form of "-?d+.d+e-?d", like -5.4e-3
void prepare_ext_value(char *input) {
    uint8_t len_input = strlen(input);
    char *cpy_input = (char *)malloc(sizeof(char) * len_input);
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
    char *decimal = strtok(cpy_input, ".");
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
    char *fraction = strtok(NULL, "e");
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
    char *sign_exp = strtok(NULL, "");
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
    cli();
    // catch rising edge of clock
    while (digitalReadFast(EXT_CLK)) {
    }
    while (!digitalReadFast(EXT_CLK)) {
    }
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
    digitalWriteFast(EXT_START, LOW);
    __asm__("nop\n\t");
    __asm__("nop\n\t");
    __asm__("nop\n\t");
    __asm__("nop\n\t");
    __asm__("nop\n\t");
    __asm__("nop\n\t");
    __asm__("nop\n\t");
    __asm__("nop\n\t");
    // TODO: implement 13 pulses for external loading
    digitalWriteFast(EXT_START, HIGH);
    // catch rising edge of ext step complete
    if (ext) {
        PORTF = ext_value[ctr++];
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
    //  reset state
    //  cannot yet access val & recv here without impacting performance during transmit and read
    digitalWriteFast(KB_SIGN_E, LOW);
    digitalWriteFast(LOAD_E_KB, HIGH);
    digitalWriteFast(KD1, HIGH);
    digitalWriteFast(KD2, HIGH);
    digitalWriteFast(KD4, HIGH);
    digitalWriteFast(KD8, HIGH);
    digitalWriteFast(SQ1, HIGH);
    digitalWriteFast(SQ2, HIGH);
    digitalWriteFast(SQ4, HIGH);
    digitalWriteFast(SQ8, HIGH);
    digitalWriteFast(EXT_PROG_A, HIGH);
    digitalWriteFast(EXT_PROG_B, HIGH);
    digitalWriteFast(EXT_PROG_C, HIGH);
    digitalWriteFast(EXT_PROG_D, HIGH);
    digitalWriteFast(EXT_PROG_E, HIGH);
}
void loop() {
    // put your main code here, to run repeatedly:
    char c = Serial.read();
    if (!external_module_enabled()) {
        // do certain things
        return;
    }
    switch (c) {
        case '\r':
        case '\n':
        default:
            Serial.println("do nothing");
            return;
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
        case '0':
            // load character
            // write_character(c);
            break;
    }
}

void write_character(char c) {
    uint8_t val = atoi(&c);
    // set_opcode(LOAD);
    digitalWriteFast(DEFEAT_HOLD, LOW);
    digitalWriteFast(EXT_PROG_A, HIGH);
    digitalWriteFast(EXT_PROG_B, LOW);
    digitalWriteFast(EXT_PROG_C, LOW);
    digitalWriteFast(EXT_PROG_D, HIGH);
    digitalWriteFast(EXT_PROG_E, LOW);
}

bool external_module_enabled() {
    if (digitalRead(EXT_SW) == LOW) {
        return true;
    }
    return false;
}

bool mainframe_reset() {
    if (digitalReadFast(EXT_RESET) == LOW) {
        return true;
    }
    return false;
}

void select_module_input(bool A) {
    if (A) {
        digitalWriteFast(REMOTE_AB, HIGH);
    }
}

void write_ext_register(uint8_t value) {
}

void write_pin(IOpins_t pin, bool value) {
    if (pin.mode == INPUT) {
        Serial.println("Pin marked as input");
        return;
    }
    if (pin.mode == OUTPUT_OC) {
        pinModeFast(pin.number, OUTPUT);
    }
    // cannot set pinModeFast back until clockcycle
}

void wait_for_input() {
    while (Serial.available() == 0) {
        delay(1);
    }
    while (Serial.available() > 0) {
        Serial.read();
    }
}
