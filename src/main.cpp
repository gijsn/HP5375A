#include <Arduino.h>
#include <digitalWriteFast.h>
#include <pins_arduino.h>

#define OUTPUT_OC 0x3
enum _opcodes {
    NORMALIZE = 0,
    MODULE = 2,
    PLUG_IN = 3,
    SQRT_X = 5,
    SELF_CHECK = 6,
    CALIBRATE = 7,
    DISP = 8,
    TEN_X = 9,
    ADD = 10,
    SUBTRACT = 11,
    LOAD = 13,
    DIVIDE = 14,
    MULTIPLY = 15,
    SWAP_INT_X = 17,
    INT_X_Y = 19,
    TWO_X = 20,
    SWAP_X_Y = 21,
    CLEAR_X = 22,
    X_Y = 23,
    SWAP_EXT_X = 25,
    EXT_X_Y = 27,
    DIVIDE_X_10 = 28,
    SWAP_Z_X = 29,
    CLEAR_XYZ = 30,
    Z_X_Y = 31
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
    SQ1 = X,
    SQ2 = X,
    SQ4 = X,
    SQ8 = X,
    KB_SIGN_E = 30,
    KD1 = 23,
    KD2 = 25,
    KD4 = 27,
    KD8 = 29,
    DP_LATCH = X,
    DP1 = X,
    DP2 = X,
    DP4 = X,
    DP8 = X,
    EXT_PROG_A = 45,
    EXT_PROG_B = 47,
    EXT_PROG_C = 49,
    EXT_PROG_D = 51,
    EXT_PROG_E = 53,
    REMOTE_AB = X,
    EXT_START = 52,
    EXT_RESET = 50,
    EXT_SHIFT13 = X,
    EXT_STEP_COMPL = 19,
    SW_RST = X,
    EXT_CLK = 18,
    EXT_STORE = X,
    PRESET_SIGNX_NEG = X,
    EXT_S1 = X,
    EXT_S2 = X,
    EXT_S4 = X,
    EXT_S8 = X,
    SIGNX = X,
    DEFEAT_HOLD = X,
    EXT_SW = 42,
    PI_ERROR_DEFEAT = X,
    DIRTY_ZEROS = X
};

typedef struct {
    uint8_t number;
    uint8_t mode;
    bool inverted_logic;
} IOpins_t;

#define NO_PINS 38
IOpins_t pin[NO_PINS] = {
    {LOAD_E_KB, OUTPUT, true},
    {SQ1, OUTPUT, true},
    {SQ2, OUTPUT, true},
    {SQ4, OUTPUT, true},
    {SQ8, OUTPUT, true},
    {KB_SIGN_E, OUTPUT, false},
    {KD1, OUTPUT, true},  // open collector
    {KD2, OUTPUT, true},  // open collector
    {KD4, OUTPUT, true},  // open collector
    {KD8, OUTPUT, true},  // open collector
    {DP_LATCH, OUTPUT, true},
    {DP1, OUTPUT, false},  // open collector
    {DP2, OUTPUT, true},   // open collector
    {DP4, OUTPUT, true},   // open collector
    {DP8, OUTPUT, false},  // open collector
    {EXT_PROG_A, OUTPUT, true},
    {EXT_PROG_B, OUTPUT, true},
    {EXT_PROG_C, OUTPUT, true},
    {EXT_PROG_D, OUTPUT, true},
    {EXT_PROG_E, OUTPUT, true},
    {REMOTE_AB, OUTPUT, false},
    {EXT_START, OUTPUT, true},
    {EXT_RESET, OUTPUT, true},
    {EXT_SHIFT13, OUTPUT, true},
    {EXT_STEP_COMPL, OUTPUT, false},
    {SW_RST, OUTPUT, true},  // open collector
    {EXT_CLK, INPUT, false},
    {EXT_STORE, INPUT, true},
    {PRESET_SIGNX_NEG, OUTPUT, true},
    {EXT_S1, INPUT, false},
    {EXT_S2, INPUT, false},
    {EXT_S4, INPUT, false},
    {EXT_S8, INPUT, false},
    {SIGNX, INPUT, false},
    {DEFEAT_HOLD, OUTPUT, true},
    {EXT_SW, INPUT, true},
    {
        PI_ERROR_DEFEAT,
    },
    {
        DIRTY_ZEROS,
    }};

// put function declarations here:
int myFunction(int, int);

void set_opcode(uint8_t opcode);
void set_keyboard_digit(uint8_t digit);

bool external_module_enabled();
void step();

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
    digitalWriteFast(LOAD_E_KB, HIGH);
    digitalWriteFast(KB_SIGN_E, HIGH);
    int i = 0;
    // attachInterrupt(digitalPinToInterrupt(EXT_CLK), clock, RISING);
    // attachInterrupt(digitalPinToInterrupt(EXT_STEP_COMPL), step_complete, RISING);
    while (true) {
        // set_opcode(CLEAR_XYZ);
        // step();
        // digitalWrite(LOAD_E_KB, LOW);
        set_keyboard_digit(1);
        set_opcode(LOAD);
        step();
        // digitalWriteFast(LOAD_E_KB, LOW);
        // set_keyboard_digit(G);
        // set_opcode(LOAD);
        // step();
        // digitalWriteFast(LOAD_E_KB, HIGH);
        // digitalWrite(LOAD_E_KB, HIGH);
        // delay(1000);
        Serial.println("Display digit");
        // digitalWriteFast(KD1, HIGH);
        // digitalWriteFast(KD2, HIGH);
        // digitalWriteFast(KD4, HIGH);
        // digitalWriteFast(KD8, HIGH);
        set_opcode(DISP);
        step();

        Serial.print("ext module selected ");
        Serial.println(external_module_enabled());
        delay(1000);
        i++;
        i %= 16;
    }
}

// void clock(){
//   digitalWriteFast(3, HIGH);
//   digitalWriteFast(3, LOW);
// }

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

void set_keyboard_digit(uint8_t digit) {
    digitalWrite(KD1, !(digit & 1));
    digitalWrite(KD2, !((digit >> 1) & 1));
    digitalWrite(KD4, !((digit >> 2) & 1));
    digitalWrite(KD8, !((digit >> 3) & 1));
    Serial.print("digit: ");
    Serial.print(!((digit >> 3) & 1));
    Serial.print(!((digit >> 2) & 1));
    Serial.print(!((digit >> 1) & 1));
    Serial.print(!(digit & 1));
    Serial.println();
}

void step() {
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
    digitalWriteFast(EXT_START, HIGH);
    // catch rising edge of ext step complete
    while (!digitalReadFast(EXT_STEP_COMPL)) {
    }
    sei();
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
    set_opcode(LOAD);
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

bool select_module_input(bool A) {
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
    if (pin.inverted_logic) {
    }
    // cannot set pinModeFast back until clockcycle
}