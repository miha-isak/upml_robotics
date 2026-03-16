#define MOTOR_PORT_DIR  PORTB 
#define L_DIR_BIT       (1 << 7) // Pin 13
#define R_DIR_BIT       (1 << 6) // Pin 12
#define L_PWM_REG       OCR1A    // Pin 11
#define R_PWM_REG       OCR3C    // Pin 3

namespace motors {
    void setup() {
        DDRB |= (1 << 7) | (1 << 6) | (1 << 5);
        DDRE |= (1 << 5);
        // Timer 1: Fast PWM 8-bit
        TCCR1A = (1 << COM1A1) | (1 << WGM10);
        TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);
        // Timer 3: Fast PWM 8-bit
        TCCR3A = (1 << COM3C1) | (1 << WGM30);
        TCCR3B = (1 << WGM32) | (1 << CS31) | (1 << CS30);
    }

    inline void move_left(int speed) {
        if (speed > 0) { MOTOR_PORT_DIR |= L_DIR_BIT; L_PWM_REG = (uint8_t)speed; }
        else if (speed < 0) { MOTOR_PORT_DIR &= ~L_DIR_BIT; L_PWM_REG = (uint8_t)(-speed); }
        else { L_PWM_REG = 0; }
    }

    inline void move_right(int speed) {
        if (speed > 0) { MOTOR_PORT_DIR &= ~R_DIR_BIT; R_PWM_REG = (uint8_t)speed; }
        else if (speed < 0) { MOTOR_PORT_DIR |= R_DIR_BIT; R_PWM_REG = (uint8_t)(-speed); }
        else { R_PWM_REG = 0; }
    }
}