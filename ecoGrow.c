/*
    ======= PORT AND PIN ASSIGNMENTS =======
    RA1      - Analog input for LM35 temperature sensor
    RB0      - Digital input for soil moisture sensor (input only)
    RC2      - PWM output to control servo motor (vent)
    RC6      - Digital output for Ultrasonic Trigger (HC-SR04 Trig)
    RC7      - Digital input for Ultrasonic Echo (HC-SR04 Echo)
    RD0      - Digital output to control Water Pump (via relay/transistor)
    RD1      - Digital output for Fan or temperature indicator LED (fan logic)
    RD2      - Digital output for Ultrasonic “distance detected” LED
    Other pins - Unused, available for expansion
*/

/*
    ======= FUNCTIONAL OVERVIEW =======
    - Soil Moisture: If soil is dry (sensor on RB0 reads HIGH), water pump (RD0) turns ON.
    - Temperature: Reads temp sensor (LM35 on RA1), if >25°C, activates fan (RD1) and opens vent (servo on RC2).
    - Servo (vent): Opens or closes vent based on temperature.
    - Ultrasonic: Measures distance. If an object is detected >=10 cm away, LED on RD2 turns ON.
*/

// === Variable Declarations ===
float voltage;                   // For temp sensor voltage
float temperature_Celsius;       // For calculated temperature value
unsigned int myreadingtemp;      // Raw ADC value for temperature
unsigned char j, k;              // Loop variables for delays
unsigned long T1counts;          // Ultrasonic timer counts
unsigned long T1time;            // Ultrasonic pulse width
unsigned long Distance;          // Calculated distance in cm
int test_moisture;               // Result from soil moisture check
unsigned long loop_counter = 0;

// === Function Prototypes ===
void Delay_ms(void);
void msDelay(unsigned int msCnt);
void pwm_init(void);
void set_servo_position1(int degrees);
void ATD_init(void);
unsigned int ATD_read(unsigned char port);
void init_sonar(void);
void read_sonar(void);
int moisture(void);

// === Delay function for general use ===
void Delay_ms(void){
   for(j=0; j<255; j++){
       for(k=0; k<200; k++);
   }
}

// === Millisecond delay (calibrated for your MCU clock) ===
void msDelay(unsigned int msCnt) {
    unsigned int ms = 0;
    unsigned int cc = 0;
    for (ms = 0; ms < msCnt; ms++) {
        for (cc = 0; cc < 155; cc++);
    }
}

// === PWM setup for servo motor on RC2 ===
void pwm_init() {
    // Set RC2 (CCP1) as output for PWM
    TRISC = TRISC & 0xFB; // 0b11111011: RC2=0(output), rest unchanged
    CCP1CON = 0x0C;       // PWM mode on CCP1 (RC2)
    T2CON = T2CON | 0x07; // Timer2 ON, prescaler
    PR2 = 249;            // For ~50Hz (servo pulse)
}

// === Set servo position (vent) by degree on RC2 ===
void set_servo_position1(int degrees) {
    // Converts degrees to pulse width for typical servo (adjust as needed)
    int pulse_width = (degrees + 90) * 8 + 500; // from -90 to +90 degrees
    CCPR1L = pulse_width >> 2;
    CCP1CON = (CCP1CON & 0xCF) | ((pulse_width & 0x03) << 4);
    Delay_ms(200); // Allow servo to reach position
}

// === Analog-to-Digital setup for temp sensor ===
void ATD_init(void){
      ADCON0=0x41;           // Turn on ADC, select channel 0, Fosc/16
      ADCON1=0xC0;           // RA0/RA1 analog, rest digital
}

// === Read analog value from specified port ===
unsigned int ATD_read(unsigned char port) {
     ADCON0 = (ADCON0 & 0xC7) | (port << 3);
     Delay_ms(100);
     ADCON0 = ADCON0 | 0x04;
     while(ADCON0 & 0x04);
     return ((ADRESH << 8) | ADRESL);
}

// === Read soil moisture from RB0 (input only) ===
int moisture() {
    // RB0 reads HIGH if soil is dry, LOW if moist
    if (PORTB & 0x01) {
        // Soil is dry
        return 1;
    } else {
        // Soil is moist
        return 0;
    }
}

// === Ultrasonic sensor functions (RC6=Trig, RC7=Echo) ===

// Triggers ultrasonic and measures distance
void read_sonar(void) {
    T1counts = 0;
    T1time = 0;
    Distance = 0;

    TMR1H = 0;
    TMR1L = 0;

    PORTC |= 0b01000000;   // Set RC6 (Trig) HIGH
    msDelay(10);           // Wait 10ms
    PORTC &= ~0b01000000;  // Set RC6 (Trig) LOW

    // Wait for Echo to go HIGH (RC7)
    while (!(PORTC & 0b10000000));

    T1CON = 0x19;          // Timer1 ON, Fosc/4, prescaler
    // Wait for Echo to go LOW (RC7)
    while (PORTC & 0b10000000);
    T1CON = 0x18;          // Timer1 OFF

    T1counts = ((TMR1H << 8) | TMR1L);
    T1time = T1counts;     // Time in microseconds (roughly)
    Distance = (T1time * 34) / (1000 * 2);  // Convert to cm

    if (Distance > 400) {
        Distance = 0;  // Ignore out-of-range readings
    }
}

// Initializes ultrasonic timer variables
void init_sonar(void) {
    T1counts = 0;
    T1time = 0;
    Distance = 0;
    TMR1H = 0;
    TMR1L = 0;
    T1CON = 0x18;    // Timer1 OFF initially
}

// === MAIN FUNCTION ===

void main(){
    // === PORT/PIN SETUP ===

    // PWM/Servo on RC2 (output), Ultrasonic Trig on RC6 (output), Echo on RC7 (input)
    pwm_init();
    TRISC = (TRISC & 0x39) | 0b10000000; // RC2 output, RC6 output, RC7 input (0b10000000)

    // Temperature sensor analog input on RA1, rest digital
    ADCON1 = 0x06;      // RA0/RA1 analog, rest digital
    TRISA = 0x03;       // RA0, RA1 as input (others output)

    // Soil Moisture sensor input on RB0 only, rest outputs
    TRISB = 0x01;       // RB0 input, RB1-7 output (but we only use RB0 for input)
    PORTB = 0x00;

    // Outputs: RD0 (water pump), RD1 (fan), RD2 (ultrasonic LED)
    TRISD = TRISD & 0xF8;   // RD0, RD1, RD2 output, rest unchanged
    PORTD = 0x00;

    ATD_init();
    init_sonar();

    while (1) {
        // === TEMPERATURE & FAN/SERVO LOGIC ===
        myreadingtemp = ATD_read(1);  // Read from RA1 (temp sensor)
        msDelay(1000);                // 1 second delay between readings

        voltage = (myreadingtemp / 1023.0) * 500;        // Calculate voltage (5V ADC)
        temperature_Celsius = voltage;            // LM35: 10mV/°C

        if(temperature_Celsius >= 30){
            PORTD = PORTD | 0x02;    // Set RD1 HIGH: Fan ON
        } else {
            PORTD = PORTD & (~0x02); // Set RD1 LOW: Fan OFF
        }

        // === SOIL MOISTURE & WATER PUMP LOGIC ===
        test_moisture = moisture();      // Read soil state on RB0
        if(test_moisture == 1){
            PORTD = PORTD | 0x01;       // Set RD0 HIGH: Pump ON
        } else {
            PORTD = PORTD & (~0x01);    // Set RD0 LOW: Pump OFF
        }

        // === ULTRASONIC SENSOR LOGIC ===
        read_sonar();                   // Trigger and read ultrasonic
        if (Distance >= 10) {
            PORTD = PORTD | 0x04;       // Set RD2 HIGH: LED ON
        } else {
            PORTD = PORTD & (~0x04);    // Set RD2 LOW: LED OFF
        }
        
        
        // === 5-MINUTE SERVO TIMER LOGIC ===
        loop_counter++;
        if (loop_counter >= 10) { // 250 * 1200ms = 300,000ms = 5 minutes
        set_servo_position1(90);   // Rotate to 90 degrees
        msDelay(10000);            // Hold for 10 seconds
        set_servo_position1(-90);  // Return to original position
        loop_counter = 0;          // Reset counter
        }

        msDelay(100);                   // Short general delay
    }
}
