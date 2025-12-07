#include "PS2Mouse.h"
#include <SoftwareSerial.h>

#define DATA_PIN 5
#define CLOCK_PIN 6

// UART hacia FPGA usando SoftwareSerial
#define FPGA_RX_PIN 8   // No usado (FPGA no envía datos)
#define FPGA_TX_PIN 7   // TX hacia FPGA -> conectar al pin B2 de Tang Primer 25K
#define FPGA_BAUD 9600

PS2Mouse mouse(CLOCK_PIN, DATA_PIN);
SoftwareSerial fpgaSerial(FPGA_RX_PIN, FPGA_TX_PIN);

// Variables de 8 bits para datos del mouse
int8_t delta_x;
int8_t delta_y;
uint8_t buttons;
uint8_t prev_buttons = 0;

// Factor de escala para sensibilidad (dividir delta)
#define SCALE_FACTOR 3

void setup() {
    Serial.begin(9600);           // Debug
    fpgaSerial.begin(FPGA_BAUD);  // UART hacia FPGA
    mouse.initialize();
    Serial.println("Mouse PS2 inicializado");
}

void loop() {
    MouseData data = mouse.readData();
    
    // Extraer y escalar delta X e Y
    int raw_x = data.position.x;
    int raw_y = data.position.y;
    
    // Aplicar escala para reducir sensibilidad
    int scaled_x = raw_x / SCALE_FACTOR;
    int scaled_y = raw_y / SCALE_FACTOR;
    
    // Limitar a rango de int8_t
    if (scaled_x > 127) delta_x = 127;
    else if (scaled_x < -128) delta_x = -128;
    else delta_x = (int8_t)scaled_x;
    
    if (scaled_y > 127) delta_y = 127;
    else if (scaled_y < -128) delta_y = -128;
    else delta_y = (int8_t)scaled_y;
    
    // Extraer botones (bits 0, 1, 2 del status)
    // Bit 0: Botón izquierdo
    // Bit 1: Botón derecho
    // Bit 2: Botón central
    buttons = (uint8_t)(data.status & 0x07);
    
    // Solo enviar si hay movimiento O si cambiaron los botones
    bool hasMovement = (delta_x != 0) || (delta_y != 0);
    bool buttonChanged = (buttons != prev_buttons);
    
    if (hasMovement || buttonChanged) {
        // Debug en Serial
        Serial.print("Btn: 0x");
        Serial.print(buttons, HEX);
        Serial.print("\tdX=");
        Serial.print(delta_x);
        Serial.print("\tdY=");
        Serial.println(delta_y);
        
        // Enviar 3 bytes a la FPGA por UART
        // Byte 0: buttons (bit0=izq, bit1=der, bit2=medio)
        // Byte 1: delta_x (movimiento en X, con signo)
        // Byte 2: delta_y (movimiento en Y, con signo)
        fpgaSerial.write(buttons);
        fpgaSerial.write((uint8_t)delta_x);
        fpgaSerial.write((uint8_t)delta_y);
        
        prev_buttons = buttons;
    }
    
    delay(20);  // 20ms entre lecturas
}
