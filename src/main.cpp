#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_NeoMatrix.h>
#include <Adafruit_NeoPixel.h>
#include "SensorQMI8658.hpp"
#include <math.h>
#include <numeric>

// --- CONFIGURACIÓN DE DEPURACIÓN ---
#define DEBUG
#define EVDEBUG
// #define DEBUG_IN_METERS_PER_SECOND

// --- CONFIGURACIÓN DE REPOSO ---
#define INACTIVITY_TIMEOUT_S 15  // Segundos de inactividad para apagar los LEDs.

// --- Configuración de la Matriz de LEDs ---
#define LED_PIN 14
#define MATRIX_WIDTH 8
#define MATRIX_HEIGHT 8
#define BRIGHTNESS 50

Adafruit_NeoMatrix matrix = Adafruit_NeoMatrix(MATRIX_WIDTH, MATRIX_HEIGHT, LED_PIN,
    NEO_MATRIX_BOTTOM + NEO_MATRIX_LEFT + NEO_MATRIX_ROWS +
    NEO_MATRIX_ZIGZAG, NEO_GRB + NEO_KHZ800);

// --- Configuración del IMU ---
SensorQMI8658 qmi;
#define I2C_SDA 11
#define I2C_SCL 12

// --- Umbrales de Detección ---
#define BRAKE_DELTA_THRESHOLD 2.0
#define TURN_ACCEL_THRESHOLD 40
#define VERTICAL_TOLERANCE_DEG 25.0
#define MOTION_DETECT_THRESHOLD 0.3
#define MOTION_Z_THRESHOLD -0.30
#define VERTICAL_X_TOLERANCE 0.90

// --- Temporizadores ---
#define MIN_ANIMATION_DURATION_MS 2000

// --- Estados de la Lámpara ---
enum LightState { NORMAL, BRAKING, TURN_LEFT, TURN_RIGHT, POWER_SAVE, POWER_SAVE_LEDS};
// --- Estados de la Animacion ---
// enum AnimationState { NORMAL, BRAKING, TURN_LEFT, TURN_RIGHT};
// El estado inicial ahora es POWER_SAVE. La luz empieza apagada.
LightState currentState = POWER_SAVE;

// --- Variables Globales de Estado ---
float ax, ay, az, gx, gy, gz;
unsigned long lastMotionTime = 0;
unsigned long lastAnimationStartTime = 0;
static unsigned long lastPrintTime = 0;

// --- Historial de Aceleración ---
#define ACCEL_HISTORY_SIZE 100
float accel_z_history[ACCEL_HISTORY_SIZE] = {0};
int accel_history_index = 0;

// --- Calibración del Giroscopio ---
#define GYRO_X_OFFSET 2.23
#define GYRO_Y_OFFSET 0.13
#define GYRO_Z_OFFSET -0.37

// --- Animaciones (Bitmaps) ---
const uint8_t arrow_left[] PROGMEM = { 0b00011000, 0b00110000, 0b01111110, 0b11111111, 0b01111110, 0b00110000, 0b00011000, 0b00000000 };
const uint8_t arrow_right[] PROGMEM = { 0b00011000, 0b00001100, 0b01111110, 0b11111111, 0b01111110, 0b00001100, 0b00011000, 0b00000000 };

// --- Funciones de Ayuda ---
void displayAnimation(const uint8_t bitmap[]) { matrix.clear(); matrix.drawBitmap(0, 0, bitmap, 8, 8, matrix.Color(255, 50, 0)); matrix.show(); }
void breathingEffect() { float breath = (exp(sin(millis() / 2000.0 * PI)) - 0.36787944) * 108.0; matrix.clear(); matrix.fillScreen(matrix.Color(breath, 0, 0)); matrix.show(); }
void brakeLight() { matrix.clear(); matrix.fillScreen(matrix.Color(0, 255, 0)); matrix.show(); }
void powerOffLeds() { matrix.clear(); matrix.show(); }

void initialize_sensor() {
    qmi.reset();
    qmi.configAccelerometer(SensorQMI8658::ACC_RANGE_8G, SensorQMI8658::ACC_ODR_250Hz);
    qmi.configGyroscope(SensorQMI8658::GYR_RANGE_1024DPS, SensorQMI8658::GYR_ODR_224_2Hz);
    qmi.enableAccelerometer();
    qmi.enableGyroscope();
    printf("Sensor inicializado en modo activo.\n");
}

// --- Declaración de Funciones ---
void update_sensors();
void update_state();
void execute_state_action();

void setup() {
    Serial.begin(115200);
    delay(4000);
    printf("Iniciando Lampara Trasera Inteligente v5.7 (Motion-Based Logic)\n");

    matrix.begin();
    matrix.setBrightness(BRIGHTNESS);
    powerOffLeds();

    Wire.begin(I2C_SDA, I2C_SCL);
    if (!qmi.begin(Wire, QMI8658_L_SLAVE_ADDRESS, I2C_SDA, I2C_SCL)) {
        printf("FATAL: No se pudo encontrar el sensor QMI8658.\n");
        while (1) delay(10);
    }
    printf("QMI8658 encontrado!\n");

    initialize_sensor();
    lastMotionTime = millis();
}

void loop() {
    update_sensors();
    update_state();
    execute_state_action();
    delay(50);
}

void update_sensors() {
    bool accel_ok = qmi.getAccelerometer(ax, ay, az);
    bool gyro_ok = qmi.getGyroscope(gx, gy, gz);

    if (!accel_ok || !gyro_ok || (ax == 0 && ay == 0 && az == 0) ){
        printf("!!! ERROR: Fallo al leer datos del sensor! Accel:%d, Gyro:%d\n", accel_ok, gyro_ok);
        return; // No procesar datos si son inválidos
    }

    gx -= GYRO_X_OFFSET; gy -= GYRO_Y_OFFSET; gz -= GYRO_Z_OFFSET;

    accel_z_history[accel_history_index] = az * 9.81;
    accel_history_index = (accel_history_index + 1) % ACCEL_HISTORY_SIZE;
#ifdef DEBUG
    unsigned long currentTime = millis();
    if (currentTime - lastPrintTime > 500) {
#ifdef DEBUG_IN_METERS_PER_SECOND
        printf("ax:%.2f, ay:%.2f, az:%.2f, gx:%.2f, gy:%.2f, gz:%.2f, state:%d (m/s^2)\n", ax * 9.81, ay * 9.81, az * 9.81, gx, gy, gz, currentState);
#else
        printf("ax:%.2f, ay:%.2f, az:%.2f, gx:%.2f, gy:%.2f, gz:%.2f, state:%d (G's)\n", ax, ay, az, gx, gy, gz, currentState);
#endif
        lastPrintTime = currentTime;
    }
#endif
}

void update_state() {
    unsigned long currentTime = millis();

    // 1. Condición principal de funcionamiento: ax DEBE ser mayor que 0.93
    if (ax <= VERTICAL_X_TOLERANCE) {
        currentState = POWER_SAVE_LEDS;
        // printf("--no vertical-- \n");
        return;
    }

    // --- Si la condición ax > 0.93 se cumple, proceder con la lógica normal ---
    bool is_moving = (abs(ax) > MOTION_DETECT_THRESHOLD || abs(ay) > MOTION_DETECT_THRESHOLD || abs(az) > MOTION_DETECT_THRESHOLD);
    bool is_moving_forward = (ay < MOTION_Z_THRESHOLD) && ax > VERTICAL_X_TOLERANCE ;

    if (is_moving_forward) {
        lastMotionTime = currentTime;
        // Si estaba apagado, despertar al estado NORMAL.
        if (currentState == POWER_SAVE) {
            printf("--- SISTEMA: Movimiento detectado. Activando... ---\n");
            currentState = NORMAL;
        }
    } else {
        // Si no hay movimiento, comprobar si ha pasado el tiempo de inactividad.
        if (currentState != POWER_SAVE && (currentTime - lastMotionTime) > (INACTIVITY_TIMEOUT_S * 1000)) {
            printf("---" "SISTEMA: Inactividad prolongada. Entrando en modo de ahorro de energia...""---\n");
            currentState = POWER_SAVE;
        }
    }

    // Si después de las comprobaciones anteriores estamos en POWER_SAVE, no hacer nada más.
    if (currentState == POWER_SAVE) {
        return;
    }


    // 5. Detección de eventos físicos (Freno > Giro)
    LightState detectedState = NORMAL;
    float current_az = az * 9.81;
    float avg_az = 0; for(int i=0; i<ACCEL_HISTORY_SIZE; i++) avg_az += accel_z_history[i]; avg_az /= ACCEL_HISTORY_SIZE;

    if ((current_az - avg_az) > BRAKE_DELTA_THRESHOLD) { detectedState = BRAKING; }
    else if (gx > TURN_ACCEL_THRESHOLD) { detectedState = TURN_LEFT; }
    else if (gx < -TURN_ACCEL_THRESHOLD) { detectedState = TURN_RIGHT; }

    // 6. Máquina de Transición de Estados y Animaciones
    bool isMajorStateActive = (currentState == BRAKING || currentState == TURN_LEFT || currentState == TURN_RIGHT);
    if (detectedState != NORMAL && detectedState != currentState) {
#ifdef EVDEBUG
        if (currentState != detectedState) { 
            switch(detectedState) {
                case BRAKING:    printf("---" "EVENTO: FRENADO" "---\n"); break;
                case TURN_LEFT:  printf("---" "EVENTO: GIRO IZQUIERDA" "---\n"); break;
                case TURN_RIGHT: printf("---" "EVENTO: GIRO DERECHA" "---\n"); break;
                default: break;
            }
        }
#endif
        currentState = detectedState;
        lastAnimationStartTime = currentTime;
        return;
    }

    if (isMajorStateActive && (currentTime - lastAnimationStartTime > MIN_ANIMATION_DURATION_MS)) {
        currentState = NORMAL;
    }
}

void execute_state_action() {
    switch (currentState) {
        case BRAKING: brakeLight(); break;
        case TURN_LEFT: displayAnimation(arrow_left); break;
        case TURN_RIGHT: displayAnimation(arrow_right); break;
        case NORMAL: breathingEffect(); break;
        case POWER_SAVE: powerOffLeds(); break;
    }
}
