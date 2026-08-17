// Calibration dành cho:
// ESP32 NodeMCU-32S
// Differential Drive
// 2 x BTS7960
// Serial baud: 115200

#include <Arduino.h>
#include "config.h"

#define ENCODER_USE_INTERRUPTS
#define ENCODER_OPTIMIZE_INTERRUPTS
#include "encoder.h"

#include "kinematics.h"

/* =====================================================
 * BTS7960 ENABLE PINS
 * Dùng giá trị trong esp32_config.h nếu đã khai báo.
 * ===================================================== */

#ifndef MOTOR1_REN
#define MOTOR1_REN 32
#endif

#ifndef MOTOR1_LEN
#define MOTOR1_LEN 25
#endif

#ifndef MOTOR2_REN
#define MOTOR2_REN 13
#endif

#ifndef MOTOR2_LEN
#define MOTOR2_LEN 12
#endif

/*
 * Theo cấu hình của bạn:
 *
 * MOTOR1_IN_A = RPWM = GPIO33
 * MOTOR1_IN_B = LPWM = GPIO26
 *
 * MOTOR2_IN_A = RPWM = GPIO27
 * MOTOR2_IN_B = LPWM = GPIO14
 */

#define M1_RPWM MOTOR1_IN_A
#define M1_LPWM MOTOR1_IN_B

#define M2_RPWM MOTOR2_IN_A
#define M2_LPWM MOTOR2_IN_B

#define SAMPLE_TIME_SECONDS 10
#define SAMPLE_TIME_US (SAMPLE_TIME_SECONDS * 1000000UL)

/* =====================================================
 * ENCODER
 * ===================================================== */

Encoder motor1_encoder(
    MOTOR1_ENCODER_A,
    MOTOR1_ENCODER_B,
    COUNTS_PER_REV1,
    MOTOR1_ENCODER_INV
);

Encoder motor2_encoder(
    MOTOR2_ENCODER_A,
    MOTOR2_ENCODER_B,
    COUNTS_PER_REV2,
    MOTOR2_ENCODER_INV
);

Encoder *encoders[2] = {
    &motor1_encoder,
    &motor2_encoder
};

/* =====================================================
 * KINEMATICS
 * ===================================================== */

Kinematics kinematics(
    Kinematics::LINO_BASE,
    MOTOR_MAX_RPM,
    MAX_RPM_RATIO,
    MOTOR_OPERATING_VOLTAGE,
    MOTOR_POWER_MAX_VOLTAGE,
    WHEEL_DIAMETER,
    LR_WHEELS_DISTANCE
);

/* =====================================================
 * VARIABLES
 * ===================================================== */

long long encoder_readings[2] = {0, 0};
long long calculated_cpr[2] = {0, 0};

const char *motor_labels[2] = {
    "FRONT LEFT  - M1",
    "FRONT RIGHT - M2"
};
#define CPR_TEST_TURNS 10
/* =====================================================
 * BTS7960 INITIALIZATION
 * ===================================================== */

void initializeBTS7960()
{
    /* Motor 1 PWM pins */
    pinMode(M1_RPWM, OUTPUT);
    pinMode(M1_LPWM, OUTPUT);

    /* Motor 2 PWM pins */
    pinMode(M2_RPWM, OUTPUT);
    pinMode(M2_LPWM, OUTPUT);

    /* Motor 1 Enable */
    pinMode(MOTOR1_REN, OUTPUT);
    pinMode(MOTOR1_LEN, OUTPUT);

    /* Motor 2 Enable */
    pinMode(MOTOR2_REN, OUTPUT);
    pinMode(MOTOR2_LEN, OUTPUT);

    /* Dừng motor trước */
    digitalWrite(M1_RPWM, LOW);
    digitalWrite(M1_LPWM, LOW);

    digitalWrite(M2_RPWM, LOW);
    digitalWrite(M2_LPWM, LOW);

    /* Enable hai BTS7960 */
    digitalWrite(MOTOR1_REN, HIGH);
    digitalWrite(MOTOR1_LEN, HIGH);

    digitalWrite(MOTOR2_REN, HIGH);
    digitalWrite(MOTOR2_LEN, HIGH);
}

/* =====================================================
 * MOTOR CONTROL
 * ===================================================== */

void stopMotor(uint8_t motor_index)
{
    if (motor_index == 0)
    {
        digitalWrite(M1_RPWM, LOW);
        digitalWrite(M1_LPWM, LOW);
    }
    else
    {
        digitalWrite(M2_RPWM, LOW);
        digitalWrite(M2_LPWM, LOW);
    }
}

void stopAllMotors()
{
    stopMotor(0);
    stopMotor(1);
}
/* =====================================================
 * MANUAL CPR MEASUREMENT
 * ===================================================== */

void resetManualCPR()
{
    // Bảo đảm motor không chạy
    stopAllMotors();

    // Reset encoder về 0
    encoders[0]->write(0);
    encoders[1]->write(0);

    Serial.println();
    Serial.println("============= MANUAL CPR TEST =============");
    Serial.println("Encoder M1 va M2 da duoc reset ve 0.");
    Serial.print("Quay bang tay mot banh dung ");
    Serial.print(CPR_TEST_TURNS);
    Serial.println(" vong.");
    Serial.println("Chi quay mot banh trong moi lan do.");
    Serial.println("Sau khi quay xong, nhap: readcpr");
    Serial.println("===========================================");
    Serial.println();
}

void printManualCPR()
{
    stopAllMotors();

    long long m1_count = encoders[0]->read();
    long long m2_count = encoders[1]->read();

    long long m1_abs = (m1_count < 0) ? -m1_count : m1_count;
    long long m2_abs = (m2_count < 0) ? -m2_count : m2_count;

    float cpr1 = m1_abs / static_cast<float>(CPR_TEST_TURNS);
    float cpr2 = m2_abs / static_cast<float>(CPR_TEST_TURNS);

    Serial.println();
    Serial.println("=========== MANUAL CPR RESULT ===========");

    Serial.print("FRONT LEFT  - M1: count = ");
    Serial.print(m1_count);
    Serial.print(" | CPR = ");
    Serial.println(cpr1, 2);

    Serial.print("FRONT RIGHT - M2: count = ");
    Serial.print(m2_count);
    Serial.print(" | CPR = ");
    Serial.println(cpr2, 2);

    Serial.println("=========================================");
    Serial.println("Nhap cpr de reset va thuc hien lan do moi.");
    Serial.println();
}
/*
 * Chạy motor theo chiều tiến ở 100%.
 *
 * Quy ước ban đầu dựa trên code Arduino đã chạy:
 * RPWM = HIGH
 * LPWM = LOW
 *
 * MOTORX_INV=true sẽ tự đảo hai tín hiệu.
 */
void runMotorForward(uint8_t motor_index)
{
    bool inverted = false;

    if (motor_index == 0)
    {
        inverted = MOTOR1_INV;

        if (!inverted)
        {
            digitalWrite(M1_LPWM, LOW);
            digitalWrite(M1_RPWM, HIGH);
        }
        else
        {
            digitalWrite(M1_RPWM, LOW);
            digitalWrite(M1_LPWM, HIGH);
        }
    }
    else
    {
        inverted = MOTOR2_INV;

        if (!inverted)
        {
            digitalWrite(M2_LPWM, LOW);
            digitalWrite(M2_RPWM, HIGH);
        }
        else
        {
            digitalWrite(M2_RPWM, LOW);
            digitalWrite(M2_LPWM, HIGH);
        }
    }
}

/* =====================================================
 * MOTOR SAMPLING
 * ===================================================== */

void sampleMotors(bool show_summary)
{
    const float measured_voltage = constrain(
        MOTOR_POWER_MEASURED_VOLTAGE,
        0.0f,
        static_cast<float>(MOTOR_OPERATING_VOLTAGE)
    );

    const float scaled_max_rpm =
        (measured_voltage / MOTOR_OPERATING_VOLTAGE) *
        MOTOR_MAX_RPM;

    const float total_revolutions =
        scaled_max_rpm *
        (SAMPLE_TIME_SECONDS / 60.0f);

    Serial.println();
    Serial.print("Measured voltage: ");
    Serial.print(measured_voltage);
    Serial.println(" V");

    Serial.print("Estimated maximum RPM: ");
    Serial.println(scaled_max_rpm);

    Serial.print("Estimated revolutions in 10 seconds: ");
    Serial.println(total_revolutions);
    Serial.println();

    for (uint8_t i = 0; i < 2; i++)
    {
        stopAllMotors();
        delay(1000);

        encoders[i]->write(0);

        Serial.print("SPINNING ");
        Serial.print(motor_labels[i]);
        Serial.print(": ");

        const unsigned long start_time = micros();
        unsigned long last_status_time = start_time;

        runMotorForward(i);

        while ((unsigned long)(micros() - start_time) < SAMPLE_TIME_US)
        {
            if ((unsigned long)(micros() - last_status_time) >= 1000000UL)
            {
                last_status_time = micros();
                Serial.print(".");
            }

            // Cho ESP32 xử lý các tác vụ nền, tránh watchdog reset.
            delay(1);
        }

        stopMotor(i);
        delay(500);

        encoder_readings[i] = encoders[i]->read();

        if (total_revolutions > 0.0f)
        {
            calculated_cpr[i] =
                static_cast<long long>(
                    encoder_readings[i] / total_revolutions
                );
        }
        else
        {
            calculated_cpr[i] = 0;
        }

        Serial.println();
    }

    stopAllMotors();

    if (show_summary)
    {
        printSummary();
    }
}

/* =====================================================
 * SUMMARY
 * ===================================================== */

void printSummary()
{
    Serial.println();
    Serial.println(
        "================ MOTOR ENCODER READINGS ================"
    );

    Serial.print("FRONT LEFT  - M1: ");
    Serial.println(encoder_readings[0]);

    Serial.print("FRONT RIGHT - M2: ");
    Serial.println(encoder_readings[1]);

    Serial.println();
    Serial.println(
        "================ COUNTS PER REVOLUTION ================="
    );

    Serial.print("FRONT LEFT  - M1: ");
    Serial.println(calculated_cpr[0]);

    Serial.print("FRONT RIGHT - M2: ");
    Serial.println(calculated_cpr[1]);

    Serial.println();
    Serial.println(
        "==================== MAX VELOCITIES ===================="
    );

    const float max_rpm = kinematics.getMaxRPM();

    Kinematics::velocities max_linear =
        kinematics.getVelocities(
            max_rpm,
            max_rpm,
            max_rpm,
            max_rpm
        );

    Kinematics::velocities max_angular =
        kinematics.getVelocities(
            -max_rpm,
            max_rpm,
            -max_rpm,
            max_rpm
        );

    Serial.print("Linear Velocity: +/- ");
    Serial.print(max_linear.linear_x);
    Serial.println(" m/s");

    Serial.print("Angular Velocity: +/- ");
    Serial.print(max_angular.angular_z);
    Serial.println(" rad/s");

    Serial.println();
    Serial.println("Danh gia ket qua:");

    if (encoder_readings[0] < 0)
    {
        Serial.println(
            "- M1 encoder am: dat MOTOR1_ENCODER_INV = true"
        );
    }
    else
    {
        Serial.println("- M1 encoder duong: dung");
    }

    if (encoder_readings[1] < 0)
    {
        Serial.println(
            "- M2 encoder am: dat MOTOR2_ENCODER_INV = true"
        );
    }
    else
    {
        Serial.println("- M2 encoder duong: dung");
    }

    Serial.println();
}

/* =====================================================
 * COMMAND HANDLER
 * ===================================================== */

void processCommand(String command)
{
    command.trim();
    command.toLowerCase();

    if (command == "spin")
    {
        Serial.println();
        Serial.println("BAT DAU MOTOR CHECK");
        Serial.println("Moi motor chay 10 giay.");

        sampleMotors(false);

        Serial.println();
        Serial.println("KET THUC MOTOR CHECK");
    }
    else if (command == "sample")
    {
        Serial.println();
        Serial.println("BAT DAU ENCODER CALIBRATION");

        sampleMotors(true);

        Serial.println("KET THUC ENCODER CALIBRATION");
    }
    else if (command == "cpr")
    {
        resetManualCPR();
    }
    else if (command == "readcpr")
    {
        printManualCPR();
    }
    else if (command == "stop")
    {
        stopAllMotors();
        Serial.println("Da dung tat ca motor.");
    }
    else if (command.length() > 0)
    {
        Serial.print("Lenh khong hop le: ");
        Serial.println(command);
    }
}

/* =====================================================
 * SETUP
 * ===================================================== */

void setup()
{
    Serial.begin(115200);
    delay(1000);

    initializeBTS7960();
    stopAllMotors();

    Serial.println();
    Serial.println("ESP32 NodeMCU-32S calibration");
    Serial.println("BTS7960 enabled.");
    Serial.println();

    Serial.println(
        "Dam bao robot dang duoc ke cao, banh xe khong cham dat."
    );

    Serial.println();
    Serial.println("Nhap spin    : kiem tra chieu motor");
    Serial.println("Nhap sample  : lay encoder va CPR uoc luong");
    Serial.println("Nhap cpr     : reset encoder de do CPR bang tay");
    Serial.println("Nhap readcpr : doc count va tinh CPR sau khi quay 10 vong");
    Serial.println("Nhap stop    : dung motor");
    Serial.println();
}

/* =====================================================
 * LOOP
 * ===================================================== */

void loop()
{
    static String command = "";

    while (Serial.available() > 0)
    {
        const char character = Serial.read();

        if (character == '\r' || character == '\n')
        {
            if (command.length() > 0)
            {
                Serial.println(command);
                processCommand(command);
                command = "";
            }
        }
        else
        {
            command += character;
        }
    }
}