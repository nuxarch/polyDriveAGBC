#include <Arduino.h>
#include <SimpleFOC.h>
#include "DRV8301.h"
float target_velocity;
int delay_test = 5000;
#define THROTTLE_PIN 33
#define INH_A 25
#define INH_B 26
#define INH_C 27

#define EN_GATE 14
#define M_PWM 19
#define M_OC 18
#define OC_ADJ 21

// Motor instance
BLDCMotor motor = BLDCMotor(23, 2.5);
BLDCDriver3PWM driver = BLDCDriver3PWM(INH_A, INH_B, INH_C, EN_GATE);

// SENSOR
// HallSensor sensor = HallSensor(32, 35, 34, 13);
HallSensor sensor = HallSensor(32, 35, 33, 23);
void doA() { sensor.handleA(); }
void doB() { sensor.handleB(); }
void doC() { sensor.handleC(); }

Commander command = Commander(Serial);
void onMotor(char *cmd) { command.motor(&motor, cmd); }
void doTarget(char *cmd) { command.scalar(&target_velocity, cmd); }

void vel_PID()
{
    motor.PID_velocity.P = 1;
    motor.PID_velocity.I = 0;
    motor.PID_velocity.D = 0;
    motor.PID_velocity.output_ramp = 10000;
    motor.PID_velocity.limit = 1;
    motor.LPF_velocity = 1;
}
void angle_PID()
{
    motor.P_angle.P = 100;
    motor.P_angle.I = 0;
    motor.P_angle.D = 0;
    motor.P_angle.output_ramp = 10000;
    motor.P_angle.limit = 10;
    motor.LPF_angle = 1;
}
void limits()
{
    // motor.phase_resistance = 2.5;
    motor.phase_resistance = 1.5;
    motor.velocity_limit = 20; // 2.0;
    motor.voltage_limit = 20;
    motor.current_limit = 3; // 1.0;
}

xTaskHandle taskBlinkHandle;
int angle_max = 3.14 * 20;
void taskAngle(void *parameter)
{
    while (1)
    {
        vTaskDelay(delay_test / portTICK_PERIOD_MS);
        target_velocity = 6.28;
        vTaskDelay(delay_test / portTICK_PERIOD_MS);
        target_velocity = 0;
    }
}
void taskVel(void *parameter)
{
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    for (;;)
    {
        /*place your rtos code here*/
        for (int i = 0; i < 5; i += 2)
        {
            vTaskDelay(delay_test / portTICK_PERIOD_MS);
            target_velocity = i;
        }
        for (int i = 5; i >= 0; i -= 2)
        {
            vTaskDelay(delay_test / portTICK_PERIOD_MS);
            target_velocity = i;
        }
    }
}
void init_hall()
{
    _delay(100);
    sensor.pullup = Pullup::USE_EXTERN;
    sensor.init();
    sensor.enableInterrupts(doA, doB, doC);
    Serial.println("Sensor ready");
    _delay(500);
    
}
void init_board()
{
    // DRV8302 specific code
    // M_OC  - enable overcurrent protection
    pinMode(M_OC, OUTPUT);
    digitalWrite(M_OC, LOW);
    // M_PWM  - enable 3pwm mode
    pinMode(M_PWM, OUTPUT);
    digitalWrite(M_PWM, HIGH);
    // OD_ADJ - set the maximum overcurrent limit possible
    // Better option would be to use voltage divisor to set exact value
    pinMode(OC_ADJ, OUTPUT);
    digitalWrite(OC_ADJ, HIGH);
    // driver config
    // power supply voltage [V]
    driver.voltage_power_supply = 45;
    driver.init();
    // link the motor and the driver
    motor.linkDriver(&driver);
    motor.voltage_sensor_align = 10;
    motor.velocity_index_search = 4;
    motor.linkSensor(&sensor);
    // motor.phase_resistance = 0.0;
    // choose FOC modulation
    // motor.foc_modulation = FOCModulationType::SinePWM;
}

void clear_error(char *cmd)
{
    // gate_driver.begin(PWM_INPUT_MODE_3PWM);
    // delay(1000);
    // Serial.println("Is error " + String(gate_driver.get_PWM_Mode()));
    // Serial.println("error :" + String(gate_driver.read_fault()));
    // Serial.println("resetting eeror");
    // delay(1000);
    // gate_driver.reset();
    // delay(1000);
    // Serial.println("Is error " + String(gate_driver.get_PWM_Mode()));
    // Serial.println("error :" + String(gate_driver.read_fault()));
    // delay(1000);
}
void restart(char *cmd)
{
    // if (motor.controller == MotionControlType::angle)
    // {
    //     xTaskCreate(taskAngle, "task ANGLE", 4000, NULL, 1, NULL);
    // }
    // else if (motor.controller == MotionControlType::velocity)
    // {
    //     xTaskCreate(taskVel, "task velocity", 4000, NULL, 1, NULL);
    // }
    motor.init();
    // motor.initFOC();
    motor.initFOC(1.05, Direction::CW);
}
void reboot(char *cmd)
{
    init_hall();
    init_board();
    vel_PID();
    angle_PID();
    limits();
    Serial.begin(115200);
    motor.useMonitoring(Serial);
    // motor.controller = MotionControlType::angle_openloop;
    // motor.controller = MotionControlType::velocity_openloop;
    // motor.controller = MotionControlType::torque;
    // motor.controller = MotionControlType::velocity;
    motor.controller = MotionControlType::angle;
    // motor.PID_velocity.limit = 3;
    // if (motor.controller == MotionControlType::angle)
    // {
    //     xTaskCreate(taskAngle, "task ANGLE", 4000, NULL, 1, NULL);
    // }
    // else if (motor.controller == MotionControlType::velocity)
    // {
    //     xTaskCreate(taskVel, "task velocity", 4000, NULL, 1, NULL);
    // }
    motor.init();
    // motor.initFOC();
    motor.initFOC(1.05, Direction::CW);
}
void vel_mode(char *cmd)
{
    motor.controller = MotionControlType::velocity;
}
void pos_mode(char *cmd)
{
    motor.controller = MotionControlType::angle;
}
void setup()
{
    init_hall();
    init_board();
    vel_PID();
    // angle_PID();
    limits();
    Serial.begin(115200);
    motor.useMonitoring(Serial);
    // motor.controller = MotionControlType::angle_openloop;
    motor.controller = MotionControlType::velocity_openloop;
    // motor.controller = MotionControlType::torque;
    // motor.controller = MotionControlType::velocity;
    // motor.controller = MotionControlType::angle;

    motor.init();
    // motor.initFOC();
    delay(2000);
    // motor.initFOC(1.05, Direction::CW);
    motor.initFOC(4.19, Direction::CW);
    command.add('M', onMotor, "on motor command");
    command.add('T', doTarget, "set target");
    command.add('I', restart, "restart FOC");
    command.add('B', reboot, "reinit PID value");
    command.add('C', clear_error, "clear fault controller");
    command.add('V', vel_mode, "set vel mode");
    command.add('P', pos_mode, "set pos mode");

    Serial.println(F("Motor ready."));
    Serial.println(F("Set the target velocity using serial terminal:"));
    _delay(1000);
    // if (motor.controller == MotionControlType::angle)
    // {
    //     xTaskCreate(taskAngle, "task ANGLE", 4000, NULL, 1, NULL);
    // }
    // else if (motor.controller == MotionControlType::velocity)
    // {
    //     xTaskCreate(taskVel, "task velocity", 4000, NULL, 1, NULL);
    // }
}

void loop()
{
    motor.move(target_velocity);
    motor.monitor();
    motor.loopFOC();
    command.run();
}