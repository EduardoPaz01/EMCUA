#ifndef MOTOR_CONTROLLER_H_
#define MOTOR_CONTROLLER_H_

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "../uart/UART.h"

// PINNING
#define clockwise_PIN PD3        // CH5 - 5 - PIN_3
#define counterclockwise_PIN PD5 // CH5 - 6 - PIN_5

// ABSTRACTION
#define ON 1
#define OFF 0

// CLOSED-LOOP CONTROL VARIABLES
#define FREQ_HZ 1000
#define PERIOD_MS (1000/FREQ_HZ)
#define DUTY_MAX 100
#define DUTY_MIN 36
#define A_MOTOR 0.396f
#define B_MOTOR 7.17f
#define KP 0.05f
#define KI 2.0f
#define TS 0.05f

typedef struct {
  unsigned long rpm;
  uint8_t duty;
} LogData;

// GENERAL
/**
 * @typedef LogData
 * @property {unsigned long} rpm - Measured RPM value.
 * @property {uint8_t} duty - Applied duty cycle (0-100).
 */

/**
 * Initialize motor control GPIO pins.
 *
 * Configures the direction control pins as outputs. This must be called
 * before any other motor control functions.
 *
 * @function motorInit
 * @returns {void}
 */
void motorInit(void);

/**
 * Set or clear motor direction output pins.
 *
 * @function motorWrite
 * @param {int8_t} direction - Direction selector: 1 = clockwise, -1 = counterclockwise.
 * @param {uint8_t} powerMode - Power mode: ON (1) to enable, OFF (0) to disable.
 * @returns {void}
 */
void motorWrite(int8_t direction, uint8_t powerMode);

/**
 * Initialize PWM timers used for motor speed control.
 *
 * Configures Timer0 and Timer2 for Fast PWM and sets initial duty to 0%.
 *
 * @function pwmInit
 * @returns {void}
 */
void pwmInit(void);

/**
 * Apply PWM to the motor for the given direction.
 *
 * Sets the appropriate hardware compare register and enables the
 * corresponding output compare channel for the requested direction.
 * Duty is clamped to the 0-100 range.
 *
 * @function applyPWM
 * @param {int8_t} direction - 1 for clockwise, -1 for counterclockwise, 0 to stop.
 * @param {uint8_t} duty - Duty cycle percentage (0-100).
 * @returns {void}
 */
void applyPWM(int8_t direction, uint8_t duty);

// OPEN-LOOP CONTROL
/**
 * Convert an applied voltage magnitude to an 8-bit duty value.
 *
 * The function clamps the input voltage to the supported range (-12..12 V)
 * and returns an 8-bit PWM duty scaled from the magnitude of the voltage.
 *
 * @function voltageToDuty
 * @param {float} voltage - Desired motor voltage (-12.0 .. 12.0 V).
 * @returns {uint8_t} 8-bit duty value (0-255).
 */
uint8_t voltageToDuty(float voltage);

/**
 * Ramp motor voltage from v_init to v_target over accel_time_ms.
 *
 * The routine steps the applied voltage in timed increments using
 * periodic delays of PERIOD_MS between steps.
 *
 * @function motorRamp
 * @param {float} v_init - Initial voltage (V).
 * @param {float} v_target - Target voltage (V).
 * @param {uint16_t} accel_time_ms - Total ramp time in milliseconds.
 * @returns {void}
 */
void motorRamp(float v_init, float v_target, uint16_t accel_time_ms);

// CLOSED-LOOP CONTROL
/**
 * Discrete motor model update.
 *
 * Computes the next predicted RPM based on the discrete-time model
 * y[k] = A_MOTOR * y[k-1] + B_MOTOR * u[k-1], where the input is the duty.
 *
 * @function motorDiscrete
 * @param {float} duty - Applied duty value (0-100).
 * @returns {float} Predicted RPM value.
 */
float motorDiscrete(float duty);

/**
 * Compute and apply PI control action.
 *
 * Uses the measured RPM and internal PI state to compute a new duty
 * command. Outputs are saturated to DUTY_MIN..DUTY_MAX to prevent wind-up.
 *
 * @function applyControl
 * @param {unsigned long} rpm_measured - Measured motor RPM.
 * @returns {uint8_t} Duty command as a percentage (clamped to 0-100).
 */
uint8_t applyControl(unsigned long rpm_measured);

/**
 * Store a sample (RPM and duty) in the internal log buffer.
 *
 * @function storeValues
 * @param {unsigned long} rpm - Measured RPM to store.
 * @param {uint8_t} duty - Applied duty to store.
 * @returns {void}
 */
void storeValues(unsigned long rpm, uint8_t duty);

/**
 * Transmit or process the stored log buffer.
 *
 * Sends collected `LogData` entries (e.g., over UART). Implementation may
 * be platform-specific; the declaration is provided for higher-level use.
 *
 * @function sendLog
 * @returns {void}
 */
void sendLog(void);

/**
 * Update the reference RPM used by the closed-loop controller.
 *
 * @function setRPM_REF
 * @param {uint16_t} ref - Desired reference RPM.
 * @returns {void}
 */
void setRPM_REF(uint16_t ref);

/**
 * Retrieve the last applied duty cycle.
 *
 * Returns the most recent duty cycle command applied to the motor
 * (as a percentage between 0 and 100). Useful for monitoring or
 * debugging the closed-loop controller state.
 *
 * @function getDuty
 * @returns {uint8_t} Last duty cycle percentage (0-100).
 */
uint8_t getDuty(void);

/**
 * Set open-loop PWM reference duty.
 *
 * Stores the desired duty reference used by the open-loop routines
 * (ramp/step). The value is interpreted as percentage (0-100).
 *
 * @param duty_ref_ Duty reference percentage (0..100).
 */
void setPWM_Ref(uint8_t duty_ref_);

/**
 * Set the open-loop PWM step increment.
 *
 * Controls how much the applied duty changes each call to
 * `applyPWM_Ref()` when moving toward the stored reference.
 *
 * @param duty_step_ Step increment in percentage points (typically 1..100).
 */
void setStepPWM(uint8_t duty_step_);

/**
 * Gradually adjust and apply PWM toward the stored reference.
 *
 * This function should be called periodically. It moves the current
 * duty toward `duty_ref` by `duty_step` and applies the resulting
 * PWM output (uses clockwise direction in open-loop mode).
 */
void applyPWM_Ref(void);

/**
 * Set the numeric gain multiplier used by the closed-loop controller.
 *
 * The value is used to scale the proportional contribution of the PI
 * controller (useful for coarse tuning). Valid range depends on the
 * application but is typically 1..255.
 *
 * @param GAIN_ Integer gain multiplier (1..255).
 */
void setGAIN(uint8_t GAIN_);

// G(s)= 11,87 / 0,054s+1
// G(z)= 7,17​​  / z−0,396
// y[k] = 0,396y[k−1] + 7,17u[k−1]
// y[k] = RPM 
// u[k] = duty cicle (%)	​

#endif /* MOTOR_CONTROLLER_H_ */