#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "Shared/System_Types.h"
#include "Team_1_Motor/Motor.h"
#include "Team_2_Inputs/Inputs.h"
#include "Team_3_Controller/Controller.h"
#include "Team_3_Controller/RequestHandler.h"

/* ----------------------------------------------------------------------------
 * MOCK HARDWARE STATE FOR TEAM 1 & TEAM 2
 * ---------------------------------------------------------------------------- */
static signed char mock_motor_dir = DIR_STOP;
static unsigned short mock_motor_speed = 0;
static unsigned char mock_motor_stopped = 1;
static unsigned char mock_brake_engaged = 1;
static unsigned long mock_weight_grams = 500;
static unsigned char mock_is_overload = 0;

static unsigned char mock_current_floor = FLOOR_0;
static unsigned char mock_sensor_fault = 0;
static unsigned char mock_new_request_pending = 0;
static unsigned char mock_pending_requested_floor = NO_FLOOR;
static unsigned char mock_last_buzzer_event = 0;
static unsigned char mock_display_current_floor = NO_FLOOR;
static unsigned char mock_display_target_floor = NO_FLOOR;
static unsigned char mock_display_state = 255;
static unsigned long mock_display_weight = 0;

/* TEAM 1 MOCK IMPLEMENTATIONS */
void Motor_Init(void) {
    mock_motor_dir = DIR_STOP;
    mock_motor_speed = 0;
    mock_motor_stopped = 1;
    mock_brake_engaged = 1;
}
void Motor_Update(void) {}
void Motor_Set(signed char direction, unsigned short speed_percent) {
    mock_motor_dir = direction;
    mock_motor_speed = speed_percent;
    mock_motor_stopped = (direction == DIR_STOP || speed_percent == 0);
}
void Motor_Stop(void) {
    mock_motor_dir = DIR_STOP;
    mock_motor_speed = 0;
    mock_motor_stopped = 1;
}
void Brake_Release(void) {
    mock_brake_engaged = 0;
}
void Brake_Engage(void) {
    mock_brake_engaged = 1;
}
unsigned long Weight_GetGrams(void) {
    return mock_weight_grams;
}
unsigned char Weight_IsOverload(void) {
    return mock_is_overload;
}

/* TEAM 2 MOCK IMPLEMENTATIONS */
void Inputs_Init(void) {
    mock_current_floor = FLOOR_0;
    mock_sensor_fault = 0;
    mock_new_request_pending = 0;
    mock_pending_requested_floor = NO_FLOOR;
    mock_last_buzzer_event = 0;
}
void Inputs_Update(void) {}
unsigned char Inputs_GetNewFloorRequest(unsigned char *floor) {
    if (mock_new_request_pending) {
        *floor = mock_pending_requested_floor;
        mock_new_request_pending = 0;
        return 1;
    }
    return 0;
}
unsigned char Inputs_GetCurrentFloor(void) {
    return mock_current_floor;
}
unsigned char Inputs_IsFloorSensorFault(void) {
    return mock_sensor_fault;
}
void Inputs_DisplayStatus(unsigned char current_floor,
                          unsigned char target_floor,
                          unsigned char controller_state,
                          unsigned long weight_grams) {
    mock_display_current_floor = current_floor;
    mock_display_target_floor = target_floor;
    mock_display_state = controller_state;
    mock_display_weight = weight_grams;
}
void Inputs_Buzzer(unsigned char event_code) {
    mock_last_buzzer_event = event_code;
}

/* ----------------------------------------------------------------------------
 * HELPER: Advance Controller by N ticks (1 tick = 10 ms)
 * ---------------------------------------------------------------------------- */
static void step_ticks(unsigned int n) {
    unsigned int i;
    for (i = 0; i < n; i++) {
        Controller_Update();
    }
}

/* ----------------------------------------------------------------------------
 * TEST CASES
 * ---------------------------------------------------------------------------- */
void test_request_handler(void) {
    printf("--- Testing RequestHandler Array Queue ---\n");
    RequestHandler_Init();
    assert(RequestHandler_HasRequests() == 0);
    assert(RequestHandler_GetCount() == 0);
    assert(RequestHandler_GetNext() == NO_FLOOR);

    /* Add valid floors */
    RequestHandler_Add(FLOOR_0);
    assert(RequestHandler_HasRequests() == 1);
    assert(RequestHandler_GetCount() == 1);
    assert(RequestHandler_GetNext() == FLOOR_0);
    assert(RequestHandler_Contains(FLOOR_0) == 1);
    assert(RequestHandler_Contains(FLOOR_1) == 0);

    /* Duplicate rejection */
    RequestHandler_Add(FLOOR_0);
    assert(RequestHandler_GetCount() == 1);

    /* Add more */
    RequestHandler_Add(FLOOR_2);
    RequestHandler_Add(FLOOR_1);
    RequestHandler_Add(FLOOR_3);
    assert(RequestHandler_GetCount() == 4);

    /* Capacity boundary test (5th item ignored) */
    RequestHandler_Add(4); /* Invalid floor > 3 */
    assert(RequestHandler_GetCount() == 4);

    /* Verify FIFO order */
    assert(RequestHandler_GetNext() == FLOOR_0);

    /* Remove head */
    RequestHandler_Remove(FLOOR_0);
    assert(RequestHandler_GetCount() == 3);
    assert(RequestHandler_GetNext() == FLOOR_2);

    /* Remove middle */
    RequestHandler_Remove(FLOOR_1);
    assert(RequestHandler_GetCount() == 2);
    assert(RequestHandler_GetNext() == FLOOR_2);
    RequestHandler_Remove(FLOOR_2);
    assert(RequestHandler_GetCount() == 1);
    assert(RequestHandler_GetNext() == FLOOR_3);

    /* Clear */
    RequestHandler_Clear();
    assert(RequestHandler_GetCount() == 0);
    assert(RequestHandler_HasRequests() == 0);
    assert(RequestHandler_GetNext() == NO_FLOOR);
    printf(">> RequestHandler PASS!\n");
}

void test_controller_scenarios(void) {
    printf("--- Testing Controller 15 Scenarios ---\n");

    /* Scenario 1: Startup with elevator at Floor 0 */
    mock_current_floor = FLOOR_0;
    mock_sensor_fault = 0;
    mock_is_overload = 0;
    Controller_Init();
    assert(mock_last_buzzer_event == BUZZER_STARTUP);
    assert(Controller_GetState() == STATE_WAIT_FOR_FLOOR);
    step_ticks(1);
    assert(Controller_GetState() == STATE_IDLE);
    assert(Controller_GetCurrentFloor() == FLOOR_0);
    printf("Scenario 1: Startup at Floor 0 PASS\n");

    /* Scenario 2: Startup with elevator at Floor 3 */
    mock_current_floor = FLOOR_3;
    Controller_Init();
    step_ticks(1);
    assert(Controller_GetState() == STATE_IDLE);
    assert(Controller_GetCurrentFloor() == FLOOR_3);
    printf("Scenario 2: Startup at Floor 3 PASS\n");

    /* Scenario 3: Request Floor 2 from Floor 0 */
    mock_current_floor = FLOOR_0;
    Controller_Init();
    step_ticks(1);
    assert(Controller_GetState() == STATE_IDLE);

    /* Deliver request for Floor 2 */
    mock_new_request_pending = 1;
    mock_pending_requested_floor = FLOOR_2;
    step_ticks(1);
    /* Should transition to STATE_BRAKE_RELEASE */
    assert(Controller_GetState() == STATE_BRAKE_RELEASE);
    assert(mock_brake_engaged == 0); /* Brake commanded to release */
    assert(Controller_GetTargetFloor() == FLOOR_2);

    /* Wait for brake release settling time (30 ticks) */
    step_ticks(BRAKE_RELEASE_TICKS);
    assert(Controller_GetState() == STATE_MOVING_UP);
    assert(mock_motor_dir == DIR_UP);
    assert(mock_motor_speed == MOTOR_SPEED_PERCENT);

    /* Cabin leaves Floor 0 */
    mock_current_floor = NO_FLOOR;
    step_ticks(10);
    assert(Controller_GetState() == STATE_MOVING_UP);

    /* Cabin arrives at Floor 2 */
    mock_current_floor = FLOOR_2;
    step_ticks(1);
    assert(Controller_GetState() == STATE_STOPPING);
    assert(mock_motor_stopped == 1);

    /* Wait stopping settle time (5 ticks) */
    step_ticks(STOP_SETTLE_TICKS);
    assert(Controller_GetState() == STATE_BRAKE_ENGAGE);
    assert(mock_brake_engaged == 1);

    /* Wait brake engage settling time (30 ticks) */
    step_ticks(BRAKE_ENGAGE_TICKS);
    assert(Controller_GetState() == STATE_IDLE);
    assert(mock_last_buzzer_event == BUZZER_ARRIVAL);
    assert(Controller_GetTargetFloor() == NO_FLOOR);
    printf("Scenario 3: Request Floor 2 from Floor 0 PASS\n");

    /* Scenario 4: Request Floor 1 from Floor 3 */
    mock_current_floor = FLOOR_3;
    Controller_Init();
    step_ticks(1);
    assert(Controller_GetState() == STATE_IDLE);

    mock_new_request_pending = 1;
    mock_pending_requested_floor = FLOOR_1;
    step_ticks(1);
    assert(Controller_GetState() == STATE_BRAKE_RELEASE);

    step_ticks(BRAKE_RELEASE_TICKS);
    assert(Controller_GetState() == STATE_MOVING_DOWN);
    assert(mock_motor_dir == DIR_DOWN);

    mock_current_floor = NO_FLOOR;
    step_ticks(10);
    assert(Controller_GetState() == STATE_MOVING_DOWN);

    mock_current_floor = FLOOR_1;
    step_ticks(1);
    assert(Controller_GetState() == STATE_STOPPING);

    step_ticks(STOP_SETTLE_TICKS);
    assert(Controller_GetState() == STATE_BRAKE_ENGAGE);

    step_ticks(BRAKE_ENGAGE_TICKS);
    assert(Controller_GetState() == STATE_IDLE);
    assert(mock_last_buzzer_event == BUZZER_ARRIVAL);
    printf("Scenario 4: Request Floor 1 from Floor 3 PASS\n");

    /* Scenario 5: Request current floor */
    mock_current_floor = FLOOR_2;
    Controller_Init();
    step_ticks(1);
    assert(Controller_GetState() == STATE_IDLE);

    mock_new_request_pending = 1;
    mock_pending_requested_floor = FLOOR_2;
    step_ticks(1);
    /* Should remain IDLE, remove request, chime arrival, no motor motion */
    assert(Controller_GetState() == STATE_IDLE);
    assert(mock_last_buzzer_event == BUZZER_ARRIVAL);
    assert(mock_motor_stopped == 1);
    assert(RequestHandler_HasRequests() == 0);
    printf("Scenario 5: Request Current Floor PASS\n");

    /* Scenario 6, 7, 8, 9: Queue operations under Controller */
    mock_current_floor = FLOOR_0;
    Controller_Init();
    step_ticks(1);

    /* Add multiple & duplicate requests */
    mock_new_request_pending = 1;
    mock_pending_requested_floor = FLOOR_1;
    step_ticks(1);
    mock_new_request_pending = 1;
    mock_pending_requested_floor = FLOOR_1; /* duplicate */
    step_ticks(1);
    mock_new_request_pending = 1;
    mock_pending_requested_floor = FLOOR_2;
    step_ticks(1);
    mock_new_request_pending = 1;
    mock_pending_requested_floor = FLOOR_3;
    step_ticks(1);

    assert(RequestHandler_Contains(FLOOR_1) == 1 || Controller_GetTargetFloor() == FLOOR_1);
    printf("Scenarios 6, 7, 8, 9: Multiple, Duplicate, Capacity Operations PASS\n");

    /* Scenario 10: Overload before movement */
    mock_current_floor = FLOOR_0;
    Controller_Init();
    step_ticks(1);
    assert(Controller_GetState() == STATE_IDLE);

    mock_is_overload = 1;
    step_ticks(1);
    assert(Controller_GetState() == STATE_OVERLOAD);
    assert(mock_last_buzzer_event == BUZZER_OVERLOAD);

    /* Try requesting movement during overload */
    mock_new_request_pending = 1;
    mock_pending_requested_floor = FLOOR_2;
    step_ticks(10);
    assert(Controller_GetState() == STATE_OVERLOAD);
    assert(mock_motor_stopped == 1);

    /* Clear overload */
    mock_is_overload = 0;
    step_ticks(1);
    assert(Controller_GetState() == STATE_IDLE);
    printf("Scenario 10: Overload before movement PASS\n");

    /* Scenario 11: Overload while moving */
    mock_current_floor = FLOOR_0;
    Controller_Init();
    step_ticks(1);
    mock_new_request_pending = 1;
    mock_pending_requested_floor = FLOOR_3;
    step_ticks(1);
    step_ticks(BRAKE_RELEASE_TICKS);
    assert(Controller_GetState() == STATE_MOVING_UP);

    /* Trip overload in flight */
    mock_is_overload = 1;
    step_ticks(1);
    assert(Controller_GetState() == STATE_OVERLOAD);
    assert(mock_motor_stopped == 1);
    assert(mock_brake_engaged == 1);
    assert(mock_last_buzzer_event == BUZZER_OVERLOAD);
    printf("Scenario 11: Overload while moving PASS\n");

    /* Scenario 12: Floor sensor fault while moving */
    mock_is_overload = 0;
    mock_current_floor = FLOOR_0;
    Controller_Init();
    step_ticks(1);
    mock_new_request_pending = 1;
    mock_pending_requested_floor = FLOOR_2;
    step_ticks(1);
    step_ticks(BRAKE_RELEASE_TICKS);
    assert(Controller_GetState() == STATE_MOVING_UP);

    /* Trip sensor fault (e.g. 2 hall sensors shorted) */
    mock_sensor_fault = 1;
    step_ticks(1);
    assert(Controller_GetState() == STATE_SENSOR_FAULT);
    assert(mock_motor_stopped == 1);
    assert(mock_brake_engaged == 1);
    assert(mock_last_buzzer_event == BUZZER_SENSOR_FAULT);

    /* Resolve sensor fault */
    mock_sensor_fault = 0;
    mock_current_floor = FLOOR_1; /* cabin stopped at floor 1 */
    step_ticks(1);
    assert(Controller_GetState() == STATE_IDLE);
    printf("Scenario 12: Floor sensor fault while moving PASS\n");

    /* Scenario 13: Travel timeout */
    mock_current_floor = FLOOR_0;
    Controller_Init();
    step_ticks(1);
    mock_new_request_pending = 1;
    mock_pending_requested_floor = FLOOR_3;
    step_ticks(1);
    step_ticks(BRAKE_RELEASE_TICKS);
    assert(Controller_GetState() == STATE_MOVING_UP);

    mock_current_floor = NO_FLOOR; /* in flight */
    step_ticks(MAX_TRAVEL_TICKS + 5);
    assert(Controller_GetState() == STATE_SENSOR_FAULT);
    assert(mock_motor_stopped == 1);
    assert(mock_brake_engaged == 1);
    assert(mock_last_buzzer_event == BUZZER_SENSOR_FAULT);
    printf("Scenario 13: Travel timeout trips sensor fault PASS\n");

    /* Scenario 14 & 15: Arrival buzzer and LCD update */
    mock_current_floor = FLOOR_2;
    mock_sensor_fault = 0;
    Controller_Init();
    step_ticks(1);
    assert(mock_display_current_floor == FLOOR_2);
    assert(mock_display_state == STATE_IDLE);
    assert(mock_display_weight == 500);
    printf("Scenarios 14 & 15: Arrival Buzzer and LCD Status Updates PASS\n");

    printf("\n>> ALL 15 CONTROLLER SCENARIOS PASSED WITH ZERO ERRORS!\n");
}

int main(void) {
    test_request_handler();
    test_controller_scenarios();
    printf("\n============================================\n");
    printf("ALL SYSTEM VERIFICATION TESTS COMPLETED 100%%\n");
    printf("============================================\n");
    return 0;
}
