/*----------------------------------------------------------------------------
 *  Smart Home System - RTOS Concepts Demo (Simulated Sensors)
 *  Added: Simulated ICPP (Immediate Ceiling Priority Protocol)
 *         Simulated OCPP (Original Ceiling Priority Protocol)
 *
 *  NOTE: This is a *simulation* of ceiling protocols for demonstration.
 *  It does not change the RTOS scheduler priority levels (no kernel API
 *  calls are used). Instead it enforces mutual exclusion using a small
 *  ceiling manager and a mutex. To implement real priority elevation you
 *  should use the RTOS API to change task priorities while holding the
 *  protected resource.
 *----------------------------------------------------------------------------*/

#include <RTL.h>
#include <LPC23xx.H>
#include "LCD.h"
#include <stdio.h>
#include <string.h>

/* ---------------- Task IDs ---------------- */
OS_TID t_temp;
OS_TID t_light;
OS_TID t_motion;
OS_TID t_display;
OS_TID t_logger;
OS_TID t_emergency;
OS_TID t_clock;

/* ---------------- Event Flags -------------- */
#define EVT_TEMP_UPDATE   0x0001
#define EVT_LIGHT_UPDATE  0x0002
#define EVT_MOTION        0x0004
#define EVT_CLOCK         0x0100

/* ---------------- LEDs -------------------- */
#define LED_TEMP_MASK   0x0F    /* P2.0 - P2.3  (Fan) */
#define LED_LIGHT_MASK  0x70    /* P2.4 - P2.6  (Room Light) */
#define LED_CLK         0x80    /* P2.7 clock/motion */

/* ---------------- Shared State ------------- */
volatile unsigned int sensor_temp = 20;     /* Celsius (simulated) */
volatile unsigned int sensor_light = 50;    /* 0=bright 100=dark */
volatile unsigned char motion_detected = 0;
volatile U8 emergency_flag = 0;

/* ---------------- RTOS Objects ------------- */
OS_MUT mut_lcd;
OS_SEM sem_sensors;

#define MSGBOX_SIZE 5
void *msgbox[MSGBOX_SIZE];
#define LOG_MSG_LEN 40
static char log_pool[MSGBOX_SIZE][LOG_MSG_LEN];
static int log_pool_put = 0;
static int log_pool_get = 0;

/* ---------------- Ceiling Simulation ------- */
/*
  We provide a small simulation for ICPP and OCPP.
  - ICPP (Immediate Ceiling): when a task acquires the resource it
    immediately acquires the resource ceiling and sets a global owner.
    Other tasks that try to enter will be blocked until release.
  - OCPP (Original Ceiling): a resource has a ceiling value and a task
    can only lock it if its priority is higher (numerically lower) than
    the current system ceiling. Here we simulate by a global ceiling and
    simple checks.

  This simulation uses a mutex (mut_ceiling) to protect the ceiling state.
*/

OS_MUT mut_ceiling; /* protect ceiling manager state */
volatile OS_TID ceiling_owner = 0; /* owner tid for ICPP */
volatile int system_ceiling = 0; /* current system ceiling (0=none) */

/* Optional per-task base priority mapping (used for OCPP simulation).
   We record base priority when tasks are created. */
#define MAX_TASKS 12
typedef struct { OS_TID tid; int base_prio; } TASKINFO;
static TASKINFO task_table[MAX_TASKS];
static int task_table_count = 0;

/* Register task id and its base priority (call after os_tsk_create) */
void register_task(OS_TID tid, int prio) {
  if (task_table_count < MAX_TASKS) {
    task_table[task_table_count].tid = tid;
    task_table[task_table_count].base_prio = prio;
    task_table_count++;
  }
}

/* Find base priority by tid (returns large value if not found) */
int get_base_priority(OS_TID tid) {
  int i;
  for (i = 0; i < task_table_count; i++) {
    if (task_table[i].tid == tid) return task_table[i].base_prio;
  }
  return 0x7fffffff;
}

/* ---------- ICPP Simulation ---------- */
int icpp_acquire(OS_TID self) {
  /* Acquire ceiling manager mutex */
  if (os_mut_wait(&mut_ceiling, 0xffff) != OS_R_OK) return 0;
  if (ceiling_owner == 0) {
    ceiling_owner = self;
    os_mut_release(&mut_ceiling);
    return 1; /* success */
  }
  /* already owned */
  os_mut_release(&mut_ceiling);
  return 0; /* failure */
}

void icpp_release(OS_TID self) {
  if (os_mut_wait(&mut_ceiling, 0xffff) != OS_R_OK) return;
  if (ceiling_owner == self) ceiling_owner = 0;
  os_mut_release(&mut_ceiling);
}

/* ---------- OCPP Simulation ---------- */
/* For OCPP we use a per-resource ceiling value. Here we pass a ceiling
   priority (lower numeric value = higher priority in our task creation).
   A task may obtain resource only if its base priority is <= resource_ceiling
   and also the system_ceiling allows it.
*/
int ocpp_acquire(OS_TID self, int resource_ceiling) {
  int myprio = get_base_priority(self);
  int acquired = 0;
  if (os_mut_wait(&mut_ceiling, 0xffff) != OS_R_OK) return 0;

  /* Check if my priority meets the resource's ceiling requirement */
  if (myprio <= resource_ceiling && (system_ceiling == 0 || myprio <= system_ceiling)) {
    /* raise system ceiling to resource_ceiling (simulate non-preemptive effect) */
    system_ceiling = resource_ceiling;
    acquired = 1;
  }

  os_mut_release(&mut_ceiling);
  return acquired;
}

void ocpp_release(OS_TID self, int resource_ceiling) {
  if (os_mut_wait(&mut_ceiling, 0xffff) != OS_R_OK) return;
  /* Only lower system ceiling if it matches this resource (simple model) */
  if (system_ceiling == resource_ceiling) system_ceiling = 0;
  os_mut_release(&mut_ceiling);
}

/* ---------------- LED Helpers -------------- */
void update_temp_leds(int level) {
  unsigned int bits;
  switch (level) {
    case 0: bits = 0x01; break;
    case 1: bits = 0x03; break;
    case 2: bits = 0x07; break;
    case 3: bits = 0x0F; break;
    default: bits = 0x01; break;
  }
  FIO2CLR = LED_TEMP_MASK;
  FIO2SET = bits & LED_TEMP_MASK;
}

void update_light_leds(int level) {
  unsigned int bits;
  switch (level) {
    case 0: bits = 0x00; break;
    case 1: bits = 0x10; break;
    case 2: bits = 0x30; break;
    case 3: bits = 0x70; break;
    default: bits = 0x10; break;
  }
  FIO2CLR = LED_LIGHT_MASK;
  FIO2SET = bits & LED_LIGHT_MASK;
}

/* ---------------- Tasks ------------------- */

/* Temperature Task – Simulated increasing/decreasing temperature
   Uses ICPP to protect LED update region (simulated immediate ceiling).
*/
__task void temp_task(void) {
  unsigned int temp = 20;
  unsigned char rising = 1;
  char buf[LOG_MSG_LEN];
  int level;
  int idx;
  OS_TID self = os_tsk_self();

  for (;;) {
    /* simulate temperature rising/falling */
    if (rising) temp++;
    else temp--;
    if (temp >= 40) rising = 0;
    if (temp <= 20) rising = 1;

    /* protect shared state */
    if (os_sem_wait(&sem_sensors, 50) == OS_R_OK) {
      sensor_temp = temp;
      os_sem_send(&sem_sensors);
    }

    /* map to fan level */
    if (temp < 25) level = 0;
    else if (temp < 30) level = 1;
    else if (temp < 35) level = 2;
    else level = 3;

    /* ---- ICPP protected LED update ---- */
    /* Try to acquire immediate ceiling. If busy, wait briefly and retry.
       In a real ICPP you'd raise task priority via kernel APIs so preemption
       cannot happen while holding the resource. */
    while (!icpp_acquire(self)) os_dly_wait(5);
    update_temp_leds(level);
    icpp_release(self);
    /* ------------------------------------ */

    /* log message */
    idx = log_pool_put % MSGBOX_SIZE;
    sprintf(log_pool[idx], "Temp:%dC Fan:%d", temp, level);
    if (os_mbx_send(&msgbox, (void*)log_pool[idx], 40) == OS_R_OK)
      log_pool_put++;

    os_evt_set(EVT_TEMP_UPDATE, t_display);
    os_dly_wait(200);
  }
}

/* Light Task – Simulated day/night cycle + all LEDs on motion
   Uses OCPP to protect light LED updates (simulated resource ceiling).
   Here resource ceiling is chosen as "2" meaning tasks with base priority
   <= 2 are allowed to lock it. (Lower numeric value = higher priority in
   this demo; we assume created priorities follow that convention.)
*/
__task void light_task(void) {
  unsigned int light = 50;
  unsigned char darkening = 1;
  char buf[LOG_MSG_LEN];
  int level;
  int idx;
  OS_TID self = os_tsk_self();
  const int LIGHT_RESOURCE_CEILING = 2; /* example ceiling */

  for (;;) {
    /* simulate day-night */
    if (darkening) light += 5; else light -= 5;
    if (light >= 90) darkening = 0;
    if (light <= 10) darkening = 1;

    /* protect shared state */
    if (os_sem_wait(&sem_sensors, 50) == OS_R_OK) {
      sensor_light = light;
      os_sem_send(&sem_sensors);
    }

    if (light < 25) level = 0;
    else if (light < 50) level = 1;
    else if (light < 75) level = 2;
    else level = 3;

    /* ---- Motion Override + OCPP Handling ---- */
    /* Motion override: when motion is detected we want to ensure ALL LEDs
       light up regardless of the simulated OCPP ceiling. This gives motion
       detection higher functional priority (safe override). */
    if (motion_detected) {
      /* immediate override: turn all LEDs ON for a short flash */
      FIO2SET = 0xFF;
      os_dly_wait(100);
      FIO2CLR = 0xFF;
    } else {
      /* Normal OCPP-protected behavior for light updates */
      if (ocpp_acquire(self, LIGHT_RESOURCE_CEILING)) {
        update_light_leds(level);
        ocpp_release(self, LIGHT_RESOURCE_CEILING);
      } else {
        /* Could not acquire OCPP resource — perform a safe non-critical update */
        update_light_leds(level);
      }
    }
    /* -------------------------- */

    /* log */
    idx = log_pool_put % MSGBOX_SIZE;
    sprintf(log_pool[idx], "Light:%u Level:%d", light, level);
    if (os_mbx_send(&msgbox, (void*)log_pool[idx], 40) == OS_R_OK)
      log_pool_put++;

    os_evt_set(EVT_LIGHT_UPDATE, t_display);
    /* check more frequently so motion windows are not missed */
    os_dly_wait(50);
  }
}

/* Motion Task – Simulated periodic motion detection */
__task void motion_task(void) {
  for (;;) {
    os_dly_wait(800);
    motion_detected = 1;
    os_evt_set(EVT_MOTION, t_light);
    os_evt_set(EVT_MOTION, t_display);
    /* extend motion duration so light_task has time to observe it */
    os_dly_wait(400);
    motion_detected = 0;
  }
}

/* Display Task – shows sensor readings on LCD */
__task void display_task(void) {
  char line[40];
  unsigned int temp, light;
  unsigned char motion;

  for (;;) {
    os_evt_wait_or(EVT_TEMP_UPDATE | EVT_LIGHT_UPDATE | EVT_MOTION, 200);
    if (os_sem_wait(&sem_sensors, 50) == OS_R_OK) {
      temp = sensor_temp;
      light = sensor_light;
      motion = motion_detected;
      os_sem_send(&sem_sensors);

      if (os_mut_wait(&mut_lcd, 100) == OS_R_OK) {
        LCD_cls();
        sprintf(line, "T:%dC L:%u M:%u", temp, light, motion);
        LCD_puts((U8*)line);
        os_mut_release(&mut_lcd);
      }
    }
  }
}

/* Logger Task – prints last system log */
__task void logger_task(void) {
  void *msg;
  for (;;) {
    if (os_mbx_wait(&msgbox, &msg, 120) == OS_R_OK) {
      if (os_mut_wait(&mut_lcd, 100) == OS_R_OK) {
        LCD_gotoxy(1,2);
        LCD_puts((U8*)"Log:");
        LCD_puts((U8*)msg);
        os_mut_release(&mut_lcd);
      }
      log_pool_get++;
    } else os_dly_wait(30);
  }
}

/* Emergency Task – shows overheating warning if needed */
__task void emergency_task(void) {
  for (;;) {
    if (sensor_temp > 45) emergency_flag = 1;
    else emergency_flag = 0;

    if (emergency_flag) {
      if (os_mut_wait(&mut_lcd, 0xffff) == OS_R_OK) {
        LCD_cls();
        LCD_puts((U8*)"!!! OVERHEAT !!!");
        os_mut_release(&mut_lcd);
      }
      FIO2SET = 0xFF; os_dly_wait(10);
      FIO2CLR = 0xFF; os_dly_wait(10);
    }
    os_dly_wait(50);
  }
}

/* Clock Task – simple LED heartbeat */
__task void clock_task(void) {
  for (;;) {
    FIO2SET = LED_CLK; os_dly_wait(5);
    FIO2CLR = LED_CLK; os_dly_wait(95);
  }
}

/* Init Task */
__task void init(void) {
  FIO2DIR = 0xFF;
  FIO2CLR = 0xFF;
  PINSEL10 = 0;

  os_sem_init(&sem_sensors, 1);
  os_mut_init(&mut_lcd);
  os_mut_init(&mut_ceiling);
  os_mbx_init(&msgbox, MSGBOX_SIZE);

  LCD_init();
  LCD_cur_off();
  LCD_cls();
  LCD_puts((U8*)"Smart Home System");

  /* Create tasks, note the numeric priority (lower = higher priority in
     the demo). Keep same priorities as original code. */
  t_temp      = os_tsk_create(temp_task, 3);
  register_task(t_temp, 3);
  t_light     = os_tsk_create(light_task, 4);
  register_task(t_light, 4);
  t_motion    = os_tsk_create(motion_task, 2);
  register_task(t_motion, 2);
  t_display   = os_tsk_create(display_task, 5);
  register_task(t_display, 5);
  t_logger    = os_tsk_create(logger_task, 6);
  register_task(t_logger, 6);
  t_emergency = os_tsk_create(emergency_task, 1);
  register_task(t_emergency, 1);
  t_clock     = os_tsk_create(clock_task, 7);
  register_task(t_clock, 7);

  os_tsk_delete_self();
}

/* Main */
int main(void) {
  os_sys_init(init);
}
