#ifndef INC_SCHEDULER_H_
#define INC_SCHEDULER_H_

#include "main.h"

#define SCH_MAX_TASKS   40
#define NO_TASK_ID      0     // returned by SCH_Add_Task when the task list is full
#define TICK            10    // 10 ms timer tick (TIM2)
#define SCH_NIL         0xFF  // "null" index used by the linked lists

typedef enum {
    TASK_FREE = 0,   // slot is unused
    TASK_WAITING,    // in the sorted (delta) list, waiting for its time
    TASK_READY,      // expired, waiting in the ready queue for the dispatcher
    TASK_RUNNING     // currently being executed by the dispatcher
} sTaskState;

typedef struct {
    void (*pTask)(void);   // pointer to the task function
    uint32_t Delay;        // DELTA delay (ticks) relative to the previous task in the list
    uint32_t Period;       // period in ticks (0 = one-shot)
    uint32_t Due;          // absolute tick at which the task is due
    uint32_t TaskID;       // ID returned to the user (slot index + 1)
    uint8_t  Next;         // index of next node in list (SCH_NIL = end)
    uint8_t  State;        // sTaskState
} sTask;

void     SCH_Init(void);
void     SCH_Update(void);          // call from the 10 ms timer ISR, O(1) search
void     SCH_Dispatch_Tasks(void);  // call from the main loop
uint32_t SCH_Add_Task(void (*pFunction)(), uint32_t DELAY, uint32_t PERIOD);
uint8_t  SCH_Delete_Task(uint32_t taskID);

#endif /* INC_SCHEDULER_H_ */
