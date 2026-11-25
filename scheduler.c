#include "scheduler.h"

/*
 * Design (delta list):
 *  - "Active list": tasks waiting for their time, sorted by expiry time.
 *    Each node stores only the DELTA to the previous node, so on every tick
 *    SCH_Update() only decrements the head node  -> no O(n) search in the ISR.
 *  - "Ready queue": FIFO of tasks that have expired, run by SCH_Dispatch_Tasks().
 *    Tasks expiring at the same tick keep their registration order, so callbacks
 *    are always delivered in the correct order.
 *  - Periodic tasks are re-inserted by the dispatcher using absolute due time,
 *    so the period does not drift even if a task runs late.
 */

static sTask SCH_tasks_G[SCH_MAX_TASKS];

static uint8_t  active_head = SCH_NIL;   // sorted delta list
static uint8_t  ready_head  = SCH_NIL;   // ready queue (FIFO)
static uint8_t  ready_tail  = SCH_NIL;
static volatile uint32_t tick_count = 0; // number of ticks since start

/* ---- critical section helpers (protect lists shared with the ISR) ---- */
static inline uint32_t enter_critical(void) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}
static inline void exit_critical(uint32_t primask) {
    __set_PRIMASK(primask);
}

static void free_slot(uint8_t idx) {
    SCH_tasks_G[idx].pTask  = 0;
    SCH_tasks_G[idx].Delay  = 0;
    SCH_tasks_G[idx].Period = 0;
    SCH_tasks_G[idx].Due    = 0;
    SCH_tasks_G[idx].Next   = SCH_NIL;
    SCH_tasks_G[idx].State  = TASK_FREE;
}

/* Insert node into the active list so that it expires after 'ticks' ticks.
 * Equal expiry times are placed AFTER existing nodes (FIFO order).
 * Must be called inside a critical section. Runs in the main loop, not in the ISR. */
static void insert_active(uint8_t idx, uint32_t ticks) {
    uint32_t remaining = ticks;
    uint8_t prev = SCH_NIL;
    uint8_t cur  = active_head;

    while (cur != SCH_NIL && SCH_tasks_G[cur].Delay <= remaining) {
        remaining -= SCH_tasks_G[cur].Delay;
        prev = cur;
        cur  = SCH_tasks_G[cur].Next;
    }

    SCH_tasks_G[idx].Delay = remaining;
    SCH_tasks_G[idx].Next  = cur;
    SCH_tasks_G[idx].State = TASK_WAITING;
    if (cur != SCH_NIL) {
        SCH_tasks_G[cur].Delay -= remaining; // keep following nodes' absolute time
    }
    if (prev == SCH_NIL) {
        active_head = idx;
    } else {
        SCH_tasks_G[prev].Next = idx;
    }
}

void SCH_Init(void) {
    for (uint8_t i = 0; i < SCH_MAX_TASKS; i++) {
        free_slot(i);
        SCH_tasks_G[i].TaskID = i + 1;
    }
    active_head = SCH_NIL;
    ready_head  = SCH_NIL;
    ready_tail  = SCH_NIL;
    tick_count  = 0;
}

/* Called every 10 ms from the timer ISR.
 * Only the head of the sorted list is touched: O(1) per tick (no list search). */
void SCH_Update(void) {
    tick_count++;

    if (active_head != SCH_NIL && SCH_tasks_G[active_head].Delay > 0) {
        SCH_tasks_G[active_head].Delay--;
    }

    // Move every expired task (delta == 0 at the head) to the ready queue
    while (active_head != SCH_NIL && SCH_tasks_G[active_head].Delay == 0) {
        uint8_t idx = active_head;
        active_head = SCH_tasks_G[idx].Next;

        SCH_tasks_G[idx].Next  = SCH_NIL;
        SCH_tasks_G[idx].State = TASK_READY;
        if (ready_tail == SCH_NIL) {
            ready_head = idx;
        } else {
            SCH_tasks_G[ready_tail].Next = idx;
        }
        ready_tail = idx;
    }
}

uint32_t SCH_Add_Task(void (*pFunction)(), uint32_t DELAY, uint32_t PERIOD) {
    if (pFunction == 0) {
        return NO_TASK_ID;
    }

    uint32_t delay_ticks  = DELAY / TICK;
    uint32_t period_ticks = PERIOD / TICK;
    if (delay_ticks < 1) delay_ticks = 1;                 // earliest possible: next tick
    if (PERIOD > 0 && period_ticks < 1) period_ticks = 1; // keep periodic tasks periodic

    uint32_t primask = enter_critical();

    // Find a free slot
    uint8_t idx = 0;
    while (idx < SCH_MAX_TASKS && SCH_tasks_G[idx].State != TASK_FREE) {
        idx++;
    }
    if (idx == SCH_MAX_TASKS) {
        exit_critical(primask);
        return NO_TASK_ID; // task list is full
    }

    SCH_tasks_G[idx].pTask  = pFunction;
    SCH_tasks_G[idx].Period = period_ticks;
    SCH_tasks_G[idx].Due    = tick_count + delay_ticks;
    insert_active(idx, delay_ticks);

    exit_critical(primask);
    return SCH_tasks_G[idx].TaskID;
}

void SCH_Dispatch_Tasks(void) {
    while (1) {
        uint32_t primask = enter_critical();

        // Pop the oldest ready task
        if (ready_head == SCH_NIL) {
            exit_critical(primask);
            break;
        }
        uint8_t idx = ready_head;
        ready_head = SCH_tasks_G[idx].Next;
        if (ready_head == SCH_NIL) {
            ready_tail = SCH_NIL;
        }
        SCH_tasks_G[idx].Next  = SCH_NIL;
        SCH_tasks_G[idx].State = TASK_RUNNING;

        void (*task)(void) = SCH_tasks_G[idx].pTask;
        exit_critical(primask);

        task(); // run the task (interrupts enabled, ticks keep being counted)

        primask = enter_critical();
        // The task may have deleted itself (state changed); only handle it if still RUNNING
        if (SCH_tasks_G[idx].State == TASK_RUNNING) {
            if (SCH_tasks_G[idx].Period == 0) {
                free_slot(idx); // one-shot: remove after running
            } else {
                // Periodic: next due time based on the previous due time (no drift)
                uint32_t next_due = SCH_tasks_G[idx].Due + SCH_tasks_G[idx].Period;
                int32_t  wait     = (int32_t)(next_due - tick_count);
                if (wait < 1) {            // we are late: run again at the next tick
                    wait = 1;
                    next_due = tick_count + 1;
                }
                SCH_tasks_G[idx].Due = next_due;
                insert_active(idx, (uint32_t)wait);
            }
        }
        exit_critical(primask);
    }
}

uint8_t SCH_Delete_Task(uint32_t taskID) {
    if (taskID == NO_TASK_ID || taskID > SCH_MAX_TASKS) {
        return 0;
    }
    uint8_t idx = (uint8_t)(taskID - 1);

    uint32_t primask = enter_critical();

    switch (SCH_tasks_G[idx].State) {
    case TASK_WAITING: {
        // Unlink from the active list and give its delta to the next node
        uint8_t prev = SCH_NIL;
        uint8_t cur  = active_head;
        while (cur != SCH_NIL && cur != idx) {
            prev = cur;
            cur  = SCH_tasks_G[cur].Next;
        }
        if (cur == idx) {
            uint8_t next = SCH_tasks_G[idx].Next;
            if (next != SCH_NIL) {
                SCH_tasks_G[next].Delay += SCH_tasks_G[idx].Delay;
            }
            if (prev == SCH_NIL) {
                active_head = next;
            } else {
                SCH_tasks_G[prev].Next = next;
            }
        }
        free_slot(idx);
        break;
    }
    case TASK_READY: {
        // Unlink from the ready queue
        uint8_t prev = SCH_NIL;
        uint8_t cur  = ready_head;
        while (cur != SCH_NIL && cur != idx) {
            prev = cur;
            cur  = SCH_tasks_G[cur].Next;
        }
        if (cur == idx) {
            uint8_t next = SCH_tasks_G[idx].Next;
            if (prev == SCH_NIL) {
                ready_head = next;
            } else {
                SCH_tasks_G[prev].Next = next;
            }
            if (ready_tail == idx) {
                ready_tail = prev;
            }
        }
        free_slot(idx);
        break;
    }
    case TASK_RUNNING:
        // Task deleting itself: free it, the dispatcher will not re-insert it
        free_slot(idx);
        break;
    default: // TASK_FREE
        exit_critical(primask);
        return 0;
    }

    exit_critical(primask);
    return 1;
}
