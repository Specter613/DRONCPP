/*
 * sheluder.cpp
 *
 *  Created on: 16 jul 2026
 *      Author: specter0163
 */

#include "scheduler.hpp"
#include "stm32f4xx_hal.h"

bool Scheduler::AddTask(TaskCallback callback, void *context, uint32_t periodMs)
{
    if(taskCount_ >= kMaxTasks) return false;

    tasks_[taskCount_].callback = callback;
    tasks_[taskCount_].context = context;
    tasks_[taskCount_].periodMs = periodMs;
    tasks_[taskCount_].lastRunTick = HAL_GetTick();
    taskCount_++;
    return true;
}

void Scheduler::Run()
{
    uint32_t now = HAL_GetTick();

    for(uint8_t i = 0; i < taskCount_; i++)
    {
        Task &t = tasks_[i];
        uint32_t elapsed = now - t.lastRunTick;

        if(elapsed >= t.periodMs)
        {
            t.lastRunTick = now;
            t.callback(t.context, elapsed);
        }
    }
}


