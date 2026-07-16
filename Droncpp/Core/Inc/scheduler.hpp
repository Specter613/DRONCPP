/*
 * scheluder.hpp
 *
 *  Created on: 16 jul 2026
 *      Author: specter0163
 */

#ifndef INC_SCHEDULER_HPP_
#define INC_SCHEDULER_HPP_

#include <cstdint>

class Scheduler
{
public:
    // El callback recibe el contexto (el objeto real) y cuántos ms
    // pasaron desde la ÚLTIMA vez que esta tarea específica corrió —
    // este delta es el que se usa para reportar el momento real de muestreo.
    using TaskCallback = void(*)(void *context, uint32_t deltaMs);

    static constexpr uint8_t kMaxTasks = 8;

    bool AddTask(TaskCallback callback, void *context, uint32_t periodMs);
    void Run();

private:
    struct Task
    {
        TaskCallback callback = nullptr;
        void *context = nullptr;
        uint32_t periodMs = 0;
        uint32_t lastRunTick = 0;
    };

    Task tasks_[kMaxTasks];
    uint8_t taskCount_ = 0;
};

#endif /* INC_SCHEDULER_HPP_ */
