
#include "Platform/Chassis/ChassisU.hpp"

namespace Task
{
    enum TaskType {
        Move      = 0,
        Rotate    = 1,
        NegRotate = 2,
        Inject    = 3,
    };

    struct MoveTask {
        Platform::Chassis::MoveDirection direction;
        double distance;
    };

    struct Task {
        TaskType type;
        union Content {
            MoveTask move;
            uint16_t injectDuration; // 喷水任务的持续时间
        } content;
    };

    bool GetIsMoving();

    void ReceiveBuffer();

    extern Task tasks[32];

    extern unsigned int taskSize;

    void JetWater(int _duration);

    void TaskClear();

}