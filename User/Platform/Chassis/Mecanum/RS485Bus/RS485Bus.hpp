//
// Created by Administrator on 25-3-3.
//

#ifndef RS485BUS_HPP
#define RS485BUS_HPP

#include "Modules/StepMotor/Connect/Serial/Serial.hpp"
#include "Platform/Chassis/MoveTasks.hpp"
#include "Platform/Chassis/Mecanum/MecanumU.hpp"
#include "Peripheral/Uart.hpp"
#include "Modules/RS485/rs485.hpp"

#include <type_traits>

namespace Platform::Chassis {
    extern MecanumChassis<RS485Bus> * MCRSBPtr;

    template<>
    class MecanumChassis<RS485Bus> {
    public:
        using CPin = Modules::RS485::CPin;

        static void Create(const uint8_t *_addrs, const Peripheral::Uart<Peripheral::Interrupt> *_bus,CPin _pin, const uint8_t _cent,
                           const bool *_dirs, const bool _is18Angle, const double _radius,const double _motorDsitance) {
            static MecanumChassis instance(_bus,_pin, _addrs, _dirs, _cent, _is18Angle, _radius,_motorDsitance);
            MCRSBPtr = &instance;
        }


        static void SetRotateFixFactor(double _factor);

        template<typename Func, typename... Args>
            requires std::is_invocable_r_v<bool, Func>
        bool RunTask(const MoveDirection _dir, const double _distance, const uint32_t _checkCount, Func func,
                     Args... args) {
            moveAction(_dir, _distance);
            if (WaitForStop(_checkCount)) {
                return func(args...);
            }
            return false;
        }

        bool  RunTask(const MoveDirection _dir, const double _distance, const uint16_t _checkCount) const {
            moveAction(_dir, _distance);
            return WaitForStop(_checkCount);
        }

        void RunTask(const MoveDirection _dir, const double _distance) const {
            moveAction(_dir, _distance);
        }

        [[nodiscard]] bool WaitForStop(uint16_t _checkCount) const;

    private:
        const CPin flowControlPin;

        void SendEnable() const {
            flowControlPin.Write(true);
        }

        void ReceiveEnable() const  {
            flowControlPin.Write(false);
        }

        const Modules::StepMotor<Modules::Serial> lfMotor;
        const Modules::StepMotor<Modules::Serial> rfMotor;
        const Modules::StepMotor<Modules::Serial> lbMotor;
        const Modules::StepMotor<Modules::Serial> rbMotor;

        const Peripheral::Uart<Peripheral::Interrupt> *bus;

        bool is18Angle;

        const double radius;

        const double motorDistance;

        struct MoveInfos {
            uint8_t acceleration;
            uint16_t velocity;
            uint32_t distance;
        };

        explicit MecanumChassis(const Peripheral::Uart<Peripheral::Interrupt> *_bus,CPin _pin, const uint8_t *_addrs,
                                const bool *_dirs, const uint8_t _cent, const bool _is18Angle,
                                const double _radius,const double _motorDistance) : flowControlPin(_pin), lfMotor(_bus, _addrs[0]),
                                                       rfMotor(_bus, _addrs[1]), lbMotor(_bus, _addrs[2]),rbMotor(_bus, _addrs[3]),
                                                       bus(_bus), is18Angle(_is18Angle), radius(_radius),motorDistance(_motorDistance) {
            setDirection(_dirs);
            setCent(_cent);
            targetPluses = 0;
        }

        static void setDirection(const bool *_dirs);

        void enableAll() const;

        void moveForWard(MoveInfos _infos) const;

        void moveBackWard(MoveInfos _infos) const;

        void turnRight(MoveInfos _infos) const;

        void turnLeft(MoveInfos _infos) const;

        void transLeft(MoveInfos _infos) const;

        void transRight(MoveInfos _infos) const;

        void transRightForward(MoveInfos _infos) const;

        void transLeftForward(MoveInfos _infos) const;

        void transRightBackward(MoveInfos _infos) const;

        void transLeftBackward(MoveInfos _infos) const;

        void setCent(uint8_t _cent) const;

        void moveAction(MoveDirection _direction, double _distance) const;

        void allClear() const;

        [[nodiscard]] MoveInfos getMoveInfos(double _distance, double _factor) const;

        mutable uint32_t targetPluses;
    };
}


#endif //RS485Bus_HPP
