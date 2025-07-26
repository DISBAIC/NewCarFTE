#include "Config/Config.hpp"
#include "Platform/Chassis/Mecanum/RS485Bus/RS485Bus.hpp"
#include "Program.hpp"
#include "Task.hpp"

#include "action.hpp"

#include <cmath>
#include <numbers>
#include <ext/concurrence.h>

using namespace Platform::Chassis;
using mv = Platform::Chassis::MoveDirection;

static Point points[4]{};
bool HasReceived = false;
static constexpr double angleToPixel = 25.6;
uint8_t buffer[2 * packetSize - 1];

void ServoReset() {
    yaw.SetCompare(yawStandard);
    pitch.SetCompare(pitchStandard);
}

namespace ActionConstValue {
    // 14 - 500
    // 39 - 800
    // ImageWidth from -14 to 39

    constexpr double rightBorderAngle = -14.0;
    constexpr double midAngle = 22.5;
    constexpr double leftBorderAngle = 29.0;

    constexpr double servoRange = leftBorderAngle - rightBorderAngle;

    constexpr double leftBorderLine = 27.22;
    constexpr double leftExternAngle = 81.21;
    constexpr double leftInterAngle = 61.26;

    constexpr double viewLineWidth = 21.77;

    enum {
        // N-Negative A-Angle P-Positive
        NA14 = 500,
        PA29 = 707
    };
} // namespace ActionConstValue

static double cosineTheorem(double _angle, double _line1, double _line2) {
    return std::sqrt(_line1 * _line1 + _line2 * _line2 - 2 * _line1 * _line2 * std::cos(_angle));
}

static bool getFire() {
    /*
     *    0x54,
     *    0x01,P1x/256,P1x%256,P1y/256,P1y%256,
     *    0x02,P2x/256,P2x%256,P2y/256,P2y%256,
     *    0x03,P3x/256,P3x%256,P3y/256,P3y%256,
     *    0x04,P4x/256,P4x%256,P4y/256,P4y%256,
     *    0x45
     */
    for (auto &i: buffer) {
        i = 0;
    }

    for (int i = 0; i < 50; i++) {
        vision.Receive(buffer, sizeof(buffer), 50);
        for (uint16_t j = 0; j < packetSize; j++) {
            if (buffer[j] == 0xff) {
                return false;
            }
            if (buffer[j] == 0x54 && buffer[j + packetSize - 1] == 0x45) {
                if (buffer[j + 1] != 0x01) {
                    continue;
                } else if (buffer[j + 6] != 0x02) {
                    continue;
                } else if (buffer[j + 11] != 0x03) {
                    continue;
                } else if (buffer[j + 16] != 0x04) {
                    continue;
                }
                points[0].x = (buffer[j + 2] << 8) | buffer[j + 3];
                points[0].y = (buffer[j + 4] << 8) | buffer[j + 5];
                points[1].x = (buffer[j + 7] << 8) | buffer[j + 8];
                points[1].y = (buffer[j + 9] << 8) | buffer[j + 10];
                points[2].x = (buffer[j + 12] << 8) | buffer[j + 13];
                points[2].y = (buffer[j + 14] << 8) | buffer[j + 15];
                points[3].x = (buffer[j + 17] << 8) | buffer[j + 18];
                points[3].y = (buffer[j + 19] << 8) | buffer[j + 20];

                return true;
            }
        }
    }

    return false;
}

static void aim() {
    using namespace ActionConstValue;

    pitch.SetCompare(pitchStandard);
    //

    int times = 0;
    bool pitchComplete = false;

    while (1) {
        auto x = getFire();
        if (!x) {
            continue;
        }
        auto bottom = points[3].y;
        auto center = points[0].x + points[1].x + points[2].x + points[3].x;
        center /= 4;

        if (!pitchComplete) {
            HAL_Delay(1000);
            if (center >= yawStart && center <= yawEnd) {
                if (times >= 2) {
                } else {
                    auto angle = (double) (yawStart - center) / (double) (yawEnd - yawStart);
                    auto target = yawServoStart - yawServoEnd;
                    target *= (int) angle;
                    if (center > 384) {
                        target += yawServoEnd;
                    } else {
                        target = yawServoStart - target;
                    }
                    yaw.SetCompare(target);
                    times++;
                }
            }
            if (bottom <= 350) {
                if (pitch.GetCompare() >= pitchHighest) {
                    pitch.SetCompare(pitchHighest);
                    pitchComplete = true;
                    continue;
                }
                pitch.SetCompare(pitch.GetCompare() + 10);
                continue;
            }
            if (bottom >= 380) {
                if (pitch.GetCompare() <= pitchLowest) {
                    pitch.SetCompare(pitch.GetCompare() - 10);
                    pitchComplete = true;
                    continue;
                }
                pitch.SetCompare(pitch.GetCompare() - 10);
                continue;
            }
            pitchComplete = true;
        } else {
            return;
        }
    }


    Delay(2000);

    //Task::JetWater(2000,50);

    Delay(2000);

    ServoReset();
}

void auxAim() {
    auto center = (points[0].x + points[1].x) / 2;
    if (center <= yawStart) {
        MCRSBPtr->RunTaskTime(mv::Rotate, -15.0);
    } else if (center >= yawEnd) {
        MCRSBPtr->RunTaskTime(mv::Rotate, 15.0);
    }
}

void AimFire() {
    auto test = getFire();
    if (test) {
        auxAim();
        aim();
        Task::JetWater(2000, 50);
    } else {
        while (!test) {
            MCRSBPtr->RunTaskTime(mv::Rotate, -15.0);
            test = getFire();
            if (test) {
                auxAim();
                break;
            }
            MCRSBPtr->RunTaskTime(mv::Rotate, -15.0);
            test = getFire();
            if (test) {
                auxAim();
                break;
            }

            MCRSBPtr->RunTaskTime(mv::Rotate, 45);
            test = getFire();
            if (test) {
                auxAim();
                break;
            }

            MCRSBPtr->RunTaskTime(mv::Rotate, 15.0);
            test = getFire();
            if (test) {
                auxAim();
                break;
            }

            return;
        }
        aim();
        Task::JetWater(2000, 50);
    }
}
