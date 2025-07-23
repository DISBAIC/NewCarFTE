
#include "Config/Config.hpp"
#include "Platform/Chassis/Mecanum/RS485Bus/RS485Bus.hpp"
#include "Program.hpp"
#include "Task.hpp"

#include <cmath>

using namespace Platform::Chassis;
using mv = Platform::Chassis::MoveDirection;

static Point points[4]{};
static constexpr double angleToPixel = 25.6;
uint8_t buffer[2 * packetSize - 1];

void ServoReset() {
    yaw.SetCompare(yawStandard);
    pitch.SetCompare(pitchStandard);
}

namespace ActionConstValue
{

    // 14 - 500
    // 39 - 800
    // ImageWidth from -14 to 39

    constexpr double rightBorderAngle = -14.0;
    constexpr double midAngle         = 22.5;
    constexpr double leftBorderAngle  = 29.0;

    constexpr double servoRange = leftBorderAngle - rightBorderAngle;

    constexpr double leftBorderLine  = 27.22;
    constexpr double leftExternAngle = 81.21;
    constexpr double leftInterAngle  = 61.26;

    constexpr double viewLineWidth = 21.77;

    enum {
        // N-Negative A-Angle P-Positive
        NA14 = 500,
        PA29 = 707

    };

} // namespace ActionConstValue

static double cosineTheorem(double _angle, double _line1, double _line2)
{
    return std::sqrt(_line1 * _line1 + _line2 * _line2 - 2 * _line1 * _line2 * std::cos(_angle));
}

static bool getFire()
{

    /*
     *    0x54,
     *    0x01,P1x/256,P1x%256,P1y/256,P1y%256,
     *    0x02,P2x/256,P2x%256,P2y/256,P2y%256,
     *    0x03,P3x/256,P3x%256,P3y/256,P3y%256,
     *    0x04,P4x/256,P4x%256,P4y/256,P4y%256,
     *    0x45
     */

    for (int i = 0; i < 200; i++) {
        vision.Receive(buffer, sizeof(buffer), 100);
        for (uint16_t i = 0; i < packetSize; i++) {
            if (buffer[i] == 0xff) {
                return false;
            }
            if (buffer[i] == 0x54 && buffer[i + packetSize - 1] == 0x45) {
                if (buffer[i + 1] != 0x01) {
                    continue;
                } else if (buffer[i + 6] != 0x02) {
                    continue;
                } else if (buffer[i + 11] != 0x03) {
                    continue;
                } else if (buffer[i + 16] != 0x04) {
                    continue;
                }
                points[0].x = (buffer[i + 2] << 8) | buffer[i + 3];
                points[0].y = (buffer[i + 4] << 8) | buffer[i + 5];
                points[1].x = (buffer[i + 7] << 8) | buffer[i + 8];
                points[1].y = (buffer[i + 9] << 8) | buffer[i + 10];
                points[2].x = (buffer[i + 12] << 8) | buffer[i + 13];
                points[2].y = (buffer[i + 14] << 8) | buffer[i + 15];
                points[3].x = (buffer[i + 17] << 8) | buffer[i + 18];
                points[3].y = (buffer[i + 19] << 8) | buffer[i + 20];

                return true;
            }
        }
    }

    return false;
}

static void aim()
{
    using namespace ActionConstValue;
//
    const auto center = (points[0].x + points[1].x + points[2].x + points[3].x) / 4.0;
//
    const auto line1 = center / imageWidth ;
//
    //const auto lineCast = cosineTheorem(leftExternAngle, line1, 27.22);
//
    //const auto angleView = line1 * std::sin(leftExternAngle) / lineCast;
//
    //const auto targetAngle = leftBorderAngle - angleView;



    const auto servoValue =  PA29 - line1 * (PA29 - NA14);

    const auto fireHeight = points[0].y - points[3].y;

    const auto firePosition = fireHeight * 0.12;    

    const auto screenPosition = points[3].y + firePosition;

    const auto h1 = screenPosition - plS;
    auto percent = h1/(float)(plH - plS);

    if (percent < 0.0f) {
        percent = 0.0f;
    }
    if (percent > 1.0f) {
        percent = 1.0f;
    }

    yaw.SetCompare(int(servoValue));

    pitch.SetCompare(int(pitchLower + (pitchHighest - pitchLower) * percent) * 0.9);

    Delay(2000);

    Task::JetWater(2000,50);

    Delay(2000);

    ServoReset();
}

void AimFire()
{
    auto test = getFire();
    if (test) {
        aim();
    } else {
        while (!test) {
            MCRSBPtr->RunTaskTime(mv::Rotate, -15.0);
            test = getFire();
            if (test) {
                break;
            }
            MCRSBPtr->RunTaskTime(mv::Rotate, -15.0);
            test = getFire();
            if (test) {
                break;
            }

            MCRSBPtr->RunTaskTime(mv::Rotate, 45);
            test = getFire();
            if (test) {
                break;
            }

            MCRSBPtr->RunTaskTime(mv::Rotate, 15.0);
            test = getFire();
            if (test) {
                break;
            }

            return;
        }
        aim();
    }
}
