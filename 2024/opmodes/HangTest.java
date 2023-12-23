/*
 * Copyright (c) September 2017 FTC Teams 25/5218
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without modification,
 *  are permitted (subject to the limitations in the disclaimer below) provided that
 *  the following conditions are met:
 *
 *  Redistributions of source code must retain the above copyright notice, this list
 *  of conditions and the following disclaimer.
 *
 *  Redistributions in binary form must reproduce the above copyright notice, this
 *  list of conditions and the following disclaimer in the documentation and/or
 *  other materials provided with the distribution.
 *
 *  Neither the name of FTC Teams 25/5218 nor the names of their contributors may be used to
 *  endorse or promote products derived from this software without specific prior
 *  written permission.
 *
 *  NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 *  LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 *  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 *  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package opmodes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import team25core.GamepadTask;
import team25core.MechanumGearedDrivetrain;
import team25core.OneWheelDriveTask;
import team25core.RobotEvent;
import team25core.StandardFourMotorRobot;
import team25core.TwoStickMechanumControlScheme;
import team25core.TeleopDriveTask;


@TeleOp(name = "HangTest")
//@Disabled
public class HangTest extends StandardFourMotorRobot {

    private TeleopDriveTask drivetask;

    private enum Direction {
        CLOCKWISE,
        COUNTERCLOCKWISE,
    }
    //added field centric
    private Telemetry.Item buttonTlm;

    private static final double DRONE_SET_LEFT = 0.95;
    private static final double DRONE_SET_RIGHT = 0;
    private static final double DRONE_RELEASE = 0.5;
    private static final int HANGING_FULLY_EXTENDED_RIGHT= 9856;
    private static final int HANGING_FULLY_RETRACTED_RIGHT = 5;
    private static final int HANGING_FULLY_EXTENDED_LEFT = 7156;
    private static final int HANGING_FULLY_RETRACTED_LEFT = 5;
    private static final double CLAW_OPEN = 0.5;
    private static final double CLAW_CLOSE = 0.2;

    private Telemetry.Item leftEncoderTlm;
    private Telemetry.Item rightEncoderTlm;
    private BNO055IMU imu;

    private Servo clawServo;

    private DcMotor hangingMotorLeft;
    private DcMotor hangingMotorRight;

    private DcMotor liftMotor;

    //    private DcMotor intakeMotor;
    private DcMotor transportMotor;
    private boolean currentlySlow = false;
    private OneWheelDriveTask hangingMotorRighttask;
    private OneWheelDriveTask hangingMotorLeftask;

    MecanumFieldCentricDriveScheme scheme;

    private Servo droneServoLeft;
    private Servo droneServoRight;
    private MechanumGearedDrivetrain drivetrain;

    @Override
    public void handleEvent(RobotEvent e) {
    }

    @Override
    public void init() {
        super.init();

        //mechanisms
        hangingMotorLeft = hardwareMap.get(DcMotor.class,"hangingMotorLeft");
        hangingMotorRight = hardwareMap.get(DcMotor.class,"hangingMotorRight");

        // the motor must be at its set position zero, at the beginning of the opmode
        hangingMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // hangingMotorLeft.setTargetPosition(0);
        // encoder allows you to know how much the motor has spun (distance)
        hangingMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // the brake allows the motor to hold its position when power is not currently being applied
        hangingMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangingMotorLeft.setPower(0.75);

        hangingMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // hangingMotorRight.setTargetPosition(0);
        // encoder allows you to know how much the motor has spun (distance)
        hangingMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // the brake allows the motor to hold its position when power is not currently being applied
        hangingMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangingMotorRight.setPower(0.75);

        hangingMotorLeftask = new OneWheelDriveTask(this, hangingMotorLeft, false);
        hangingMotorRighttask = new OneWheelDriveTask(this, hangingMotorRight, true);

        leftEncoderTlm = telemetry.addData("Left Encoder: ", 0.0);
        rightEncoderTlm = telemetry.addData("Right Encoder: ", 0.0);

    }

    public void initIMU()
    {
        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);

    }

    @Override
    public void start() {

        //Gamepad 1
        this.addTask(hangingMotorRighttask);
        this.addTask(hangingMotorLeftask);

        this.addTask(new GamepadTask(this, GamepadTask.GamepadNumber.GAMEPAD_1) {
            public void handleEvent(RobotEvent e) {
                GamepadEvent gamepadEvent = (GamepadEvent) e;

                switch (gamepadEvent.kind) {
                    case BUTTON_X_DOWN:
                        // If slow, then normal speed. If fast, then slow speed of motors.
                        //pertains to slowmode
                        if (currentlySlow) {
                            drivetask.slowDown(1.0);
                            currentlySlow = false;
                        } else {
                            drivetask.slowDown(0.3);
                            currentlySlow = true;
                        }
                        break;
                    default:
                        // buttonTlm.setValue("Not Moving");
                        break;
                }
            }
        });

        //Gamepad 2

        this.addTask(new GamepadTask(this, GamepadTask.GamepadNumber.GAMEPAD_2) {
            public void handleEvent(RobotEvent e) {
                GamepadEvent gamepadEvent = (GamepadEvent) e;

                switch (gamepadEvent.kind) {
                    case BUTTON_X_DOWN:
                        leftEncoderTlm.setValue(hangingMotorLeft.getCurrentPosition());
                        rightEncoderTlm.setValue(hangingMotorRight.getCurrentPosition());
                        break;

                    default:
                        buttonTlm.setValue("Not Moving");
                        break;
                }
            }
        });
    }
}
