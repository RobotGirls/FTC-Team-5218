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


@TeleOp(name = "TwoStickTeleop")
//@Disabled
public class TwoStickTeleop extends StandardFourMotorRobot {

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
    private static final int HANGING_FULLY_EXTENDED_RIGHT= 6800;
    private static final int HANGING_FULLY_RETRACTED_RIGHT = 5;
    private static final int HANGING_FULLY_EXTENDED_LEFT = 6900;
    private static final int HANGING_FULLY_RETRACTED_LEFT = 5;
    private static final double CLAW_CLOSE = 0.5;
    private static final double CLAW_OPEN = 0.2;

    private BNO055IMU imu;

    private Servo clawServo;

    private DcMotor hangingMotorLeft;
    private DcMotor hangingMotorRight;

    private DcMotor liftMotor;

    //    private DcMotor intakeMotor;
    private DcMotor transportMotor;
    private boolean currentlySlow = false;
    private OneWheelDriveTask liftMotorTask;
    private OneWheelDriveTask transporIntakeMotorTask;


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

        clawServo = hardwareMap.servo.get("clawServo");

//        intakeMotor =  hardwareMap.get(DcMotor.class,"intakeMotor");
        transportMotor  =  hardwareMap.get(DcMotor.class,"transportMotor");

        clawServo = hardwareMap.servo.get("clawServo");
        liftMotor = hardwareMap.get(DcMotor.class,"liftMotor");

        droneServoLeft = hardwareMap.servo.get("droneServoLeft");
        droneServoRight = hardwareMap.servo.get("droneServoRight");

        // using encoders to record ticks
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        droneServoLeft.setPosition(DRONE_SET_LEFT);
        droneServoRight.setPosition(DRONE_SET_RIGHT);

        // the motor must be at its set position zero, at the beginning of the opmode
        hangingMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangingMotorLeft.setTargetPosition(0);
        // encoder allows you to know how much the motor has spun (distance)
        hangingMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // the brake allows the motor to hold its position when power is not currently being applied
        hangingMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangingMotorLeft.setPower(0.75);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hangingMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangingMotorRight.setTargetPosition(0);
        // encoder allows you to know how much the motor has spun (distance)
        hangingMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // the brake allows the motor to hold its position when power is not currently being applied
        hangingMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangingMotorRight.setPower(0.75);

//        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        transportMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transportMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        clawServo.setPosition(CLAW_CLOSE);

        //telemetry
        buttonTlm = telemetry.addData("buttonState", "unknown");

        TwoStickMechanumControlScheme scheme = new TwoStickMechanumControlScheme(gamepad1);
        drivetrain = new MechanumGearedDrivetrain(motorMap);
        drivetrain.setNoncanonicalMotorDirection();
        // Note we are swapping the rights and lefts in the arguments below
        // since the gamesticks were switched for some reason and we need to do
        // more investigation
        liftMotorTask = new OneWheelDriveTask(this, liftMotor, true);
        liftMotorTask.slowDown(false);

        transporIntakeMotorTask = new OneWheelDriveTask(this, transportMotor, false);
        transporIntakeMotorTask.slowDown(false);

        drivetask = new TeleopDriveTask(this, scheme, frontLeft, frontRight, backLeft, backRight);
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
        this.addTask(drivetask);
        this.addTask(liftMotorTask);
        this.addTask(transporIntakeMotorTask);


        this.addTask(new GamepadTask(this, GamepadTask.GamepadNumber.GAMEPAD_1) {
            public void handleEvent(RobotEvent e) {
                GamepadEvent gamepadEvent = (GamepadEvent) e;

                switch (gamepadEvent.kind) {
                    case RIGHT_BUMPER_DOWN:
                        // If slow, then normal speed. If fast, then slow speed of motors.
                        //pertains to slowmode


                            drivetask.slowDown(1);
                            currentlySlow = false;


                        break;
                    case RIGHT_BUMPER_UP:

                            drivetask.slowDown(0.35);
                            currentlySlow = true;

                    break;
                    case BUTTON_X_DOWN:
                        // If slow, then normal speed. If fast, then slow speed of motors.
                        //pertains to slowmode
                        if (currentlySlow) {
                            drivetask.slowDown(0.5);
                            currentlySlow = false;
                        } else {
                            drivetask.slowDown(0.1);
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
                    case RIGHT_BUMPER_DOWN:
                        droneServoLeft.setPosition(DRONE_RELEASE);
                        droneServoRight.setPosition(DRONE_RELEASE);
                        break;
                    case LEFT_TRIGGER_DOWN:
                        // set claw's position to 0
                        clawServo.setPosition(CLAW_CLOSE);
                        break;
                    case RIGHT_TRIGGER_DOWN:
                        // set claw's position to 1
                        clawServo.setPosition(CLAW_OPEN);
                        break;
                    case BUTTON_Y_DOWN:
                        // set arm to extend to its highest capacity to lift robot
                        hangingMotorRight.setTargetPosition(HANGING_FULLY_EXTENDED_RIGHT);
                        hangingMotorLeft.setTargetPosition(HANGING_FULLY_EXTENDED_LEFT);

                        break;
                    case BUTTON_A_DOWN:
                        // set arm to retract to its lowest capacity to lift robot
                        hangingMotorRight.setTargetPosition(HANGING_FULLY_RETRACTED_RIGHT);
                        hangingMotorLeft.setTargetPosition(HANGING_FULLY_RETRACTED_LEFT);
                        break;

//                    case BUTTON_X_DOWN:
//                        transportMotor.setPower(1);
//                        // intake pixels into robot
//                        break;
//                    case BUTTON_X_UP:
////
//                        transportMotor.setPower(0);
////                    // outtakes pixels out of robot
//                        break;
//                    case BUTTON_B_DOWN:
//                        transportMotor.setPower(-1);
//                        // outtakes pixels out of robot
//                        break;
//                    case BUTTON_B_UP:
//                        transportMotor.setPower(0);
//                        // outtakes pixels out of robot
//                        break;
                    default:
                        buttonTlm.setValue("Not Moving");
                        break;
                }
            }
        });
    }
}
