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
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
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

import team25core.RGBColorSensorTask;

import team25core.SingleShotColorSensorTask;

@TeleOp(name = "TwoStickTeleopWithColorSensor")
//@Disabled
public class TwoStickTeleopWithColorSensor extends StandardFourMotorRobot {


    private TeleopDriveTask drivetask;

    private enum Direction {
        CLOCKWISE,
        COUNTERCLOCKWISE,
    }
    //added field centric
    private Telemetry.Item buttonTlm;
    private Telemetry.Item colorDetectedTlm;
    private Telemetry.Item redDetectedTlm;
    private Telemetry.Item blueDetectedTlm;
    private Telemetry.Item greenDetectedTlm;
    private Telemetry.Item coneTlm;
    private static final double CONE_GRAB = 0.12;
    private static final double CONE_RELEASE = 1.00;

    private static final double ARM_FRONT = 0.8;
    private static final double ARM_BACK = 0;

    private static final double ALIGNER_FRONT = .6;
    private static final double ALIGNER_BACK = .2;

    //arm is 5, cone is 3
    private BNO055IMU imu;

    private DcMotor liftMotor;
    // private DcMotor intakeMotor;

    private Servo coneServo;
    private Servo junctionAligner;
    private Servo armServo;

    private boolean currentlySlow = false;

    private OneWheelDriveTask liftMotorTask;
    protected RGBColorSensorTask colorSensorTask;
    private ColorSensor colorSensor;
    MecanumFieldCentricDriveScheme scheme;

    private MechanumGearedDrivetrain drivetrain;

    private static final int TICKS_PER_INCH = 79;

    protected int[] colorArray = new int[3];

    @Override
    public void handleEvent(RobotEvent e) {
    }

    @Override
    public void init() {

        super.init();

        //mechanisms
        liftMotor = hardwareMap.get(DcMotor.class,"liftMotor");

        coneServo = hardwareMap.servo.get("coneServo");
        junctionAligner = hardwareMap.servo.get("junctionAligner");
        armServo = hardwareMap.servo.get("armServo");
        colorSensor = hardwareMap.colorSensor.get("colorSensor");

        // using encoders to record ticks
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        coneServo.setPosition(CONE_GRAB);
        junctionAligner.setPosition(.2);
        armServo.setPosition(0.8);

        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //telemetry
        buttonTlm = telemetry.addData("buttonState", "unknown");
        colorDetectedTlm = telemetry.addData("color detected", "unknown");

        blueDetectedTlm = telemetry.addData("blue", 0);
        redDetectedTlm = telemetry.addData("red", 0);
        greenDetectedTlm = telemetry.addData("green", 0);

        TwoStickMechanumControlScheme scheme = new TwoStickMechanumControlScheme(gamepad1);
        drivetrain = new MechanumGearedDrivetrain(motorMap);
        drivetrain.setNoncanonicalMotorDirection();
        // Note we are swapping the rights and lefts in the arguments below
        // since the gamesticks were switched for some reason and we need to do
        // more investigation
        drivetask = new TeleopDriveTask(this, scheme, frontLeft, frontRight, backLeft, backRight);

        liftMotorTask = new OneWheelDriveTask(this, liftMotor, true);
        liftMotorTask.slowDown(false);
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
        colorSensorTask = new RGBColorSensorTask(this, colorSensor) {
            public void handleEvent(RobotEvent e) {
                RGBColorSensorTask.ColorSensorEvent event = (RGBColorSensorTask.ColorSensorEvent) e;
                // sets threshold for blue, red, and green to ten thousand
                colorSensorTask.setThresholds(10000, 10000, 5000);
                colorArray = colorSensorTask.getColors();
                // shows the values of blue, red, green on the telemetry
                blueDetectedTlm.setValue(colorArray[0]);
                redDetectedTlm.setValue(colorArray[1]);
                greenDetectedTlm.setValue(colorArray[2]);
                switch(event.kind) {
                    // red is at the end
                    case RED_DETECTED:
                        //  colorSensorTask.suspend();
                        //  hLift.setPower(0.0);
                        //  state = LiftStates.DROPPING;
                        colorDetectedTlm.setValue("red");
                        break;
                    case BLUE_DETECTED:
                        //  hLift.setPower(0.0);
                        //  this.removeTask(colorSensorTask);
                        colorDetectedTlm.setValue("blue");
                        //  state = LiftStates.LOWERING;
                        break;
                    case GREEN_DETECTED:
                        //  hLift.setPower(0.0);
                        //  this.removeTask(colorSensorTask);
                        colorDetectedTlm.setValue("green");
                        //  state = LiftStates.LOWERING;
                        break;
                    default:
                        colorDetectedTlm.setValue("none");
                        break;
                }
            }
        };
        this.addTask(colorSensorTask);

        this.addTask(new GamepadTask(this, GamepadTask.GamepadNumber.GAMEPAD_1) {
            //@Override
            public void handleEvent(RobotEvent e) {
                GamepadEvent gamepadEvent = (GamepadEvent) e;

                switch (gamepadEvent.kind) {
                    case BUTTON_X_DOWN:
                        // If slow, then normal speed. If fast, then slow speed of motors.
                        //pertains to slowmode
                        if (currentlySlow) {
                            drivetask.slowDown(0.7);
                            currentlySlow = false;
                        } else {
                            drivetask.slowDown(0.3);
                            currentlySlow = true;
                        }
                        break;
                    default:
                        buttonTlm.setValue("Not Moving");
                        break;
                }
            }
        });

        //Gamepad 2
        this.addTask(liftMotorTask);

        this.addTask(new GamepadTask(this, GamepadTask.GamepadNumber.GAMEPAD_2) {
            //@Override
            public void handleEvent(RobotEvent e) {
                GamepadEvent gamepadEvent = (GamepadEvent) e;

                switch (gamepadEvent.kind) {

                    case BUTTON_X_DOWN:
                        //position 0
                        armServo.setPosition(ARM_FRONT);
                        break;
                    case BUTTON_B_DOWN:
                        //position 1
                        armServo.setPosition(ARM_BACK);
                        break;
                    case BUTTON_A_DOWN:
                        //position 1
                        coneServo.setPosition(CONE_GRAB);
                        break;
                    case BUTTON_Y_DOWN:
                        //position 0 (original pos)
                        coneServo.setPosition(CONE_RELEASE);
                        break;
                    case LEFT_TRIGGER_DOWN:
                        //position 1
                        junctionAligner.setPosition(ALIGNER_FRONT);
                        break;
                    case RIGHT_TRIGGER_DOWN:
                        //position 0 (original pos)
                        junctionAligner.setPosition(ALIGNER_BACK);
                        break;
                    case LEFT_BUMPER_DOWN:
                        // position 1
                        // raise lift motor to blue
                        break;

                    default:
                        buttonTlm.setValue("Not Moving");
                        break;
                }
            }
        });
    }
}
