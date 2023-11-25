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
import team25core.ObjectDetectionNewTask;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import team25core.GamepadTask;
import team25core.MechanumGearedDrivetrain;
import team25core.OneWheelDriveTask;
import team25core.RobotEvent;
import team25core.StandardFourMotorRobot;
import team25core.TwoStickMechanumControlScheme;
import team25core.TeleopDriveTask;
import team25core.vision.apriltags.AprilTagDetectionTask;
import com.qualcomm.robotcore.util.Range;



@TeleOp(name = "TeleopAprilTags")
//@Disabled
public class TeleopAprilTags extends StandardFourMotorRobot {

    private TeleopDriveTask drivetask;

    private enum Direction {
        CLOCKWISE,
        COUNTERCLOCKWISE,
    }
        private ObjectDetectionNewTask objDetectionTask;
        private final static String TAG = "Prop";
        final double DESIRED_DISTANCE = 2.0; //  this is how close the camera should get to the target (inches)
        //  Set the GAIN constants to control the relationship between the measured position error,
        //  and how much power is applied to the drive motors to correct the error.
        //  Drive = Error * Gain    Make these values smaller for smoother control, or larger
        //  for a more aggressive response.
        final double SPEED_GAIN = 0.02;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
        final double STRAFE_GAIN = 0.015;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
        final double TURN_GAIN = 0.01;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

        final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
        final double MAX_AUTO_STRAFE = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
        final double MAX_AUTO_TURN = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

        private DcMotor frontLeft=null;
        private DcMotor frontRight=null;
        private DcMotor backLeft=null;
        private DcMotor backRight=null;


    private int desiredTagID;     // Choose the tag you want to approach or set to -1 for ANY tag.

        private final float APRIL_TAG_DECIMATION = 2;

        private final int EXPOSURE_MS = 6;
        private final int GAIN = 250;

        public String position; // this will contain the actual prop position information in final auto

        public AprilTagDetection aprilTag;
        boolean targetFound = false;
        boolean targetReached = false;
    private Telemetry.Item buttonTlm;

    private static final double DRONE_SET_LEFT = 0.95;
    private static final double DRONE_SET_RIGHT = 0;
    private static final double DRONE_RELEASE = 0.5;
    private static final int HANGING_FULLY_EXTENDED = 9856;
    private static final int HANGING_FULLY_RETRACTED = 0;

    private static final double CLAW_OPEN = 0.5;
    private static final double CLAW_CLOSE = 0.3;

    private BNO055IMU imu;

    private Servo clawServo;

    private DcMotor hangingMotor;
    private DcMotor liftMotor;

    private DcMotor intakeMotor;
    private DcMotor transportMotor;
    private boolean currentlySlow = false;

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
        hangingMotor = hardwareMap.get(DcMotor.class,"hangingMotor");

        intakeMotor =  hardwareMap.get(DcMotor.class,"intakeMotor");
        transportMotor  =  hardwareMap.get(DcMotor.class,"transportMotor");

        clawServo = hardwareMap.servo.get("clawServo");
        liftMotor = hardwareMap.get(DcMotor.class,"liftMotor");

        droneServoLeft = hardwareMap.servo.get("droneServoLeft");
        droneServoRight = hardwareMap.servo.get("droneServoRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");

        // using encoders to record ticks
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        droneServoLeft.setPosition(DRONE_SET_LEFT);
        droneServoRight.setPosition(DRONE_SET_RIGHT);

        // the motor must be at its set position zero, at the beginning of the opmode
        hangingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangingMotor.setTargetPosition(0);
        // encoder allows you to know how much the motor has spun (distance)
        hangingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // the brake allows the motor to hold its position when power is not currently being applied
        hangingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangingMotor.setPower(0.75);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        drivetask = new TeleopDriveTask(this, scheme, frontLeft, frontRight, backLeft, backRight);
    }

    public void findAprilTag() {
        objDetectionTask = new ObjectDetectionNewTask(this, telemetry, ObjectDetectionNewTask.DetectionKind.APRILTAG_DETECTED) {
            @Override
            public void handleEvent(RobotEvent e) {
                AprilTagDetectionTask.TagDetectionEvent event = (AprilTagDetectionTask.TagDetectionEvent) e;
                switch (event.kind) {
                    case OBJECTS_DETECTED:
                        break;
                }
            }
        };
        objDetectionTask.init(telemetry, hardwareMap);
        objDetectionTask.rateLimit(1000); // currently calling objDetectionTask every second
        objDetectionTask.start();
        objDetectionTask.resumeStreaming();
        objDetectionTask.setAprilTagDecimation(APRIL_TAG_DECIMATION);
        objDetectionTask.doManualExposure(EXPOSURE_MS, GAIN); // Use low exposure time to reduce motion blur
        objDetectionTask.setDesiredTagID(desiredTagID);
        addTask(objDetectionTask);
    }



    public void AlignWithAprilTag(AprilTagDetection tag) {
        double drive = 0;
        double strafe = 0;
        double turn = 0;
        if (objDetectionTask.getAprilTag(desiredTagID) != null) {

            double rangeError = (tag.ftcPose.range - DESIRED_DISTANCE);
            double headingError = tag.ftcPose.bearing;
            double yawError = tag.ftcPose.yaw;

            drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            if (rangeError < 0.05 && headingError < 0.05 && yawError < 0.05) {
                targetReached = true;
            }
        }
        telemetry.update();


        // Apply desired axes motions to the drivetrain.
        moveRobot(drive, strafe, turn);
        sleep(10);
    }

    public AprilTagDetection findAprilTagData() {
        if (desiredTagID == 1) {
            while (objDetectionTask.getAprilTag(desiredTagID) == null) {
                frontLeft.setPower(0.3);
                frontRight.setPower(-0.3);
                backLeft.setPower(-0.3);
                backRight.setPower(0.3);
            }
            targetFound = true;
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        } else if (desiredTagID == 3) {
            while (objDetectionTask.getAprilTag(desiredTagID) == null) {
                frontLeft.setPower(-0.3);
                frontRight.setPower(0.3);
                backLeft.setPower(0.3);
                backRight.setPower(-0.3);
            }
            targetFound = true;
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }
        return objDetectionTask.getAprilTag(desiredTagID);
    }

    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        frontLeft.setPower(leftFrontPower);
        frontRight.setPower(rightFrontPower);
        backLeft.setPower(leftBackPower);
        backRight.setPower(rightBackPower);

    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
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
                    case BUTTON_Y_DOWN:
                        desiredTagID = 1;
                        findAprilTag();
                        aprilTag = findAprilTagData();
                        AlignWithAprilTag(aprilTag);
                        while (!targetReached && gamepad1.y) {
                            AlignWithAprilTag(aprilTag);
                        }
                        frontLeft.setPower(0);
                        frontRight.setPower(0);
                        backLeft.setPower(0);
                        backRight.setPower(0);
                break;
                    case BUTTON_B_DOWN:
                        desiredTagID = 2;
                        findAprilTag();
                        aprilTag = findAprilTagData();
                        AlignWithAprilTag(aprilTag);
                        while (!targetReached) {
                            AlignWithAprilTag(aprilTag);
                        }
                        frontLeft.setPower(0);
                        frontRight.setPower(0);
                        backLeft.setPower(0);
                        backRight.setPower(0);
                        break;
                    case BUTTON_A_DOWN:
                        desiredTagID = 3;
                        findAprilTag();
                        aprilTag = findAprilTagData();
                        AlignWithAprilTag(aprilTag);
                        while (!targetReached) {
                            AlignWithAprilTag(aprilTag);
                        }
                        frontLeft.setPower(0);
                        frontRight.setPower(0);
                        backLeft.setPower(0);
                        backRight.setPower(0);
                        break;
                    default:
                        buttonTlm.setValue("Not Moving");
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
                        //position 0
                        droneServoLeft.setPosition(DRONE_RELEASE);
                        droneServoRight.setPosition(DRONE_RELEASE);
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
                        hangingMotor.setTargetPosition(HANGING_FULLY_EXTENDED);
                        break;
                    case BUTTON_A_DOWN:
                        // set arm to extend to its highest capacity to lift robot
                        hangingMotor.setTargetPosition(HANGING_FULLY_RETRACTED);
                        break;

                    case BUTTON_X_DOWN:
                        intakeMotor.setPower(-1);
                        transportMotor.setPower(1);
                        // intake pixels into robot
                        break;
                    case LEFT_BUMPER_DOWN:
                        intakeMotor.setPower(0);
                        transportMotor.setPower(0);
                        // stops pixel motor
                        break;

                    case BUTTON_B_DOWN:
                        intakeMotor.setPower(1);
                        transportMotor.setPower(-1);
                        // outtakes pixels out of robot
                        break;
                    default:
                        buttonTlm.setValue("Not Moving");
                        break;
                }
            }
        });
    }
}