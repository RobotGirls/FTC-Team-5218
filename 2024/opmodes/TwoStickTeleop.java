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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

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
    private static final int HANGING_FULLY_EXTENDED = 9856;
    private static final int HANGING_FULLY_RETRACTED = 0;

    private static final double CLAW_OPEN = 0.5;
    private static final double CLAW_CLOSE = 0.3;

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    private BNO055IMU imu;

    private Servo clawServo;

    private DcMotor hangingMotor;
    private DcMotor liftMotor;

    private DcMotor intakeMotor;
    private DcMotor transportMotor;
    private boolean currentlySlow = false;
    volatile boolean stopRequested = false;

    public final boolean isStopRequested() {
        return this.stopRequested || Thread.currentThread().isInterrupted();
    }

    private OneWheelDriveTask liftMotorTask;

    MecanumFieldCentricDriveScheme scheme;

    private Servo droneServoLeft;
    private Servo droneServoRight;
    private MechanumGearedDrivetrain drivetrain;

    final double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN = 0.02;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN = 0.015;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN = 0.01;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)ƒ
    final double MAX_AUTO_TURN = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    private DcMotor frontLeft;  //  Used to control the left front drive wheel
    private DcMotor frontRight;  //  Used to control the right front drive wheel
    private DcMotor backLeft;  //  Used to control the left back drive wheel
    private DcMotor backRight;  //  Used to control the right back drive wheel

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag


    @Override
    public void handleEvent(RobotEvent e) {
    }

    @Override
    public void init() {
        super.init();

        boolean targetFound = false;    // Set to true when an AprilTag target is detected
        double drive = 0;        // Desired forward power/speed (-1 to +1)
        double strafe = 0;        // Desired strafe power/speed (-1 to +1)
        double turn = 0;        // Desired turning power/speed (-1 to +1)


        //mechanisms
        hangingMotor = hardwareMap.get(DcMotor.class, "hangingMotor");
        clawServo = hardwareMap.servo.get("clawServo");

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        transportMotor = hardwareMap.get(DcMotor.class, "transportMotor");

        clawServo = hardwareMap.servo.get("clawServo");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");

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
        liftMotorTask = new OneWheelDriveTask(this, liftMotor, true);
        liftMotorTask.slowDown(false);


        drivetask = new TeleopDriveTask(this, scheme, frontLeft, frontRight, backLeft, backRight);

        initAprilTag();

        if (USE_WEBCAM)
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur


    }

    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);

        }
    }


    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower = x - y - yaw;
        double rightFrontPower = x + y + yaw;
        double leftBackPower = x + y - yaw;
        double rightBackPower = x - y + yaw;

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

    public void initIMU() {
        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);

    }

    public void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);
        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }


        boolean targetFound = false;    // Set to true when an AprilTag target is detected
        desiredTag = null;

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }
        if (targetFound) {
            telemetry.addData("\n>", "HOLD Left-Bumper to Drive to Target\n");
            telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
            telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);
        } else {
            telemetry.addData("\n>", "Drive using joysticks to find valid target\n");
        }
    }

        @Override
        public void start() {

            boolean targetFound = false;    // Set to true when an AprilTag target is detected
            desiredTag = null;
            //Gamepad 1
            this.addTask(drivetask);
            this.addTask(liftMotorTask);

           boolean finalTargetFound = targetFound;
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
                        case RIGHT_BUMPER_DOWN:
                            // If slow, then normal speed. If fast, then slow speed of motors.
                            //pertains to slowmode
                            double drive;
                            double turn;
                            double strafe;

                            if (finalTargetFound) {
                                double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                                double headingError = desiredTag.ftcPose.bearing;
                                double yawError = desiredTag.ftcPose.yaw;

                                // Use the speed and turn "gains" to calculate how we want the robot to move.
                                drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                                telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
                            } else {

                                // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
                                drive = -gamepad1.left_stick_y / 2.0;  // Reduce drive rate to 50%.
                                strafe = -gamepad1.left_stick_x / 2.0;  // Reduce strafe rate to 50%.
                                turn = -gamepad1.right_stick_x / 3.0;  // Reduce turn rate to 33%.
                                telemetry.addData("Manual", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
                            }
                            telemetry.update();
                            // Apply desired axes motions to the drivetrain.
                            moveRobot(drive, strafe, turn);
                            sleep(10);
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
                            hangingMotor.setTargetPosition(HANGING_FULLY_EXTENDED);
                            break;
                        case BUTTON_A_DOWN:
                            // set arm to retract to its lowest capacity to lift robot
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
