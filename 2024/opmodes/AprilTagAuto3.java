package opmodes;//package opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import team25core.DeadReckonTask;
import team25core.FourWheelDirectDrivetrain;
import team25core.GamepadTask;
import team25core.ObjectDetectionNewTask;
import team25core.Robot;
import team25core.RobotEvent;

@Autonomous(name = "AprilTagAuto3")
public class AprilTagAuto3 extends Robot {
    private ObjectDetectionNewTask objDetectionTask;
    private final static String TAG = "CODA";
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

    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private FourWheelDirectDrivetrain drivetrain;

    private final int DEFAULT_TAG_ID = 3;
    private int desiredTagID = DEFAULT_TAG_ID;     // Choose the tag you want to approach or set to -1 for ANY tag.

    private final float APRIL_TAG_DECIMATION = 3;

    private final int EXPOSURE_MS = 6;
    private final int GAIN = 250;

    //public String tagPositionOnProp; // this will contain the actual prop position information in final auto

    public AprilTagDetection aprilTag;
    boolean targetFound = false;
    boolean targetReached = false;

    private AprilTagDetection foundAprilTag;
    private int foundAprilTagId;

    //------- Telemetry ---------------------
    private Telemetry.Item whereAmI;
    private Telemetry.Item timesCalled;
    private int numCalls = 0;

    //------- Constants used for Driving ---
    private final double MOTOR_SPEED = 0.25;
    private final int LEFT = -1;
    private final int RIGHT = 1;
    private final int BACKWARD = -1;
    private final int FORWARD = 1;

    //------- Gamepad Variables-------------
    private GamepadTask gamepad;

    protected enum AllianceColor {
        BLUE,
        RED,
        DEFAULT,
    }

    private enum TagPositionOnProp {
        LEFT,
        RIGHT,
        MIDDLE
    }
    private enum DriveDirection {
        FORWARD,
        BACKWARD,
        LEFT,
        RIGHT
    }

    // Declare constants and autonomous variables.
    private AllianceColor allianceColor;
    private Telemetry.Item allianceColorTlm;
    private Telemetry.Item tagChoiceTlm;
    private Telemetry.Item desiredIdTlm;
    private TagPositionOnProp tagPosition = TagPositionOnProp.RIGHT;
    private boolean coneAprilTags = true;
    private DriveDirection driveDirection = DriveDirection.LEFT;
    private Telemetry.Item driveDirectionTlm;

    //--------------------------------------

    @Override
    public void handleEvent(RobotEvent e) {
        /*
         * Every time we complete a segment drop a note in the robot log.
         */
        //if (e instanceof DeadReckonTask.DeadReckonEvent) {
        //  RobotLog.i("Completed path segment %d", ((DeadReckonTask.DeadReckonEvent)e).segment_num);
        //}
        if (e instanceof GamepadTask.GamepadEvent) {
            GamepadTask.GamepadEvent event = (GamepadTask.GamepadEvent) e;
            handleGamepadSelection(event);
        }
    }

    public void findAprilTag() {
        whereAmI.setValue("findAprilTag", "beginning");

        RobotLog.ii(TAG, "Setup findAprilTag");
        objDetectionTask = new ObjectDetectionNewTask(this, telemetry, ObjectDetectionNewTask.DetectionKind.APRILTAG_DETECTED) {
            @Override
            public void handleEvent(RobotEvent e) {
                whereAmI.setValue("findAprilTag", "handleEvent");

                TagDetectionEvent event = (TagDetectionEvent) e;
                switch (event.kind) {
                    case APRIL_TAG_DETECTED:
                        RobotLog.ii(TAG, "AprilTag detected");
                        // AprilTag information (x, y, z, pose numbers)
                        foundAprilTag = event.aprilTag;
                        // AprilTag ID
                        foundAprilTagId = foundAprilTag.id;
                        numCalls += 1;
                        whereAmI.setValue("handleEvent", "APRIL_TAG_DETECTED");
                        timesCalled.setValue(numCalls);

                        break;
                }
            }
        };
        // initializes the aprilTag processor
        objDetectionTask.init(telemetry, hardwareMap);
        // FIXME make sure this is the rate we want to use
        objDetectionTask.rateLimit(250); // currently calling objDetectionTask every 1/4 second
        objDetectionTask.start(); // instantiates the rate limiting timer
        // start acquiring and processing images. Note this
        // uses a lot of computational resources, so make sure
        // you stop streaming if you aren't using the AprilTag
        // detection.
        objDetectionTask.resumeStreaming();
        // we are using Logitech c920
        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        objDetectionTask.setAprilTagDecimation(APRIL_TAG_DECIMATION);
        objDetectionTask.doManualExposure(EXPOSURE_MS, GAIN); // Use low exposure time to reduce motion blur
        objDetectionTask.setDesiredTagID(desiredTagID);
        addTask(objDetectionTask);
    }


//    public AprilTagDetection findAprilTagData() {
//        if (desiredTagID == 1) {
//            while (objDetectionTask.getAprilTag(desiredTagID) == null) {
//                telemetry.addData("inside findAprilTagData looking for ID ", desiredTagID);
//
//                frontLeft.setPower(0.3);
//                frontRight.setPower(0.3);
//                backLeft.setPower(-0.3);
//                backRight.setPower(-0.3);
//            }
//            telemetry.addData("inside findAprilTagData found ID ", desiredTagID);
//            targetFound = true;
//            frontLeft.setPower(0);
//            frontRight.setPower(0);
//            backLeft.setPower(0);
//            backRight.setPower(0);
//        } else if (desiredTagID == 3) {
//            while (objDetectionTask.getAprilTag(desiredTagID) == null) {
//
//                telemetry.addData("inside findAprilTagData looking for ID ", desiredTagID);
//
//                frontLeft.setPower(-0.3);
//                frontRight.setPower(-0.3);
//                backLeft.setPower(0.3);
//                backRight.setPower(0.3);
//            }
//            telemetry.addData("inside findAprilTagData found ID ", desiredTagID);
//            targetFound = true;
//            frontLeft.setPower(0);
//            frontRight.setPower(0);
//            backLeft.setPower(0);
//            backRight.setPower(0);
//        } else {
//            telemetry.addData("inside findAprilTagData looking for ID ", desiredTagID);
//        }
//        // FIXME later do the assignment of the AprilTag detection in
//        //  the while loop to reduce redundancy
//        return objDetectionTask.getAprilTag(desiredTagID);
//    }

    public void alignWithAprilTag(AprilTagDetection tag) {
        double drive = 0;
        double strafe = 0;
        double turn = 0;

        AprilTagDetection myTagDetection;

        //while (!targetReached) {
        //if ((myTagDetection = objDetectionTask.getAprilTag(desiredTagID)) != null) {

        double rangeError = (tag.ftcPose.range - DESIRED_DISTANCE);
        double headingError = tag.ftcPose.bearing;
        double yawError = tag.ftcPose.yaw;

        drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
        strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

        telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);

        if (rangeError < 0.05 && headingError < 0.05 && yawError < 0.05) {
            targetReached = true;
            //break;
        } // FIXME print rangeError, headingError, and yawErrer
        //} // if
        telemetry.update();
        // Apply desired axes motions to the drivetrain.
        moveRobot(drive, strafe, turn);
        //sleep(10);
        //} // while
    }

    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        // from RobotAutoDriveToAprilTagOmni.java
        double leftFrontPower = x - y - yaw;
        double rightFrontPower = x + y + yaw;
        double leftBackPower = x + y - yaw;
        double rightBackPower = x - y + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        telemetry.addData("Auto", "leftFrontPower %5.2f, leftBackPower %5.2f, rightBackPower %5.2f, rightFrontPower %5.2f", leftFrontPower, leftBackPower, rightBackPower, rightFrontPower);

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }
        telemetry.update();
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


    @Override
    public void init() {
        //initializes the motors for the wheels
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // from FTC RobotAutoDriveToAprilTagTank.java example
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
//        frontLeft.setDirection(DcMotor.Direction.FORWARD);
//        backLeft.setDirection(DcMotor.Direction.FORWARD);
//        frontRight.setDirection(DcMotor.Direction.REVERSE);
//        backRight.setDirection(DcMotor.Direction.REVERSE);

        //----- Gamepad stuff -------------------------
        allianceColor = AllianceColor.DEFAULT;
        gamepad = new GamepadTask(this, GamepadTask.GamepadNumber.GAMEPAD_1);
        addTask(gamepad);
        allianceColorTlm = telemetry.addData("Alliance: ", "NOT SELECTED");
        tagChoiceTlm = telemetry.addData("Tag Position:", TagPositionOnProp.RIGHT);
        desiredIdTlm = telemetry.addData("Desired Tag ID:", 0);
        driveDirectionTlm = telemetry.addData("Drive Direction:", driveDirection);

        //---------------------------------------------

        whereAmI = telemetry.addData("whereami:","init", "part", "body");
        timesCalled = telemetry.addData("num calls", numCalls);
    }

    public void strafe(DriveDirection driveDirection) {
        int direction;
        if (driveDirection == DriveDirection.LEFT) {
            direction = LEFT;
        } else {
            direction = RIGHT;
        }
        moveRobot(0, MOTOR_SPEED * direction, 0);
    }

    public void drive(DriveDirection driveDirection) {
        int direction;
        if (driveDirection == DriveDirection.FORWARD) {
            direction = FORWARD;
        } else {
            direction = BACKWARD;
        }
        moveRobot(MOTOR_SPEED * direction, 0, 0);
    }


    // find desired id for blue alliance (1, 2, or 3)
    public void findDesiredID(TagPositionOnProp tagPos) {
        int delta = 0;
        if (allianceColor == AllianceColor.RED) {
            delta = 3;
        }
        switch (tagPos) {
            case LEFT:
                // tag ID 1 on blue and 4 on red
                if (coneAprilTags) {
                    desiredTagID = 5;
                } else {
                    desiredTagID = 1 + delta;
                }
                break;
            case MIDDLE:
                // tag ID 2 on blue and 5 on red
                if (coneAprilTags) {
                    desiredTagID = 2;
                } else {
                    desiredTagID = 2 + delta;
                }
                break;
            case RIGHT:
                // tag ID 3 on blue and 6 on red
                if (coneAprilTags) {
                    desiredTagID = 18;
                } else {
                    desiredTagID = 3 + delta;
                }
                break;
        }
        desiredIdTlm.setValue(desiredTagID);
        //findAprilTagData();
    }

    public void handleGamepadSelection(GamepadTask.GamepadEvent event) {
        switch (event.kind) {
            case BUTTON_X_DOWN:
                allianceColor = AllianceColor.BLUE;
                allianceColorTlm.setValue("BLUE");
                break;
            case BUTTON_B_DOWN:
                allianceColor = AllianceColor.RED;
                allianceColorTlm.setValue("RED");
                break;
            case BUTTON_Y_DOWN:
                driveDirection = DriveDirection.FORWARD;
                driveDirectionTlm.setValue(DriveDirection.FORWARD);
                break;
            case BUTTON_A_DOWN:
                driveDirection = DriveDirection.BACKWARD;
                driveDirectionTlm.setValue(DriveDirection.BACKWARD);

                break;
            case RIGHT_BUMPER_DOWN:
                driveDirection = DriveDirection.RIGHT;
                driveDirectionTlm.setValue(DriveDirection.RIGHT);
                break;
            case LEFT_BUMPER_DOWN:
                driveDirection = DriveDirection.LEFT;
                driveDirectionTlm.setValue(DriveDirection.RIGHT);
                break;
            case DPAD_UP_DOWN:
                tagPosition = TagPositionOnProp.MIDDLE;
                tagChoiceTlm.setValue("MIDDLE");
                break;
            case DPAD_RIGHT_DOWN:
                tagPosition = TagPositionOnProp.RIGHT;
                tagChoiceTlm.setValue("RIGHT");
                break;
            case DPAD_LEFT_DOWN:
                tagPosition = TagPositionOnProp.LEFT;
                tagChoiceTlm.setValue("LEFT");
                break;
        }
        findDesiredID(tagPosition);
    }

    @Override
    public void start() {
        //findDesiredID();
        if (coneAprilTags) {
            desiredTagID = 18;
        }
        findAprilTag();

        if ((driveDirection == DriveDirection.RIGHT) ||
                (driveDirection == DriveDirection.LEFT)) {
            strafe(driveDirection);
        } else {
            drive(driveDirection);
        }


//        aprilTag = findAprilTagData();
//        alignWithAprilTag(aprilTag);
//        while (!targetReached) {
//         alignWithAprilTag(aprilTag);
//        }
//        frontLeft.setPower(0);
//        frontRight.setPower(0);
//        backLeft.setPower(0);
//        backRight.setPower(0);
    }
}