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
    final double DESIRED_DISTANCE = 10.0; //  this is how close the camera should get to the target (inches)
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
    private final int CONE_TAG_ID = 6;
    private int desiredTagID = DEFAULT_TAG_ID;     // Choose the tag you want to approach or set to -1 for ANY tag.

    private final float APRIL_TAG_DECIMATION = 3;

    private final int EXPOSURE_MS = 6;
    private final int GAIN = 250;

    private final int SLOW_DOWN = 4; // slow down twice as much

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
    private final int LEFT = 1;
    private final int RIGHT = -1;
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
        RIGHT,
        DONT_MOVE
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


    private Telemetry.Item coneTlm;

    private boolean startHasBeenPushed = false;
    private boolean skipAlign = false;
    private Telemetry.Item skipAlignTlm;

    private int PRINT_APRIL_TAG_LEVEL = 3;
    private boolean doAlign = false;

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
        whereAmI.setValue("findAprilTag-beginning");

        RobotLog.ii(TAG, "Setup findAprilTag");
        objDetectionTask = new ObjectDetectionNewTask(this, telemetry, ObjectDetectionNewTask.DetectionKind.APRILTAG_DETECTED) {
            @Override
            public void handleEvent(RobotEvent e) {
                whereAmI.setValue("findAprilTag-handleEvent");

                TagDetectionEvent event = (TagDetectionEvent) e;
                switch (event.kind) {
                    case APRIL_TAG_DETECTED:
                        RobotLog.ii(TAG, "AprilTag detected");
                        // AprilTag information (x, y, z, pose numbers)
                        foundAprilTag = event.aprilTag;
                        // AprilTag ID
                        foundAprilTagId = foundAprilTag.id;
                        numCalls += 1;
                        whereAmI.setValue("handleEvent-APRIL_TAG_DETECTED");
                        timesCalled.setValue(numCalls);
                        if (startHasBeenPushed) {
                            //objDetectionTask.setVerbosity(PRINT_APRIL_TAG_LEVEL);
                            if (skipAlign) {
                                whereAmI.setValue("handleEvent-stopRobot");

                                stopRobot();
                            } else {
                                objDetectionTask.setVerbosity(PRINT_APRIL_TAG_LEVEL);
                                whereAmI.setValue("handleEvent-align");
                                alignWithAprilTag(foundAprilTag);
                            }
                        }
                        break;
                    default:
                        whereAmI.setValue("handleEvent-default case");

                }
            }
        };
        // initializes the aprilTag processor
        objDetectionTask.init(telemetry, hardwareMap);
        // FIXME make sure this is the rate we want to use
        objDetectionTask.rateLimit(10); // currently calling objDetectionTask every 1/4 second
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
        objDetectionTask.setDriveGains(SPEED_GAIN, STRAFE_GAIN, TURN_GAIN);
        objDetectionTask.setMaxAuto(MAX_AUTO_SPEED, MAX_AUTO_STRAFE, MAX_AUTO_TURN);
        objDetectionTask.setDesiredDistance(DESIRED_DISTANCE);
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
        double rangeError;
        double headingError;
        double yawError;
        double[] errorData;

        whereAmI.setValue("alignWithAprilTag");

        if (gamepad1.left_bumper) {
            errorData = objDetectionTask.getDriveErrors(tag);
            drive = errorData[0]/SLOW_DOWN;
            turn = errorData[1]/SLOW_DOWN;
            strafe = errorData[2]/SLOW_DOWN;
//        rangeError = errorData[ObjectDetectionNewTask.ErrDataType.RANGE_ERROR.getValue()];
//        headingError = errorData[ObjectDetectionNewTask.ErrDataType.HEADING_ERROR.getValue()];
//        yawError = errorData[ObjectDetectionNewTask.ErrDataType.YAW_ERROR.getValue()];
            rangeError = errorData[3];
            headingError = errorData[4];
            yawError = errorData[5];

            objDetectionTask.printErrorData();
        } else {
            // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
            drive  = -gamepad1.left_stick_y  / 2.0;  // Reduce drive rate to 50%.
            strafe = -gamepad1.left_stick_x  / 2.0;  // Reduce strafe rate to 50%.
            turn   = -gamepad1.right_stick_x / 3.0;  // Reduce turn rate to 33%.
            //telemetry.addData("Manual","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        }

//        double rangeError = (tag.ftcPose.range - DESIRED_DISTANCE);
//        double headingError = tag.ftcPose.bearing;
//        double yawError = tag.ftcPose.yaw;
//
//        drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
//        turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
//        strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
//
//
//        if (rangeError < 0.05 && headingError < 0.05 && yawError < 0.05) {
//            targetReached = true;
//            //break;
//        } // FIXME print rangeError, headingError, and yawErrer
//        telemetry.update();

//       if (drive < 0.02 && turn < 0.02 && strafe < 0.02) {
//        if (Math.abs(rangeError) < 2 && Math.abs(headingError) < 2 &&
//            Math.abs(yawError) < 2) {

//        if (Math.abs(rangeError) < 5 && Math.abs(headingError) < 5 ) {
//
//             stopRobot();
//        } else {
            // Apply desired axes motions to the drivetrain.
            //moveRobot(drive, strafe, turn);
            moveRobot(drive, strafe, turn);

//        }


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

        //telemetry.addData("Auto", "leftFrontPower %5.2f, leftBackPower %5.2f, rightBackPower %5.2f, rightFrontPower %5.2f", leftFrontPower, leftBackPower, rightBackPower, rightFrontPower);

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }
        //telemetry.update();
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



    public void strafe(DriveDirection driveDirection) {
        int direction;
        if (driveDirection == DriveDirection.LEFT) {
            direction = LEFT;
        } else {
            direction = RIGHT;
        }
        driveDirectionTlm.setValue(driveDirection);
        moveRobot(0, MOTOR_SPEED * direction, 0);
    }

    public void drive(DriveDirection driveDirection) {
        int direction;
        if (driveDirection == DriveDirection.FORWARD) {
            direction = FORWARD;
        } else {
            direction = BACKWARD;
        }
        driveDirectionTlm.setValue(driveDirection);

        moveRobot(MOTOR_SPEED * direction, 0, 0);
    }

    public void turn(DriveDirection driveDirection) {
        int direction;
        if (driveDirection == DriveDirection.RIGHT) {
            direction = RIGHT;
        } else {
            direction = LEFT;
        }
        driveDirectionTlm.setValue(driveDirection);

        moveRobot(0,0,MOTOR_SPEED * direction);
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
                    desiredTagID = 0;
                } else {
                    desiredTagID = 1 + delta;
                }
                break;
            case MIDDLE:
                // tag ID 2 on blue and 5 on red
                if (coneAprilTags) {
                    desiredTagID = 19;
                } else {
                    desiredTagID = 2 + delta;
                }
                break;
            case RIGHT:
                // tag ID 3 on blue and 6 on red
                if (coneAprilTags) {
                    desiredTagID = 6;
                } else {
                    desiredTagID = 3 + delta;
                }
                break;
        }
        desiredIdTlm.setValue(desiredTagID);
        //findAprilTagData();
    }

    public void stopRobot() {
        whereAmI.setValue("stopRobot");

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
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
            case DPAD_DOWN_DOWN:
                driveDirection = DriveDirection.DONT_MOVE;
                driveDirectionTlm.setValue(DriveDirection.DONT_MOVE);
                //stopRobot();
                break;
            case RIGHT_STICK_DOWN:
                skipAlign = true;
                skipAlignTlm.setValue(skipAlign);
                break;
            case RIGHT_STICK_LEFT:
                turn(driveDirection.LEFT);
                break;
            case RIGHT_STICK_RIGHT:
                turn(driveDirection.RIGHT);
                break;
            case RIGHT_STICK_NEUTRAL:
                stopRobot();
                break;
            case LEFT_STICK_LEFT:
                strafe(driveDirection.LEFT);
                break;
            case LEFT_STICK_RIGHT:
                strafe(driveDirection.RIGHT);
                break;
            case LEFT_STICK_UP:
                drive(driveDirection.FORWARD);
                break;
            case LEFT_STICK_DOWN:
                drive(driveDirection.BACKWARD);
                break;
            case LEFT_STICK_NEUTRAL:
                stopRobot();
                break;
            case RIGHT_TRIGGER_DOWN:
                driveDirection = DriveDirection.RIGHT;
                driveDirectionTlm.setValue(DriveDirection.RIGHT);
                break;
            case LEFT_TRIGGER_DOWN:
                driveDirection = DriveDirection.LEFT;
                driveDirectionTlm.setValue(DriveDirection.LEFT);
                break;
            case LEFT_BUMPER_DOWN:
                driveDirection = DriveDirection.DONT_MOVE;
                driveDirectionTlm.setValue(DriveDirection.DONT_MOVE);
                doAlign = true;
                break;
            case LEFT_BUMPER_UP:
                doAlign = false;
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
    public void init() {
        //initializes the motors for the wheels
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);


        //----- Gamepad stuff -------------------------
        allianceColor = AllianceColor.DEFAULT;
        gamepad = new GamepadTask(this, GamepadTask.GamepadNumber.GAMEPAD_1);
        addTask(gamepad);
        allianceColorTlm = telemetry.addData("Alliance: ", "NOT SELECTED");
        tagChoiceTlm = telemetry.addData("Tag Position:", TagPositionOnProp.RIGHT);

        if (coneAprilTags) {
            desiredTagID = CONE_TAG_ID;
        }
        desiredIdTlm = telemetry.addData("Desired Tag ID:", desiredTagID);
        driveDirectionTlm = telemetry.addData("Drive Direction:", driveDirection);
        skipAlignTlm = telemetry.addData("Skip Align:", skipAlign);

        //---------------------------------------------

        whereAmI = telemetry.addData("whereami:","init");
        timesCalled = telemetry.addData("num calls", numCalls);
        coneTlm = telemetry.addData("cone", coneAprilTags);

        if (coneAprilTags) {
            desiredTagID = 6;
        }
        findAprilTag();
    }

    @Override
    public void start() {
        //findDesiredID();
        startHasBeenPushed = true;

//        if ((driveDirection == DriveDirection.RIGHT) ||
//                (driveDirection == DriveDirection.LEFT)) {
//            strafe(driveDirection);
//        } else if ((driveDirection == DriveDirection.FORWARD) ||
//                (driveDirection == DriveDirection.BACKWARD)){
//            drive(driveDirection);
//        }


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