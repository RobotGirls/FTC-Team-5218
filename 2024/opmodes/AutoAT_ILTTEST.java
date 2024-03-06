package opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import team25core.DeadReckonPath;
import team25core.DeadReckonTask;
import team25core.DistanceSensorTask;
import team25core.FourWheelDirectDrivetrain;
import team25core.GamepadTask;
import team25core.ObjectDetectionNewTask;
import team25core.OneWheelDirectDrivetrain;
import team25core.Robot;
import team25core.RobotEvent;
import team25core.SingleShotTimerTask;

@Autonomous(name = "ILTBLUERIGHTFAR")
public class AutoAT_ILTTEST extends Robot {

    private ElapsedTime timer;

    private DcMotor frontLeft;
    private double aprilTagSpeed = 0.1;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor outtake;
    final double DESIRED_DISTANCE = 0.5; //  this is how close the camera should get to the target (inches)

    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    double  drive           = 0;        // Desired forward power/speed (-1 to +1)
    double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
    double  turn            = 0;

    private OneWheelDirectDrivetrain liftMotorDrivetrain;
    private DcMotor liftMotor;
    private OneWheelDirectDrivetrain outtakeDrivetrain;

    private DeadReckonPath leftPixelBoardPath;
    private DeadReckonPath rightPixelBoardPath;
    private DeadReckonPath middlePixelBoardPath;
    private FourWheelDirectDrivetrain drivetrain;

    private static final double CLAW_GRAB = .6;
    private static final double CLAW_RELEASE = .9;
    private static final double PIXEL_GRAB = .05;
    private static final double PIXEL_RELEASE = .95;

    private Servo clawServo;
    private Servo pixelHolderServo;

    private DistanceSensorTask distanceTask;
    private final static String TAG = "PROP";
    private DistanceSensor rightSensor;
    private DistanceSensor leftSensor;
    private Telemetry.Item rightSensorTlm;
    private Telemetry.Item leftSensorTlm;

    public String position;
    private DeadReckonPath outtakePath;

    public static double OUTTAKE_DISTANCE = 16;
    public static double OUTTAKE_SPEED = -.9;

    public static double LIFT_DISTANCE = 17.5;
    public static double LIFT_SPEED = .6;

    private GamepadTask gamepad;

    private Telemetry.Item locationTlm;
    private Telemetry.Item whereAmI;
    private Telemetry.Item eventTlm;
    private Telemetry.Item tagIDTlm;
    private Telemetry.Item allianceTlm;
    private Telemetry.Item tagPositionTlm;

//    private enum AllianceColor {
//        BLUE,
//        RED
//    }

    private enum TagPosition {
        LEFT,
        MIDDLE,
        RIGHT
    }

    // -------------------------------------------------------------
    // the next three lines have to match each other
    private int desiredTagID = 2;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private Alliance alliance = Alliance.BLUE;
    private TagPosition tagPosition = TagPosition.MIDDLE;
    // -------------------------------------------------------------

    private DeadReckonPath driveFromMiddlePropPathStage;
    private DeadReckonPath driveFromMiddlePropPathTruss;
    private DeadReckonPath driveFromLeftPropPathStage;
    private DeadReckonPath driveFromLeftPropPathTruss;
    private DeadReckonPath driveFromRightPropPathStage;
    private DeadReckonPath driveFromRightPropPathTruss;

    private DeadReckonPath leftBoardParkPathCenter;
    private DeadReckonPath middleBoardParkPathCenter;
    private DeadReckonPath rightBoardParkPathCenter;
    private DeadReckonPath leftBoardParkPathEdge;
    private DeadReckonPath middleBoardParkPathEdge;
    private DeadReckonPath rightBoardParkPathEdge;
    private DeadReckonPath leftPropPath;
    private DeadReckonPath middlePropPath;
    private DeadReckonPath rightPropPath;
    private DeadReckonPath driveToLinesPath;
    private DeadReckonPath driveToBoardPathStage;
    private DeadReckonPath driveToBoardPathTruss;

    private DeadReckonPath liftToBoardPath;

    double rightDistance;
    double leftDistance;

    double minDistance;
    double maxDistance;

    private ObjectDetectionNewTask objDetectionTask;
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    private final float APRIL_TAG_DECIMATION = 2;

    private final int EXPOSURE_MS = 6;
    private final int GAIN = 250;
    public AprilTagDetection aprilTag;
    boolean targetFound = false;
    boolean targetReached = false;

    private AprilTagDetection foundAprilTag;

    // ===============================================
    //  RED LEFT FAR FROM BACKDROP

    // make sure all defaults align with each other**
    private PassThrough passThrough = PassThrough.STAGEDOOR;
    private ParkSide parkside = ParkSide.CENTER;
    private Pause pause = Pause.NO;
    // alliance is declared above
//    private Alliance alliance = Alliance.BLUE;
    private EdgeDirection edgeDirection = EdgeDirection.RIGHT; //

    private Telemetry.Item passThroughTlm;
    private Telemetry.Item parksideTlm;
    private Telemetry.Item pauseTlm;
    // allianceTlm is declared above
//    private Telemetry.Item allianceTlm;
    private Telemetry.Item edgeDirectionTlm;

    // gamepad declared above
//    private GamepadTask gamepad;

    private enum PassThrough {
        TRUSS,
        STAGEDOOR
    }

    private enum ParkSide {
        CENTER,
        EDGE
    }

    private enum Pause {
        YES,
        NO
    }

    private enum Alliance {
        RED,
        BLUE
    }
    private enum EdgeDirection {
        RIGHT,
        LEFT
    }

    // ===============================================

    @Override
    public void handleEvent(RobotEvent e)
    {
        whereAmI.setValue("in handleEvent");
        /*
         * Every time we complete a segment drop a note in the robot log.
         */
        if (e instanceof DeadReckonTask.DeadReckonEvent) {
            RobotLog.i("Completed path segment %d", ((DeadReckonTask.DeadReckonEvent)e).segment_num);
        } else if (e instanceof GamepadTask.GamepadEvent) {
            GamepadTask.GamepadEvent event = (GamepadTask.GamepadEvent) e ;
            handleGamepadSelection(event);
            whereAmI.setValue("inside GamePadTask");
        }
    }

    // ==========================================
    // RED LEFT FAR FROM BACKDROP
    // FIXME assign passthrough and pause button
    public void handleGamepadSelection(GamepadTask.GamepadEvent selection) {
        whereAmI.setValue("inside handleGamepadSelection");
        switch (selection.kind) {
            case BUTTON_X_DOWN:
                if (alliance == Alliance.RED) {
                    parkside = ParkSide.CENTER;
                    edgeDirection = EdgeDirection.LEFT;
                } else { // Alliance is BLUE
                    parkside = ParkSide.EDGE;
                    edgeDirection = EdgeDirection.LEFT;
                }
                parksideTlm.setValue(parkside);
                whereAmI.setValue("inside BUTTON_X_DOWN");
                break;
            case BUTTON_B_DOWN:
                if (alliance == Alliance.RED) {
                    parkside = ParkSide.EDGE;
                    edgeDirection = EdgeDirection.RIGHT;
                } else { // Alliance is BLUE
                    parkside = ParkSide.CENTER;
                    edgeDirection = EdgeDirection.RIGHT;
                }
                parksideTlm.setValue(parkside);
                break;
            case DPAD_LEFT_DOWN:
                if (alliance == Alliance.RED) {
                    passThrough = PassThrough.STAGEDOOR;

                } else { // Alliance is BLUE
                    passThrough = PassThrough.TRUSS;

                }
                passThroughTlm.setValue(passThrough);
                break;
            case DPAD_RIGHT_DOWN:
                if (alliance == Alliance.RED) {
                    passThrough = PassThrough.TRUSS;

                } else { // Alliance is BLUE
                    passThrough = PassThrough.STAGEDOOR;

                }
                passThroughTlm.setValue(passThrough);
                break;
            case LEFT_BUMPER_DOWN:
                if (pause == Pause.NO) {
                    pause = Pause.YES;
                } else {
                    pause = Pause.NO;
                }
                pauseTlm.setValue(pause);
                break;
        }
    }
    // ==================================

    public void updateDesiredTagID(TagPosition tagPosition, Alliance alliance) {

        int delta = 0;
        if (alliance == Alliance.RED) {
            delta = 3;
        }

        if (tagPosition == TagPosition.LEFT) {
            desiredTagID = 1 + delta;
        } else if (tagPosition == TagPosition.RIGHT){
            desiredTagID = 3 + delta;
        } else { // tagPosition is middle
            desiredTagID = 2 + delta;
        }

        if (objDetectionTask != null) {
            objDetectionTask.setDesiredTagID(desiredTagID);

        }
        tagIDTlm.setValue(desiredTagID);
        tagPositionTlm.setValue(tagPosition);
    }

    public void driveToPropLines(DeadReckonPath driveToLinesPath)
    {
        whereAmI.setValue("in driveToPropLines");
        RobotLog.i("drives straight onto the launch line");
        // drivesToLinesPath drives forward to better detect the distance of the team prop
        this.addTask(new DeadReckonTask(this, driveToLinesPath, drivetrain){
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE)
                {
                    RobotLog.i("finished parking");
                    detectProp();
                    addTask(distanceTask);
                }
            }
        });
    }
    public void driveToTeamProp(DeadReckonPath propPath) {
        whereAmI.setValue("in driveToTeamProp");
        RobotLog.i("drives towards the team prop in preparation for pixel drop");

        this.addTask(new DeadReckonTask(this, propPath, drivetrain) {
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE) {
                    RobotLog.i("finished placing pixel");
                    pixelHolderServo.setPosition(PIXEL_RELEASE);
                    releaseOuttake();

                }
            }
        });
    }

    public void driveToBackStage(DeadReckonPath driveFromPropPath) {
//        try{
//            Thread.sleep(4000);
//
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }
        whereAmI.setValue("in driveToBackStage");
        RobotLog.i("drive from the prop to backstage");

        this.addTask(new DeadReckonTask(this, driveFromPropPath, drivetrain) {
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE) {
                    RobotLog.i("in park");
                    findAprilTagData();
                }
            }
        });
    }

    public void driveToPark(DeadReckonPath driveToParkPath) {
        whereAmI.setValue("in driveToProp");
        RobotLog.i("drive from the left pixel to park");

        this.addTask(new DeadReckonTask(this, driveToParkPath, drivetrain) {

            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                delay(6000);
                if (path.kind == EventKind.PATH_DONE) {
                }
            }
        });
    }
    private void delay(int delayInMsec) {
        try{
            Thread.sleep(delayInMsec);

        } catch (InterruptedException e) {throw new RuntimeException(e);
        }

    }

    public void detectProp() {
        RobotLog.ii(TAG, "Setup detectProp");
        delay(3);
        distanceTask = new DistanceSensorTask(this, leftSensor, rightSensor, telemetry, 0, 8, 8 ,
                5,false) {
            @Override
            public void handleEvent(RobotEvent e) {
                DistanceSensorEvent event = (DistanceSensorEvent) e;
                switch (event.kind) {
                    case LEFT_DISTANCE:
                        locationTlm.setValue("left");
                        tagPosition = TagPosition.LEFT;
                        //position = "left";
                        // turn counter clockwise to drop off pixel
                        driveToTeamProp(leftPropPath);
                       // pixelHolderServo.setPosition(PIXEL_RELEASE);
                        break;
                    case RIGHT_DISTANCE:
                        tagPosition = TagPosition.RIGHT;
                        //position = "right";
                        //RobotLog.ii(TAG, " right distance %d", event.distance);
                        locationTlm.setValue("right");
                        // turn clockwise to drop off pixel
                        driveToTeamProp(rightPropPath);
                        break;
                    case UNKNOWN:
                        locationTlm.setValue("middle");
                        tagPosition = TagPosition.MIDDLE;
                        //position = "middle";
                        // go straight to drop off pixel
                        driveToTeamProp(middlePropPath);
                        break;
                }
                updateDesiredTagID(tagPosition, alliance);
            }
        };
    }

    public void moveToObjectAndReleasePixel(DeadReckonPath path)
    {

        this.addTask(new DeadReckonTask(this, path, drivetrain ){
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE)
                {
                    RobotLog.i("Drove to the object");
                    whereAmI.setValue("At the object");
                    releaseOuttake();

                }
            }
        });
    }

    private void releaseOuttake() {
        this.addTask(new DeadReckonTask(this, outtakePath, outtakeDrivetrain) {
            @Override
            public void handleEvent(RobotEvent e) {
                pixelHolderServo.setPosition(PIXEL_RELEASE);
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE) {
                    whereAmI.setValue("released purple pixel");
                    if(tagPosition == TagPosition.LEFT)
                    {   // FIXME not sure whether the pause will delay properly,
                        // if delay still not working, talk to Cindy
                        if (pause == Pause.NO) {
                        } else {
                            delay(2000);
                        }
                        if (passThrough == PassThrough.STAGEDOOR) {
                            driveToBackStage(driveFromLeftPropPathStage);
                        } else { // TRUSS
                            driveToBackStage(driveFromLeftPropPathTruss);
                        }
                    }
                    else if(tagPosition == TagPosition.RIGHT)
                    {   // FIXME not sure whether the pause will delay properly,
                        // if delay still not working, talk to Cindy
                        if (pause == Pause.NO) {
                        } else {
                            delay(2000);
                        }
                        if (passThrough == PassThrough.STAGEDOOR) {
                            driveToBackStage(driveFromRightPropPathStage);
                        } else { // TRUSS
                            driveToBackStage(driveFromRightPropPathTruss);
                        }
                    }
                    else // MIDDLE
                    {   // FIXME not sure whether the pause will delay properly,
                        // if delay still not working, talk to Cindy
                        if (pause == Pause.NO) {
                        } else {
                            delay(2000);
                        }
                        if (passThrough == PassThrough.STAGEDOOR) {
                            driveToBackStage(driveFromMiddlePropPathStage);
                        } else { // TRUSS
                            driveToBackStage(driveFromMiddlePropPathTruss);
                        }
                    }
                }
            }
        });
    }

    public void liftToPlacePixelOnBoard() {
        this.addTask(new DeadReckonTask(this, liftToBoardPath, liftMotorDrivetrain) {
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE) {
                    RobotLog.i("liftedToBoard");
                    ElapsedTime localtimer1 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
                    while(localtimer1.time() < 1000) {}
                    clawServo.setPosition(CLAW_RELEASE);
                    ElapsedTime localtimer2 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
                    while(localtimer2.time() < 1000) {}
                }
                if (tagPosition == TagPosition.LEFT) {
                    if (parkside == ParkSide.CENTER) {
                        driveToPark(leftBoardParkPathCenter);
                    } else { // EDGE
                        driveToPark(leftBoardParkPathEdge);
                    }
                } else if (tagPosition == TagPosition.RIGHT) {
                    if (parkside == ParkSide.CENTER) {
                        driveToPark(rightBoardParkPathCenter);
                    } else { // EDGE
                        driveToPark(rightBoardParkPathEdge);
                    }
                } else { // tagPosition == TagPosition.MIDDLE
                    if (parkside == ParkSide.CENTER) {
                        driveToPark(middleBoardParkPathCenter);
                    } else { // EDGE
                        driveToPark(middleBoardParkPathEdge);
                    }
                }
            }
        });
    }

    public void findAprilTag() {
        RobotLog.ii(TAG, "Setup findAprilTag");
        objDetectionTask = new ObjectDetectionNewTask(this, telemetry, ObjectDetectionNewTask.DetectionKind.APRILTAG_DETECTED) {
            @Override
            public void handleEvent(RobotEvent e) {
                TagDetectionEvent event = (TagDetectionEvent) e;
                switch (event.kind) {
                    case APRIL_TAG_DETECTED:
                        RobotLog.ii(TAG, "AprilTag detected");
                        // AprilTag information (x, y, z, pose numbers)
                        foundAprilTag = event.aprilTag;
                        // AprilTag ID number
                        int foundAprilTagId = foundAprilTag.id;
                        break;
                }
            }
        };
        // initializes the AprilTag processor
        objDetectionTask.init(telemetry, hardwareMap);
        // objectDetection only has 1000 ms to run its process before another task has its turn in timeslice
        objDetectionTask.rateLimit(100); // currently calling objDetectionTask every second
        objDetectionTask.start();  // instantiates the rate limiting timer
        //start processing images; this uses a lot of computational resources so
        // stop streaming if you aren't using AprilTag detection
        objDetectionTask.resumeStreaming();
        // we are using Logitech C9-20
        //Adjust Image Detection to trade-off detection range
        objDetectionTask.setAprilTagDecimation(APRIL_TAG_DECIMATION);
        objDetectionTask.doManualExposure(EXPOSURE_MS, GAIN); // Use low exposure time to reduce motion blur
        objDetectionTask.setDesiredTagID(desiredTagID);
        tagIDTlm.setValue(desiredTagID);
        // starts running the task in the timeslice
        addTask(objDetectionTask);
    }


    // find desired id for blue alliance (1, 2, or 3)
//    public void findDesiredID() {
//        if (position.equals("left")) {
//            desiredTagID = 1; // 4 on red
//        } else if (position.equals("right")) {
//            desiredTagID = 3; // 5 on red
//        } else {
//            desiredTagID = 2; // 6 on red
//        }
//        findAprilTagData();
//    }

    public double alignWithAprilTag(){
            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
            double  headingError    = desiredTag.ftcPose.bearing;
            double  yawError        = desiredTag.ftcPose.yaw;

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            moveRobot(drive, turn, strafe);
            return headingError;
        }

    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  -(x -y -yaw);
        double rightFrontPower   =  -(x +y +yaw);
        double leftBackPower     =  -(x +y -yaw);
        double rightBackPower    =  -(x -y +yaw);

//        frontLeft.setPower(aprilTagSpeed);
//        frontRight.setPower(-aprilTagSpeed);
//        backLeft.setPower(-aprilTagSpeed);
//        backRight.setPower(aprilTagSpeed);

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

    // FIXME fix up with Cindy later to clean up
    public AprilTagDetection findAprilTagData() {
        if (desiredTagID == 1) {
            while (objDetectionTask.getAprilTag(desiredTagID) == null) {
                telemetry.addData("inside findAprilTagData looking for ID ", desiredTagID);

                frontLeft.setPower(aprilTagSpeed);
                frontRight.setPower(-aprilTagSpeed);
                backLeft.setPower(-aprilTagSpeed);
                backRight.setPower(aprilTagSpeed);
            }
            telemetry.addData("inside findAprilTagData found ID ", desiredTagID);
            targetFound = true;
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            if (passThrough == PassThrough.STAGEDOOR) {
                driveToBoard(driveToBoardPathStage);
            } else { // TRUSS
                driveToBoard(driveToBoardPathTruss);

            }
        } else if (desiredTagID == 3) {
            while (objDetectionTask.getAprilTag(desiredTagID) == null) {

                telemetry.addData("inside findAprilTagData looking for ID ", desiredTagID);

                frontLeft.setPower(aprilTagSpeed);
                frontRight.setPower(-aprilTagSpeed);
                backLeft.setPower(-aprilTagSpeed);
                backRight.setPower(aprilTagSpeed);
            }
            telemetry.addData("inside findAprilTagData found ID ", desiredTagID);
            targetFound = true;
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            if (passThrough == PassThrough.STAGEDOOR) {
                driveToBoard(driveToBoardPathStage);
            } else { // TRUSS
                driveToBoard(driveToBoardPathTruss);

            }
        } else if (desiredTagID == 2){
            while (objDetectionTask.getAprilTag(desiredTagID) == null) {

                telemetry.addData("inside findAprilTagData looking for ID ", desiredTagID);

                frontLeft.setPower(aprilTagSpeed);
                frontRight.setPower(-aprilTagSpeed);
                backLeft.setPower(-aprilTagSpeed);
                backRight.setPower(aprilTagSpeed);

            }
            desiredTag = objDetectionTask.getAprilTag(desiredTagID);

//            while (Math.abs(alignWithAprilTag() )> 0.2 && timer.time() < 28000) {
//                telemetry.addData("searching for apriltag 2 ", desiredTagID);
//
//            }
//
//                telemetry.addData("inside findAprilTagData found ID ", desiredTagID);
            targetFound = true;
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            if (passThrough == PassThrough.STAGEDOOR) {
                driveToBoard(driveToBoardPathStage);
            } else { // TRUSS
                driveToBoard(driveToBoardPathTruss);

            }
            telemetry.addData("inside findAprilTagData looking for ID ", desiredTagID);
        }
        // FIXME later do the assignment of the AprilTag detection in
        //  the while loop to reduce redundancy
        return objDetectionTask.getAprilTag(desiredTagID);
    }
    public void driveToBoard(DeadReckonPath driveToBoardPath) {
       // delay(4000);
        whereAmI.setValue("in driveToBoard");
        RobotLog.i("drive to board");

        this.addTask(new DeadReckonTask(this, driveToBoardPath, drivetrain) {
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE) {
                    liftToPlacePixelOnBoard();
                }
            }
        });
    }
    public void init()
    {
        // hardware mapping
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        clawServo = hardwareMap.servo.get("clawServo");
        pixelHolderServo = hardwareMap.servo.get("pixelHolderServo");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // FIXME. we can probably do this with drive train setCanonicalMotorDirection()
        // redundant since this is consistent with canonical
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        gamepad = new GamepadTask(this, GamepadTask.GamepadNumber.GAMEPAD_1);
        addTask(gamepad);

        rightSensor = hardwareMap.get(DistanceSensor.class, "rightSensor");
        leftSensor = hardwareMap.get(DistanceSensor.class, "leftSensor");

        // instantiating FourWheelDirectDrivetrain
        drivetrain = new FourWheelDirectDrivetrain(frontRight, backRight, frontLeft, backLeft);
        // FIXME is the detectProp needed here?
        // detectProp();
        //sets motors position to 0
        drivetrain.resetEncoders();

        clawServo.setPosition(CLAW_GRAB);
        pixelHolderServo.setPosition(PIXEL_GRAB);

        //motor will try to tun at the targeted velocity
        drivetrain.encodersOn();

        outtake = hardwareMap.get(DcMotor.class, "transportMotor");
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeDrivetrain = new OneWheelDirectDrivetrain(outtake);
        outtakeDrivetrain.resetEncoders();
        outtakeDrivetrain.encodersOn();

        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotorDrivetrain = new OneWheelDirectDrivetrain(liftMotor);
        liftMotorDrivetrain.resetEncoders();
        liftMotorDrivetrain.encodersOn();

        // telemetry shown on the phone
        telemetry.setAutoClear(false);
        whereAmI = telemetry.addData("location in code", "init");
        tagIDTlm = telemetry.addData("tagId", desiredTagID);
        rightSensorTlm = telemetry.addData("rightSensor", "none");
        leftSensorTlm = telemetry.addData("leftSensor", "none");
        locationTlm = telemetry.addData("prop position", "none");
        allianceTlm = telemetry.addData("Alliance:", alliance);
        tagPositionTlm = telemetry.addData("Tag position: ", tagPosition);

        // =========================
        // BLUE RIGHT FAR FROM BACKDROP
//        gamepad specified above
//        gamepad = new GamepadTask(this, GamepadTask.GamepadNumber.GAMEPAD_1);
//        addTask(gamepad);

        pauseTlm = telemetry.addData("pause", pause);
        parksideTlm = telemetry.addData("parkside", parkside);
        passThroughTlm = telemetry.addData("passThrough", passThrough);
        // allianceTlm specified above
//        allianceTlm = telemetry.addData("alliance", alliance);
        edgeDirectionTlm = telemetry.addData("edgeDirection", edgeDirection);
        // =========================

        initPaths();
    }

    public void start()
    {
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        whereAmI.setValue("in Start");
        // drives straight to lines where pixel will be placed in order be closer for distance sensing
        driveToPropLines(driveToLinesPath);
        findAprilTag();
        //aprilTag = findAprilTagData();
    }

    public void initPaths() {

        driveToLinesPath = new DeadReckonPath();
        driveToLinesPath.stop();

        leftPropPath = new DeadReckonPath();
        middlePropPath = new DeadReckonPath();
        rightPropPath = new DeadReckonPath();
        leftPropPath.stop();
        middlePropPath.stop();
        rightPropPath.stop();

        outtakePath = new DeadReckonPath();
        outtakePath.stop();

        driveFromLeftPropPathStage = new DeadReckonPath();
        driveFromMiddlePropPathStage = new DeadReckonPath();
        driveFromRightPropPathStage = new DeadReckonPath();
        driveFromLeftPropPathStage.stop();
        driveFromMiddlePropPathStage.stop();
        driveFromRightPropPathStage.stop();

        driveFromLeftPropPathTruss = new DeadReckonPath();
        driveFromMiddlePropPathTruss = new DeadReckonPath();
        driveFromRightPropPathTruss = new DeadReckonPath();
        driveFromLeftPropPathTruss.stop();
        driveFromMiddlePropPathTruss.stop();
        driveFromRightPropPathTruss.stop();

        driveToBoardPathStage = new DeadReckonPath();
        driveToBoardPathStage.stop();
        driveToBoardPathTruss = new DeadReckonPath();
        driveToBoardPathTruss.stop();

        liftToBoardPath = new DeadReckonPath();
        liftToBoardPath.stop();

        leftBoardParkPathCenter = new DeadReckonPath();
        middleBoardParkPathCenter = new DeadReckonPath();
        rightBoardParkPathCenter = new DeadReckonPath();
        leftBoardParkPathCenter.stop();
        middleBoardParkPathCenter.stop();
        rightBoardParkPathCenter.stop();

        leftBoardParkPathEdge = new DeadReckonPath();
        middleBoardParkPathEdge = new DeadReckonPath();
        rightBoardParkPathEdge = new DeadReckonPath();
        leftBoardParkPathEdge.stop();
        middleBoardParkPathEdge.stop();
        rightBoardParkPathEdge.stop();

        // drives closer to lines to better detect distance
        driveToLinesPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 13, 0.25);

        // turn counter clock-wise to in order to drop pixel
        leftPropPath.addSegment(DeadReckonPath.SegmentType.TURN, 35.6, -0.5);

        // just goes backwards in order to drop the pixel
        middlePropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 0.5, -0.5);

        // turn clockwise and go staight to drop pixel
        rightPropPath.addSegment(DeadReckonPath.SegmentType.TURN, 37.5, 0.5);
        rightPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 1, 0.5);

        // outtake pixel onto spike/line
        outtakePath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, OUTTAKE_DISTANCE, OUTTAKE_SPEED);

        //if (passThrough == PassThrough.STAGEDOOR) {
            makeStageDoorPath();
       // } else { // PassThrough.TRUSS
            makeTrussPath();
       // }

        // drives forward and places pixel on backdrop/board
        liftToBoardPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, LIFT_DISTANCE, LIFT_SPEED);

        //if (parkside == ParkSide.CENTER) {
            makeCenterParkPath();
        //} else { // ParkSide.EDGE
            makeEdgeParkPath();
        //}

    }

    public void makeCenterParkPath() {
        // park in CENTER from LEFT April Tag
        leftBoardParkPathCenter.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 2, 0.5);
        leftBoardParkPathCenter.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 10, -0.5);

        // park in CENTER from MIDDLE April Tag
        middleBoardParkPathCenter.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 2, 0.5);
        middleBoardParkPathCenter.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 8, -0.5);

        // park in CENTER from RIGHT April Tag
        rightBoardParkPathCenter.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 2, 0.5);
        rightBoardParkPathCenter.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 5, -0.5);

    }

    public void makeEdgeParkPath() {
        // park in EDGE from LEFT April Tag (EDGE is LEFT for BLUE FAR)
        leftBoardParkPathEdge.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 2, 0.5);
        leftBoardParkPathEdge.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 10, 0.5);

        // park in EDGE from MIDDLE April Tag (EDGE is LEFT for BLUE FAR)
        // FIXME collapse the two straight commands; above and below
        middleBoardParkPathEdge.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 2, 0.5);
        middleBoardParkPathEdge.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 8, 0.5);
        // park in EDGE from RIGHT April Tag (EDGE is LEFT for BLUE FAR)
        rightBoardParkPathEdge.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 2, 0.5);
        rightBoardParkPathEdge.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 5, 0.5);


        // park in EDGE from RIGHT April Tag
        rightBoardParkPathEdge.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 2, 0.5);
        rightBoardParkPathEdge.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 10, -0.5);
    }

    public void makeTrussPath() {
        // drives to left april tag through TRUSS

        driveFromLeftPropPathTruss.addSegment(DeadReckonPath.SegmentType.STRAIGHT, .5, 0.5);
        driveFromLeftPropPathTruss.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 2, -0.5);
        driveFromLeftPropPathTruss.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 14, -0.45); // changed distance from 11 to 13
        driveFromLeftPropPathTruss.addSegment(DeadReckonPath.SegmentType.TURN, 77.5, -0.5);
        driveFromLeftPropPathTruss.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 34, -0.5);;
        // drives to middle april tag through TRUSS
        // driveFromMiddlePropPath goes straight, goes backwards to avoid hitting the prop,
        // then strafes to the right, then goes forward,
        // then turns clockwise to the right, then goes backwards
        // FIXME collapse the two straight commands; above and below

        driveFromMiddlePropPathTruss.addSegment(DeadReckonPath.SegmentType.STRAIGHT, .8, 0.5);
        driveFromMiddlePropPathTruss.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 10, -0.5);
        driveFromMiddlePropPathTruss.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 40, -0.3); // changed distance from 9 to 11
        driveFromMiddlePropPathTruss.addSegment(DeadReckonPath.SegmentType.TURN, 33, 0.5);
        // drives to right april tag through TRUSS
        driveFromRightPropPathTruss.addSegment(DeadReckonPath.SegmentType.STRAIGHT, .5, 0.5);
        driveFromRightPropPathTruss.addSegment(DeadReckonPath.SegmentType.STRAIGHT, .5, -0.5);
        driveFromRightPropPathTruss.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 12.25, 0.5); // changed distance from 9 to 11
        driveFromRightPropPathTruss.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 32.5, -0.5);
        // strafes RIGHT to detect desired april tag for BLUE FAR after passing through TRUSS
        driveToBoardPathTruss.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, .5, -0.25);
        driveToBoardPathTruss.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 8.5, -0.25);

    }

    public void makeStageDoorPath() {
        // drives to left april tag through STAGEDOOR
        driveFromLeftPropPathStage.addSegment(DeadReckonPath.SegmentType.STRAIGHT, .4, 0.5);
        driveFromLeftPropPathStage.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 2.2, -0.5);
        driveFromLeftPropPathStage.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 11, 0.45); // changed distance from 11 to 13
        driveFromLeftPropPathStage.addSegment(DeadReckonPath.SegmentType.TURN, 78, -0.5);
        driveFromLeftPropPathStage.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 35, -0.5);

        // drives to middle april tag through STAGEDOOR
        // driveFromMiddlePropPath goes straight, goes backwards to avoid hitting the prop,
        // then strafes to the right, then goes forward,
        // then turns clockwise to the right, then goes backwards
        // FIXME collapse the two straight commands; above and below
        driveFromMiddlePropPathStage.addSegment(DeadReckonPath.SegmentType.STRAIGHT, .8, 0.5);
        driveFromMiddlePropPathStage.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 2, -0.5);
        driveFromMiddlePropPathStage.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 12, 0.3); // changed distance from 9 to 11
        driveFromMiddlePropPathStage.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 12, 0.5);
        driveFromMiddlePropPathStage.addSegment(DeadReckonPath.SegmentType.TURN, 39, 0.5);
        driveFromMiddlePropPathStage.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 39, -0.5);

        // drives to right april tag through STAGEDOOR
        driveFromRightPropPathStage.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 1, 0.5);
        driveFromRightPropPathStage.addSegment(DeadReckonPath.SegmentType.STRAIGHT, .7, -0.5);
        driveFromRightPropPathStage.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 12.5, -0.5); // changed distance from 9 to 11
        driveFromRightPropPathStage.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 32, -0.5);

        // strafes LEFT to detect desired april tag after passing through STAGEDOOR
        driveToBoardPathStage.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 6, 0.25);
        driveToBoardPathStage.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 8.5, -0.25);

    }

}