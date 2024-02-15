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

@Autonomous(name = "AutoAT_ILTRED")
public class AutoAT_ILTRED extends Robot {

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

    private static final double CLAW_GRAB = 1;
    private static final double CLAW_RELEASE = 0.5;
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

    public static double OUTTAKE_DISTANCE = 20;
    public static double OUTTAKE_SPEED = -.9;

    public static double LIFT_DISTANCE = 20;
    public static double LIFT_SPEED = .6;

    private GamepadTask gamepad;

    private Telemetry.Item locationTlm;
    private Telemetry.Item whereAmI;
    private Telemetry.Item eventTlm;
    private Telemetry.Item tagIDTlm;
    private Telemetry.Item allianceTlm;
    private Telemetry.Item tagPositionTlm;

    private enum AllianceColor {
        BLUE,
        RED
    }

    private enum TagPosition {
        LEFT,
        MIDDLE,
        RIGHT
    }

    // -------------------------------------------------------------
    // the next three lines have to match each other
    private int desiredTagID = 2;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private AllianceColor alliance = AllianceColor.BLUE;
    private TagPosition tagPosition = TagPosition.MIDDLE;
    // -------------------------------------------------------------

    private DeadReckonPath driveFromMiddlePropPath;
    private DeadReckonPath driveFromLeftPropPath;
    private DeadReckonPath driveFromRightPropPath;

    private DeadReckonPath leftBoardParkPath;
    private DeadReckonPath middleBoardParkPath;
    private DeadReckonPath rightBoardParkPath;
    private DeadReckonPath leftPropPath;
    private DeadReckonPath middlePropPath;
    private DeadReckonPath rightPropPath;
    private DeadReckonPath driveToLinesPath;
    private DeadReckonPath driveToBoardPath;

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

    public void updateDesiredTagID(TagPosition tagPosition, AllianceColor alliance) {

        int delta = 0;
        if (alliance == AllianceColor.RED) {
            delta = 3;
        }

        if (tagPosition == TagPosition.LEFT) {
            desiredTagID = 1 + delta;
        } else if (tagPosition == TagPosition.MIDDLE){
            desiredTagID = 2 + delta;
        } else { // tagPosition is RIGHT
            desiredTagID = 3 + delta;
        }

        if (objDetectionTask != null) {
            objDetectionTask.setDesiredTagID(desiredTagID);

        }
        tagIDTlm.setValue(desiredTagID);
        tagPositionTlm.setValue(tagPosition);
    }

    public void handleGamepadSelection(GamepadTask.GamepadEvent selection) {
        whereAmI.setValue("inside handleGamepadSelection");
        switch (selection.kind) {
            case BUTTON_X_DOWN:
                alliance = AllianceColor.BLUE;
                allianceTlm.setValue(AllianceColor.BLUE);
//                if (tagPosition == TagPosition.LEFT) {
//                    desiredTagID = 1;
//                } else if (tagPosition == TagPosition.MIDDLE){
//                    desiredTagID = 2;
//                } else { // tagPosition is RIGHT
//                    desiredTagID = 3;
//                }
                whereAmI.setValue("inside BUTTON_X_DOWN");
                break;
            case BUTTON_B_DOWN:
                alliance = AllianceColor.RED;
                allianceTlm.setValue(AllianceColor.RED);
  //              if (tagPosition == TagPosition.LEFT) {
 //                   desiredTagID = 4;
//                } else if (tagPosition == TagPosition.MIDDLE){
//                    desiredTagID = 5;
//                } else { // tagPosition is RIGHT
//                    desiredTagID = 6;
//                }
                whereAmI.setValue("inside BUTTON_B_DOWN");
                break;
            case DPAD_LEFT_DOWN:
                tagPosition = TagPosition.LEFT;
                tagPositionTlm.setValue(tagPosition);
                whereAmI.setValue("inside DPAD_LEFT_DOWN");
//                if (alliance == AllianceColor.BLUE) {
//                    desiredTagID = 1;
//                } else { //alliance color is RED
//                    desiredTagID = 4;
//                }

                break;
            case DPAD_UP_DOWN:
                tagPosition = TagPosition.MIDDLE;
                tagPositionTlm.setValue(tagPosition);
                whereAmI.setValue("inside DPAD_UP_DOWN");
//                if (alliance == AllianceColor.BLUE) {
//                    desiredTagID = 2;
//                } else { // alliance color is RED
//                    desiredTagID = 5;
//                }
                break;
            case DPAD_RIGHT_DOWN:
                tagPosition = TagPosition.RIGHT;
                tagPositionTlm.setValue(tagPosition);
                whereAmI.setValue("inside DPAD_RIGHT_DOWN");
//                if (alliance == AllianceColor.BLUE) {
//                    desiredTagID = 3;
//                } else { // alliance color is RED
//                    desiredTagID = 6;
//                }
                break;
        }
        updateDesiredTagID(tagPosition, alliance);
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
        this.addTask(new SingleShotTimerTask(this, delayInMsec) {
            @Override
            public void handleEvent(RobotEvent e) {
                SingleShotTimerEvent event = (SingleShotTimerEvent) e;
                if (event.kind == EventKind.EXPIRED ) {
                    whereAmI.setValue("in delay task");

                }
            }
        });

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
                    {
                        delay(1000);
                        driveToBackStage(driveFromLeftPropPath);
                    }
                    else if(tagPosition == TagPosition.RIGHT)
                    {
                        delay(1000);
                        driveToBackStage(driveFromRightPropPath);
                    }
                    else // MIDDLE
                    {
                        delay(1000);
                        driveToBackStage(driveFromMiddlePropPath);
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
                    driveToPark(leftBoardParkPath);
                } else if (tagPosition == TagPosition.RIGHT) {
                    driveToPark(rightBoardParkPath);
                } else { // tagPosition == TagPosition.MIDDLE
                    driveToPark(middleBoardParkPath);

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
        if (desiredTagID == 4) {
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
            driveToBoard(driveToBoardPath);

        } else if (desiredTagID == 6) {
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
            driveToBoard(driveToBoardPath);

        } else if (desiredTagID == 5){
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
            driveToBoard(driveToBoardPath);
            telemetry.addData("inside findAprilTagData looking for ID ", desiredTagID);
        }
        // FIXME later do the assignment of the AprilTag detection in
        //  the while loop to reduce redundancy
        return objDetectionTask.getAprilTag(desiredTagID);
    }
    public void driveToBoard(DeadReckonPath driveToParkPath) {
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
        leftPropPath = new DeadReckonPath();
        middlePropPath = new DeadReckonPath();
        rightPropPath = new DeadReckonPath();

        leftBoardParkPath = new DeadReckonPath();
        middleBoardParkPath = new DeadReckonPath();
        rightBoardParkPath= new DeadReckonPath();

        liftToBoardPath = new DeadReckonPath();
        liftToBoardPath.stop();
        liftToBoardPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, LIFT_DISTANCE, LIFT_SPEED);

        outtakePath = new DeadReckonPath();
        outtakePath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, OUTTAKE_DISTANCE, OUTTAKE_SPEED);

        leftPropPath.stop();
        middlePropPath.stop();
        rightPropPath.stop();

        leftBoardParkPath.stop();
        middleBoardParkPath.stop();
        rightBoardParkPath.stop();

        driveToLinesPath = new DeadReckonPath();
        driveToLinesPath.stop();
        driveFromMiddlePropPath = new DeadReckonPath();
        driveFromMiddlePropPath.stop();

        driveFromLeftPropPath = new DeadReckonPath();
        driveFromLeftPropPath.stop();
        driveFromRightPropPath = new DeadReckonPath();
        driveFromRightPropPath.stop();

        driveToBoardPath = new DeadReckonPath();
        driveToBoardPath.stop();
        driveToBoardPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 5, 0.25);
        driveToBoardPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 8.5, -0.25);

        // drives closer to lines to better detect distance
        driveToBoardPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, .75, -0.25);
        driveToBoardPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 9.75, -0.25);


        driveToLinesPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 12.75, 0.25);

        leftPropPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS,1 , 0.5);
        leftPropPath.addSegment(DeadReckonPath.SegmentType.TURN, 35, -0.5);
        leftPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 1, 0.5);

        driveFromLeftPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 1, 0.5);
        driveFromLeftPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 1, -0.5);
        driveFromLeftPropPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 9, 0.5);
        driveFromLeftPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 32, -0.5);
//        driveFromLeftPropPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 13, 0.5);
//        driveFromLeftPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 2.2, -0.5);



        leftBoardParkPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 1, 0.5);
        leftBoardParkPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 7, 0.5);

        rightPropPath.addSegment(DeadReckonPath.SegmentType.TURN, 38, 0.5);
        rightPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, .75, 0.5);

        driveFromRightPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 1.25, 0.5);
        driveFromRightPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 1.5, -0.5);
        driveFromRightPropPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 11.5, -0.5);
        driveFromRightPropPath.addSegment(DeadReckonPath.SegmentType.TURN, 77, -0.5);
        driveFromRightPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 32, -0.5);


        rightBoardParkPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 2, 0.5);
        rightBoardParkPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 12, 0.5);




        middlePropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 0.5, -0.5);


        driveFromMiddlePropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, .3, 0.5);
        driveFromMiddlePropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 2, -0.5);
        driveFromMiddlePropPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 9, -0.3);
        driveFromMiddlePropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 12, 0.5);
        driveFromMiddlePropPath.addSegment(DeadReckonPath.SegmentType.TURN, 37.5, -0.5);
        driveFromMiddlePropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 38, -0.5);



        middleBoardParkPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 2, 0.5);
        middleBoardParkPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 9, 0.5);




    }
}