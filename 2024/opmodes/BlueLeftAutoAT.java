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

@Autonomous(name = "ILTBLUELEFTNEAR")
public class BlueLeftAutoAT extends Robot {

    private ElapsedTime timer;

    private DcMotor frontLeft;
    private double aprilTagSpeed = 0.255;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor outtake;
    final double DESIRED_DISTANCE = 1.0; //  this is how close the camera should get to the target (inches)

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

    private Servo clawServo;

    private DistanceSensorTask distanceTask;
    private final static String TAG = "PROP";
    private DistanceSensor rightSensor;
    private DistanceSensor leftSensor;
    private Telemetry.Item tagIdTlm;
    private Telemetry.Item rightSensorTlm;
    private Telemetry.Item leftSensorTlm;

    public String position;
    private DeadReckonPath outtakePath;

    public static double OUTTAKE_DISTANCE = 20;
    public static double OUTTAKE_SPEED = -0.7;

    public static double LIFT_DISTANCE = 17;
    public static double LIFT_SPEED = .6;


    private Telemetry.Item locationTlm;
    private Telemetry.Item whereAmI;
    private Telemetry.Item eventTlm;

    private DeadReckonPath driveFromMiddlePropPath;
    private DeadReckonPath driveFromLeftPropPath;
    private DeadReckonPath driveFromRightPropPath;

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
    private int desiredTagID;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    private final float APRIL_TAG_DECIMATION = 2;

    private final int EXPOSURE_MS = 6;
    private final int GAIN = 250;
    public AprilTagDetection aprilTag;
    boolean targetFound = false;
    boolean targetReached = false;

    private AprilTagDetection foundAprilTag;

    // ===============================================
    //  LEFT BLUE NEAR BACKDROP

    // paths from spot in front of April Tag on the Back Drop to Park on the EDGE
    // (instead of parking on CENTER)

    DeadReckonPath leftBoardParkPathCenter;
    DeadReckonPath middleBoardParkPathCenter;
    DeadReckonPath rightBoardParkPathCenter;
    DeadReckonPath leftBoardParkPathEdge;
    DeadReckonPath middleBoardParkPathEdge;
    DeadReckonPath rightBoardParkPathEdge;
//    DeadReckonPath fromLeftATtoEdgePath;
//    DeadReckonPath fromMiddleATtoEdgePath;
//    DeadReckonPath fromRightATtoEdgePath;

    // make sure all defaults align with each other**
    private ParkSide parkside = ParkSide.CENTER;
    private Alliance alliance = Alliance.BLUE;
    private EdgeDirection edgeDirection = EdgeDirection.LEFT;

    private Telemetry.Item parksideTlm;
    private Telemetry.Item allianceTlm;
    private Telemetry.Item edgeDirectionTlm;

    private GamepadTask gamepad;

    private enum ParkSide {
        CENTER,
        EDGE
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
        } // =============================
        else if (e instanceof GamepadTask.GamepadEvent) {
            GamepadTask.GamepadEvent event = (GamepadTask.GamepadEvent) e ;
            handleGamepadSelection(event);
            whereAmI.setValue("inside GamePadTask");
        }
        // ================================
    }

    // ==========================================
    // BLUE LEFT NEAR BACKDROP
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
                break;
        }
    }
    // ==================================

    public void driveToSpikes(DeadReckonPath driveToLinesPath)
    {
        whereAmI.setValue("in driveToSpikes");
        RobotLog.i("drives straight closer to spikes/line");

        this.addTask(new DeadReckonTask(this, driveToLinesPath, drivetrain){
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE)
                {
                    RobotLog.i("finished  driving closer to spikes");
                    detectProp();
                    addTask(distanceTask);
                }
            }
        });
    }
    public void headsTowardsProp(DeadReckonPath propPath) {
        whereAmI.setValue("in headsTowardsProp");
        RobotLog.i("head towards prop on spikes");

        this.addTask(new DeadReckonTask(this, propPath, drivetrain) {
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE) {
                    RobotLog.i("about to place pixel");
                    releaseOuttake();

                }
            }
        });
    }

    public void driveAwayFromProp(DeadReckonPath driveFromPropPath) {
        whereAmI.setValue("in driveAwayFromProp");
        RobotLog.i("drive from the prop to backstage");

        this.addTask(new DeadReckonTask(this, driveFromPropPath, drivetrain) {
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE) {
                    RobotLog.i("at backstage");
                    findDesiredID();
                }
            }
        });
    }

    public void driveToPark(DeadReckonPath driveToParkPath) {
        whereAmI.setValue("in driveToPark");
        RobotLog.i("drive from the left pixel to park");

        this.addTask(new DeadReckonTask(this, driveToParkPath, drivetrain) {


            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
              //  delay(6000);
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
        distanceTask = new DistanceSensorTask(this, leftSensor, rightSensor, telemetry, 0, 7, 8 ,
                5,false) {
            @Override
            public void handleEvent(RobotEvent e) {
                DistanceSensorEvent event = (DistanceSensorEvent) e;
                switch (event.kind) {
                    case LEFT_DISTANCE:
                        locationTlm.setValue("left");
                        position ="left";
                        headsTowardsProp(leftPropPath);
                        break;
                    case RIGHT_DISTANCE:
                        position ="right";
                        //RobotLog.ii(TAG, " right distance %d", event.distance);
                        locationTlm.setValue("right");
                        headsTowardsProp(rightPropPath);
                        break;
                    case UNKNOWN:
                        locationTlm.setValue("middle");
                        position ="middle";
                        headsTowardsProp(middlePropPath);
                        break;
                }
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
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE) {
                    whereAmI.setValue("released purple pixel");
                    if(position.equals("left"))
                    {
                       // delay(1000);
                        driveAwayFromProp(driveFromLeftPropPath);
                    }
                    else if(position.equals("right"))
                    {
                       // delay(1000);
                        driveAwayFromProp(driveFromRightPropPath);
                    }
                    else
                    {
                       // delay(1000);
                        driveAwayFromProp(driveFromMiddlePropPath);
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
                    delay(500);
                    clawServo.setPosition(CLAW_RELEASE);
               delay(250);
                    if (position.equals("left")) {
                        if (parkside == ParkSide.CENTER) {
                            driveToPark(leftBoardParkPathCenter);
                        } else { // park on EDGE which is left for BLUE NEAR
                            driveToPark(leftBoardParkPathEdge);
                        }
                    } else if (position.equals("right")) {
                        if (parkside == ParkSide.CENTER) {
                            driveToPark(rightBoardParkPathCenter);
                        } else { // park on EDGE which is left for BLUE NEAR
                            driveToPark(rightBoardParkPathEdge);
                        }
                    } else { // MIDDLE
                        if (parkside == ParkSide.CENTER) {
                            driveToPark(middleBoardParkPathCenter);
                        } else { // park on EDGE which is left for BLUE NEAR
                            driveToPark(middleBoardParkPathEdge);
                        }
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
                        foundAprilTag = event.aprilTag;
                        int foundAprilTagId = foundAprilTag.id;
                        break;
                }
            }
        };
        objDetectionTask.init(telemetry, hardwareMap);
        objDetectionTask.rateLimit(100); // currently calling objDetectionTask every second
        objDetectionTask.start();
        objDetectionTask.resumeStreaming();
        objDetectionTask.setAprilTagDecimation(APRIL_TAG_DECIMATION);
        objDetectionTask.doManualExposure(EXPOSURE_MS, GAIN); // Use low exposure time to reduce motion blur
        objDetectionTask.setDesiredTagID(desiredTagID);
        addTask(objDetectionTask);
    }


    // find desired id for blue alliance (1, 2, or 3)
    public void findDesiredID() {
        if (position.equals("left")) {
            desiredTagID = 1; // 4 on red
        } else if (position.equals("right")) {
            desiredTagID = 3; // 5 on red
        } else {
            desiredTagID = 2; // 6 on red
        }
        findAprilTagData();
    }

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

    public AprilTagDetection findAprilTagData() {
        if (desiredTagID == 1) {
            while (objDetectionTask.getAprilTag(desiredTagID) == null) {
                telemetry.addData("inside findAprilTagData looking for ID ", desiredTagID);

                frontLeft.setPower(-aprilTagSpeed);
                frontRight.setPower(aprilTagSpeed);
                backLeft.setPower(aprilTagSpeed);
                backRight.setPower(-aprilTagSpeed);
            }
            telemetry.addData("inside findAprilTagData found ID ", desiredTagID);
            targetFound = true;
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            driveToBoard(driveToBoardPath);

        } else if (desiredTagID == 3) {
            while (objDetectionTask.getAprilTag(desiredTagID) == null) {

                telemetry.addData("inside findAprilTagData looking for ID ", desiredTagID);

                frontLeft.setPower(-aprilTagSpeed);
                frontRight.setPower(aprilTagSpeed);
                backLeft.setPower(aprilTagSpeed);
                backRight.setPower(-aprilTagSpeed);
            }
            telemetry.addData("inside findAprilTagData found ID ", desiredTagID);
            targetFound = true;
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            driveToBoard(driveToBoardPath);

        } else if (desiredTagID == 2){
            while (objDetectionTask.getAprilTag(desiredTagID) == null) {

                telemetry.addData("inside findAprilTagData looking for ID ", desiredTagID);

                frontLeft.setPower(-aprilTagSpeed);
                frontRight.setPower(aprilTagSpeed);
                backLeft.setPower(aprilTagSpeed);
                backRight.setPower(-aprilTagSpeed);

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

        rightSensor = hardwareMap.get(DistanceSensor.class, "rightSensor");
        leftSensor = hardwareMap.get(DistanceSensor.class, "leftSensor");

        // instantiating FourWheelDirectDrivetrain
        drivetrain = new FourWheelDirectDrivetrain(frontRight, backRight, frontLeft, backLeft);

        detectProp();
        //sets motors position to 0
        drivetrain.resetEncoders();

        clawServo.setPosition(CLAW_GRAB);

        //motor will try to tun at the targeted velocity
        drivetrain.encodersOn();

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        whereAmI = telemetry.addData("location in code", "init");
        tagIdTlm = telemetry.addData("tagId", "none");
        rightSensorTlm = telemetry.addData("rightSensor", "none");
        leftSensorTlm = telemetry.addData("leftSensor", "none");
        locationTlm = telemetry.addData("prop position", "none");

        // =========================
        // BLUE LEFT NEAR BACKDROP
        gamepad = new GamepadTask(this, GamepadTask.GamepadNumber.GAMEPAD_1);
        addTask(gamepad);

        parksideTlm = telemetry.addData("parkside", parkside);
        allianceTlm = telemetry.addData("alliance", alliance);
        edgeDirectionTlm = telemetry.addData("edgeDirection", edgeDirection);
        // =========================

        initPaths();
    }

    public void start()
    {
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        whereAmI.setValue("in Start");
        driveToSpikes(driveToLinesPath);
        findAprilTag();
        aprilTag = findAprilTagData();
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

        driveFromLeftPropPath = new DeadReckonPath();
        driveFromLeftPropPath.stop();
        driveFromMiddlePropPath = new DeadReckonPath();
        driveFromMiddlePropPath.stop();
        driveFromRightPropPath = new DeadReckonPath();
        driveFromRightPropPath.stop();
        driveToBoardPath = new DeadReckonPath();
        driveToBoardPath.stop();

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

//        fromLeftATtoEdgePath = new DeadReckonPath();
//        fromMiddleATtoEdgePath = new DeadReckonPath();
//        fromRightATtoEdgePath = new DeadReckonPath();
//        fromLeftATtoEdgePath.stop();
//        fromMiddleATtoEdgePath.stop();
//        fromRightATtoEdgePath.stop();

        // drives to lines to better see the team prop
        driveToLinesPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 13, 0.25);

        // drives to left spike/line to prepare to drop pixel
        leftPropPath.addSegment(DeadReckonPath.SegmentType.TURN, 35, -0.5);
       // leftPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, .5, 0.5);

        // drives forward to middle spike/line to prepare to drop pixel
        middlePropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 0.5, -0.5);

        // drives to right spike/line to prepare to drop pixel
        rightPropPath.addSegment(DeadReckonPath.SegmentType.TURN, 37.85, 0.5);
        rightPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, .5, 0.5);

        // outtake pixel onto spike/line
        outtakePath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, OUTTAKE_DISTANCE, OUTTAKE_SPEED);

        // drives to left april tag
        driveFromLeftPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, .5, 0.5);
        driveFromLeftPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 1, -0.5);
        driveFromLeftPropPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 5, -0.5);
        driveFromLeftPropPath.addSegment(DeadReckonPath.SegmentType.TURN,77 , 0.5);
        driveFromLeftPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 10, -0.5);

        // drives to middle april tag
        driveFromMiddlePropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, .5, 0.5);
        driveFromMiddlePropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 5, -0.5);
        driveFromMiddlePropPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS,2 , -0.5);
        driveFromMiddlePropPath.addSegment(DeadReckonPath.SegmentType.TURN, 37.5, 0.5);
        driveFromMiddlePropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 10, -0.8);

        // drives to right april tag
        driveFromRightPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, .5, 0.5);
        driveFromRightPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 13, -0.5);
       // driveFromRightPropPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 1, 0.5);

        // strafes LEFT to detect desired april tag
        driveToBoardPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, .8, 0.25);
        driveToBoardPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 8, -0.25);

        // drives forward and places pixel on backdrop/board
        liftToBoardPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, LIFT_DISTANCE, LIFT_SPEED);

        // park in CENTER from LEFT April Tag
        leftBoardParkPathCenter.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 2, 0.5);
        leftBoardParkPathCenter.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 8, 0.9);
        leftBoardParkPathCenter.addSegment(DeadReckonPath.SegmentType. STRAIGHT, 5, -0.9);


        // park in CENTER from MIDDLE April Tag
        middleBoardParkPathCenter.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 2, 0.5);
        middleBoardParkPathCenter.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 13, 0.9);
        middleBoardParkPathCenter.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 5, -0.5);


        // park in CENTER from RIGHT April Tag
        rightBoardParkPathCenter.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 2, 0.5);
        rightBoardParkPathCenter.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 15, 0.9);
        rightBoardParkPathCenter.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 5, -0.5);
     //   rightBoardParkPathCenter.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 15, 0.9);

        // park in EDGE from LEFT April Tag (EDGE is left for BLUE NEAR)
        leftBoardParkPathEdge.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 4, 0.5);
        leftBoardParkPathEdge.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 10, -0.5);

        // park in EDGE from MIDDLE April Tag (EDGE is left for BLUE NEAR)
        middleBoardParkPathEdge.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 2, 0.5);
        middleBoardParkPathEdge.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 9, -0.5);

        // park in EDGE from RIGHT April Tag (EDGE is left for BLUE NEAR)
        rightBoardParkPathEdge.addSegment(DeadReckonPath.SegmentType.STRAIGHT, .5, -0.25);
        rightBoardParkPathEdge.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 2, 0.5);
        rightBoardParkPathEdge.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 6, -0.5);

    }
}