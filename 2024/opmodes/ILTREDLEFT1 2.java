//package opmodes;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//import com.qualcomm.robotcore.util.RobotLog;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//
//import team25core.DeadReckonPath;
//import team25core.DeadReckonTask;
//import team25core.DistanceSensorTask;
//import team25core.FourWheelDirectDrivetrain;
//import team25core.ObjectDetectionNewTask;
//import team25core.OneWheelDirectDrivetrain;
//import team25core.Robot;
//import team25core.RobotEvent;
//import team25core.SingleShotTimerTask;
//
//@Autonomous(name = "ILTREDLEFT1")
//public class ILTREDLEFT1 extends Robot {
//
//    private ElapsedTime timer;
//
//    private DcMotor frontLeft;
//    private double aprilTagSpeed = 0.2;
//    private DcMotor frontRight;
//    private DcMotor backLeft;
//    private DcMotor backRight;
//    private DcMotor outtake;
//    final double DESIRED_DISTANCE = 1.0; //  this is how close the camera should get to the target (inches)
//
//    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
//    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
//    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
//
//    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
//    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
//    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
//
//    double  drive           = 0;        // Desired forward power/speed (-1 to +1)
//    double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
//    double  turn            = 0;
//
//    private OneWheelDirectDrivetrain liftMotorDrivetrain;
//    private DcMotor liftMotor;
//    private OneWheelDirectDrivetrain outtakeDrivetrain;
//
//    private DeadReckonPath leftPixelBoardPath;
//    private DeadReckonPath rightPixelBoardPath;
//    private DeadReckonPath middlePixelBoardPath;
//    private FourWheelDirectDrivetrain drivetrain;
//
//    private static final double CLAW_GRAB = .6;
//    private static final double CLAW_RELEASE = .9;
//
//    private Servo clawServo;
//
//    private DistanceSensorTask distanceTask;
//    private final static String TAG = "PROP";
//    private DistanceSensor rightSensor;
//    private DistanceSensor leftSensor;
//    private Telemetry.Item tagIdTlm;
//    private Telemetry.Item rightSensorTlm;
//    private Telemetry.Item leftSensorTlm;
//
//    public String position;
//    private DeadReckonPath outtakePath;
//
//    public static double OUTTAKE_DISTANCE = 20;
//    public static double OUTTAKE_SPEED = -0.7;
//
//    public static double LIFT_DISTANCE = 20;
//    public static double LIFT_SPEED = .8;
//
//    private Telemetry.Item locationTlm;
//    private Telemetry.Item whereAmI;
//    private Telemetry.Item eventTlm;
//
//    private DeadReckonPath driveFromMiddlePropPath;
//    private DeadReckonPath driveFromLeftPropPath;
//    private DeadReckonPath driveFromRightPropPath;
//    private DeadReckonPath driveToBoardPath;
//
//    private DeadReckonPath leftBoardParkPath;
//    private DeadReckonPath middleBoardParkPath;
//    private DeadReckonPath rightBoardParkPath;
//    private DeadReckonPath leftPropPath;
//    private DeadReckonPath middlePropPath;
//    private DeadReckonPath rightPropPath;
//    private DeadReckonPath driveToLinesPath;
//    private DeadReckonPath driveToRightBoardPath;
//    private DeadReckonPath driveToLeftBoardPath;
//    private DeadReckonPath driveToMiddleBoardPath;
//
//    private DeadReckonPath liftToBoardPath;
//
//    double rightDistance;
//    double leftDistance;
//
//    double minDistance;
//    double maxDistance;
//
//    private ObjectDetectionNewTask objDetectionTask;
//    private int desiredTagID;     // Choose the tag you want to approach or set to -1 for ANY tag.
//    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
//
//    private final float APRIL_TAG_DECIMATION = 2;
//
//    private final int EXPOSURE_MS = 6;
//    private final int GAIN = 250;
//    public AprilTagDetection aprilTag;
//    boolean targetFound = false;
//    boolean targetReached = false;
//
//    private AprilTagDetection foundAprilTag;
//
//    @Override
//    public void handleEvent(RobotEvent e)
//    {
//        whereAmI.setValue("in handleEvent");
//        /*
//         * Every time we complete a segment drop a note in the robot log.
//         */
//        if (e instanceof DeadReckonTask.DeadReckonEvent) {
//            RobotLog.i("Completed path segment %d", ((DeadReckonTask.DeadReckonEvent)e).segment_num);
//        }
//    }
//    public void driveToProp(DeadReckonPath driveToLinesPath)
//    {
//        whereAmI.setValue("in driveToProp");
//        RobotLog.i("drives straight onto the launch line");
//
//        this.addTask(new DeadReckonTask(this, driveToLinesPath, drivetrain){
//            @Override
//            public void handleEvent(RobotEvent e) {
//                DeadReckonEvent path = (DeadReckonEvent) e;
//                if (path.kind == EventKind.PATH_DONE)
//                {
//                    RobotLog.i("finished parking");
//                    detectProp();
//                    addTask(distanceTask);
//
//                }
//            }
//        });
//    }
//    public void driveToMiddleProp(DeadReckonPath propPath) {
//        whereAmI.setValue("in driveToSignalZone");
//        RobotLog.i("drives straight onto the launch line");
//
//        this.addTask(new DeadReckonTask(this, propPath, drivetrain) {
//            @Override
//            public void handleEvent(RobotEvent e) {
//                DeadReckonEvent path = (DeadReckonEvent) e;
//                if (path.kind == EventKind.PATH_DONE) {
//                    RobotLog.i("finished placing pixel");
//                    releaseOuttake();
//
//                }
//            }
//        });
//    }
//
//    public void driveToRightProp(DeadReckonPath propPath) {
//        whereAmI.setValue("in driveToSignalZone");
//        RobotLog.i("drives straight onto the launch line");
//
//        this.addTask(new DeadReckonTask(this, propPath, drivetrain) {
//            @Override
//            public void handleEvent(RobotEvent e) {
//                DeadReckonEvent path = (DeadReckonEvent) e;
//                if (path.kind == EventKind.PATH_DONE) {
//                    RobotLog.i("finished placing pixel");
//                    releaseOuttake();
//
//                }
//            }
//        });
//    }
//
//    public void driveToLeftProp(DeadReckonPath propPath) {
//        whereAmI.setValue("in driveToLeftProp");
//        RobotLog.i("drives straight onto the line");
//
//        this.addTask(new DeadReckonTask(this, propPath, drivetrain) {
//            @Override
//            public void handleEvent(RobotEvent e) {
//                DeadReckonEvent path = (DeadReckonEvent) e;
//                if (path.kind == EventKind.PATH_DONE) {
//                    releaseOuttake();
//                    RobotLog.i("finished placing pixel");
//
//                }
//            }
//        });
//    }
//
//    public void driveAwayFromMiddleProp(DeadReckonPath driveFromPropPath) {
//        whereAmI.setValue("in driveAwayFromProp");
//        RobotLog.i("drive from the prop to park");
//        try{
//            Thread.sleep(2000);
//
//        } catch (InterruptedException e) {throw new RuntimeException(e);
//        }
//
//        this.addTask(new DeadReckonTask(this, driveFromPropPath, drivetrain) {
//            @Override
//            public void handleEvent(RobotEvent e) {
//                DeadReckonEvent path = (DeadReckonEvent) e;
//                if (path.kind == EventKind.PATH_DONE) {
//                    RobotLog.i("in park");
//                    findDesiredID();
//
//                }
//            }
//        });
//    }
//
//    public void driveAwayFromRightProp(DeadReckonPath driveFromRightPropPath) {
//        whereAmI.setValue("in driveAwayFromRightProp");
//        RobotLog.i("drive from the right pixel to park");
//        try{
//            Thread.sleep(2000);
//
//        } catch (InterruptedException e) {throw new RuntimeException(e);
//        }
//        this.addTask(new DeadReckonTask(this, driveFromRightPropPath, drivetrain) {
//            @Override
//            public void handleEvent(RobotEvent e) {
//                DeadReckonEvent path = (DeadReckonEvent) e;
//                if (path.kind == EventKind.PATH_DONE) {
//                    RobotLog.i("in park");
//                    findDesiredID();
//
//                }
//            }
//        });
//    }
//
//    public void driveAwayFromLeftProp(DeadReckonPath driveFromLeftPropPath) {
//        whereAmI.setValue("in driveAwayFromLeftProp");
//        RobotLog.i("drive from the left pixel to park");
//        try{
//            Thread.sleep(2000);
//
//        } catch (InterruptedException e) {throw new RuntimeException(e);
//        }
//
//        this.addTask(new DeadReckonTask(this, driveFromLeftPropPath, drivetrain) {
//            @Override
//            public void handleEvent(RobotEvent e) {
//                DeadReckonEvent path = (DeadReckonEvent) e;
//                if (path.kind == EventKind.PATH_DONE) {
//                    findDesiredID();
//
//                }
//            }
//        });
//    }
//
//    public void driveToPark(DeadReckonPath driveToParkPath) {
//
//        whereAmI.setValue("in driveAwayFromLeftProp");
//        RobotLog.i("drive from the left pixel to park");
//
//        this.addTask(new DeadReckonTask(this, driveToParkPath, drivetrain) {
//
//
//            @Override
//            public void handleEvent(RobotEvent e) {
//                DeadReckonEvent path = (DeadReckonEvent) e;
//                delay(4000);
//                if (path.kind == EventKind.PATH_DONE) {
//                }
//            }
//        });
//    }
//    private void delay(int delayInMsec) {
//        this.addTask(new SingleShotTimerTask(this, delayInMsec) {
//            @Override
//            public void handleEvent(RobotEvent e) {
//                SingleShotTimerEvent event = (SingleShotTimerEvent) e;
//                if (event.kind == EventKind.EXPIRED ) {
//                    whereAmI.setValue("in delay task");
//
//                }
//            }
//        });
//    }
//
//    public void detectProp() {
//        RobotLog.ii(TAG, "Setup detectProp");
//        delay(3);
//        distanceTask = new DistanceSensorTask(this, leftSensor, rightSensor, telemetry, 0, 10, 8 ,
//                5,false) {
//            @Override
//            public void handleEvent(RobotEvent e) {
//                DistanceSensorEvent event = (DistanceSensorEvent) e;
//                switch (event.kind) {
//                    case LEFT_DISTANCE:
//                        locationTlm.setValue("left");
//                        position ="left";
//                        driveToLeftProp(leftPropPath);
//                        break;
//                    case RIGHT_DISTANCE:
//                        position ="right";
//                        //RobotLog.ii(TAG, " right distance %d", event.distance);
//                        locationTlm.setValue("right");
//                        driveToRightProp(rightPropPath);
//                        break;
//                    case UNKNOWN:
//                        locationTlm.setValue("middle");
//                        position ="middle";
//                        driveToMiddleProp(middlePropPath);
//                        break;
//                }
//            }
//        };
//    }
//    public void moveToObjectAndReleasePixel(DeadReckonPath path)
//    {
//
//        this.addTask(new DeadReckonTask(this, path, drivetrain ){
//            @Override
//            public void handleEvent(RobotEvent e) {
//                DeadReckonEvent path = (DeadReckonEvent) e;
//                if (path.kind == EventKind.PATH_DONE)
//                {
//                    RobotLog.i("Drove to the object");
//                    whereAmI.setValue("At the object");
//                    releaseOuttake();
//
//                }
//            }
//        });
//    }
//
//    private void releaseOuttake() {
//        this.addTask(new DeadReckonTask(this, outtakePath, outtakeDrivetrain) {
//            @Override
//            public void handleEvent(RobotEvent e) {
//                DeadReckonEvent path = (DeadReckonEvent) e;
//                if (path.kind == EventKind.PATH_DONE) {
//                    whereAmI.setValue("released purple pixel");
//                    if(position.equals("left"))
//                    {
//                       // delay(1000);
//                        driveAwayFromLeftProp(driveFromLeftPropPath);
//                    }
//                    else if(position.equals("right"))
//                    {
//                        //delay(1000);
//                        driveAwayFromRightProp(driveFromRightPropPath);
//                    }
//                    else
//                    {
//                        //delay(1000);
//                        driveAwayFromMiddleProp(driveFromMiddlePropPath);
//
//                    }
//
//                }
//            }
//        });
//    }
//
//    public void liftToPlacePixelOnBoard() {
//        this.addTask(new DeadReckonTask(this, liftToBoardPath, liftMotorDrivetrain) {
//            @Override
//            public void handleEvent(RobotEvent e) {
//                DeadReckonEvent path = (DeadReckonEvent) e;
//                if (path.kind == EventKind.PATH_DONE) {
//                    RobotLog.i("liftedToBoard");
//                    ElapsedTime localtimer1 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
//                    while(localtimer1.time() < 1000) {}
//                    clawServo.setPosition(CLAW_RELEASE);
//                    ElapsedTime localtimer2 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
//                    while(localtimer2.time() < 1000) {}
//                }
//                if (position.equals("left")) {
//                    driveToPark(leftBoardParkPath);
//                } else if (position.equals("right")) {
//                    driveToPark(rightBoardParkPath);
//                } else {
//                    driveToPark(middleBoardParkPath);
//
//                }
//            }
//        });
//    }
//
//    public void findAprilTag() {
//        RobotLog.ii(TAG, "Setup findAprilTag");
//        objDetectionTask = new ObjectDetectionNewTask(this, telemetry, ObjectDetectionNewTask.DetectionKind.APRILTAG_DETECTED) {
//            @Override
//            public void handleEvent(RobotEvent e) {
//                TagDetectionEvent event = (TagDetectionEvent) e;
//                switch (event.kind) {
//                    case APRIL_TAG_DETECTED:
//                        RobotLog.ii(TAG, "AprilTag detected");
//                        foundAprilTag = event.aprilTag;
//                        int foundAprilTagId = foundAprilTag.id;
//                        break;
//                }
//            }
//        };
//        objDetectionTask.init(telemetry, hardwareMap);
//        objDetectionTask.rateLimit(100); // currently calling objDetectionTask every second
//        objDetectionTask.start();
//        objDetectionTask.resumeStreaming();
//        objDetectionTask.setAprilTagDecimation(APRIL_TAG_DECIMATION);
//        objDetectionTask.doManualExposure(EXPOSURE_MS, GAIN); // Use low exposure time to reduce motion blur
//        objDetectionTask.setDesiredTagID(desiredTagID);
//        addTask(objDetectionTask);
//    }
//
//    // find desired id for blue alliance (1, 2, or 3)
//    public void findDesiredID() {
//        if (position.equals("left")) {
//            desiredTagID = 4; // 4 on red
//        } else if (position.equals("right")) {
//            desiredTagID = 6; // 5 on red
//        } else {
//            // middle tag
//            desiredTagID = 5; // 6 on red
//        }
//        findAprilTagData();
//    }
//
//    public double alignWithAprilTag(){
//            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
//            double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
//            double  headingError    = desiredTag.ftcPose.bearing;
//            double  yawError        = desiredTag.ftcPose.yaw;
//
//            // Use the speed and turn "gains" to calculate how we want the robot to move.
//            drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
//            turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
//            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
//            moveRobot(drive, turn, strafe);
//            return headingError;
//        }
//
//    public void moveRobot(double x, double y, double yaw) {
//        // Calculate wheel powers.
//        double leftFrontPower    =  -(x -y -yaw);
//        double rightFrontPower   =  -(x +y +yaw);
//        double leftBackPower     =  -(x +y -yaw);
//        double rightBackPower    =  -(x -y +yaw);
//
////        frontLeft.setPower(aprilTagSpeed);
////        frontRight.setPower(-aprilTagSpeed);
////        backLeft.setPower(-aprilTagSpeed);
////        backRight.setPower(aprilTagSpeed);
//
//        // Normalize wheel powers to be less than 1.0
//        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
//        max = Math.max(max, Math.abs(leftBackPower));
//        max = Math.max(max, Math.abs(rightBackPower));
//
//        if (max > 1.0) {
//            leftFrontPower /= max;
//            rightFrontPower /= max;
//            leftBackPower /= max;
//            rightBackPower /= max;
//        }
//
//        // Send powers to the wheels.
//        frontLeft.setPower(leftFrontPower);
//        frontRight.setPower(rightFrontPower);
//        backLeft.setPower(leftBackPower);
//        backRight.setPower(rightBackPower);
//    }
//
//    public AprilTagDetection findAprilTagData() {
//        if (desiredTagID == 4) {
//            while (objDetectionTask.getAprilTag(desiredTagID) == null) {
//                telemetry.addData("inside findAprilTagData looking for ID ", desiredTagID);
//
//                frontLeft.setPower(-aprilTagSpeed);
//                frontRight.setPower(aprilTagSpeed);
//                backLeft.setPower(aprilTagSpeed);
//                backRight.setPower(-aprilTagSpeed);
//            }
//            telemetry.addData("inside findAprilTagData found ID ", desiredTagID);
//            targetFound = true;
//            frontLeft.setPower(0);
//            frontRight.setPower(0);
//            backLeft.setPower(0);
//            backRight.setPower(0);
//            driveToBoard(driveToBoardPath);
//
//        } else if (desiredTagID == 5) {
//            while (objDetectionTask.getAprilTag(desiredTagID) == null) {
//
//                telemetry.addData("inside findAprilTagData looking for ID ", desiredTagID);
//
//                frontLeft.setPower(-aprilTagSpeed);
//                frontRight.setPower(aprilTagSpeed);
//                backLeft.setPower(aprilTagSpeed);
//                backRight.setPower(-aprilTagSpeed);
//            }
//            telemetry.addData("inside findAprilTagData found ID ", desiredTagID);
//            targetFound = true;
//            frontLeft.setPower(0);
//            frontRight.setPower(0);
//            backLeft.setPower(0);
//            backRight.setPower(0);
//            driveToBoard(driveToBoardPath);
//
//        } else if (desiredTagID == 6){
//            while (objDetectionTask.getAprilTag(desiredTagID) == null) {
//
//                telemetry.addData("inside findAprilTagData looking for ID ", desiredTagID);
//
//                frontLeft.setPower(-aprilTagSpeed);
//                frontRight.setPower(aprilTagSpeed);
//                backLeft.setPower(aprilTagSpeed);
//                backRight.setPower(-aprilTagSpeed);
//
//            }
////            while (Math.abs(alignWithAprilTag() )> 0.2 && timer.time() < 28000) {
////                telemetry.addData("searching for apriltag 2 ", desiredTagID);
////
////            }
////
////                telemetry.addData("inside findAprilTagData found ID ", desiredTagID);
//            targetFound = true;
//            frontLeft.setPower(0);
//            frontRight.setPower(0);
//            backLeft.setPower(0);
//            backRight.setPower(0);
//            driveToBoard(driveToBoardPath);
//            telemetry.addData("inside findAprilTagData looking for ID ", desiredTagID);
//            desiredTag = objDetectionTask.getAprilTag(desiredTagID);
//
//        }
//
//        // FIXME later do the assignment of the AprilTag detection in
//        //  the while loop to reduce redundancy
//        return objDetectionTask.getAprilTag(desiredTagID);
//    }
//    public void driveToBoard(DeadReckonPath driveToBoardPath) {
//        whereAmI.setValue("in driveToBoard");
//        RobotLog.i("drive to board");
//        this.addTask(new DeadReckonTask(this, driveToBoardPath, drivetrain) {
//            @Override
//            public void handleEvent(RobotEvent e) {
//                DeadReckonEvent path = (DeadReckonEvent) e;
//                if (path.kind == EventKind.PATH_DONE){
//                liftToPlacePixelOnBoard();
//            }}
//        });
//    }
//    public void init()
//    {
//        // hardware mapping
//        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
//        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
//        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
//        backRight = hardwareMap.get(DcMotor.class, "backRight");
//        clawServo = hardwareMap.servo.get("clawServo");
//
//        rightSensor = hardwareMap.get(DistanceSensor.class, "rightSensor");
//        leftSensor = hardwareMap.get(DistanceSensor.class, "leftSensor");
//
//        // instantiating FourWheelDirectDrivetrain
//        drivetrain = new FourWheelDirectDrivetrain(frontRight, backRight, frontLeft, backLeft);
//
//        detectProp();
//        //sets motors position to 0
//        drivetrain.resetEncoders();
//
//        clawServo.setPosition(CLAW_GRAB);
//
//        //motor will try to tun at the targeted velocity
//        drivetrain.encodersOn();
//
//        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        outtake = hardwareMap.get(DcMotor.class, "transportMotor");
//        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        outtakeDrivetrain = new OneWheelDirectDrivetrain(outtake);
//        outtakeDrivetrain.resetEncoders();
//        outtakeDrivetrain.encodersOn();
//
//        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
//        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        liftMotorDrivetrain = new OneWheelDirectDrivetrain(liftMotor);
//        liftMotorDrivetrain.resetEncoders();
//        liftMotorDrivetrain.encodersOn();
//
//        // telemetry shown on the phone
//        whereAmI = telemetry.addData("location in code", "init");
//        tagIdTlm = telemetry.addData("tagId", "none");
//        rightSensorTlm = telemetry.addData("rightSensor", "none");
//        leftSensorTlm = telemetry.addData("leftSensor", "none");
//        locationTlm = telemetry.addData("prop position", "none");
//
//        initPaths();
//    }
//
//    public void start()
//    {
//        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
//        whereAmI.setValue("in Start");
//        driveToProp(driveToLinesPath);
//        findAprilTag();
//        aprilTag = findAprilTagData();
//    }
//
//    public void initPaths() {
//        leftPropPath = new DeadReckonPath();
//        middlePropPath = new DeadReckonPath();
//        rightPropPath = new DeadReckonPath();
//
//        leftBoardParkPath = new DeadReckonPath();
//        middleBoardParkPath = new DeadReckonPath();
//        rightBoardParkPath= new DeadReckonPath();
//
//        liftToBoardPath = new DeadReckonPath();
//        liftToBoardPath.stop();
//        liftToBoardPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, LIFT_DISTANCE, LIFT_SPEED);
//
//        outtakePath = new DeadReckonPath();
//        outtakePath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, OUTTAKE_DISTANCE, OUTTAKE_SPEED);
//
//        leftPropPath.stop();
//        middlePropPath.stop();
//        rightPropPath.stop();
//
//        leftBoardParkPath.stop();
//        middleBoardParkPath.stop();
//        rightBoardParkPath.stop();
//
//        driveToLinesPath = new DeadReckonPath();
//        driveToLinesPath.stop();
//        driveFromMiddlePropPath = new DeadReckonPath();
//        driveFromMiddlePropPath.stop();
//
//        driveFromLeftPropPath = new DeadReckonPath();
//        driveFromLeftPropPath.stop();
//        driveFromRightPropPath = new DeadReckonPath();
//        driveFromRightPropPath.stop();
//
//        driveToRightBoardPath = new DeadReckonPath();
//        driveToRightBoardPath.stop();
//
//        driveToMiddleBoardPath = new DeadReckonPath();
//        driveToMiddleBoardPath.stop();
//        driveToBoardPath = new DeadReckonPath();
//        driveToBoardPath.stop();
//
//        driveToLeftBoardPath = new DeadReckonPath();
//        driveToLeftBoardPath.stop();
//
////        driveToLeftBoardPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 1, -0.25);
////        driveToLeftBoardPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 8, -0.25);
////
////        driveToMiddleBoardPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 8, -0.25);
//
////        driveToRightBoardPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 4, -0.25);
////        driveToRightBoardPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 8, -0.25);
//        driveToBoardPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, .75, -0.25);
//        driveToBoardPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 9.75, -0.25);
//
//        driveToLinesPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 12.75, 0.25);
//
//        leftPropPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS,1 , 0.5);
//        leftPropPath.addSegment(DeadReckonPath.SegmentType.TURN, 35, -0.5);
//        leftPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 1, 0.5);
//
//        driveFromLeftPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 1, 0.5);
//        driveFromLeftPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 1, -0.5);
//        driveFromLeftPropPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 9, 0.5);
//        driveFromLeftPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 32, -0.5);
////        driveFromLeftPropPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 13, 0.5);
////        driveFromLeftPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 2.2, -0.5);
//
//        leftBoardParkPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 1, 0.5);
//        leftBoardParkPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 11, 0.9);
//        leftBoardParkPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 2, -0.9);
//
//        rightPropPath.addSegment(DeadReckonPath.SegmentType.TURN, 38, 0.5);
//        rightPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, .75, 0.5);
//
//        driveFromRightPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 1.25, 0.5);
//        driveFromRightPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 1.5, -0.5);
//        driveFromRightPropPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 11.5, -0.5);
//        driveFromRightPropPath.addSegment(DeadReckonPath.SegmentType.TURN, 77, -0.5);
//        driveFromRightPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 32, -0.5);
//
//        rightBoardParkPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 2, 0.5);
//        rightBoardParkPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 12, 0.5);
//        rightBoardParkPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 2, -0.5);
//
//        middlePropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 0.5, -0.5);
//
//        driveFromMiddlePropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 1, 0.5);
//        driveFromMiddlePropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 2, -0.5);
//        driveFromMiddlePropPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 9, -0.3);
//        driveFromMiddlePropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 12, 0.5);
//        driveFromMiddlePropPath.addSegment(DeadReckonPath.SegmentType.TURN, 37.5, -0.5);
//        driveFromMiddlePropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 38, -0.5);
//
//        middleBoardParkPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 2, 0.5);
//        middleBoardParkPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 12, 0.5);
//        middleBoardParkPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 2, -0.5);
//
//    }
//}