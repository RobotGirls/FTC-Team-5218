package opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import team25core.DeadReckonPath;
import team25core.DeadReckonTask;
import team25core.DistanceSensorTask;
import team25core.FourWheelDirectDrivetrain;
import team25core.OneWheelDirectDrivetrain;
import team25core.Robot;
import team25core.RobotEvent;
import team25core.SingleShotTimerTask;

@Autonomous(name = "CSBlueRightAutoDS2")
public class CenterStageBlueRightAutoDS2 extends Robot {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor outtake;

    private OneWheelDirectDrivetrain liftMotorDrivetrain;
    private DcMotor liftMotor;
    private OneWheelDirectDrivetrain outtakeDrivetrain;

    private DeadReckonPath leftPixelBoardPath;
    private DeadReckonPath rightPixelBoardPath;
    private DeadReckonPath middlePixelBoardPath;
    private FourWheelDirectDrivetrain drivetrain;

    private static final double CLAW_GRAB = 0.5;
    private static final double CLAW_RELEASE = 0.3;

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

    public static double OUTTAKE_DISTANCE = 2;
    public static double OUTTAKE_SPEED = 0.7;

    public static double LIFT_DISTANCE = 50;
    public static double LIFT_SPEED = .6;


    private Telemetry.Item locationTlm;
    private Telemetry.Item whereAmI;
    private Telemetry.Item eventTlm;

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

    private DeadReckonPath liftToBoardPath;

    double rightDistance;
    double leftDistance;

    double minDistance;
    double maxDistance;

    @Override
    public void handleEvent(RobotEvent e)
    {
        whereAmI.setValue("in handleEvent");
        /*
         * Every time we complete a segment drop a note in the robot log.
         */
        if (e instanceof DeadReckonTask.DeadReckonEvent) {
            RobotLog.i("Completed path segment %d", ((DeadReckonTask.DeadReckonEvent)e).segment_num);
        }
    }
    public void driveToProp(DeadReckonPath driveToLinesPath)
    {
        whereAmI.setValue("in driveToProp");
        RobotLog.i("drives straight onto the launch line");

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
    public void driveToMiddleProp(DeadReckonPath propPath) {
        whereAmI.setValue("in driveToSignalZone");
        RobotLog.i("drives straight onto the launch line");

        this.addTask(new DeadReckonTask(this, propPath, drivetrain) {
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE) {
                    RobotLog.i("finished placing pixel");
                    releaseOuttake();

                }
            }
        });
    }

    public void driveToRightProp(DeadReckonPath propPath) {
        whereAmI.setValue("in driveToSignalZone");
        RobotLog.i("drives straight onto the launch line");

        this.addTask(new DeadReckonTask(this, propPath, drivetrain) {
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE) {
                    RobotLog.i("finished placing pixel");
                    releaseOuttake();

                }
            }
        });
    }

    public void driveToLeftProp(DeadReckonPath propPath) {
        whereAmI.setValue("in driveToLeftProp");
        RobotLog.i("drives straight onto the line");

        this.addTask(new DeadReckonTask(this, propPath, drivetrain) {
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE) {
                    releaseOuttake();
                    RobotLog.i("finished placing pixel");


                }
            }
        });
    }

    public void driveAwayFromMiddleProp(DeadReckonPath driveFromPropPath) {
        whereAmI.setValue("in driveAwayFromProp");
        RobotLog.i("drive from the prop to park");

        this.addTask(new DeadReckonTask(this, driveFromPropPath, drivetrain) {
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE) {
                    RobotLog.i("in park");
                    liftToPlacePixelOnBoard();

                }
            }
        });
    }

    public void driveAwayFromRightProp(DeadReckonPath driveFromRightPropPath) {
        whereAmI.setValue("in driveAwayFromRightProp");
        RobotLog.i("drive from the right pixel to park");

        this.addTask(new DeadReckonTask(this, driveFromRightPropPath, drivetrain) {
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE) {
                    RobotLog.i("in park");
                    liftToPlacePixelOnBoard();
                }
            }
        });
    }

    public void driveAwayFromLeftProp(DeadReckonPath driveFromLeftPropPath) {
        whereAmI.setValue("in driveAwayFromLeftProp");
        RobotLog.i("drive from the left pixel to park");

        this.addTask(new DeadReckonTask(this, driveFromLeftPropPath, drivetrain) {
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE) {
                   // liftToPlacePixelOnBoard();

                }
            }
        });
    }

    public void driveToPark(DeadReckonPath driveToParkPath) {
        whereAmI.setValue("in driveAwayFromLeftProp");
        RobotLog.i("drive from the left pixel to park");

        this.addTask(new DeadReckonTask(this, driveToParkPath, drivetrain) {
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
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
                        position ="left";
                        driveToLeftProp(leftPropPath);
                        break;
                    case RIGHT_DISTANCE:
                        position ="right";
                        //RobotLog.ii(TAG, " right distance %d", event.distance);
                        locationTlm.setValue("right");
                        driveToRightProp(rightPropPath);

                        break;
                    case UNKNOWN:
                        locationTlm.setValue("middle");
                        position ="middle";
                        driveToMiddleProp(middlePropPath);

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
                        delay(1000);
                        driveAwayFromLeftProp(driveFromLeftPropPath);
                    }
                    else if(position.equals("right"))
                    {
                        delay(1000);
                        driveAwayFromRightProp(driveFromRightPropPath);
                    }
                    else
                    {
                        delay(1000);
                        driveAwayFromMiddleProp(driveFromMiddlePropPath);

                    }

                }
            }


        });
    }
    public void liftToPlacePixelOnBoard()
    {
        this.addTask(new DeadReckonTask(this, liftToBoardPath, liftMotorDrivetrain){
            @Override
            public void handleEvent (RobotEvent e){
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE)
                {
                    RobotLog.i("liftedToBoard");
                    delay(6);
                    clawServo.setPosition(CLAW_RELEASE);
                    delay(  1000);

                }
            }
        });
    }
    public void pixelBoardPark(DeadReckonPath pixelBoardPath)
    {
        whereAmI.setValue("in pixelBoardAlignment");
        RobotLog.i("drives to correct pixel position");

        this.addTask(new DeadReckonTask(this, pixelBoardPath, drivetrain){
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE)
                {
                    if(position.equals("left"))
                    {
                        driveToPark(leftBoardParkPath);
                    }
                    else if(position.equals("right"))
                    {
                        driveToPark(rightBoardParkPath);
                    }
                    else
                    {
                        driveToPark(middleBoardParkPath);

                    }

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

        outtake = hardwareMap.get(DcMotor.class, "intakeMotor");
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

        initPaths();
    }

    public void start()
    {
        whereAmI.setValue("in Start");
        driveToProp(driveToLinesPath);
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

        driveToLinesPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 13, 0.25);

        leftPropPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS,1 , 0.5);
        leftPropPath.addSegment(DeadReckonPath.SegmentType.TURN, 35, -0.5);
        leftPropPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 2, 0.5);
        leftPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 1, 0.5);

        driveFromLeftPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 4, -0.5);
        driveFromLeftPropPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 14, 0.5);
        driveFromLeftPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 50, 0.5);

        leftBoardParkPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 2, 0.5);
        leftBoardParkPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 5, -0.5);
        leftBoardParkPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 2, -0.5);



        rightPropPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 2, -0.5);
        rightPropPath.addSegment(DeadReckonPath.SegmentType.TURN, 36, 0.5);
        rightPropPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 1, -0.5);
        rightPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 2, 0.5);

        driveFromRightPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 3, -0.5);
        driveFromRightPropPath.addSegment(DeadReckonPath.SegmentType.TURN, 37, -0.5);
        driveFromRightPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 9, 0.5);
        driveFromRightPropPath.addSegment(DeadReckonPath.SegmentType.TURN, 39, -0.5);
        driveFromRightPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 48, 0.5);
       // driveFromRightPropPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 7, -0.5);
       // driveFromRightPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 5, 0.5);

        rightBoardParkPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 2, 0.5);
        rightBoardParkPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 5, -0.5);
        rightBoardParkPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 2, -0.5);




        middlePropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 0.5, -0.5);

        driveFromMiddlePropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 3, -0.5);
        driveFromMiddlePropPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 9, 0.3);
        driveFromMiddlePropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 13, 0.5);
        driveFromMiddlePropPath.addSegment(DeadReckonPath.SegmentType.TURN, -39, 0.5);
        driveFromMiddlePropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 43, 0.5);
        driveFromMiddlePropPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 15, 0.5);
        driveFromMiddlePropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 1, -0.5);

        middleBoardParkPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 2, 0.5);
        middleBoardParkPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 5, -0.5);
        middleBoardParkPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 2.5, -0.5);




    }
}


