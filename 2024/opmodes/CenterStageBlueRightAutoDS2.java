package opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import team25core.DeadReckonPath;
import team25core.DeadReckonTask;
import team25core.DistanceSensorTask;
import team25core.FourWheelDirectDrivetrain;
import team25core.Robot;
import team25core.RobotEvent;
import team25core.SingleShotTimerTask;

@Autonomous(name = "CSBlueRightAutoDS2")
public class CenterStageBlueRightAutoDS2 extends Robot {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private FourWheelDirectDrivetrain drivetrain;

    private DistanceSensorTask distanceTask;
    private final static String TAG = "PROP";
    private DistanceSensor rightSensor;
    private DistanceSensor leftSensor;
    private Telemetry.Item tagIdTlm;
    private Telemetry.Item rightSensorTlm;
    private Telemetry.Item leftSensorTlm;

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
    private DeadReckonPath leftBoardParkPath;
    private DeadReckonPath middleBoardParkPath;
    private DeadReckonPath rightBoardParkPath;

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
        whereAmI.setValue("in driveToMiddleProp");
        RobotLog.i("drives straight onto the middle prop line");

        this.addTask(new DeadReckonTask(this, propPath, drivetrain) {
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE) {
                    RobotLog.i("finished placing pixel");
                    delay(10);

                }
            }
        });
    }

    public void driveToRightProp(DeadReckonPath propPath) {
        whereAmI.setValue("in driveToRightProp");
        RobotLog.i("drives straight onto the right prop line");

        this.addTask(new DeadReckonTask(this, propPath, drivetrain) {
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE) {
                    RobotLog.i("finished placing pixel");
                    delay(10);
                    detectProp();

                }
            }
        });
    }

    public void driveToLeftProp(DeadReckonPath propPath) {
        whereAmI.setValue("in driveToLeftProp");
        RobotLog.i("drives straight onto the left prop line");

        this.addTask(new DeadReckonTask(this, propPath, drivetrain) {
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE) {
                    RobotLog.i("finished placing pixel");
                    delay(10);
                    detectProp();

                }
            }
        });
    }

    public void driveAwayFromMiddleProp(DeadReckonPath driveFromPropPath) {
        whereAmI.setValue("in driveAwayFromMiddleProp");
        RobotLog.i("drive from the middle prop to park");

        this.addTask(new DeadReckonTask(this, driveFromPropPath, drivetrain) {
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE) {
                    RobotLog.i("in park");

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
                    RobotLog.i("in park");

                }
            }
        });
    }

    public void detectProp() {
        RobotLog.ii(TAG, "Setup detectProp");
        delay(3);
        distanceTask = new DistanceSensorTask(this, rightSensor, leftSensor, telemetry, 0, 4, 8 ,
       5,false) {
            @Override
            public void handleEvent(RobotEvent e) {
                DistanceSensorEvent event = (DistanceSensorEvent) e;
                switch (event.kind) {
                    case LEFT_DISTANCE:
                        locationTlm.setValue("left");
                        driveAwayFromLeftProp(driveFromLeftPropPath);
                        pixelBoardAlignment(leftBoardParkPath);
                        break;
                    case RIGHT_DISTANCE:
                        //RobotLog.ii(TAG, " right distance %d", event.distance);
                        locationTlm.setValue("right");
                        driveAwayFromRightProp(driveFromRightPropPath);
                        pixelBoardAlignment(rightBoardParkPath);

                        break;
                    case UNKNOWN:
                        locationTlm.setValue("middle");
                        driveAwayFromMiddleProp(driveFromMiddlePropPath);
                        pixelBoardAlignment(middleBoardParkPath);

                        break;

                }
            }
        };
    }

    //creates a delay for robot task and sets telemetry to display that robot is in delay task
    private void delay(int delayInsec) {
        this.addTask(new SingleShotTimerTask(this, delayInsec) {
            @Override
            public void handleEvent(RobotEvent e) {
                SingleShotTimerEvent event = (SingleShotTimerEvent) e;
                if (event.kind == EventKind.EXPIRED ) {
                    whereAmI.setValue("in delay task");

                }
            }
        });

    }
    public void pixelBoardAlignment(DeadReckonPath pixelBoardPath)
    {
        whereAmI.setValue("in pixelBoardAlignment");
        RobotLog.i("drives to correct pixel position");

        this.addTask(new DeadReckonTask(this, pixelBoardPath, drivetrain){
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE)
                {
                    RobotLog.i("finished driving up to prop line");
                    detectProp();

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


        setNoncanonicalMotorDirection();

        rightSensor = hardwareMap.get(DistanceSensor.class, "rightSensor");
        leftSensor = hardwareMap.get(DistanceSensor.class, "leftSensor");

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // instantiating FourWheelDirectDrivetrain
        drivetrain = new FourWheelDirectDrivetrain(frontRight, backRight, frontLeft, backLeft);

        detectProp();
        //sets motors position to 0
        drivetrain.resetEncoders();

        //motor will try to tun at the targeted velocity
        drivetrain.encodersOn();

        // telemetry shown on the phone
        whereAmI = telemetry.addData("location in code", "init");
        tagIdTlm = telemetry.addData("tagId", "none");
        rightSensorTlm = telemetry.addData("rightSensor", "none");
        leftSensorTlm = telemetry.addData("leftSensor", "none");
        locationTlm = telemetry.addData("prop position", "none");

        initPaths();
    }

    public void setNoncanonicalMotorDirection()
    {
        // This reverses the direction of the drivetrain.
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
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

        driveToLinesPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 15, -0.25);

      //  leftPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 20, 0.35);
        driveFromLeftPropPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS,1 , 0.5);
        driveFromLeftPropPath.addSegment(DeadReckonPath.SegmentType.TURN, 32, -0.5);
        driveFromLeftPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 2, 0.5);
        driveFromLeftPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 3, -0.5);
        driveFromLeftPropPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 13.5, 0.5);
        driveFromLeftPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 50, 0.5);

       // rightPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 18, 0.35);
        driveFromRightPropPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 2, -0.5);
        driveFromRightPropPath.addSegment(DeadReckonPath.SegmentType.TURN, 35, 0.5);
        driveFromRightPropPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 2, -0.5);
        driveFromRightPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 4, 0.5);
        driveFromRightPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 2, -0.5);
        driveFromRightPropPath.addSegment(DeadReckonPath.SegmentType.TURN, 35, -0.5);
        driveFromRightPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 12, 0.5);
        driveFromRightPropPath.addSegment(DeadReckonPath.SegmentType.TURN, 33, -0.5);
        driveFromRightPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 52, 0.5);

        middlePropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 18, 0.35);
        driveFromMiddlePropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 2, 0.5);
        driveFromMiddlePropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 3, -0.5);
        driveFromMiddlePropPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 12, 0.3);
        driveFromMiddlePropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 15, 0.5);
        driveFromMiddlePropPath.addSegment(DeadReckonPath.SegmentType.TURN, 32, -0.5);
        driveFromMiddlePropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 56, 0.5);



    }
    }


