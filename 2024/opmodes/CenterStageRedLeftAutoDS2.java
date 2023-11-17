package opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
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

@Autonomous(name = "CSRedLeftAutoDS2")
public class CenterStageRedLeftAutoDS2 extends Robot {

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
                    driveAwayFromMiddleProp(driveFromMiddlePropPath);

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
                    // delay(10);
                    driveAwayFromRightProp(driveFromRightPropPath);

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
                    RobotLog.i("finished placing pixel");
                    driveAwayFromLeftProp(driveFromLeftPropPath);

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
        distanceTask = new DistanceSensorTask(this, rightSensor, leftSensor, telemetry, 0, 4, 8 ,
                5,false) {
            @Override
            public void handleEvent(RobotEvent e) {
                DistanceSensorEvent event = (DistanceSensorEvent) e;
                switch (event.kind) {
                    case LEFT_DISTANCE:
                        locationTlm.setValue("left");
                        driveAwayFromLeftProp(driveFromLeftPropPath);
                        break;
                    case RIGHT_DISTANCE:
                        //RobotLog.ii(TAG, " right distance %d", event.distance);
                        locationTlm.setValue("right");
                        driveAwayFromRightProp(driveFromRightPropPath);

                        break;
                    case UNKNOWN:
                        locationTlm.setValue("middle");
                        driveAwayFromMiddleProp(driveFromMiddlePropPath);

                        break;


                }
            }
        };
    }


    public void init()
    {
        // hardware mapping
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        rightSensor = hardwareMap.get(DistanceSensor.class, "rightSensor");
        leftSensor = hardwareMap.get(DistanceSensor.class, "leftSensor");

        // instantiating FourWheelDirectDrivetrain
        drivetrain = new FourWheelDirectDrivetrain(frontRight, backRight, frontLeft, backLeft);

        detectProp();
        //sets motors position to 0
        drivetrain.resetEncoders();

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

        leftPropPath.stop();
        middlePropPath.stop();
        rightPropPath.stop();

        driveToLinesPath = new DeadReckonPath();
        driveToLinesPath.stop();
        driveFromMiddlePropPath = new DeadReckonPath();
        driveFromMiddlePropPath.stop();

        driveFromLeftPropPath = new DeadReckonPath();
        driveFromLeftPropPath.stop();
        driveFromRightPropPath = new DeadReckonPath();
        driveFromRightPropPath.stop();

        driveToLinesPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 15, 0.25);

        driveFromRightPropPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS,1 , -0.5);
        driveFromRightPropPath.addSegment(DeadReckonPath.SegmentType.TURN, 35, 0.5);
        driveFromRightPropPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 2, -0.5);
        driveFromRightPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 3.4, 0.5);
        driveFromRightPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 4, -0.5);
        driveFromRightPropPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 13.5, -0.5);
        driveFromRightPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 50, 0.5);

        driveFromLeftPropPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 2, 0.5);
        driveFromLeftPropPath.addSegment(DeadReckonPath.SegmentType.TURN, 36, -0.5);
        driveFromLeftPropPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 3, 0.5);
        driveFromLeftPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 3.5, 0.5);
        driveFromLeftPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 3, -0.5);
        driveFromLeftPropPath.addSegment(DeadReckonPath.SegmentType.TURN, 37, 0.5);
        driveFromLeftPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 9, 0.5);
        driveFromLeftPropPath.addSegment(DeadReckonPath.SegmentType.TURN, 39, 0.5);
        driveFromLeftPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 48, 0.5);
        driveFromLeftPropPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 7, 0.5);
        driveFromLeftPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 5, 0.5);


        driveFromMiddlePropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 2, 0.5);
        driveFromMiddlePropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 3, -0.5);
        driveFromMiddlePropPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 9, -0.3);
        driveFromMiddlePropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 13, 0.5);
        driveFromMiddlePropPath.addSegment(DeadReckonPath.SegmentType.TURN, 39, 0.5);
        driveFromMiddlePropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 56, 0.5);



    }
    }


