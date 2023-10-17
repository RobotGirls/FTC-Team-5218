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

@Autonomous(name = "CenterStageRedLeftAutoDS")
public class CenterStageRedLeftAutoDS1 extends Robot {

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
    public void driveToProp(DeadReckonPath propPath)
    {
        whereAmI.setValue("in driveToSignalZone");
        RobotLog.i("drives straight onto the launch line");

        this.addTask(new DeadReckonTask(this, propPath, drivetrain){
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE)
                {
                    RobotLog.i("finished parking");
                    detectProp();

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

    public void detectProp() {
        RobotLog.ii(TAG, "Setup detectProp");
        distanceTask = new DistanceSensorTask(this, rightSensor, leftSensor, telemetry, 25, 38, 15 ,
       9,false) {
            @Override
            public void handleEvent(RobotEvent e) {
                DistanceSensorEvent event = (DistanceSensorEvent) e;
                switch (event.kind) {
                    case LEFT_DISTANCE:
                        locationTlm.setValue("right");
                        driveToRightProp(rightPropPath);
                        break;
                    case RIGHT_DISTANCE:
                        //RobotLog.ii(TAG, " right distance %d", event.distance);
                        locationTlm.setValue("left");
                        driveToLeftProp(leftPropPath);

                        break;
                    case UNKNOWN:
                        locationTlm.setValue("middle");
                        driveToMiddleProp(middlePropPath);

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

        addTask(distanceTask);
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

        driveToLinesPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 3, 0.25);

        rightPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 15, 0.5);
        rightPropPath.addSegment(DeadReckonPath.SegmentType.TURN, 35, 0.5);
        rightPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 3, 0.5);
        driveFromRightPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 2, -0.5);
        driveFromRightPropPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 13.5, -0.5);
        driveFromRightPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 50, 0.5);

        leftPropPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 6, -0.3);
        leftPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 12, 0.5);
        driveFromLeftPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 9, -0.5);
        driveFromLeftPropPath.addSegment(DeadReckonPath.SegmentType.TURN, 38, 0.5);
        driveFromLeftPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 52, 0.5);

        middlePropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 17, 0.4);
        driveFromMiddlePropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 13.5, -0.5);
        driveFromMiddlePropPath.addSegment(DeadReckonPath.SegmentType.TURN, 38, 0.5);
        driveFromMiddlePropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 50, 0.5);



    }
    }


