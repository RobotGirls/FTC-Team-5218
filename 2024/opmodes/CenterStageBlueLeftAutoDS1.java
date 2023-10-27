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

@Autonomous(name = "CSBlueLeftAutoDS")
public class CenterStageBlueLeftAutoDS1 extends Robot {

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

    private DeadReckonPath leftPropPath;
    private DeadReckonPath driveToLinesPath;

    private DeadReckonPath middlePropPath;
    private DeadReckonPath rightPropPath;
    double rightDistance;
    double leftDistance;

    double minDistance;
    double maxDistance;
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
        whereAmI.setValue("in driveToProp");
        RobotLog.i("drives straight to the prop lines");

        this.addTask(new DeadReckonTask(this, propPath, drivetrain){
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE)
                {
                    RobotLog.i("finished driving up to prop line");
                    addTask(distanceTask);
                    detectProp();


                }
            }
        });
    }



    public void detectProp() {

        delay(3);
        RobotLog.ii(TAG, "Setup detectProp");
        distanceTask = new DistanceSensorTask(this, rightSensor, leftSensor, telemetry, 0, 4, 8 ,
       5,false) {
            @Override
            public void handleEvent(RobotEvent e) {
                DistanceSensorEvent event = (DistanceSensorEvent) e;
                switch (event.kind) {
                    case LEFT_DISTANCE:
                        locationTlm.setValue("left");
                        driveToProp(leftPropPath);
                        break;
                    case RIGHT_DISTANCE:
                        //RobotLog.ii(TAG, " right distance %d", event.distance);
                        locationTlm.setValue("right");
                        driveToProp(rightPropPath);

                        break;
                    case UNKNOWN:
                        locationTlm.setValue("middle");
                        driveToProp(middlePropPath);

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

    public void start()
    {
        whereAmI.setValue("in Start");
        driveToProp(driveToLinesPath);
    }

    public void initPaths() {
        leftPropPath = new DeadReckonPath();
        middlePropPath = new DeadReckonPath();
        rightPropPath= new DeadReckonPath();
        driveToLinesPath= new DeadReckonPath();


        leftPropPath.stop();
        middlePropPath.stop();
        rightPropPath.stop();
        driveToLinesPath.stop();


        driveToLinesPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 15, 0.25);

        leftPropPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS,1 , 0.5);
        leftPropPath.addSegment(DeadReckonPath.SegmentType.TURN, 32, -0.5);
        leftPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 2, 0.5);
        leftPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 3, -0.5);
        leftPropPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 5, -0.5);
        leftPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 20, 0.5);

        rightPropPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 2, -0.5);
        rightPropPath.addSegment(DeadReckonPath.SegmentType.TURN, 35, 0.5);
        rightPropPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 2, -0.5);
        rightPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 4, 0.5);
        rightPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 2, -0.5);
        rightPropPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 5, -0.5);
        rightPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 20, 0.5);

        middlePropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 2, 0.5);
        middlePropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 12, -0.5);
        middlePropPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 12, 0.3);



    }
}


