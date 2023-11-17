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
import team25core.OneWheelDirectDrivetrain;
import team25core.Robot;
import team25core.RobotEvent;

@Autonomous(name = "CSRedRightAutoDS")
public class CenterStageRedRightAutoDS1 extends Robot {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private DcMotor outtake;
    private OneWheelDirectDrivetrain outtakeDrivetrain;

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
    private DeadReckonPath middlePropPath;
    private DeadReckonPath rightPropPath;
    private DeadReckonPath leftPixelBoardPath;
    private DeadReckonPath rightPixelBoardPath;
    private DeadReckonPath middlePixelBoardPath;



    private DeadReckonPath outtakePath;

    private DeadReckonPath leftBoardParkPath;
    private DeadReckonPath middleBoardParkPath;
    private DeadReckonPath rightBoardParkPath;
    double rightDistance;
    double leftDistance;

    double minDistance;
    double maxDistance;
    public String position;

    public static double OUTTAKE_DISTANCE = 3;
    public static double OUTTAKE_SPEED = 0.1;


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

    // drive to props for an accurate distance sensor reading
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
                    detectProp();
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



    public void detectProp() {
        RobotLog.ii(TAG, "Setup detectProp");
        distanceTask = new DistanceSensorTask(this, rightSensor, leftSensor, telemetry, 0, 4, 15 ,
       9,false) {
            @Override
            public void handleEvent(RobotEvent e) {
                DistanceSensorEvent event = (DistanceSensorEvent) e;
                switch (event.kind) {
                    case LEFT_DISTANCE:
                        locationTlm.setValue("left");
                        position.equals("left");
                        driveToProp(leftPropPath);
                        releaseOuttake();
                        pixelBoardAlignment(leftBoardParkPath);
                        break;
                    case RIGHT_DISTANCE:
                        locationTlm.setValue("right");
                        position.equals("right");
                        driveToProp(rightPropPath);
                        releaseOuttake();
                        pixelBoardAlignment(rightBoardParkPath);
                        break;
                    case UNKNOWN:
                        locationTlm.setValue("middle");
                        position.equals("middle");
                        driveToProp(middlePropPath);
                        releaseOuttake();
                        pixelBoardAlignment(middleBoardParkPath);
                        break;

                }
            }
        };
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
                        moveToObjectAndReleasePixel(leftPixelBoardPath);
                    }
                    else if(position.equals("right"))
                    {
                        moveToObjectAndReleasePixel(rightPixelBoardPath);
                    }
                    else
                    {
                        moveToObjectAndReleasePixel(middlePixelBoardPath);

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

        outtake = hardwareMap.get(DcMotor.class, "outtake");
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        outtakeDrivetrain = new OneWheelDirectDrivetrain(outtake);
        outtakeDrivetrain.resetEncoders();
        outtakeDrivetrain.encodersOn();

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
        addTask(distanceTask);
    }

    public void initPaths() {
        leftPropPath = new DeadReckonPath();
        middlePropPath = new DeadReckonPath();
        rightPropPath= new DeadReckonPath();

        leftBoardParkPath = new DeadReckonPath();
        middleBoardParkPath = new DeadReckonPath();
        rightBoardParkPath= new DeadReckonPath();

        leftPixelBoardPath = new DeadReckonPath();
        rightPixelBoardPath = new DeadReckonPath();
        middlePixelBoardPath = new DeadReckonPath();


        leftPropPath.stop();
        middlePropPath.stop();
        rightPropPath.stop();

        leftPixelBoardPath.stop();
        rightPixelBoardPath.stop();
        middlePixelBoardPath.stop();

        leftBoardParkPath.stop();
        middleBoardParkPath.stop();
        rightBoardParkPath.stop();

        outtakePath = new DeadReckonPath();
        outtakePath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, OUTTAKE_DISTANCE, OUTTAKE_SPEED);


        rightPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 20, 0.5);
        rightPropPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 2, -0.5);
        rightPropPath.addSegment(DeadReckonPath.SegmentType.TURN, 35, 0.5);
        rightPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 2, 0.5);


        rightPixelBoardPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 3, -0.5);
        rightPixelBoardPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 8, 0.5);
        rightPixelBoardPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 15, 0.5);
        rightPixelBoardPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 5, -0.5);


        leftPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 20, 0.5);
        leftPropPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 2, 0.5);
        leftPropPath.addSegment(DeadReckonPath.SegmentType.TURN, 35, -0.5);
        leftPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 2, 0.5);
        leftPixelBoardPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 2, -0.5);
        leftPixelBoardPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 8, -0.5);
        leftPixelBoardPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 15, -0.5);
        leftPixelBoardPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 5, -0.5);



        middlePropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 17, 0.5);
        middlePixelBoardPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 12, -0.5);
        middlePixelBoardPath.addSegment(DeadReckonPath.SegmentType.TURN, 35, 0.5);
        middlePixelBoardPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 15, 0.5);
        middlePixelBoardPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 20, 0.5);


       leftBoardParkPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 3, 0.5);
       leftBoardParkPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 7, 0.5);
       leftBoardParkPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 3, -0.5);

       middleBoardParkPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 3, 0.5);
       middleBoardParkPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 10, 0.5);
       middleBoardParkPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 3, -0.5);

       rightBoardParkPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 3, 0.5);
       rightBoardParkPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 10, 0.5);
       rightBoardParkPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 3, -0.5);


    }
}


