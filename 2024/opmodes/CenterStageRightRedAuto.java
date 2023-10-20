package opmodes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import team25core.DeadReckonPath;
import team25core.DeadReckonTask;
import team25core.FourWheelDirectDrivetrain;
import team25core.Robot;
import team25core.RobotEvent;
import com.qualcomm.robotcore.hardware.DistanceSensor;

@Autonomous(name = "CenterStageRightRedAuto")
public class CenterStageRightRedAuto extends Robot {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private FourWheelDirectDrivetrain drivetrain;

    private final static String TAG = "PROP";
    private DistanceSensor rightSensor;
    private DistanceSensor leftSensor;
    private Telemetry.Item tagIdTlm;
    private Telemetry.Item rightSensorTlm;
    private Telemetry.Item leftSensorTlm;

    private DeadReckonPath leftPropPath;
    private DeadReckonPath middlePropPath;
    private DeadReckonPath rightPropPath;
    private DeadReckonPath driveToLinesPath;

    private Telemetry.Item locationTlm;
    private Telemetry.Item whereAmI;
    private Telemetry.Item eventTlm;

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
                    RobotLog.i("finished placing pixel");

                }
            }
        });
    }
    public void driveToDetectProp(DeadReckonPath driveToLinesPath)
    {
        whereAmI.setValue("in driveToDetectProp");
        RobotLog.i("drives straight to the lines");

        this.addTask(new DeadReckonTask(this, driveToLinesPath, drivetrain){
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE)
                {
                    driveToProp(middlePropPath);
                    RobotLog.i("drive up to the lines");

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

        rightSensor = hardwareMap.get(DistanceSensor.class, "rightSensor");
        leftSensor = hardwareMap.get(DistanceSensor.class, "leftSensor");

        // instantiating FourWheelDirectDrivetrain
        drivetrain = new FourWheelDirectDrivetrain(frontRight, backRight, frontLeft, backLeft);

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
//        while(true){
//            leftDistance = leftSensor.getDistance(DistanceUnit.CM);
//            rightDistance = rightSensor.getDistance(DistanceUnit.CM);
//            rightSensorTlm.setValue(rightDistance);
//            leftSensorTlm.setValue(leftDistance);
        driveToProp(rightPropPath);
    }

    public void initPaths() {
        leftPropPath = new DeadReckonPath();
        middlePropPath = new DeadReckonPath();
        rightPropPath= new DeadReckonPath();

        leftPropPath.stop();
        middlePropPath.stop();
        rightPropPath.stop();

        driveToLinesPath= new DeadReckonPath();
        driveToLinesPath.stop();

        driveToLinesPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 3, 0.25);


        leftPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 14, 0.5);
        leftPropPath.addSegment(DeadReckonPath.SegmentType.TURN, 35, -0.5);
        leftPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 1, 0.5);
        leftPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 3, -0.5);
        leftPropPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 9, -0.5);
        leftPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 20, -0.5);

        rightPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 2, 0.5);
        rightPropPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, -7, -0.5);
        rightPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 11, 0.5);
        rightPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 9, -0.5);
        rightPropPath.addSegment(DeadReckonPath.SegmentType.TURN, 35, 0.5);
        rightPropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 20, 0.5);

        middlePropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 17, 0.5);
        middlePropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 12, -0.5);
        middlePropPath.addSegment(DeadReckonPath.SegmentType.TURN, 35, 0.5);
        middlePropPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 20, 0.5);
    }
}



