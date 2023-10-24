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
import team25core.vision.apriltags.AprilTagDetectionTask;

@Autonomous(name = "N0")
public class CenterStageRightAuto extends Robot {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private FourWheelDirectDrivetrain drivetrain;

    private Telemetry.Item tagIdTlm;
    private Telemetry.Item whereAmI;
    private Telemetry.Item eventTlm;

    DeadReckonPath rigOnePath;

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
    public void driveToBackDropRigOne(DeadReckonPath rigOnePath)
    {
        whereAmI.setValue("in driveToBackDropRigOne");
        RobotLog.i("drives to back drop");

        this.addTask(new DeadReckonTask(this, rigOnePath, drivetrain) {
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE) {
                }
                RobotLog.i("finished driving to the mosaic");

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

        // instantiating FourWheelDirectDrivetrain
        drivetrain = new FourWheelDirectDrivetrain(frontRight, backRight, frontLeft, backLeft);

        //sets motors position to 0
        drivetrain.resetEncoders();

        //motor will try to tun at the targeted velocity
        drivetrain.encodersOn();

        // telemetry shown on the phone
        whereAmI = telemetry.addData("location in code", "init");
        tagIdTlm = telemetry.addData("tagId", "none");
        initPaths();
    }
    public void start()
    {

        driveToBackDropRigOne(rigOnePath);
        whereAmI.setValue("in Start");
    }
    public void initPaths() {
        rigOnePath = new DeadReckonPath();

        rigOnePath.stop();

        rigOnePath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 10, -0.5);
        rigOnePath.addSegment(DeadReckonPath.SegmentType.TURN, 35, -0.5);
        rigOnePath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 20, 0.5);
        // rigTwoPath.addSegment(DeadReckonPath.SegmentType.TURN, 20, 0.5);
        // rigTwoPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 5, -0.5);
        // rigTwoPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 20, 0.5);
       // rigTwoPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 6, -0.5);

    }
}



