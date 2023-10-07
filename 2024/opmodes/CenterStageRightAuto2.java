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
// CHANGE THIS this is the name of your autonomous that will show on the phone
@Autonomous(name = "RightCenterRigAuto")
// CHANGE THIS class name to something to your new class name
public class CenterStageRightAuto2 extends Robot {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private FourWheelDirectDrivetrain drivetrain;

    private Telemetry.Item tagIdTlm;
    private Telemetry.Item whereAmI;
    private Telemetry.Item eventTlm;

   //CHANGE THIS TO WHAT YOUR PATH DOES
    DeadReckonPath rigCenterPath;

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
    // CHANGE the method name and the path name (current method name:driveToBackDropRigCenter, current path name:rigCenterPath)
    public void driveToBackDropRigCenter(DeadReckonPath rigCenterPath)
    {
        // CHANGE this to the correct method name (current: ("in driveToBackDropRigCenter"))
        whereAmI.setValue("in driveToBackDropRigCenter");
        //CHANGE robotlog to what your robot is going to do (current:"drives to back drop")
        RobotLog.i("drives to back drop");
        //CHANGE the path name (current: rigCenterPath)
        this.addTask(new DeadReckonTask(this, rigCenterPath, drivetrain) {
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE) {
                }
                //CHANGE robotlog to what your robot is done doing (current:"finished driving to the mosaic")
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
        //CHANGE the method name and path name(current method:driveToBackDropRigCenter, current path: rigCenterPath)
        driveToBackDropRigCenter(rigCenterPath);
        whereAmI.setValue("in Start");
    }
    public void initPaths() {
        //CHANGE path name (current path: rigCenterPath)
        rigCenterPath = new DeadReckonPath();
        //CHANGE path name (current path: rigCenterPath)
        rigCenterPath.stop();


        //FOR ALL OF THE BELOW------------------------------------------------------------------
        // CHANGE path name (current path: rigCenterPath), SegmentType, distance, speed
        // positive speed goes foward and negative speed goes backwards
        // SegmentType for the direction you want the robot to go
        // distance how far
        // delete or add segments to your desire
        rigCenterPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 5, -0.5);
        rigCenterPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 5, 0.5);
        rigCenterPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT,10 , 0.5);
        rigCenterPath.addSegment(DeadReckonPath.SegmentType.TURN, 35, 0.5);
        rigCenterPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT,20 , 0.5);
        rigCenterPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 5, -0.5);

    }
}



