package opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import team25core.DeadReckonTask;
import team25core.FourWheelDirectDrivetrain;
import team25core.ObjectDetectionNewTask;
import team25core.Robot;
import team25core.RobotEvent;

@Autonomous(name = "AprilTagAuto")
public class CenterstageAutoAprilTags extends Robot {
    private ObjectDetectionNewTask objDetectionTask;
    private final static String TAG = "Prop";
    final double DESIRED_DISTANCE = 2.0; //  this is how close the camera should get to the target (inches)
    //  Set the GAIN constants to control the relationship between the measured position error,
    //  and how much power is applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger
    //  for a more aggressive response.
    final double SPEED_GAIN = 0.01;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN = 0.015;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN = 0.01;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    private DcMotor frontLeft=null;
    private DcMotor frontRight=null;
    private DcMotor backLeft=null;
    private DcMotor backRight=null;
    private FourWheelDirectDrivetrain drivetrain;

    private int desiredTagID;     // Choose the tag you want to approach or set to -1 for ANY tag.

    private final float APRIL_TAG_DECIMATION = 2;

    private final int EXPOSURE_MS = 6;
    private final int GAIN = 250;

    public String position = "left"; // this will contain the actual prop position information in final auto

    public AprilTagDetection aprilTag;
    boolean targetFound = false;
    boolean targetReached = false;

    @Override
    public void handleEvent(RobotEvent e) {
        /*
         * Every time we complete a segment drop a note in the robot log.
         */
        //if (e instanceof DeadReckonTask.DeadReckonEvent) {
        //  RobotLog.i("Completed path segment %d", ((DeadReckonTask.DeadReckonEvent)e).segment_num);
<<<<<<< HEAD
        //}
=======
>>>>>>> 9edf1438b6dd5f5170907702f469b183e45ae052
    }

    public void findAprilTag() {
        RobotLog.ii(TAG, "Setup findAprilTag");
        objDetectionTask = new ObjectDetectionNewTask(this, telemetry, ObjectDetectionNewTask.DetectionKind.APRILTAG_DETECTED) {
            @Override
            public void handleEvent(RobotEvent e) {
                TagDetectionEvent event = (TagDetectionEvent) e;
                switch (event.kind) {
                    case OBJECTS_DETECTED:
                        RobotLog.ii(TAG, "Object detected");
                        break;
                }
            }
        };
        objDetectionTask.init(telemetry, hardwareMap);
        objDetectionTask.rateLimit(1000); // currently calling objDetectionTask every second
        objDetectionTask.start();
        objDetectionTask.resumeStreaming();
<<<<<<< HEAD
        objDetectionTask.setAprilTagDecimation(APRIL_TAG_DECIMATION);
        objDetectionTask.doManualExposure(EXPOSURE_MS, GAIN); // Use low exposure time to reduce motion blur
        objDetectionTask.setDesiredTagID(desiredTagID);
        addTask(objDetectionTask);
    }


    // find desired id for blue alliance (1, 2, or 3)
    public void findDesiredID() {
        if (position.equals("left")) {
            desiredTagID = 1; // 4 on red
        } else if (position.equals("center")) {
            targetFound = true;
            desiredTagID = 2; // 5 on red
        } else {
            desiredTagID = 3; // 6 on red
        }
        //aprilTag = findAprilTagData();
        //findAprilTag();
    }

    public AprilTagDetection findAprilTagData() {
//        targetReached = false;
//        targetFound = false;

        if (desiredTagID == 1) {
            while (objDetectionTask.getAprilTag(desiredTagID) == null) {
                frontLeft.setPower(0.3);
                frontRight.setPower(0.3);
                backLeft.setPower(-0.3);
                backRight.setPower(-0.3);
            }
            targetFound = true;
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        } else if (desiredTagID == 3) {
            while (objDetectionTask.getAprilTag(desiredTagID) == null) {
                frontLeft.setPower(-0.3);
                frontRight.setPower(-0.3);
                backLeft.setPower(0.3);
                backRight.setPower(0.3);
            }
            targetFound = true;
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }

        return objDetectionTask.getAprilTag(desiredTagID);
    }

//    public void AlignWithAprilTag(AprilTagDetection tag) {
//        double drive = 0;
//        double strafe = 0;
//        double turn = 0;
//        if (objDetectionTask.getAprilTag(desiredTagID) != null) {
//
//            double rangeError = (tag.ftcPose.range - DESIRED_DISTANCE);
//            double headingError = tag.ftcPose.bearing;
//            double yawError = tag.ftcPose.yaw;
//
//            drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
//            turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
//            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
//
//            if (rangeError < 0.05 && headingError < 0.05 && yawError < 0.05) {
//                targetReached = true;
//            }
//        }
//        // Apply desired axes motions to the drivetrain.
//        moveRobot(drive, strafe, turn);
//        sleep(10);
//    }

//    public void AlignWhileTargetNotReach()
//    {
//        AlignWithAprilTag(aprilTag);
//        while (!targetReached) {
//            AlignWithAprilTag(aprilTag);
//        }
//    }
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        frontLeft.setPower(leftFrontPower);
        frontRight.setPower(rightFrontPower);
        backLeft.setPower(leftBackPower);
        backRight.setPower(rightBackPower);

    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

=======
        addTask(objDetectionTask);
    }
    public void findDesireID()
    {
        if(position.equals("right")){
            desireTagID = 3;
        }
        else if(position.equals("left")){
            desireTagID = 1;
        }
        else
        {
            desireTagID = 2;
        }

    }
>>>>>>> 9edf1438b6dd5f5170907702f469b183e45ae052
    @Override
    public void init(){
        //initializes the motors for the wheels
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
    }
    @Override
    public void start(){
        findDesiredID();
        //desiredTagID = 1;
        findAprilTag();
        aprilTag = findAprilTagData();
        //AlignWhileTargetNotReach();

    }
}