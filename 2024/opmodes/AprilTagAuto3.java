package opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import team25core.DeadReckonTask;
import team25core.FourWheelDirectDrivetrain;
import team25core.ObjectDetectionNewTask;
import team25core.Robot;
import team25core.RobotEvent;

@Autonomous(name = "AprilTagAuto3")
public class AprilTagAuto3 extends Robot {
    private ObjectDetectionNewTask objDetectionTask;
    private final static String TAG = "Prop";
    final double DESIRED_DISTANCE = 2.0; //  this is how close the camera should get to the target (inches)
    //  Set the GAIN constants to control the relationship between the measured position error,
    //  and how much power is applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger
    //  for a more aggressive response.
    final double SPEED_GAIN = 0.02;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
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

    private int desiredTagID=3;     // Choose the tag you want to approach or set to -1 for ANY tag.

    private final float APRIL_TAG_DECIMATION = 3;

    private final int EXPOSURE_MS = 6;
    private final int GAIN = 250;

    public String position; // this will contain the actual prop position information in final auto

    public AprilTagDetection aprilTag;
    boolean targetFound = false;
    boolean targetReached = false;

    private int numCalls = 0;
    private Telemetry.Item whereAmI;
    private Telemetry.Item timesCalled;


    private AprilTagDetection foundAprilTag;

    private int foundAprilTagId;

    private final double MOTOR_SPEED = 0.25;
    private final int LEFT = -1;
    private final int RIGHT = 1;
    private final int BACKWARDS = -1;
    private final int FORWARDS = 1;


    @Override
    public void handleEvent(RobotEvent e) {
        /*
         * Every time we complete a segment drop a note in the robot log.
         */
        //if (e instanceof DeadReckonTask.DeadReckonEvent) {
        //  RobotLog.i("Completed path segment %d", ((DeadReckonTask.DeadReckonEvent)e).segment_num);
        //}
    }

    public void findAprilTag() {
        RobotLog.ii(TAG, "Setup findAprilTag");
        objDetectionTask = new ObjectDetectionNewTask(this, telemetry, ObjectDetectionNewTask.DetectionKind.APRILTAG_DETECTED) {
            @Override
            public void handleEvent(RobotEvent e) {
                TagDetectionEvent event = (TagDetectionEvent) e;
                switch (event.kind) {
                    case APRIL_TAG_DETECTED:
                        RobotLog.ii(TAG, "AprilTag detected");
                        // AprilTag information (x, y, z, pose numbers)
                        foundAprilTag = event.aprilTag;
                        // AprilTag ID number
                        foundAprilTagId = foundAprilTag.id;
                        numCalls += 1;
                        whereAmI.setValue("handleEven");
                        timesCalled.setValue(numCalls);
                        alignWithAprilTag(foundAprilTag);
                        break;
                }
            }
        };
        // initializes the AprilTag processor
        objDetectionTask.init(telemetry, hardwareMap);
        // FIXME make sure this is the rate we want to use
        // objectDetection only has 1000 ms to run its process before another task has its turn in timeslice
        objDetectionTask.rateLimit(250); // currently calling objDetectionTask every second
        objDetectionTask.start(); // instantiates the rate limiting timer
        //start processing images; this uses a lot of computational resources so
        // stop streaming if you aren't using AprilTag detection
        objDetectionTask.resumeStreaming();
        // we are using Logitech C9-20
        //Adjust Image Detection to trade-off detection range
        objDetectionTask.setAprilTagDecimation(APRIL_TAG_DECIMATION);
        objDetectionTask.doManualExposure(EXPOSURE_MS, GAIN); // Use low exposure time to reduce motion blur
        //
        objDetectionTask.setDesiredTagID(desiredTagID);
        // starts running the task in the timeslice
        addTask(objDetectionTask);
    }

//
    // find desired id for blue alliance (1, 2, or 3)
    public void findDesiredID() {
        if (position.equals("left")) {
            desiredTagID = 1; // 4 on red
        } else if (position.equals("center")) {
            desiredTagID = 2; // 5 on red
        } else {
            desiredTagID = 3; // 6 on red
        }
        findAprilTagData();
    }

    public AprilTagDetection findAprilTagData() {
        if (desiredTagID == 1) {
            while (objDetectionTask.getAprilTag(desiredTagID) == null) {
                telemetry.addData("inside findAprilTagData looking for ID ", desiredTagID);

                frontLeft.setPower(0.3);
                frontRight.setPower(0.3);
                backLeft.setPower(-0.3);
                backRight.setPower(-0.3);
            }
            telemetry.addData("inside findAprilTagData found ID ", desiredTagID);
            targetFound = true;
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        } else if (desiredTagID == 3) {
            while (objDetectionTask.getAprilTag(desiredTagID) == null) {

                telemetry.addData("inside findAprilTagData looking for ID ", desiredTagID);

                frontLeft.setPower(-0.3);
                frontRight.setPower(-0.3);
                backLeft.setPower(0.3);
                backRight.setPower(0.3);
            }
            telemetry.addData("inside findAprilTagData found ID ", desiredTagID);
            targetFound = true;
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        } else {
            telemetry.addData("inside findAprilTagData looking for ID ", desiredTagID);
        }
        // FIXME later do the assignment of the AprilTag detection in
        //  the while loop to reduce redundancy
        return objDetectionTask.getAprilTag(desiredTagID);
    }

    public void alignWithAprilTag(AprilTagDetection tag) {
        double drive = 0;
        double strafe = 0;
        double turn = 0;

        AprilTagDetection myTagDetection;

        double rangeError = (tag.ftcPose.range - DESIRED_DISTANCE);
        double headingError = tag.ftcPose.bearing;
        double yawError = tag.ftcPose.yaw;

        drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
        strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

        telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);

//        if (rangeError < 0.05 && headingError < 0.05 && yawError < 0.05) {
//            targetReached = true;
//        } // FIXME print rangeError, headingError, and yawErrer

        telemetry.update();
        // Apply desired axes motions to the drivetrain.
        // FIXME just commented out to run code please uncomment
        moveRobot(drive, strafe, turn);
        //sleep(10);

    }


    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x +y +yaw;
        double rightFrontPower   =  x -y -yaw;
        double leftBackPower     =  x -y +yaw;
        double rightBackPower    =  x +y -yaw;

        // Normalize wheel powers to be less tha=n 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        telemetry.addData("Auto", "leftFrontPower %5.2f, leftBackPower %5.2f, rightBackPower %5.2f, rightFrontPower %5.2f", leftFrontPower, leftBackPower, rightBackPower, rightFrontPower);

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }
        telemetry.update();
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

    @Override
    public void init(){
        //initializes the motors for the wheels
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        whereAmI = telemetry.addData("whereami", "init");
        timesCalled = telemetry.addData("num calls", numCalls);

    }

    public void strafe(int direction) {
        moveRobot(0,MOTOR_SPEED * direction,  0);
    }

    public void drive(int direction) {
        moveRobot(MOTOR_SPEED * direction, 0, 0);

    }

    @Override
    public void start(){
        //findDesiredID();
        desiredTagID = 3;
        findAprilTag();

        //strafed to the right
        //moveRobot(0, 0.25, 0);
        strafe(LEFT);

        // go forward
        //moveRobot(0.25, 0, 0);
        //drive(FORWARDS);

        //FIXME
//        aprilTag = findAprilTagData();
//        alignWithAprilTag(aprilTag);
////        while (!targetReached) {
////         alignWithAprilTag(aprilTag);
////        }
//        frontLeft.setPower(0);
//        frontRight.setPower(0);
//        backLeft.setPower(0);
//        backRight.setPower(0);
    }
}
