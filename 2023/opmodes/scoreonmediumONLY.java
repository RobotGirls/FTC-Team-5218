/*
Copyright (c) September 2017 FTC Teams 25/5218

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FTC Teams 25/5218 nor the names of their contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.apriltag.AprilTagDetection;

import team25core.DeadReckonPath;
import team25core.DeadReckonTask;
import team25core.DeadReckonTaskWithIMU;
import team25core.FourWheelDirectIMUDrivetrain;
import team25core.OneWheelDirectDrivetrain;
import team25core.Robot;
import team25core.RobotEvent;
import team25core.RunToEncoderValueTask;
import team25core.sensors.color.RGBColorSensorTask;
import team25core.vision.apriltags.AprilTagDetectionTask;

@Config
@Autonomous(name = "leftscoreonmediumONLY")
//@Disabled
public class scoreonmediumONLY extends Robot {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private FourWheelDirectIMUDrivetrain drivetrain;

    private OneWheelDirectDrivetrain liftMotorDrivetrain;
    private DcMotor liftMotor;
    private boolean goingToFirstJunction = true;

    private boolean goingToHighJunction = true;

    private ColorSensor groundColorSensor;
    private Servo coneServo;
    private Servo armServo;

    public static final double CONE_GRAB = 0.12;
    public static final double CONE_RELEASE = 1.00;

    private static final double ARM_FRONT = .875;
    private static final double ARM_BACK = 0.0918;

    public static double STRAFE_FROM_MEDIUM_JUNCTION = 6;

    public static double STRAIGHT_TO_CONE_STACK = 4;

    public static double DRIVE_FROM_MEDIUM_JUNCTION = 2;

    public static double DRIVE_CLOSER_TO_CONE_STACK = 9;
    public static int LOWER_LIFT_TO_CONE_STACK = 700;

    public static int DRIVE_FROM_CONE_STACK = 10;

    public static int TURN_TO_LOW_JUNCTION2 = 27;

    public static int RAISE_LIFT_ENC_COUNTS = 400;



    //public static int TURN = 10;

    public static int STRAIGHT_TO_LOW_JUNCTION2 = 3;

    public static int BACK_FROM_LOW_JUNCTION2 = 2;





    private DeadReckonPath leftPath;
    private DeadReckonPath middlePath;
    private DeadReckonPath rightPath;

    private DeadReckonPath coneStrafePath;
    private DeadReckonPath coneStackLiftPath;
    private DeadReckonPath coneStrafeAwayPath;



    public static int SIGNAL_LEFT = 5;
    public static int SIGNAL_MIDDLE = 2;
    public static int SIGNAL_RIGHT = 18;
    public static double FORWARD_DISTANCE = 6;
    public static double DRIVE_SPEED = -0.5;

    public static int REV_40_TO_1_COUNTS_PER_REV = 1120;
    public static int REV_20_TO_1_COUNTS_PER_REV = 560;

    //public static int LOWER_LIFT_ENC_COUNTS = 2 * REV_40_TO_1_COUNTS_PER_REV;
    public static int LOWER_LIFT_ENC_COUNTS = REV_20_TO_1_COUNTS_PER_REV;

    public static double  CONE_STACK_SIDEWAYS_1 = 7;
    public static double CONE_STACK_STRAIGHT_FORWARD_1 = 10;
    public static double CONE_STACK_STRAIGHT_BACK_1 = 2.5;
    public static double CONE_STACK_STRAIGHT_FORWARD_2 = 28;
    public static double CONE_STACK_STRAIGHT_BACK_2 = 2.3;
    public static double CONE_STACK_STRAIGHT_FWD_SPEED_2 = 0.8;

    public static double TARGET_YAW_FOR_NEW_DIRECTION = -90;


    DeadReckonPath driveToMedium1Path;

    DeadReckonPath backUpToMediumJunctionPath;
    DeadReckonPath driveFromLow1Path;
    DeadReckonPath liftToLowJunctionPath;
    DeadReckonPath lowerLiftToLowJunctionPath;
    DeadReckonPath raiseLiftOffLowJunctionPath;
    DeadReckonPath driveFromMediumJunctionPath;
    DeadReckonPath driveFromLowJunctionPath;
    DeadReckonPath lowerLiftToHighJunctionPath;

    DeadReckonPath coneStackPath;
    DeadReckonPath raiseLiftOffConeStackPath1;
    DeadReckonPath raiseLiftOffConeStackPath2;

    DeadReckonPath driveFromConeStackPath;
    boolean firstConeLift = true;
    DeadReckonPath driveToHighJunctionPath;
    DeadReckonPath driveFromHighJunctionPath;
    DeadReckonPath raiseLiftToHighJunctionPath;

    DeadReckonPath lowerLiftBeforeConeStackPath;
    DeadReckonPath coneStackCloserPath;

    DeadReckonPath coneStackToJunctionPath;

    DeadReckonPath colorDetectionStrafePath;


    private BNO055IMU imu;
    private DeadReckonTaskWithIMU gyroTask;
    private Telemetry.Item headingTlm;

    private Telemetry.Item tagIdTlm;

    private int detectedAprilTagID;

    protected RGBColorSensorTask colorSensorTask;

    AprilTagDetection tagObject;
    private AprilTagDetectionTask detectionTask;
    //private double tagID;
    private Telemetry.Item colorDetectedTlm;
    private Telemetry.Item redDetectedTlm;
    private Telemetry.Item greenDetectedTlm;
    private Telemetry.Item blueDetectedTlm;
    private Telemetry.Item whereAmI;

    // MultipleTelemetry telemetry;

    private static final double TARGET_YAW_FOR_DRIVING_STRAIGHT = 0.0;
    private boolean showHeading = true;

    DeadReckonTask deadReckonTask;

    /*
     * The default event handler for the robot.
     */

    @Override
    public void handleEvent(RobotEvent e) {
        /*
         * Every time we complete a segment drop a note in the robot log.
         */
        if (e instanceof DeadReckonTask.DeadReckonEvent) {
            RobotLog.i("Completed path segment %d", ((DeadReckonTask.DeadReckonEvent) e).segment_num);
        }
    }


    public void setAprilTagDetection() {
        whereAmI.setValue("before detectionTask");
        detectionTask = new AprilTagDetectionTask(this, "Webcam 1") {
            @Override
            public void handleEvent(RobotEvent e) {
                TagDetectionEvent event = (TagDetectionEvent) e;
                tagObject = event.tagObject;
                tagIdTlm.setValue(tagObject.id);
                whereAmI.setValue("in handleEvent");
                detectedAprilTagID = tagObject.id;
            }
        };
        whereAmI.setValue("setAprilTagDetection");
        detectionTask.init(telemetry, hardwareMap);
    }


    private void decideWhichSignalWasSeen() {
        if (detectedAprilTagID == SIGNAL_LEFT) {
            driveToSignalZone(leftPath);
        } else if (detectedAprilTagID == SIGNAL_MIDDLE) {
            driveToSignalZone(middlePath);
        } else {
            driveToSignalZone(rightPath);
        }
    }


    private void driveToMediumJunction(DeadReckonPath driveToMedium1Path) {
        whereAmI.setValue("in driveToFromMediumJunction");
        RobotLog.i("drives to First Medium Junction");
        gyroTask = new DeadReckonTaskWithIMU(this, driveToMedium1Path, drivetrain)
                //this.addTask(new DeadReckonTask(this, driveToMedium1Path, drivetrain)
        {
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE) {
                    RobotLog.i("finished parking");
                    if (goingToFirstJunction) {
                        raiseLiftToMediumJunction();
                        goingToFirstJunction = false;
                    } else {
                        // do nothing :)
                        decideWhichSignalWasSeen();
                    }
                }
            }
        };
//        if (goingToLowerJunction) {
            gyroTask.initializeImu(imu, (double) TARGET_YAW_FOR_DRIVING_STRAIGHT, showHeading, headingTlm);
            gyroTask.initTelemetry(this.telemetry);
            addTask(gyroTask);
//        }
    }



    public void raiseLiftToMediumJunction() {
        whereAmI.setValue("in raiseLiftToMediumJunction");

        this.addTask(new RunToEncoderValueTask(this, liftMotor, 2650 , 1) {
            @Override
            public void handleEvent(RobotEvent e) {
                RunToEncoderValueEvent evt = (RunToEncoderValueEvent)e;
                if (evt.kind == EventKind.DONE) {
                    RobotLog.i("raiseLiftToMediumJunction");
                    armServo.setPosition(ARM_BACK);
                    //coneServo.setPosition(CONE_RELEASE);
                    backUpToMediumJunction(backUpToMediumJunctionPath);

                    //armServo.setPosition(ARM_FRONT);
                }
            }
        });
    }

    private void backUpToMediumJunction (DeadReckonPath backUpToMediumJunctionPath) {
        whereAmI.setValue("in backUpToMediumJunction");
        RobotLog.i("back up to medium junction");
        gyroTask = new DeadReckonTaskWithIMU(this, backUpToMediumJunctionPath, drivetrain)
                //this.addTask(new DeadReckonTask(this, backUpToMediumJunctionPath, drivetrain)
        {
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE) {
                    RobotLog.i("finished parking");
                        lowerLiftToFirstMediumJunction();
                }
            }
        };
//        if (goingToLowerJunction) {
        gyroTask.initializeImu(imu, (double) TARGET_YAW_FOR_DRIVING_STRAIGHT, showHeading, headingTlm);
        gyroTask.initTelemetry(this.telemetry);
        addTask(gyroTask);
//        }
    }

    public void lowerLiftToFirstMediumJunction() {
        whereAmI.setValue("in lowerLiftToFirstMediumJunction");

        this.addTask(new RunToEncoderValueTask(this, liftMotor, 850 , -1) {
            @Override
            public void handleEvent(RobotEvent e) {
                RunToEncoderValueEvent evt = (RunToEncoderValueEvent)e;
                if (evt.kind == EventKind.DONE) {
                    RobotLog.i("lowerLiftToFirstMediumJunction");
                    coneServo.setPosition(CONE_RELEASE);
                    raiseLiftOffFirstMediumJunction();

                    //armServo.setPosition(ARM_FRONT);
                }
            }
        });
    }



    public void raiseLiftOffFirstMediumJunction() {
        whereAmI.setValue("in raiseLiftOffFirstMediumJunction");

        this.addTask(new RunToEncoderValueTask(this, liftMotor, 600 , 1) {
            @Override
            public void handleEvent(RobotEvent e) {
                RunToEncoderValueEvent evt = (RunToEncoderValueEvent)e;
                if (evt.kind == EventKind.DONE) {
                    RobotLog.i("raiseLiftOffFirstMediumJunction");
                   // driveToFromFirstLowJunction(driveFromLow1Path);
                   driveFromMediumJunction(driveFromMediumJunctionPath);
                }
            }
        });
    }


    public void lowerLiftBeforeConeStack() {
            whereAmI.setValue("in lowerLiftBeforeConeStack");

            this.addTask(new RunToEncoderValueTask(this, liftMotor, 1000 , -1) {
                @Override
                public void handleEvent(RobotEvent e) {
                    RunToEncoderValueEvent evt = (RunToEncoderValueEvent)e;
                    if (evt.kind == EventKind.DONE) {
                        RobotLog.i("lowered lift before cone stack");
                     //   decideWhichSignalWasSeen();
                        //armServo.setPosition(ARM_FRONT);
                  }
                }
            });
    }
    public void driveFromMediumJunction() {
        whereAmI.setValue("driveFromMediumJunction");

        gyroTask = new DeadReckonTaskWithIMU(this, driveFromMediumJunctionPath, drivetrain) {
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                whereAmI.setValue("driveFromMediumJunction handleEvent");

                // when path done we are approximatly a foot away from the cone stack
                if (path.kind == EventKind.PATH_DONE) {
                    whereAmI.setValue("driveFromMediumJunction PATH_DONE");

                    // as we're driving closer to the cone stack we're simultaneously lifting the lift
                    // drivetrain.setTargetYaw(TARGET_YAW_FOR_NEW_DIRECTION);
                    //  Stack(raiseLiftOffConeStackPath1);raiseLiftOffCone

                        decideWhichSignalWasSeen();
//armServo.setPosition(ARM_FRONT);
                    // driveCloserToConeStack(coneStackCloserPath);

                }
            }
        };
        gyroTask.initializeImu(imu,(double)TARGET_YAW_FOR_DRIVING_STRAIGHT, showHeading, headingTlm); //TARGET_YAW_FOR_NEW_DIRECTION testing this on Friday
        gyroTask.initTelemetry(this.telemetry);
        //gyroTask.useSmoothStart(true);
        addTask(gyroTask);
    }



    public void driveToConeStack(DeadReckonPath coneStackPath) {
        whereAmI.setValue("driveToConeStack");
        RobotLog.i("drives to cone stack line");

        gyroTask = new DeadReckonTaskWithIMU(this, coneStackPath, drivetrain) {
            @Override
            public void handleEvent(RobotEvent e) {
                whereAmI.setValue("driveToConeStack handleEvent");

                DeadReckonEvent path = (DeadReckonEvent) e;
                // when path done we are approximatly a foot away from the cone stack
                if (path.kind == EventKind.PATH_DONE) {
                    whereAmI.setValue("driveToConeStack PATH_DONE");

                    // as we're driving closer to the cone stack we're simultaneously lifting the lift
                    // drivetrain.setTargetYaw(TARGET_YAW_FOR_NEW_DIRECTION);
                  //  Stack(raiseLiftOffConeStackPath1);raiseLiftOffCone
                    // strafeFromMediumJunction();
                   // colorDetectionStrafe();
                    //armServo.setPosition(ARM_FRONT);
                   // driveCloserToConeStack(coneStackCloserPath);

                }
            }
        };
        gyroTask.initializeImu(imu,(double)TARGET_YAW_FOR_DRIVING_STRAIGHT, showHeading, headingTlm); //TARGET_YAW_FOR_NEW_DIRECTION testing this on Friday
        gyroTask.initTelemetry(this.telemetry);
        //gyroTask.useSmoothStart(true);
        addTask(gyroTask);
    }


    public void driveCloserToConeStack(DeadReckonPath coneStackCloserPath) {
        whereAmI.setValue("driveCloserToConeStack");
        RobotLog.i("drives to First Low Junction");

      //  this.addTask( new DeadReckonTask(this, coneStackCloserPath, drivetrain){
        gyroTask = new DeadReckonTaskWithIMU(this, coneStackCloserPath, drivetrain) {
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                whereAmI.setValue("driveCloserToConeStack handleEvent");
                // when the path is done we have completed the driveCloserToConeStack
                if (path.kind == EventKind.PATH_DONE) {
                    whereAmI.setValue("driveCloserToConeStack");
                    lowerLiftToGrabConeOnStack();
//                    coneServo.setPosition(CONE_GRAB);
                  //  raiseLiftOffConeStack(raiseLiftOffConeStackPath2);
                  //do nothing here
                }
            }
        };
        gyroTask.initializeImu(imu, (double) TARGET_YAW_FOR_DRIVING_STRAIGHT, showHeading, headingTlm);
        gyroTask.initTelemetry(this.telemetry);
      //  gyroTask.useSmoothStart(true);
        addTask(gyroTask);
    }


    public void lowerLiftToGrabConeOnStack() {
        whereAmI.setValue("in lowerLiftToGrabConeOnStack");

        this.addTask(new RunToEncoderValueTask(this, liftMotor, LOWER_LIFT_TO_CONE_STACK, -0.7) {
            @Override
            public void handleEvent(RobotEvent e) {
                RunToEncoderValueEvent evt = (RunToEncoderValueEvent)e;
                whereAmI.setValue("lowerLiftToGrabConeOnStack handleEvent");
                if (evt.kind == EventKind.DONE) {
                    RobotLog.i("liftedToLowJunction");
                    whereAmI.setValue("lowerLiftToGrabConeOnStack DONE");

                    // MADDIEFIXME in this current code we're grabbing the cone and raising the
                    // lift at the same time. There is a possibility that we may be raising the lift
                    // before the claw can fully extend. In that case we may have to put a delay
                    // before the lift so we may need to delay.
                    coneServo.setPosition(CONE_GRAB);
                    raiseLiftOffConeStack(raiseLiftOffConeStackPath2);
                   // decideWhichSignalWasSeen();
                    //armServo.setPosition(ARM_FRONT);
                }
            }
        });
    }



    public void colorDetectionStrafe() {
        whereAmI.setValue("colorDetectionStrafe");
        handleColorSensor();
        gyroTask = new DeadReckonTaskWithIMU(this, colorDetectionStrafePath, drivetrain) {
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                // when the path is done we have completed the driveCloserToConeStack
                if (path.kind == EventKind.SENSOR_SATISFIED) { //fixme change to path done if it doesn't work
                    // FIXME add driveCloserToConeStack
                    //coneServo.setPosition(CONE_GRAB);
                  //  raiseLiftOffConeStack(raiseLiftOffConeStackPath1);
                 //   driveCloserToConeStack(coneStackCloserPath);
                }
            }
        };
        gyroTask.initializeImu(imu, (double) TARGET_YAW_FOR_DRIVING_STRAIGHT, showHeading, headingTlm);
        // FIXME Add the initTlm back in
//        gyroTask.initTelemetry(this.telemetry);
        //gyroTask.useSmoothStart(true);
        addTask(gyroTask);
    }

    public void handleColorSensor () {
        whereAmI.setValue("handleColorSensor");
        colorSensorTask = new RGBColorSensorTask(this, groundColorSensor) {
            public void handleEvent(RobotEvent e) {
                whereAmI.setValue("handleColorSensor handleEvent");
                ColorSensorEvent event = (ColorSensorEvent) e;
                colorArray = colorSensorTask.getColors();
                blueDetectedTlm.setValue(colorArray[0]);
                redDetectedTlm.setValue(colorArray[1]);
                greenDetectedTlm.setValue(colorArray[2]);
                switch(event.kind) {
                    // red is at the end
                    case RED_DETECTED:
                        drivetrain.stop();
                        robot.removeTask(colorSensorTask);
                        gyroTask.resume();
                      // raiseLiftOffConeStack(raiseLiftOffConeStackPath1);
                        driveCloserToConeStack(coneStackCloserPath);
                        colorDetectedTlm.setValue("red");
                        break;
                    case BLUE_DETECTED:
                        drivetrain.stop();
                        robot.removeTask(colorSensorTask);
                        gyroTask.resume();
                        driveCloserToConeStack(coneStackCloserPath);
                        colorDetectedTlm.setValue("blue");
                        break;
                    default:
                        colorDetectedTlm.setValue("none");
                        break;
                }
            }
        };
        colorSensorTask.setThresholds(10000, 10000, 5000);
        colorSensorTask.setDrivetrain(drivetrain);
        addTask(colorSensorTask);
    }

    public void raiseLiftOffConeStack(DeadReckonPath coneStackLiftPath) {
        whereAmI.setValue("in raiseLiftOffConeStack");
        this.addTask(new DeadReckonTask(this, coneStackLiftPath, liftMotorDrivetrain) {

            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                // when this path is done we just raised the lift so it doesn't collide with the cone stack
                if (path.kind == EventKind.PATH_DONE) {
                    if (firstConeLift) {
                        whereAmI.setValue("in raiseLiftOffConeStack firstConeLift");
                      //  driveCloserToConeStack(coneStackCloserPath);
                        // now lower the lift in order to grab the cone
                       //lowerLiftToGrabConeOnStack();
                       //coneServo.setPosition(CONE_GRAB);
                       // driveCloserToConeStack(coneStackCloserPath);
                       // driveCloserToConeStack();
                        // decideWhichSignalWasSeen();
                        armServo.setPosition(ARM_FRONT);
                        driveFromConeStackToLowJunction2(driveFromConeStackPath);
                        firstConeLift = false;
                    } else {
                        // at this point we have already lifted the cone off the cone stack
                        // MADDIEFIXME (we either lift the cone only high enough to clear the
                        // cone stack or high enough to clear the low junction we can change
                        // raiseLiftOffConeStackPath2)
//                        armServo.setPosition(ARM_FRONT);
//                        driveFromConeStackToLowJunction2(driveFromConeStackPath);
                        // now we'll move the arm to the front
                       // decideWhichSignalWasSeen();
                       // driveFromConeStackToJunction();
                    }


                }
            }
        });
    }

    public void lowerLiftToConeStackPath() {
        whereAmI.setValue("in lowerLiftToGrabConeOnStack");

        this.addTask(new RunToEncoderValueTask(this, liftMotor, LOWER_LIFT_ENC_COUNTS, -1) {
            @Override
            public void handleEvent(RobotEvent e) {
                RunToEncoderValueEvent evt = (RunToEncoderValueEvent)e;
                if (evt.kind == EventKind.DONE) {
                    RobotLog.i("loweredLiftTo2ndLowJunction");
                    coneServo.setPosition(CONE_RELEASE);
                    whereAmI.setValue("finished lowerLiftToConeStackPath");


                }
            }
        });
    }


    public void driveFromConeStackToLowJunction2(DeadReckonPath driveFromConeStackPath) {
        whereAmI.setValue("driveFromConeStackToLowJunction2");

        gyroTask = new DeadReckonTaskWithIMU(this,driveFromConeStackPath, drivetrain) {
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                // when the path is done we have completed the driveCloserToConeStack
                if (path.kind == EventKind.PATH_DONE) {
                   // raiseLiftOffConeStack(raiseLiftOffConeStackPath2);

                    strafeFromConeStack(coneStrafeAwayPath);

                    //do nothing here
                }
            }
        };
        gyroTask.initializeImu(imu, (double) TARGET_YAW_FOR_DRIVING_STRAIGHT, showHeading, headingTlm);
        gyroTask.initTelemetry(this.telemetry);
        //gyroTask.useSmoothStart(true);
        addTask(gyroTask);
    }

    public void strafeFromConeStack(DeadReckonPath coneStrafeAwayPath) {
        whereAmI.setValue("driveFromConeStackToLowJunction2");

        gyroTask = new DeadReckonTaskWithIMU(this, coneStrafeAwayPath, drivetrain) {
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                // when the path is done we have completed the driveCloserToConeStack
                if (path.kind == EventKind.PATH_DONE) {
                    // raiseLiftOffConeStack(raiseLiftOffConeStackPath2);

                    lowerLiftTo2ndLowJunctionPath();
                    //do nothing here
                }
            }
        };
        gyroTask.initializeImu(imu, (double) TARGET_YAW_FOR_DRIVING_STRAIGHT, showHeading, headingTlm);
        gyroTask.initTelemetry(this.telemetry);
        //gyroTask.useSmoothStart(true);
        addTask(gyroTask);
    }


    public void lowerLiftTo2ndLowJunctionPath() {
        whereAmI.setValue("in lowerLiftToGrabConeOnStack");

        this.addTask(new RunToEncoderValueTask(this, liftMotor, LOWER_LIFT_ENC_COUNTS, -0.20) {
            @Override
            public void handleEvent(RobotEvent e) {
                RunToEncoderValueEvent evt = (RunToEncoderValueEvent)e;
                if (evt.kind == EventKind.DONE) {
                    RobotLog.i("loweredLiftTo2ndLowJunction");
                    coneServo.setPosition(CONE_RELEASE);
                    whereAmI.setValue("finished lowerLiftToConeStackPath");
                    raiseLiftOffLowJunction2();
                  //  decideWhichSignalWasSeen();

                }
            }
        });
    }
    public void raiseLiftOffLowJunction2() {
        whereAmI.setValue("in raise lift off low junction");

        this.addTask(new RunToEncoderValueTask(this, liftMotor, RAISE_LIFT_ENC_COUNTS, 0.20) {
            @Override
            public void handleEvent(RobotEvent e) {
                RunToEncoderValueEvent evt = (RunToEncoderValueEvent)e;
                if (evt.kind == EventKind.DONE) {
                    RobotLog.i("raiseLiftOffLowJunction");
                    coneServo.setPosition(CONE_RELEASE);
                    whereAmI.setValue("finished raiseLiftOffLowJunction");
                    driveFromLowJunction2(driveFromLowJunctionPath);
                    //  decideWhichSignalWasSeen();

                }
            }
        });
    }

    public void driveFromLowJunction2(DeadReckonPath driveFromLowJunctionPath) {
        whereAmI.setValue("driveFromLowJunction");

        gyroTask = new DeadReckonTaskWithIMU(this,driveFromLowJunctionPath, drivetrain) {
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                // when the path is done we have completed the driveCloserToConeStack
                if (path.kind == EventKind.PATH_DONE) {
                    //decideWhichSignalWasSeen();
                    // raiseLiftOffConeStack(raiseLiftOffConeStackPath2);
                    //lowerLiftTo2ndLowJunctionPath();
                    //do nothing here
                }
            }
        };
        gyroTask.initializeImu(imu, (double) TARGET_YAW_FOR_DRIVING_STRAIGHT, showHeading, headingTlm);
        gyroTask.initTelemetry(this.telemetry);
        //gyroTask.useSmoothStart(true);
        addTask(gyroTask);
    }

    public void driveToSignalZone(DeadReckonPath signalPath) {
        whereAmI.setValue("in driveToSignalZone");
        RobotLog.i("drives straight onto the launch line");
        gyroTask = new DeadReckonTaskWithIMU(this, signalPath, drivetrain) {
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE) {
                    RobotLog.i("finished parking");
                }
            }
        };
        gyroTask.initializeImu(imu, (double) TARGET_YAW_FOR_DRIVING_STRAIGHT, showHeading, headingTlm);
        gyroTask.initTelemetry(this.telemetry);
        addTask(gyroTask);
    }

    public void initPaths() {
        driveToMedium1Path = new DeadReckonPath();
        backUpToMediumJunctionPath = new DeadReckonPath();
        driveFromLow1Path = new DeadReckonPath();
        liftToLowJunctionPath = new DeadReckonPath();
        lowerLiftToLowJunctionPath = new DeadReckonPath();
        raiseLiftOffLowJunctionPath = new DeadReckonPath();
        driveFromLowJunctionPath = new DeadReckonPath();

        coneStrafePath = new DeadReckonPath();

        coneStackPath = new DeadReckonPath();
        raiseLiftOffConeStackPath1 = new DeadReckonPath();
        raiseLiftOffConeStackPath2 = new DeadReckonPath();
        driveToHighJunctionPath = new DeadReckonPath();
        driveFromHighJunctionPath = new DeadReckonPath();
        raiseLiftToHighJunctionPath = new DeadReckonPath();
        lowerLiftToHighJunctionPath = new DeadReckonPath();
        driveFromMediumJunctionPath = new DeadReckonPath();

        lowerLiftBeforeConeStackPath = new DeadReckonPath();

        driveFromConeStackPath = new DeadReckonPath();

        leftPath = new DeadReckonPath();
        middlePath = new DeadReckonPath();
        rightPath = new DeadReckonPath();
        coneStackCloserPath = new DeadReckonPath();
        coneStackToJunctionPath = new DeadReckonPath();

        colorDetectionStrafePath = new DeadReckonPath();

        coneStackLiftPath = new DeadReckonPath();

        coneStrafeAwayPath = new DeadReckonPath();

        driveToMedium1Path.stop();
        backUpToMediumJunctionPath.stop();
        driveFromLow1Path.stop();
        liftToLowJunctionPath.stop();
        lowerLiftToLowJunctionPath.stop();
        raiseLiftOffLowJunctionPath.stop();
        driveFromLowJunctionPath.stop();

        coneStrafePath.stop();

        driveFromConeStackPath.stop();

        driveFromMediumJunctionPath.stop();
        raiseLiftOffConeStackPath1.stop();
        raiseLiftOffConeStackPath2.stop();
        driveToHighJunctionPath.stop();
        raiseLiftToHighJunctionPath.stop();
        lowerLiftToHighJunctionPath.stop();

        lowerLiftBeforeConeStackPath.stop();
        coneStackCloserPath.stop();
        coneStackToJunctionPath.stop();

        colorDetectionStrafePath.stop();

        coneStackLiftPath.stop();

        coneStrafeAwayPath.stop();

        leftPath.stop();
        middlePath.stop();
        rightPath.stop();
        coneStackPath.stop();


        // drive straight to medium junction
        driveToMedium1Path.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 21, 0.55);
        //turn to face medium junction
        driveToMedium1Path.addSegment(DeadReckonPath.SegmentType.TURN, 27.5, -0.45);

        // back up to medium junction to score
        backUpToMediumJunctionPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 1.5, -0.55);

        driveFromMediumJunctionPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 1.5, 0.55);


         // Stays in first parking spot
       leftPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS,2.5, 0.5);
       leftPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 12, .5);

        // return to initial then go forward
        middlePath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS,1, -0.5);
       // middlePath.addSegment(DeadReckonPath.SegmentType.STRAIGHT,10, -1);
        ;
        // return to initial then go forward then right
        // strafe to right
        rightPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS,10, 0.5);
        rightPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 11, -0.6);
     //   rightPath.addSegment(DeadReckonPath.SegmentType.TURN, 2, -0.75);


        //30
    }

    @Override
    public void init() {
        // cindy added
        super.init();
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");

        coneServo = hardwareMap.servo.get("coneServo");
        armServo = hardwareMap.servo.get("armServo");
        groundColorSensor = hardwareMap.colorSensor.get("groundColorSensor");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        drivetrain = new FourWheelDirectIMUDrivetrain(frontRight, backRight, frontLeft, backLeft);
        drivetrain.resetEncoders();
        drivetrain.encodersOn();
        // We are setting the target yaw to the value of the constant
        // TARGET_YAW_FOR_DRIVING_STRAIGHT which is zero for our robot to
        // go straight.
       // drivetrain.setTargetYaw(TARGET_YAW_FOR_DRIVING_STRAIGHT);

        liftMotorDrivetrain = new OneWheelDirectDrivetrain(liftMotor);
        liftMotorDrivetrain.resetEncoders();
        liftMotorDrivetrain.encodersOn();

        coneServo.setPosition(CONE_GRAB);
        armServo.setPosition(ARM_FRONT);

        // initialized heading telemetry
        headingTlm = telemetry.addData("Current/target heading is: ", "0.0");

        colorDetectedTlm = telemetry.addData("color detected", "unknown");

        blueDetectedTlm = telemetry.addData("blue color sensor value", 0);
        redDetectedTlm = telemetry.addData("red color sensor value", 0);
        greenDetectedTlm = telemetry.addData("green color sensor value", 0);

        whereAmI = telemetry.addData("location in code", "init");
        tagIdTlm = telemetry.addData("tagId", "none");
        initPaths();

    }

    @Override
    public void start() {

        setAprilTagDetection();
        addTask(detectionTask);
        driveToMediumJunction(driveToMedium1Path);
       // decideWhichSignalWasSeen();
        whereAmI.setValue("in Start");


        // liftToFirstLowJunction();
        // FIXME Start from Color detection strafe to test 2nd junction
       // colorDetectionStrafe();
    }
}
