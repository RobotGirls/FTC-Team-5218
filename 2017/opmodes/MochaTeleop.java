
package opmodes;

/*
 * FTC Team 5218: izzielau, October 30, 2016
 */

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;

import team25core.DeadmanMotorTask;
import team25core.FourWheelDirectDrivetrain;
import team25core.GamepadTask;
import team25core.LimitSwitchTask;
import team25core.PersistentTelemetryTask;
import team25core.Robot;
import team25core.RobotEvent;
import team25core.TankDriveTask;

@TeleOp(name="5218 Mocha", group = "5218")
public class MochaTeleop extends Robot {

    private final static double SHOOTER_Y = MochaCalibration.SHOOTER_Y;
    private final static double SHOOTER_B = MochaCalibration.SHOOTER_B;
    private final static double SHOOTER_A= MochaCalibration.SHOOTER_A;
    private final static double SHOOTER_X = MochaCalibration.SHOOTER_X;
    private final static double BRUSH_SPEED = MochaCalibration.BRUSH_SPEED;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor shooterLeft;
    private DcMotor shooterRight;
    private DcMotor sbod;
    private Servo beacon;
    private DeviceInterfaceModule interfaceModule;
    private LightSensor one;
    private LightSensor two;
    private FourWheelDirectDrivetrain drivetrain;

    private PersistentTelemetryTask ptt;

    @Override
    public void init()
    {
        // CDIM.
        interfaceModule = hardwareMap.deviceInterfaceModule.get("interface");

        // Light sensors.
        one = hardwareMap.lightSensor.get("lightLeft");
        one.enableLed(false);
        two = hardwareMap.lightSensor.get("lightRight");
        two.enableLed(false);

        // Drivetrain.
        frontRight = hardwareMap.dcMotor.get("motorFR");
        frontLeft = hardwareMap.dcMotor.get("motorFL");
        backRight = hardwareMap.dcMotor.get("motorBR");
        backLeft = hardwareMap.dcMotor.get("motorBL");

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        drivetrain = new FourWheelDirectDrivetrain(frontRight, backRight, frontLeft, backLeft);

        shooterLeft = hardwareMap.dcMotor.get("shooterLeft");
        shooterRight = hardwareMap.dcMotor.get("shooterRight");

        shooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Hook.
        sbod = hardwareMap.dcMotor.get("brush");

        // Servo.
        beacon = hardwareMap.servo.get("beacon");

    }

    @Override
    public void handleEvent(RobotEvent e) {

    }

    @Override
    public void start() {
        super.start();

        /* DRIVER ONE */
        // Four motor drive.
        final TankDriveTask drive = new TankDriveTask(this, drivetrain);
        this.addTask(drive);

        // SBOD
        DeadmanMotorTask collect = new DeadmanMotorTask(this, sbod, BRUSH_SPEED, GamepadTask.GamepadNumber.GAMEPAD_2, DeadmanMotorTask.DeadmanButton.RIGHT_BUMPER);
        addTask(collect);
        DeadmanMotorTask dispense = new DeadmanMotorTask(this, sbod, -BRUSH_SPEED, GamepadTask.GamepadNumber.GAMEPAD_2, DeadmanMotorTask.DeadmanButton.RIGHT_TRIGGER);
        addTask(dispense);

        DeadmanMotorTask collectSlow = new DeadmanMotorTask(this, sbod, 0.5 * BRUSH_SPEED, GamepadTask.GamepadNumber.GAMEPAD_2, DeadmanMotorTask.DeadmanButton.LEFT_BUMPER);
        addTask(collectSlow);
        DeadmanMotorTask dispenseSlow = new DeadmanMotorTask(this, sbod, -0.5 * BRUSH_SPEED, GamepadTask.GamepadNumber.GAMEPAD_2, DeadmanMotorTask.DeadmanButton.LEFT_TRIGGER);
        addTask(dispenseSlow);

        // Shooters
        DeadmanMotorTask shootLeftY = new DeadmanMotorTask(this, shooterLeft, SHOOTER_Y, GamepadTask.GamepadNumber.GAMEPAD_2, DeadmanMotorTask.DeadmanButton.BUTTON_Y);
        addTask(shootLeftY);
        DeadmanMotorTask shootRightY = new DeadmanMotorTask(this, shooterRight, -SHOOTER_Y, GamepadTask.GamepadNumber.GAMEPAD_2, DeadmanMotorTask.DeadmanButton.BUTTON_Y);
        addTask(shootRightY);
        DeadmanMotorTask shootLeftB = new DeadmanMotorTask(this, shooterLeft, SHOOTER_B, GamepadTask.GamepadNumber.GAMEPAD_2, DeadmanMotorTask.DeadmanButton.BUTTON_B);
        addTask(shootLeftB);
        DeadmanMotorTask shootRightB = new DeadmanMotorTask(this, shooterRight, -SHOOTER_B, GamepadTask.GamepadNumber.GAMEPAD_2, DeadmanMotorTask.DeadmanButton.BUTTON_B);
        addTask(shootRightB);
        DeadmanMotorTask shootLeftA = new DeadmanMotorTask(this, shooterLeft, SHOOTER_A, GamepadTask.GamepadNumber.GAMEPAD_2, DeadmanMotorTask.DeadmanButton.BUTTON_A);
        addTask(shootLeftA);
        DeadmanMotorTask shootRightA = new DeadmanMotorTask(this, shooterRight, -SHOOTER_A, GamepadTask.GamepadNumber.GAMEPAD_2, DeadmanMotorTask.DeadmanButton.BUTTON_A);
        addTask(shootRightA);
        DeadmanMotorTask shootLeftX = new DeadmanMotorTask(this, shooterLeft, SHOOTER_X, GamepadTask.GamepadNumber.GAMEPAD_2, DeadmanMotorTask.DeadmanButton.BUTTON_X);
        addTask(shootLeftX);
        DeadmanMotorTask shootRightX = new DeadmanMotorTask(this, shooterRight, -SHOOTER_X, GamepadTask.GamepadNumber.GAMEPAD_2, DeadmanMotorTask.DeadmanButton.BUTTON_X);
        addTask(shootRightX);

        this.addTask(new GamepadTask(this, GamepadTask.GamepadNumber.GAMEPAD_1) {
            public void handleEvent(RobotEvent e) {
                GamepadEvent event = (GamepadEvent) e;

                if (event.kind == EventKind.LEFT_TRIGGER_DOWN) {
                    beacon.setPosition(1.0);
                } else if (event.kind == EventKind.LEFT_TRIGGER_UP) {
                    beacon.setPosition(0.5);
                } else if (event.kind == EventKind.LEFT_BUMPER_DOWN) {
                    beacon.setPosition(0);
                } else if (event.kind == EventKind.LEFT_BUMPER_UP) {
                    beacon.setPosition(0.5);
                } else if (event.kind == EventKind.BUTTON_B_DOWN) {
                    drive.slowDown(true);
                    drive.slowDown(0.35);
                } else if (event.kind == EventKind.BUTTON_A_DOWN) {
                    drive.slowDown(true);
                    drive.slowDown(1.0);
                }
            }
        });

        addTask(new LimitSwitchTask(this, interfaceModule, 0) {
            @Override
            public void handleEvent(RobotEvent e)
            {
                DistanceSensorEvent event = (DistanceSensorEvent)e;

                if (event.kind == EventKind.CLOSED) {
                    telemetry.addData("Status (1): ", "closed");
                } else if (event.kind == EventKind.OPEN) {
                    telemetry.addData("Status (1): ", "open");
                } else {
                    telemetry.addData("Status (1): ", "unknown");
                }
            }
        });

        addTask(new LimitSwitchTask(this, interfaceModule, 1) {
            @Override
            public void handleEvent(RobotEvent e)
            {
                DistanceSensorEvent event = (LimitSwitchEvent)e;

                if (event.kind == EventKind.CLOSED) {
                    telemetry.addData("Status (2): ", "closed");
                } else if (event.kind == EventKind.OPEN) {
                    telemetry.addData("Status (2): ", "open");
                } else {
                    telemetry.addData("Status (2): ", "unknown");
                }
            }
        });
    }
}
