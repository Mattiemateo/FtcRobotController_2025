package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous
public class Auto extends LinearOpMode {

    private DcMotor arm, wrist;
    private Servo claw, intake;
    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    private BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        // Hardware initialization
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
        arm = hardwareMap.get(DcMotor.class, "arm");
        wrist = hardwareMap.get(DcMotor.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");
        intake = hardwareMap.get(Servo.class, "intake");

        // Set drive directions
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // IMU setup
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters imuParams = new BNO055IMU.Parameters();
        imuParams.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParams.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParams.loggingEnabled = false;
        imu.initialize(imuParams);

        // Encoder modes
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // Start IMU telemetry thread
        Thread imuTelemetryThread = new Thread(() -> {
            while (opModeIsActive()) {
                telemetry.addData("IMU Heading", getHeading());
                telemetry.update();
                sleep(100);
            }
        });
        imuTelemetryThread.start();

        // === AUTONOMOUS ROUTINE ===
        if (opModeIsActive()) {
            // Move arm to 1000
            moveArmToPosition(1000, 0.5);
            sleep(1000);

            // Move arm to 2000
            moveArmToPosition(2000, 0.5);
            sleep(1000);

            // Claw half open
            claw.setPosition(0.5);
            sleep(500);

            // Turn to 90 degrees
            turnToAngle(90, 0.25);
            sleep(500);

            // Drive forward 1 second
            driveForwardTime(1000, 0.4);
        }
    }

    // === HELPERS ===

    private void moveArmToPosition(int pos, double power) {
        arm.setTargetPosition(pos);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(power);
        while (opModeIsActive() && arm.isBusy()) {
            telemetry.addData("Arm Target", pos);
            telemetry.addData("Arm Current", arm.getCurrentPosition());
            telemetry.update();
        }
        arm.setPower(0);
    }

    private void driveForwardTime(int ms, double power) {
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);
        sleep(ms);
        stopDrive();
    }

    private void stopDrive() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    private double getHeading() {
        Orientation angles = imu.getAngularOrientation();
        return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
    }

    private void turnToAngle(double targetAngle, double power) {
        double heading = getHeading();
        while (opModeIsActive() && Math.abs(angleDifference(targetAngle, heading)) > 2) {
            double direction = angleDifference(targetAngle, heading) > 0 ? 1 : -1;

            leftFrontDrive.setPower(power * direction);
            rightFrontDrive.setPower(-power * direction);
            leftBackDrive.setPower(power * direction);
            rightBackDrive.setPower(-power * direction);

            heading = getHeading();
        }

        stopDrive();
    }

    private double angleDifference(double target, double current) {
        double diff = target - current;
        while (diff > 180) diff -= 360;
        while (diff < -180) diff += 360;
        return diff;
    }
}
