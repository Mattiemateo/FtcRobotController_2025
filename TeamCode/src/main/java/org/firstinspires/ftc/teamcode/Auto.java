package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous
public class Auto extends LinearOpMode {
    private Servo claw;
    private CRServo arm_extend;
    private DcMotor liftR;
    private DcMotor liftL;
    private DcMotor arm_rot;
    private double clawPos = 0;
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;
    private int targetLiftPos = 0;
    private int minLift = 0;
    private BNO055IMU imu;


    @Override
    public void runOpMode() throws InterruptedException {
        // Hardware initialization
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");

        claw = hardwareMap.get(Servo.class, "claw");
        arm_extend = hardwareMap.get(CRServo.class, "armext");
        arm_rot = hardwareMap.get(DcMotor.class, "armrot");

        liftR = hardwareMap.get(DcMotor.class, "liftR");
        liftL = hardwareMap.get(DcMotor.class, "liftL");

        // Drive motor setup
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // Lift motor setup
        liftL.setDirection(DcMotor.Direction.REVERSE); // So both go "up" with + power
        liftR.setDirection(DcMotor.Direction.REVERSE);

        // Reset encoders
        liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftL.setTargetPosition(0);
        liftR.setTargetPosition(0);

        // Use encoder and position control
        liftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // Start IMU telemetry thread
        Thread imuTelemetryThread = new Thread(() -> {
            while (opModeIsActive()) {
                telemetry.addData("IMU Heading", getHeading());
                telemetry.addLine();

                int liftPosR = liftR.getCurrentPosition();
                int liftPosL = liftL.getCurrentPosition();
                int armRot = arm_rot.getCurrentPosition();

                telemetry.addData("Status", "Waiting for input");
                telemetry.addData("lx1", gamepad1.left_stick_x);
                telemetry.addData("ly1", gamepad1.left_stick_y);
                telemetry.addData("rx1", gamepad1.right_stick_x);
                telemetry.addLine();

                telemetry.addData("lx2", gamepad2.left_stick_x);
                telemetry.addData("ry2", gamepad2.right_stick_y);
                telemetry.addData("rx2", gamepad2.right_stick_x);
                telemetry.addLine();

                telemetry.addData("Target Pos", targetLiftPos);
                telemetry.addData("Lift Position R", liftPosR);
                telemetry.addData("Lift Position L", liftPosL);
                telemetry.addLine();

                telemetry.addData("armRot", armRot);
                telemetry.addData("clawPos", clawPos);

                telemetry.update();


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
        arm_rot.setTargetPosition(pos);
        arm_rot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm_rot.setPower(power);
        while (opModeIsActive() && arm_rot.isBusy()) {
            telemetry.addData("Arm Target", pos);
            telemetry.addData("Arm Current", arm_rot.getCurrentPosition());
            telemetry.update();
        }
        arm_rot.setPower(0);
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
