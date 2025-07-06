package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous
public class AutoPrreload extends LinearOpMode {
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

    private IMU imu;

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

        // IMU setup for BHI260AP
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
            )
        );
        imu.initialize(imuParams);

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

                // Get yaw, pitch, roll from IMU
                YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
                double yaw = orientation.getYaw(AngleUnit.DEGREES);
                double pitch = orientation.getPitch(AngleUnit.DEGREES);
                double roll = orientation.getRoll(AngleUnit.DEGREES);

                telemetry.addData("Yaw (Z)", yaw);
                telemetry.addData("Pitch (Y)", pitch);
                telemetry.addData("Roll (X)", roll);
                telemetry.addLine();

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
                sleep(100);
            }
        });
        imuTelemetryThread.start();

        // === AUTONOMOUS ROUTINE ===
        if (opModeIsActive()) {
            driveXTime(1000, 1);
//            Thread base_movement = new Thread(() -> {
//                sleep(1000);
//                driveYTime(1000, -0.5);
//                sleep(1000);
//                turnToAngle(90, 1);
//                driveYTime(1000, -0.5);
//                sleep(1000);
//                driveYTime(650, -0.5);
//                sleep(3000);
//                driveYTime(950, 1);
//                sleep(950);
//                driveXTime(2000, -0.5);
//                turnToAngle(90, 1);
//                sleep(500);
//                driveYTime(1500, -0.5);
//                sleep(500);
//                driveXTime(700, -0.5);
//                driveYTime(1900,-0.6);
//                driveYTime(1900,-0.6);
//                driveXTime(700, -0.5);
//                driveYTime(1900,-0.6);
//                //turnToAngle(, -0.5);
//                driveYTime(1900,-0.6);
//                driveXTime(850, -0.5);
//                driveYTime(1900,-0.6);
//            });
//            base_movement.start();
//            base_movement.join();
        }
    }

    // === HELPERS ===
    private void driveYTime(int ms, double power) {
        double targetHeading = getHeading(); // desired direction in degrees
        long startTime = System.currentTimeMillis();

        while (opModeIsActive() && System.currentTimeMillis() - startTime < ms) {
            double currentHeading = getHeading();
            double error = angleDifference(targetHeading, currentHeading);

            // Proportional heading correction
            double kP = 0.015; // tune for your bot
            double correction = error * kP;

            // Flip correction if driving backwards
            double directionSign = Math.signum(power);

            double leftPower  = power + directionSign * correction;
            double rightPower = power - directionSign * correction;

            leftFrontDrive.setPower(leftPower);
            leftBackDrive.setPower(leftPower);
            rightFrontDrive.setPower(rightPower);
            rightBackDrive.setPower(rightPower);

            telemetry.addLine();
            telemetry.addData("Heading", currentHeading);
            telemetry.addData("Target", targetHeading);
            telemetry.addData("Error", error);
            telemetry.addData("Correction", correction);
            telemetry.update();
        }

        stopDrive();
    }

    private void driveXTime(int ms, double power) {
        long startTime = System.currentTimeMillis();

        while (opModeIsActive() && System.currentTimeMillis() - startTime < ms) {
            double x = power;     // strafe only
            double y = 0;         // no forward/backward
            double turn = 0;      // no rotation

            double theta = Math.atan2(y, x);
            double magnitude = Math.hypot(x, y);

            double sin = Math.sin(theta - Math.PI / 4);
            double cos = Math.cos(theta - Math.PI / 4);

            double leftFrontPower = magnitude * cos + turn;
            double rightFrontPower = magnitude * sin - turn;
            double leftBackPower = magnitude * sin + turn;
            double rightBackPower = magnitude * cos - turn;

            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            telemetry.addData("LF", leftFrontPower);
            telemetry.addData("RF", rightFrontPower);
            telemetry.addData("LB", leftBackPower);
            telemetry.addData("RB", rightBackPower);
            telemetry.update();
        }

        stopDrive();
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

    private void stopDrive() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    private double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    private double angleDifference(double target, double current) {
        double diff = target - current;
        while (diff > 180) diff -= 360;
        while (diff < -180) diff += 360;
        return diff;
    }

    private void moveClaw(double clawpos) {
        claw.setPosition(clawpos);
    }

    private void moveLift(int pos, double power) {
        liftL.setTargetPosition(pos);
        liftR.setTargetPosition(pos);
        liftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftL.setPower(power);
        liftR.setPower(power);
        while (opModeIsActive() && liftL.isBusy() && liftR.isBusy()) {
            telemetry.addLine();
            telemetry.addData("Lift Target", pos);
            telemetry.addData("Lift Current", ((liftL.getCurrentPosition() + liftR.getCurrentPosition()) / 2));
            telemetry.update();
        }
        liftL.setPower(0);
        liftR.setPower(0);
    }

    private void moveArmRot(int pos, double power) {
        arm_rot.setTargetPosition(pos);
        arm_rot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm_rot.setPower(power);
        while (opModeIsActive() && arm_rot.isBusy()) {
            telemetry.addLine();
            telemetry.addData("Arm Target", pos);
            telemetry.addData("Arm Current", arm_rot.getCurrentPosition());
            telemetry.update();
        }
        arm_rot.setPower(0);
    }

}
