package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous
public class Autoscrape extends LinearOpMode {
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
            Thread base_movement = new Thread(() -> {
                moveClaw(0.45);
                driveYTime(1200 , -0.8);//forward
                sleep(1000);
                //turnToAngle(90, -0.5);
                driveXTime(600, 0.5);//front of block
                sleep(1000);
                driveYTime(1000, 0.8);//back ...... score
                sleep(1000);
                driveXTime(400, -0.5);
                sleep(200);
                driveYTime(500, 0.8);
                sleep(200);
                driveYTime(1000, -0.8);//go front
                sleep(1000);
                driveXTime(300, 1);//align
                sleep(200);
                driveYTime(1000, 1);//back
                sleep(200);
                //driveYTime(1000, -1);//front
                //sleep(200);
                //driveXTime(1000, 1);//align
                //sleep(200);
                //driveYTime(1000, 1);//back

                //driveXTime(500, -1);
            });
            base_movement.start();
            base_movement.join();
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
        double targetHeading = getHeading();
        long startTime = System.currentTimeMillis();

        while (opModeIsActive() && System.currentTimeMillis() - startTime < ms) {
            double currentHeading = getHeading();
            double error = angleDifference(targetHeading, currentHeading);

            // Proportional heading correction
            double kP = 0.015;
            double correction = error * kP;

            // Proper mecanum strafing layout (power + correction)
            double leftFrontPower  =  power + correction;
            double rightFrontPower = -power - correction;
            double leftBackPower   = -power + correction;
            double rightBackPower  =  power - correction;

            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            telemetry.addData("Heading", currentHeading);
            telemetry.addData("Target", targetHeading);
            telemetry.addData("Error", error);
            telemetry.addData("Correction", correction);
            telemetry.update();
        }

        stopDrive();
    }

    private void straighten() {
        double currentHeading = getHeading();
        double targetHeading = 0; // Adjust if your "start" heading is different

        telemetry.addLine("Straightening...");
        telemetry.addData("Current Heading", currentHeading);
        telemetry.update();

        turnToAngle(targetHeading, 0.8); // Use your desired max power
    }

    private void turnToAngle(double targetAngle, double maxPower) {
        double kP = 0.002;   // Proportional gain — tune as needed
        double kD = 0.004;  // Derivative gain — helps dampen oscillation
        double lastError = 0;
        double acceptableError = 2.0;  // degrees

        while (opModeIsActive()) {
            double currentHeading = getHeading();
            double error = angleDifference(targetAngle, currentHeading);
            double derivative = error - lastError;
            lastError = error;

            if (Math.abs(error) <= acceptableError) break;

            double turnPower = (kP * error) + (kD * derivative);

            // Clamp to maxPower
            turnPower = Math.max(-maxPower, Math.min(maxPower, turnPower));

            // Prevent jitter/stalling due to too-low power
            if (Math.abs(turnPower) < 0.08) {
                turnPower = 0.08 * Math.signum(turnPower);
            }

            // Apply turning
            leftFrontDrive.setPower(turnPower);
            leftBackDrive.setPower(turnPower);
            rightFrontDrive.setPower(-turnPower);
            rightBackDrive.setPower(-turnPower);

            telemetry.addData("Target", targetAngle);
            telemetry.addData("Heading", currentHeading);
            telemetry.addData("Error", error);
            telemetry.addData("Turn Power", turnPower);
            telemetry.update();
            //sleep(1000);
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

    private void turn(int mm, float left) {
        leftFrontDrive.setPower(0.5 * left);
        rightFrontDrive.setPower(-0.5 * left);
        leftBackDrive.setPower(0.5 * left);
        rightBackDrive.setPower(-0.5 * left);
        sleep(mm);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

}
