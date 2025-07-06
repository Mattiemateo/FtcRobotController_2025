package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp
public class main_2p_imu extends LinearOpMode {
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
    private int targetArmRotPos = 0;
    private int minLift = 0;
    private boolean hasRumbled = false;
    private IMU imu;

    @Override
    public void runOpMode() {
        // Initialize motors and servos
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");

        claw = hardwareMap.get(Servo.class, "claw");
        arm_extend = hardwareMap.get(CRServo.class, "armext");
        arm_rot = hardwareMap.get(DcMotor.class, "armrot");
        liftR = hardwareMap.get(DcMotor.class, "liftR");
        liftL = hardwareMap.get(DcMotor.class, "liftL");

        // Motor directions
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        liftL.setDirection(DcMotor.Direction.FORWARD);
        liftR.setDirection(DcMotor.Direction.REVERSE);

        //arm_rot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Lift setup
        liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftL.setTargetPosition(0);
        liftR.setTargetPosition(0);
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

        while (opModeIsActive()) {
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

            telemetry.addData("Target Rot", targetArmRotPos);
            telemetry.addData("Arm Rot", armRot);
            telemetry.addData("clawPos", clawPos);
            telemetry.update();

            // Driving
            double x = -gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double theta = Math.atan2(y, x);
            double power = Math.hypot(x, y);
            double sin = Math.sin(theta - Math.PI / 4);
            double cos = Math.cos(theta - Math.PI / 4);
            double leftFront = power * cos + turn;
            double rightFront = power * sin - turn;
            double leftBack = power * sin + turn;
            double rightBack = power * cos - turn;

            if ((power + Math.abs(turn)) > 1) {
                leftFront /= power + Math.abs(turn);
                leftBack /= power + Math.abs(turn);
                rightFront /= power + Math.abs(turn);
                rightBack /= power + Math.abs(turn);
            }

            double scale = gamepad1.left_trigger > 0.5 ? 0.25 : 1.0;

            leftFrontDrive.setPower(leftFront * scale);
            leftBackDrive.setPower(leftBack * scale);
            rightFrontDrive.setPower(rightFront * scale);
            rightBackDrive.setPower(rightBack * scale);

            // Reset
            if (gamepad2.start) {
                targetLiftPos = minLift;
                claw.setPosition(0.6);
            }

            // Claw
            if (gamepad2.triangle) {
                claw.setPosition(0.8);
                clawPos = 0.8;
            } else if (gamepad2.circle) {
                claw.setPosition(0.6);
                clawPos = 0.6;
            } else if (gamepad2.cross) {
                claw.setPosition(0.45);
                clawPos = 0.45;
            }

            // Extension
            if (gamepad2.right_stick_x < -0.5) {
                arm_extend.setPower(1);
            } else if (gamepad2.right_stick_x > 0.5) {
                arm_extend.setPower(-1);
            } else {
                arm_extend.setPower(0);
            }

            //rotation arm
            if (gamepad2.right_stick_y < -0.5) {
                arm_rot.setPower(gamepad2.right_stick_y * -1);
            } else if (gamepad2.right_stick_y > 0.5) {
                arm_rot.setPower(0);
            }else{
                if (arm_rot.getCurrentPosition() <= 230) {
                    arm_rot.setPower(0.4);
                }else{
                    arm_rot.setPower(-0.3);
                }

            }



            // Lift
            if (gamepad2.left_stick_y < -0.5) {
                targetLiftPos += 40;
                if (gamepad2.right_trigger >= 0.9) {
                    targetLiftPos -= 10;
                }
            } else if (gamepad2.left_stick_y > 0.5) {
                targetLiftPos -= 40;
                if (gamepad2.right_trigger >= 0.9) {
                    targetLiftPos += 10;
                }
            }

            if (gamepad2.left_trigger >= 0.9) {
                minLift = targetLiftPos;
                gamepad2.rumble(500);
            } else {
                targetLiftPos = Math.max(minLift, Math.min(targetLiftPos, 7280 + minLift));
            }


            if (targetLiftPos >= 7275 + minLift) {
                if (!hasRumbled) {
                    gamepad2.rumble(500);
                    hasRumbled = true;
                }
            } else {
                hasRumbled = false;
            }

            // Apply lift target
            liftR.setTargetPosition(targetLiftPos);
            liftL.setTargetPosition(targetLiftPos);
            liftR.setPower(0.6);
            liftL.setPower(0.6);

            // Touchpad ping
            if (gamepad1.touchpad_finger_1) gamepad2.rumble(200);
            if (gamepad2.touchpad_finger_1) gamepad1.rumble(200);
        }
    }
}
