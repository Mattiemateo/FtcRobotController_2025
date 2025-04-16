package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class LinearTest extends LinearOpMode {
    private DcMotor lift;
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;

    int targetpos = 0;
    int liftpos = 0;

    @Override
    public void runOpMode() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");  // FIXED
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive"); // FIXED
        lift = hardwareMap.get(DcMotor.class, "lift");

        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Waiting for input");
            telemetry.addData("lx", gamepad1.left_stick_x);
            telemetry.addData("ly", gamepad1.left_stick_y);
            telemetry.addData("rx", gamepad1.right_stick_x);

            telemetry.update();

            liftpos = lift.getCurrentPosition();

            if (gamepad1.left_trigger < 0.5) {
                double x = gamepad1.left_stick_x;
                double y = -gamepad1.left_stick_y;
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
                    leftFront /= power + turn;
                    leftBack /= power + turn;
                    rightFront /= power + turn;
                    rightBack /= power + turn;
                }

                leftFrontDrive.setPower(leftFront);
                leftBackDrive.setPower(leftBack);
                rightFrontDrive.setPower(rightFront);
                rightBackDrive.setPower(rightBack);

            } else if (gamepad1.left_trigger > 0.5) {
                double x = gamepad1.left_stick_x;
                double y = -gamepad1.left_stick_y;
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
                    leftFront /= power + turn;
                    leftBack /= power + turn;
                    rightFront /= power + turn;
                    rightBack /= power + turn;
                }

                leftFrontDrive.setPower(leftFront * 0.25);
                leftBackDrive.setPower(leftBack * 0.25);
                rightFrontDrive.setPower(rightFront * 0.25);
                rightBackDrive.setPower(rightBack * 0.25);
            }

            // Lift control
            if (gamepad2.dpad_up) {
                lift.setPower(0.6);
                targetpos = 0;
            } else if (gamepad2.dpad_down) {
                lift.setPower(-0.6);
                targetpos = 0;
            } else {
                if (targetpos == 0) {
                    targetpos = liftpos;
                }
                if (targetpos + 15 < liftpos) {
                    lift.setPower(-0.1);
                } else if (targetpos - 15 > liftpos){
                    lift.setPower(0.1);
                } else {
                    lift.setPower(0);
                }
            }

        }

    }
}
