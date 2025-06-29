package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class main_2p_old extends LinearOpMode {
    private Servo claw;
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;
    private DcMotor liftR;
    private DcMotor liftL;
    boolean is_open_claw = true;
    private int targetLiftPos = 0;

    @Override
    public void runOpMode() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
        claw = hardwareMap.get(Servo.class, "claw");
        liftR = hardwareMap.get(DcMotor.class, "liftR");
        liftL = hardwareMap.get(DcMotor.class, "liftL");

        liftL.setDirection(DcMotor.Direction.REVERSE);
        liftR.setDirection(DcMotor.Direction.FORWARD);

        liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        liftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Waiting for input");
            telemetry.addData("lx", gamepad1.left_stick_x);
            telemetry.addData("ly", gamepad1.left_stick_y);
            telemetry.addData("rx", gamepad1.right_stick_x);

            telemetry.addData("is_open_claw", is_open_claw);
            double lift_pos = liftR.getCurrentPosition();
            telemetry.addData("Lift Position", lift_pos);

            telemetry.update();

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

                double scale = gamepad1.left_trigger > 0.5 ? 0.25 : 1.0;

                leftFrontDrive.setPower(leftFront * scale);
                leftBackDrive.setPower(leftBack * scale);
                rightFrontDrive.setPower(rightFront * scale);
                rightBackDrive.setPower(rightBack * scale);
            }

            if (gamepad2.square) {
                if (is_open_claw){
                    claw.setPosition(0.6);
                    is_open_claw = false;
                }
            }else if(gamepad2.cross) {
                if (!is_open_claw){
                    claw.setPosition(0.4);
                    is_open_claw = true;
                }
            }

            if (gamepad2.dpad_up) {
                targetLiftPos += 20;
            } else if (gamepad2.dpad_down) {
                targetLiftPos -= 20;
            }

            targetLiftPos = Math.max(0, Math.min(targetLiftPos, 2750));

            liftR.setTargetPosition(targetLiftPos);
            liftL.setTargetPosition(targetLiftPos);

            liftR.setPower(1.0);
            liftL.setPower(1.0);
        }

    }
}
