package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class main_2p extends LinearOpMode {
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

    @Override
    public void runOpMode() {
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

        while (opModeIsActive()) {
            int liftPosR = liftR.getCurrentPosition();
            int liftPosL = liftL.getCurrentPosition();

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

            telemetry.addData("clawPos", clawPos);
            telemetry.addLine();


            telemetry.update();

            // Driving
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

            //reset
            if (gamepad2.start) {
                targetLiftPos = 0;
                claw.setPosition(0.6);
            }

            //claw TWEAK HERE
            if (gamepad2.triangle) {
                claw.setPosition(0.8);
                clawPos = 0.8;
            }else if(gamepad2.circle) {
                claw.setPosition(0.6);
                clawPos = 0.6;
            }else if(gamepad2.cross) {
                claw.setPosition(0.45);
                clawPos = 0.45;
            }

            //extention arm
            if(gamepad2.right_stick_x < -0.5){
                arm_extend.setPower(1);
            }else if (gamepad2.right_stick_x > 0.5){
                arm_extend.setPower(-1);
            }else{
                arm_extend.setPower(0);
            }

            //rotation arm
            if (gamepad2.right_stick_y < -0.5) {
                arm_rot.setPower(0.9 );
            } else if (gamepad2.right_stick_y > 0.5) {
                arm_rot.setPower(-0.9);
            }else{
                arm_rot.setPower(0);
            }

            //lift
            if (gamepad2.left_stick_y < -0.5) {
                targetLiftPos += 15;
                if (gamepad2.left_trigger >= 0.9) {
                    targetLiftPos -= 10;
                }
            } else if (gamepad2.left_stick_y > 0.5) {
                targetLiftPos -= 15;
                if (gamepad2.left_trigger >= 0.9) {
                    targetLiftPos += 10;
                }
            }

            // Clamp target position within limits (tune these)
            if (gamepad2.left_trigger >= 0.9) {
                minLift = targetLiftPos;
                gamepad2.rumble(500);
            } else {
                targetLiftPos = Math.max(minLift, Math.min(targetLiftPos, 2700+minLift));
            }

            if (targetLiftPos >= 2650+minLift) {
                gamepad2.rumble(500);
            }

            // ptort
            if (gamepad1.touchpad_finger_1) {
                gamepad2.rumble(200);
            }
            if (gamepad2.touchpad_finger_1) {
                gamepad1.rumble(200);
            }

            // Apply target to both motors
            liftR.setTargetPosition(targetLiftPos);
            liftL.setTargetPosition(targetLiftPos);

            liftR.setPower(1.0);
            liftL.setPower(1.0);
        }
    }
}
