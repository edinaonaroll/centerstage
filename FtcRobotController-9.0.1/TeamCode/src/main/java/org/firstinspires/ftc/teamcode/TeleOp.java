package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Basic: TrigTeleOp", group="TrigTeleOp")
public class TeleOp extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        // Declare drive motors
        // Make sure your ID's match your configuration
        DcMotor leftFront = hardwareMap.dcMotor.get("leftFront");
        DcMotor leftBack = hardwareMap.dcMotor.get("leftBack");
        DcMotor rightFront = hardwareMap.dcMotor.get("rightFront");
        DcMotor rightBack = hardwareMap.dcMotor.get("rightBack");

        // Declare launcher/wrist/hand
        Servo launcherServo = hardwareMap.servo.get("launcher");
        Servo wristServo = hardwareMap.servo.get("wrist");
        Servo handServo = hardwareMap.servo.get("hand");

        // Declare arm
        DcMotor armMotor = hardwareMap.dcMotor.get("arm");
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        //rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        //hand.setPosition(handDefault);

        waitForStart();

        while (opModeInInit())
            handServo.setPosition(0.62);

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            //////////// Drivetrain code ////////////

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            double r = Math.hypot(x, y);
            double robotAngle = Math.atan2(y, x) - Math.PI / 4;
            double rightX = rx;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            leftFront.setPower(v1);
            rightFront.setPower(v2);
            leftBack.setPower(v3);
            rightBack.setPower(v4);

            //////// Arm and wrist code //////////

            // Arm, wrist, hand variables
            double armPowerUp = -.5;
            double armPowerDown = -.4;
            double armBrake = .5;
            double wristUp = .7;
            double wristDown = 0.1;
            double handOpen = 0.12;
            double handClose = 0.22;
            int armVert = -680;

            if(gamepad1.a) {
                // close hand
                handServo.setPosition(handClose);

                // wait for good grip
                sleep(300);

                // lift arm
                armMotor.setTargetPosition(-640);
                armMotor.setPower(armPowerUp);
                while (armMotor.isBusy());
                armMotor.setPower(0);

                // turn wrist
                wristServo.setPosition(wristUp);
            }

            if (gamepad1.b) {
                // open hand
                handServo.setPosition(handOpen);

                // wait for drop
                sleep(100);

                // move fast
                armMotor.setTargetPosition(-650);
                armMotor.setPower(armPowerDown);
                while (armMotor.isBusy());

                // turn wrist
                wristServo.setPosition(wristDown);

                // move slow
                armMotor.setTargetPosition(-10);
                armMotor.setPower(-armPowerDown/4);
                while (armMotor.isBusy());
                armMotor.setPower(0);

            }

            // Servo code
            if(gamepad1.right_trigger != 0) {
                // move to 0 degrees.
                handServo.setPosition(handClose);
            } else if (gamepad1.left_trigger != 0) {
                // move to 180 degrees.
                handServo.setPosition(handOpen);
            }

            //int preLaunchAngle = (int) 0;
            //int launchAngle = (int) -0.7;

            if(gamepad1.dpad_up && gamepad1.dpad_down && gamepad1.dpad_left && gamepad1.dpad_right) {
                // move to 0 degrees.
                launcherServo.setPosition(0.9);
            }

            telemetry.addData("launcherPos", launcherServo.getPosition());
            telemetry.addData("status", "Running");
            telemetry.addData("armEncoder", armMotor.getCurrentPosition());
            telemetry.addData("armEncoderTar", armMotor.getTargetPosition());
            telemetry.addData("handPos", handServo.getPosition());
            telemetry.update();

            idle();

        }
    }
}
