package org.firstinspires.ftc.teamcode;
/*
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Team 9960 Revision 170119.0
 * This program is the scaffold for autonomous operation.
 * Designed to push the correct button on both beacons
 *
 * This robot uses four VEX Mecanum wheels, each direct driven by Neverest 40 motors.
 * It is designed as a linear op mode, and uses RUN_TO_POSITION motor operation.
 *
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous", group="Autonomous")
public class Autonomous extends LinearOpMode {

    // Declare Devices
    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor leftBack = null;
    DcMotor rightBack = null;

    DcMotor arm = null;
    AnalogInput redEye = null;
    OpticalDistanceSensor mrOds = null;


    // drive motor position variables
    private int lfPos; private int rfPos; private int lbPos; private int rbPos;

    // operational constants
    private double fast = 0.5; // Limit motor power to this value for Andymark RUN_USING_ENCODER mode
    private double medium = 0.3; // medium speed
    private double slow = 0.1; // slow speed
    private double clicksPerInch = 52; // 54 best for slow speed
    private double clicksPerDeg = 14; // empirically measured
    private double lineThreshold = 0.7; // floor should be below this value, line above
    private double redThreshold = 1.9; // red should be below this value, blue above

    @Override
    public void runOpMode() {
        telemetry.setAutoClear(true);

        // Initialize the hardware variables.
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        arm = hardwareMap.dcMotor.get("arm");
        //redEye = hardwareMap.analogInput.get("redEye");
        //mrOds = hardwareMap.opticalDistanceSensor.get("mrOds");
        //mrOds.enableLed(true);

        // The right motors need reversing
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        //rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        // setTargetPosition before setMode (not sure why)
        leftFront.setTargetPosition(0);
        leftBack.setTargetPosition(0);
        rightFront.setTargetPosition(0);
        rightBack.setTargetPosition(0);
        arm.setTargetPosition(0);

        // Set the drive motor run modes:
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // *****************Dead reckoning list*************
        // Distances in inches, angles in deg, speed 0.0 to 0.6
        /* moveForward(16, fast);
        turnClockwise(-45, fast);
        moveForward(33, fast);
        turnClockwise(-45, fast);
        moveForward(24, fast);
        moveToLine(24, medium);
        pushRedButton();
        moveForward(-6, fast);
        turnClockwise(-3, medium); // aiming tweak
        moveRight(36, fast);
        moveToLine(24, medium);
        pushRedButton();
        moveForward(-12, fast);
        turnClockwise(-135, fast);
        moveForward(66, fast); */

        moveForward(27, medium);
        turnClockwise(90, medium);
        moveForward(-93, fast);
        moveForward(2, medium);
        moveRight(40, fast);
        moveForward(-19, fast);
    }

    private void moveForward(int howMuch, double speed) {
        // howMuch is in inches. A negative howMuch moves backward.

        // fetch motor positions
        lfPos = leftFront.getCurrentPosition();
        rfPos = rightFront.getCurrentPosition();
        lbPos = leftBack.getCurrentPosition();
        rbPos = rightBack.getCurrentPosition();

        // calculate new targets
        lfPos += howMuch * clicksPerInch;
        rfPos += howMuch * clicksPerInch;
        lbPos += howMuch * clicksPerInch;
        rbPos += howMuch * clicksPerInch;

        // move robot to new position
        leftFront.setTargetPosition(lfPos);
        rightFront.setTargetPosition(rfPos);
        leftBack.setTargetPosition(lbPos);
        rightBack.setTargetPosition(rbPos);
        leftFront.setPower(speed);
        rightFront.setPower(speed);
        leftBack.setPower(speed);
        rightBack.setPower(speed);

        // wait for move to complete
        while (leftFront.isBusy() && rightFront.isBusy() &&
                leftBack.isBusy() && rightBack.isBusy()) {

            // Display it for the driver.
            telemetry.addLine("Move Foward");
            telemetry.addData("Target", "%7d :%7d", lfPos, rfPos, lbPos, rbPos);
            telemetry.addData("Actual", "%7d :%7d", leftFront.getCurrentPosition(),
                    rightFront.getCurrentPosition(), leftBack.getCurrentPosition(),
                    rightBack.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    private void moveRight(int howMuch, double speed) {
        // howMuch is in inches. A negative howMuch moves backward.

        // fetch motor positions
        lfPos = leftFront.getCurrentPosition();
        rfPos = rightFront.getCurrentPosition();
        lbPos = leftBack.getCurrentPosition();
        rbPos = rightBack.getCurrentPosition();

        // calculate new targets
        lfPos += howMuch * clicksPerInch;
        rfPos -= howMuch * clicksPerInch;
        lbPos -= howMuch * clicksPerInch;
        rbPos += howMuch * clicksPerInch;

        // move robot to new position
        leftFront.setTargetPosition(lfPos);
        rightFront.setTargetPosition(rfPos);
        leftBack.setTargetPosition(lbPos);
        rightBack.setTargetPosition(rbPos);
        leftFront.setPower(speed);
        rightFront.setPower(speed);
        leftBack.setPower(speed);
        rightBack.setPower(speed);

        // wait for move to complete
        while (leftFront.isBusy() && rightFront.isBusy() &&
                leftBack.isBusy() && rightBack.isBusy()) {

            // Display it for the driver.
            telemetry.addLine("Strafe Right");
            telemetry.addData("Target", "%7d :%7d", lfPos, rfPos, lbPos, rbPos);
            telemetry.addData("Actual", "%7d :%7d", leftFront.getCurrentPosition(),
                    rightFront.getCurrentPosition(), leftBack.getCurrentPosition(),
                    rightBack.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

    }

    private void turnClockwise(int whatAngle, double speed) {
        // whatAngle is in degrees. A negative whatAngle turns counterclockwise.

        // fetch motor positions
        lfPos = leftFront.getCurrentPosition();
        rfPos = rightFront.getCurrentPosition();
        lbPos = leftBack.getCurrentPosition();
        rbPos = rightBack.getCurrentPosition();

        // calculate new targets
        lfPos += whatAngle * clicksPerDeg;
        rfPos -= whatAngle * clicksPerDeg;
        lbPos += whatAngle * clicksPerDeg;
        rbPos -= whatAngle * clicksPerDeg;

        // move robot to new position
        leftFront.setTargetPosition(lfPos);
        rightFront.setTargetPosition(rfPos);
        leftBack.setTargetPosition(lbPos);
        rightBack.setTargetPosition(rbPos);
        leftFront.setPower(speed);
        rightFront.setPower(speed);
        leftBack.setPower(speed);
        rightBack.setPower(speed);

        // wait for move to complete
        while (leftFront.isBusy() && rightFront.isBusy() &&
                leftBack.isBusy() && rightBack.isBusy()) {

            // Display it for the driver.
            telemetry.addLine("Turn Clockwise");
            telemetry.addData("Target", "%7d :%7d", lfPos, rfPos, lbPos, rbPos);
            telemetry.addData("Actual", "%7d :%7d", leftFront.getCurrentPosition(),
                    rightFront.getCurrentPosition(), leftBack.getCurrentPosition(),
                    rightBack.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }
    private void moveToLine(int howMuch, double speed) {
        // howMuch is in inches. The robot will stop if the line is found before
        // this distance is reached. A negative howMuch moves left, positive moves right.

        // fetch motor positions
        lfPos = leftFront.getCurrentPosition();
        rfPos = rightFront.getCurrentPosition();
        lbPos = leftBack.getCurrentPosition();
        rbPos = rightBack.getCurrentPosition();

        // calculate new targets
        lfPos += howMuch * clicksPerInch;
        rfPos -= howMuch * clicksPerInch;
        lbPos -= howMuch * clicksPerInch;
        rbPos += howMuch * clicksPerInch;

        // move robot to new position
        leftFront.setTargetPosition(lfPos);
        rightFront.setTargetPosition(rfPos);
        leftBack.setTargetPosition(lbPos);
        rightBack.setTargetPosition(rbPos);
        leftFront.setPower(speed);
        rightFront.setPower(speed);
        leftBack.setPower(speed);
        rightBack.setPower(speed);

        // wait for move to complete
        while (leftFront.isBusy() && rightFront.isBusy() &&
                leftBack.isBusy() && rightBack.isBusy()) {
            if (mrOds.getLightDetected() > lineThreshold) break;

            // Display it for the driver.
            telemetry.addLine("Move To Line");
            telemetry.addData("Target", "%7d :%7d", lfPos, rfPos, lbPos, rbPos);
            telemetry.addData("Actual", "%7d :%7d", leftFront.getCurrentPosition(),
                    rightFront.getCurrentPosition(), leftBack.getCurrentPosition(),
                    rightBack.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

    }
    private void pushRedButton() {
        // Red alliance version
        // Assumes prepositioned in front of left button
        // if beacon is red, push it, otherwise move right 5" and push the other button
        if (redEye.getVoltage() > redThreshold) moveRight(5, slow); // we see a blue beacon
        moveForward(8, slow);

    }
    private void pushBlueButton() {
        // Blue alliance version
        // Assumes prepositioned in front of left button
        // if beacon is blue, push it, otherwise move right 5" and push the other button
        if (redEye.getVoltage() < redThreshold) moveRight(5, slow); // we see a red beacon
        moveForward(8, slow);

    }

}
