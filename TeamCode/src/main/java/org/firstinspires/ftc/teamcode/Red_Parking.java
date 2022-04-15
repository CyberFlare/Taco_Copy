package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="Red/BlueParking")
public class Red_Parking extends LinearOpMode {
    BennyHW robot = new BennyHW();   // Use a Pushbot's hardware


    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

//anything below failed

        robot.leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.srightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.sleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.foundation.setPosition(111);

        waitForStart();


        driveDashBackwards(500);

        stop();

    }


    public void driveForward(int distance) {

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.sleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.srightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setTargetPosition(distance);
        robot.rightDrive.setTargetPosition(distance);
        robot.sleftDrive.setTargetPosition(distance);
        robot.srightDrive.setTargetPosition(distance);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.sleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.srightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftDrive.setPower(0.15);
        robot.rightDrive.setPower(0.15);
        robot.sleftDrive.setPower(0.15);
        robot.srightDrive.setPower(0.15);

        while (robot.leftDrive.isBusy() && robot.rightDrive.isBusy() && robot.sleftDrive.isBusy() && robot.srightDrive.isBusy()) {
            telemetry.addData("encoderCountL", robot.leftDrive.getCurrentPosition());
            telemetry.addData("encoderCountR", robot.rightDrive.getCurrentPosition());
            telemetry.addData("encoderCountLS", robot.sleftDrive.getCurrentPosition());
            telemetry.addData("encoderCountRS", robot.srightDrive.getCurrentPosition());
            telemetry.update();

        }

        while (robot.leftDrive.isBusy() && robot.rightDrive.isBusy() && robot.sleftDrive.isBusy() && robot.srightDrive.isBusy()) {

        }

        stopDriving();

        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.srightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.sleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void driveDashForward(int distance) {

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.sleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.srightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setTargetPosition(distance);
        robot.rightDrive.setTargetPosition(distance);
        robot.sleftDrive.setTargetPosition(distance);
        robot.srightDrive.setTargetPosition(distance);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.sleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.srightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftDrive.setPower(1);
        robot.rightDrive.setPower(1);
        robot.sleftDrive.setPower(1);
        robot.srightDrive.setPower(1);

        while (robot.leftDrive.isBusy() && robot.rightDrive.isBusy() && robot.sleftDrive.isBusy() && robot.srightDrive.isBusy()) {
            telemetry.addData("encoderCountL", robot.leftDrive.getCurrentPosition());
            telemetry.addData("encoderCountR", robot.rightDrive.getCurrentPosition());
            telemetry.addData("encoderCountLS", robot.sleftDrive.getCurrentPosition());
            telemetry.addData("encoderCountRS", robot.srightDrive.getCurrentPosition());
            telemetry.update();

        }

        while (robot.leftDrive.isBusy() && robot.rightDrive.isBusy() && robot.sleftDrive.isBusy() && robot.srightDrive.isBusy()) {

        }

        stopDriving();

        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.srightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.sleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void driveTurnLeft(int distance) {

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.sleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.srightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setTargetPosition(-distance);
        robot.rightDrive.setTargetPosition(-distance);
        robot.sleftDrive.setTargetPosition(distance);
        robot.srightDrive.setTargetPosition(distance);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.sleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.srightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftDrive.setPower(1);
        robot.rightDrive.setPower(1);
        robot.sleftDrive.setPower(1);
        robot.srightDrive.setPower(1);


        while (robot.leftDrive.isBusy() && robot.rightDrive.isBusy() && robot.sleftDrive.isBusy() && robot.srightDrive.isBusy()) {
            telemetry.addData("encoderCountL", robot.leftDrive.getCurrentPosition());
            telemetry.addData("encoderCountR", robot.rightDrive.getCurrentPosition());
            telemetry.addData("encoderCountLS", robot.sleftDrive.getCurrentPosition());
            telemetry.addData("encoderCountRS", robot.srightDrive.getCurrentPosition());
            telemetry.update();

        }

        while (robot.leftDrive.isBusy() && robot.rightDrive.isBusy() && robot.sleftDrive.isBusy() && robot.srightDrive.isBusy()) {

        }

        stopDriving();

        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.srightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.sleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    public void raiseServo(double position) {
        robot.foundation.setPosition(-0.4);

        //telemetry.addData("3", robot.Foundation.getPortNumber());
    }

    public void lowerServo() {
        robot.foundation.setPosition(-0.4);
    }

    public void stopDriving() {
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.sleftDrive.setPower(0);
        robot.srightDrive.setPower(0);

    }

    public void driveTurnRight(int distance) {

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.sleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.srightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setTargetPosition(distance);
        robot.rightDrive.setTargetPosition(distance);
        robot.sleftDrive.setTargetPosition(-distance);
        robot.srightDrive.setTargetPosition(-distance);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.sleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.srightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftDrive.setPower(0.6);
        robot.rightDrive.setPower(0.6);
        robot.sleftDrive.setPower(0.6);
        robot.srightDrive.setPower(0.6);

        while (robot.leftDrive.isBusy() && robot.rightDrive.isBusy() && robot.sleftDrive.isBusy() && robot.srightDrive.isBusy()) {
            telemetry.addData("encoderCountL", robot.leftDrive.getCurrentPosition());
            telemetry.addData("encoderCountR", robot.rightDrive.getCurrentPosition());
            telemetry.addData("encoderCountLS", robot.sleftDrive.getCurrentPosition());
            telemetry.addData("encoderCountRS", robot.srightDrive.getCurrentPosition());
            telemetry.update();
        }

        while (robot.leftDrive.isBusy() && robot.rightDrive.isBusy() && robot.sleftDrive.isBusy() && robot.srightDrive.isBusy()) {

        }
        stopDriving();

        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.srightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.sleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void driveBackwards(int distance) {
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.sleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.srightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setTargetPosition(-distance);
        robot.rightDrive.setTargetPosition(-distance);
        robot.sleftDrive.setTargetPosition(-distance);
        robot.srightDrive.setTargetPosition(-distance);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.sleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.srightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftDrive.setPower(0.25);
        robot.rightDrive.setPower(0.25);
        robot.sleftDrive.setPower(0.25);
        robot.srightDrive.setPower(0.25);

        while (robot.leftDrive.isBusy() && robot.rightDrive.isBusy() && robot.sleftDrive.isBusy() && robot.srightDrive.isBusy()) {
            telemetry.addData("encoderCountL", robot.leftDrive.getCurrentPosition());
            telemetry.addData("encoderCountR", robot.rightDrive.getCurrentPosition());
            telemetry.addData("encoderCountLS", robot.sleftDrive.getCurrentPosition());
            telemetry.addData("encoderCountRS", robot.srightDrive.getCurrentPosition());
            telemetry.update();
        }
        while (robot.leftDrive.isBusy() && robot.rightDrive.isBusy() && robot.sleftDrive.isBusy() && robot.srightDrive.isBusy()) {

        }
        stopDriving();

        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.srightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.sleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void driveDashBackwards(int distance) {
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.sleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.srightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setTargetPosition(-distance);
        robot.rightDrive.setTargetPosition(-distance);
        robot.sleftDrive.setTargetPosition(-distance);
        robot.srightDrive.setTargetPosition(-distance);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.sleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.srightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftDrive.setPower(0.5);
        robot.rightDrive.setPower(0.5);
        robot.sleftDrive.setPower(0.5);
        robot.srightDrive.setPower(0.5);

        while (robot.leftDrive.isBusy() && robot.rightDrive.isBusy() && robot.sleftDrive.isBusy() && robot.srightDrive.isBusy()) {
            telemetry.addData("encoderCountL", robot.leftDrive.getCurrentPosition());
            telemetry.addData("encoderCountR", robot.rightDrive.getCurrentPosition());
            telemetry.addData("encoderCountLS", robot.sleftDrive.getCurrentPosition());
            telemetry.addData("encoderCountRS", robot.srightDrive.getCurrentPosition());
            telemetry.update();
        }
        while (robot.leftDrive.isBusy() && robot.rightDrive.isBusy() && robot.sleftDrive.isBusy() && robot.srightDrive.isBusy()) {

        }
        stopDriving();

        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.srightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.sleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void strafeRight (int distance){
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.sleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.srightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setTargetPosition(distance);
        robot.rightDrive.setTargetPosition(-distance);
        robot.sleftDrive.setTargetPosition(-distance);
        robot.srightDrive.setTargetPosition(distance);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.sleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.srightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftDrive.setPower(0.5);
        robot.rightDrive.setPower(0.5);
        robot.sleftDrive.setPower(0.5);
        robot.srightDrive.setPower(0.5);

        while (robot.leftDrive.isBusy() && robot.rightDrive.isBusy() && robot.sleftDrive.isBusy() && robot.srightDrive.isBusy()) {
            telemetry.addData("encoderCountL", robot.leftDrive.getCurrentPosition());
            telemetry.addData("encoderCountR", robot.rightDrive.getCurrentPosition());
            telemetry.addData("encoderCountLS", robot.sleftDrive.getCurrentPosition());
            telemetry.addData("encoderCountRS", robot.srightDrive.getCurrentPosition());
            telemetry.update();
        }
        while (robot.leftDrive.isBusy() && robot.rightDrive.isBusy() && robot.sleftDrive.isBusy() && robot.srightDrive.isBusy()) {

        }
        stopDriving();

        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.srightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.sleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void strafeDashRight (int distance){
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.sleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.srightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setTargetPosition(distance);
        robot.rightDrive.setTargetPosition(-distance);
        robot.sleftDrive.setTargetPosition(-distance);
        robot.srightDrive.setTargetPosition(distance);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.sleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.srightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftDrive.setPower(1);
        robot.rightDrive.setPower(1);
        robot.sleftDrive.setPower(1);
        robot.srightDrive.setPower(1);

        while (robot.leftDrive.isBusy() && robot.rightDrive.isBusy() && robot.sleftDrive.isBusy() && robot.srightDrive.isBusy()) {
            telemetry.addData("encoderCountL", robot.leftDrive.getCurrentPosition());
            telemetry.addData("encoderCountR", robot.rightDrive.getCurrentPosition());
            telemetry.addData("encoderCountLS", robot.sleftDrive.getCurrentPosition());
            telemetry.addData("encoderCountRS", robot.srightDrive.getCurrentPosition());
            telemetry.update();
        }
        while (robot.leftDrive.isBusy() && robot.rightDrive.isBusy() && robot.sleftDrive.isBusy() && robot.srightDrive.isBusy()) {

        }
        stopDriving();

        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.srightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.sleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void strafeLeft (int distance) {
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.sleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.srightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setTargetPosition(-distance);
        robot.rightDrive.setTargetPosition(distance);
        robot.sleftDrive.setTargetPosition(distance);
        robot.srightDrive.setTargetPosition(-distance);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.sleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.srightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftDrive.setPower(0.5);
        robot.rightDrive.setPower(0.5);
        robot.sleftDrive.setPower(0.5);
        robot.srightDrive.setPower(0.5);

        while (robot.leftDrive.isBusy() && robot.rightDrive.isBusy() && robot.sleftDrive.isBusy() && robot.srightDrive.isBusy()) {
            telemetry.addData("encoderCountL", robot.leftDrive.getCurrentPosition());
            telemetry.addData("encoderCountR", robot.rightDrive.getCurrentPosition());
            telemetry.addData("encoderCountLS", robot.sleftDrive.getCurrentPosition());
            telemetry.addData("encoderCountRS", robot.srightDrive.getCurrentPosition());
            telemetry.update();
        }
        while (robot.leftDrive.isBusy() && robot.rightDrive.isBusy() && robot.sleftDrive.isBusy() && robot.srightDrive.isBusy()) {

        }
        stopDriving();

        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.srightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.sleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void strafeDashLeft (int distance) {
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.sleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.srightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setTargetPosition(-distance);
        robot.rightDrive.setTargetPosition(distance);
        robot.sleftDrive.setTargetPosition(distance);
        robot.srightDrive.setTargetPosition(-distance);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.sleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.srightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftDrive.setPower(1);
        robot.rightDrive.setPower(1);
        robot.sleftDrive.setPower(1);
        robot.srightDrive.setPower(1);

        while (robot.leftDrive.isBusy() && robot.rightDrive.isBusy() && robot.sleftDrive.isBusy() && robot.srightDrive.isBusy()) {
            telemetry.addData("encoderCountL", robot.leftDrive.getCurrentPosition());
            telemetry.addData("encoderCountR", robot.rightDrive.getCurrentPosition());
            telemetry.addData("encoderCountLS", robot.sleftDrive.getCurrentPosition());
            telemetry.addData("encoderCountRS", robot.srightDrive.getCurrentPosition());
            telemetry.update();
        }
        while (robot.leftDrive.isBusy() && robot.rightDrive.isBusy() && robot.sleftDrive.isBusy() && robot.srightDrive.isBusy()) {

        }
        stopDriving();

        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.srightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.sleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}


