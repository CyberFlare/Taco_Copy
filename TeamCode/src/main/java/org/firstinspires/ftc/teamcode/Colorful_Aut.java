package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="GrabbyBlocky")
public class Colorful_Aut extends LinearOpMode {
    BennyHW taco = new BennyHW();   // Use a Pushbot's hardware

    @Override
    public void runOpMode() throws InterruptedException {
        taco.init(hardwareMap);

        taco.leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        taco.sleftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        taco.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        taco.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        taco.srightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        taco.sleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        driveForward(724,0.25);

        taco.Color1.enableLed(true);
        taco.Color2.enableLed(true);

        while (opModeIsActive()) {
            telemetry.addData("Color1A", taco.Color1.alpha());
            telemetry.addData("Color1R", taco.Color1.red());
            telemetry.addData("Color1G", taco.Color1.green());
            telemetry.addData("Color1B", taco.Color1.blue());
            telemetry.update();

            while (taco.leftDrive.isBusy() && taco.rightDrive.isBusy() && taco.sleftDrive.isBusy() && taco.srightDrive.isBusy()) {
                telemetry.addData("encoderCountL", taco.leftDrive.getCurrentPosition());
                telemetry.addData("encoderCountR", taco.rightDrive.getCurrentPosition());
                telemetry.addData("encoderCountLS", taco.sleftDrive.getCurrentPosition());
                telemetry.addData("encoderCountRS", taco.srightDrive.getCurrentPosition());
                telemetry.update();

            }
        }
    }
    public void driveForward(int distance, double pwr) {

        taco.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        taco.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        taco.sleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        taco.srightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        taco.leftDrive.setTargetPosition(distance);
        taco.rightDrive.setTargetPosition(distance);
        taco.sleftDrive.setTargetPosition(distance);
        taco.srightDrive.setTargetPosition(distance);

        taco.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        taco.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        taco.sleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        taco.srightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        taco.leftDrive.setPower(pwr);
        taco.rightDrive.setPower(pwr);
        taco.sleftDrive.setPower(pwr);
        taco.srightDrive.setPower(pwr);

        while (taco.leftDrive.isBusy() && taco.rightDrive.isBusy() && taco.sleftDrive.isBusy() && taco.srightDrive.isBusy()) {

        }

        stopDriving();

        taco.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        taco.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        taco.srightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        taco.sleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    public void driveTurnLeft(int distance, int pwr) {

        taco.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        taco.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        taco.sleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        taco.srightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        taco.leftDrive.setTargetPosition(-distance);
        taco.rightDrive.setTargetPosition(-distance);
        taco.sleftDrive.setTargetPosition(distance);
        taco.srightDrive.setTargetPosition(distance);

        taco.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        taco.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        taco.sleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        taco.srightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        taco.leftDrive.setPower(pwr);
        taco.rightDrive.setPower(pwr);
        taco.sleftDrive.setPower(pwr);
        taco.srightDrive.setPower(pwr);

        while (taco.leftDrive.isBusy() && taco.rightDrive.isBusy() && taco.sleftDrive.isBusy() && taco.srightDrive.isBusy()) {

        }

        stopDriving();

        taco.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        taco.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        taco.srightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        taco.sleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void serverStuff(double pwr, int time) {
        taco.foundation.setPower(pwr);
        sleep(time);
    }

    public void stopDriving() {
        taco.leftDrive.setPower(0);
        taco.rightDrive.setPower(0);
        taco.sleftDrive.setPower(0);
        taco.srightDrive.setPower(0);
    }

    public void driveTurnRight(int distance, double pwr) {

        taco.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        taco.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        taco.sleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        taco.srightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        taco.leftDrive.setTargetPosition(-distance);
        taco.rightDrive.setTargetPosition(distance);
        taco.sleftDrive.setTargetPosition(-distance);
        taco.srightDrive.setTargetPosition(distance);

        taco.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        taco.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        taco.sleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        taco.srightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        taco.leftDrive.setPower(pwr);
        taco.rightDrive.setPower(pwr);
        taco.sleftDrive.setPower(pwr);
        taco.srightDrive.setPower(pwr);

        while (taco.leftDrive.isBusy() && taco.rightDrive.isBusy() && taco.sleftDrive.isBusy() && taco.srightDrive.isBusy()) {

        }
        stopDriving();

        taco.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        taco.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        taco.srightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        taco.sleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void driveBackwards(int distance, double pwr) {
        taco.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        taco.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        taco.sleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        taco.srightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        taco.leftDrive.setTargetPosition(-distance);
        taco.rightDrive.setTargetPosition(-distance);
        taco.sleftDrive.setTargetPosition(-distance);
        taco.srightDrive.setTargetPosition(-distance);

        taco.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        taco.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        taco.sleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        taco.srightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        taco.leftDrive.setPower(pwr);
        taco.rightDrive.setPower(pwr);
        taco.sleftDrive.setPower(pwr);
        taco.srightDrive.setPower(pwr);

        while (taco.leftDrive.isBusy() && taco.rightDrive.isBusy() && taco.sleftDrive.isBusy() && taco.srightDrive.isBusy()) {
            telemetry.addData("encoderCountL", taco.leftDrive.getCurrentPosition());
            telemetry.addData("encoderCountR", taco.rightDrive.getCurrentPosition());
            telemetry.addData("encoderCountLS", taco.sleftDrive.getCurrentPosition());
            telemetry.addData("encoderCountRS", taco.srightDrive.getCurrentPosition());
            telemetry.update();
        }
        while (taco.leftDrive.isBusy() && taco.rightDrive.isBusy() && taco.sleftDrive.isBusy() && taco.srightDrive.isBusy()) {

        }
        stopDriving();

        taco.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        taco.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        taco.srightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        taco.sleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}

