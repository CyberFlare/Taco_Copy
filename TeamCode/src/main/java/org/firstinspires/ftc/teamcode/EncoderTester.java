package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.BatteryChecker;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class EncoderTester extends OpMode {

    DcMotor Motor0; //chassis
    DcMotor Motor1;
    DcMotor Motor2;
    DcMotor Motor3;
    //BatteryChecker.BatteryStatus Battery1;

    DcMotor Linearout0; //linear pivots out
    DcMotor Linearup1; //linear move up or down
    DcMotor Intake2; //intake
    DcMotor Intake3; //intake

    CRServo Intake4; //for intake up and down
    Servo arm3; //for claw realignment
    Servo Claw; //close and opens stone grab thing
    Servo Foundation; //moves foundation
    Servo Stone; //pushes stone in

    DigitalChannel Touch;
    ColorSensor Color1;
    ColorSensor Color2;
    Boolean elevatorCheck = true;


    public void loop(){
        Motor1.getCurrentPosition();
        Motor2.getCurrentPosition();
        Motor3.getCurrentPosition();
        Motor0.getCurrentPosition();

        Linearout0.getCurrentPosition();
        Linearup1.getCurrentPosition();

        telemetry.addData("Motor0:", Motor0.getCurrentPosition());
        telemetry.addData("Motor1:", Motor1.getCurrentPosition());
        telemetry.addData("Motor2:", Motor2.getCurrentPosition());
        telemetry.addData("Motor3:", Motor3.getCurrentPosition());
        telemetry.addData("Linearout0:", Linearout0.getCurrentPosition());
        telemetry.addData("Linearup1:", Linearup1.getCurrentPosition());
        telemetry.update();

        if (gamepad1.a){
            Motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Linearout0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Linearup1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

    }

    public void init() {


        Motor0 = hardwareMap.dcMotor.get ("Motor0"); //Chassis
        Motor1 = hardwareMap.dcMotor.get ("Motor1");
        Motor2 = hardwareMap.dcMotor.get ("Motor2");
        Motor3 = hardwareMap.dcMotor.get ("Motor3");

        Linearout0 = hardwareMap.dcMotor.get ("Linear0"); //Big Boi
        Linearup1 = hardwareMap.dcMotor.get ("Linear1");
        Intake2 = hardwareMap.dcMotor.get ("Intake2");
        Intake3 = hardwareMap.dcMotor.get ("Intake3");

        Intake4 = hardwareMap.crservo.get ("Intakes2"); //big boi
        Stone = hardwareMap.servo.get ("Stone4");

        Claw = hardwareMap.servo.get ("Claw5"); //chassis
        arm3 = hardwareMap.servo.get ("arm3");
        Foundation = hardwareMap.servo.get ("Foundation2");

        Touch = hardwareMap.get (DigitalChannel.class, "touchboi"); //Chassis
        //Color1 = hardwareMap.colorSensor.get ("bcolor");
        //Color2 = hardwareMap.colorSensor.get ("scolor");

        Linearout0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Linearup1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Linearout0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Linearup1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);




        //Battery1 = hardwareMap.battery ("g");
    }

}
