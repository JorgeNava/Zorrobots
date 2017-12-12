package org.firstinspires.ftc.teamcode.VelocityVortex.ColorSensor;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Jorge on 08/12/2016.
 */
//@Autonomous(name = "PruebaTape")
public class PruebaTape extends LinearOpMode
{
    private DcMotor motorLeftF;
    private DcMotor motorLeftB;
    private DcMotor motorRightF;
    private DcMotor motorRightB;
    private ColorSensor LineSensor;

    String white = "white";
    String blue = "blue";
    String red = "red";
    String green = "green";
    String black = "black";
    String sColor="";

    @Override
    public void runOpMode() throws InterruptedException {

        motorLeftB = hardwareMap.dcMotor.get("motorLeftB");
        motorLeftF = hardwareMap.dcMotor.get("motorLeftF");
        motorRightB = hardwareMap.dcMotor.get("motorRightB");
        motorRightF = hardwareMap.dcMotor.get("motorRightF");
        LineSensor = hardwareMap.colorSensor.get("LineSensor");

        motorLeftB.setDirection(DcMotor.Direction.REVERSE);
        motorLeftF.setDirection(DcMotor.Direction.REVERSE);
        LineSensor.enableLed(false);
        LineSensor.enableLed(true);

        waitForStart();
            RevisarColor();
            while (sColor.equals(black)) {
                Avanzar(.2);
                RevisarColor();
            }
            Frenar();
        }

    public void Avanzar(double power){
        motorLeftB.setPower(power);
        motorLeftF.setPower(power);
        motorRightB.setPower(power);
        motorRightF.setPower(power);
    }
    public void Frenar()
    {
        Avanzar(0);
    }
    public void RevisarColor (){
        float hsvValues[] = {0,0,0};

        Color.RGBToHSV(LineSensor.red()*8, LineSensor.green()*8,LineSensor.blue()*8,hsvValues);
        if (LineSensor.blue()>0 && LineSensor.red()> 0 && LineSensor.green() > 0){
            telemetry.addData("Color", white);
            sColor=white;
        }
        else if (LineSensor.blue()>LineSensor.red()&& LineSensor.blue()>LineSensor.green()){
            telemetry.addData("Color", blue);
            sColor=blue;
        }
        else if (LineSensor.red()>LineSensor.blue()&& LineSensor.red()>LineSensor.green()){
            telemetry.addData("Color", red);
            sColor=red;
        }
        else if (LineSensor.green()>LineSensor.blue()&& LineSensor.green()>LineSensor.red()){
            telemetry.addData("Color", green);
            sColor=green;
        }
        else {
            telemetry.addData("Color", black);
            sColor=black;
        }


    }

}
