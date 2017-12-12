package org.firstinspires.ftc.teamcode.VelocityVortex.GyroSensor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
/*
1cm = 1022  6cm = 60    11cm = 21
2cm = 440   7cm = 45    12cm = 17
3cm = 225   8cm = 35    13cm = 15
4cm = 130   9cm = 30    14cm = 13
5cm = 85    10cm =24    15cm = 12
 */

/**
 * Created by Jorge on 08/12/2016.
 */
//@Autonomous(name = "GyroTest_1")
public class GyroTest_1 extends LinearOpMode {
    private DcMotor motorLeftF;
    private DcMotor motorLeftB;
    private DcMotor motorRightF;
    private DcMotor motorRightB;
    private GyroSensor GyroSensor;
    String Multi;

    @Override
    public void runOpMode() throws InterruptedException {
        motorLeftB = hardwareMap.dcMotor.get("motorLeftB");
        motorLeftF = hardwareMap.dcMotor.get("motorLeftF");
        motorRightB = hardwareMap.dcMotor.get("motorRightB");
        motorRightF = hardwareMap.dcMotor.get("motorRightF");
        GyroSensor = hardwareMap.gyroSensor.get("GyroSensor");
        motorLeftB.setDirection(DcMotor.Direction.REVERSE);
        motorLeftF.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        double currentHeading = GyroSensor.getHeading();
        while (currentHeading < 350) {
            GirarDerecha(.2);
            currentHeading = GyroSensor.getHeading();
            telemetry.addData("Heading ", currentHeading);
            telemetry.update();
        }
        Frenar();
    }
        /* while (currentHeading > 308) {
            GirarIzquierda(.2);
          //  GirarIzquierda(.18);
          //  sleep(50);
          //  Frenar();
            currentHeading = GyroSensor.getHeading();
            telemetry.addData("Heading ", currentHeading);
            telemetry.update();
        }*/


    public void Todo1 (){
        double currentHeading = GyroSensor.getHeading();
        while (currentHeading >= 220){
            GirarIzquierda(.2);
            currentHeading = GyroSensor.getHeading();
            telemetry.addData("Heading ", currentHeading);
            telemetry.update();
        }
        Frenar();
        telemetry.addLine("Alineado");
        telemetry.update();
        sleep(5000);

        Multi = "Start1";
    }

    public void Todo2 (){
        while (Multi.equals("Start1")){
            double currentHeading = GyroSensor.getHeading();
            while (currentHeading <= 300){
                GirarDerecha(.2);
                currentHeading = GyroSensor.getHeading();
                telemetry.addData("Heading ", currentHeading);
                telemetry.update();
            }
            Frenar();
            telemetry.addLine("Alineado");
            telemetry.update();
            sleep(1000);
            currentHeading = GyroSensor.getHeading();
            telemetry.addData("Heading ", currentHeading);
            telemetry.update();
            sleep(5000);

            Multi = "Start2";
        }
    }

    public void GirarIzquierda(double power) {
        motorRightB.setPower(-power);
        motorRightF.setPower(-power);
        motorLeftB.setPower(power);
        motorLeftF.setPower(power);
    }

    public void GirarDerecha(double power){
        motorRightB.setPower(power);
        motorRightF.setPower(power);
        motorLeftB.setPower(-power);
        motorLeftF.setPower(-power);
    }

    public void Frenar() {

        GirarIzquierda(0);
    }


}
