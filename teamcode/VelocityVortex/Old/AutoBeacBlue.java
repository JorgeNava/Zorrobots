package org.firstinspires.ftc.teamcode.VelocityVortex.Old;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
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
//@Autonomous(name = "Prueba_1")
public class AutoBeacBlue extends LinearOpMode {
    private DcMotor motorLeftF;
    private DcMotor motorLeftB;
    private DcMotor motorRightF;
    private DcMotor motorRightB;
    private DcMotor LauncherR;
    private DcMotor LauncherL;
    private DcMotor banda;
    private DcMotor aspas;
    Servo ServoR;
    Servo ServoL;
    private ColorSensor LineSensor;
    private ColorSensor BeaconSensor;
    private OpticalDistanceSensor WallSensor;
    private GyroSensor GyroSensor;

    String white = "white";
    String blue = "blue";
    String red = "red";
    String black = "black";
    String sColor = "";
    String sColorB = "";
    String Multi;


    static double odsReadngRaw;
    static double odsReadingLinear;
    static int odsEstimatedDistance;
    static double m = 56;
    static double b = -0.356;
    static double currentHeading;



    @Override
    public void runOpMode() throws InterruptedException {
        motorLeftB = hardwareMap.dcMotor.get("motorLeftB");
        motorLeftF = hardwareMap.dcMotor.get("motorLeftF");
        motorRightB = hardwareMap.dcMotor.get("motorRightB");
        motorRightF = hardwareMap.dcMotor.get("motorRightF");
        LauncherL = hardwareMap.dcMotor.get("LauncherL");
        LauncherR = hardwareMap.dcMotor.get("LauncherR");
        ServoL = hardwareMap.servo.get("ServoL");
        ServoR = hardwareMap.servo.get("ServoR");
        banda = hardwareMap.dcMotor.get("banda");
        aspas = hardwareMap.dcMotor.get("aspas");
        ServoL = hardwareMap.servo.get("ServoL");
        ServoR = hardwareMap.servo.get("ServoR");
        BeaconSensor = hardwareMap.colorSensor.get("BeaconSensor");
        BeaconSensor.setI2cAddress(I2cAddr.create7bit(0x26));//4c
        LineSensor = hardwareMap.colorSensor.get("LineSensor");
        LineSensor.setI2cAddress(I2cAddr.create7bit(0x1e));//3c
        WallSensor = hardwareMap.opticalDistanceSensor.get("WallSensor");
        GyroSensor = hardwareMap.gyroSensor.get("GyroSensor");
        motorLeftB.setDirection(DcMotor.Direction.REVERSE);
        motorLeftF.setDirection(DcMotor.Direction.REVERSE);
        LauncherL.setDirection(DcMotor.Direction.REVERSE);
        LineSensor.enableLed(false);
        BeaconSensor.enableLed(false);

        waitForStart();
        Multi = "Start";
        while (Multi.equals("Start")){
            RevisarGiro9();
        }

    }

    //HERRAMIENTAS
    private void Launchers(double power) {
        LauncherL.setPower(power);
        LauncherR.setPower(power);
    }

    private void SubirPelotas(double power) {
        banda.setPower(-power);
        aspas.setPower(-power);
    }

    //VUELTAS
    public void GirarDerecha(double power) {
        motorRightB.setPower(power);
        motorRightF.setPower(power);
        motorLeftB.setPower(-power);
        motorLeftF.setPower(-power);
    }

    public void GirarIzquierda(double power) {
        motorRightB.setPower(-power);
        motorRightF.setPower(-power);
        motorLeftB.setPower(power);
        motorLeftF.setPower(power);
    }

    //MANEJO
    public void Avanzar(double power) {
        motorRightB.setPower(-power);
        motorRightF.setPower(-power);
        motorLeftB.setPower(-power);
        motorLeftF.setPower(-power);
    }

    public void Reversa(double power) {
        motorRightB.setPower(power);
        motorRightF.setPower(power);
        motorLeftB.setPower(power);
        motorLeftF.setPower(power);
    }

    //CANCELACIONES
    public void NoDisparar() {
        Launchers(0);
        SubirPelotas(0);
    }

    public void Frenar() {
        Avanzar(0);
        Reversa(0);
    }

    //REVISAR COLORES
    public void RevisarColorBeacon() {
        float hsvValues[] = {0, 0, 0};

        Color.RGBToHSV(BeaconSensor.red() * 8, BeaconSensor.green() * 8, BeaconSensor.blue() * 8, hsvValues);
        if (BeaconSensor.blue() > BeaconSensor.red() && BeaconSensor.blue() > BeaconSensor.green()) {
            telemetry.addData("Color", blue);
            telemetry.update();
            sColorB = blue;
        } else {
            telemetry.addData("Color", red);
            telemetry.update();
            sColorB = red;
        }
    }

    public void RevisarColorLinea(){
        float hsvValues[] = {0,0,0};

        Color.RGBToHSV(LineSensor.red() *8, LineSensor.green() *8, LineSensor.blue() *8, hsvValues);
        if (LineSensor.blue() > 0 && LineSensor.red() > 0 && LineSensor.green() > 0) {
            telemetry.addData("Color", white);
            telemetry.update();
            sColor = white;
        }
        else {
            telemetry.addData("Color", black);
            telemetry.update();
            sColor = black;
        }
    }

    public void RevisarDistancia(){
        telemetry.addData("Distance", odsEstimatedDistance);
        telemetry.update();
    }

    //USO DE SENSORES

    public void Todo(){
        //Avanzar hasta Tape
        RevisarColorLinea();
        LineSensor.enableLed(true);
        while (sColor.equals(black)) {
            Reversa(.2);
            RevisarColorLinea();
        }
        Frenar();

        sleep(1000);

        //Acomodarse
        GirarDerecha(.5);
        sleep(450);
        Frenar();

        sleep(1000);

        //Avanzar
        Avanzar(.3);
        sleep(350);
        Frenar();

        sleep(500);

        //Lanzar Pelotas
        Launchers(1);
        sleep(1500);
        SubirPelotas(1);
        sleep(4000);
        NoDisparar();

        sleep(2000);

        Multi = "Start";
        telemetry.addLine(Multi);
        telemetry.update();


    }

    public void AvanzarHastaDistanciaX()    {
        odsReadngRaw = WallSensor.getRawLightDetected();
        odsReadingLinear = Math.pow(odsReadngRaw, -0.5);
        odsEstimatedDistance = (int) ((m * odsReadingLinear) + b);

        while (Multi.equals("Start") && odsEstimatedDistance > 145){
            odsReadngRaw = WallSensor.getRawLightDetected();
            odsReadingLinear = Math.pow(odsReadngRaw, -0.5);
            odsEstimatedDistance = (int) ((m * odsReadingLinear) + b);
            Avanzar(-.2);
        } //AL LLEGAR A LA POSICION LOS MOTORES SE FRENAN E INICIA UNA NUEVA ETAPA
        motorLeftB.setPower(0);
        motorLeftF.setPower(0);
        motorRightB.setPower(0);
        motorRightF.setPower(0);
        Multi = "Start2";
        telemetry.addLine(Multi);
        telemetry.update();
        sleep(1000);
        //INICIO DE NUEVA ETAPA PARA PARA PICAR BEACON
    }

    public void PicarBeacon() {
        while (Multi.equals("Start2")) {
            BeaconSensor.enableLed(false);
            RevisarColorBeacon();
            if (sColorB.equals(blue)) {
                Avanzar(.1);
                sleep(1000);
                Frenar();
                ServoR.setPosition(.5);
                ServoL.setPosition(0);
                Reversa(.1);
                sleep(1200);
                Frenar();
                Multi = "Start3";
                telemetry.addLine(Multi);
                telemetry.update();
            }
            else {
                Avanzar(.1);
                sleep(1000);
                Frenar();
                ServoR.setPosition(1);
                ServoL.setPosition(.6);
                Reversa(.1);
                sleep(1200);
                Frenar();
                Multi = "Start3";
                telemetry.addLine(Multi);
                telemetry.update();
            }
        }
    }

    public void TirarCapBall(){
        sleep(1000);

        Avanzar(.5);
        sleep(1300);
        Frenar();

        GirarIzquierda(.5);
        sleep(800);
        Frenar();

        GirarDerecha(.5);
        sleep(850);
        Frenar();

        Avanzar(.3);
        sleep(600);
        Frenar();

        telemetry.addLine("!!!GANAMOS EQUIPO YA HICIMOS 70 PUTNOS!!!");
        telemetry.update();
    }

    public void RevisarGiro9 (){

        GyroSensor.getHeading();
        currentHeading = GyroSensor.getHeading ();
        telemetry.addData("current Heading ", currentHeading);
        telemetry.update();

    }

}
