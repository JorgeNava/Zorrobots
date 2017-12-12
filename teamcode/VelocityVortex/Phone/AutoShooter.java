package org.firstinspires.ftc.teamcode.VelocityVortex.Phone;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Jorge on 08/12/2016.
 */
//@Autonomous(name = "AutoShooter")
public class AutoShooter extends LinearOpMode
{
    private DcMotor motorLeftF;
    private DcMotor motorLeftB;
    private DcMotor motorRightF;
    private DcMotor motorRightB;
    private DcMotor LauncherR;
    private DcMotor LauncherL;
    private DcMotor banda;
    private DcMotor aspas;
    private Servo ServoR;
    private Servo ServoL;
    private ColorSensor LineSensor;
    private ColorSensor BeaconSensor;
    private OpticalDistanceSensor WallSensor;

    String white = "white";
    String blue = "blue";
    String red = "red";
    String green = "green";
    String black = "black";
    String sColor="";


    @Override
    public void runOpMode() throws InterruptedException
    {
        motorLeftB = hardwareMap.dcMotor.get("motorLeftB");
        motorLeftF = hardwareMap.dcMotor.get("motorLeftF");
        motorRightB = hardwareMap.dcMotor.get("motorRightB");
        motorRightF = hardwareMap.dcMotor.get("motorRightF");
        LauncherL = hardwareMap.dcMotor.get("LauncherL");
        LauncherR = hardwareMap.dcMotor.get("LauncherR");
        banda = hardwareMap.dcMotor.get("banda");
        aspas = hardwareMap.dcMotor.get("aspas");
        LineSensor = hardwareMap.colorSensor.get("LineSensor");
        BeaconSensor = hardwareMap.colorSensor.get("BeaconSensor");
        WallSensor = hardwareMap.opticalDistanceSensor.get("WallSensor");
        motorLeftB.setDirection(DcMotor.Direction.REVERSE);
        motorLeftF.setDirection(DcMotor.Direction.REVERSE);
        LauncherL.setDirection(DcMotor.Direction.REVERSE);
        LineSensor.enableLed(true);
        BeaconSensor.enableLed(false);


        waitForStart();

        Avanzar(1); //GET CLOSER TO THE VORTEX
        sleep(950);
        Frenar();

        Launchers(1); // CHARGE LAUNCHER MOTORS FOR 2 SEC
        sleep(1000);
        SubirPelotas(1); //RISE AND SHOOT PARTICLES
        sleep(7000);
        NoDisparar(0); //STOP SHOOTING

        sleep(500);

        Avanzar(1); //MOVE CAP BALL
        sleep(800);
        Frenar();


    }
    //LAUNCHER
    private void Launchers (double power){
        LauncherL.setPower(power);
        LauncherR.setPower(power);
    }

    //RISE PARTICLES
    private void SubirPelotas (double power) {
        banda.setPower(-power);
        aspas.setPower(-power);
    }

    //TURN RIGHT
    public void GirarDerecha(double power){
        motorRightB.setPower(power);
        motorRightF.setPower(power);
        motorLeftB.setPower(-power);
        motorLeftF.setPower(-power);
    }

    //TURN LEFT
    public void GirarIzquierda(double power){
        motorRightB.setPower(-power);
        motorRightF.setPower(-power);
        motorLeftB.setPower(power);
        motorLeftF.setPower(power);
    }

    // GO FORWARD
    public void Avanzar(double power){
        motorRightB.setPower(-power);
        motorRightF.setPower(-power);
        motorLeftB.setPower(-power);
        motorLeftF.setPower(-power);
    }

    // GO BACKWARDS
    public void Reversa(double power){
        motorRightB.setPower(power);
        motorRightF.setPower(power);
        motorLeftB.setPower(power);
        motorLeftF.setPower(power);
    }

    //STOP SHOOTING
    public void NoDisparar(double power){
        Launchers(0);
        SubirPelotas(0);
    }

    //STOP ROBOT
    public void Frenar(){
        Avanzar(0);
        Reversa(0);
        GirarDerecha(0);
        GirarIzquierda(0);
    }


}
