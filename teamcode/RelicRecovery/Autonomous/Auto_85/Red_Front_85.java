package org.firstinspires.ftc.teamcode.RelicRecovery.Autonomous.Auto_85;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


/**
 * Created by Jorge on 23/11/2017.
 */
//@Autonomous(name="Red_Front_85")
public class Red_Front_85 extends LinearOpMode {
    //WHEELS
    public DcMotor Left_motors = null;
    public DcMotor Right_motors = null;
    public DcMotor Lifter = null;

    //SERVOS
    public Servo RightJServo = null;
    public Servo LeftJServo = null;
    public Servo RightCServo = null;
    public Servo LeftCServo = null;

    public String Multi;
    public String JewelColor;
    public String blue = "blue";
    public String red = "red";

    //SENSORS
    public ColorSensor LineSensor;
    public ColorSensor JewelSensor;
    public DistanceSensor RevDistanceSensor;
    public ColorSensor RevColorSensor;



    //VUFORIA
    VuforiaLocalizer vuforia;
    @Override
    public void runOpMode() throws InterruptedException {
        //HARDWARE MAPPING
        Left_motors = hardwareMap.dcMotor.get("Left_motors");
        Right_motors = hardwareMap.dcMotor.get("Right_motors");
        Lifter = hardwareMap.dcMotor.get("Lifter");

        Right_motors.setDirection(DcMotor.Direction.REVERSE);

        RightJServo = hardwareMap.servo.get("RightJServo");
        LeftJServo = hardwareMap.servo.get("LeftJServo");
        RightCServo = hardwareMap.servo.get("RightCServo");
        LeftCServo = hardwareMap.servo.get("LeftCServo");

        //VUFORIA INICIO
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AYy3kgb/////AAAAGfahknPNfEOHpk9drxT3x5s+h85enQDuwX5Y/R9chthrPe1AQ1A+iYyS9PoUpVOVcu4TM/lzJa/PqlyaHKJWh+fI63xLIftsjqQ15b+MoQNZrgG4sw0swD9/yYSfSn3AU6PuQ6OozHZf4zrEOiL2AL/1OMMxbd9KddgiIIX5X/rnx7VFMFiNR8vq+otCHameCqnzdRcCkp1rqo+bewMyMYjTeYIyl29wn0oElYjg1PdBoYgDiUIjQu4sVECgCH7c6+pmEYe37ypfeMCxoGmG60L8bUmq5RrzZ1mxdJkugZ4hRbG/UIm1aHApSHE+ljsAexK3crM78qRfdVK6B9PTsnEEq9C40FuYu/ZqcCglO5VZ\n";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        relicTrackables.activate();
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        while (!isStopRequested() && vuMark == RelicRecoveryVuMark.UNKNOWN)  {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            telemetry.addData("VuMark", "%s visible", vuMark);
            sleep(250);
            idle();
        }

        if (vuMark == RelicRecoveryVuMark.LEFT){
            Multi = "LEFT";
        }
        else if (vuMark == RelicRecoveryVuMark.RIGHT){
            Multi = "RIGHT";
        }
        else if (vuMark == RelicRecoveryVuMark.CENTER){
            Multi = "CENTER";
        }
        
        telemetry.addData(">", Multi);
        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        waitForStart();
        GrabCubeandLift(.5,.5,1, 800);
        HitJewel(.7, 0);
        VuMarkRunToCryptoBox();
    }
    //FUNCTIONS
    public void Forward (double power){
        Right_motors.setPower(power);
        Left_motors.setPower(power);
    }
    public void Reverse(double power){
        Forward(-power);
    }
    public void LeftTurn(double power){
        Right_motors.setPower(power);
        Left_motors.setPower(-power);
    }
    public void RightTurn(double power){
        Right_motors.setPower(-power);
        Left_motors.setPower(power);
    }
    public void VuMarkCheckandRun(){
        if (Multi=="LEFT") {
            telemetry.addLine("Izquierda");
            telemetry.update();
            Forward(.3);
            sleep(500);
            Forward(0);
            sleep(1500);
        }
        else if (Multi=="RIGHT") {
            telemetry.addLine("Derecha");
            telemetry.update();
            Forward(.3);
            sleep(500);
            Forward(0);
            sleep(1500);
        }
        if (Multi=="CENTER") {
            telemetry.addLine("Centro");
            telemetry.update();
            Forward(.3);
            sleep(500);
            Forward(0);
            sleep(1500);
        }
        else{
            telemetry.addLine("Fallaste en VuMarkCheck and Run");
            telemetry.update();
            Forward(0);
            sleep(1000);
        }
    }
    public void VuMarkCheck(){
        if (Multi=="LEFT") {
            telemetry.addLine("Izquierda");
            telemetry.update();
        }
        else if (Multi=="RIGHT") {
            telemetry.addLine("Derecha");
            telemetry.update();
        }
        if (Multi=="CENTER") {
            telemetry.addLine("Centro");
            telemetry.update();
        }
        else{
            telemetry.addLine("Fallaste en VuMarkCheck and Run");
            telemetry.update();
        }
    }
    public void GrabCubeandLift(double posR, double posL, double power, int time){
        RightCServo.setPosition(posR);
        LeftCServo.setPosition(posL);
        Lifter.setPower(power);
        sleep(time);
        telemetry.addLine(">  GrabCubeandLift phase complete");
    }
    public void HitJewel(double posD,double posU) {
        RightCServo.setPosition(posD);
        CheckJewelColor();
        if (JewelColor == "blue") {
            Forward(.2);
            sleep(200);
        } else if (JewelColor == "red") {
            Reverse(.2);
            sleep(200);
        } else {
            telemetry.addLine("ERROR ON HIT JEWEL");
        }
        RightCServo.setPosition(posU);
        telemetry.addLine(">  HitJewel phase complete");
    }
    public void CheckJewelColor(){
            float hsvValues[] = {0, 0, 0};

            Color.RGBToHSV(JewelSensor.red() * 8, JewelSensor.green() * 8, JewelSensor.blue() * 8, hsvValues);
            if (JewelSensor.blue() > JewelSensor.red() && JewelSensor.blue() > JewelSensor.green()) {
                telemetry.addData("Color", blue);
                telemetry.update();
                JewelColor = "blue";
            } else if (JewelSensor.red() > JewelSensor.blue() && JewelSensor.red() > JewelSensor.green()) {
                telemetry.addData("Color", red);
                telemetry.update();
                JewelColor = "red";
            }
        telemetry.addLine(">  CheckJewelColor phase complete");

    }
    public void RunToCryptoBoxNatural(double LftTrnPwr, int TurnTime, double FwrdPwr, int FwrdTime){
        LeftTurn(LftTrnPwr);
        sleep(TurnTime);
        Forward(FwrdPwr);
        sleep(FwrdTime);
        telemetry.addLine(">  RunToCryptoBoxNatural phase complete");
    }
    public void VuMarkRunToCryptoBox(){
        if (Multi=="LEFT") {
            telemetry.addLine("Moving to Left");
            telemetry.update();
            //SHIT TO MOVE TO LEFT CORNER
            //USE OF ENCODERS AND/OR GYRO SENSOR
        }
        else if (Multi=="RIGHT") {
            telemetry.addLine("Moving to Right");
            telemetry.update();
            //SHIT TO MOVE TO RIGHT CORNER
            //USE OF ENCODERS AND/OR GYRO SENSOR
        }
        if (Multi=="CENTER") {
            telemetry.addLine("Moving to Center");
            telemetry.update();
            //SHIT TO MOVE TO THE CENTER
            //USE OF ENCODERS AND/OR GYRO SENSOR
        }
        else{
            telemetry.addLine("ERROR on VuMarkRunToCryptoBox");
            telemetry.update();
        }
    }
}
