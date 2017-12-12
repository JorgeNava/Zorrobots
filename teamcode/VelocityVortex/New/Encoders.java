package org.firstinspires.ftc.teamcode.VelocityVortex.New;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Jorge on 10/01/2017.
 */
//@Autonomous(name = "Encoders")
public class Encoders extends LinearOpMode {

    DcMotor motorRightF;
    DcMotor motorRightB;
    DcMotor motorLeftF;
    DcMotor motorLeftB;
  //  DcMotor Motor1;

    @Override
    public void runOpMode() throws InterruptedException {

        motorRightF = hardwareMap.dcMotor.get("motorRightF");
        motorRightB = hardwareMap.dcMotor.get("motorRightB");
        motorLeftF = hardwareMap.dcMotor.get("motorLeftF");
        motorLeftB = hardwareMap.dcMotor.get("motorLeftB");
        // Motor1 = hardwareMap.dcMotor.get("Motor1");

        motorLeftF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        AvanzarHastaDistanciaX(1,1440*4);

    }
    public void AvanzarHastaDistanciaX(double power, int distance){

        //Reset Encoders
        motorLeftF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Set Target Position
        motorLeftF.setTargetPosition(distance);
        motorLeftB.setTargetPosition(distance);
        motorRightF.setTargetPosition(distance);
        motorRightB.setTargetPosition(distance);
        //Motor1.setTargetPosition(distance);

        //Set to Run To Position
        motorLeftF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //Motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Avanzar(power);

        while (motorLeftF.isBusy() && motorLeftB.isBusy() && motorRightB.isBusy() && motorRightF.isBusy()){
            //wait until target position is reached
        }

        //stop and change modes back to normal
        Frenar();

        motorLeftF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void GirarDerecha(double power){
        motorRightB.setPower(power);
        motorRightF.setPower(power);
        motorLeftB.setPower(-power);
        motorLeftF.setPower(-power);
    }
    public void GirarIzquierda(double power){
        motorRightB.setPower(-power);
        motorRightF.setPower(-power);
        motorLeftB.setPower(power);
        motorLeftF.setPower(power);
    }
    //MANEJO
    public void Avanzar(double power){
        motorRightB.setPower(-power);
        motorRightF.setPower(-power);
        motorLeftB.setPower(-power);
        motorLeftF.setPower(-power);
    }
    public void Reversa(double power){
        motorRightB.setPower(power);
        motorRightF.setPower(power);
        motorLeftB.setPower(power);
        motorLeftF.setPower(power);
    }
    public void Frenar(){
        Avanzar(0);
        Reversa(0);
    }

}
