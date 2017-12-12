package org.firstinspires.ftc.teamcode.VelocityVortex.New;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Jorge on 10/01/2017.
 */
//@Autonomous(name = "Encoders_2")
public class Encoders_2 extends LinearOpMode {

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
        GirarDerecha(.3);
        sleep(1000);
        Frenar();

        Avanzar4v();

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
    public void Avanzar4v(){
        motorRightB.setTargetPosition(1440*4);// 4 complete rotations of the motor shaft
        motorRightF.setTargetPosition(1440*4);
        motorLeftB.setTargetPosition(1440*4);
        motorLeftF.setTargetPosition(1440*4);

        motorLeftF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Avanzar(1);

        while (motorLeftF.isBusy() && motorLeftB.isBusy() && motorRightB.isBusy() && motorRightF.isBusy()){
            //wait until target position is reached
        }

        //stop and change modes back to normal
        Frenar();

        motorLeftF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}
