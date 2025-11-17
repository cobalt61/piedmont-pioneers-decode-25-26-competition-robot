 /* Copyright (c) 2017 FIRST. All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted (subject to the limitations in the disclaimer below) provided that
  * the following conditions are met:
  *
  * Redistributions of source code must retain the above copyright notice, this list
  * of conditions and the following disclaimer.
  *
  * Redistributions in binary form must reproduce the above copyright notice, this
  * list of conditions and the following disclaimer in the documentation and/or
  * other materials provided with the distribution.
  *
  * Neither the name of FIRST nor the names of its contributors may be used to endorse or
  * promote products derived from this software without specific prior written permission.
  *
  * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
  * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
  * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  */

 package org.firstinspires.ftc.teamcode.opmode.autonomous;
 import static java.lang.Thread.sleep;

 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.hardware.DcMotor;
 import com.qualcomm.robotcore.hardware.DcMotorSimple;
 import com.qualcomm.robotcore.util.ElapsedTime;

 import org.firstinspires.ftc.teamcode.hardware.Globals;
 import org.firstinspires.ftc.teamcode.hardware.robot.Config;
 import org.firstinspires.ftc.teamcode.*;

public class Base extends LinearOpMode {
    /* Declare OpMode members. */

    protected DcMotor leftFrontDrive;
    protected DcMotor rightFrontDrive;
    protected DcMotor leftBackDrive;
    protected DcMotor rightBackDrive;
    protected DcMotor bottomRoller, indexer, topFlywheel, intakeMotor, bottomFlywheel;

    public Config config;
    protected final ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    private void forward(int milseconds) {
        ElapsedTime forwardTime = new ElapsedTime();
        while (forwardTime.milliseconds() < milseconds) {
            leftFrontDrive.setPower(Globals.Auto.FORWARD_SPEED);
            rightFrontDrive.setPower(Globals.Auto.FORWARD_SPEED);
            leftBackDrive.setPower(Globals.Auto.FORWARD_SPEED);
            rightBackDrive.setPower(Globals.Auto.FORWARD_SPEED);
        }
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }


    private void right(int Degrees) {
        ElapsedTime rightTurnTime = new ElapsedTime();
        while (rightTurnTime.milliseconds() < Degrees) {
            leftFrontDrive.setPower(Globals.Auto.TURN_SPEED);
            rightFrontDrive.setPower(-Globals.Auto.TURN_SPEED);
            leftBackDrive.setPower(Globals.Auto.TURN_SPEED);
            rightBackDrive.setPower(-Globals.Auto.TURN_SPEED);
        }
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    private void left(int turn) {
        ElapsedTime leftTurnTime = new ElapsedTime();
        while (leftTurnTime.milliseconds() < turn) {
            leftFrontDrive.setPower(-Globals.Auto.TURN_SPEED);
            rightFrontDrive.setPower(Globals.Auto.TURN_SPEED);
            leftBackDrive.setPower(-Globals.Auto.TURN_SPEED);
            rightBackDrive.setPower(Globals.Auto.TURN_SPEED);
        }
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

    }

    private void backward(int mS) {
        ElapsedTime backwardTime = new ElapsedTime();

        while (backwardTime.milliseconds() <= mS) {
            leftFrontDrive.setPower(-Globals.Auto.FORWARD_SPEED);
            rightFrontDrive.setPower(-Globals.Auto.FORWARD_SPEED);
            leftBackDrive.setPower(-Globals.Auto.FORWARD_SPEED);
            rightBackDrive.setPower(-Globals.Auto.FORWARD_SPEED);
        }
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    public void index() {
        indexer.setPower(Globals.Intake.POWER_ON);
    }

    public void runIntake() {
        intakeMotor.setPower(Globals.Intake.POWER_ON);
        bottomRoller.setPower(Globals.Intake.POWER_ON);
    }

    public void stopIntake() {
        intakeMotor.setPower(Globals.Intake.POWER_OFF);
        bottomRoller.setPower(Globals.Intake.POWER_OFF);

    }

    public void outtake() {
        topFlywheel.setPower(Globals.Outtake.POWER_ON);
        bottomFlywheel.setPower(Globals.Outtake.POWER_ON);
        sleep(300);
        indexer.setPower(Globals.Intake.POWER_ON);
        sleep(250);
        topFlywheel.setPower(Globals.Outtake.POWER_OFF);
        bottomFlywheel.setPower(Globals.Outtake.POWER_OFF);
        indexer.setPower(Globals.Intake.POWER_OFF);
    }

    public void runOpMode() {

    }

    protected void internalRunMode() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, Globals.Auto.LEFT_FRONT_DRIVE);
        rightFrontDrive = hardwareMap.get(DcMotor.class, Globals.Auto.RIGHT_FRONT_DRIVE);
        leftBackDrive = hardwareMap.get(DcMotor.class, Globals.Auto.LEFT_BACK_DRIVE);
        rightBackDrive = hardwareMap.get(DcMotor.class, Globals.Auto.RIGHT_BACK_DRIVE);
        intakeMotor = config.hardwareMap.get(DcMotor.class, Globals.Intake.INTAKE_MOTOR);
        indexer = config.hardwareMap.get(DcMotor.class, Globals.Intake.INDEXER);
        bottomRoller = config.hardwareMap.get(DcMotor.class, Globals.Intake.BOTTOM_INTAKE_ROLLER);
        topFlywheel = config.hardwareMap.get(DcMotor.class, Globals.Outtake.TOP_FLYWHEEL);
        bottomFlywheel = config.hardwareMap.get(DcMotor.class, Globals.Outtake.BOTTOM_FLYWHEEL);

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        indexer.setDirection(DcMotorSimple.Direction.REVERSE);
        indexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomRoller.setDirection(DcMotorSimple.Direction.FORWARD);
        bottomRoller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        topFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        topFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        bottomFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    protected enum Combo {
        PPG(),
        PGP(),
        GPP(),

    }
    protected enum Color {
        red(),
        blue(),
    }

    protected void startFromBack(Color currentColor, Combo currentCombo) {
        switch (currentCombo) {
            case PPG:
                forward(500);
            break;
            case PGP:
                forward(600);
            break;
            case GPP:
                forward(700);
            break;


        }
        switch (currentColor) {
            case red:
                right(500);
            break;
            case blue:
                left(500);
            break;

        }

        runIntake();
        forward(150);
        stopIntake();
        switch (currentColor) {
            case red:
                right(400);
            break;
            case blue:
                left(400);
            break;
        }
        outtake();

    }

}
