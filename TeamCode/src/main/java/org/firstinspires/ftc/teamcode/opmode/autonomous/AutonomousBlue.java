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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Globals;

/*
 * This OpMode illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Robot: Auto Drive By Time", group="Robot")


public class AutonomousBlue extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;

    private final ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

// is cod!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    private void Forward(double mseconds){
        runtime.reset();
        while  (runtime.seconds() < mseconds) {
            leftFrontDrive.setPower(-Globals.BigBack.FORWARD_SPEED);
            rightFrontDrive.setPower(Globals.BigBack.FORWARD_SPEED);
            leftBackDrive.setPower(-Globals.BigBack.FORWARD_SPEED);
            rightBackDrive.setPower(Globals.BigBack.FORWARD_SPEED);

        }
    }

    private void Right(int Degrees){
        runtime.reset();
        while  (runtime.seconds() < Degrees){
            leftFrontDrive.setPower(Globals.BigBack.TURN_SPEED);
            rightFrontDrive.setPower(-Globals.BigBack.TURN_SPEED);
            leftBackDrive.setPower(Globals.BigBack.TURN_SPEED);
            rightBackDrive.setPower(-Globals.BigBack.TURN_SPEED);
        }
    }
    private void Left(int turn){
        runtime.reset();
        while  (runtime.seconds() < turn) {
            leftFrontDrive.setPower(-Globals.BigBack.TURN_SPEED);
            rightFrontDrive.setPower(Globals.BigBack.TURN_SPEED);
            leftBackDrive.setPower(-Globals.BigBack.TURN_SPEED);
            rightBackDrive.setPower(Globals.BigBack.TURN_SPEED);
        }
    }
    private void Backward(int distance)
    {runtime.reset();
    while  (runtime.seconds() < distance){
        leftFrontDrive.setPower(Globals.BigBack.FORWARD_SPEED);
        rightFrontDrive.setPower(-Globals.BigBack.FORWARD_SPEED);
        leftBackDrive.setPower(Globals.BigBack.FORWARD_SPEED);
        rightBackDrive.setPower(-Globals.BigBack.FORWARD_SPEED);
        }
    }

    private void Input(boolean on) {


    }


    public void runOpMode() {

        // Initialize the drive system variables.
        leftFrontDrive = hardwareMap.get(DcMotor.class, Globals.BigBack.LEFT_FRONT_DRIVE);
        rightFrontDrive = hardwareMap.get(DcMotor.class, Globals.BigBack.RIGHT_FRONT_DRIVE);
        leftBackDrive = hardwareMap.get(DcMotor.class, Globals.BigBack.LEFT_BACK_DRIVE);
        rightBackDrive = hardwareMap.get(DcMotor.class, Globals.BigBack.RIGHT_BACK_DRIVE);


        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        waitForStart();



    }
}
