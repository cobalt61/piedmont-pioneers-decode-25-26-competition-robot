package org.firstinspires.ftc.teamcode.hardware.subsystem;

import android.provider.Settings;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.hardware.robot.Config;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.robot.enums.GameStage;

import java.util.ArrayList;
import java.util.List;

public class Outtake implements SubSystem {

    // Lift positions
    public enum OuttakeState {
        SPINNING_UP,
        INDEXING,
        STOPPED
    }


    // Constants for joystick thresholds
    private static final double JOYSTICK_THRESHOLD = 0.25;

    private final Config config;
    private DcMotor bottomFlywheel, topFlywheel, indexer;
    OuttakeState state;


    public Outtake(Config config) {
        this.config = config;
    }

    @Override
    public void init() {
        bottomFlywheel = config.hardwareMap.get(DcMotor.class, Globals.Outtake.BOTTOM_FLYWHEEL);
        topFlywheel = config.hardwareMap.get(DcMotor.class, Globals.Outtake.TOP_FLYWHEEL);
        indexer = config.hardwareMap.get(DcMotor.class, Globals.Outtake.INDEXER);



        // Set motor directions
        bottomFlywheel.setDirection(DcMotor.Direction.REVERSE);
        topFlywheel.setDirection(DcMotor.Direction.FORWARD);
        indexer.setDirection(DcMotor.Direction.REVERSE);

        // Reset encoders and set motor modes
        resetMotors();


    }

    @Override
    public void start() {}

    @Override
    public List<Action> update() {
        List<Action> newActions = new ArrayList<>();

        // Zero the lift if the back button is pressed
        if (config.gamepad1.right_trigger >= 0.1){
            newActions.add(runFlywheels(config.gamepad1.right_trigger));
        }
        else
        {
            newActions.add(stopFlywheels());
        }
        if (config.gamepad1.right_bumper) {
            newActions.add(runIndexer());
        }else{
            newActions.add(stopIndexer());
        }



/*
        // Handle joystick controls
        double leftStickY = -config.gamepad2.left_stick_y;
        double rightStickY = -config.gamepad2.right_stick_y;

        if (leftStickY >= JOYSTICK_THRESHOLD && position != LiftPosition.TOP_BASKET) {
            direction = LiftDirection.UP;
            newActions.add(bucket());
        } else if (leftStickY <= -JOYSTICK_THRESHOLD && position != LiftPosition.BOTTOM) {
            direction = LiftDirection.DOWN;
            newActions.add(down());
        }

        if (rightStickY >= JOYSTICK_THRESHOLD && position != LiftPosition.TOP_BAR) {
            direction = LiftDirection.UP;
            newActions.add(bar());
        } else if (rightStickY <= -JOYSTICK_THRESHOLD && position != LiftPosition.BOTTOM) {
            if (position != LiftPosition.CLIPPING) {
                direction = LiftDirection.DOWN;
                newActions.add(clip());
            } else {
                direction = LiftDirection.DOWN;
                newActions.add(down());
            }
        }
        */

        // Add telemetry data
        addTelemetryData();

        return newActions;
    }

    private void addTelemetryData() {
//        config.telemetry.addData("Right Lift Pos", right.getCurrentPosition());
//        config.telemetry.addData("Right Lift Power", right.getPower());
//        config.telemetry.addData("Left Lift Pos", left.getCurrentPosition());
//        config.telemetry.addData("Left Lift Power", left.getPower());
//        config.telemetry.addData("Lift Position", position);
//        config.telemetry.addData("Lift Direction", direction);
//        config.telemetry.addData("Switch", switchV.getState());
        config.telemetry.addData("Flywheel State:",state);
        config.telemetry.addData("Top Flywheel Position",topFlywheel.getCurrentPosition());
        config.telemetry.addData("Bottom Flywheel Position",bottomFlywheel.getCurrentPosition());
        config.telemetry.addData("Flywheel Power (both are the same)",topFlywheel.getPower());;
        config.telemetry.addData("indexer power", indexer.getPower());
    }








    public InstantAction runFlywheels(double speed) {
        return new InstantAction(() -> {
            bottomFlywheel.setPower(speed);
            topFlywheel.setPower(speed);
            state = OuttakeState.SPINNING_UP;
        });
    }
    public InstantAction stopFlywheels(){
        return new InstantAction(() -> {
            bottomFlywheel.setPower(0);
            topFlywheel.setPower(0);
            state = OuttakeState.STOPPED;
        });
    }
    public InstantAction runIndexer(){
        return new InstantAction(() -> {
            indexer.setPower(Globals.Outtake.INDEXER_SPINNING_POWER);
            state = OuttakeState.INDEXING;
        });
    }
    public InstantAction stopIndexer(){
        return new InstantAction(() -> {
            indexer.setPower(Globals.Outtake.INDEXER_SPINNING_POWER_RESTING);
            state = OuttakeState.STOPPED;
        });
    }
    private void resetMotors() {
        indexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        indexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        state = OuttakeState.STOPPED;

    }

}