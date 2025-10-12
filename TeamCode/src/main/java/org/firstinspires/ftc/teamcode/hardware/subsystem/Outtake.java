package org.firstinspires.ftc.teamcode.hardware.subsystem;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.hardware.robot.Config;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.robot.enums.GameStage;

import java.util.ArrayList;
import java.util.List;

public class Outtake implements SubSystem {

    // Lift positions
    public enum OuttakeStates {
        SPINNING,
        INDEXING,
        STOPPED
    }


    // Constants for joystick thresholds
    private static final double JOYSTICK_THRESHOLD = 0.25;

    private final Config config;
    private DcMotor bottomFlywheel, topFlywheel, indexer;
    private DigitalChannel switchV;

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
        topFlywheel.setDirection(DcMotor.Direction.REVERSE);
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


        }
//        if (config.gamepad2.back) {
//            resetMotors();
//        }
//        if (config.gamepad2.right_trigger >= 0.1) {
//            setLiftPower(Math.min(1,config.gamepad2.right_trigger*2));
//        } else if (config.gamepad2.left_trigger >= 0.1 && !switchV.getState()) {
//            setLiftPower(-Math.min(1,config.gamepad2.left_trigger*2));
//        } else if (!switchV.getState()) {
//            setLiftPower(Globals.Outtake.LIFT_IDLE);
//        } else {
//            setLiftPower(Globals.Outtake.LIFT_OFF);
//            resetMotors();
//        }

        if (config.gamepad2.dpad_left) {
            newActions.add(lowerToBottom());
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
        config.telemetry.addData("Right Lift Pos", right.getCurrentPosition());
        config.telemetry.addData("Right Lift Power", right.getPower());
        config.telemetry.addData("Left Lift Pos", left.getCurrentPosition());
        config.telemetry.addData("Left Lift Power", left.getPower());
        config.telemetry.addData("Lift Position", position);
        config.telemetry.addData("Lift Direction", direction);
        config.telemetry.addData("Switch", switchV.getState());
    }







    private void updateTelemetry(TelemetryPacket telemetryPacket) {
        telemetryPacket.put("Right Lift Pos", right.getCurrentPosition());
        telemetryPacket.put("Right Lift Power", right.getPower());
        telemetryPacket.put("Left Lift Pos", left.getCurrentPosition());
        telemetryPacket.put("Left Lift Power", left.getPower());
        telemetryPacket.put("Lift Position", position);
        telemetryPacket.put("Lift Direction", direction);
        telemetryPacket.put("Switch", switchV.getState());

        addTelemetryData();
    }

    public InstantAction runFlywheels() {
        return new InstantAction(() -> {
            bottomFlywheel.setPower(config.gamepad1.right_trigger*2);
            topFlywheel.setPower(config.gamepad1.right_trigger*2);
            state = Globals.Outtake.IntakeState.INTAKING;
        });
    }


}