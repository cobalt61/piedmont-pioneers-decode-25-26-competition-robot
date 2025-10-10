package org.firstinspires.ftc.teamcode.hardware.subsystem;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.robot.Config;
import org.firstinspires.ftc.teamcode.hardware.Globals;

import java.util.ArrayList;
import java.util.List;

public class Intake implements SubSystem {
    public enum IntakeState {
        INTAKING,
        STOPPED
    }


//    public enum IntakeContent {
//        RED,
//        BLUE,
//        YELLOW,
//        NULL
//    }

    Config config;

    DcMotor intakeMotor;

    //IntakeColorSensor sensor;

    IntakeState state;


//    IntakeContent detected;

    public Intake(Config config) {this.config = config;}

    @Override
    public void init() {

        intakeMotor = config.hardwareMap.get(DcMotor.class, Globals.Intake.INTAKE_MOTOR);


        //sensor = new IntakeColorSensor(config.hardwareMap.get(ColorSensor.class, "colorSensor"));

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        detected = IntakeContent.NULL;
    }

    @Override
    public void start() {

    }

    @Override
    public List<Action> update() {
        List<Action> newActions = new ArrayList<>();

        if (config.gamepad1.x){
            newActions.add(runIntake());
        }
        else
        {
            newActions.add(stopIntake());
        }

        config.telemetry.addData("intake state", state);
        config.telemetry.addData("intake power (0-1)", intakeMotor.getPower());
        //config.telemetry.addData("color sensor detections", detected);

        return newActions;
    }

//    public Action purge() {
//        return new SequentialAction(
//                new InstantAction(() -> {
//                    bucket.setPosition(Globals.Intake.BUCKET_DOWN);
//                    bucketPosition = BucketPosition.DOWN;
//                    }),
//                new SleepAction(0.6),
//                new InstantAction(() -> {
//                    intakeMotor.setPower(Globals.Intake.POWER_PURGE);
//
//                    state = IntakeState.PURGING;
//        }));
//    }

    public InstantAction stopIntake() {
        return new InstantAction(() -> {
            intakeMotor.setPower(Globals.Intake.POWER_OFF);

            state = IntakeState.STOPPED;
        });
    }

    public InstantAction runIntake() {
        return new InstantAction(() -> {
            intakeMotor.setPower(Globals.Intake.POWER_ON);

            state = IntakeState.INTAKING;
        });
    }

//    public Action dump() {
//        return new SequentialAction(
//                new InstantAction(() -> {
//                    bucket.setPosition(Globals.Intake.BUCKET_DUMP);
//
//                    state = IntakeState.DUMPING;
//                    bucketPosition = BucketPosition.DUMP;}),
//                new SleepAction(0.5),
//                new InstantAction(() -> {
//                    intakeMotor.setPower(Globals.Intake.POWER_DUMP);}),
//                new SleepAction(0.3),
//                raise()
//        );
//    }

//    public InstantAction raise() {
//        return new InstantAction(() -> {
//            intakeMotor.setPower(Globals.Intake.POWER_OFF);
//            bucket.setPosition(Globals.Intake.BUCKET_UP);
//            state = (state == IntakeState.DUMPING)? IntakeState.RETRACTED: IntakeState.EXTENDED;
//
//            bucketPosition = BucketPosition.ZERO;
//        });
//    }

//    public Action extend() {
//        return telemetryPacket -> {
//            if (!(state == IntakeState.EXTENDING)) {
//                extendo.setPower(Globals.Intake.EXTENDO_POWER_OUT);
//
//                state = IntakeState.EXTENDING;
//            }
//
//            if (extendo.getCurrentPosition() >= Globals.Intake.EXTENDO_OUT) {
//                extendo.setPower(0);
//                state = IntakeState.EXTENDED;
//                return false;
//            }
//
//            return true;
//        };
//    }

//    public Action retract() {
//        return telemetryPacket -> {
//            if (!(state == IntakeState.RETRACTING)) {
//                extendo.setPower(Globals.Intake.EXTENDO_POWER_IN);
//
//                state = IntakeState.RETRACTING;
//            }
//
//            if (extendo.getCurrentPosition() <= Globals.Intake.EXTENDO_IN) {
//                extendo.setPower(0);
//                state = IntakeState.RETRACTED;
//                return false;
//            }
//
//            return true;
//        };
//    }

//    public Action raiseAndRetract() {
//        return new SequentialAction(
//                raise(),
//                retract()
//        );
//    }
}
