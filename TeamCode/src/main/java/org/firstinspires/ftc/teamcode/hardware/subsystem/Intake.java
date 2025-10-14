package org.firstinspires.ftc.teamcode.hardware.subsystem;

import static java.lang.Thread.sleep;

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
        INDEXING,
        STOPPED
    }


//    public enum IntakeContent {
//        RED,
//        BLUE,
//        YELLOW,
//        NULL
//    }

    Config config;

    DcMotor intakeMotor, indexer, bottomRoller;

    //IntakeColorSensor sensor;

    IntakeState state;


//    IntakeContent detected;

    public Intake(Config config) {this.config = config;}

    @Override
    public void init() {

        intakeMotor = config.hardwareMap.get(DcMotor.class, Globals.Intake.INTAKE_MOTOR);
        indexer = config.hardwareMap.get(DcMotor.class, Globals.Intake.INDEXER);
        bottomRoller = config.hardwareMap.get(DcMotor.class, Globals.Intake.BOTTOM_INTAKE_ROLLER);

        //sensor = new IntakeColorSensor(config.hardwareMap.get(ColorSensor.class, "colorSensor"));

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        indexer.setDirection(DcMotorSimple.Direction.REVERSE);
        indexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);// run it with encoder later
        bottomRoller.setDirection(DcMotorSimple.Direction.FORWARD);
        bottomRoller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

        if (config.gamepad1.y){
            newActions.add(index());
        }else{
            newActions.add(stopIndex());
        }

        config.telemetry.addData("intake state", state);
        config.telemetry.addData("intake position", intakeMotor.getCurrentPosition());
        config.telemetry.addData("bottom roller power", bottomRoller.getPower());
        config.telemetry.addData("intake power", intakeMotor.getPower());
        config.telemetry.addData("indexer power", indexer.getPower());

        config.telemetry.addData("bottom roller position",bottomRoller.getCurrentPosition());
        config.telemetry.addData("index position",indexer.getCurrentPosition());
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
            bottomRoller.setPower(Globals.Intake.POWER_OFF);
            state = IntakeState.STOPPED;
        });
    }

    public InstantAction runIntake() {
        return new InstantAction(() -> {
            intakeMotor.setPower(Globals.Intake.POWER_ON);
            bottomRoller.setPower(Globals.Intake.POWER_ON);
            state = IntakeState.INTAKING;
        });
    }
    public Action index() {
        return new InstantAction(() -> {
            //come back to fix
            indexer.setPower(Globals.Intake.POWER_ON);

            state = IntakeState.INDEXING;

        });
    }
    public Action stopIndex() {
        return new InstantAction(() -> {
            indexer.setPower(Globals.Intake.POWER_OFF);
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
