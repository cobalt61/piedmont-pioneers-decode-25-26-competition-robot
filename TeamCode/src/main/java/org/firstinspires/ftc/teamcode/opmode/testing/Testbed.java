//package org.firstinspires.ftc.teamcode.opmode.testing;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.teamcode.hardware.Globals;
//import org.firstinspires.ftc.teamcode.hardware.robot.enums.CycleTarget;
//
//@Disabled
//@TeleOp(name="TeleOp", group="Testing")
//public class Testbed extends OpMode {
//    public DcMotor lift, extendo, intake;
//    private DcMotor leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive;
//    public Servo bucket;
//
//    public int lTarget = 0;
//    public int eTarget = 0;
//    public boolean direction = true;
//
//    boolean x;
//    public CycleTarget target;
//
//    @Override
//    public void init() {
//        // Initialize the hardware variables. Note that the strings used here must correspond
//        // to the names assigned during the robot configuration step on the DS or RC devices.
//
//        leftFrontDrive = hardwareMap.get(DcMotor.class, "rightBack");
//        rightFrontDrive = hardwareMap.get(DcMotor.class, "leftBack");
//        leftBackDrive = hardwareMap.get(DcMotor.class, "leftFront");
//        rightBackDrive = hardwareMap.get(DcMotor.class, "rightFront");
//        // Most robots need the motors on one side to be reversed to drive forward.
//        // When you first test your robot, push the left joystick forward
//        // and flip the direction ( FORWARD <-> REVERSE ) of any wheel that runs backwards
//        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE); // DO NOT CHANGE
//        leftBackDrive.setDirection(DcMotor.Direction.FORWARD); // DO NOT CHANGE
//        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE); // DO NOT CHANGE
//        rightBackDrive.setDirection(DcMotor.Direction.FORWARD); // DO NOT CHANGE
//
//        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // DO NOT CHANGE
//        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // DO NOT CHANGE
//        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // DO NOT CHANGE
//        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // DO NOT CHANGE
//
//        lift = hardwareMap.get(DcMotor.class, "lift");
//        extendo = hardwareMap.get(DcMotor.class, "extendo");
//        intake = hardwareMap.get(DcMotor.class, "intake");
//
//        bucket = hardwareMap.get(Servo.class, "bucket");
//
//        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        extendo.setDirection(DcMotorSimple.Direction.REVERSE);
//        lift.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        target = CycleTarget.SAMPLE;
//
//        x =false;
//    }
//
//    @Override
//    public void start() {
//        bucket.setPosition(Globals.Intake.BUCKET_UP);
//        intake.setPower(Globals.Intake.POWER_OFF);
//    }
//
//    @Override
//    public void loop() {
//        if (gamepad2.back) {
//            target = (target == CycleTarget.SAMPLE)?CycleTarget.SPECIMEN:CycleTarget.SAMPLE;
//        }
//
//        if (gamepad2.left_trigger >= 0.1 && lTarget != -40) {
//            lift.setPower(-1);
//            if (target == CycleTarget.SAMPLE || x) {
//                lTarget = -40;
//                x = false;
//            } else {
//                lTarget = 450;
//                x = true;
//            }
//            direction = false;
//            intake.setPower(0);
//
//        } else if (gamepad2.right_trigger >= 0.1 && lTarget != 2300) {
//            lift.setPower(1);
//            if (target == CycleTarget.SAMPLE) {
//                lTarget = 2350;
//            } else {
//                lTarget = 870;
//            }
//            intake.setPower(0);
//        }
//
//        if (lTarget != 0) {
//            if (direction) {
//                if (lift.getCurrentPosition() >= lTarget) {
//                    lift.setPower(0.35);
//                    lTarget = 0;
//                }
//            } else {
//                if (lift.getCurrentPosition() <= lTarget) {
//                    if (lTarget == 15) {
//                        lift.setPower(0);
//                    } else {
//                        lift.setPower(-0.8);
//                    }
//                    lTarget = 0;
//                    direction = true;
//                }
//            }
//        }
//        if (eTarget != 0) {
//            if (extendo.getCurrentPosition() <= eTarget) {
//                lift.setPower(0);
//                eTarget = 0;
//            }
//        }
//
//        if (gamepad2.right_bumper && !(extendo.getCurrentPosition() >= 2100)) {
//            extendo.setPower(1);
//        } else if (gamepad2.left_bumper && !(extendo.getCurrentPosition() <= 50)) {
//            extendo.setPower(-1);
//        } else {
//            extendo.setPower(0);
//        }
//
//        if (gamepad2.b) {
//            bucket.setPosition(Globals.Intake.BUCKET_UP);
//            intake.setPower(Globals.Intake.POWER_OFF);
//            if (!(extendo.getCurrentPosition() <= 50)) {
//                extendo.setPower(-1);
//            }
//            eTarget = 20;
//        }
//
//
//        if (gamepad2.a) {
//            bucket.setPosition(Globals.Intake.BUCKET_DOWN);
//            intake.setPower(Globals.Intake.POWER_ON);
//        }
//
//        if (gamepad2.x) {
//            bucket.setPosition(Globals.Intake.BUCKET_UP);
//            intake.setPower(Globals.Intake.POWER_OFF);
//        }
//
//        if (gamepad2.dpad_left) {
//            bucket.setPosition(Globals.Intake.BUCKET_PURGE);
//            intake.setPower(Globals.Intake.POWER_PURGE);
//        }
//
//        if (gamepad2.y) {
//            bucket.setPosition(Globals.Intake.BUCKET_DUMP);
//            try {
//                Thread.sleep(400);
//                intake.setPower(Globals.Intake.POWER_DUMP);
//                Thread.sleep(500);
//                intake.setPower(Globals.Intake.POWER_OFF);
//            } catch (InterruptedException e) {
//                throw new RuntimeException(e);
//            }
//
//        }
//
//        if (gamepad2.dpad_down) {
//            intake.setPower(Globals.Intake.POWER_OFF);
//        }
//        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
//        double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
//        double lateral = gamepad1.left_stick_x * 1.1; // 1.1 fixes strafing issues
//        // MUST BE INVERTED!
//        double yaw = -gamepad1.right_stick_x;
//        // Take the average of the 2 triggers
//        double speed = 1 - (gamepad1.right_trigger + gamepad1.left_trigger) / 2;
//
//        // Combine the joystick requests for each axis-motion to determine each wheel's power.
//        // Set up a variable for each drive wheel to save the power level for telemetry.
//        double leftFrontPower = (axial + lateral - yaw) * speed; // DO NOT CHANGE
//        double rightFrontPower = (axial - lateral - yaw) * speed; // DO NOT CHANGE
//        double leftBackPower = (axial + lateral + yaw) * speed; // DO NOT CHANGE
//        double rightBackPower = (axial - lateral + yaw) * speed; // DO NOT CHANGE
//
//        // Normalize the values so no wheel power exceeds 100%
//        // This ensures that the robot maintains the desired motion.
//        double max;
//        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
//        max = Math.max(max, Math.abs(leftBackPower));
//        max = Math.max(max, Math.abs(rightBackPower));
//
//        if (max > 1.0) {
//            leftFrontPower /= max;
//            rightFrontPower /= max;
//            leftBackPower /= max;
//            rightBackPower /= max;
//        }
//
//        // Send calculated power to wheels`
//        leftFrontDrive.setPower(leftFrontPower);
//        rightFrontDrive.setPower(rightFrontPower);
//        leftBackDrive.setPower(leftBackPower);
//        rightBackDrive.setPower(rightBackPower);
//
//        // Show the elapsed game time and wheel power.
//        //telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
//        //telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
//        //telemetry.addData("Right Stick x Position", "%4.2f", yaw);
//        //telemetry.addData("lift", lift.getCurrentPosition());
//        //telemetry.addData("extendo", extendo.getCurrentPosition());
//        telemetry.addData("ct", target);
//        //telemetry.addData("speed", lTarget);
//        //telemetry.addData("power", lift.getPower());
//        //telemetry.addData("bucket", bucket.getPosition());
//        telemetry.update();
//    }
//}