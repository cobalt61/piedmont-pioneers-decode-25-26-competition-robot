//package org.firstinspires.ftc.teamcode.hardware.device;
//
//import com.qualcomm.hardware.rev.RevColorSensorV3;
//
//import org.firstinspires.ftc.teamcode.hardware.Globals;
//import org.firstinspires.ftc.teamcode.hardware.subsystem.Intake.IntakeContent;
//
//public class IntakeColorSensor {
//    RevColorSensorV3 sensor;
//
//    public IntakeColorSensor(RevColorSensorV3 sensor) {
//        this.sensor = sensor;
//    }
//
//    public IntakeContent get() {
//        if (sensor.red() >= Globals.Intake.SENSOR_YELLOW_THRESHOLD_RED && sensor.green() >= Globals.Intake.SENSOR_YELLOW_THRESHOLD_GREEN) {
//            return IntakeContent.YELLOW;
//        } else if (sensor.red() >= Globals.Intake.SENSOR_RED_THRESHOLD) {
//            return IntakeContent.RED;
//        } else if (sensor.blue() >= Globals.Intake.SENSOR_BLUE_THRESHOLD) {
//            return IntakeContent.RED;
//        } else {
//            return IntakeContent.NULL;
//        }
//    }
//}
