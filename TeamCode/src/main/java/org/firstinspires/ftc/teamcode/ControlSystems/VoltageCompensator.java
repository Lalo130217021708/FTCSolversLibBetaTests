package org.firstinspires.ftc.teamcode.ControlSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class VoltageCompensator {
    public VoltageSensor voltageSensor;

    public VoltageCompensator(HardwareMap hardwareMap){
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Expansion Hub 2");
    }

    public double getVoltage(){
        return voltageSensor.getVoltage();
    }

    public double compensateVoltage( double output){
        return output * (12.4 / getVoltage());
    }
}
