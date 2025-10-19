package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class touchsensor {
    private DigitalChannel touchsensor;


     public void init(HardwareMap hwMap) {
        touchsensor = hwMap.get(DigitalChannel.class, "touch_sensor");
        touchsensor.setMode(DigitalChannel.Mode.INPUT);
    }

    public boolean getTouchSensorState(){
         if (touchsensor.getState() == false) {
             return true;
         }
         return false;

    }
}

