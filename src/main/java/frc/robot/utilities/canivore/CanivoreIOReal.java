package frc.robot.utilities.canivore;

import com.ctre.phoenix6.CANBus;

import frc.robot.Constants;

public class CanivoreIOReal implements CanivoreIO {
    
    public CanivoreIOReal() {

    }

    @Override
    public void updateInputs(CanivoreIOInputs inputs) {
        var canbusStatus = CANBus.getStatus(Constants.CANBUS_NAME);
        inputs.busUtilization = canbusStatus.BusUtilization;
        inputs.receiveErrorCounter = canbusStatus.REC;
        inputs.transmitErrorCounter = canbusStatus.TEC;
    }
}
