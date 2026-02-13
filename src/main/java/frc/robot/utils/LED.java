package frc.robot.utils;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj.util.Color;

import frc.robot.Constants.LEDConstants;

public class LED {

    private CANdle candle;
    public LED() {
        candle = new CANdle(LEDConstants.candleId, LEDConstants.canBus);

        CANdleConfiguration cfg = new CANdleConfiguration();
        cfg.LED.BrightnessScalar = 1.0;
        cfg.LED.StripType = StripTypeValue.BRG;

        candle.getConfigurator().apply(cfg);
    }

    public void setSolidColor(Color color, double brightness){
        candle.setControl(new SolidColor(0, LEDConstants.endIndex).withColor(new RGBWColor(color).scaleBrightness(brightness)));
    }

    public void setSolidColor(Color color){
        this.setSolidColor(color, 1);
    }
    
    public void clearColor(){
        candle.setControl(new SolidColor(0, LEDConstants.endIndex).withColor(new RGBWColor(new Color(0,0,0)).scaleBrightness(1)));
    }
    
    public void startFireAnimation(){
        FireAnimation FIRE = new FireAnimation(0, LEDConstants.endIndex).withBrightness(1).withCooling(0.3);
        candle.setControl(FIRE);   
    }

}