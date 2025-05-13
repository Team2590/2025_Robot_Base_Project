package frc.robot.util;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

public class StickyFaultUtil {

  public static void clearCancoderStickyFaults(CANcoder cancoder, String cancoderName) {
    int faulted = cancoder.getStickyFaultField().getValue();
    if (faulted > 0) {
      if (cancoder.getStickyFault_BadMagnet().getValue()) {
        // System.out.println("Clearing Sticky Fault - Bad Magnet on " + cancoderName);
      }
      if (cancoder.getStickyFault_BootDuringEnable().getValue()) {
        // System.out.println("Clearing Sticky Fault - Boot During Enable on " + cancoderName);
      }
      if (cancoder.getStickyFault_Hardware().getValue()) {
        // System.out.println("Clearing Sticky Fault - Hardware on " + cancoderName);
      }
      if (cancoder.getStickyFault_Undervoltage().getValue()) {
        // System.out.println("Clearing Sticky Fault - Under Voltage on " + cancoderName);
      }
      if (cancoder.getStickyFault_UnlicensedFeatureInUse().getValue()) {
        // System.out.println("Clearing Sticky Fault - Unlicensed Feature In Use on " +
        // cancoderName);
      }
      cancoder.clearStickyFaults();
    }
  }

  public static void clearMotorStickyFaults(TalonFX motor, String motorName) {
    int faulted = motor.getStickyFaultField().getValue();
    if (faulted > 0) {
      if (motor.getStickyFault_BootDuringEnable().getValue()) {
        // System.out.println("Clearing Sticky Fault - Boot During Enable on " + motorName);
      }
      if (motor.getStickyFault_BridgeBrownout().getValue()) {
        // System.out.println("Clearing Sticky Fault - Bridge Brownout on " + motorName);
      }
      if (motor.getStickyFault_DeviceTemp().getValue()) {
        // System.out.println("Clearing Sticky Fault - Device Temp on " + motorName);
      }
      if (motor.getStickyFault_ForwardHardLimit().getValue()) {
        // System.out.println("Clearing Sticky Fault - Forward Hard Limit on " + motorName);
      }
      if (motor.getStickyFault_ForwardSoftLimit().getValue()) {
        // System.out.println("Clearing Sticky Fault - Forward Soft Limit on " + motorName);
      }
      if (motor.getStickyFault_FusedSensorOutOfSync().getValue()) {
        // System.out.println("Clearing Sticky Fault - Fused Sensor Out Of Sync on " + motorName);
      }
      if (motor.getStickyFault_Hardware().getValue()) {
        // System.out.println("Clearing Sticky Fault - Hardware on " + motorName);
      }
      if (motor.getStickyFault_MissingDifferentialFX().getValue()) {
        // System.out.println("Clearing Sticky Fault - Missing Differential FX on " + motorName);
      }
      if (motor.getStickyFault_MissingHardLimitRemote().getValue()) {
        // System.out.println("Clearing Sticky Fault - Missing Hard Limit Remote on " + motorName);
      }
      if (motor.getStickyFault_MissingSoftLimitRemote().getValue()) {
        // System.out.println("Clearing Sticky Fault - Missing Soft Limit Remote on " + motorName);
      }
      if (motor.getStickyFault_OverSupplyV().getValue()) {
        // System.out.println("Clearing Sticky Fault - Over Supply on " + motorName);
      }
      if (motor.getStickyFault_ProcTemp().getValue()) {
        // System.out.println("Clearing Sticky Fault - Proc Temp on " + motorName);
      }
      if (motor.getStickyFault_RemoteSensorDataInvalid().getValue()) {
        // System.out.println("Clearing Sticky Fault - Remote Sensor Data Invalid on " + motorName);
      }
      if (motor.getStickyFault_RemoteSensorPosOverflow().getValue()) {
        // System.out.println("Clearing Sticky Fault - Remote Sensor Pos Overflow on " + motorName);
      }
      if (motor.getStickyFault_RemoteSensorReset().getValue()) {
        // System.out.println("Clearing Sticky Fault - Remote Sensor Reset on " + motorName);
      }
      if (motor.getStickyFault_ReverseHardLimit().getValue()) {
        // System.out.println("Clearing Sticky Fault - Reverse Hard Limit on " + motorName);
      }
      if (motor.getStickyFault_ReverseSoftLimit().getValue()) {
        // System.out.println("Clearing Sticky Fault - Reverse Soft Limit on " + motorName);
      }
      if (motor.getStickyFault_StaticBrakeDisabled().getValue()) {
        // System.out.println("Clearing Sticky Fault - Static Brake Disabled on " + motorName);
      }
      if (motor.getStickyFault_StatorCurrLimit().getValue()) {
        // System.out.println("Clearing Sticky Fault - Stator Curr Limit on " + motorName);
      }
      if (motor.getStickyFault_SupplyCurrLimit().getValue()) {
        // System.out.println("Clearing Sticky Fault - Supply Curr Limit on " + motorName);
      }
      if (motor.getStickyFault_Undervoltage().getValue()) {
        // System.out.println("Clearing Sticky Fault - Undervoltage on " + motorName);
      }
      if (motor.getStickyFault_UnlicensedFeatureInUse().getValue()) {
        // System.out.println("Clearing Sticky Fault - Unlicensed Feature In Use on " + motorName);
      }
      if (motor.getStickyFault_UnstableSupplyV().getValue()) {
        // System.out.println("Clearing Sticky Fault - Unstable Supply V on " + motorName);
      }
      if (motor.getStickyFault_UsingFusedCANcoderWhileUnlicensed().getValue()) {
        // System.out.println(
        // "Clearing Sticky Fault - Using Fused CANcoder While Unlicensed on " + motorName);
      }
      motor.clearStickyFaults();
    }
  }
}
