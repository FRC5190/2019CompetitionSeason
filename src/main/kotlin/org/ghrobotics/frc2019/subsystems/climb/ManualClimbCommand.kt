package org.ghrobotics.frc2019.subsystems.climb

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.GenericHID
import org.ghrobotics.frc2019.Controls
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.utils.withDeadband
import org.ghrobotics.lib.wrappers.hid.getY
import kotlin.math.absoluteValue

class ManualClimbCommand : FalconCommand(ClimbSubsystem) {

    var frontset = false
    var backset = false

    override suspend fun execute() {
        if (Controls.isClimbing) {

            val frontWinch = -frontWinchSource()
            val backWinch = -backWinchSource()
            val wheelPercent = -wheelSource()

            ClimbSubsystem.wheelPercentOutput = wheelPercent

            if (frontWinch.absoluteValue > 0.1) {
                ClimbSubsystem.frontWinchPercentOutput = frontWinch
                frontset = false
            } else if (!frontset) {
                ClimbSubsystem.Winch.FRONT.motor.set(ControlMode.MotionMagic, ClimbSubsystem.rawFront.toDouble())
                frontset = true
            }

            if (backWinch.absoluteValue > 0.2) {
                ClimbSubsystem.backWinchPercentOutput = backWinch
                backset = false
            } else if (!backset) {
                ClimbSubsystem.Winch.BACK.motor.set(ControlMode.MotionMagic, ClimbSubsystem.rawBack.toDouble())
                backset = true
            }
        } else {
            ClimbSubsystem.frontWinchPercentOutput = 0.0
            ClimbSubsystem.backWinchPercentOutput = 0.0
            ClimbSubsystem.wheelPercentOutput = 0.0
            backset = false
            frontset = false
        }
    }

    private companion object {
        val frontWinchSource = Controls.operatorFalconXbox.getY(GenericHID.Hand.kLeft)
        val backWinchSource = Controls.operatorFalconXbox.getY(GenericHID.Hand.kRight)
        val wheelSource = Controls.driverFalconXbox.getY(GenericHID.Hand.kRight).withDeadband(0.1)
    }
}
