package org.ghrobotics.frc2019.sensors

import edu.wpi.first.wpilibj.SPI
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.frc2019.Robot

val flowSensor = ADNS3080FlowSensor(
    SPI.Port.kOnboardCS0,
    2866.0 / 10.0,
    0.degree,
    Robot.coroutineContext
)