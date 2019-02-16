@file:Suppress("MatchingDeclarationName")

package org.ghrobotics.frc2019.vision

import com.github.salomonbrys.kotson.fromJson
import com.google.gson.Gson
import com.google.gson.JsonObject
import com.google.gson.JsonParseException
import edu.wpi.first.wpilibj.SerialPort
import edu.wpi.first.wpilibj.Timer
import org.ghrobotics.lib.mathematics.units.Time
import org.ghrobotics.lib.mathematics.units.second
import java.util.concurrent.TimeUnit
import java.util.concurrent.TimeoutException
import kotlin.concurrent.thread

class JeVois(
    private val port: SerialPort.Port,
    private val processData: (VisionData) -> Unit
) {

    private var fpgaOffset = 0.second

    init {
        thread {
            while (true) {
                try {
                    val port = createSerialPort()
                    initPort(port)
                    readPort(port)
                } catch (@Suppress("TooGenericExceptionCaught") e: Throwable) {
                    e.printStackTrace()
                    println("[JeVois-${port.name}] Failure in connection!!!")
                }
            }
        }
    }

    private fun createSerialPort(): SerialPort {
        println("[JeVois-${port.name}] Trying to create serial port...")
        while (true) {
            try {
                val serialPort = SerialPort(115200, port)
                serialPort.setTimeout(0.5)
                println("[JeVois-${port.name}] Serial port created!")
                return serialPort
            } catch (@Suppress("TooGenericExceptionCaught") e: Throwable) {
                e.printStackTrace()
                TimeUnit.MILLISECONDS.sleep(1000)
            }
        }
    }

    private fun initPort(serialPort: SerialPort) {
        serialPort.writeString("setpar serout USB\n")
        serialPort.writeString("date 0101000070\n")
        serialPort.writeString("streamon\n")

        fpgaOffset = Timer.getFPGATimestamp().second
    }

    private fun readPort(serialPort: SerialPort) {
        val tempStringBuilder = StringBuilder()
        var lastReceivedTime = System.currentTimeMillis()
        while (true) {
            val newData = serialPort.read(serialPort.bytesReceived)
            if(newData.isNotEmpty()){
                lastReceivedTime = System.currentTimeMillis()
            }
            if(System.currentTimeMillis() - lastReceivedTime > 1000) {
                throw TimeoutException("Roborio didnt receive data from jevois in time")
            }
            for(byte in newData) {
                val charReceived = byte.toChar()
                if(charReceived == '\n') {
                    val line = tempStringBuilder.toString()
                    tempStringBuilder.clear()
                    if (!line.startsWith('{')) continue
                    try {
                        val jsonData = kJevoisGson.fromJson<JsonObject>(line)

                        val timestamp = jsonData["Epoch Time"].asDouble.second + fpgaOffset
                        val contours = jsonData["Targets"].asJsonArray
                            .filterIsInstance<JsonObject>()

                        processData(VisionData(timestamp, contours))
                    } catch (e: JsonParseException) {
                        e.printStackTrace()
                        println("[JeVois-${port.name}] Got Invalid Data: $line")
                    }
                }else{
                    tempStringBuilder.append(charReceived)
                }
            }
            Thread.sleep(1)
        }
    }

    companion object {
        private val kJevoisGson = Gson()
    }

}

data class VisionData(
    val timestamp: Time,
    val targets: List<JsonObject>
)