#ifndef DIFFDRIVE_DDSM115_COMMS_HPP
#define DIFFDRIVE_DDSM115_COMMS_HPP

#include <sstream>
#include <libserial/SerialPort.h>
#include <iostream>
#include <cmath>
#include <vector>
#include "CRC.h"

#define MAX_CURRENT				2
#define MIN_CURRENT				-2
#define MAX_VELOCITY			200
#define MIN_VELOCITY			-200
#define MAX_ANGLE				360
#define MIN_ANGLE				0

#define CRC8_MAXIM_POLY			0x31
#define CRC8_MAXIM_INIT			0x00
#define CRC8_MAXIM_REFIN		true
#define CRC8_MAXIM_REFOUT		true
#define CRC8_MAXIM_XOROUT		0x00

typedef enum {
  CURRENT_LOOP = 1,
  VELOCITY_LOOP = 2,
  POSITION_LOOP = 3,
} ddsm115_mode_t;

typedef enum {
  DDSM115_PROTOCOL_V1 = 1,
  DDSM115_PROTOCOL_V2 = 2,
} ddsm115_protocol_t;

typedef enum {
  DDSM115_TROUBLESHOOTING = 0x10,
  DDSM115_STALL_ERROR = 0x08,
  DDSM115_PHASE_OVERCURRENT_ERROR = 0x04,
  DDSM115_OVERCURRENT_ERROR = 0x02,
  DDSM115_SENSOR_ERROR = 0x01,
} ddsm115_error_t;

struct Response { 
	uint8_t id; 
	ddsm115_mode_t mode; 
	float current;
	int16_t velocity;
	float angle;
	uint8_t winding_temp;
	int16_t position;
	uint8_t error;
};

class DDSM115Comms
{
    public:
    DDSM115Comms() = default;
    Response responseData;

    std::vector<uint8_t> commandBuffer;
    std::vector<uint8_t> responseBuffer;
    

    // Function to calculate CRC Maxim
    uint8_t calculateCRC8(const std::vector<uint8_t>& data) {
    uint8_t crc = 0x00;
    for (const uint8_t& byte : data) {
        crc ^= byte;
        for (int i = 0; i < 8; i++) {
            if (crc & 0x01) {
                crc = (crc >> 1) ^ 0x8C;  // 0x8C is the polynomial for CRC-8
            } else {
                crc = crc >> 1;
            }
        }
    }
        return crc;
    }

    void connect(const std::string &serial_device, int32_t timeout_ms)
    {  
        timeout_ms_ = timeout_ms;
        serial_conn_.Open(serial_device);
        serial_conn_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    }

    void disconnect()
    {
        serial_conn_.Close();
    }

    bool connected() const
    {
        return serial_conn_.IsOpen();
    }

    void sendCommand(const std::vector<uint8_t>& msg_to_send, bool print_output = false, bool crc = true) 
    {
        // Calculate CRC and append it to the command
        std::vector<uint8_t> command = msg_to_send;

        if (crc) 
        {
            uint8_t crcValue = calculateCRC8(msg_to_send);
            command.push_back(crcValue);
        }
        serial_conn_.FlushIOBuffers(); // Just in case
        serial_conn_.Write(command);



        // std::vector<uint8_t> responseBuffer;
        // if (!receive(responseBuffer)) {
        //     std::cerr << "Receive failed." << std::endl;
        // }

        if (print_output)
        {
            std::cout << "DDSM115 command sent: ";
            for (uint8_t byte : command) {
                printf("%02X ", byte);
            }
            std::cout << std::endl;

            // std::cout << "Received " << responseBuffer.size() << " bytes." << std::endl;
            // displayReceivedData(responseBuffer);
        }


    }

    void displayReceivedData(const std::vector<uint8_t>& data) 
    {
        std::cout << "Received data: ";
        for (const auto& byte : data) {
            std::cout << std::hex << static_cast<int>(byte) << " ";
        }
        std::cout << std::dec << std::endl;
    }

    bool receive() 
    {
        uint8_t _b = 0;
        responseBuffer.clear();
        while (true) {
            try {
                uint8_t responseByte;
                serial_conn_.ReadByte(responseByte, 25);
                responseBuffer.push_back(responseByte);
                _b++;

                if (_b > 9) {
                    // Received all expected bytes
                    //parse(DDSM115_PROTOCOL_V1, responseBuffer);
                    //return true;
                    return checkCRC(responseBuffer);
                }
            } catch (const std::exception& e) {
                std::cerr << "Error while reading: " << e.what() << std::endl;
                return false;
            }
        }
        return false;
    }

    bool checkCRC(std::vector<uint8_t>& responseBuffer){
        if (crc8(responseBuffer.data(), 9, CRC8_MAXIM_POLY, CRC8_MAXIM_INIT, CRC8_MAXIM_REFIN, CRC8_MAXIM_REFOUT, CRC8_MAXIM_XOROUT) == responseBuffer[9]){
            return true;
        }
        return false;
    }

    void parse(ddsm115_protocol_t protocol) 
    {
        responseData.id = responseBuffer[0];
        responseData.mode = static_cast<ddsm115_mode_t>(responseBuffer[1]);

        uint16_t current = (static_cast<uint16_t>(responseBuffer[2]) << 8) | static_cast<uint16_t>(responseBuffer[3]);
        int currentR = current;
        if (currentR > 32767) {
            currentR -= 0xFFFF;
            currentR--;
        }
        if (currentR >= 0) {
            responseData.current = static_cast<float>(currentR) * MAX_CURRENT / 32767.0;
        } else {
            responseData.current = static_cast<float>(currentR) * MIN_CURRENT / -32767.0;
        }

        uint16_t velocity = (static_cast<uint16_t>(responseBuffer[4]) << 8) | static_cast<uint16_t>(responseBuffer[5]);
        int16_t velocityR = velocity;
        if (velocityR > MAX_VELOCITY) {
            velocityR -= 0xFFFF;
            velocityR--;
        }
        responseData.velocity = velocityR;

        if (protocol == DDSM115_PROTOCOL_V1) {
            uint16_t position = (static_cast<uint16_t>(responseBuffer[6]) << 8) | static_cast<uint16_t>(responseBuffer[7]);
            int16_t positionR = position;
            if (positionR > 32767) {
                positionR -= 0xFFFF;
                positionR--;
            }
            if (positionR >= 0) {
                responseData.angle = static_cast<float>(positionR) * MAX_ANGLE / 32767.0;
            } else {
                responseData.angle = static_cast<float>(positionR) * MIN_ANGLE / -32767.0;
            }
        }

        if (protocol == DDSM115_PROTOCOL_V2) {
            responseData.winding_temp = responseBuffer[6];
            responseData.angle = std::round(static_cast<float>(responseBuffer[7]) * MAX_ANGLE / 255.0);
        }

        responseData.error = responseBuffer[8];
        // std::cout << "Angle of motor " << responseData.angle << " degrees." << std::endl;
        // std::cout << "Current of motor " << responseData.current << " amps." << std::endl;
    }

    void set_ddsm115_mode(uint8_t motor_id, ddsm115_mode_t mode)
    {
        std::vector<uint8_t> modeCmd = {
            (uint8_t)motor_id, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, (uint8_t)mode
        };
        sendCommand(modeCmd, true, false);
    }

    bool get_ddsm115_mode(uint8_t motor_id)
    {
        std::vector<uint8_t> modeCmd = {
            (uint8_t)motor_id, 0x74, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        };
        sendCommand(modeCmd, false, true);
        if (receive()) { 

            parse(DDSM115_PROTOCOL_V2);
            return true;  
        }
        return false;
    }

    bool set_ddsm115_brakes(uint8_t motor_id)
    {
        std::vector<uint8_t> modeCmd = {
            (uint8_t)motor_id, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00
        };
        sendCommand(modeCmd, true);
        if (receive()) { 
            parse(DDSM115_PROTOCOL_V1);
            return true;  
        }
        return false;
    }

    bool set_ddsm115_velocity(uint8_t motor_id, int16_t velocity, uint8_t acceleration)
    {
        //01 64 00 32 00 00 00 00 00 D3
        if (velocity > MAX_VELOCITY) velocity = MAX_VELOCITY;
        if (velocity < MIN_VELOCITY) velocity = MIN_VELOCITY;
        uint16_t velocityRecalc = abs(velocity);
        if (velocity < 0 && velocityRecalc != 0) velocityRecalc = 0xFFFF - velocityRecalc + 1; 
        uint8_t velocityHighByte = (uint8_t)(velocityRecalc >> 8) & 0xFF;
        uint8_t velocityLowByte = (uint8_t)(velocityRecalc) & 0xFF;
        std::vector<uint8_t> modeCmd = {
            (uint8_t)motor_id, 
            0x64, 
            velocityHighByte,
            velocityLowByte, 
            0x00, 
            0x00, 
            acceleration, 
            0x00, 
            0x00 // This will be converted to checksum value
        };
        sendCommand(modeCmd, false, true);
        if (receive()) { 
            parse(DDSM115_PROTOCOL_V1);
            return true;  
        }
        return false;
    }

    // void read_encoder_values(int &val_1, int &val_2)
    // {
    //     //std::string response = send_msg("");
    // }

Response getData()
{
    return responseData;
}

private:
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_;
};

#endif // DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP