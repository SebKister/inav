
#include <stdbool.h>
#include <stdint.h>
#include "drivers/bus.h"
#include "drivers/time.h"
#include "build/debug.h"
#include "common/vector.h"
#include "drivers/accgyro/accgyro_bno08x.h"

#define PI 3.1415926535897932384626433832795
// Registers
const uint8_t CHANNEL_COMMAND = 0;
const uint8_t CHANNEL_EXECUTABLE = 1;
const uint8_t CHANNEL_CONTROL = 2;
const uint8_t CHANNEL_REPORTS = 3;
const uint8_t CHANNEL_WAKE_REPORTS = 4;
const uint8_t CHANNEL_GYRO = 5;

// Global Variables

uint8_t shtpHeader[4]; //Each packet has a header of 4 bytes
uint8_t shtpData[MAX_PACKET_SIZE];
uint8_t sequenceNumber[6] = {0, 0, 0, 0, 0, 0}; //There are 6 com channels. Each channel has its own seqnum
uint8_t commandSequenceNumber = 0;				//Commands have a seqNum as well. These are inside command packet, the header uses its own seqNum per channel
uint32_t metaData[MAX_METADATA_SIZE];			//There is more than 10 words in a metadata record but we'll stop at Q point 3

int16_t rotationVector_Q1 = 14;
uint16_t rawQuatI, rawQuatJ, rawQuatK, rawQuatReal;

bool _hasReset = false;		// Keeps track of any Reset Complete packets we receive. 


static busDevice_t *busDev;
uint8_t calibrationStatus;


//Given a register value and a Q point, convert to float
//See https://en.wikipedia.org/wiki/Q_(number_format)
float qToFloat(int16_t fixedPointValue, uint8_t qPoint)
{
	float qFloat = fixedPointValue;
	qFloat *= pow(2, qPoint * -1);
	return (qFloat);
}

static bool BNO080_deviceDetect(busDevice_t *busDev)
{
    for (int retry = 0; retry < 5; retry++)
    {
        uint8_t sig;

        delay(150);

        bool ack = busRead(busDev, 0x00, &sig);
        if (ack)
        {
            return true;
        }
    };

    return false;
}
// Sends the packet to enable the ar/vr stabilized rotation vector
void BNO080_enableARVRStabilizedRotationVector(uint16_t timeBetweenReports)
{
    setFeatureCommand(SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR, timeBetweenReports);
}

// Given a register value and a Q point, convert to float
// See https://en.wikipedia.org/wiki/Q_(number_format)
float BNO080_qToFloat(int16_t fixedPointValue, uint8_t qPoint)
{
    float qFloat = fixedPointValue;
    qFloat *= pow(2, qPoint * -1);
    return (qFloat);
}

// Quaternion to Euler conversion
// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
// https://github.com/sparkfun/SparkFun_MPU-9250-DMP_Arduino_Library/issues/5#issuecomment-306509440
// Return the roll (rotation around the x-axis) in Radians
float BNO080_getRoll()
{
    float dqw = BNO080_getQuatReal();
    float dqx = BNO080_getQuatI();
    float dqy = BNO080_getQuatJ();
    float dqz = BNO080_getQuatK();

    float norm = sqrt(dqw * dqw + dqx * dqx + dqy * dqy + dqz * dqz);
    dqw = dqw / norm;
    dqx = dqx / norm;
    dqy = dqy / norm;
    dqz = dqz / norm;

    float ysqr = dqy * dqy;

    // roll (x-axis rotation)
    float t0 = +2.0 * (dqw * dqx + dqy * dqz);
    float t1 = +1.0 - 2.0 * (dqx * dqx + ysqr);
    float roll = atan2(t0, t1);

    return (roll);
}

// Return the pitch (rotation around the y-axis) in Radians
float BNO080_getPitch()
{
    float dqw = BNO080_getQuatReal();
    float dqx = BNO080_getQuatI();
    float dqy = BNO080_getQuatJ();
    float dqz = BNO080_getQuatK();

    float norm = sqrt(dqw * dqw + dqx * dqx + dqy * dqy + dqz * dqz);
    dqw = dqw / norm;
    dqx = dqx / norm;
    dqy = dqy / norm;
    dqz = dqz / norm;

    float ysqr = dqy * dqy;

    // pitch (y-axis rotation)
    float t2 = +2.0 * (dqw * dqy - dqz * dqx);
    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    float pitch = asin(t2);

    return (pitch);
}

// Return the yaw / heading (rotation around the z-axis) in Radians
float BNO080_getYaw()
{
    float dqw = BNO080_getQuatReal();
    float dqx = BNO080_getQuatI();
    float dqy = BNO080_getQuatJ();
    float dqz = BNO080_getQuatK();

    float norm = sqrt(dqw * dqw + dqx * dqx + dqy * dqy + dqz * dqz);
    dqw = dqw / norm;
    dqx = dqx / norm;
    dqy = dqy / norm;
    dqz = dqz / norm;

    float ysqr = dqy * dqy;

    // yaw (z-axis rotation)
    float t3 = +2.0 * (dqw * dqz + dqx * dqy);
    float t4 = +1.0 - 2.0 * (ysqr + dqz * dqz);
    float yaw = atan2(t3, t4);

    return (yaw);
}

// Return the rotation vector quaternion I
float BNO080_getQuatI()
{
    float quat = qToFloat(rawQuatI, rotationVector_Q1);
    return (quat);
}

// Return the rotation vector quaternion J
float BNO080_getQuatJ()
{
    float quat = qToFloat(rawQuatJ, rotationVector_Q1);
    return (quat);
}

// Return the rotation vector quaternion K
float BNO080_getQuatK()
{
    float quat = qToFloat(rawQuatK, rotationVector_Q1); 
    return (quat);
}

// Return the rotation vector quaternion Real
float BNO080_getQuatReal()
{
    float quat = qToFloat(rawQuatReal, rotationVector_Q1);
    return (quat);
}

//This function pulls the data from the command response report

//Unit responds with packet that contains the following:
//shtpHeader[0:3]: First, a 4 byte header
//shtpData[0]: The Report ID
//shtpData[1]: Sequence number (See 6.5.18.2)
//shtpData[2]: Command
//shtpData[3]: Command Sequence Number
//shtpData[4]: Response Sequence Number
//shtpData[5 + 0]: R0
//shtpData[5 + 1]: R1
//shtpData[5 + 2]: R2
//shtpData[5 + 3]: R3
//shtpData[5 + 4]: R4
//shtpData[5 + 5]: R5
//shtpData[5 + 6]: R6
//shtpData[5 + 7]: R7
//shtpData[5 + 8]: R8
uint16_t BNO080_parseCommandReport(void)
{
	if (shtpData[0] == SHTP_REPORT_COMMAND_RESPONSE)
	{
		//The BNO080 responds with this report to command requests. It's up to use to remember which command we issued.
		uint8_t command = shtpData[2]; //This is the Command byte of the response

		if (command == COMMAND_ME_CALIBRATE)
		{
			calibrationStatus = shtpData[5 + 0]; //R0 - Status (0 = success, non-zero = fail)
		}
		return shtpData[0];
	}
	else
	{
		//This sensor report ID is unhandled.
		//See reference manual to add additional feature reports as needed
	}

	//TODO additional feature reports may be strung together. Parse them all.
	return 0;
}
// This function pulls the data from the input report
// The input reports vary in length so this function stores the various 16-bit values as globals

// Unit responds with packet that contains the following:
// shtpHeader[0:3]: First, a 4 byte header
// shtpData[0:4]: Then a 5 byte timestamp of microsecond clicks since reading was taken
// shtpData[5 + 0]: Then a feature report ID (0x01 for Accel, 0x05 for Rotation Vector)
// shtpData[5 + 1]: Sequence number (See 6.5.18.2)
// shtpData[5 + 2]: Status
// shtpData[3]: Delay
// shtpData[4:5]: i/accel x/gyro x/etc
// shtpData[6:7]: j/accel y/gyro y/etc
// shtpData[8:9]: k/accel z/gyro z/etc
// shtpData[10:11]: real/gyro temp/etc
// shtpData[12:13]: Accuracy estimate
uint16_t BNO080_parseInputReport(void)
{
    // Calculate the number of data bytes in this packet
    int16_t dataLength = ((uint16_t)shtpHeader[1] << 8 | shtpHeader[0]);
    dataLength &= ~(1 << 15); // Clear the MSbit. This bit indicates if this package is a continuation of the last.
    // Ignore it for now. TODO catch this as an error and exit

    dataLength -= 4; // Remove the header bytes from the data count

    // The gyro-integrated input reports are sent via the special gyro channel and do no include the usual ID, sequence, and status fields
    if (shtpHeader[2] == CHANNEL_GYRO)
    {
        rawQuatI = (uint16_t)shtpData[1] << 8 | shtpData[0];
        rawQuatJ = (uint16_t)shtpData[3] << 8 | shtpData[2];
        rawQuatK = (uint16_t)shtpData[5] << 8 | shtpData[4];
        rawQuatReal = (uint16_t)shtpData[7] << 8 | shtpData[6];

        return SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR;
    }

    uint8_t status = shtpData[5 + 2] & 0x03; // Get status bits
    uint16_t data1 = (uint16_t)shtpData[5 + 5] << 8 | shtpData[5 + 4];
    uint16_t data2 = (uint16_t)shtpData[5 + 7] << 8 | shtpData[5 + 6];
    uint16_t data3 = (uint16_t)shtpData[5 + 9] << 8 | shtpData[5 + 8];
    uint16_t data4 = 0;
    uint16_t data5 = 0; // We would need to change this to uin32_t to capture time stamp value on Raw Accel/Gyro/Mag reports

    if (dataLength - 5 > 9)
    {
        data4 = (uint16_t)shtpData[5 + 11] << 8 | shtpData[5 + 10];
    }
    if (dataLength - 5 > 11)
    {
        data5 = (uint16_t)shtpData[5 + 13] << 8 | shtpData[5 + 12];
    }

    if (shtpData[5] == SENSOR_REPORTID_ROTATION_VECTOR ||
        shtpData[5] == SENSOR_REPORTID_GAME_ROTATION_VECTOR ||
        shtpData[5] == SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR ||
        shtpData[5] == SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR)
    {
     
        rawQuatI = data1;
        rawQuatJ = data2;
        rawQuatK = data3;
        rawQuatReal = data4;
  
    }

    // TODO additional feature reports may be strung together. Parse them all.
    return shtpData[5];
}

// Updates the latest variables if possible
// Returns false if new readings are not available
bool BNO080_dataAvailable(void)
{
    return (BNO080_getReadings() != 0);
}

uint16_t BNO080_getReadings(void)
{

    if (BNO080_receivePacket() == true)
    {
        // Check to see if this packet is a sensor reporting its data to us
        if (shtpHeader[2] == CHANNEL_REPORTS && shtpData[0] == SHTP_REPORT_BASE_TIMESTAMP)
        {
            return BNO080_parseInputReport(); // This will update the rawAccelX, etc variables depending on which feature report is found
        }
        else if (shtpHeader[2] == CHANNEL_CONTROL)
        {
            return BNO080_parseCommandReport(); // This will update responses to commands, calibrationStatus, etc.
        }
        else if (shtpHeader[2] == CHANNEL_GYRO)
        {
            return BNO080_parseInputReport(); // This will update the rawAccelX, etc variables depending on which feature report is found
        }
    }
    return 0;
}

// Check to see if there is any new data available
// Read the contents of the incoming packet into the shtpData array
bool BNO080_receivePacket(void)
{
    uint8_t buf[4];
    busReadBuf(busDev, 0xFF, buf, 4);

    
    // Get the first four bytes, aka the packet header
    uint8_t packetLSB = buf[0];
    uint8_t packetMSB = buf[1];
    uint8_t channelNumber = buf[2];
    uint8_t sequenceNumber = buf[3]; // Not sure if we need to store this or not

    // Store the header info.
    shtpHeader[0] = packetLSB;
    shtpHeader[1] = packetMSB;
    shtpHeader[2] = channelNumber;
    shtpHeader[3] = sequenceNumber;

    // Calculate the number of data bytes in this packet
    uint16_t dataLength = (((uint16_t)packetMSB) << 8) | ((uint16_t)packetLSB);
    dataLength &= ~(1 << 15); // Clear the MSbit.
    // This bit indicates if this package is a continuation of the last. Ignore it for now.
    // TODO catch this as an error and exit

    // if (_printDebug == true)
    // {
    // 	_debugPort->print(F("receivePacket (I2C): dataLength is: "));
    // 	_debugPort->println(dataLength);
    // }

    if (dataLength == 0)
    {
        // Packet is empty
        return (false); // All done
    }
    dataLength -= 4; // Remove the header bytes from the data count

    BNO080_getData(dataLength);

    // Quickly check for reset complete packet. No need for a seperate parser.
    // This function is also called after soft reset, so we need to catch this
    // packet here otherwise we need to check for the reset packet in multiple
    // places.
    if (shtpHeader[2] == CHANNEL_EXECUTABLE && shtpData[0] == EXECUTABLE_RESET_COMPLETE)
    {
        _hasReset = true;
    }

    return (true); // We're done!
}

// Sends multiple requests to sensor until all data bytes are received from sensor
// The shtpData buffer has max capacity of MAX_PACKET_SIZE. Any bytes over this amount will be lost.
// Arduino I2C read limit is 32 bytes. Header is 4 bytes, so max data we can read per interation is 28 bytes
bool BNO080_getData(uint16_t bytesRemaining)
{
    uint16_t dataSpot = 0; // Start at the beginning of shtpData array

    // Setup a series of chunked 32 byte reads
    while (bytesRemaining > 0)
    {
        uint16_t numberOfBytesToRead = bytesRemaining;
        if (numberOfBytesToRead > (I2C_BUFFER_LENGTH - 4))
            numberOfBytesToRead = (I2C_BUFFER_LENGTH - 4);

    uint8_t buf[numberOfBytesToRead + 4];
    busReadBuf(busDev, 0xFF, buf, numberOfBytesToRead + 4);


        for (uint8_t x = 0; x < numberOfBytesToRead; x++)
        {
            uint8_t incoming = buf[x+4];
            if (dataSpot < MAX_PACKET_SIZE)
            {
                shtpData[dataSpot++] = incoming; // Store data into the shtpData array
            }
            else
            {
                // Do nothing with the data
            }
        }

        bytesRemaining -= numberOfBytesToRead;
    }
    return (true); // Done!
}


//Given the data packet, send the header then the data
//Returns false if sensor does not ACK
//TODO - Arduino has a max 32 byte send. Break sending into multi packets if needed.
bool BNO080_sendPacket(uint8_t channelNumber, uint8_t dataLength)
{
	uint8_t packetLength = dataLength + 4; //Add four bytes for the header

	uint8_t buf[dataLength+ 4];
		//if(packetLength > I2C_BUFFER_LENGTH) return(false); //You are trying to send too much. Break into smaller packets.

	
		//Send the 4 byte packet header
		buf[0]=(packetLength & 0xFF);			  //Packet length LSB
	    buf[1]=(packetLength >> 8);				  //Packet length MSB
		buf[2]=(channelNumber);					  //Channel number
		buf[3]=(sequenceNumber[channelNumber]++); //Send the sequence number, increments with each packet sent, different counter for each channel

		//Send the user's data packet
		for (uint8_t i = 0; i < dataLength; i++)
		{
			buf[4+i]=(shtpData[i]);
		}

		busWriteBuf(busDev, 0xFF, buf, dataLength + 4);

    return (true);
}

//Send command to reset IC
//Read all advertisement packets from sensor
//The sensor has been seen to reset twice if we attempt too much too quickly.
//This seems to work reliably.
void BNO080_softReset(void)
{
	shtpData[0] = 1; //Reset

	//Attempt to start communication with sensor
	BNO080_sendPacket(CHANNEL_EXECUTABLE, 1); //Transmit packet on channel 1, 1 byte

	//Read all incoming data and flush it
	delay(50);
	while (BNO080_receivePacket() == true)
		; //delay(1);
	delay(50);
	while (BNO080_receivePacket() == true)
		; //delay(1);
}
bool BNO080_Init()
{
    busDev = busDeviceInit(BUSTYPE_I2C, DEVHW_BNO08X, 0, 0);
    if (busDev == NULL)
    {
        return false;
    }

    if (!BNO080_deviceDetect(busDev))
    {
        busDeviceDeInit(busDev);
        return false;
    }
//Begin by resetting the IMU
	BNO080_softReset();

	//Check communication with device
	shtpData[0] = SHTP_REPORT_PRODUCT_ID_REQUEST; //Request the product ID and reset info
	shtpData[1] = 0;							  //Reserved

	//Transmit packet on channel 2, 2 bytes
	BNO080_sendPacket(CHANNEL_CONTROL, 2);

	//Now we wait for response
	if (BNO080_receivePacket() == true)
	{
		if (shtpData[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE)
		{
		BNO080_enableARVRStabilizedRotationVector(50);
			return (true);
		}
	}

	return (false); //Something went wrong
}

void BNO080_FetchEulerAngles(int16_t *buffer)
{
     if (BNO080_dataAvailable() == true)
  {
  
    buffer[0] = (int16_t) (BNO080_getRoll() * 1800.0 / PI);
    buffer[1] = (int16_t)(BNO080_getPitch() * 1800.0 / PI); //Pitch has to be reversed to match INAV notation
    buffer[2] = (int16_t)(BNO080_getYaw() * 1800.0 / PI);
  }
}