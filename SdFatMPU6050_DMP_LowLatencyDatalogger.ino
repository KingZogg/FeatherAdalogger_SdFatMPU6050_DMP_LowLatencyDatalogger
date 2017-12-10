/**
* This program logs data to a binary file.  Functions are included
* to convert the binary file to a csv text file.
*
* Samples are logged at regular intervals.  The maximum logging rate
* depends on the quality of your SD card and the time required to
* read sensor data.  This example has been tested at 500 Hz with
* good SD card on an Uno.  4000 HZ is possible on a Due.
*
* If your SD card has a long write latency, it may be necessary to use
* slower sample rates.  Using a Mega Arduino helps overcome latency
* problems since 13 512 byte buffers will be used.
*
* Data is written to the file using a SD multiple block write command.
*/
#include <SPI.h>
#include <SdFat.h>
#include <SdFatUtil.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <stdint.h>
#include "SparkFunBME280.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file
#include "HMC5883L.h"
#include <RTClib.h>
#include "UserDataType.h"  // Edit this include file to change data_t.

#define VBATPIN A7

float EMA_a = 0.6;      //initialization of EMA alpha

#define BUTTON_PIN 6
#define INTERRUPT_PIN 5  // MPU6050 interrupt pin is using pin 5 on my adafruit datalogger feather.
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6, feather is 13 too)
#define OUTPUT_READABLE_WORLDACCEL

						// MPU control/status vars
boolean dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[128]; // FIFO storage buffer

int ButtonPresses = 0;


						 // orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
uint16_t aaWorldx;
uint16_t aaWorldy;
uint16_t aaWorldz;

float temperature;
float altitude;
float humidity;

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile boolean mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
	mpuInterrupt = true;
}


boolean debug = false; //This MUST be set to false for the button to work in stand alone mode with no USB attached.

float measuredvbat;

// used for flashing the led.
#define LED_ON_LOGGING 1       //milliseconds
#define LED_OFF_LOGGING 10000
#define LED_ON_TRUNCATE_FILE 1000       //milliseconds
#define LED_OFF_TRUNCATE_FILE 1000

#define LED_ON_CONVERT_FILE 1       //milliseconds
#define LED_OFF_CONVERT_FILE 500

unsigned long ms;        //time from millis()
unsigned long msLast;    //last time the LED changed state
boolean ledState;        //current LED state

						 // RTC setup
int16_t year;
uint8_t month;
uint8_t day;
uint8_t hour;
uint8_t min;
uint8_t sec;

// Magnotometer setup.
int16_t mx;
int16_t my;
int16_t mz;

int16_t logFifo;

//MPU6050 mpu; // <-- used for default chip address without vcc tied to ADO 0x68
MPU6050 mpu(0x69); // <-- use for AD0 high
HMC5883L mag; // Magnetometer
RTC_DS3231 rtc; //Real time clock
BME280 BME280_PressTempHumid; //pressure/temp/humidity


				 //========================================================================================================================
				 //                                               Acquire a data record.
				 //========================================================================================================================
void acquireData(data_t* data) {
	data->time = micros();

	mpu.getMotion6(&data->ax, &data->ay, &data->az, &data->gx, &data->gy, &data->gz);

	mag.getHeading(&data->mx, &data->my, &data->mz);

	data->aaWorldx = aaWorldx;
	data->aaWorldy = aaWorldy;
	data->aaWorldz = aaWorldz;

	data->measuredvbat = measuredvbat;

	data->year = year;
	data->month = month;
	data->day = day;
	data->hour = hour;
	data->min = min;
	data->sec = sec;

	data->mx = mx;
	data->my = my;
	data->mz = mz;

	data->temperature = temperature;
	data->altitude = altitude;
	data->humidity = humidity;

	data->logFifo = logFifo;
}

// setup AVR I2C
void setupData() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	Wire.begin();
	Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
	Fastwire::setup(400, true);
	Serial.println(F("Using Fastwire"));
#endif  
}



//==============================================================================
// Start of configuration constants.
//==============================================================================
//Interval between data records in microseconds.
const uint32_t LOG_INTERVAL_USEC = 40000;
//------------------------------------------------------------------------------
// Pin definitions.
//
// SD chip select pin.
const uint8_t SD_CS_PIN = 4;
//
// Digital pin to indicate an error, set to -1 if not used.
// The led blinks for fatal errors. The led goes on solid for SD write
// overrun errors and logging continues.
const int8_t ERROR_LED_PIN = 13;
//------------------------------------------------------------------------------
// File definitions.
//
// Maximum file size in blocks.
// The program creates a contiguous file with FILE_BLOCK_COUNT 512 byte blocks.
// This file is flash erased using special SD commands.  The file will be
// truncated if logging is stopped early.
// must also be one exactly of these depending on how big you want the file size to be.
// 25600 51200 1024000 20480000 4096000 8192000 16384000 for a 16 gig card ... etc.
const uint32_t FILE_BLOCK_COUNT = 16384000; //16GB

											// log file base name.  Must be six characters or less.
#define FILE_BASE_NAME "data"
											//------------------------------------------------------------------------------
											// Buffer definitions.
											//
											// The logger will use SdFat's buffer plus BUFFER_BLOCK_COUNT additional
											// buffers.
											//
#ifndef RAMEND
											// Assume ARM. Use total of nine 512 byte buffers.
const uint8_t BUFFER_BLOCK_COUNT = 8;
//
#elif RAMEND < 0X8FF
#error Too little SRAM
											//
#elif RAMEND < 0X10FF
											// Use total of two 512 byte buffers.
const uint8_t BUFFER_BLOCK_COUNT = 1;
//
#elif RAMEND < 0X20FF
											// Use total of five 512 byte buffers.
const uint8_t BUFFER_BLOCK_COUNT = 4;
//
#else  // RAMEND
											// Use total of 13 512 byte buffers.
const uint8_t BUFFER_BLOCK_COUNT = 12;
#endif  // RAMEND
//==============================================================================
// End of configuration constants.
//==============================================================================
// Temporary log file.  Will be deleted if a reset or power failure occurs.
#define TMP_FILE_NAME "tmp_log.bin"

// Size of file base name.  Must not be larger than six.
const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;

SdFat sd;

SdBaseFile binFile;

char binName[13] = FILE_BASE_NAME "00.bin";

// Number of data records in a block.
const uint16_t DATA_DIM = (512 - 4) / sizeof(data_t);

//Compute fill so block size is 512 bytes.  FILL_DIM may be zero.
const uint16_t FILL_DIM = 512 - 4 - DATA_DIM * sizeof(data_t);

struct block_t {
	uint16_t count;
	uint16_t overrun;
	data_t data[DATA_DIM];
	uint8_t fill[FILL_DIM];
};

const uint8_t QUEUE_DIM = BUFFER_BLOCK_COUNT + 2;

block_t* emptyQueue[QUEUE_DIM];
uint8_t emptyHead;
uint8_t emptyTail;

block_t* fullQueue[QUEUE_DIM];
uint8_t fullHead;
uint8_t fullTail;

// Advance queue index.
inline uint8_t queueNext(uint8_t ht) {
	return ht < (QUEUE_DIM - 1) ? ht + 1 : 0;
}
//==============================================================================
// Error messages stored in flash.
#define error(msg) errorFlash(F(msg))
//------------------------------------------------------------------------------
void errorFlash(const __FlashStringHelper* msg) {
	sd.errorPrint(msg);
	fatalBlink();
}
//------------------------------------------------------------------------------
//
void fatalBlink() {
	while (true) {
		if (ERROR_LED_PIN >= 0) {
			digitalWrite(ERROR_LED_PIN, HIGH);
			delay(200);
			digitalWrite(ERROR_LED_PIN, LOW);
			delay(200);
		}
	}
}

//========================================================================================================================
//                                               Print a data record.
//========================================================================================================================
// only used for convert .bin to .csv and the dumpdata routines.
void printData(Print* pr, data_t* data) {

	pr->print(data->year);
	pr->write('/');
	pr->print(data->month);
	pr->write('/');
	pr->print(data->day);
	pr->write(' ');
	pr->print(data->hour);
	pr->write(':');
	pr->print(data->min);
	pr->write(':');
	pr->print(data->sec);
	pr->write('\t');


	pr->print(data->time);
	pr->write('\t');
	pr->print(data->measuredvbat);
	pr->write('\t');

	pr->print(data->aaWorldx);
	pr->write('\t');
	pr->print(data->aaWorldy);
	pr->write('\t');
	pr->print(data->aaWorldz);
	pr->write('\t');

	
	pr->print(data->mx);
	pr->write('\t');
	pr->print(data->my);
	pr->write('\t');
	pr->print(data->mz);
	pr->write('\t');

	pr->print(data->ax);
	pr->write('\t');
	pr->print(data->ay);
	pr->write('\t');
	pr->print(data->az);
	pr->write('\t');
	pr->print(data->gx);
	pr->write('\t');
	pr->print(data->gy);
	pr->write('\t');
	pr->print(data->gz);
	pr->write('\t');
	

	pr->print(data->temperature);
	pr->write('\t');
	pr->print(data->altitude);
	pr->write('\t');
	pr->print(data->humidity);
	pr->write('\t');
	pr->println(data->logFifo);


}

// Print data header.
void printHeader(Print* pr) {
	pr->println(F("#Date	MicroSeconds	Batt	worldx	worldy	worldz	mx	my	mz	ax	ay	az	gx	gy	gz	Temp	Altitude	Humidity	Fifo"));
}

//==============================================================================
// Convert binary file to csv file.
void binaryToCsv() {
	uint8_t lastPct = 0;
	block_t block;
	uint32_t t0 = millis();
	uint32_t syncCluster = 0;
	SdFile csvFile;
	char csvName[13];
	

	char MyBinName[] = "data00.bin";
	binFile.open(MyBinName, O_READ);
	
	Serial.println(MyBinName);
	
	if (!binFile.isOpen()) {
		Serial.println();
		Serial.println(F("On Convert - No current binary file"));
		return;
	}
	binFile.rewind();
	// Create a new csvFile.
	strcpy(csvName, MyBinName);
	strcpy(&csvName[BASE_NAME_SIZE + 3], "csv");

	if (!csvFile.open(csvName, O_WRITE | O_CREAT | O_TRUNC)) {
		error("open csvFile failed");
	}


	// get the date and time.
	DateTime now = rtc.now();
	year = now.year();
	month = now.month();
	day = now.day();
	hour = now.hour();
	min = now.minute();
	sec = now.second();

	// Create the Timestamp for the file on the SD.
	if (!csvFile.timestamp(T_CREATE, year, month, day, hour, min, sec)) {
		error("create .CSV timestamp failed");
	}

	// Write the Timestamp to the file on the SD.
	if (!csvFile.timestamp(T_WRITE, year, month, day, hour, min, sec)) {
		error("write .CSV timestamp failed");
	}



	Serial.println();
	Serial.print(F("Writing: "));
	Serial.print(csvName);
	printHeader(&csvFile);
	uint32_t tPct = millis();
	while (!Serial.available() && binFile.read(&block, 512) == 512) {



		uint16_t i;
		if (block.count == 0) {
			Serial.println(F("hit break"));
			break;
		}
		if (block.overrun) {
			csvFile.print(F("OVERRUN,"));
			csvFile.println(block.overrun);
		}
		for (i = 0; i < block.count; i++) {
			printData(&csvFile, &block.data[i]);
		}

		// blink the LED.
		ms = millis();

		if (ms - msLast >(ledState ? LED_ON_CONVERT_FILE : LED_OFF_CONVERT_FILE)) {
			digitalWrite(LED_PIN, ledState = !ledState);
			msLast = ms;
		}

		if (csvFile.curCluster() != syncCluster) {
			csvFile.sync();
			syncCluster = csvFile.curCluster();
		}
		if ((millis() - tPct) > 1000) {
			uint8_t pct = binFile.curPosition() / (binFile.fileSize() / 100);
			if (pct != lastPct) {
				tPct = millis();
				lastPct = pct;
				Serial.print(pct, DEC);
				Serial.println('%');
			}
		}
		if (Serial.available()) {
			break;
		}
	}
	csvFile.close();
	Serial.print(F("Done: "));
	Serial.print(0.001*(millis() - t0));
	Serial.println(F(" Seconds"));
	
	
}
//------------------------------------------------------------------------------
//Dump data file and check for overruns
void checkOverrun() {
	boolean headerPrinted = false;
	block_t block;
	uint32_t bgnBlock, endBlock;
	uint32_t bn = 0;

	if (!binFile.isOpen()) {
		Serial.println();
		Serial.println(F("On CheckOverrun - No current binary file"));
		return;
	}
	if (!binFile.contiguousRange(&bgnBlock, &endBlock)) {
		error("contiguousRange failed");
	}
	binFile.rewind();
	Serial.println();
	Serial.println(F("Checking overrun errors"));
	while (binFile.read(&block, 512) == 512) {
		if (block.count == 0) {
			break;
		}
		if (block.overrun) {
			if (!headerPrinted) {
				Serial.println();
				Serial.println(F("Overruns:"));
				Serial.println(F("fileBlockNumber,sdBlockNumber,overrunCount"));
				headerPrinted = true;
			}
			Serial.print(bn);
			Serial.print(',');
			Serial.print(bgnBlock + bn);
			Serial.print(',');
			Serial.println(block.overrun);
		}
		bn++;
	}
	if (!headerPrinted) {
		Serial.println(F("No errors found"));
	}
	else {
		Serial.println(F("Done"));
	}
}





//------------------------------------------------------------------------------
// log data
// max number of blocks to erase per erase call
uint32_t const ERASE_SIZE = 262144L;
void logData() {
	uint32_t bgnBlock, endBlock;

	// Allocate extra buffer space.
	block_t block[BUFFER_BLOCK_COUNT];
	block_t* curBlock = 0;
	Serial.println();

	// Find unused file name.
	if (BASE_NAME_SIZE > 6) {
		error("FILE_BASE_NAME too long");
	}
	while (sd.exists(binName)) {
		if (binName[BASE_NAME_SIZE + 1] != '9') {
			binName[BASE_NAME_SIZE + 1]++;
		}
		else {
			binName[BASE_NAME_SIZE + 1] = '0';
			if (binName[BASE_NAME_SIZE] == '9') {
				error("Can't create file name");
			}
			binName[BASE_NAME_SIZE]++;
		}
	}
	// Delete old tmp file.
	if (sd.exists(TMP_FILE_NAME)) {
		Serial.println(F("Deleting tmp file"));
		if (!sd.remove(TMP_FILE_NAME)) {
			error("Can't remove tmp file");
		}
	}

	// Create new file.
	digitalWrite(LED_PIN, LOW);
	Serial.println(F("Creating new file"));
	binFile.close();

	if (!binFile.createContiguous(sd.vwd(),
		TMP_FILE_NAME, 512 * FILE_BLOCK_COUNT)) {
		error("createContiguous failed");
	}


	// get the date and time.
	DateTime now = rtc.now();
	year = now.year();
	month = now.month();
	day = now.day();
	hour = now.hour();
	min = now.minute();
	sec = now.second();

	// Create the Timestamp for the file on the SD.
	if (!binFile.timestamp(T_CREATE, year, month, day, hour, min, sec)) {
		error("Create .BIN timestamp failed");
	}

	// Write the Timestamp to the file on the SD.
	if (!binFile.timestamp(T_WRITE, year, month, day, hour, min, sec)) {
		error("Write .BIN timestamp failed");
	}



	// Get the address of the file on the SD.
	if (!binFile.contiguousRange(&bgnBlock, &endBlock)) {
		error("contiguousRange failed");
	}
	// Use SdFat's internal buffer.
	uint8_t* cache = (uint8_t*)sd.vol()->cacheClear();
	if (cache == 0) {
		error("cacheClear failed");
	}

	// Flash erase all data in the file.
	Serial.println(F("Initialising binary data file"));
	uint32_t bgnErase = bgnBlock;
	uint32_t endErase;
	while (bgnErase < endBlock) {
		endErase = bgnErase + ERASE_SIZE;
		if (endErase > endBlock) {
			endErase = endBlock;
		}
		if (!sd.card()->erase(bgnErase, endErase)) {
			error("initialise failed");
		}
		bgnErase = endErase + 1;
	}
	// Start a multiple block write.
	if (!sd.card()->writeStart(bgnBlock, FILE_BLOCK_COUNT)) {
		error("writeBegin failed");
	}
	// Initialize queues.
	emptyHead = emptyTail = 0;
	fullHead = fullTail = 0;

	// Use SdFat buffer for one block.
	emptyQueue[emptyHead] = (block_t*)cache;
	emptyHead = queueNext(emptyHead);

	// Put rest of buffers in the empty queue.
	for (uint8_t i = 0; i < BUFFER_BLOCK_COUNT; i++) {
		emptyQueue[emptyHead] = &block[i];
		emptyHead = queueNext(emptyHead);
	}
	Serial.println(F("Logging - press button to stop"));
	// Wait for Serial Idle.
	Serial.flush();
	delay(10);
	uint32_t bn = 0;
	uint32_t t0 = millis();
	uint32_t t1 = t0;
	uint32_t overrun = 0;
	uint32_t overrunTotal = 0;
	uint32_t count = 0;
	uint32_t maxLatency = 0;
	int32_t diff;
	// Start at a multiple of interval.
	uint32_t logTime = micros() / LOG_INTERVAL_USEC + 1;
	logTime *= LOG_INTERVAL_USEC;
	boolean closeFile = false;
	//=============================================================================================================================================
	//========================================                  Main Logging Loop                 =================================================
	//=============================================================================================================================================
	while (1) {

		// if programming failed, don't try to do anything
		if (!dmpReady) return;

		// wait for MPU interrupt or extra packet(s) available
		while (!mpuInterrupt && fifoCount < packetSize) {
		}
		mpuInterrupt = false;
		fifoCount = mpu.getFIFOCount();
		// check for overflow (this should never happen unless our code is too inefficient)
		if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
			// reset so we can continue cleanly
			mpu.resetFIFO();
			//Serial.println(F("FIFO overflow!"));

			// otherwise, check for DMP data ready interrupt (this should happen frequently)
		}
		else if (mpuIntStatus & 0x02) {
			// wait for correct available data length, should be a VERY short wait
			while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

			// read a packet from FIFO
			mpu.getFIFOBytes(fifoBuffer, packetSize);
			mpu.resetFIFO();             //http://arduino.stackexchange.com/questions/10308/how-to-clear-fifo-buffer-on-mpu6050
										 // track FIFO count here in case there is > 1 packet available
										 // (this lets us immediately read more without waiting for an interrupt)
			fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_WORLDACCEL
			// display initial world-frame acceleration, adjusted to remove gravity
			// and rotated based on known orientation from quaternion
			mpu.dmpGetQuaternion(&q, fifoBuffer);
			mpu.dmpGetAccel(&aa, fifoBuffer);
			mpu.dmpGetGravity(&gravity, &q);
			mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
			mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

			// read raw heading measurements from mag device
			mag.getHeading(&mx, &my, &mz);

			aaWorldx = aaWorld.x;
			aaWorldy = aaWorld.y;
			aaWorldz = aaWorld.z;

			temperature = BME280_PressTempHumid.readTempC(), 2;
			altitude = BME280_PressTempHumid.readFloatAltitudeMeters(), 2;
			humidity = BME280_PressTempHumid.readFloatHumidity(), 2;

			logFifo = fifoCount;
		
			Serial.print(BME280_PressTempHumid.readTempC(), 2);
			Serial.print("\t");

			Serial.print(BME280_PressTempHumid.readFloatAltitudeMeters(), 2);
			Serial.print("\t");

			Serial.print(BME280_PressTempHumid.readFloatHumidity(), 2);
			Serial.print("\t");


			Serial.print("aworld\t");
			Serial.print(aaWorld.x);
			Serial.print("\t");
			Serial.print(aaWorld.y);
			Serial.print("\t");
			Serial.print(aaWorld.z);
			Serial.print("\t");
			Serial.print(mx);
			Serial.print("\t");
			Serial.print(my);
			Serial.print("\t");
			Serial.println(mz);
		



#endif

		}

		// get the date and time.
		DateTime now = rtc.now();
		year = now.year();
		month = now.month();
		day = now.day();
		hour = now.hour();
		min = now.minute();
		sec = now.second();

		// measure the battery voltage.
		measuredvbat = analogRead(VBATPIN);
		measuredvbat *= 2;    // we divided by 2, so multiply back
		measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
		measuredvbat /= 1024; // convert to voltage

							  //Apply the EMA to the reading
		measuredvbat = (EMA_a*measuredvbat) + ((1 - EMA_a)*measuredvbat);


		//Serial.println(measuredvbat);

		//While logging, blink the LED.
		ms = millis();

		if (ms - msLast > (ledState ? LED_ON_LOGGING : LED_OFF_LOGGING)) {
			digitalWrite(LED_PIN, ledState = !ledState);
			msLast = ms;
		}

		// Time for next data record.
		logTime += LOG_INTERVAL_USEC;
		//Start logging :

		if ((digitalRead(BUTTON_PIN) == LOW) || (measuredvbat < 3.60)) {
			closeFile = true;
		}

		if (closeFile) {
			if (curBlock != 0 && curBlock->count >= 0) {
				// Put buffer in full queue.
				fullQueue[fullHead] = curBlock;
				fullHead = queueNext(fullHead);
				curBlock = 0;
			}
		}
		else {
			if (curBlock == 0 && emptyTail != emptyHead) {
				curBlock = emptyQueue[emptyTail];
				emptyTail = queueNext(emptyTail);
				curBlock->count = 0;
				curBlock->overrun = overrun;
				overrun = 0;
			}
			do {
				diff = logTime - micros();
			} while (diff > 0);
			if (diff < -10) {
				error("LOG_INTERVAL_USEC too small");
			}
			if (curBlock == 0) {
				overrun++;
			}
			else {
				acquireData(&curBlock->data[curBlock->count++]);
				if (curBlock->count == DATA_DIM) {
					fullQueue[fullHead] = curBlock;
					fullHead = queueNext(fullHead);
					curBlock = 0;
				}
			}
		}

		if (fullHead == fullTail) {
			// Exit loop if done.
			if (closeFile) {
				break;
			}
		}
		else if (!sd.card()->isBusy()) {
			// Get address of block to write.
			block_t* pBlock = fullQueue[fullTail];
			fullTail = queueNext(fullTail);
			// Write block to SD.
			uint32_t usec = micros();
			if (!sd.card()->writeData((uint8_t*)pBlock)) {
				error("write data failed");
			}
			usec = micros() - usec;
			t1 = millis();
			if (usec > maxLatency) {
				maxLatency = usec;
			}
			count += pBlock->count;

			// Add overruns and possibly light LED.
			if (pBlock->overrun) {
				overrunTotal += pBlock->overrun;
				if (ERROR_LED_PIN >= 0) {
					digitalWrite(ERROR_LED_PIN, HIGH);
				}
			}
			// Move block to empty queue.
			emptyQueue[emptyHead] = pBlock;
			emptyHead = queueNext(emptyHead);
			bn++;
			if (bn == FILE_BLOCK_COUNT) {
				// File full so stop
				break;
			}

		}
	}
	if (!sd.card()->writeStop()) {
		error("writeStop failed");
	}
	// Truncate file if recording stopped early.
	if (bn != FILE_BLOCK_COUNT) {
		Serial.println(F("Truncating file"));
		if (!binFile.truncate(512L * bn)) {
			error("Can't truncate file");
		}
	}
	if (!binFile.rename(sd.vwd(), binName)) {
		error("Can't rename file");
	}
	Serial.print(F("File renamed: "));
	Serial.println(binName);
	Serial.print(F("Max block write usec: "));
	Serial.println(maxLatency);
	Serial.print(F("Record time sec: "));
	Serial.println(0.001*(t1 - t0), 3);
	Serial.print(F("Sample count: "));
	Serial.println(count);
	Serial.print(F("Samples/sec: "));
	Serial.println((1000.0)*count / (t1 - t0));
	Serial.print(F("Overruns: "));
	Serial.println(overrunTotal);
	
	//Convert files to CSV
	binaryToCsv();
	checkOverrun();


	Serial.println(F("Done"));

	for (int i = 0; i < 5; i++)
	{
		digitalWrite(LED_PIN, LOW);
		delay(500);
		digitalWrite(LED_PIN, HIGH);
		delay(50);
	}



}
//------------------------------------------------------------------------------
void setup(void) {

	//Setup BME280 bosh pressure/temp/humidity sensor
	BME280_PressTempHumid.settings.commInterface = I2C_MODE;
	BME280_PressTempHumid.settings.I2CAddress = 0x77;
	BME280_PressTempHumid.settings.runMode = 3; //Normal mode
	BME280_PressTempHumid.settings.tStandby = 0;
	BME280_PressTempHumid.settings.filter = 4;
	BME280_PressTempHumid.settings.tempOverSample = 2;
	BME280_PressTempHumid.settings.pressOverSample = 5;
	BME280_PressTempHumid.settings.humidOverSample = 1;


	// Setup the button with an internal pull-up :
	pinMode(BUTTON_PIN, INPUT_PULLUP);

	if (ERROR_LED_PIN >= 0) {
		pinMode(ERROR_LED_PIN, OUTPUT);
	}


	if (debug == true) {
		Serial.begin(115200); // opens serial port, sets data rate.

		while (!Serial) {}

		Serial.print(F("FreeRam: "));
		Serial.println(FreeRam());
		Serial.print(F("Records/block: "));
		Serial.println(DATA_DIM);
	};


	if (sizeof(block_t) != 512) {
		error("Invalid block size");
	}

	setupData();
	// initialize file system.
	if (!sd.begin(SD_CS_PIN, SPI_FULL_SPEED)) {
		sd.initErrorPrint();
		fatalBlink();
	}

	//	Serial.println(F("Initializing I2C devices..."));
	delay(10);  //Make sure sensor had enough time to turn on. BME280 requires 2ms to start up.
	BME280_PressTempHumid.begin();
	mpu.initialize();
	mag.initialize();
	pinMode(INTERRUPT_PIN, INPUT);

	// verify connection
	Serial.println(F("Testing device connections..."));
	Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
	Serial.println(mag.testConnection() ? F("HMC5883L connection successful") : F("HMC5883L connection failed"));
	//Serial.println(BME280_PressTempHumid.testConnection() ? F("BME280 connection successful") : F("BME280 connection failed"));

	// Startup Real time clock.
	if (!rtc.begin()) {
		Serial.println("Couldn't find RTC");
		while (1);
	}

	if (rtc.lostPower()) {
		Serial.println("RTC lost power, lets set the time!");
		// following line sets the RTC to the date & time this sketch was compiled
		rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
		// This line sets the RTC with an explicit date & time, for example to set
		// January 21, 2014 at 3am you would call:
		// rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
	}

	// load and configure the DMP
	Serial.println(F("Initializing DMP..."));
	devStatus = mpu.dmpInitialize();

	// supply your own gyro offsets here, scaled for min sensitivity
	mpu.setXGyroOffset(220);
	mpu.setYGyroOffset(76);
	mpu.setZGyroOffset(-85);
	mpu.setZAccelOffset(1892); // 1688 factory default for my test chip

							   // make sure it worked (returns 0 if so)
	if (devStatus == 0) {
		// turn on the DMP, now that it's ready
		Serial.println(F("Enabling DMP..."));
		mpu.setDMPEnabled(true);

		// enable Arduino interrupt detection
		Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
		attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
		mpuIntStatus = mpu.getIntStatus();

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		Serial.println(F("DMP ready! Waiting for first interrupt..."));
		dmpReady = true;

		// get expected DMP packet size for later comparison
		packetSize = mpu.dmpGetFIFOPacketSize();
	}
	else {
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		Serial.print(F("DMP Initialization failed (code "));
		Serial.print(devStatus);
		Serial.println(F(")"));
	}

	Serial.println(F("Press Button to Start Logging"));


}
//------------------------------------------------------------------------------

void loop(void) {


	if (digitalRead(BUTTON_PIN) == LOW) {
		ButtonPresses++;
		Serial.println(F("Start Button Pressed"));

		delay(1000);

		logData();



	}


	if ((ButtonPresses) >= 2) {
		
		Serial.println(F("Stop Button Pressed"));

		binaryToCsv();
		checkOverrun();

	};

};


/* read raw heading measurements from device
mag.getHeading(&mx, &my, &mz);

// display tab-separated gyro x/y/z values
Serial.print("mag:\t");
Serial.print(mx); Serial.print("\t");
Serial.print(my); Serial.print("\t");
Serial.print(mz); Serial.print("\t");

// To calculate heading in degrees. 0 degree indicates North
float heading = atan2(my, mx);
if (heading < 0)
heading += 2 * M_PI;
Serial.print("heading:\t");
Serial.println(heading * 180 / M_PI);

*/