/*********************************************************************
 BleuIO Example part 4.
 Example of using a Adafruit Feather RP2040 Board with a PDM MEMS 
 Microphone then advertising the results using a BleuIO.

 Copyright (c) 2024 Smart Sensor Devices AB
*********************************************************************/

#include <PDM.h>
// USBHost is defined in usbh_helper.h
#include "usbh_helper.h"
// CDC Host object
Adafruit_USBH_CDC SerialHost;


/* Starting commands for the BleuIO. 
  First it will turn of echo, then set 'BleuIO Arduino Example' as Complete Local Name in the Advertising Response data.
  Lastly it will start Advertising.
*/
#define START_CMDS "ATE0\rAT+ADVRESP=17:09:42:6C:65:75:49:4F:20:41:72:64:75:69:6E:6F:20:45:78:61:6D:70:6C:65\rAT+ADVSTART\r"

// How often we update the advertising message (in seconds)
#define READ_UPDATE_FREQUENCY   1

/* Global variables */
int loop_cnt;
char dongle_cmd[120];
uint16_t noise = 0;
#define MAX_SOUND_PRESSURE_LEVEL 120

// default number of output channels
static const char channels = 1;

// default PCM output frequency
static const int frequency = 16000;

#define PDM_BUFF_SIZE 8000

// Buffer to read samples into, each sample is 16-bits
short sampleBuffer[PDM_BUFF_SIZE];

#define RMS_BUFFER_SIZE PDM_BUFF_SIZE
int16_t rms_buffer[RMS_BUFFER_SIZE];
double rms_normalized = 0.0;
double rms_db_spl = 0.0; 
uint16_t rms_buffer_cursor = 0;

// Number of audio samples read
volatile int samplesRead;

double calculate_rms(int16_t *buffer, uint16_t num_samples) {
  	uint64_t sum = 0;

  	for (uint16_t j = 0; j < num_samples; j++) {
    	rms_buffer[rms_buffer_cursor] = buffer[j];
  		rms_buffer_cursor = (rms_buffer_cursor + 1) % RMS_BUFFER_SIZE;
  	}

  	for (uint16_t j = 0; j < RMS_BUFFER_SIZE; j++) {
    	sum += rms_buffer[j] * rms_buffer[j];
  	}
	double tosquirt = sum / RMS_BUFFER_SIZE;
  	
	return sqrt(tosquirt);
}

double calculate_dbfs_from_normalized(double rms_normalized) {

	double fabsRmsNormalised = fabs(rms_normalized);	
  	return log10(fabsRmsNormalised) * 20;
}


double get_db_spl(double normalized_rms) {
    const uint8_t db_spl_offset = MAX_SOUND_PRESSURE_LEVEL;
  	double out = calculate_dbfs_from_normalized(normalized_rms) + db_spl_offset;
	return out;
}

void audio_callback(int16_t *buffer, uint16_t size) { 

  	// Calculate RMS of last 250 ms 
  	double rms = calculate_rms(buffer, size); 
  
  	// Convert int16_t to double 
  	rms_normalized = rms / (double) 32768.0; 

  	// Convert rms to dB value 
  	rms_db_spl = get_db_spl(rms_normalized);
    noise = (uint16_t) (rms_db_spl);
    Serial.print("avg dB value ");
    Serial.println(rms_db_spl);
}

void forward_serial(void) {
  uint8_t buf[256];

  // Serial -> SerialHost
  // if (Serial.available()) {
  //   size_t count = Serial.read(buf, sizeof(buf));
  //   if (SerialHost && SerialHost.connected()) {
  //     SerialHost.write(buf, count);
  //     SerialHost.flush();
  //   }
  // }

  // SerialHost -> Serial
  if (SerialHost.connected() && SerialHost.available()) {
    size_t count = SerialHost.read(buf, sizeof(buf));
    Serial.println("BleuIO response:"); 
    Serial.write(buf, count);
    Serial.println("----"); 
    Serial.flush();
  }
}

void setup() {
  Serial.begin(9600);
  loop_cnt = 0;
  samplesRead = 0;
  while (!Serial);
  Serial.println(F("BleuIO Noise Sensor Example v1.1b"));

  // Configure the data receive callback
  PDM.onReceive(onPDMdata);

  PDM.setCLK(3);
  PDM.setDIN(2);
  // Optionally set the gain
  // PDM.setGain(30);

  // Initialize PDM with:
  // - one channel (mono mode)
  // - a 16 kHz sample rate for the Arduino Feather RP2040
  if (!PDM.begin(channels, frequency)) 
  {
    Serial.println("Failed to start PDM!");
    while (1);
  }

  // init host stack on controller (rhport) 1
  USBHost.begin(1);

  // Initialize SerialHost
  SerialHost.begin(9600); 
}

/* Generate a BleuIO command to change the Advertising Data along with the Advertising Data we want to set.
   The Advertising Data is setup with the flag Manufacturer Specific Data with a made up Company ID 0x1234 (little endian) and the sensor values as the Data */
void generateAdvData(char * input_buffer, uint16_t db)
{
    sprintf(input_buffer, "AT+ADVDATA=05:FF:34:12:%02X:%02X\r", 
    (uint8_t) (db >> 8), (uint8_t) (db & 0xFF) /*big endian*/
    );
}

void loop() {
  // Forward the output from the dongle to the serial
  forward_serial();

  // Wait for samples to be read
  if (samplesRead == PDM_BUFF_SIZE) 
  {
    audio_callback(sampleBuffer, samplesRead);
    // Clear the read count
    samplesRead = 0;
  }


  if(loop_cnt >= (READ_UPDATE_FREQUENCY * 1000))
  {
      Serial.println("Reading sensors and updating Adv Msg!");

      /*Generate Advertising command to send to the BleuIO*/
      generateAdvData(dongle_cmd, noise);

      /* Sending generated command to BleuIO */
      if (SerialHost && SerialHost.connected()) {
        Serial.println(dongle_cmd);
        SerialHost.write((uint8_t *)dongle_cmd, strlen(dongle_cmd) +1);
        SerialHost.flush();
      }

      loop_cnt = 0;
  } // end of if(loop_cnt >= (READ_UPDATE_FREQUENCY * 1000))

  loop_cnt++;
  delay(1);
}

/**
   Callback function to process the data from the PDM microphone.
   NOTE: This callback is executed as part of an ISR.
   Therefore using `Serial` to print messages inside this function isn't supported.
 * */
void onPDMdata() {
  // Query the number of available bytes

    int bytesAvailable = PDM.available();

    // Read into the sample buffer
    PDM.read(sampleBuffer, bytesAvailable);

    // 16-bit, 2 bytes per sample
    samplesRead += bytesAvailable / 2;
  
}

//------------- Core1 -------------//
void setup1() {
  // configure pio-usb: defined in usbh_helper.h
  rp2040_configure_pio_usb();

  // run host stack on controller (rhport) 1
  // Note: For rp2040 pico-pio-usb, calling USBHost.begin() on core1 will have most of the
  // host bit-banging processing works done in core1 to free up core0 for other works
  USBHost.begin(1);

  // Initialize SerialHost
  SerialHost.begin(9600);
}

void loop1() {
  USBHost.task();
}

//--------------------------------------------------------------------+
// TinyUSB Host callbacks
//--------------------------------------------------------------------+
extern "C" {

// Invoked when a device with CDC interface is mounted
// idx is index of cdc interface in the internal pool.
void tuh_cdc_mount_cb(uint8_t idx) {
  // bind SerialHost object to this interface index
  SerialHost.mount(idx);
  Serial.print("SerialHost is connected to a new CDC device. Idx: ");
  Serial.println(idx);

  /* Send start commands to BleuIO when detecting that  */
  if (SerialHost && SerialHost.connected()) 
  {
    SerialHost.write((uint8_t *)START_CMDS, sizeof(START_CMDS));
    SerialHost.flush();
  }
}

// Invoked when a device with CDC interface is unmounted
void tuh_cdc_umount_cb(uint8_t idx) {
  SerialHost.umount(idx);
  Serial.println("SerialHost is disconnected");
}

}
