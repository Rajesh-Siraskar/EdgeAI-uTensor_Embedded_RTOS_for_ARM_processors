// ----------------------------------------------------------------------------------------------------
// Predictive Maintenance using uTensor DNN
// ----------------------------------------------------------------------------------------------------
// Ver 1.0 | 06-Mar-2019 | Neil Tan's issue work-around
// Ver 4.3 | 16-Mar-2019 | uTensor DNN V2 
// Ver 4.4 | 16-Mar-2019 | LCD interface 
// Ver 4.5 | 16-Mar-2019 | Improve LCD interface 
// Ver 4.6 | 16-Mar-2019 | Move sample data to header file 
// Ver 5.0 | 16-Mar-2019 | Add threads and separate main prediction loop and UI 
// Ver 5.1 | 16-Mar-2019 | Move data acquistion and prediction to another thread and not main?
// Ver 5.2 | 16-Mar-2019 | Correct threads
// Ver 6.0 | 17-Mar-2019 | Re-coding of zones and training lables
// Ver 6.1 | 17-Mar-2019 | Added noise to sample data
// Ver 6.3 | 17-Mar-2019 | Re-program LED glow to indicate safe and warning zones
// Ver 6.5 | 20-Mar-2019 | Integrate external sensor readings for prediction
// Ver 7.0 | 21-Mar-2019 | Code completed with Sensor integration
// ----------------------------------------------------------------------------------------------------
// OUTPUT = Time-to-failure labelling:
// State            | Training Label | Prediction Label
// -----------------------------------------------------  
// Motor fault      |    0           |   0
// Less than 15 mins|   15           |   1
//           30 mins|   30           |   2
//           60 mins|   60           |   3
//           90 mins|   90           |   4
// Normal operation | 9999           |   5
//
// INPUT = Sensors i.e. feature data: 
// Active.Current,	DC.Link.Voltage,	Temperature,	Vx,	Vy,	Vz and StateNCode
// ----------------------------------------------------------------------------------------------------
// References: Using uTensor based DNN was built using a tutorial by Neil Tan
//  (https://blog.hackster.io/simple-neural-network-on-mcus-a7cbd3dc108c)
// ----------------------------------------------------------------------------------------------------

// ----------------------------------------------------------------------------------------------------
// LIBRARIES
// ----------------------------------------------------------------------------------------------------
// Standard libraries
#include <mbed.h>
#include <mbed_events.h>
#include <stdio.h>
#include <LCD_DISCO_L476VG.h>   // LCD support

// Sensor libraries
#include "ADXL345.h"            // Vibration sensor
#include "LM35.h"               // Temperature sensor
#include <pinmap.h>             // Pin-mapping
#include "PeripheralPins.h"     // Peripheral pin-maps

bool SIMULATED_DATA = false;

// uTensor and Deep-Neural-Network (DNN) support
#include "DNN_model.hpp"        // DNN file generated using uTensor cli
#include "SampleData.h"         // Sample data to simulate data acquistion
#include "tensor.hpp"           // Tensor classes

// ----------------------------------------------------------------------------------------------------
// EDGE-AI: PREDICTIVE-MAINTENANCE SYSTEM: uTensor, DNN set-up
// ----------------------------------------------------------------------------------------------------

// DNN uTensor variables: Network input configuration
const int time_prediction_zones =  6;     // Inputs: 6 sensor inputs
const unsigned long sensor_features =  6; // Outputs: 6 time zones of prediction

// Create thread to run data-acquisition and perdiction of time-to-failure
Thread threadDataAcquisition;
void dataAcqusition_thread_function(void);

// Time-to-failure zones
enum ZONES {FAULT, LT_15, LT_30, LT_60, LT_90, NORMAL};

// ----------------------------------------------------------------------------------------------------
// CONFIGURATIONS: Sensors, LED and LCD
// ----------------------------------------------------------------------------------------------------
// Configure LEDs and LCD
DigitalOut led_GREEN(LED1);
DigitalOut led_RED(LED2);
LCD_DISCO_L476VG lcd;
bool BLINK_WARNING_LED = false;

// Define pins for the two sensors
AnalogIn LM35(PA_1);
ADXL345 accelerometer(PE_15, PE_14, PE_13, PE_12);

const PinMap PinMap_ADC[] = {{PA_1, ADC_1, STM_PIN_DATA_EXT(STM_MODE_ANALOG_ADC_CONTROL, GPIO_NOPULL, 0, 6, 0)}};
// AnalogIn analog_value(PA_1); 

// UI threads and functions
Thread threadLEDWarning;
void ledWarning_thread_function(void);
void Initialise_Warning_Panel(void);
void Warning_UI(int pred_label, int actual_data_label);
void Initialise_Sensors(void);

// ----------------------------------------------------------------------------------------------------
// SENSOR: Base and derived classes
// ----------------------------------------------------------------------------------------------------
// Base class
class Sensor {
    public:
        void InitialiseSensor(int id) {
            identifier = id;
        }    
        void ReadData() { }

    protected: 
      int identifier;
};

// Derived class: Temperature
class Temperature: public Sensor {    
    public:          
        float ReadData() {
          /* float t = 24.0;
          float meas;

          meas = analog_value.read(); // converts the analog value to eq. temprature. 
          t = ((meas*5000)/100); 
          
          printf("Temp: %2.2f degree C\r\n",temp_value); wait(1); }; }

          */

          float t = 48.9796*LM35.read();
          return (t);
        }    
};

// Derived class: Vibration
class Vibration: public Sensor {
    public:          
        void InitialiseSensor(int id) { 
            identifier = id;

            //  Test device by gettting the DeviceID
            printf("Test ADXL345...\r\n");
            printf(" - Device ID is: 0x%02x\r\n", accelerometer.getDevId()); 

            // Move into standby mode to configure the device
            accelerometer.setPowerControl(0x00);

            // Calibrate: Full resolution, +/-16g, 4mg/LSB.
            accelerometer.setDataFormatControl(0x0B);            
            // 3.2kHz data rate
            accelerometer.setDataRate(ADXL345_3200HZ);            
        }

    public:
        int* ReadData() { 
            // Set measurement mode
            accelerometer.setPowerControl(0x08);

            // Create space to read data
            int data_array[3] = {0, 0, 0};  
            accelerometer.getOutput(data_array);            
            return (data_array);
        }    
};

// Instantiate objects of class Temperature and Vibration
Temperature temperature;
Vibration vibration;

// ----------------------------------------------------------------------------------------------------
// MAIN function
// ----------------------------------------------------------------------------------------------------
int main(void) {
  printf("\n ==============================================================================");
  printf("\n    PREDICTIVE MAINTENANCE SYSTEM");
  printf("\n    uTensor Deep Neural Network Model");
  printf("\n    V.7.0 [21-Mar-2019: 07:53 PM]");
  printf("\n ==============================================================================\n");
  
  // UI: Initialise Warning-panel and warning-LED's  
  Initialise_Warning_Panel();
  threadLEDWarning.start(ledWarning_thread_function);

  // Initialise the sensors
  if (!SIMULATED_DATA) { Initialise_Sensors(); }

  // Initialise the data acquisition and prediction thread  
  threadDataAcquisition.start(dataAcqusition_thread_function); 

  return (0);
}

// ----------------------------------------------------------------------------------------------------
// Function: Initialise the sensors (temperature and vibration)
// ----------------------------------------------------------------------------------------------------
void  Initialise_Sensors(void) {  
  printf("\n Initialising TEMPERATURE sensor...");
  temperature.InitialiseSensor(01);

  printf("\n Initialising VIBRATION sensor...");
  vibration.InitialiseSensor(02);
}

// ----------------------------------------------------------------------------------------------------
// DATA ACQUISITION THREAD
//  - Collect sensor data and predict using the per-trained DNN model
// ----------------------------------------------------------------------------------------------------
void dataAcqusition_thread_function(void){
  
  // uTensor specific variables
  Context ctx;
  Tensor* input_x;
  S_TENSOR pred_tensor;
  int pred_label=0;
  int n = 0;

  float temperature_value = 0.0;
  int* vibration_values;  
    
  float sensor_data[sensor_features] = {0.00, 0.00, 0.00, 0.00, 0.00, 0.00};

  while(true) {
    printf("\n\n ----------------------------------------------------------------------------\n");

    // If tesing, use sample data from header file 
    if (SIMULATED_DATA) {
      n = rand() % number_data_points;            
      printf("[%02d] Sensor data: ", n, actual_data_label[n]);
      for (unsigned int k=0; k < sensor_features; k++) {
        printf("%04.2f ", input_data_array[n][k]);
        //sensor_data[1][k] = input_data_array[n][k];
      }      
      input_x = new WrappedRamTensor<float>({1, sensor_features}, (float*) &input_data_array[n]);
    } else {
      // Read data from actual connected sensors
      n = 0;
      temperature_value = temperature.ReadData();
      vibration_values = vibration.ReadData();        

      // 13-bit, sign extended values.        
      printf(" Sensor data: Vx %04i, Vy %04i, Vz %04i, T %03.2f\r\n", (int16_t)vibration_values[0],
                  (int16_t)vibration_values[1], (int16_t)vibration_values[2], temperature_value);

      // Build the sensor_data array (6 features): 
      // Active.Current,	DC.Link.Voltage,	Temperature,	Vx, Vy, Vz
      sensor_data[0] = 0.0; // Sensors not available
      sensor_data[1] = 0.0; // Sensors not available
      sensor_data[2] = temperature_value;
      sensor_data[3] = vibration_values[0];
      sensor_data[4] = vibration_values[1];
      sensor_data[5] = vibration_values[2];
      input_x = new WrappedRamTensor<float>({1, sensor_features}, (float*) &sensor_data);
    }

    // Registered issue - so need to reinitiaize ctx every time it needs to be invoked
    Context ctx;
    get_DNN_model_ctx(ctx, input_x);

    // Pass the 'input' data tensor to the context
    pred_tensor = ctx.get("y_pred:0");              // Get a reference to the 'output' tensor
    ctx.eval();                                     // Trigger the inference engine
  
    pred_label = *(pred_tensor->read<int>(0, 0));   // Get the result back

    // Warning raised on basis of prediction results
    Warning_UI(pred_label, actual_data_label[n]);

    // Wait before acquiring next sensor data number_data_points
    wait (2.0);
  } // end while
} // end thread function

// ----------------------------------------------------------------------------------------------------
// Function: Initialise warning planel (LCD)
// ----------------------------------------------------------------------------------------------------
void Warning_UI(int pred_label, int actual_data_label) {  

    // Glow LEDs in response to warning or safe zone operations
    // If normal, glow only GREEN
    if (pred_label == NORMAL) {
          led_GREEN = 1;
          led_RED = 0; 
          BLINK_WARNING_LED = false;
    } 
    
    // If less than 30 minutes to failure, glow only RED
    if ((pred_label > FAULT) & (pred_label <= LT_30)) {
          led_GREEN = 0;
          led_RED = 1; 
          BLINK_WARNING_LED = false;
    }

    // Between 60 and 90 mins, warn, blink both
    if ((pred_label > LT_30) & (pred_label <= LT_90)) {
          led_GREEN = 1;
          led_RED = 1; 
          BLINK_WARNING_LED = true;
    }

    // If we are testing, print prediction errors
    if (SIMULATED_DATA) {

      int zone_error=0;
      // Check prediction accuracy with labeled test data
      // How far is the perdicted zone from actual?
      zone_error = abs(actual_data_label-pred_label);

      printf("\n >> Actual: %s | Predicted: %s | ", time_zone_labels[actual_data_label], time_zone_labels[pred_label]);

      switch (zone_error) {
          case 0:
            printf("[CORRECT prediction]");
            break;
          case 1:
            printf("[Fair prediction. Single-zone error]");
            break;  
          default:
            if (zone_error > 1) printf("[ !! ERROR !! of %d zones in prediction]", zone_error);
            if (actual_data_label == NORMAL && pred_label != NORMAL) printf("[FALSE ALARM despite normal operation!]");
            break;
      }
    } else {
        printf("\n >> Predicted time-to-failure: %s ", time_zone_labels[pred_label]);
    }

    // Display warnings on LCD
    lcd.Clear();
    lcd.DisplayString((uint8_t *)time_zone_labels[pred_label]);

  return;
}

// ----------------------------------------------------------------------------------------------------
// Function: Initialise warning planel (LCD)
// ----------------------------------------------------------------------------------------------------
void Initialise_Warning_Panel(void) {  
  
  printf("\n\n Initialising System UI...");
  printf("\n - PREDICTIVE MAINTENANCE - ");

  uint8_t title1[] = " - PREDICTIVE MAINTENANCE-Test";
  uint8_t title2[] = " - PREDICTIVE MAINTENANCE-Connected";

  lcd.Clear();
  if (SIMULATED_DATA){  
    printf("\n - Test Data - ");
    lcd.ScrollSentence(title1, 1, 175); // text, scroll number of times and the speed 
  } else {
    printf("\n - Real sensor data - ");
    lcd.ScrollSentence(title2, 1, 175); // text, scroll number of times and the speed 
  }
}

// ----------------------------------------------------------------------------------------------------
// Function: Warning LED thread: Blink RED if warning zones are > 30 but < 90
// ----------------------------------------------------------------------------------------------------
void ledWarning_thread_function(){
  
  while(true) {
    if (BLINK_WARNING_LED) {
      led_RED = !led_RED;
      wait(0.3);
    }
  }
}