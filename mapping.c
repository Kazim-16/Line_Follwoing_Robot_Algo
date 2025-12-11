//Heyyyyyyyyy
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/ledc.h>
#include "sdkconfig.h"
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_adc/adc_oneshot.h"
#include "hal/adc_types.h"



//Normal GPIO digital pins for motor direction control
#define R_F 5
#define R_B 23
#define L_F 4
#define L_B 27

//PWM pins for motor speed control
#define R_ENABLE 18
#define L_ENABLE 19


#define L_base_pwm 100
#define R_base_pwm 100


#define R_ENABLE_CHANNEL LEDC_CHANNEL_0
#define L_ENABLE_CHANNEL LEDC_CHANNEL_1
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES LEDC_TIMER_10_BIT //pwm resolution is 10 bits so the duty cycle ranges from 0 to 1023
#define LEDC_FREQUENCY 10000 //defines the pwm frequency 

//Sensor pins (ADC Channels)
//ADC 1 channels
#define E2LEFT ADC_CHANNEL_8   //weight = -3 //declare as ADC_CHANNEL_X //s_state[1]
#define E1LEFT ADC_CHANNEL_5 //weight = -2 //s_state[2]
#define LEFT  ADC_CHANNEL_4  //weight = -1 //s_state[3]
#define RIGHT  ADC_CHANNEL_7  //weight = +1 //s_state[5]
#define E1RIGHT ADC_CHANNEL_6 //weight = +2 //s_state[6]
#define E2RIGHT ADC_CHANNEL_3 //weight = +3 //s_state[7]

//ADC 2 channels
#define E3LEFT ADC_CHANNEL_9  //weight = -4 //s_state[0]
#define E3RIGHT ADC_CHANNEL_0   //weight = +4 //s_state[8]


int s_state[9];
float r_pwm, l_pwm;

// PID constants
float Kp = 110 ;   
float Ki = 1;  
float Kd = 0.1;   

//PID variables
float previous_error = 0;
float integral = 0;

typedef enum {
    NORTH = 0,
    EAST  = 1,
    SOUTH = 2,
    WEST  = 3
} Direction;

Direction current_dir = NORTH;   // starting direction


void motor_drive(int l_pwm,int r_pwm)
{
    if (l_pwm > 1023) l_pwm = 1023;
    if (r_pwm > 1023) r_pwm = 1023;
    if (l_pwm < 0) l_pwm = 0;
    if (r_pwm < 0) r_pwm = 0;

    ledc_set_duty(LEDC_MODE, R_ENABLE_CHANNEL, r_pwm); //sets the new duty cycle value
    ledc_update_duty(LEDC_MODE, R_ENABLE_CHANNEL); //applies the new duty cycle value to the hardware

    ledc_set_duty(LEDC_MODE, L_ENABLE_CHANNEL, l_pwm); //sets the new duty cycle value
    ledc_update_duty(LEDC_MODE, L_ENABLE_CHANNEL);
}

void motor_drive_f(int l_pwm, int r_pwm)
{
    gpio_set_level(L_F, 1);
    gpio_set_level(L_B, 0);

    gpio_set_level(R_F, 1);
    gpio_set_level(R_B, 0);

    motor_drive(l_pwm, r_pwm);

}
void motor_drive_r(int speed)
{
    gpio_set_level(L_F, 1);   
    gpio_set_level(L_B, 0);

    gpio_set_level(R_F, 0);   
    gpio_set_level(R_B, 1);

    motor_drive(speed, speed);

}

void motor_drive_l(int speed)
{

    gpio_set_level(L_F, 0);   
    gpio_set_level(L_B, 1);

    gpio_set_level(R_F, 1);   
    gpio_set_level(R_B, 0);

    motor_drive(speed, speed);

}

void motor_drive_u(int speed)
{
    gpio_set_level(L_F, 0);   
    gpio_set_level(L_B, 1);

    gpio_set_level(R_F, 1);  
    gpio_set_level(R_B, 0);

    motor_drive(speed, speed);

}
void small_forward_step()
{
    motor_drive_f(180, 180);   // smooth forward to check for 'T' and'+' junctions
    vTaskDelay(120);
}


int detect_junction()
{
    bool pure_left  = (s_state[0] > 2000 && s_state[1] > 2000);
    bool pure_right = (s_state[7] > 2000 && s_state[8] > 2000);
    bool center = (s_state[3] >2000 &&s_state[5] > 2000); // these 2 sensors more will be present in the line
    // ---- Dead end ----
    bool all_white = true;
    for(int i = 0; i < 9; i++)
    {
        if(s_state[i] > 800) all_white = false;
    }
    if (all_white)
        return 0;

    // ---- Pure left or Pure right ----
    if (pure_left || pure_right)
        return 1;

    if (!all_white)
{    small_forward_step();
    if (center)
            return 3;//+ junction
        else
            return 2;// T junction
}

    // ---- Normal line ----
    return -1;
}

void update_direction(int turn_type)
{
    // turn_type:
    // 0 = dead end (reverse turn)
    // 1 = left or right (detected by sensors)
    // -1 = straight (no change)

    if (turn_type == -1)
        return;  // no change in direction

    // Dead end → reverse (180 degree turn)
    if (turn_type == 0)
    {
        current_dir = (current_dir + 2) % 4;
        return;
    }

    // turn_type == 1 : we must check WHICH side caused it
    bool left_junc  = (s_state[0] > 2000 && s_state[1] > 2000);
    bool right_junc = (s_state[7] > 2000 && s_state[8] > 2000);

    if (left_junc)
    {
        // turn left → anticlockwise
        current_dir = (current_dir + 3) % 4;
    }
    else if (right_junc)
    {
        // turn right → clockwise
        current_dir = (current_dir + 1) % 4;
    }
}

void junction_response(int j)
{
    if (j == 0)     // Dead end
    {
        motor_drive_u(220);
        vTaskDelay(350);
    }
    else if (j == 1)    // Pure left OR pure right
    {
        // Priority-based decision
        if (s_state[0] > 2000 && s_state[1] > 2000)
            motor_drive_l(200);
        else
            motor_drive_r(200);

        vTaskDelay(150);  
    }
    else if (j==2)
    {
        special_response();
    }
    else if (j==3)
    {
        special_response();
    }
    else // j == -1 → normal line
    {
        motor_drive_f(l_pwm, r_pwm);
    }
}



float position()
{
  float s_state_sum = 0;
  float s_num = 0;
  float s_avg = 0;

  for (int i = 0; i < 9; i++) {
    s_state_sum += s_state[i];
  }

  for (int i = 0; i < 9; i++) {
    s_num += (i - 4) * s_state[i];
  }

  if (s_state_sum != 0)
  {
    s_avg = s_num / s_state_sum;
  } 
  else
  {
    s_avg = 0;
  }    
    return s_avg;
}

/* RTOS- Real time operating system
OS designed to process data as it comes in, typically within strict timing deadlines
Normal operating systems may delay tasks so tasks which require immediate attention or execution cant be handles via a normal OS
The higher priority tasks run first
*/
TaskHandle_t LineFollowerTaskHandle = NULL;
//Declares a FreeRTOS task variable. You later use it when creating the task; can be used to control or delete the task externally.

void LineFollowerTask(void *arg){
//LineFollowerTask is the function that will run as a free Rtos task

//Creating the ADC unit FOR ADC 1
//The handle is just a reference(id or pointer). Instead of using raw hardware, you create a driver object, and ESP-IDF returns the handle to it so you can talk to it later
//The "driver object" is basically a software structure(like a C struct) that stores all the internal configuration to control the hardware 
adc_oneshot_unit_handle_t handle1= NULL;
adc_oneshot_unit_init_cfg_t init_config1 =
{
    .unit_id= ADC_UNIT_1, //Configuring the ADC channel 1
    .ulp_mode= ADC_ULP_MODE_DISABLE, //turning off ULP interaction
};

ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &handle1));//For error checking dont have to worry too much about this(hopefully)
   //the adc_oneshot_new_unit leads to esp idf creating the adc unit object inside the memory, configuring it to init_config1 and stores the pointer to that object in handle variable
   

//Creating the ADC unit for ADC 2
adc_oneshot_unit_handle_t handle2= NULL;
adc_oneshot_unit_init_cfg_t init_config2 =
{
    .unit_id= ADC_UNIT_2, //Configuring the ADC channel 2
    .ulp_mode= ADC_ULP_MODE_DISABLE, //turning off ULP interaction
};

   ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config2, &handle2));//For error checking dont have to worry too much about this(hopefully)
   //the adc_oneshot_new_unit leads to esp idf creating the adc unit object inside the memory, configuring it to init_config1 and stores the pointer to that object in handle variable
   
    adc_oneshot_chan_cfg_t config = {
    .bitwidth = ADC_BITWIDTH_12, // or you can use ADC_BITWIDTH_DEFAULT here
    .atten = ADC_ATTEN_DB_12, //to map attenuation to 3.3 V, according to the sensor
    };
    
    //For channel 1 pins
    ESP_ERROR_CHECK(adc_oneshot_config_channel(handle2, E2LEFT, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(handle1, E1LEFT, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(handle1, LEFT, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(handle1, RIGHT, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(handle1, E1RIGHT, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(handle1, E2RIGHT, &config));
    //Configures the adc channels with the configuartion made

    //For channel 2 pins
    ESP_ERROR_CHECK(adc_oneshot_config_channel(handle2, E3LEFT, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(handle1, E3RIGHT, &config));


    while(1)
    {
        ESP_ERROR_CHECK(adc_oneshot_read(handle2, E3LEFT, &s_state[0]));
        ESP_ERROR_CHECK(adc_oneshot_read(handle2, E2LEFT, &s_state[1]));
        ESP_ERROR_CHECK(adc_oneshot_read(handle1, E1LEFT, &s_state[2]));
        ESP_ERROR_CHECK(adc_oneshot_read(handle1, LEFT, &s_state[3]));
        s_state[4]=0;
        ESP_ERROR_CHECK(adc_oneshot_read(handle1, RIGHT, &s_state[5]));
        ESP_ERROR_CHECK(adc_oneshot_read(handle1, E1RIGHT, &s_state[6]));
        ESP_ERROR_CHECK(adc_oneshot_read(handle1, E2RIGHT, &s_state[7]));
        ESP_ERROR_CHECK(adc_oneshot_read(handle1, E3RIGHT, &s_state[8]));

     
      float error = 0.65 - position();

      // PID calculations
      float P = error;
      //integral += error;
      float derivative = error - previous_error;
      float PIDvalue = (Kp * P) + (Kd * derivative);
      previous_error = error;



    if(PIDvalue>0)
     {
      r_pwm=R_base_pwm-PIDvalue;
      l_pwm=L_base_pwm+PIDvalue;
     }
       else 
     {
      r_pwm=R_base_pwm-PIDvalue;
      l_pwm=L_base_pwm+PIDvalue;
     }
     
    int j = detect_junction();

    update_direction(j);

    junction_response(j);

     for(int i=0;i<9;i++)
     {
      printf("Sensor readings:");
      printf("%d \t",s_state[i]);
     }
     printf("Error: %f",error);
     printf("Position: %f",position());
     printf("L_PWM is %f",l_pwm);
     printf("R_PWM is %f",r_pwm);
     

     vTaskDelay(10);

    }
    return;

}



void app_main(void)
{
    //Configuring the Motor direction pins
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << R_F) | (1ULL << R_B) | (1ULL << L_F) | (1ULL << L_B),  
           /*ESP- IDF configures pins using a 64 bit bitmask
          (1ULL << LED_PIN) shifts 1 left by LED_PIN bits
          So it basically says i want to configure GPIO2
          This is helpful because we can configure multiple pins at once*/
        .mode = GPIO_MODE_OUTPUT,              // pins set as output
        .pull_up_en = GPIO_PULLUP_DISABLE,     // Pull-ups disabled
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // Pull-down disabled
        .intr_type = GPIO_INTR_DISABLE         // Interrupts disables
    };
    gpio_config(&io_conf);

    //Configuring the motor speed pins(PWM)

    //Configuring the LEDC timer
    ledc_timer_config_t ledc_timer=
    {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz= LEDC_FREQUENCY,
    };
    ledc_timer_config(&ledc_timer); //Applies the timer settings to the hardware

    //Configuring the LEDC channel
    //Left motor 
    ledc_channel_config_t ledc_channel_left=
    {
        .gpio_num = L_ENABLE, //which GPIO pin will the PWM occur in
        .speed_mode = LEDC_MODE, //speed mode
        .channel = L_ENABLE_CHANNEL, //which channel outputs pwm
        .timer_sel = LEDC_TIMER, //which timer it uses
        .duty = 0 
    };
    ledc_channel_config(&ledc_channel_left); //Applies the channel settings to the hardware

    //Right Motor
    ledc_channel_config_t ledc_channel_right=
    {
        .gpio_num = R_ENABLE, //which GPIO pin will the PWM occur in
        .speed_mode = LEDC_MODE, //speed mode
        .channel = R_ENABLE_CHANNEL, //which channel outputs pwm
        .timer_sel = LEDC_TIMER, //which timer it uses
        .duty = 0 
    };
    ledc_channel_config(&ledc_channel_right); //Applies the channel settings to the hardware

    //ADC channels
    xTaskCreatePinnedToCore(LineFollowerTask, "Line Follower Task", 4096, NULL, 10, &LineFollowerTaskHandle,0);
}
