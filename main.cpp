#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "pico/multicore.h"
#include "pico/util/queue.h"
#include "hardware/uart.h"

#include <cmath>
////////////////////////////////////////////////////////PIN DEFINITIONS//////////////////////////////////////////////////
//current sense pins
#define _CURRENT_SENSE_PIN_A 26
#define _CURRENT_SENSE_PIN_B 27
#define _CURRENT_SENSE_PIN_C 28
#define _CURRENT_SENSE_CHANNEL_A 0
#define _CURRENT_SENSE_CHANNEL_B 1
#define _CURRENT_SENSE_CHANNEL_C 2
//motor two phase resistance:16.6

// SPI Defines
#define _PIN_MISO 16
#define _PIN_CS   17  //for encoder
#define _PIN_SCK  18
#define _PIN_MOSI 19

//driver pins
#define _DRIVER_ENABLE_PIN 12
#define _PWM_A_PIN 13
#define _PWM_B_PIN 14
#define _PWM_C_PIN 15

//stepper pins
#define _DIR_PINA 0
#define _STEP_PINA 1
#define _DIR_PINB 2
#define _STEP_PINB 3

//limit switch pins
#define _LIMIT_SWITCH_RIGHT 4
#define _LIMIT_SWITCH_LEFT 5

//UART defines
#define _UART_ID uart1
#define _BAUD_RATE 115200
#define _UART_TX_PIN 8
#define _UART_RX_PIN 9

/////////////////////////////////////////////////////////////////Useful constants//////////////////
#define _2PI        6.2831853072
#define _PIover3    1.0471975512
#define _1overSQRT3 0.5773502691
#define _twothirds  0.666666666
#define _sqrt3over2 0.86602540378

////////////////////////////////////////////////////////////////////MONITORING SEGMENT//////////////////////////////

uint32_t monitoring_mask=0; // not monitoring anything by default
// mask for monitoring variables
#define BIT(n) (1 << (n))  //macro pentru selectat biti
#define MASK_CURRENTS_A  BIT(0) // 1 << 0
#define MASK_CURRENTS_B  BIT(1) // 1 << 1
#define MASK_CURRENTS_C  BIT(2) // 1 << 2
#define MASK_DELTA_TIME  BIT(3)
#define MASK_ANGLE_T     BIT(4)
#define MASK_ANGLE_M     BIT(5) 
#define MASK_VELOCITY_T  BIT(6)
#define MASK_VELOCITY_M  BIT(7) 
#define MASK_IQ_T        BIT(8) 
#define MASK_IQ_M        BIT(9) 
#define MASK_UQ          BIT(10) 
#define MASK_EL_ANGLE    BIT(11) 
//structure used for monitoring variables inside second core
volatile struct monitoring_data{
    float currents_abc[3]; // a,b,c
    float delta_time;
    float angle[2]; //target, meas
    float velocity[2]; //target, meas
    float iq[2]; // target, meas
    float uq;
    float el_angle;
    //pid values

} shared_monitoring_data;
const volatile float* values[] = {  //array used to iterate through the possible variables
    &shared_monitoring_data.currents_abc[0], &shared_monitoring_data.currents_abc[1], &shared_monitoring_data.currents_abc[2], //currents
    &shared_monitoring_data.delta_time, 
    &shared_monitoring_data.angle[0], &shared_monitoring_data.angle[1], // angle
    &shared_monitoring_data.velocity[0], &shared_monitoring_data.velocity[1], //velocity
    &shared_monitoring_data.iq[0], &shared_monitoring_data.iq[1], //iq
    &shared_monitoring_data.uq,
    &shared_monitoring_data.el_angle 
};
void handle_monitoring(uint32_t mask){
    for(int i=0;i<12;i++){
        if(mask&(1<<i)){
            printf("%f ",*values[i]);
        }
    }
    printf("\n");
}

//////////////////////////////////////////////////////////UART COMMS SEGENT//////////////////////////////

//this initializes the UART comms
void initialize_uart(){
    uart_init(_UART_ID, _BAUD_RATE);
    gpio_set_function(_UART_TX_PIN, UART_FUNCSEL_NUM(_UART_ID, _UART_TX_PIN));
    gpio_set_function(_UART_RX_PIN, UART_FUNCSEL_NUM(_UART_ID, _UART_RX_PIN));
    uart_set_format(_UART_ID, 8, 1, UART_PARITY_NONE);
}
//reads a string from uart into buffer
void uart_read_string(char *buffer, size_t buffer_size, uart_inst_t *uart_instance){
    size_t index=0;
    while(index < buffer_size-1){ //leave space for null terminator
        char c=uart_getc(uart_instance);
        if(c=='\n'){
            break;
        }
        buffer[index++]=c;
    }
    buffer[index++]='\0';
}
queue_t comm_queue_01; //queue for communication from core 0 to core 1
queue_t comm_queue_10; //queue for communication from core 1 to core 0
struct command_packet{
    uint command; // 0-control mode; 1-new target; 2-change regulator variable
    float arguments[5];
};
// checks uart for new commands
void uart_check_command(){
    if(uart_is_readable(_UART_ID)){
        char uart_message[32];
        uart_read_string(uart_message,32,_UART_ID); //read uart into buffer until newline

        /// received message can contain multiple commands (begining with a letter)
        uint index=0;
        command_packet cmd_packet;
        while(1){
            //change the multiple if elses to switch (check if switch works with case 'char':)
            if(uart_message[index]=='M'){ //control mode (voltage,current, velocity, angle)
                cmd_packet.command = 0;
                index++;
                if(uart_message[index]>='0' && uart_message[index]<='9'){ //check if next char is a number
                    cmd_packet.arguments[0] = uart_message[index]-'0'; //conver char to int
                    index++;
                    // printf("NEW MODE: %f\n",cmd_packet.arguments[0]);
                    queue_add_blocking(&comm_queue_01,&cmd_packet);
                }
                // else
                //     printf("Error: Mode has to be a number\n");
            }
            else if(uart_message[index]=='T'){ // new target (float)
                cmd_packet.command = 1;
                index++;
                char argument[30];
                uint arg_index=0;
                // char valid_char[20]="0123456789.-";  //maybe this is cleaner and use with strchr   ALSO maybe make this a function because converting a segment of string to float is done frequently
                while((uart_message[index]>='0' && uart_message[index]<='9') || uart_message[index]=='.' || uart_message[index]=='-'){
                    argument[arg_index]=uart_message[index];
                    index++;
                    arg_index++;                    
                } 
                cmd_packet.arguments[0] = strtof(argument, NULL);
                // printf("NEW TARGET: %f\n",cmd_packet.arguments[0]);
                queue_add_blocking(&comm_queue_01,&cmd_packet);
            }
            else if(uart_message[index]=='R'){ // change regulator variable
                cmd_packet.command=2;
                index++;
                
                //decode regulator
                if(uart_message[index]=='C'){ //current regulator
                    cmd_packet.arguments[0]=1; //stands for mode 1 (current)
                    index++;
                }
                else if(uart_message[index]=='V'){ //velocity regulator
                    cmd_packet.arguments[0]=2; //stands for mode 2 (velocity)
                    index++;
                }
                else if(uart_message[index]=='A'){ //angle regulator
                    cmd_packet.arguments[0]=3; //stands for mode 3 (angle)
                    index++;
                }
                else{
                    // printf("Error: Regulator not found\n");
                    break;
                }

                //decode term
                if(uart_message[index]=='P'){ //select kp
                    cmd_packet.arguments[1]=0;
                    index++;
                }
                else if(uart_message[index]=='I'){ //select ki
                    cmd_packet.arguments[1]=1;
                    index++;
                }
                else{
                    // printf("Regulators only have kp and ki\n");
                    break;
                }

                //decode value value
                char value[30];
                uint value_index=0;
                while((uart_message[index]>='0' && uart_message[index]<='9') || uart_message[index]=='.' || uart_message[index]=='-'){
                    value[value_index]=uart_message[index];
                    index++;
                    value_index++;                    
                } 
                cmd_packet.arguments[2] = strtof(value, NULL);
                
                //send packet
                //printf("REGULATOR: Changed %f regulator's %f term to: %f\n",cmd_packet.arguments[0],cmd_packet.arguments[1],cmd_packet.arguments[2]);
                queue_add_blocking(&comm_queue_01,&cmd_packet);
            }
            else if(uart_message[index]=='P'){ //monitoring command (print) takes a binary 32bit number
                index++;
                monitoring_mask=0;
                while(uart_message[index]=='1' || uart_message[index]=='0'){
                    monitoring_mask=(monitoring_mask<<1)|(uart_message[index++]-'0');
                }
            }
            else if(uart_message[index]==' '){ //if whitespace, skip
                index++;
            }
            else if(uart_message[index]=='\0'){  //if null terminator, commands have ended
                break;
            }
            else{
                //printf("Error: Command not found: %c\n",uart_message[index]); //if command doesnt exist, cancel everything that comes after
                break;
            }
        }
    }
}

//////////////////////////////////////////////////////////////TRIGONOMETRY FUNCTIONS////////////////////////////

//this wraps an angle to the 0 to 2PI range
float wrap_rad(float angle_rad){
    angle_rad=fmod(angle_rad,_2PI);
    if(angle_rad<0)
        angle_rad+=_2PI;
    return angle_rad;
}

//sin and cos lookup table functions

#define LOOKUP_RESOLUTION 1024
// only for 0 to pi/2
float sin_table[LOOKUP_RESOLUTION];
float lookup_factor=(float)LOOKUP_RESOLUTION/M_PI_2;
//constructs sin lookup table
void construct_sin_table(){
    for(int i=0;i<LOOKUP_RESOLUTION;i++){
        sin_table[i]=sinf(i*M_PI_2/(float)LOOKUP_RESOLUTION);
    }
}
// aproximates sin with a lookup table
float sin_aprox(float angle_rad){
    //BIGGEST ERROR ON WHOLE CYCLE  -0.001531
    if (angle_rad <= M_PI_2) {
        // First quadrant: sin(x)
        return sin_table[(int)(angle_rad * lookup_factor)];
    } else if (angle_rad <= M_PI) {
        // Second quadrant: sin(PI - x)
        return sin_table[(int)((M_PI - angle_rad) * lookup_factor)];
    } else if (angle_rad <= 3 * M_PI_2) {
        // Third quadrant: -sin(x - PI)
        return -sin_table[(int)((angle_rad - M_PI) * lookup_factor)];
    } else {
        // Fourth quadrant: -sin(2PI - x)
        return -sin_table[(int)((_2PI - angle_rad) * lookup_factor)];
    }
}
// wraps angle and aprox. sin with a lookup table
float sin_aprox_wrap(float angle_rad){
    angle_rad=wrap_rad(angle_rad);
    return sin_aprox(angle_rad);
}
//aproximates cos with a lookup table
float cos_aprox(float angle_rad){
    //made this work
    return sin_aprox(wrap_rad(angle_rad+M_PI_2));
}


/////////////////////////////////////////////////////////////////////////CLASS DEFINITIONS///////////////////////////////////////////////

//current class. Contains a,b,c,d,q and the associated methods
// how do i make this efficient. Maybe an update function? That could open opportunities for errors
class motor_current{
    public:
        float a;
        float b;
        float c;
        float alpha;
        float beta;
        float d;
        float q;
        // clarke transform
        void update_ab_values(){
            //formulas from https://www.ti.com/lit/an/bpra048/bpra048.pdf   ; tested using https://www.mathworks.com/help/mcb/ref/clarketransform.html
            //simplefoc does something about sign too
            alpha=_twothirds*(a-0.5*(b+c));
            beta=_twothirds*(_sqrt3over2*(b-c));
        }
        // park transform
        void update_dq_values(float el_angle){
            update_ab_values();
            float cos_theta=cos_aprox(el_angle);
            float sin_theta=sin_aprox(el_angle);
            d=alpha*cos_theta+beta*sin_theta;
            q=beta*cos_theta-alpha*sin_theta;
        }
};

//class for the current sensor
class current_sensors{
    public:
        current_sensors(int cs_phase_a_pin, int cs_phase_b_pin, int cs_phase_c_pin){
            pinA=cs_phase_a_pin;
            pinB=cs_phase_b_pin;
            pinC=cs_phase_c_pin;

            adc_init();
            adc_gpio_init(pinA);
            adc_gpio_init(pinB);
            adc_gpio_init(pinC);

            calculate_offset_voltage();
        }
        motor_current get_motor_current(){
            motor_current current;
            current.a=read_raw_voltage(_CURRENT_SENSE_CHANNEL_A)-center_offset_voltage_a;
            current.b=read_raw_voltage(_CURRENT_SENSE_CHANNEL_B)-center_offset_voltage_b;
            current.c=read_raw_voltage(_CURRENT_SENSE_CHANNEL_C)-center_offset_voltage_c;
            current.a/=gain;
            current.b/=gain;
            current.c/=gain;
            return current;
        }
        
    private:
        uint pinA;
        uint pinB;
        uint pinC;
        const float VCC_ADC_PICO=3.3;
        const float BIT_STEP=4096.0;
        const float gain=0.165; //test this without current limiting by changing voltage
        float center_offset_voltage_a=0;
        float center_offset_voltage_b=0;
        float center_offset_voltage_c=0;

        float read_raw_voltage(int channel){
            adc_select_input(channel);
            float voltage=adc_read();
            return (voltage/BIT_STEP)*VCC_ADC_PICO;
        }
        void calculate_offset_voltage(){
            sleep_ms(500);
            center_offset_voltage_a=0;
            center_offset_voltage_b=0;
            center_offset_voltage_c=0;
            for(int i=0;i<10000;i++){
                center_offset_voltage_a+=read_raw_voltage(_CURRENT_SENSE_CHANNEL_A);
                center_offset_voltage_b+=read_raw_voltage(_CURRENT_SENSE_CHANNEL_B);
                center_offset_voltage_c+=read_raw_voltage(_CURRENT_SENSE_CHANNEL_C);
            }
            center_offset_voltage_a/=10000.0;
            center_offset_voltage_b/=10000.0;
            center_offset_voltage_c/=10000.0;
        }
};

// LP filter (exponential moving average)
class LPF{
    public:
        LPF(float alpha){
            this->alpha=alpha;
            prev_output=0;
        }
        float compute(float input){
            float output=alpha*input+(1.0f-alpha)*prev_output;
            prev_output=output;
            return output;
        }
    private:
        float alpha; //weight
        float prev_output;
};

// spi encoder class
class encoder{
    public:
        // class constructor
        encoder(spi_inst_t *spi_channel,uint sck_pin,uint cs_pin, uint miso_pin, uint mosi_pin, bool reverse=false):vel_filter(0.01){
            spi_init(spi_channel,1000*1000); // spi @ 1MHZ
            spi_set_format(spi_channel,16,SPI_CPOL_0,SPI_CPHA_1,SPI_MSB_FIRST); //mode 1 spi, 16 bit
            gpio_set_function(miso_pin, GPIO_FUNC_SPI);
            gpio_set_function(sck_pin,  GPIO_FUNC_SPI);
            gpio_set_function(mosi_pin, GPIO_FUNC_SPI);

            this->cs_pin=cs_pin;
            this->spi_channel=spi_channel;
            gpio_init(cs_pin);
            gpio_set_dir(cs_pin, GPIO_OUT);
            gpio_put(cs_pin, 1);
            this->reverse=reverse;
            zero_sensor();
            old_angle=get_angle_rad();
        }
        // returns angle in int form
        uint16_t get_angle(){ //first read sends the read command and the second has the data
            uint16_t angle_int;
            gpio_put(cs_pin,0);
            spi_write16_read16_blocking(spi_channel,&ANGLE_READ_COMMAND,&angle_int,1);
            gpio_put(cs_pin,1);
            sleep_us(1);//delay for transmission
            gpio_put(cs_pin,0);
            spi_write16_read16_blocking(spi_channel,&ANGLE_READ_COMMAND,&angle_int,1);
            gpio_put(cs_pin,1);
            angle_int=angle_int & 0b0011111111111111;
            if(reverse)     //this is needed in case the encoder direction is not the same as the phase succession
                angle_int=16383-angle_int;
            
            int32_t delta_angle=angle_int-previous_angle_int; //phase unwrapping!
            if(delta_angle>8192){
                //wrapped negatively
                delta_angle-=16384;
            }
            else if(delta_angle<-8192){
                //wrapped positively
                delta_angle+=16384;
            }
            total_rotations+=delta_angle;
            previous_angle_int=angle_int;
            return angle_int;
        }
        //makes the current angle be the 0 absolute angle
        void zero_sensor(){// see if this still works
            previous_angle_int=get_angle();
            total_rotations=0;
        }
        // returns angle in DEG
        float get_angle_deg(){
            return (float)get_angle()/16384.0*360.0;   //16384 is the number of pulses per rotation (cpr)
        }
        //returns angle in RAD
        float get_angle_rad(){
            return (float)get_angle()/16384.0*_2PI;
        }
        //returns the FILTERED VELOCITY in rad/s
        float get_velocity(){  ///velocity timer might overflow. test/solve
            uint64_t current_time=time_us_64();
            float delta_time=current_time-old_time; //us
            delta_time/=1000000.0; //s

            float current_angle=get_angle_rad(); //rad
            float delta_angle=current_angle-old_angle;

            //angle weapping
            if (delta_angle > M_PI) { // Wrapped from 0 to 2PI
                delta_angle -= _2PI;
            } else if (delta_angle < -M_PI) { // Wrapped from 2PI to 0
                delta_angle += _2PI;
            }

            old_angle=current_angle;
            old_time=current_time;

            float raw_velocity=delta_angle/delta_time; //rad/s
            return vel_filter.compute(raw_velocity);
        }
        
        //returns unwrapped angle as an int
        int32_t get_unwrapped_angle(){
            get_angle(); //we just update the function
            return total_rotations;
        }
        //returns unwrapped angle as degrees
        float get_unwrapped_angle_deg(){
            return (float)get_unwrapped_angle()/16384.0*360.0;
        }
        //returns unwrapped angle as radians
        float get_unwrapped_angle_rad(){
            return (float)get_unwrapped_angle()/16384.0*_2PI;
        }
        
        private:
            uint cs_pin;
            spi_inst_t* spi_channel;
            const uint16_t ANGLE_READ_COMMAND=0xFFFF;
            
            //variables to track absoulte angle
            uint16_t previous_angle_int;
            int32_t total_rotations;
            bool reverse;

            //variables for velocity funcion
            LPF vel_filter;
            float old_angle=0;
            uint64_t old_time=0;
};

// class for the 1/2 bridge drv8318 driver
class bridge_driver{
    public:
        bridge_driver(uint pin_a, uint pin_b, uint pin_c, uint pin_enable, uint pwm_freq=50000){ //default 50kHz frequency
            pin_A = pin_a;
            pin_B = pin_b;
            pin_C = pin_c;
            pin_EN = pin_enable;
            this->pwm_freq=pwm_freq;

            gpio_set_function(pin_A, GPIO_FUNC_PWM);
            gpio_set_function(pin_B, GPIO_FUNC_PWM);
            gpio_set_function(pin_C, GPIO_FUNC_PWM);
            slice_A=pwm_gpio_to_slice_num(pin_A);
            slice_B=pwm_gpio_to_slice_num(pin_B);
            slice_C=pwm_gpio_to_slice_num(pin_C);
            pwm_config conf=pwm_get_default_config();
            pwm_wrap=calc_wrap(pwm_freq);
            pwm_config_set_phase_correct(&conf,true); // the pwm uses a numerical comparator, but this is similar to making our carrier wave a triangle wave instead of a sawtooth, but it also halves our calculated frequency, so it has to be adjusted (halve wrap)
            pwm_config_set_wrap(&conf, pwm_wrap);
            pwm_config_set_clkdiv(&conf,1.0);
            pwm_init(slice_A, &conf, false);
            pwm_init(slice_B, &conf, false);
            pwm_init(slice_C, &conf, false);
            pwm_set_counter(slice_A,0);
            pwm_set_counter(slice_B,0);
            pwm_set_counter(slice_C,0);
            pwm_set_mask_enabled((1<<slice_A) | (1<<slice_B) | (1<<slice_C)); // we do this to sync all pwm pins because they are on different slices(we need 3 pins for the half bridge and a slice only serves two pins). If we start the pwm with sequential enables, the signals wont be in phase.

            gpio_init(pin_EN);
            gpio_set_dir(pin_EN,GPIO_OUT);

            set_pwm_duty(0,0,0);
            disable();  //default disabled
        }
        //enable the driver idk if it needs delays
        void enable(){
            gpio_put(pin_EN,1);
        }
        //disable the driver
        void disable(){
            gpio_put(pin_EN,0);
        }
        void set_pwm_duty(float dutyA,float dutyB,float dutyC){
            pwm_set_gpio_level(pin_A,dutyA*pwm_wrap);
            pwm_set_gpio_level(pin_B,dutyB*pwm_wrap);
            pwm_set_gpio_level(pin_C,dutyC*pwm_wrap);
        }
    private:
        uint pin_A;
        uint pin_B;
        uint pin_C;
        uint pin_EN;
        uint slice_A;
        uint slice_B;
        uint slice_C;
        uint16_t pwm_wrap;
        uint pwm_freq;
        //function that returns wrap based on frequency; divisor is assumed to be 1; the value has to be possible(max wrap is 65535)
        uint calc_wrap(int freq){
            // 125000000=125MHz = clock freq
            return (125000000/freq/2); //we halve wrap because we are counting up and down instead of wrapping to 0
        }
};  

//PID Controller
class PIController{
    public:
        PIController(float kp,float ki,float max_output,float ramp){
            this->kp=kp;
            this->ki=ki;
            this->integral_comp=0;
            this->prev_error=0;
            this->max_output=max_output;
            this->ramp=ramp; //max rate of change
            this->previous_output=0;
            this->previous_integral_comp=0;
            previous_time=time_us_64();
        }
        float compute(float error){
            uint64_t current_time=time_us_64();
            float delta_time=(current_time-previous_time)/1000000.0; //i could get rid of this by using the time from the foc loop
            previous_time=current_time;
            
            float proportional_comp=kp*error;

            integral_comp+=ki*delta_time*0.5*(error+prev_error);  //magic from simplefoc  ;; Trapezoidal integration?
            prev_error=error;
            
            //antiwindup
            if(integral_comp<-max_output/2)
                integral_comp=-max_output/2;
            else if(integral_comp>max_output/2)
                integral_comp=max_output/2;
            
            //calculate output
            float output=proportional_comp + integral_comp;
            
            //ramp
            float max_delta=ramp*delta_time;
            if(output-previous_output>max_delta){
                output=previous_output+max_delta;
                integral_comp=previous_integral_comp;
            }
            else if(previous_output-output>max_delta){
                output=previous_output-max_delta;
                integral_comp=previous_integral_comp;
            }
            previous_integral_comp=integral_comp;

            //limit output value
            if(output<-max_output)
                output=-max_output;
            else if(output>max_output)
                output=max_output;
            previous_output=output;

            return output;
        }
        float kp;
        float ki;
        float integral_comp;
        float previous_integral_comp;
        float prev_error;
        uint64_t previous_time;
        float max_output;
        float ramp;
        float previous_output;
};

//class for foc algorithm
class foc_controller{
    public:
        foc_controller(bridge_driver* associated_driver, encoder* associated_encoder, current_sensors* associated_current_sensors, uint motor_pole_pairs, uint power_supply_voltage, float phase_resistance):current_controller(50,1200,14,9999),iq_filter(0.01),vel_controller(-0.05,-0.3,1.4,90),angle_controller(8,1,45,5){ //old angle ki and kp: 50,550,25,9999
            this->asoc_driver=associated_driver;
            this->asoc_encoder=associated_encoder;
            this->asoc_cs=associated_current_sensors;

            this->motor_pole_pairs=motor_pole_pairs;
            this->current_limit=current_limit;
            this->power_supply_voltage=power_supply_voltage;
            max_uq_reference=power_supply_voltage/M_SQRT3;
            
            //current limiting; not done
            this->motor_phase_resistance=phase_resistance;
            
            //default reference values
            uq=0;
            iq_target=0;
            velocity_target=0;
            angle_target=0;
            //default manual reference values
            mode=3; //default mode is angle control
            manual_uq=0;
            manual_iq_target=0;
            manual_velocity_target=0;
            manual_angle_target=0;

            el_angle_offset=0;
            old_update_time=0;
            align();
        }
        //find offset for electrical angle
        void align(){
            float tests=30;
            float sum_sin=0;
            float sum_cos=0;
            asoc_driver->enable();
            setSVPWM(6.9,M_PI); //move motor to 180 deg (pi rad)
            sleep_ms(1000);
            for(int i=0;i<tests;i++){
                float el_ang=get_electrical_angle();
                sum_sin+=sin(el_ang);
                sum_cos+=cos(el_ang);
                sleep_ms(10);
            }
            
            float el_angle_offset_reconstructed=atan2(sum_sin/tests,sum_cos/tests);
            el_angle_offset=wrap_rad(el_angle_offset_reconstructed);
            asoc_encoder->zero_sensor(); //cant remember where this went
            old_angle_target=asoc_encoder->get_unwrapped_angle_rad();
            sleep_ms(1000);
            asoc_driver->disable();
            setSVPWM(0,0);
        }
        //foc loop
        void loop(){
            //calculate dt
            uint64_t current_time=time_us_64();
            float delta_time=(current_time-old_update_time)/1000000.0;
            old_update_time=current_time;

            //angle controller (this jumps for angle_target=0; also it only sometimes jumps)
            if(mode==3)
                angle_target=manual_angle_target;
            float angle_meas=asoc_encoder->get_unwrapped_angle_rad();
            old_angle_target=angle_target;
            float angle_error=old_angle_target-angle_meas;
            velocity_target=angle_controller.compute(angle_error);

            //velocity controller conf ~200us exec (this doesnt jump for velocity target=0)
            if(mode==2)
                velocity_target=manual_velocity_target;
            float velocity_meas=asoc_encoder->get_velocity(); 
            float vel_error=velocity_target-velocity_meas;
            iq_target=vel_controller.compute(vel_error);

            //current controller
            if(mode==1)
                iq_target=manual_iq_target;
            float electrical_angle=get_electrical_angle();
            motor_current meas_current=asoc_cs->get_motor_current(); 
            meas_current.update_dq_values(electrical_angle);
            meas_current.q=iq_filter.compute(meas_current.q); //low pass filter for iq current
            float current_error=iq_target-meas_current.q;
            uq=current_controller.compute(current_error);
            
            //voltage controller
            if(mode==0)
                uq=manual_uq;
            //avoid over-modulation
            if(uq>max_uq_reference && uq>0){
                uq=max_uq_reference;
            }
            else if(uq<-max_uq_reference && uq<0){
                uq=-max_uq_reference;
            }
            
            //send pwm to motor
            setSVPWM(uq,electrical_angle+M_PI_2);

            //update monitored values
            uint64_t st_time=time_us_64();
            shared_monitoring_data.currents_abc[0]=meas_current.a;
            shared_monitoring_data.currents_abc[1]=meas_current.b;
            shared_monitoring_data.currents_abc[2]=meas_current.c;
            shared_monitoring_data.delta_time=delta_time;
            shared_monitoring_data.angle[0]=angle_target;
            shared_monitoring_data.angle[1]=angle_meas;
            shared_monitoring_data.velocity[0]=velocity_target;
            shared_monitoring_data.velocity[1]=velocity_meas;
            shared_monitoring_data.iq[0]=iq_target;
            shared_monitoring_data.iq[1]=meas_current.q;
            shared_monitoring_data.uq=uq;
            shared_monitoring_data.el_angle=electrical_angle;
        }
        //pid target variables
        float uq;
        float iq_target;
        float velocity_target;
        float angle_target;

        float old_angle_target; //for angle rate_limit
        float old_update_time; //for calculating dt;

        ///// FUNCTIONS FOR MODE SELECTION AND TARGETS
        int mode;
        float manual_uq;
        float manual_iq_target;
        float manual_velocity_target;
        float manual_angle_target;
        //sets the controller mode; 3=angle, 2=velocity, 1=torque, 0=voltage reference
        void set_mode(int mode){
            if(mode >=0 && mode <=3)
                //this should also set a default target for the mode
                this->mode=mode;
        }
        //sets the angle target: use in angle control mode
        void set_angle(float angle){
            if(mode==3)
                manual_angle_target=angle;
        }
        //sets the velocity target; use in velocity control mode
        void set_velocity(float velocity){
            if(mode==2)
                manual_velocity_target=velocity;
        }
        //sets the torque (iq) target; use in torque control mode
        void set_torque(float iq){
            if(mode==1)
                manual_iq_target=iq;
        }
        //sets the uq reference directly; use in voltage reference mode
        void set_uq(float uq){
            if(mode ==0)
                manual_uq=uq;
        }
        //sets a target based on the current mode
        void set_target(float target){  //helper function for setting a target based on the mode; i use this when getting commands from core 0
            switch (mode){
                case 0:
                    set_uq(target);
                    break;
                case 1:
                    set_torque(target);
                    break;
                case 2:
                    set_velocity(target);
                    break;
                case 3:
                    set_angle(target);
                    break;
            }
        }
        // sets the pwm from a voltage reference
        void setSVPWM(float Vref, float el_angle){ //implemented according to https://www.e3s-conferences.org/articles/e3sconf/pdf/2021/64/e3sconf_suse2021_01059.pdf, but modified the m and the formulas for correct scaling
            el_angle=wrap_rad(el_angle); //el_angle has to be clamped between 0 and 2pi
            int sector = int(el_angle/_PIover3)+1;
            //calculate times
            float m=M_SQRT3*(fabs(Vref)/(float)power_supply_voltage); //modulation index
            float T1=m*sin_aprox(sector*_PIover3-el_angle);
            float T2=m*sin_aprox(el_angle-(sector-1)*_PIover3);
            float T0=1.0-T1-T2;
            // translate duty cycles to sectors
            float dA,dB,dC; //duty cycles for each phase;
            switch(sector){
                case 1:
                    //sector 1
                    dA=T1+T2+T0/2;
                    dB=T2+T0/2;
                    dC=T0/2;
                    break;
                case 2:
                    //sector 2
                    dA=T1+T0/2;
                    dB=T1+T2+T0/2;
                    dC=T0/2;
                    break;
                case 3:
                    //sector 3
                    dA=T0/2;
                    dB=T1+T2+T0/2;
                    dC=T2+T0/2;
                    break;
                case 4:
                    //sector 4
                    dA=T0/2;
                    dB=T1+T0/2;
                    dC=T1+T2+T0/2;
                    break;
                case 5:
                    //sector 5
                    dA=T2+T0/2;
                    dB=T0/2;
                    dC=T1+T2+T0/2;
                    break;
                case 6:
                    //sector 6
                    dA=T1+T2+T0/2;
                    dB=T0/2;
                    dC=T1+T0/2;
                    break;
            }
            if(Vref<0){ //reverse duty cycle if ref is negative
                dA=1.0-dA;
                dB=1.0-dB;
                dC=1.0-dC;
            }
            asoc_driver->set_pwm_duty(dA,dB,dC);
        }
        enum direction{
            CW=0,
            CCW=1
        };
        bridge_driver* asoc_driver;
        encoder* asoc_encoder;
        current_sensors* asoc_cs;
    // private:
        uint power_supply_voltage;
        float motor_phase_resistance;
        float current_limit;

        uint motor_pole_pairs;
        float el_angle_offset;
        float get_electrical_angle(){
            return wrap_rad(motor_pole_pairs*asoc_encoder->get_angle_rad()-el_angle_offset);
        }
        float max_uq_reference;

        //velocity PID
        PIController current_controller;
        PIController vel_controller;
        PIController angle_controller;
        LPF iq_filter;
};

// limit switch stuff
#define DEBOUNCE_DELAY_MS 500
volatile bool g_limit_switch_right_triggered=false;
volatile bool g_limit_switch_left_triggered=false;
volatile uint32_t last_debounce_time_right = 0;
volatile uint32_t last_debounce_time_left = 0;
void limit_switch_callback(uint gpio, uint32_t events){
    uint32_t current_time = time_us_32();
    if(gpio==_LIMIT_SWITCH_RIGHT){
        if (current_time - last_debounce_time_right > DEBOUNCE_DELAY_MS) {
            // printf("right        ");
            g_limit_switch_right_triggered = true;
            last_debounce_time_right = current_time;
        }
    }
    else if(gpio==_LIMIT_SWITCH_LEFT){
        if (current_time - last_debounce_time_left > DEBOUNCE_DELAY_MS) {
            // printf("left");
            g_limit_switch_left_triggered = true;
            last_debounce_time_left = current_time;
        }
    }
    // printf("    %d\n",time_us_32());
}
void add_limit_switch(uint pin){
    gpio_init(pin);
    gpio_set_dir(pin,GPIO_IN);
    gpio_pull_up(pin);
    gpio_set_irq_enabled_with_callback(pin, GPIO_IRQ_EDGE_FALL, true, limit_switch_callback);
}

// class for A4988 stepper driver
class stepper_driver{
    public:
        uint absolute_position_steps;
        bool moving;
        stepper_driver(uint step_pin, uint dir_pin,volatile bool* asoc_limit_switch,float hw_angle_per_step=1.8,uint microstepping_mult=1,bool invert_dir=false){
            this->step_pin=step_pin;
            this->dir_pin=dir_pin;
            this->hw_angle_per_step=hw_angle_per_step;
            this->microstepping_mult=microstepping_mult;
            angle_per_step=hw_angle_per_step/microstepping_mult;
            this->invert_dir=invert_dir;
            this->moving=false;
            this->asoc_limit_switch=asoc_limit_switch;

            gpio_init(step_pin);
            gpio_set_dir(step_pin,GPIO_OUT);
            gpio_put(step_pin,0);
            gpio_init(dir_pin);
            gpio_set_dir(dir_pin,GPIO_OUT);
            gpio_put(dir_pin,0);

            c.init(microstepping_mult);
            //this accounts for microstepping
            steps_per_rot=360/angle_per_step;
        }
        enum direction{
            CW=0,
            CCW=1
        };
        direction current_dir;
        // function to set direction
        void set_dir(direction dir){
            gpio_put(dir_pin,dir^invert_dir);
            current_dir=dir;
            sleep_us(5);
        }
        //function that takes ones step
        void step(){
            gpio_put(step_pin,1);
            sleep_us(10);
            gpio_put(step_pin,0);
            sleep_us(10);
        }
        // function to move stepper by steps in a direction with acceleration and deceleration
        void move(uint nr_steps,direction dir){
            increment_position(nr_steps,dir);
            set_dir(dir);
            c.change_curve(nr_steps);
            current_step=0;
            moving=true;
            this->nr_steps=nr_steps;
            next_step_time=time_us_32()+c.calculate_delay(current_step);
        }
        // main loop function (has to be called continouusly)
        void loop(){
            if(current_step<nr_steps && moving){
                uint current_time=time_us_32();
                if(current_time>=next_step_time){
                    step();
                    current_step++;
                    next_step_time=current_time+c.calculate_delay(current_step);
                }
            }
            else{
                moving=false;
            }
        }

        //moves the chain by a certain number of mm (rounded to the nearest step)
        void move_mm(float mm,direction dir){
            uint steps=mm_to_steps(mm);
            move(steps,dir);
            absolute_position_steps+=(steps*(dir==CW?1:-1));
        }
        // function to zero stepper with a limit switch
        void zero_motor(){
            uint delay_stage_1=1000;
            uint delay_stage_2=5000;
            uint delay_stage_3=10000;
            set_dir(CW);
            while(!*asoc_limit_switch){
                step();
                sleep_us(delay_stage_1);
            }
            set_dir(CCW);
            for(int i=0;i<mm_to_steps(80);i++){
                step();
                sleep_us(delay_stage_1);
            }
            //found true limit
            set_dir(CW);
            *asoc_limit_switch=false;
            while(!*asoc_limit_switch){
                step();
                sleep_us(delay_stage_1);
            }
            set_dir(CCW);
            for(int i=0;i<mm_to_steps(10);i++){
                step();
                sleep_us(delay_stage_2);
            }
            set_dir(CW);
            *asoc_limit_switch=false;
            while(!*asoc_limit_switch){
                step();
                sleep_us(delay_stage_3);
            }
            absolute_position_steps=0;
            //now go to center position

        }

        private:
        uint step_pin;
        uint dir_pin;
        float hw_angle_per_step;
        float angle_per_step;
        uint microstepping_mult;
        bool invert_dir;
        uint steps_per_rot;

        uint current_step;
        uint nr_steps;
        uint32_t next_step_time;
        volatile bool *asoc_limit_switch;

        //curve for acceleration/deceleration
        class curve{
            public:
                void init(uint microstepping_mult,uint min_delay=4000,uint max_delay=8000,uint accel_decel_phase_steps=500){
                    this->min_delay=min_delay/microstepping_mult;
                    this->max_delay=max_delay/microstepping_mult;
                    this->accel_decel_phase_steps=accel_decel_phase_steps;
                }
                // this function is used when you want to change the curve for example inside the move function of the stepper
                void change_curve(uint steps_total){
                    this->steps_total=steps_total;
                    if(steps_total<2*accel_decel_phase_steps){
                        this->accel_decel_phase_steps=steps_total/2;
                    }
                }
                // this returns the delay needed for the acceleration/deceleration curve at the specified index.
                uint calculate_delay(uint step_index){
                    if(step_index < accel_decel_phase_steps){
                        // Acceleration phase
                        float factor = static_cast<float>(step_index) / accel_decel_phase_steps;
                        return max_delay - static_cast<uint>((max_delay - min_delay) * factor);
                    }
                    else if(step_index >= steps_total - accel_decel_phase_steps){
                        // Deceleration phase
                        float factor = static_cast<float>(step_index - (steps_total - accel_decel_phase_steps)) / accel_decel_phase_steps;
                        return min_delay + static_cast<uint>((max_delay - min_delay) * factor);
                    }
                    else{
                        // Constant speed phase
                        return min_delay;
                    }
                }
            private:
                uint min_delay; //(us- microseconds)this changes the actual speed of the motor rotation. might want to add a helper funtion to calc delay as a funtion of rot/sec delaythis should not be lower or the motor stalls; it might be better with the curves implemented; please check
                uint max_delay; //(us- microseconds)change this if you want; this changes the minimum speed of the motor's acceleration/deceleration
                uint accel_decel_phase_steps; //change this to make the acceleration/deceleration longer or shorter; phases lenght is equal.
                uint steps_total;
        } c;

        // function calculates the delay needed for acceleration/deceleration curves
        // this function returns the number of steps needed to move a certain number of mm; steps are an integer, so it's going to get rounded probably...
        const float mm_per_rot=77; //placeholder, please check calibration if it's correct  ; checked and seems right aprox
        uint mm_to_steps(float mm){
            return static_cast<uint>(std::round((mm/mm_per_rot)*steps_per_rot));
        }
        //reverse of steps_to_mm
        float steps_to_mm(uint steps){
            return ((float)steps/steps_per_rot)*mm_per_rot;
        }
        //this should be uint but comparing it with an int promotes the int to uint and the comparison is always true
        int max_position_steps = 3500; //placeholder 111111111111!!!!!!!!!!!!!!!
        int offset_other_tooth; //placeholder this is the offset in case i want to zero from the other side
        int offset_center; //placeholder this is the offset to get to the center position(the middle of the zone)
        //this function adds or subtracts steps from absolute position counter
        void increment_position(uint steps,direction dir){
            //cw is pos, ccw is neg
            // int newpos=position_steps+steps*(dir==CW?1:-1);
            int newpos=absolute_position_steps+steps*(dir==CW?1:-1);
            if(newpos>max_position_steps){
                newpos-=max_position_steps;
            }
            else if(newpos<0){
                newpos+=max_position_steps;
            }
            absolute_position_steps=newpos;
        }   

};


//temporary functions for testing

//////////////////////////////////////////////////   MAIN LOOPS  ///////////////////////////////////////////////////////////////////////////////////////////
void foc_second_core(){
    stdio_usb_init();
    sleep_ms(1000);
    // foc objects initialization
    current_sensors cs(_CURRENT_SENSE_PIN_A,_CURRENT_SENSE_PIN_B,_CURRENT_SENSE_PIN_C);
    bridge_driver drv(_PWM_A_PIN,_PWM_B_PIN,_PWM_C_PIN,_DRIVER_ENABLE_PIN);
    encoder encoder(spi0,_PIN_SCK,_PIN_CS,_PIN_MISO,_PIN_MOSI,true);
    foc_controller foc(&drv,&encoder, &cs,7,24,15);

    foc.set_mode(3); 
    foc.set_angle(0);
    foc.asoc_driver->enable();
    
    command_packet comm_packet;
    while(true){
        //listen for commands from core 0
        if(!queue_is_empty(&comm_queue_01)){
            queue_remove_blocking(&comm_queue_01,&comm_packet);
            if(comm_packet.command==0){// new mode
                foc.set_mode(comm_packet.arguments[0]);
                // printf("C1: SET MODE: %f\n",comm_packet.arguments[0]);
            }
            else if(comm_packet.command==1){ //new target
                foc.set_target(comm_packet.arguments[0]);
                // printf("C1: SET TARGET: %f\n",comm_packet.arguments[0]);
            }
            else if(comm_packet.command==2){ //change regulator
                if(comm_packet.arguments[0]==1){ //current
                    if(comm_packet.arguments[1]==0){
                        foc.current_controller.kp=comm_packet.arguments[2];
                        // printf("C1: CURRENT NEW KP %f\n",foc.current_controller.kp);
                    }
                    else if(comm_packet.arguments[1]==1){
                        foc.current_controller.ki=comm_packet.arguments[2];
                        // printf("C1: CURRENT NEW KI %f\n",foc.current_controller.ki);
                    }
                }
                else if(comm_packet.arguments[0]==2){ //velocity
                    if(comm_packet.arguments[1]==0){
                        foc.vel_controller.kp=comm_packet.arguments[2];
                        // printf("C1: VEL NEW KP %f\n",foc.vel_controller.kp);
                    }
                    else if(comm_packet.arguments[1]==1){
                        foc.vel_controller.ki=comm_packet.arguments[2];
                        // printf("C1: VEL NEW KI %f\n",foc.vel_controller.ki);
                    }
                }
                else if(comm_packet.arguments[0]==3){ //angle
                    if(comm_packet.arguments[1]==0){
                        foc.angle_controller.kp=comm_packet.arguments[2];
                        // printf("C1: ANG NEW KP %f\n",foc.angle_controller.kp);
                    }
                    else if(comm_packet.arguments[1]==1){
                        foc.angle_controller.ki=comm_packet.arguments[2];
                        // printf("C1: ANG NEW KI %f\n",foc.angle_controller.ki);
                    }
                }
            }
        }
        // main foc loop
        foc.loop();
    }
}

int main()
{
    // I use usb for printing and uart for getting commands. I do this because i want to use the "better serial plotter" software on pc to tune PI controllers.
    // The software doesnt have a way to write to serial, so i use the Raspberry Zero W to send commands
    stdio_usb_init();
    initialize_uart();
 
    construct_sin_table();

    //multi core init stuff
    queue_init(&comm_queue_01,sizeof(command_packet),1);
    multicore_launch_core1(foc_second_core);

    //adds limit switch interrupts for steppers
    // add_limit_switch(_LIMIT_SWITCH_RIGHT);
    // add_limit_switch(_LIMIT_SWITCH_LEFT);
    
    //stepper driver initialization
    stepper_driver stp1(_STEP_PINA,_DIR_PINA,&g_limit_switch_left_triggered,1.8,4);
    stepper_driver stp2(_STEP_PINB,_DIR_PINB,&g_limit_switch_right_triggered,1.8,4);
    // stp1.zero_motor();
    // stp2.zero_motor();
    
    stp1.set_dir(stepper_driver::CW);
    stp2.set_dir(stepper_driver::CW);
    stp1.move_mm(50,stepper_driver::CCW);
    stp2.move(200*4,stepper_driver::CCW);
    
    //tuning
    // command_packet m_cmd_packet;
    // m_cmd_packet.command = 0;
    // m_cmd_packet.arguments[0]=3; //current
    // queue_add_blocking(&comm_queue_01,&m_cmd_packet);
    
    // uint64_t stp_tim=time_us_64()+500000;
    // int phs=0;
    // m_cmd_packet.command=1;
    while (true) {
        uart_check_command();
        handle_monitoring(monitoring_mask); //monitoring segment

        //tune setup
        // if(time_us_64()>stp_tim){
        //     if(phs==0){
        //         m_cmd_packet.arguments[0]=5;
        //         phs++;
        //     } 
        //     else if(phs==1){
        //         m_cmd_packet.arguments[0]=0;
        //         phs++;
        //     }
        //     else if(phs==2){
        //         m_cmd_packet.arguments[0]=-5;
        //         phs++;
        //     }
        //     else if(phs==3){
        //         m_cmd_packet.arguments[0]=0;
        //         phs=0;
        //     }
        //     queue_add_blocking(&comm_queue_01,&m_cmd_packet);
        //     stp_tim=time_us_64()+3500000;
        // }

        // test current transforms
        // for(float test_theta=0;test_theta<_2PI;test_theta+=0.05){
        //     test_c.a=sin(test_theta);
        //     test_c.b=sin(test_theta-(2*M_PI)/3.0);
        //     test_c.c=sin(test_theta+(2*M_PI)/3.0);
        //     // test_c.a=1.0;
        //     // test_c.b=2.0;
        //     // test_c.c=3.0;
        //     test_c.update_dq_values(test_theta);
        //     printf("%f %f %f %f %f %f %f      %f\n",test_c.a,test_c.b,test_c.c,test_c.alpha,test_c.beta,test_c.d,test_c.q,test_theta);
        //     sleep_ms(50);
        // }

        //read command from usb
        // char buffer[100];
        // int increment;
        // if(!stp1.moving){
        //     //i have to figure out how to find out the number of steps for a full cycle
        //     // mayve I keep adding uintil we pass the limit switch and then add a command that goes backwards until it finds the limit switch
        //     printf("CURRENT STEPS  %d\n",stp1.absolute_position_steps);
        //    
        //     fgets(buffer, sizeof(buffer), stdin);
        //     if(strcmp(buffer,"back\n")==0){
        //         stp1.set_dir(stepper_driver::CW);
        //         while(!g_limit_switch_left_triggered){
        //             stp1.step();
        //             sleep_us(5000);
        //             stp1.absolute_position_steps++;
        //         }
        //         printf("FOUND TOTAL LENGHT: %d\n",stp1.absolute_position_steps);
        //     }
        //     else{
        //         increment=atoi(buffer);
        //         printf("MOVING %d steps\n",increment);
        //         stepper_driver::direction newdir=increment>0?stepper_driver::CW:stepper_driver::CCW;
        //         increment= abs(increment);
        //         stp1.move(increment,newdir);
        //     }
        // }
        
        
        /// alternate stepper rotations
        // if(!stp1.moving && !stp2.moving){
        //     sleep_ms(1000);
        //     if(i){
        //         stp1.move((int)5*200*4,stepper_driver::CCW);
        //         stp2.move((int)5*200*4,stepper_driver::CW);
        //     }
        //     else{
        //         stp1.move((int)5*200*4,stepper_driver::CW);
        //         stp2.move((int)5*200*4,stepper_driver::CCW);
        //     }
        //     i=!i;
        // }

        //reset limit switches... temp?
        // if(g_limit_switch_right_triggered){
        //     g_limit_switch_right_triggered=false;
        // }
        // if(g_limit_switch_left_triggered){
        //     g_limit_switch_left_triggered=false;
        // }

        //stepper loops
        // stp1.loop();
        // stp2.loop();
    }
}
            