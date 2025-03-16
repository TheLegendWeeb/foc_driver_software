#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/pwm.h"

#include <cmath>
//driver pins
#define _PWM_A_PIN 13
#define _PWM_B_PIN 14
#define _PWM_C_PIN 15
#define _DRIVER_ENABLE_PIN 12

//stepper pins
#define _STEP_PINA 1
#define _DIR_PINA 0
#define _STEP_PINB 3
#define _DIR_PINB 2

#define _LIMIT_SWITCH_RIGHT 4
#define _LIMIT_SWITCH_LEFT 5

// class for FOC driver
class foc_driver{
    public:
        foc_driver(uint pin_a, uint pin_b, uint pin_c, uint pin_enable, uint pwm_freq=50000){
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
            pwm_config_set_wrap(&conf, pwm_wrap);
            pwm_config_set_clkdiv(&conf,1.0);
            pwm_init(slice_A, &conf, true);
            pwm_init(slice_B, &conf, true);
            pwm_init(slice_C, &conf, true);

            pwm_set_enabled(slice_A, true);
            pwm_set_enabled(slice_B, true);
            pwm_set_enabled(slice_C, true);

            gpio_init(pin_EN);
            gpio_set_dir(pin_EN,GPIO_OUT);
            gpio_put(pin_EN,0);
        }
        //enable the driver
        void enable(){
            gpio_put(pin_EN,1);
        }
        //disable the driver
        void disable(){
            gpio_put(pin_EN,0);
        }
        void set_pwm_duty(float duty){
            pwm_set_gpio_level(pin_A,duty*pwm_wrap);
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
            return (125000000/freq);
        }
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
            printf("right        ");
            g_limit_switch_right_triggered = true;
            last_debounce_time_right = current_time;
        }
    }
    else if(gpio==_LIMIT_SWITCH_LEFT){
        if (current_time - last_debounce_time_left > DEBOUNCE_DELAY_MS) {
            printf("left");
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
            zero_motor();
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
        int max_position_steps = 3000; //placeholder
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

int main()
{
    stdio_init_all();
    // sleep_ms(1000);
    // foc_driver drv(_PWM_A_PIN,_PWM_B_PIN,_PWM_C_PIN,_DRIVER_ENABLE_PIN);
    add_limit_switch(_LIMIT_SWITCH_RIGHT);
    add_limit_switch(_LIMIT_SWITCH_LEFT);
    sleep_ms(3000);
    stepper_driver stp1(_STEP_PINA,_DIR_PINA,&g_limit_switch_left_triggered,1.8,4);
    // stepper_driver stp2(_STEP_PINB,_DIR_PINB,&g_limit_switch_right_triggered,1.8,4);
    
    // // drv.enable();
    stp1.set_dir(stepper_driver::CW);
    // stp2.set_dir(stepper_driver::CW);
    // stp1.move_mm(50,stepper_driver::CCW);
    // stp2.move(200*4,stepper_driver::CCW);
    int i=0;
    while (true) {
        // stp1.loop();
        // stp2.loop();
        // // // drv.set_pwm_duty(i);
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
        if(g_limit_switch_right_triggered){
            g_limit_switch_right_triggered=false;
        }
        if(g_limit_switch_left_triggered){
            g_limit_switch_left_triggered=false;
        }
        sleep_us(1);
    }
}
