#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/pwm.h"

//driver pins
#define _PWM_A_PIN 13
#define _PWM_B_PIN 14
#define _PWM_C_PIN 15
#define _DRIVER_ENABLE_PIN 12

class driver{
    public:
        driver(uint pin_a, uint pin_b, uint pin_c, uint pin_enable, uint pwm_freq=50000){
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
 
int main()
{
    stdio_init_all();
    sleep_ms(1000);
    driver drv(_PWM_A_PIN,_PWM_B_PIN,_PWM_C_PIN,_DRIVER_ENABLE_PIN);
    float i=0;
    drv.enable();
    while (true) {
        // printf("Duty: %.2f\n",i*100);
        i+=0.0005;
        if(i>1){
            i-=1;
        }
        drv.set_pwm_duty(i);
        sleep_ms(1);
    }
}
