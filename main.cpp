#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"

#include <cmath>
#include <cstring>

//current sense pins
#define _CURRENT_SENSE_PIN_A 26
#define _CURRENT_SENSE_PIN_B 27
#define _CURRENT_SENSE_CHANNEL_A 0
#define _CURRENT_SENSE_CHANNEL_B 1
//motor two phase resistance:16.6

// SPI Defines
#define _PIN_MISO 16
#define _PIN_CS   17  //for encoder
#define _PIN_SCK  18
#define _PIN_MOSI 19

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

//limit switch pins
#define _LIMIT_SWITCH_RIGHT 4
#define _LIMIT_SWITCH_LEFT 5

//Util values defs
#define _2PI        6.2831853072
#define _PIover3    1.0471975512
#define _SQRT3      1.7320508075
#define _1overSQRT3 0.5773502691

//useful functions


//this clamps an angle to the 0 to 2PI range
float clamp_rad(float angle_rad){
    angle_rad=fmod(angle_rad,_2PI);
    if(angle_rad<0)
        angle_rad+=_2PI;
    return angle_rad;
}

const uint lookup_resolution=1200;
const float sin_table[lookup_resolution]={0.0, 0.0052359638, 0.0104717841, 0.0157073173, 0.0209424199, 0.0261769483, 0.0314107591, 0.0366437087, 0.0418756537, 0.0471064507, 0.0523359562, 0.057564027, 0.0627905195, 0.0680152907, 0.0732381971, 0.0784590957, 0.0836778433, 0.0888942969, 0.0941083133, 0.0993197497, 0.1045284633, 0.1097343111, 0.1149371505, 0.1201368388, 0.1253332336, 0.1305261922, 0.1357155724, 0.1409012319, 0.1460830286, 0.1512608202, 0.156434465, 0.1616038211, 0.1667687467, 0.1719291003, 0.1770847403, 0.1822355255, 0.1873813146, 0.1925219665, 0.1976573404, 0.2027872954, 0.2079116908, 0.2130303863, 0.2181432414, 0.223250116, 0.2283508701, 0.2334453639, 0.2385334576, 0.2436150118, 0.2486898872, 0.2537579446, 0.2588190451, 0.26387305, 0.2689198206, 0.2739592187, 0.278991106, 0.2840153447, 0.2890317969, 0.2940403252, 0.2990407923, 0.3040330609, 0.3090169944, 0.313992456, 0.3189593093, 0.3239174182, 0.3288666467, 0.3338068592, 0.3387379202, 0.3436596946, 0.3485720473, 0.3534748438, 0.3583679495, 0.3632512305, 0.3681245527, 0.3729877826, 0.3778407868, 0.3826834324, 0.3875155865, 0.3923371166, 0.3971478906, 0.4019477767, 0.4067366431, 0.4115143586, 0.4162807923, 0.4210358134, 0.4257792916, 0.4305110968, 0.4352310994, 0.4399391699, 0.4446351792, 0.4493189986, 0.4539904997, 0.4586495545, 0.4632960351, 0.4679298143, 0.4725507649, 0.4771587603, 0.4817536741, 0.4863353804, 0.4909037536, 0.4954586684, 0.5, 0.5045276238, 0.5090414158, 0.5135412521, 0.5180270094, 0.5224985647, 0.5269557955, 0.5313985795, 0.535826795, 0.5402403205, 0.544639035, 0.549022818, 0.5533915492, 0.557745109, 0.5620833779, 0.5664062369, 0.5707135677, 0.575005252, 0.5792811723, 0.5835412114, 0.5877852523, 0.5920131788, 0.596224875, 0.6004202253, 0.6045991149, 0.608761429, 0.6129070537, 0.6170358751, 0.6211477803, 0.6252426563, 0.6293203911, 0.6333808726, 0.6374239898, 0.6414496316, 0.6454576877, 0.6494480483, 0.653420604, 0.6573752458, 0.6613118653, 0.6652303547, 0.6691306064, 0.6730125135, 0.6768759697, 0.680720869, 0.6845471059, 0.6883545757, 0.6921431739, 0.6959127966, 0.6996633405, 0.7033947028, 0.7071067812, 0.7107994739, 0.7144726796, 0.7181262978, 0.7217602281, 0.725374371, 0.7289686274, 0.7325428988, 0.7360970871, 0.739631095, 0.7431448255, 0.7466381823, 0.7501110696, 0.7535633923, 0.7569950557, 0.7604059656, 0.7637960286, 0.7671651518, 0.7705132428, 0.7738402097, 0.7771459615, 0.7804304073, 0.7836934573, 0.786935022, 0.7901550124, 0.7933533403, 0.796529918, 0.7996846585, 0.8028174752, 0.8059282823, 0.8090169944, 0.8120835269, 0.8151277957, 0.8181497174, 0.8211492091, 0.8241261886, 0.8270805743, 0.8300122851, 0.8329212407, 0.8358073614, 0.8386705679, 0.8415107819, 0.8443279255, 0.8471219214, 0.849892693, 0.8526401644, 0.8553642602, 0.8580649057, 0.860742027, 0.8633955506, 0.8660254038, 0.8686315144, 0.8712138111, 0.873772223, 0.87630668, 0.8788171127, 0.8813034521, 0.8837656301, 0.8862035792, 0.8886172327, 0.8910065242, 0.8933713883, 0.8957117602, 0.8980275758, 0.9003187714, 0.9025852844, 0.9048270525, 0.9070440143, 0.909236109, 0.9114032766, 0.9135454576, 0.9156625933, 0.9177546257, 0.9198214973, 0.9218631516, 0.9238795325, 0.9258705848, 0.9278362539, 0.9297764859, 0.9316912276, 0.9335804265, 0.9354440308, 0.9372819895, 0.9390942521, 0.940880769, 0.9426414911, 0.9443763702, 0.9460853588, 0.94776841, 0.9494254776, 0.9510565163, 0.9526614813, 0.9542403285, 0.9557930148, 0.9573194975, 0.9588197349, 0.9602936857, 0.9617413096, 0.9631625668, 0.9645574185, 0.9659258263, 0.9672677528, 0.9685831611, 0.9698720153, 0.9711342799, 0.9723699204, 0.9735789029, 0.9747611942, 0.9759167619, 0.9770455744, 0.9781476007, 0.9792228106, 0.9802711746, 0.981292664, 0.9822872507, 0.9832549076, 0.984195608, 0.9851093262, 0.9859960371, 0.9868557164, 0.9876883406, 0.9884938868, 0.989272333, 0.9900236577, 0.9907478405, 0.9914448614, 0.9921147013, 0.9927573419, 0.9933727656, 0.9939609555, 0.9945218954, 0.99505557, 0.9955619646, 0.9960410654, 0.9964928592, 0.9969173337, 0.9973144772, 0.9976842788, 0.9980267284, 0.9983418166, 0.9986295348, 0.998889875, 0.9991228301, 0.9993283938, 0.9995065604, 0.999657325, 0.9997806835, 0.9998766325, 0.9999451694, 0.9999862922, 1.0, 0.9999862922, 0.9999451694, 0.9998766325, 0.9997806835, 0.999657325, 0.9995065604, 0.9993283938, 0.9991228301, 0.998889875, 0.9986295348, 0.9983418166, 0.9980267284, 0.9976842788, 0.9973144772, 0.9969173337, 0.9964928592, 0.9960410654, 0.9955619646, 0.99505557, 0.9945218954, 0.9939609555, 0.9933727656, 0.9927573419, 0.9921147013, 0.9914448614, 0.9907478405, 0.9900236577, 0.989272333, 0.9884938868, 0.9876883406, 0.9868557164, 0.9859960371, 0.9851093262, 0.984195608, 0.9832549076, 0.9822872507, 0.981292664, 0.9802711746, 0.9792228106, 0.9781476007, 0.9770455744, 0.9759167619, 0.9747611942, 0.9735789029, 0.9723699204, 0.9711342799, 0.9698720153, 0.9685831611, 0.9672677528, 0.9659258263, 0.9645574185, 0.9631625668, 0.9617413095, 0.9602936857, 0.9588197349, 0.9573194975, 0.9557930148, 0.9542403285, 0.9526614813, 0.9510565163, 0.9494254776, 0.94776841, 0.9460853588, 0.9443763702, 0.9426414911, 0.940880769, 0.9390942521, 0.9372819895, 0.9354440308, 0.9335804265, 0.9316912276, 0.9297764859, 0.9278362539, 0.9258705848, 0.9238795325, 0.9218631516, 0.9198214973, 0.9177546257, 0.9156625933, 0.9135454576, 0.9114032766, 0.909236109, 0.9070440143, 0.9048270525, 0.9025852843, 0.9003187714, 0.8980275758, 0.8957117602, 0.8933713883, 0.8910065242, 0.8886172327, 0.8862035792, 0.8837656301, 0.8813034521, 0.8788171127, 0.87630668, 0.873772223, 0.8712138111, 0.8686315144, 0.8660254038, 0.8633955506, 0.860742027, 0.8580649057, 0.8553642602, 0.8526401644, 0.849892693, 0.8471219214, 0.8443279255, 0.8415107819, 0.8386705679, 0.8358073614, 0.8329212407, 0.8300122851, 0.8270805743, 0.8241261886, 0.8211492091, 0.8181497174, 0.8151277957, 0.8120835269, 0.8090169944, 0.8059282822, 0.8028174752, 0.7996846585, 0.796529918, 0.7933533403, 0.7901550124, 0.786935022, 0.7836934573, 0.7804304073, 0.7771459615, 0.7738402097, 0.7705132428, 0.7671651518, 0.7637960286, 0.7604059656, 0.7569950556, 0.7535633923, 0.7501110696, 0.7466381823, 0.7431448255, 0.739631095, 0.7360970871, 0.7325428988, 0.7289686274, 0.725374371, 0.7217602281, 0.7181262978, 0.7144726796, 0.7107994739, 0.7071067812, 0.7033947028, 0.6996633405, 0.6959127966, 0.6921431739, 0.6883545757, 0.6845471059, 0.680720869, 0.6768759697, 0.6730125135, 0.6691306064, 0.6652303546, 0.6613118653, 0.6573752458, 0.653420604, 0.6494480483, 0.6454576877, 0.6414496316, 0.6374239897, 0.6333808726, 0.629320391, 0.6252426563, 0.6211477803, 0.6170358751, 0.6129070536, 0.608761429, 0.6045991149, 0.6004202253, 0.596224875, 0.5920131788, 0.5877852523, 0.5835412113, 0.5792811723, 0.575005252, 0.5707135677, 0.5664062369, 0.5620833778, 0.557745109, 0.5533915492, 0.549022818, 0.544639035, 0.5402403205, 0.535826795, 0.5313985795, 0.5269557955, 0.5224985647, 0.5180270094, 0.5135412521, 0.5090414157, 0.5045276238, 0.5, 0.4954586684, 0.4909037536, 0.4863353804, 0.4817536741, 0.4771587603, 0.4725507649, 0.4679298143, 0.4632960351, 0.4586495545, 0.4539904997, 0.4493189986, 0.4446351792, 0.4399391698, 0.4352310994, 0.4305110968, 0.4257792916, 0.4210358134, 0.4162807923, 0.4115143586, 0.4067366431, 0.4019477766, 0.3971478906, 0.3923371166, 0.3875155864, 0.3826834324, 0.3778407868, 0.3729877826, 0.3681245527, 0.3632512305, 0.3583679495, 0.3534748438, 0.3485720473, 0.3436596946, 0.3387379202, 0.3338068592, 0.3288666467, 0.3239174182, 0.3189593093, 0.313992456, 0.3090169944, 0.3040330609, 0.2990407922, 0.2940403252, 0.2890317969, 0.2840153447, 0.278991106, 0.2739592187, 0.2689198206, 0.26387305, 0.2588190451, 0.2537579446, 0.2486898872, 0.2436150118, 0.2385334576, 0.2334453638, 0.2283508701, 0.223250116, 0.2181432414, 0.2130303863, 0.2079116908, 0.2027872953, 0.1976573404, 0.1925219665, 0.1873813146, 0.1822355255, 0.1770847403, 0.1719291003, 0.1667687467, 0.1616038211, 0.156434465, 0.1512608202, 0.1460830286, 0.1409012319, 0.1357155724, 0.1305261922, 0.1253332336, 0.1201368388, 0.1149371505, 0.1097343111, 0.1045284633, 0.0993197497, 0.0941083133, 0.0888942969, 0.0836778433, 0.0784590957, 0.0732381971, 0.0680152907, 0.0627905195, 0.0575640269, 0.0523359562, 0.0471064507, 0.0418756537, 0.0366437087, 0.0314107591, 0.0261769483, 0.0209424199, 0.0157073173, 0.0104717841, 0.0052359638, -0.0, -0.0052359638, -0.0104717841, -0.0157073173, -0.0209424199, -0.0261769483, -0.0314107591, -0.0366437087, -0.0418756537, -0.0471064507, -0.0523359563, -0.057564027, -0.0627905195, -0.0680152907, -0.0732381971, -0.0784590957, -0.0836778433, -0.0888942969, -0.0941083133, -0.0993197498, -0.1045284633, -0.1097343111, -0.1149371505, -0.1201368388, -0.1253332336, -0.1305261922, -0.1357155724, -0.1409012319, -0.1460830286, -0.1512608203, -0.1564344651, -0.1616038211, -0.1667687467, -0.1719291003, -0.1770847403, -0.1822355255, -0.1873813146, -0.1925219665, -0.1976573404, -0.2027872954, -0.2079116908, -0.2130303863, -0.2181432414, -0.223250116, -0.2283508701, -0.2334453639, -0.2385334576, -0.2436150118, -0.2486898872, -0.2537579446, -0.2588190451, -0.26387305, -0.2689198206, -0.2739592187, -0.278991106, -0.2840153447, -0.289031797, -0.2940403252, -0.2990407923, -0.3040330609, -0.3090169944, -0.313992456, -0.3189593093, -0.3239174182, -0.3288666467, -0.3338068592, -0.3387379203, -0.3436596946, -0.3485720473, -0.3534748438, -0.3583679496, -0.3632512305, -0.3681245527, -0.3729877826, -0.3778407868, -0.3826834324, -0.3875155865, -0.3923371166, -0.3971478906, -0.4019477767, -0.4067366431, -0.4115143586, -0.4162807923, -0.4210358134, -0.4257792916, -0.4305110968, -0.4352310994, -0.4399391699, -0.4446351792, -0.4493189986, -0.4539904998, -0.4586495545, -0.4632960351, -0.4679298143, -0.4725507649, -0.4771587603, -0.4817536741, -0.4863353804, -0.4909037536, -0.4954586684, -0.5, -0.5045276238, -0.5090414158, -0.5135412521, -0.5180270094, -0.5224985647, -0.5269557955, -0.5313985795, -0.535826795, -0.5402403205, -0.544639035, -0.549022818, -0.5533915493, -0.557745109, -0.5620833779, -0.5664062369, -0.5707135677, -0.5750052521, -0.5792811724, -0.5835412114, -0.5877852523, -0.5920131788, -0.596224875, -0.6004202253, -0.6045991149, -0.608761429, -0.6129070537, -0.6170358752, -0.6211477803, -0.6252426563, -0.6293203911, -0.6333808726, -0.6374239898, -0.6414496316, -0.6454576877, -0.6494480483, -0.653420604, -0.6573752458, -0.6613118653, -0.6652303547, -0.6691306064, -0.6730125135, -0.6768759697, -0.680720869, -0.6845471059, -0.6883545757, -0.6921431739, -0.6959127966, -0.6996633405, -0.7033947028, -0.7071067812, -0.7107994739, -0.7144726796, -0.7181262978, -0.7217602281, -0.725374371, -0.7289686274, -0.7325428988, -0.7360970871, -0.739631095, -0.7431448255, -0.7466381823, -0.7501110696, -0.7535633923, -0.7569950557, -0.7604059656, -0.7637960286, -0.7671651518, -0.7705132428, -0.7738402097, -0.7771459615, -0.7804304073, -0.7836934573, -0.786935022, -0.7901550124, -0.7933533403, -0.796529918, -0.7996846585, -0.8028174752, -0.8059282823, -0.8090169944, -0.8120835269, -0.8151277957, -0.8181497174, -0.8211492091, -0.8241261886, -0.8270805743, -0.8300122851, -0.8329212407, -0.8358073614, -0.838670568, -0.841510782, -0.8443279255, -0.8471219214, -0.849892693, -0.8526401644, -0.8553642602, -0.8580649057, -0.860742027, -0.8633955506, -0.8660254038, -0.8686315144, -0.8712138111, -0.873772223, -0.8763066801, -0.8788171127, -0.8813034521, -0.8837656301, -0.8862035792, -0.8886172327, -0.8910065242, -0.8933713883, -0.8957117602, -0.8980275758, -0.9003187714, -0.9025852844, -0.9048270525, -0.9070440143, -0.9092361091, -0.9114032766, -0.9135454576, -0.9156625933, -0.9177546257, -0.9198214973, -0.9218631516, -0.9238795325, -0.9258705848, -0.9278362539, -0.9297764859, -0.9316912276, -0.9335804265, -0.9354440308, -0.9372819895, -0.9390942521, -0.940880769, -0.9426414911, -0.9443763702, -0.9460853588, -0.94776841, -0.9494254776, -0.9510565163, -0.9526614813, -0.9542403285, -0.9557930148, -0.9573194975, -0.9588197349, -0.9602936857, -0.9617413096, -0.9631625668, -0.9645574185, -0.9659258263, -0.9672677528, -0.9685831611, -0.9698720153, -0.9711342799, -0.9723699204, -0.9735789029, -0.9747611942, -0.9759167619, -0.9770455744, -0.9781476007, -0.9792228106, -0.9802711746, -0.981292664, -0.9822872507, -0.9832549076, -0.984195608, -0.9851093262, -0.9859960371, -0.9868557164, -0.9876883406, -0.9884938868, -0.989272333, -0.9900236577, -0.9907478405, -0.9914448614, -0.9921147013, -0.9927573419, -0.9933727656, -0.9939609555, -0.9945218954, -0.99505557, -0.9955619646, -0.9960410654, -0.9964928593, -0.9969173337, -0.9973144772, -0.9976842788, -0.9980267284, -0.9983418166, -0.9986295348, -0.998889875, -0.9991228301, -0.9993283938, -0.9995065604, -0.999657325, -0.9997806835, -0.9998766325, -0.9999451694, -0.9999862922, -1.0, -0.9999862922, -0.9999451694, -0.9998766325, -0.9997806835, -0.999657325, -0.9995065604, -0.9993283938, -0.9991228301, -0.998889875, -0.9986295348, -0.9983418166, -0.9980267284, -0.9976842788, -0.9973144772, -0.9969173337, -0.9964928592, -0.9960410654, -0.9955619646, -0.99505557, -0.9945218954, -0.9939609555, -0.9933727656, -0.9927573419, -0.9921147013, -0.9914448614, -0.9907478405, -0.9900236577, -0.989272333, -0.9884938868, -0.9876883406, -0.9868557164, -0.9859960371, -0.9851093262, -0.984195608, -0.9832549076, -0.9822872507, -0.981292664, -0.9802711746, -0.9792228106, -0.9781476007, -0.9770455744, -0.9759167619, -0.9747611942, -0.9735789029, -0.9723699204, -0.9711342799, -0.9698720153, -0.9685831611, -0.9672677528, -0.9659258263, -0.9645574185, -0.9631625668, -0.9617413095, -0.9602936857, -0.9588197349, -0.9573194975, -0.9557930148, -0.9542403285, -0.9526614812, -0.9510565163, -0.9494254776, -0.94776841, -0.9460853588, -0.9443763702, -0.9426414911, -0.9408807689, -0.9390942521, -0.9372819895, -0.9354440308, -0.9335804265, -0.9316912276, -0.9297764859, -0.9278362539, -0.9258705848, -0.9238795325, -0.9218631516, -0.9198214973, -0.9177546257, -0.9156625933, -0.9135454576, -0.9114032766, -0.909236109, -0.9070440143, -0.9048270525, -0.9025852843, -0.9003187714, -0.8980275758, -0.8957117602, -0.8933713883, -0.8910065242, -0.8886172326, -0.8862035792, -0.8837656301, -0.8813034521, -0.8788171127, -0.87630668, -0.873772223, -0.8712138111, -0.8686315144, -0.8660254038, -0.8633955506, -0.860742027, -0.8580649057, -0.8553642602, -0.8526401643, -0.849892693, -0.8471219214, -0.8443279255, -0.8415107819, -0.8386705679, -0.8358073614, -0.8329212407, -0.8300122851, -0.8270805743, -0.8241261886, -0.8211492091, -0.8181497174, -0.8151277957, -0.8120835269, -0.8090169944, -0.8059282822, -0.8028174752, -0.7996846585, -0.796529918, -0.7933533403, -0.7901550124, -0.786935022, -0.7836934573, -0.7804304073, -0.7771459614, -0.7738402097, -0.7705132428, -0.7671651518, -0.7637960286, -0.7604059656, -0.7569950556, -0.7535633923, -0.7501110696, -0.7466381823, -0.7431448255, -0.739631095, -0.7360970871, -0.7325428988, -0.7289686274, -0.725374371, -0.7217602281, -0.7181262978, -0.7144726796, -0.7107994739, -0.7071067812, -0.7033947028, -0.6996633405, -0.6959127966, -0.6921431739, -0.6883545757, -0.6845471059, -0.6807208689, -0.6768759697, -0.6730125135, -0.6691306063, -0.6652303546, -0.6613118653, -0.6573752458, -0.653420604, -0.6494480483, -0.6454576877, -0.6414496316, -0.6374239897, -0.6333808726, -0.629320391, -0.6252426563, -0.6211477803, -0.6170358751, -0.6129070536, -0.608761429, -0.6045991148, -0.6004202253, -0.596224875, -0.5920131788, -0.5877852523, -0.5835412113, -0.5792811723, -0.575005252, -0.5707135677, -0.5664062369, -0.5620833778, -0.557745109, -0.5533915492, -0.549022818, -0.544639035, -0.5402403205, -0.535826795, -0.5313985795, -0.5269557955, -0.5224985647, -0.5180270094, -0.513541252, -0.5090414157, -0.5045276238, -0.5, -0.4954586684, -0.4909037536, -0.4863353804, -0.4817536741, -0.4771587602, -0.4725507649, -0.4679298142, -0.4632960351, -0.4586495545, -0.4539904997, -0.4493189986, -0.4446351792, -0.4399391698, -0.4352310994, -0.4305110968, -0.4257792915, -0.4210358134, -0.4162807922, -0.4115143586, -0.4067366431, -0.4019477766, -0.3971478906, -0.3923371166, -0.3875155864, -0.3826834323, -0.3778407868, -0.3729877826, -0.3681245527, -0.3632512305, -0.3583679495, -0.3534748438, -0.3485720473, -0.3436596946, -0.3387379202, -0.3338068592, -0.3288666467, -0.3239174182, -0.3189593093, -0.3139924559, -0.3090169944, -0.3040330609, -0.2990407922, -0.2940403252, -0.2890317969, -0.2840153447, -0.278991106, -0.2739592187, -0.2689198206, -0.2638730499, -0.2588190451, -0.2537579446, -0.2486898871, -0.2436150118, -0.2385334576, -0.2334453638, -0.2283508701, -0.223250116, -0.2181432414, -0.2130303863, -0.2079116908, -0.2027872953, -0.1976573404, -0.1925219665, -0.1873813146, -0.1822355255, -0.1770847403, -0.1719291003, -0.1667687467, -0.1616038211, -0.156434465, -0.1512608202, -0.1460830285, -0.1409012319, -0.1357155724, -0.1305261922, -0.1253332335, -0.1201368388, -0.1149371505, -0.1097343111, -0.1045284632, -0.0993197497, -0.0941083133, -0.0888942968, -0.0836778433, -0.0784590957, -0.0732381971, -0.0680152906, -0.0627905195, -0.0575640269, -0.0523359562, -0.0471064507, -0.0418756537, -0.0366437087, -0.0314107591, -0.0261769483, -0.0209424199, -0.0157073173, -0.0104717841, -0.0052359638};
float lookup_factor=lookup_resolution/_2PI;
// aproximates sin with a lookup table
//HAVE TO TEST PRECISION FOR WHOLE RANGE or just pi/2
float sin_aprox(float angle_rad){
    //BIGGEST ERROR ON WHOLE CYCLE  +-0.002613
    angle_rad=clamp_rad(angle_rad);
    return sin_table[(int)round(angle_rad*lookup_factor)%lookup_resolution];
}
//aproximates cos with a lookup table
float cos_aprox(float angle_rad){
    return sin_aprox(angle_rad+M_PI_2);
}



//class for currents (contains each phase and stuff)
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
        void update_ab_values(){
            //formulas from https://www.ti.com/lit/an/bpra048/bpra048.pdf   ; tested using https://www.mathworks.com/help/mcb/ref/clarketransform.html
            //simplefoc does something about sign too
            // alpha = (2/3.0)*(a-(b-c));
            // beta = 2*_1overSQRT3*(b-c);
            alpha = 0.666666666 * (a - 0.5 * (b-c));
            beta = 0.666666666 * (_SQRT3 * 0.5 * (b - c));
        }
        void update_dq_values(float el_angle){
            update_ab_values();
            //make this use a lookup table
            float cos_theta=cos_aprox(el_angle);
            float sin_theta=sin_aprox(el_angle);
            d=alpha*cos_theta+beta*sin_theta;
            q=beta*cos_theta-alpha*sin_theta;
        }
};

//class for the current sensor
class current_sensor{
    public:
        current_sensor(int cs_phase_a_pin, int cs_phase_b_pin){
            pinA=cs_phase_a_pin;
            pinB=cs_phase_b_pin;
            adc_init();
            adc_gpio_init(pinA);
            adc_gpio_init(pinB);

            calculate_offset_voltage();
        }
        motor_current get_motor_current(){
            motor_current current;
            current.a=read_raw_voltage(_CURRENT_SENSE_CHANNEL_A)-center_offset_voltage_a;
            current.b=read_raw_voltage(_CURRENT_SENSE_CHANNEL_B)-center_offset_voltage_b;
            current.a*=1000.0;
            current.b*=1000.0;
            current.a=current.a/gain;
            current.b=current.b/gain;
            current.c=-current.a-current.b;
            return current;
        }
        
    private:
        uint pinA;
        uint pinB;
        const float VCC_Sensor=3.3;
        const float BIT_STEP=4096.0;
        const float gain=122.1;
        float center_offset_voltage_a=0;
        float center_offset_voltage_b=0;

        float read_raw_voltage(int channel){
            adc_select_input(channel);
            float voltage=adc_read();
            return (voltage/BIT_STEP)*VCC_Sensor;
        }
        void calculate_offset_voltage(){
            center_offset_voltage_a=0;
            center_offset_voltage_b=0;
            for(int i=0;i<1000;i++){
                center_offset_voltage_a+=read_raw_voltage(_CURRENT_SENSE_CHANNEL_A);
                sleep_us(1);
                center_offset_voltage_b+=read_raw_voltage(_CURRENT_SENSE_CHANNEL_B);
                sleep_us(1);
            }
            center_offset_voltage_a/=1000.0;
            center_offset_voltage_b/=1000.0;
        }
};

// spi encoder class
class encoder{
    // TODO: Add possibility to send offset to sensor
    // Add functions for continuous angles
    public:
        // class constructor
        encoder(spi_inst_t *spi_channel,uint sck_pin,uint cs_pin, uint miso_pin, uint mosi_pin, bool reverse=false){
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
            if(reverse)
                angle_int=16383-angle_int;
            return angle_int;
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
        float get_velocity(){
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
            smoothed_velocity = ALPHA * raw_velocity + (1 - ALPHA) * smoothed_velocity; //LP FILTER
            return smoothed_velocity;
        }
        private:
        uint cs_pin;
        spi_inst_t* spi_channel;
        const uint16_t ANGLE_READ_COMMAND=0xFFFF;
        bool reverse;
        //velocity func
        float old_angle=0;
        uint64_t old_time=0;
        float smoothed_velocity=0;
        float ALPHA=0.05; //5% weight seems fine
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

//class for foc algorithm
class foc_controller{
    public:
        foc_controller(bridge_driver* associated_driver, encoder* associated_encoder, current_sensor* associated_current_sensor, uint motor_pole_pairs, uint power_supply_voltage, float phase_resistance, float current_limit){
            this->asoc_driver=associated_driver;
            this->asoc_encoder=associated_encoder;
            this->asoc_cs=associated_current_sensor;

            this->motor_pole_pairs=motor_pole_pairs;
            this->current_limit=current_limit;
            this->power_supply_voltage=power_supply_voltage;
            max_reference=power_supply_voltage/M_SQRT3;
            //hard current limiting. This is only used w/o current sense

            this->motor_phase_resistance=phase_resistance;

            el_angle_offset=0;
            align();
        }
        //find offset for electrical angle
        void align(){
            float tests=30;
            float sum_sin=0;
            float sum_cos=0;
            asoc_driver->enable();
            setSVPWM(6.9,0,M_PI); //move motor to 180 deg (pi rad)
            sleep_ms(1000);
            for(int i=0;i<tests;i++){
                float el_ang=get_electrical_angle();
                sum_sin+=sin(el_ang);
                sum_cos+=cos(el_ang);
                sleep_ms(10);
            }
            
            float el_angle_offset_reconstructed=atan2(sum_sin/tests,sum_cos/tests);
            el_angle_offset=clamp_rad(el_angle_offset_reconstructed);
            setSVPWM(0,0,0);
            asoc_driver->disable();
        }
        //foc loop
        void loop(){
            //temp
            setSVPWM(max_reference,0,get_target_electrical_angle(direction::CCW)); // ~100us w/o lookup table   ~80-120us with lookup
            
            motor_current meas_current=asoc_cs->get_motor_current(); //~7us
            float a=asoc_encoder->get_velocity();   //~50us
            meas_current.update_dq_values(get_electrical_angle()); //~50 us w/o lookup table
            // printf("%f %f %f %f %f %f %f      %f\n",meas_current.a,meas_current.b,meas_current.c,meas_current.alpha,meas_current.beta,meas_current.d,meas_current.q,get_electrical_angle());
        }
        // i think this might be overmodulated
        void setSVPWM(float Uq, float Ud, float target_el_angle){ //implemented according to https://www.youtube.com/watch?v=QMSWUMEAejg
            // ~100us
            //el_angle has to be clamped between 0 and 2pi
            target_el_angle=clamp_rad(target_el_angle);
            int sector = floor(target_el_angle/_PIover3)+1;

            float dA,dB,dC; //duty cycles for each phase needed for bridge;
            float T1,T2,T0;
            //lookup
            T1=_SQRT3*sin_aprox(sector*_PIover3-target_el_angle)*(Uq/(float)power_supply_voltage);
            T2=_SQRT3*sin_aprox(target_el_angle-(sector-1.0)*_PIover3)*(Uq/(float)power_supply_voltage);
            T0=1-T1-T2;
            // translate duty cycles to sectors
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
            asoc_driver->set_pwm_duty(dA,dB,dC);
        }
        enum direction{
            CW=0,
            CCW=1
        };
        bridge_driver* asoc_driver;
        encoder* asoc_encoder;
        current_sensor* asoc_cs;
    // private:
        uint power_supply_voltage;
        float motor_phase_resistance;
        float current_limit;

        uint motor_pole_pairs;
        float el_angle_offset;
        float get_electrical_angle(){
            return clamp_rad(motor_pole_pairs*asoc_encoder->get_angle_rad()-el_angle_offset);
        }
        float get_target_electrical_angle(direction dir){
            float angle=get_electrical_angle();
            if(dir==direction::CW)
                angle=clamp_rad(angle-M_PI_2);
            else
                angle=clamp_rad(angle+M_PI_2);
            return angle;
        }
        float max_reference;
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
void six_step(bridge_driver* driver){
    int sixstepdelay=10;
    driver->enable();
    driver->set_pwm_duty(1,0,0);
    sleep_ms(sixstepdelay);
    driver->set_pwm_duty(1,1,0);
    sleep_ms(sixstepdelay);
    driver->set_pwm_duty(0,1,0);
    sleep_ms(sixstepdelay);
    driver->set_pwm_duty(0,1,1);
    sleep_ms(sixstepdelay);
    driver->set_pwm_duty(0,0,1);
    sleep_ms(sixstepdelay);
    driver->set_pwm_duty(1,0,1);
    sleep_ms(sixstepdelay);
}



int main()
{
    stdio_init_all();
    
    // foc objects initialization
    bridge_driver drv(_PWM_A_PIN,_PWM_B_PIN,_PWM_C_PIN,_DRIVER_ENABLE_PIN);
    encoder encoder(spi0,_PIN_SCK,_PIN_CS,_PIN_MISO,_PIN_MOSI,true);
    current_sensor cs(_CURRENT_SENSE_PIN_A,_CURRENT_SENSE_PIN_B);
    foc_controller foc(&drv,&encoder, &cs,7,24,15);

    //adds limit switch interrupts for steppers
    add_limit_switch(_LIMIT_SWITCH_RIGHT);
    add_limit_switch(_LIMIT_SWITCH_LEFT);
    
    //stepper driver initialization
    stepper_driver stp1(_STEP_PINA,_DIR_PINA,&g_limit_switch_left_triggered,1.8,4);
    stepper_driver stp2(_STEP_PINB,_DIR_PINB,&g_limit_switch_right_triggered,1.8,4);
    // stp1.zero_motor();
    // stp2.zero_motor();
    
    // stp1.set_dir(stepper_driver::CW);
    // stp2.set_dir(stepper_driver::CW);
    // stp1.move_mm(50,stepper_driver::CCW);
    // stp2.move(200*4,stepper_driver::CCW);
    
    foc.asoc_driver->enable();

    motor_current test_c;
    while (true) {
        foc.loop();

        //test current transforms
        // for(float test_theta=0;test_theta<_2PI;test_theta+=0.05){
        //     test_c.a=sin(test_theta);
        //     test_c.b=sin(test_theta+(2*M_PI)/3.0);
        //     test_c.c=sin(test_theta+(4*M_PI)/3.0);
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
        if(g_limit_switch_right_triggered){
            g_limit_switch_right_triggered=false;
        }
        if(g_limit_switch_left_triggered){
            g_limit_switch_left_triggered=false;
        }

        stp1.loop();
        stp2.loop();
        sleep_us(1);
    }
}
            