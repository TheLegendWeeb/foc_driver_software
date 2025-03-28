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

const uint lookup_resolution=1024;
// only for 0 to pi/2
//pune in startup
const float sin_table[lookup_resolution]={0.0, 0.0015339802, 0.0030679568, 0.0046019261, 0.0061358846, 0.0076698287, 0.0092037548, 0.0107376592, 0.0122715383, 0.0138053885, 0.0153392063, 0.0168729879, 0.0184067299, 0.0199404286, 0.0214740803, 0.0230076815, 0.0245412285, 0.0260747178, 0.0276081458, 0.0291415088, 0.0306748032, 0.0322080254, 0.0337411719, 0.0352742389, 0.0368072229, 0.0383401204, 0.0398729276, 0.041405641, 0.0429382569, 0.0444707719, 0.0460031821, 0.0475354842, 0.0490676743, 0.050599749, 0.0521317047, 0.0536635377, 0.0551952443, 0.0567268212, 0.0582582645, 0.0597895707, 0.0613207363, 0.0628517576, 0.0643826309, 0.0659133528, 0.0674439196, 0.0689743276, 0.0705045734, 0.0720346532, 0.0735645636, 0.0750943008, 0.0766238614, 0.0781532416, 0.079682438, 0.0812114468, 0.0827402645, 0.0842688876, 0.0857973123, 0.0873255352, 0.0888535526, 0.0903813609, 0.0919089565, 0.0934363358, 0.0949634953, 0.0964904314, 0.0980171403, 0.0995436187, 0.1010698628, 0.102595869, 0.1041216339, 0.1056471537, 0.107172425, 0.108697444, 0.1102222073, 0.1117467112, 0.1132709522, 0.1147949266, 0.1163186309, 0.1178420615, 0.1193652148, 0.1208880872, 0.1224106752, 0.1239329751, 0.1254549834, 0.1269766965, 0.1284981108, 0.1300192227, 0.1315400287, 0.1330605252, 0.1345807085, 0.1361005752, 0.1376201216, 0.1391393442, 0.1406582393, 0.1421768035, 0.1436950332, 0.1452129247, 0.1467304745, 0.148247679, 0.1497645347, 0.151281038, 0.1527971853, 0.154312973, 0.1558283977, 0.1573434556, 0.1588581433, 0.1603724572, 0.1618863938, 0.1633999494, 0.1649131205, 0.1664259035, 0.167938295, 0.1694502912, 0.1709618888, 0.172473084, 0.1739838734, 0.1754942534, 0.1770042204, 0.1785137709, 0.1800229014, 0.1815316083, 0.183039888, 0.1845477369, 0.1860551517, 0.1875621286, 0.1890686641, 0.1905747548, 0.192080397, 0.1935855873, 0.195090322, 0.1965945977, 0.1980984107, 0.1996017576, 0.2011046348, 0.2026070388, 0.2041089661, 0.2056104131, 0.2071113762, 0.208611852, 0.2101118369, 0.2116113274, 0.2131103199, 0.214608811, 0.2161067971, 0.2176042746, 0.2191012402, 0.2205976901, 0.222093621, 0.2235890292, 0.2250839114, 0.2265782638, 0.2280720832, 0.2295653658, 0.2310581083, 0.232550307, 0.2340419586, 0.2355330594, 0.237023606, 0.2385135948, 0.2400030224, 0.2414918853, 0.2429801799, 0.2444679027, 0.2459550503, 0.2474416192, 0.2489276057, 0.2504130066, 0.2518978182, 0.253382037, 0.2548656596, 0.2563486825, 0.2578311022, 0.2593129151, 0.2607941179, 0.262274707, 0.263754679, 0.2652340303, 0.2667127575, 0.2681908571, 0.2696683256, 0.2711451595, 0.2726213554, 0.2740969099, 0.2755718193, 0.2770460803, 0.2785196894, 0.2799926431, 0.2814649379, 0.2829365705, 0.2844075372, 0.2858778347, 0.2873474595, 0.2888164082, 0.2902846773, 0.2917522632, 0.2932191627, 0.2946853722, 0.2961508882, 0.2976157074, 0.2990798263, 0.3005432414, 0.3020059493, 0.3034679466, 0.3049292297, 0.3063897954, 0.30784964, 0.3093087603, 0.3107671527, 0.3122248139, 0.3136817404, 0.3151379288, 0.3165933756, 0.3180480774, 0.3195020308, 0.3209552324, 0.3224076788, 0.3238593665, 0.3253102922, 0.3267604523, 0.3282098436, 0.3296584625, 0.3311063058, 0.3325533699, 0.3339996514, 0.3354451471, 0.3368898534, 0.338333767, 0.3397768844, 0.3412192023, 0.3426607173, 0.344101426, 0.345541325, 0.3469804108, 0.3484186802, 0.3498561298, 0.3512927561, 0.3527285558, 0.3541635254, 0.3555976617, 0.3570309612, 0.3584634206, 0.3598950365, 0.3613258056, 0.3627557244, 0.3641847896, 0.3656129978, 0.3670403457, 0.36846683, 0.3698924471, 0.371317194, 0.372741067, 0.374164063, 0.3755861785, 0.3770074102, 0.3784277548, 0.3798472089, 0.3812657692, 0.3826834324, 0.384100195, 0.3855160538, 0.3869310055, 0.3883450467, 0.3897581741, 0.3911703843, 0.3925816741, 0.3939920401, 0.3954014789, 0.3968099874, 0.3982175622, 0.3996241998, 0.4010298972, 0.4024346509, 0.4038384576, 0.405241314, 0.4066432169, 0.4080441629, 0.4094441487, 0.4108431711, 0.4122412267, 0.4136383122, 0.4150344245, 0.4164295601, 0.4178237158, 0.4192168884, 0.4206090744, 0.4220002708, 0.4233904741, 0.4247796812, 0.4261678887, 0.4275550934, 0.4289412921, 0.4303264813, 0.431710658, 0.4330938189, 0.4344759606, 0.4358570799, 0.4372371737, 0.4386162385, 0.4399942713, 0.4413712687, 0.4427472276, 0.4441221446, 0.4454960165, 0.4468688402, 0.4482406123, 0.4496113297, 0.450980989, 0.4523495872, 0.453717121, 0.4550835871, 0.4564489824, 0.4578133036, 0.4591765475, 0.460538711, 0.4618997907, 0.4632597836, 0.4646186863, 0.4659764958, 0.4673332087, 0.468688822, 0.4700433325, 0.4713967368, 0.472749032, 0.4741002147, 0.4754502817, 0.4767992301, 0.4781470564, 0.4794937577, 0.4808393306, 0.4821837721, 0.4835270789, 0.484869248, 0.4862102761, 0.4875501601, 0.4888888969, 0.4902264833, 0.4915629161, 0.4928981922, 0.4942323085, 0.4955652618, 0.496897049, 0.498227667, 0.4995571125, 0.5008853826, 0.502212474, 0.5035383837, 0.5048631085, 0.5061866453, 0.5075089911, 0.5088301425, 0.5101500967, 0.5114688504, 0.5127864006, 0.5141027442, 0.515417878, 0.516731799, 0.5180445041, 0.5193559902, 0.5206662541, 0.5219752929, 0.5232831035, 0.5245896827, 0.5258950275, 0.5271991348, 0.5285020015, 0.5298036247, 0.5311040012, 0.5324031279, 0.5337010018, 0.5349976199, 0.5362929791, 0.5375870763, 0.5388799085, 0.5401714727, 0.5414617659, 0.5427507849, 0.5440385267, 0.5453249884, 0.5466101669, 0.5478940592, 0.5491766622, 0.5504579729, 0.5517379884, 0.5530167056, 0.5542941215, 0.555570233, 0.5568450373, 0.5581185312, 0.5593907119, 0.5606615762, 0.5619311212, 0.563199344, 0.5644662415, 0.5657318108, 0.5669960488, 0.5682589527, 0.5695205193, 0.5707807459, 0.5720396293, 0.5732971667, 0.574553355, 0.5758081914, 0.5770616729, 0.5783137964, 0.5795645591, 0.5808139581, 0.5820619903, 0.5833086529, 0.584553943, 0.5857978575, 0.5870403935, 0.5882815482, 0.5895213186, 0.5907597019, 0.591996695, 0.593232295, 0.5944664992, 0.5956993045, 0.5969307081, 0.598160707, 0.5993892984, 0.6006164794, 0.6018422471, 0.6030665985, 0.6042895309, 0.6055110414, 0.606731127, 0.607949785, 0.6091670123, 0.6103828063, 0.6115971639, 0.6128100824, 0.6140215589, 0.6152315906, 0.6164401745, 0.6176473079, 0.618852988, 0.6200572118, 0.6212599765, 0.6224612794, 0.6236611175, 0.6248594881, 0.6260563884, 0.6272518155, 0.6284457666, 0.6296382389, 0.6308292296, 0.6320187359, 0.6332067551, 0.6343932842, 0.6355783205, 0.6367618612, 0.6379439036, 0.6391244449, 0.6403034822, 0.6414810128, 0.642657034, 0.6438315429, 0.6450045368, 0.646176013, 0.6473459686, 0.648514401, 0.6496813074, 0.650846685, 0.6520105311, 0.653172843, 0.6543336178, 0.655492853, 0.6566505457, 0.6578066933, 0.658961293, 0.6601143421, 0.6612658378, 0.6624157776, 0.6635641586, 0.6647109782, 0.6658562337, 0.6669999223, 0.6681420414, 0.6692825883, 0.6704215604, 0.6715589548, 0.6726947691, 0.6738290004, 0.6749616461, 0.6760927036, 0.6772221701, 0.6783500431, 0.6794763199, 0.6806009978, 0.6817240742, 0.6828455464, 0.6839654118, 0.6850836678, 0.6862003117, 0.6873153409, 0.6884287528, 0.6895405447, 0.6906507141, 0.6917592584, 0.6928661748, 0.6939714609, 0.695075114, 0.6961771315, 0.6972775108, 0.6983762494, 0.6994733446, 0.7005687939, 0.7016625947, 0.7027547445, 0.7038452405, 0.7049340804, 0.7060212614, 0.7071067812, 0.708190637, 0.7092728264, 0.7103533469, 0.7114321957, 0.7125093706, 0.7135848688, 0.7146586879, 0.7157308253, 0.7168012785, 0.7178700451, 0.7189371224, 0.720002508, 0.7210661993, 0.7221281939, 0.7231884893, 0.724247083, 0.7253039724, 0.7263591551, 0.7274126286, 0.7284643904, 0.7295144381, 0.7305627692, 0.7316093812, 0.7326542717, 0.7336974381, 0.7347388781, 0.7357785892, 0.7368165689, 0.7378528148, 0.7388873245, 0.7399200955, 0.7409511254, 0.7419804117, 0.7430079521, 0.7440337442, 0.7450577854, 0.7460800735, 0.747100606, 0.7481193805, 0.7491363945, 0.7501516458, 0.7511651319, 0.7521768504, 0.753186799, 0.7541949753, 0.7552013769, 0.7562060014, 0.7572088465, 0.7582099098, 0.759209189, 0.7602066817, 0.7612023855, 0.7621962981, 0.7631884173, 0.7641787405, 0.7651672656, 0.7661539902, 0.7671389119, 0.7681220285, 0.7691033376, 0.770082837, 0.7710605243, 0.7720363972, 0.7730104534, 0.7739826906, 0.7749531066, 0.775921699, 0.7768884657, 0.7778534042, 0.7788165124, 0.7797777879, 0.7807372286, 0.7816948321, 0.7826505962, 0.7836045186, 0.7845565972, 0.7855068296, 0.7864552136, 0.787401747, 0.7883464276, 0.7892892532, 0.7902302214, 0.7911693302, 0.7921065773, 0.7930419605, 0.7939754776, 0.7949071263, 0.7958369046, 0.7967648102, 0.7976908409, 0.7986149946, 0.7995372691, 0.8004576622, 0.8013761717, 0.8022927955, 0.8032075315, 0.8041203774, 0.8050313311, 0.8059403906, 0.8068475535, 0.8077528179, 0.8086561816, 0.8095576424, 0.8104571983, 0.811354847, 0.8122505866, 0.8131444148, 0.8140363297, 0.8149263291, 0.8158144108, 0.8167005729, 0.8175848132, 0.8184671296, 0.8193475201, 0.8202259826, 0.821102515, 0.8219771153, 0.8228497814, 0.8237205112, 0.8245893028, 0.825456154, 0.8263210628, 0.8271840273, 0.8280450453, 0.8289041148, 0.8297612338, 0.8306164003, 0.8314696123, 0.8323208678, 0.8331701647, 0.8340175011, 0.834862875, 0.8357062844, 0.8365477272, 0.8373872016, 0.8382247056, 0.8390602371, 0.8398937942, 0.840725375, 0.8415549774, 0.8423825996, 0.8432082396, 0.8440318955, 0.8448535652, 0.845673247, 0.8464909388, 0.8473066387, 0.8481203448, 0.8489320552, 0.849741768, 0.8505494813, 0.8513551931, 0.8521589016, 0.8529606049, 0.8537603011, 0.8545579884, 0.8553536647, 0.8561473284, 0.8569389774, 0.85772861, 0.8585162243, 0.8593018184, 0.8600853904, 0.8608669386, 0.8616464611, 0.8624239561, 0.8631994217, 0.8639728561, 0.8647442575, 0.8655136241, 0.866280954, 0.8670462455, 0.8678094968, 0.868570706, 0.8693298713, 0.8700869911, 0.8708420635, 0.8715950867, 0.8723460589, 0.8730949784, 0.8738418435, 0.8745866523, 0.8753294031, 0.8760700942, 0.8768087238, 0.8775452902, 0.8782797917, 0.8790122264, 0.8797425928, 0.8804708891, 0.8811971135, 0.8819212643, 0.88264334, 0.8833633387, 0.8840812587, 0.8847970984, 0.8855108561, 0.8862225301, 0.8869321188, 0.8876396204, 0.8883450333, 0.8890483559, 0.8897495864, 0.8904487232, 0.8911457648, 0.8918407094, 0.8925335554, 0.8932243012, 0.8939129451, 0.8945994856, 0.895283921, 0.8959662498, 0.8966464702, 0.8973245807, 0.8980005797, 0.8986744657, 0.899346237, 0.900015892, 0.9006834292, 0.901348847, 0.9020121439, 0.9026733182, 0.9033323685, 0.9039892931, 0.9046440906, 0.9052967593, 0.9059472978, 0.9065957045, 0.9072419779, 0.9078861165, 0.9085281187, 0.9091679831, 0.9098057081, 0.9104412923, 0.9110747341, 0.911706032, 0.9123351846, 0.9129621904, 0.9135870479, 0.9142097557, 0.9148303122, 0.9154487161, 0.9160649658, 0.9166790599, 0.917290997, 0.9179007756, 0.9185083943, 0.9191138517, 0.9197171463, 0.9203182767, 0.9209172415, 0.9215140393, 0.9221086687, 0.9227011283, 0.9232914167, 0.9238795325, 0.9244654743, 0.9250492408, 0.9256308305, 0.9262102421, 0.9267874743, 0.9273625257, 0.9279353948, 0.9285060805, 0.9290745813, 0.9296408958, 0.9302050229, 0.9307669611, 0.9313267091, 0.9318842656, 0.9324396293, 0.9329927988, 0.933543773, 0.9340925504, 0.9346391298, 0.9351835099, 0.9357256895, 0.9362656672, 0.9368034417, 0.9373390119, 0.9378723764, 0.9384035341, 0.9389324835, 0.9394592236, 0.939983753, 0.9405060706, 0.9410261751, 0.9415440652, 0.9420597398, 0.9425731976, 0.9430844375, 0.9435934582, 0.9441002585, 0.9446048373, 0.9451071933, 0.9456073254, 0.9461052324, 0.9466009131, 0.9470943664, 0.947585591, 0.9480745859, 0.9485613499, 0.9490458819, 0.9495281806, 0.950008245, 0.9504860739, 0.9509616663, 0.951435021, 0.9519061368, 0.9523750127, 0.9528416476, 0.9533060404, 0.9537681899, 0.9542280951, 0.9546857549, 0.9551411683, 0.9555943341, 0.9560452513, 0.9564939189, 0.9569403357, 0.9573845008, 0.957826413, 0.9582660714, 0.9587034749, 0.9591386225, 0.9595715131, 0.9600021457, 0.9604305194, 0.9608566331, 0.9612804858, 0.9617020765, 0.9621214043, 0.962538468, 0.9629532669, 0.9633657998, 0.9637760658, 0.964184064, 0.9645897933, 0.9649932529, 0.9653944417, 0.9657933589, 0.9661900034, 0.9665843745, 0.966976471, 0.9673662922, 0.9677538371, 0.9681391047, 0.9685220943, 0.9689028048, 0.9692812354, 0.9696573851, 0.9700312532, 0.9704028387, 0.9707721407, 0.9711391584, 0.971503891, 0.9718663375, 0.9722264971, 0.9725843689, 0.9729399522, 0.9732932461, 0.9736442497, 0.9739929622, 0.9743393828, 0.9746835107, 0.9750253451, 0.9753648851, 0.97570213, 0.976037079, 0.9763697313, 0.9767000861, 0.9770281427, 0.9773539001, 0.9776773578, 0.9779985149, 0.9783173707, 0.9786339244, 0.9789481753, 0.9792601226, 0.9795697657, 0.9798771037, 0.980182136, 0.9804848618, 0.9807852804, 0.9810833912, 0.9813791933, 0.9816726862, 0.9819638691, 0.9822527414, 0.9825393023, 0.9828235512, 0.9831054874, 0.9833851103, 0.9836624192, 0.9839374134, 0.9842100924, 0.9844804554, 0.9847485018, 0.985014231, 0.9852776424, 0.9855387353, 0.9857975092, 0.9860539633, 0.9863080972, 0.9865599103, 0.9868094018, 0.9870565713, 0.9873014182, 0.9875439418, 0.9877841416, 0.9880220171, 0.9882575677, 0.9884907929, 0.988721692, 0.9889502645, 0.98917651, 0.9894004278, 0.9896220175, 0.9898412785, 0.9900582103, 0.9902728124, 0.9904850843, 0.9906950254, 0.9909026354, 0.9911079137, 0.9913108598, 0.9915114733, 0.9917097537, 0.9919057004, 0.9920993131, 0.9922905913, 0.9924795346, 0.9926661424, 0.9928504145, 0.9930323502, 0.9932119492, 0.9933892111, 0.9935641355, 0.9937367219, 0.99390697, 0.9940748793, 0.9942404495, 0.9944036801, 0.9945645707, 0.9947231211, 0.9948793308, 0.9950331994, 0.9951847267, 0.9953339121, 0.9954807555, 0.9956252564, 0.9957674145, 0.9959072294, 0.9960447009, 0.9961798286, 0.9963126122, 0.9964430514, 0.9965711458, 0.9966968952, 0.9968202993, 0.9969413578, 0.9970600703, 0.9971764367, 0.9972904567, 0.9974021299, 0.9975114561, 0.9976184351, 0.9977230666, 0.9978253504, 0.9979252862, 0.9980228738, 0.9981181129, 0.9982110034, 0.9983015449, 0.9983897374, 0.9984755806, 0.9985590742, 0.9986402182, 0.9987190122, 0.9987954562, 0.9988695499, 0.9989412932, 0.9990106859, 0.9990777278, 0.9991424187, 0.9992047586, 0.9992647473, 0.9993223846, 0.9993776704, 0.9994306046, 0.999481187, 0.9995294175, 0.999575296, 0.9996188225, 0.9996599967, 0.9996988187, 0.9997352883, 0.9997694054, 0.9998011699, 0.9998305818, 0.999857641, 0.9998823475, 0.9999047011, 0.9999247018, 0.9999423497, 0.9999576446, 0.9999705864, 0.9999811753, 0.9999894111, 0.9999952938, 0.9999988235};
float lookup_factor=lookup_resolution/M_PI_2;
// aproximates sin with a lookup table
float sin_aprox(float angle_rad){
    //BIGGEST ERROR ON WHOLE CYCLE  +-0.001531
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
// clamps angle and aprox. sin with a lookup table
float sin_aprox_clamp(float angle_rad){
    angle_rad=clamp_rad(angle_rad);
    return sin_aprox(angle_rad);
}
//aproximates cos with a lookup table
float cos_aprox(float angle_rad){
    //made this work
    return sin_aprox(clamp_rad(angle_rad+M_PI_2));
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
        // clarke transform
        void update_ab_values(){
            //formulas from https://www.ti.com/lit/an/bpra048/bpra048.pdf   ; tested using https://www.mathworks.com/help/mcb/ref/clarketransform.html
            //simplefoc does something about sign too
            // alpha = (2/3.0)*(a-(b-c));
            // beta = 2*_1overSQRT3*(b-c);
            alpha = 0.666666666 * (a - 0.5 * (b-c));
            beta = 0.666666666 * (_SQRT3 * 0.5 * (b - c));
        }
        // park transform
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
        const float gain=125;
        float center_offset_voltage_a=0;
        float center_offset_voltage_b=0;

        float read_raw_voltage(int channel){
            adc_select_input(channel);
            float voltage=adc_read();
            return (voltage/BIT_STEP)*VCC_Sensor;
        }
        void calculate_offset_voltage(){
            sleep_ms(500);
            center_offset_voltage_a=0;
            center_offset_voltage_b=0;
            for(int i=0;i<10000;i++){
                center_offset_voltage_a+=read_raw_voltage(_CURRENT_SENSE_CHANNEL_A);
                center_offset_voltage_b+=read_raw_voltage(_CURRENT_SENSE_CHANNEL_B);
            }
            center_offset_voltage_a/=10000.0;
            center_offset_voltage_b/=10000.0;
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



//class for foc algorithm
class foc_controller{
    public:
        foc_controller(bridge_driver* associated_driver, encoder* associated_encoder, current_sensor* associated_current_sensor, uint motor_pole_pairs, uint power_supply_voltage, float phase_resistance){
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
        void loop(){  //temporary
            //velocity controller conf ~200us exec (a little cogging at low speeds, prob should fix)
            float velocity_meas=asoc_encoder->get_velocity();   //~50us
            //pid
            float vel_error=velocity_target-velocity_meas;
            if(fabs(velocity_target)>0.001)
                integral_error += vel_error;
            float derivative_error=vel_error-prev_error;
            float uq=kp*vel_error+ki*integral_error+Kd * derivative_error;
            prev_error = vel_error;
        
            //change direction based on target sign and reference sign (needed for complete loop)
            direction regdir;
            if(velocity_target>0){
                regdir=direction::CW;
                if(uq<0){
                    if(regdir == direction::CW)
                        regdir=direction::CCW;
                    else
                        regdir=direction::CW;
                }
            }
            else{
                regdir=direction::CCW;
                if(uq>0){
                    if(regdir == direction::CW)
                        regdir=direction::CCW;
                    else
                        regdir=direction::CW;
                }
            }
                
            uq=fabs(uq);
            //avoid over-modulation
            if(uq>max_reference){
                uq=max_reference;
                integral_error-=vel_error;
            }

            //send pwm to motor
            setSVPWM(uq,0,get_target_electrical_angle(regdir)); // ~104-140us w/o lookup table   ~80-120us with lookup

            motor_current meas_current=asoc_cs->get_motor_current(); //~7us
            meas_current.update_dq_values(get_electrical_angle()); //~50 us w/o lookup table  ~40us with lookup
            printf("%f %f %f\n",meas_current.a,meas_current.b,meas_current.c);
            // printf("%f %f %f %f %f %f %f      %f\n",meas_current.a,meas_current.b,meas_current.c,meas_current.alpha,meas_current.beta,meas_current.d,meas_current.q,get_electrical_angle());
        }
        float velocity_target;

        // sets the pwm from a uq reference
        void setSVPWM(float Uq, float Ud, float target_el_angle){ //implemented according to https://www.youtube.com/watch?v=QMSWUMEAejg
            //el_angle has to be clamped between 0 and 2pi
            target_el_angle=clamp_rad(target_el_angle);
            int sector = int(target_el_angle/_PIover3)+1;

            float T1,T2,T0;
            float Uq_scaled = (Uq/(float)power_supply_voltage); //scales uq to [0,1]
            //lookup
            T1=_SQRT3*sin_aprox(sector*_PIover3-target_el_angle)*Uq_scaled;
            T2=_SQRT3*sin_aprox(target_el_angle-(sector-1.0)*_PIover3)*Uq_scaled;
            T0=1-T1-T2;
            // translate duty cycles to sectors
            float dA,dB,dC; //duty cycles for each phase needed for bridge;
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

        //velocity PID
        float kp=0.1;
        float ki=0.01;   //default values
        float Kd = 0.005; 
        float integral_error=0;
        float prev_error = 0;
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

// 6 step test function for the driver ; no feedback
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
    current_sensor cs(_CURRENT_SENSE_PIN_A,_CURRENT_SENSE_PIN_B);
    bridge_driver drv(_PWM_A_PIN,_PWM_B_PIN,_PWM_C_PIN,_DRIVER_ENABLE_PIN);
    encoder encoder(spi0,_PIN_SCK,_PIN_CS,_PIN_MISO,_PIN_MOSI,true);
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
    uint32_t tim=time_us_32()+2000*1000;
    foc.velocity_target=4*M_PI;
    while (true) {
        //test main loop timing
        // uint64_t exectime=time_us_64();
        foc.loop();
        // uint64_t donetime=time_us_64();
        // printf("EXEC %lld\n",donetime-exectime);
        // if(tim<time_us_32()){
        //    // foc.velocity_target*=-1;
        //     tim=time_us_32()+2000*1000;
        // }

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

        // stp1.loop();
        // stp2.loop();
        tight_loop_contents();
    }
}
            