/**
 * \file
 * \brief delay filter
 */

#include "dpd_delay_filter_p.h"

double dpd_FilterFracCoefs[DPD_FILTER_FRAC_DELAY_NUM][DPD_FILTER_FRAC_LEN] = {
    { -122.182824538950e-006, 348.927372048735e-006, -758.405012390980e-006, 1.43093436439779e-003, -2.49142712050140e-003, 4.19999594163140e-003,
      -7.33189032546160e-003, 15.8847211570659e-003, 999.008071666204e-003, -14.2506867519702e-003, 6.16380048743680e-003, -3.25022198444891e-003,
      1.74628860831910e-003, -885.324382296819e-006, 393.923152984564e-006, -133.952554483791e-006 },
    { -244.258568037916e-006, 697.846952194084e-006, -1.51750907033725e-003, 2.86491501126036e-003, -4.99243548342032e-003, 8.42778026837146e-003,
      -14.7522731071606e-003, 32.2227590090360e-003, 997.235055737893e-003, -28.0213063338312e-003, 12.2124749797400e-003, -6.45661200839039e-003,
      3.47372742454891e-003, -1.76259679400919e-003, 784.748786983529e-006, -266.992051546378e-006 },
    { -365.941199980328e-006, 1.04594237403732e-003, -2.27554087961756e-003, 4.29861304470594e-003, -7.49728052626074e-003, 12.6738880832691e-003,
      -22.2458797818866e-003, 48.9980894868177e-003, 994.683950291079e-003, -41.3010378068191e-003, 18.1344663948082e-003, -9.61231872255012e-003,
      5.17847908808299e-003, -2.62983158075733e-003, 1.17158254295489e-003, -398.811713043972e-006 },
    { -486.945139864433e-006, 1.39239716363011e-003, -3.03072558195971e-003, 5.72868488947377e-003, -10.0001728469236e-003, 16.9287291102105e-003,
      -29.7970789571400e-003, 66.1938996311517e-003, 991.358809415114e-003, -54.0799107141284e-003, 23.9186591862647e-003, -12.7106772535031e-003,
      6.85678932251251e-003, -3.48507916492042e-003, 1.55354406463626e-003, -529.108911129655e-006 },
    { -606.985899738691e-006, 1.73639656816232e-003, -3.78128869945256e-003, 7.15177961700992e-003, -12.4952905323791e-003, 21.1826067790377e-003,
      -37.3899013054408e-003, 83.7926096640941e-003, 987.264735931177e-003, -66.3488139981213e-003, 29.5543941993558e-003, -15.7452212460961e-003,
      8.50499441585681e-003, -4.32643011159374e-003, 1.92976888319418e-003, -657.585827802092e-006 },
    { -725.780725500375e-006, 2.07712938567202e-003, -4.52546009192850e-003, 8.56454632095729e-003, -14.9767916299137e-003, 25.4257376739049e-003,
      -45.0080652492241e-003, 101.775896391663e-003, 982.407872476172e-003, -78.0995034664798e-003, 35.0314825639038e-003, -18.7096943877810e-003,
      10.1195284501468e-003, -5.15201908138608e-003, 2.29941025693810e-003, -783.950100905405e-006 },
    { -843.049235490255e-006, 2.41378979166954e-003, -5.26147791861973e-003, 9.96364152630843e-003, -17.4388267355963e-003, 29.6482713088969e-003,
      -52.6350033941777e-003, 120.124717901378e-003, 976.795390969387e-003, -89.3246078031390e-003, 40.3402186080921e-003, -21.5980613978471e-003,
      11.6969302680609e-003, -5.96002866287620e-003, 2.66164096265106e-003, -907.915455362300e-006 },
    { -958.514054958310e-006, 2.74557915865522e-003, -5.98759259551371e-003, 11.3457366165837e-003, -19.8755516745820e-003, 33.8403101945541e-003,
      -60.2538896678481e-003, 138.819339516706e-003, 970.435480481361e-003, -100.017633121797e-003, 45.4713917782269e-003, -24.4045184653144e-003,
      13.2338501646580e-003, -6.74869307766553e-003, 3.01565503511362e-003, -1.02920231840995e-003 },
    { -1.07190144505219e-003, 3.07170786467672e-003, -6.70207074010029e-003, 12.7075252637484e-003, -22.2811402481805e-003, 37.9919301589900e-003,
      -67.8476671198397e-003, 157.839360969239e-003, 963.337333526007e-003, -110.172966061305e-003, 50.4162975506449e-003, -27.1235031197138e-003,
      14.7270562923503e-003, -7.51630175090741e-003, 3.36066945130047e-003, -1.14753841754445e-003 },
    { -1.18294192491656e-003, 3.39139708692576e-003, -7.40319909492854e-003, 14.0457308450923e-003, -24.6497970217882e-003, 42.0932008859026e-003,
      -75.3990763377625e-003, 177.163744746465e-003, 955.511130800469e-003, -119.785875425472e-003, 55.1667473244823e-003, -29.7497035207147e-003,
      16.1734407683227e-003, -8.26120274077613e-003, 3.69592575602109e-003, -1.26265935998948e-003 },
    { -1.29137088566486e-003, 3.70388057672741e-003, -8.08928842195702e-003, 15.3571138320543e-003, -26.9757701286227e-003, 46.1342066324520e-003,
      -82.8906844332403e-003, 196.770845572399e-003, 946.968024398279e-003, -128.852512370970e-003, 59.7150772847923e-003, -32.2780671529010e-003,
      17.5700254736510e-003, -8.98180602028020e-003, 4.02069162570302e-003, -1.37430919246610e-003 },
    { -1.39692919461919e-003, 4.00840641151084e-003, -8.75867735837992e-003, 16.6384791340206e-003, -29.2533640614531e-003, 50.1050670862917e-003,
      -90.3049145475530e-003, 216.638440972694e-003, 937.720119527519e-003, -137.369909152766e-003, 64.0541562302555e-003, -34.7038089154892e-003,
      18.9139675357137e-003, -9.67658660623734e-003, 4.33426236776084e-003, -1.48224094034447e-003 },
    { -1.49936378876247e-003, 4.30423872050419e-003, -9.40973622655697e-003, 17.8866833829338e-003, -31.4769524281137e-003, 53.9959583251075e-003,
      -97.6240758305066e-003, 236.743762879351e-003, 927.780454762704e-003, -145.335976433952e-003, 68.1773923573086e-003, -37.0224185945668e-003,
      20.2025644836537e-003, -10.3440875289106e-003, 4.63596235219725e-003, -1.58621712488702e-003 },
    { -1.59842825630102e-003, 4.59065938083065e-003, -10.0408707905661e-003, 19.0986421443715e-003, -33.6409906462487e-003, 57.7971338412989e-003,
      -104.830393844776e-003, 257.063530228067e-003, 917.162980861777e-003, -152.749499169383e-003, 72.0787389945285e-003, -39.2296677067460e-003,
      21.4332590673859e-003, -10.9829226361596e-003, 4.92514637221629e-003, -1.68601025731995e-003 },
    { -1.69388340418334e-003, 4.86696967836698e-003, -10.6505259478864e-003, 20.2713370346927e-003, -35.7400285444108e-003, 61.4989455838224e-003,
      -111.906041335194e-003, 277.573982486805e-003, 905.882538194616e-003, -159.610131086868e-003, 75.7526992934861e-003, -41.3216157129102e-003,
      22.6036437377438e-003, -11.5917792306224e-003, 5.20120093330478e-003, -1.78140330969594e-003 },
    { -1.78549781155887e-003, 5.13249193240528e-003, -11.2371893528016e-003, 21.4018227359239e-003, -37.7687228528067e-003, 65.0918649889161e-003,
      -118.833169324497e-003, 298.250914077091e-003, 893.954832807068e-003, -165.918387768816e-003, 79.1943298639008e-003, -43.2946155855345e-003,
      23.7114647738036e-003, -12.1694205305570e-003, 5.46354546557642e-003, -1.87219016032409e-003 },
    { -1.87304836677136e-003, 5.38657107795181e-003, -11.7993949591076e-003, 22.4872338865337e-003, -39.7218495486302e-003, 68.5665039484109e-003,
      -125.593938470089e-003, 319.069709619911e-003, 881.396411174895e-003, -171.675638365975e-003, 82.3992433659403e-003, -45.1453187331253e-003,
      24.7546260581954e-003, -12.7146879547840e-003, 5.71163345984701e-003, -1.95817601415432e-003 },
    { -1.95632078756165e-003, 5.62857620403361e-003, -12.3357264775578e-003, 23.5247918379776e-003, -41.5943160366372e-003, 71.9136356835086e-003,
      -132.170550637803e-003, 340.005379960299e-003, 868.224633680314e-003, -176.884095954780e-003, 85.3636100548152e-003, -46.8706792707080e-003,
      25.7311924901047e-003, -13.2265032245948e-003, 5.94495352343166e-003, -2.03917779640064e-003 },
    { -2.03511012309500e-003, 5.85790204409868e-003, -12.8448207395355e-003, 24.5118112609786e-003, -43.3811731375576e-003, 75.1242154816431e-003,
      -138.545280636332e-003, 361.032598910773e-003, 854.457646858430e-003, -181.546806563851e-003, 88.0841582849709e-003, -48.4679576349433e-003,
      26.6393930326740e-003, -13.7038702800807e-003, 6.16303035428021e-003, -2.11502451889163e-003 },
    { -2.10922123613781e-003, 6.07397041396221e-003, -13.3253709573553e-003, 25.4457065838586e-003, -45.0776268546124e-003, 78.1894012509173e-003,
      -144.700508051889e-003, 382.125740648616e-003, 840.114354466756e-003, -185.667636900956e-003, 90.5581739862289e-003, -49.9347235463289e-003,
      27.4776233942222e-003, -14.1458770100489e-003, 6.36542563300992e-003, -2.18555761903704e-003 },
    { -2.17846926513400e-003, 6.27623159586709e-003, -13.7761298769921e-003, 26.3239982533143e-003, -46.6790498992341e-003, 81.1005738597303e-003,
      -150.618749137141e-003, 403.258917716784e-003, 825.214387416818e-003, -189.251260798825e-003, 92.7834991128151e-003, -51.2688583114865e-003,
      28.2444483354540e-003, -14.5516967897172e-003, 6.55173882943762e-003, -2.25063126991773e-003 },
    { -2.24268006476144e-003, 6.46416566471751e-003, -14.1959128147172e-003, 27.1443188015668e-003, -48.1809929482956e-003, 83.8493572181109e-003,
      -156.282688695235e-003, 424.406019563270e-003, 809.778072622554e-003, -192.303144413275e-003, 94.7585290794453e-003, -52.4685564690914e-003,
      28.9386036025505e-003, -14.9205898254549e-003, 6.72160792312963e-003, -2.31011266133566e-003 },
    { -2.30169062357706e-003, 6.63728375259507e-003, -14.5836005692343e-003, 27.9044187039986e-003, -49.5791956053628e-003, 86.4276380573128e-003,
      -161.675211899279e-003, 445.540751552540e-003, 793.826400822148e-003, -194.829530209237e-003, 96.4822092004585e-003, -53.5323267842885e-003,
      29.5589974867968e-003, -15.2519043062381e-003, 6.87471003765261e-003, -2.36388225169752e-003 },
    { -2.35534945883500e-003, 6.79512925091387e-003, -14.9381422065248e-003, 28.6021720197898e-003, -50.8695970497375e-003, 88.8275853778904e-003,
      -166.779436002749e-003, 466.636674398073e-003, 777.380993415193e-003, -196.837419756969e-003, 97.9540301364190e-003, -54.4589925869301e-003,
      30.1047120041769e-003, -15.5450773565545e-003, 7.01076198529793e-003, -2.41183398926187e-003 },
    { -2.40351698623020e-003, 6.93727894452563e-003, -15.2585577059438e-003, 29.2355817951444e-003, -52.0483463396306e-003, 91.0416695147794e-003,
      -171.578741871011e-003, 487.667243937200e-003, 760.464068385771e-003, -198.334555387917e-003, 99.1740223763974e-003, -55.2476914680959e-003,
      30.5750037024541e-003, -15.7996357948269e-003, 7.12952072443016e-003, -2.45387550372391e-003 },
    { -2.44606586486429e-003, 7.06334407761856e-003, -15.5439404656775e-003, 29.8027852230133e-003, -53.1118123550625e-003, 93.0626807914790e-003,
      -176.056805290618e-003, 508.605851197054e-003, 743.098405355033e-003, -199.329400734945e-003, 100.142749762591e-003, -55.8978743317636e-003,
      30.9693040900005e-003, -16.0151966924508e-003, 7.23078372633492e-003, -2.48992826665867e-003 },
    { -2.48288131592441e-003, 7.17297134737621e-003, -15.7934596590203e-003, 30.3020585433222e-003, -54.0565933527322e-003, 94.8837477187890e-003,
      -180.197627993546e-003, 529.425862678812e-003, 725.307309829379e-003, -199.831120203131e-003, 100.861302082827e-003, -56.4093038137036e-003,
      31.2872196918317e-003, -16.1914677359476e-003, 7.31438925270672e-003, -2.51992772129876e-003 },
    { -2.51386141467406e-003, 7.26584382379181e-003, -16.0063624371561e-003, 30.7318216740694e-003, -54.8795261136729e-003, 96.4983547039764e-003,
      -183.985568345020e-003, 550.100660798955e-003, 707.114576699488e-003, -199.849557407697e-003, 101.331286748155e-003, -56.7820520727991e-003,
      31.5285317330518e-003, -16.3282473911256e-003, 7.38021654276930e-003, -2.54382338109494e-003 },
    { -2.53891735490097e-003, 7.34168179305733e-003, -16.1819759724825e-003, 31.0906425612251e-003, -55.5776946612234e-003, 97.9003592320474e-003,
      -187.405371638307e-003, 570.603684419918e-003, 688.544453052638e-003, -199.395212622877e-003, 101.554819579230e-003, -57.0164979651988e-003,
      31.6931954536669e-003, -16.4254248705762e-003, 7.42818591038772e-003, -2.56157889713540e-003 },
    { -2.55797368513545e-003, 7.40024352237648e-003, -16.3197093373163e-003, 31.3772412366515e-003, -56.1484385286271e-003, 99.0840084828950e-003,
      -190.442199941887e-003, 590.908469403935e-003, 669.621600360051e-003, -198.479219285351e-003, 101.534514725164e-003, -57.1133236115903e-003,
      31.7813390585157e-003, -16.4829799055934e-003, 7.45825875134497e-003, -2.57317209437497e-003 },
    { -2.57096851631280e-003, 7.44132594482878e-003, -16.4190552141801e-003, 31.5904935753087e-003, -56.5893605583125e-003, 100.043955351333e-003,
      -193.081661447657e-003, 610.988689126630e-003, 650.371056099138e-003, -197.113319594738e-003, 101.273473737742e-003, -57.0735103672316e-003,
      31.7932623054924e-003, -16.5009823231668e-003, 7.47043746067369e-003, -2.57859497649183e-003 },
    { -2.57785369987716e-003, 7.46476526150264e-003, -16.4795914315227e-003, 31.7294347396133e-003, -56.8983342104335e-003, 100.775273831455e-003,
      -195.309839262133e-003, 630.818194879055e-003, 630.818194879055e-003, -195.309839262133e-003, 100.775273831455e-003, -56.8983342104335e-003,
      31.7294347396133e-003, -16.4795914315227e-003, 7.46476526150264e-003, -2.57785369987716e-003 },
    { -2.57859497649183e-003, 7.47043746067369e-003, -16.5009823231668e-003, 31.7932623054924e-003, -57.0735103672316e-003, 101.273473737742e-003,
      -197.113319594738e-003, 650.371056099138e-003, 610.988689126630e-003, -193.081661447657e-003, 100.043955351333e-003, -56.5893605583125e-003,
      31.5904935753087e-003, -16.4190552141801e-003, 7.44132594482878e-003, -2.57096851631280e-003 },
    { -2.57317209437497e-003, 7.45825875134497e-003, -16.4829799055934e-003, 31.7813390585157e-003, -57.1133236115903e-003, 101.534514725164e-003,
      -198.479219285351e-003, 669.621600360051e-003, 590.908469403935e-003, -190.442199941887e-003, 99.0840084828950e-003, -56.1484385286271e-003,
      31.3772412366515e-003, -16.3197093373163e-003, 7.40024352237648e-003, -2.55797368513545e-003 },
    { -2.56157889713540e-003, 7.42818591038772e-003, -16.4254248705762e-003, 31.6931954536669e-003, -57.0164979651988e-003, 101.554819579230e-003,
      -199.395212622877e-003, 688.544453052638e-003, 570.603684419918e-003, -187.405371638307e-003, 97.9003592320474e-003, -55.5776946612234e-003,
      31.0906425612251e-003, -16.1819759724825e-003, 7.34168179305733e-003, -2.53891735490097e-003 },
    { -2.54382338109494e-003, 7.38021654276930e-003, -16.3282473911256e-003, 31.5285317330518e-003, -56.7820520727991e-003, 101.331286748155e-003,
      -199.849557407697e-003, 707.114576699488e-003, 550.100660798955e-003, -183.985568345020e-003, 96.4983547039764e-003, -54.8795261136729e-003,
      30.7318216740694e-003, -16.0063624371561e-003, 7.26584382379181e-003, -2.51386141467406e-003 },
    { -2.51992772129876e-003, 7.31438925270672e-003, -16.1914677359476e-003, 31.2872196918317e-003, -56.4093038137036e-003, 100.861302082827e-003,
      -199.831120203131e-003, 725.307309829379e-003, 529.425862678812e-003, -180.197627993546e-003, 94.8837477187890e-003, -54.0565933527322e-003,
      30.3020585433222e-003, -15.7934596590203e-003, 7.17297134737621e-003, -2.48288131592441e-003 },
    { -2.48992826665867e-003, 7.23078372633492e-003, -16.0151966924508e-003, 30.9693040900005e-003, -55.8978743317636e-003, 100.142749762591e-003,
      -199.329400734945e-003, 743.098405355033e-003, 508.605851197054e-003, -176.056805290618e-003, 93.0626807914790e-003, -53.1118123550625e-003,
      29.8027852230133e-003, -15.5439404656775e-003, 7.06334407761856e-003, -2.44606586486429e-003 },
    { -2.45387550372391e-003, 7.12952072443016e-003, -15.7996357948269e-003, 30.5750037024541e-003, -55.2476914680959e-003, 99.1740223763974e-003,
      -198.334555387917e-003, 760.464068385771e-003, 487.667243937200e-003, -171.578741871011e-003, 91.0416695147794e-003, -52.0483463396306e-003,
      29.2355817951444e-003, -15.2585577059438e-003, 6.93727894452563e-003, -2.40351698623020e-003 },
    { -2.41183398926187e-003, 7.01076198529793e-003, -15.5450773565545e-003, 30.1047120041769e-003, -54.4589925869301e-003, 97.9540301364190e-003,
      -196.837419756969e-003, 777.380993415193e-003, 466.636674398073e-003, -166.779436002749e-003, 88.8275853778904e-003, -50.8695970497375e-003,
      28.6021720197898e-003, -14.9381422065248e-003, 6.79512925091387e-003, -2.35534945883500e-003 },
    { -2.36388225169752e-003, 6.87471003765261e-003, -15.2519043062381e-003, 29.5589974867968e-003, -53.5323267842885e-003, 96.4822092004585e-003,
      -194.829530209237e-003, 793.826400822148e-003, 445.540751552540e-003, -161.675211899279e-003, 86.4276380573128e-003, -49.5791956053628e-003,
      27.9044187039986e-003, -14.5836005692343e-003, 6.63728375259507e-003, -2.30169062357706e-003 },
    { -2.31011266133566e-003, 6.72160792312963e-003, -14.9205898254549e-003, 28.9386036025505e-003, -52.4685564690914e-003, 94.7585290794453e-003,
      -192.303144413275e-003, 809.778072622554e-003, 424.406019563270e-003, -156.282688695235e-003, 83.8493572181109e-003, -48.1809929482956e-003,
      27.1443188015668e-003, -14.1959128147172e-003, 6.46416566471751e-003, -2.24268006476144e-003 },
    { -2.25063126991773e-003, 6.55173882943762e-003, -14.5516967897172e-003, 28.2444483354540e-003, -51.2688583114865e-003, 92.7834991128151e-003,
      -189.251260798825e-003, 825.214387416818e-003, 403.258917716784e-003, -150.618749137141e-003, 81.1005738597303e-003, -46.6790498992341e-003,
      26.3239982533143e-003, -13.7761298769921e-003, 6.27623159586709e-003, -2.17846926513400e-003 },
    { -2.18555761903704e-003, 6.36542563300992e-003, -14.1458770100489e-003, 27.4776233942222e-003, -49.9347235463289e-003, 90.5581739862289e-003,
      -185.667636900956e-003, 840.114354466756e-003, 382.125740648616e-003, -144.700508051889e-003, 78.1894012509173e-003, -45.0776268546124e-003,
      25.4457065838586e-003, -13.3253709573553e-003, 6.07397041396221e-003, -2.10922123613781e-003 },
    { -2.11502451889163e-003, 6.16303035428021e-003, -13.7038702800807e-003, 26.6393930326740e-003, -48.4679576349433e-003, 88.0841582849709e-003,
      -181.546806563851e-003, 854.457646858430e-003, 361.032598910773e-003, -138.545280636332e-003, 75.1242154816431e-003, -43.3811731375576e-003,
      24.5118112609786e-003, -12.8448207395355e-003, 5.85790204409868e-003, -2.03511012309500e-003 },
    { -2.03917779640064e-003, 5.94495352343166e-003, -13.2265032245948e-003, 25.7311924901047e-003, -46.8706792707080e-003, 85.3636100548152e-003,
      -176.884095954780e-003, 868.224633680314e-003, 340.005379960299e-003, -132.170550637803e-003, 71.9136356835086e-003, -41.5943160366372e-003,
      23.5247918379776e-003, -12.3357264775578e-003, 5.62857620403361e-003, -1.95632078756165e-003 },
    { -1.95817601415432e-003, 5.71163345984701e-003, -12.7146879547840e-003, 24.7546260581954e-003, -45.1453187331253e-003, 82.3992433659403e-003,
      -171.675638365975e-003, 881.396411174895e-003, 319.069709619911e-003, -125.593938470089e-003, 68.5665039484109e-003, -39.7218495486302e-003,
      22.4872338865337e-003, -11.7993949591076e-003, 5.38657107795181e-003, -1.87304836677136e-003 },
    { -1.87219016032409e-003, 5.46354546557642e-003, -12.1694205305570e-003, 23.7114647738036e-003, -43.2946155855345e-003, 79.1943298639008e-003,
      -165.918387768816e-003, 893.954832807068e-003, 298.250914077091e-003, -118.833169324497e-003, 65.0918649889161e-003, -37.7687228528067e-003,
      21.4018227359239e-003, -11.2371893528016e-003, 5.13249193240528e-003, -1.78549781155887e-003 },
    { -1.78140330969594e-003, 5.20120093330478e-003, -11.5917792306224e-003, 22.6036437377438e-003, -41.3216157129102e-003, 75.7526992934861e-003,
      -159.610131086868e-003, 905.882538194616e-003, 277.573982486805e-003, -111.906041335194e-003, 61.4989455838224e-003, -35.7400285444108e-003,
      20.2713370346927e-003, -10.6505259478864e-003, 4.86696967836698e-003, -1.69388340418334e-003 },
    { -1.68601025731995e-003, 4.92514637221629e-003, -10.9829226361596e-003, 21.4332590673859e-003, -39.2296677067460e-003, 72.0787389945285e-003,
      -152.749499169383e-003, 917.162980861777e-003, 257.063530228067e-003, -104.830393844776e-003, 57.7971338412989e-003, -33.6409906462487e-003,
      19.0986421443715e-003, -10.0408707905661e-003, 4.59065938083065e-003, -1.59842825630102e-003 },
    { -1.58621712488702e-003, 4.63596235219725e-003, -10.3440875289106e-003, 20.2025644836537e-003, -37.0224185945668e-003, 68.1773923573086e-003,
      -145.335976433952e-003, 927.780454762704e-003, 236.743762879351e-003, -97.6240758305066e-003, 53.9959583251075e-003, -31.4769524281137e-003,
      17.8866833829338e-003, -9.40973622655697e-003, 4.30423872050419e-003, -1.49936378876247e-003 },
    { -1.48224094034447e-003, 4.33426236776084e-003, -9.67658660623734e-003, 18.9139675357137e-003, -34.7038089154892e-003, 64.0541562302555e-003,
      -137.369909152766e-003, 937.720119527519e-003, 216.638440972694e-003, -90.3049145475530e-003, 50.1050670862917e-003, -29.2533640614531e-003,
      16.6384791340206e-003, -8.75867735837992e-003, 4.00840641151084e-003, -1.39692919461919e-003 },
    { -1.37430919246610e-003, 4.02069162570302e-003, -8.98180602028020e-003, 17.5700254736510e-003, -32.2780671529010e-003, 59.7150772847923e-003,
      -128.852512370970e-003, 946.968024398279e-003, 196.770845572399e-003, -82.8906844332403e-003, 46.1342066324520e-003, -26.9757701286227e-003,
      15.3571138320543e-003, -8.08928842195702e-003, 3.70388057672741e-003, -1.29137088566486e-003 },
    { -1.26265935998948e-003, 3.69592575602109e-003, -8.26120274077613e-003, 16.1734407683227e-003, -29.7497035207147e-003, 55.1667473244823e-003,
      -119.785875425472e-003, 955.511130800469e-003, 177.163744746465e-003, -75.3990763377625e-003, 42.0932008859026e-003, -24.6497970217882e-003,
      14.0457308450923e-003, -7.40319909492854e-003, 3.39139708692576e-003, -1.18294192491656e-003 },
    { -1.14753841754445e-003, 3.36066945130047e-003, -7.51630175090741e-003, 14.7270562923503e-003, -27.1235031197138e-003, 50.4162975506449e-003,
      -110.172966061305e-003, 963.337333526007e-003, 157.839360969239e-003, -67.8476671198397e-003, 37.9919301589900e-003, -22.2811402481805e-003,
      12.7075252637484e-003, -6.70207074010029e-003, 3.07170786467672e-003, -1.07190144505219e-003 },
    { -1.02920231840995e-003, 3.01565503511362e-003, -6.74869307766553e-003, 13.2338501646580e-003, -24.4045184653144e-003, 45.4713917782269e-003,
      -100.017633121797e-003, 970.435480481361e-003, 138.819339516706e-003, -60.2538896678481e-003, 33.8403101945541e-003, -19.8755516745820e-003,
      11.3457366165837e-003, -5.98759259551371e-003, 2.74557915865522e-003, -958.514054958310e-006 },
    { -907.915455362300e-006, 2.66164096265106e-003, -5.96002866287620e-003, 11.6969302680609e-003, -21.5980613978471e-003, 40.3402186080921e-003,
      -89.3246078031390e-003, 976.795390969387e-003, 120.124717901378e-003, -52.6350033941777e-003, 29.6482713088969e-003, -17.4388267355963e-003,
      9.96364152630843e-003, -5.26147791861973e-003, 2.41378979166954e-003, -843.049235490255e-006 },
    { -783.950100905405e-006, 2.29941025693810e-003, -5.15201908138608e-003, 10.1195284501468e-003, -18.7096943877810e-003, 35.0314825639038e-003,
      -78.0995034664798e-003, 982.407872476172e-003, 101.775896391663e-003, -45.0080652492241e-003, 25.4257376739049e-003, -14.9767916299137e-003,
      8.56454632095729e-003, -4.52546009192850e-003, 2.07712938567202e-003, -725.780725500375e-006 },
    { -657.585827802092e-006, 1.92976888319418e-003, -4.32643011159374e-003, 8.50499441585681e-003, -15.7452212460961e-003, 29.5543941993558e-003,
      -66.3488139981213e-003, 987.264735931177e-003, 83.7926096640941e-003, -37.3899013054408e-003, 21.1826067790377e-003, -12.4952905323791e-003,
      7.15177961700992e-003, -3.78128869945256e-003, 1.73639656816232e-003, -606.985899738691e-006 },
    { -529.108911129655e-006, 1.55354406463626e-003, -3.48507916492042e-003, 6.85678932251251e-003, -12.7106772535031e-003, 23.9186591862647e-003,
      -54.0799107141284e-003, 991.358809415114e-003, 66.1938996311517e-003, -29.7970789571400e-003, 16.9287291102105e-003, -10.0001728469236e-003,
      5.72868488947377e-003, -3.03072558195971e-003, 1.39239716363011e-003, -486.945139864433e-006 },
    { -398.811713043972e-006, 1.17158254295489e-003, -2.62983158075733e-003, 5.17847908808299e-003, -9.61231872255012e-003, 18.1344663948082e-003,
      -41.3010378068191e-003, 994.683950291079e-003, 48.9980894868177e-003, -22.2458797818866e-003, 12.6738880832691e-003, -7.49728052626074e-003,
      4.29861304470594e-003, -2.27554087961756e-003, 1.04594237403732e-003, -365.941199980328e-006 },
    { -266.992051546378e-006, 784.748786983529e-006, -1.76259679400919e-003, 3.47372742454891e-003, -6.45661200839039e-003, 12.2124749797400e-003,
      -28.0213063338312e-003, 997.235055737893e-003, 32.2227590090360e-003, -14.7522731071606e-003, 8.42778026837146e-003, -4.99243548342032e-003,
      2.86491501126036e-003, -1.51750907033725e-003, 697.846952194084e-006, -244.258568037916e-006 },
    { -133.952554483791e-006, 393.923152984564e-006, -885.324382296819e-006, 1.74628860831910e-003, -3.25022198444891e-003, 6.16380048743680e-003,
      -14.2506867519702e-003, 999.008071666204e-003, 15.8847211570659e-003, -7.33189032546160e-003, 4.19999594163140e-003, -2.49142712050140e-003,
      1.43093436439779e-003, -758.405012390980e-006, 348.927372048735e-006, -122.182824538950e-006 },
    { -82.6066959751123e-015, 168.940306320292e-015, -286.878528430729e-015, 418.067487990336e-015, -550.603578518375e-015, 659.738934377114e-015,
      -733.503623544992e-015, 1.00000000000076e+000, -733.503623544992e-015, 659.738934377114e-015, -550.603578518375e-015, 418.067487990336e-015,
      -286.878528430729e-015, 168.940306320292e-015, -82.6066959751123e-015, 0.00000000000000e+000 },
};


double dpd_FilterTxCoefs[DPD_FILTER_TX_NUM][DPD_FILTER_TX_LEN] = {
    {  3.01933028361802e-003, -4.96099322468122e-003, 6.67830481616430e-003,
       -11.3204971527707e-018, -25.1916579702603e-003, 72.0648094762661e-003,
       -130.626662776015e-003, 180.070826746406e-003, 797.892085297006e-003,
       180.070826746406e-003, -130.626662776015e-003, 72.0648094762661e-003,
       -25.1916579702603e-003, -11.3204971527707e-018, 6.67830481616430e-003,
       -4.96099322468122e-003, 3.01933028361802e-003 },    /*  0: 0.8Fs filter */
    { -1.56198277301601e-018, -5.23918106301453e-003, 4.19257420520499e-018,
      23.2111017863651e-003, -10.5433837178581e-018, -76.1058457486735e-003,
      16.8941932305112e-018, 307.698778736744e-003, 500.870292577157e-003,
      307.698778736744e-003, 16.8941932305112e-018, -76.1058457486735e-003,
      -10.5433837178581e-018, 23.2111017863651e-003, 4.19257420520499e-018,
      -5.23918106301453e-003, -1.56198277301601e-018 },    /*  1: 0.5Fs filter */
};

/*@}*/