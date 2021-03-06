#ifndef LQR3_P2_mix1_DL1_H
#define LQR3_P2_mix1_DL1_H

double LQR3_P2_mix1_DL1(double* tv, int tdim, double* param, int pdim) {

double t1 = tv[0], t2 = tv[1];

double r1 = param[0];
double r2 = param[1];
double r3 = param[2];
double r4 = param[3];
double r5 = param[4];
double r6 = param[5];
double r7 = param[6];
double r8 = param[7];
double r9 = param[8];

double t4 = r2-3.338299811;
double t5 = t2*t4;
double t6 = r1*t1;
double t7 = t1*t1;
double t8 = t2*t2;
double t13 = r1*t7;
double t14 = r2*t8;
double t15 = r3*t1*t2;
double t9 = t5+t6-t13-t14-t15+1.151035476;
double t10 = exp(t9);
double t11 = t10+1.0;
double t12 = 1.0/t11;
double t16 = 1.0/(t11*t11);
double t17 = t12*1.8E1;
double t19 = t16*9.0;
double t18 = t17-t19+1.0;
double t20 = 1.0/t18;
double t21 = t12*t20*2.18E2;
double t22 = t16*(1.0/1.0E1);
double t23 = t22+9.0/1.0E1;
double t24 = t17-1.8E1;
double t25 = 1.0/(t18*t18);
double t26 = t23*t24*t25*1.09E3;
double t27 = t21+t26;
double t28 = fabs(t27);
double t29 = r4-3.338299811;
double t30 = t1*t29;
double t31 = r5*t2;
double t36 = r4*t7;
double t37 = r5*t8;
double t38 = r6*t1*t2;
double t32 = t30+t31-t36-t37-t38+1.151035476;
double t33 = exp(t32);
double t34 = t33+1.0;
double t35 = 1.0/t34;
double t39 = 1.0/(t34*t34);
double t40 = t35*1.8E1;
double t64 = t39*9.0;
double t41 = t40-t64+1.0;
double t65 = 1.0/t41;
double t66 = t35*t65*1.962E3;
double t67 = t39*(9.0/1.0E1);
double t68 = t67+1.0/1.0E1;
double t69 = t40-1.8E1;
double t70 = 1.0/(t41*t41);
double t71 = t68*t69*t70*1.09E3;
double t72 = t66+t71;
double t42 = fabs(t72);
double t43 = r7+3.338299811;
double t44 = t1*t43;
double t45 = r8+3.338299811;
double t46 = t2*t45;
double t51 = r7*t7;
double t52 = r8*t8;
double t53 = r9*t1*t2;
double t47 = t44+t46-t51-t52-t53-2.187264336;
double t48 = exp(t47);
double t49 = t48+1.0;
double t50 = 1.0/t49;
double t54 = 1.0/(t49*t49);
double t55 = t50*1.8E1;
double t74 = t54*9.0;
double t56 = t55-t74+1.0;
double t75 = 1.0/t56;
double t76 = t55-1.8E1;
double t77 = 1.0/(t56*t56);
double t86 = t50*t75*1.962E3;
double t87 = t54*(9.0/1.0E1);
double t88 = t87+1.0/1.0E1;
double t89 = t76*t77*t88*1.09E3;
double t90 = t86+t89;
double t57 = fabs(t90);
double t58 = t12*t20*1.962E3;
double t59 = t16*(9.0/1.0E1);
double t60 = t59+1.0/1.0E1;
double t61 = t24*t25*t60*1.09E3;
double t62 = t58+t61;
double t63 = fabs(t62);
double t73 = t42*t42;
double t100 = t50*t75*2.18E2;
double t101 = t54*(1.0/1.0E1);
double t102 = t101+9.0/1.0E1;
double t103 = t76*t77*t102*1.09E3;
double t104 = t100+t103;
double t78 = fabs(t104);
double t79 = t35*t65*2.18E2;
double t80 = t39*(1.0/1.0E1);
double t81 = t80+9.0/1.0E1;
double t82 = t69*t70*t81*1.09E3;
double t83 = t79+t82;
double t84 = fabs(t83);
double t85 = t63*t63;
double t91 = t57*t57;
double t92 = t28*t28;
double t93 = t73+t91+t92;
double t94 = 1.0/sqrt(t93);
double t95 = t27*t94;
double t96 = t84*t84;
double t97 = t85+t91+t96;
double t98 = 1.0/sqrt(t97);
double t99 = t72*t94;
double t105 = t78*t78;
double t106 = t73+t85+t105;
double t107 = 1.0/sqrt(t106);
double t116 = t62*t98;
double t108 = t95-t116;
double t109 = t90*t94;
double t118 = t62*t107;
double t110 = t95-t118;
double t117 = t90*t98;
double t111 = t109-t117;
double t123 = t72*t107;
double t112 = t99-t123;
double t122 = t83*t98;
double t113 = t99-t122;
double t115 = t104*t107;
double t114 = t109-t115;
double t119 = t110*t111;
double t216 = t108*t114;
double t120 = t119-t216;
double t121 = fabs(t120);
double t124 = t111*t112;
double t218 = (t99-t122)*(t109-t115);
double t219 = -t124+t218;
double t125 = fabs(t219);
double t126 = t108*t112;
double t214 = t110*t113;
double t127 = t126-t214;
double t128 = fabs(t127);
double t129 = r4*t1*2.0;
double t130 = r6*t2;
double t131 = -r4+t129+t130+3.338299811;
double t138 = r7*t1*2.0;
double t139 = r9*t2;
double t132 = r7-t138-t139+3.338299811;
double t133 = r1*t1*2.0;
double t134 = r3*t2;
double t135 = -r1+t133+t134;
double t136 = t10*t16*t62*t135;
double t137 = t33*t39*t72*t131;
double t146 = t48*t54*t104*t132;
double t140 = t136+t137-t146;
double t141 = t33*t39*t83*t131;
double t143 = t48*t54*t90*t132;
double t142 = t136+t141-t143;
double t144 = t10*t16*t27*t135;
double t145 = t137-t143+t144;
double t147 = r2*t2*2.0;
double t148 = r3*t1;
double t149 = -r2+t147+t148+3.338299811;
double t156 = r8*t2*2.0;
double t157 = r9*t1;
double t150 = r8-t156-t157+3.338299811;
double t151 = r5*t2*2.0;
double t152 = r6*t1;
double t153 = -r5+t151+t152;
double t154 = t33*t39*t72*t153;
double t155 = t10*t16*t62*t149;
double t164 = t48*t54*t104*t150;
double t158 = t154+t155-t164;
double t159 = t10*t16*t27*t149;
double t161 = t48*t54*t90*t150;
double t160 = t154+t159-t161;
double t162 = t33*t39*t83*t153;
double t163 = t155-t161+t162;
double t165 = t27*t145;
double t166 = t62*t142;
double t167 = t62*t140;
double t168 = t165+t166+t167;
double t169 = t104*t140;
double t170 = t90*t142;
double t171 = t90*t145;
double t172 = t169+t170+t171;
double t173 = t83*t142;
double t174 = t72*t140;
double t175 = t83*t163;
double t176 = t72*t160;
double t177 = t72*t158;
double t178 = t175+t176+t177;
double t179 = t104*t158;
double t180 = t90*t160;
double t181 = t90*t163;
double t182 = t179+t180+t181;
double t183 = t27*t160;
double t184 = t62*t158;
double t185 = 1.0/AUp1;
double t186 = t20*t23*1.09E3;
double t187 = t65*t68*1.09E3;
double t188 = t75*t88*1.09E3;
double t189 = t186+t187+t188+1.9E1;
double t190 = t185*t189;
double t191 = t190-1.0;
double t192 = t1-t7;
double t193 = 1.0/(t11*t11*t11);
double t194 = 1.0/AUp2;
double t195 = t20*t60*1.09E3;
double t196 = t65*t81*1.09E3;
double t197 = t188+t195+t196+1.9E1;
double t198 = t194*t197;
double t199 = t198-1.0;
double t200 = t10*t16*t192*1.8E1;
double t207 = t10*t192*t193*1.8E1;
double t201 = t200-t207;
double t202 = 1.0/AUp3;
double t203 = t75*t102*1.09E3;
double t204 = t187+t195+t203+1.9E1;
double t205 = t202*t204;
double t206 = t205-1.0;
double t208 = t25*t60*t201*1.09E3;
double t209 = t208-t10*t20*t192*t193*1.962E3;
double t210 = t72*t145;
double t211 = t173+t174+t210;
double t212 = t62*t163;
double t213 = t183+t184+t212;
double t215 = t90*t94*t127;
double t217 = t72*t94*t120;
double t220 = t121*t121;
double t221 = t125*t125;
double t222 = t128*t128;
double t223 = t220+t221+t222;
double t224 = 1.0/sqrt(t223);
double t225 = t10*t16*t20*t192*2.18E2;
double t226 = 1.0/(t18*t18*t18);
double t227 = t10*t16*t23*t25*t192*1.962E4;
double t228 = t10*t24*t25*t192*t193*2.18E2;
double t232 = t12*t25*t201*2.18E2;
double t233 = t23*t24*t201*t226*2.18E3;
double t229 = t225+t227+t228-t232-t233;
double t230 = (t27/fabs(t27));
double t231 = 1.0/pow(t93,3.0/2.0);
double t234 = (t62/fabs(t62));
double t235 = t10*t16*t20*t192*1.962E3;
double t236 = t10*t16*t25*t60*t192*1.962E4;
double t237 = t10*t24*t25*t192*t193*1.962E3;
double t240 = t12*t25*t201*1.962E3;
double t241 = t24*t60*t201*t226*2.18E3;
double t238 = t235+t236+t237-t240-t241;
double t239 = 1.0/pow(t106,3.0/2.0);
double t242 = t28*t90*t229*t230*t231;
double t243 = 1.0/pow(t97,3.0/2.0);
double t244 = t28*t72*t229*t230*t231;
double t257 = t63*t83*t234*t238*t243;
double t245 = t244-t257;
double t259 = t63*t72*t234*t238*t239;
double t246 = t244-t259;
double t247 = t94*t229;
double t258 = t63*t104*t234*t238*t239;
double t248 = t242-t258;
double t261 = t63*t90*t234*t238*t243;
double t249 = t242-t261;
double t250 = t62*t63*t234*t238*t243;
double t252 = t27*t28*t229*t230*t231;
double t267 = t98*t238;
double t251 = t247+t250-t252-t267;
double t253 = t62*t63*t234*t238*t239;
double t265 = t107*t238;
double t254 = t247-t252+t253-t265;
double t255 = t27*t94*t219;
double t256 = t215+t217+t255;
double t260 = t111*t246;
double t262 = t112*t249;
double t263 = t260+t262-t114*t245-t113*t248;
double t264 = t108*t246;
double t266 = t113*t254;
double t268 = t264+t266-t110*t245-t112*t251;
double t269 = t108*t248;
double t270 = t111*t254;
double t271 = t269+t270-t110*t249-t114*t251;
double t272 = t33*t39*t131*t211;
double t273 = t10*t16*t135*t168;
double t291 = t48*t54*t132*t172;
double t274 = t272+t273-t291;
double t275 = t10*t16*t149*t213;
double t276 = t33*t39*t153*t178;
double t292 = t48*t54*t150*t182;
double t277 = t275+t276-t292;
double t278 = t274*t277;
double t279 = t10*t16*t149*t168;
double t280 = t33*t39*t153*t211;
double t293 = t48*t54*t150*t172;
double t281 = t279+t280-t293;
double t282 = t33*t39*t131*t178;
double t283 = t10*t16*t135*t213;
double t294 = t48*t54*t132*t182;
double t284 = t282+t283-t294;
double t295 = t281*t284;
double t285 = t278-t295;
double t286 = sqrt(t285);
double t287 = fabs(t191);
double t288 = fabs(t199);
double t289 = fabs(t206);
double t290 = fabs(t256);
double t296 = t287*t287;
double t297 = t288*t288;
double t298 = t289*t289;
double t299 = t296+t297+t298;
double t300 = t2*t4*2.0;
double t304 = r1*t7*2.0;
double t305 = r2*t8*2.0;
double t306 = r3*t1*t2*2.0;
double t301 = t133+t300-t304-t305-t306+2.302070952;
double t302 = exp(t301);
double t303 = t10*t16*t149*t238;
double t307 = t62*t149*t192*t193*t302*2.0;
double t312 = t10*t16*t62*t149*t192;
double t308 = t303+t307-t312;
double t309 = t10*t16*t149*t229;
double t310 = t27*t149*t192*t193*t302*2.0;
double t313 = t10*t16*t27*t149*t192;
double t311 = t309+t310-t313;
double t314 = t1*2.0;
double t315 = t314-1.0;
double t316 = t10*t16*t135*t238;
double t317 = t62*t135*t192*t193*t302*2.0;
double t322 = t10*t16*t62*t315;
double t323 = t10*t16*t62*t135*t192;
double t318 = t316+t317-t322-t323;
double t319 = t10*t16*t27*t315;
double t320 = t10*t16*t27*t135*t192;
double t324 = t10*t16*t135*t229;
double t325 = t27*t135*t192*t193*t302*2.0;
double t321 = t319+t320-t324-t325;
double t326 = t72*t311;
double t327 = t83*t308;
double t328 = t72*t308;
double t329 = t326+t327+t328;
double t330 = t90*t311;
double t331 = t104*t308;
double t332 = t90*t308;
double t333 = t330+t331+t332;
double t334 = t160*t229;
double t335 = t158*t238;
double t336 = t163*t238;
double t337 = t27*t311;
double t338 = t62*t308*2.0;
double t339 = t334+t335+t336+t337+t338;
double t340 = t83*t318;
double t341 = t72*t318;
double t342 = t145*t229;
double t343 = t142*t238;
double t344 = t140*t238;
double t345 = t104*t318;
double t346 = t90*t318;
double t347 = t345+t346-t90*t321;
double t348 = t340+t341-t72*t321;
double t349 = t62*t318*2.0;
double t350 = t342+t343+t344+t349-t27*t321;
double t0 = -t286*t299*(beta*t224*((t256/fabs(t256)))*(-t27*t94*t263-t72*t94*t271+t90*t94*t268-t94*t219*t229+t28*t72*t120*t229*t230*t231+t28*t90*t127*t229*t230*t231+t27*t28*t219*t229*t230*t231)+beta*1.0/pow(t223,3.0/2.0)*t290*(t121*t271*((t120/fabs(t120)))*2.0-t128*t268*((t127/fabs(t127)))*2.0+t125*t263*((t219/fabs(t219)))*2.0)*(1.0/2.0))-t286*(beta*t224*fabs(t215+t217-t27*t94*(t124-t113*t114))-1.0)*(t185*t287*((t191/fabs(t191)))*(t23*t25*t201*1.09E3-t10*t20*t192*t193*2.18E2)*2.0+t194*t209*t288*((t199/fabs(t199)))*2.0+t202*t209*t289*((t206/fabs(t206)))*2.0)-1.0/sqrt(t285)*t299*(beta*t224*t290-1.0)*(t284*(t10*t16*t149*t350+t33*t39*t153*t348-t48*t54*t150*t347-t10*t16*t149*t168*t192+t149*t168*t192*t193*t302*2.0)-t274*(t10*t16*t149*t339+t33*t39*t153*t329-t48*t54*t150*t333-t10*t16*t149*t192*t213+t149*t192*t193*t213*t302*2.0)+t277*(t10*t16*t168*t315-t10*t16*t135*t350-t33*t39*t131*t348+t48*t54*t132*t347+t10*t16*t135*t168*t192-t135*t168*t192*t193*t302*2.0)+t281*(t10*t16*t135*t339+t33*t39*t131*t329-t10*t16*t213*t315-t48*t54*t132*t333-t10*t16*t135*t192*t213+t135*t192*t193*t213*t302*2.0))*(1.0/2.0);

return t0;
}

#endif