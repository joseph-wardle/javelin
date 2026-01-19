
// Declaration of all textures

uniform sampler1D ocio_reach_m_table_0Sampler;
uniform sampler1D ocio_gamut_cusp_table_0Sampler;

// Declaration of all helper methods

float ocio_reach_m_table_0_sample(float h)
{
  float i_base = floor(h);
  float i_lo = i_base + 1;
  float i_hi = i_lo + 1;
  float lo = texture(ocio_reach_m_table_0Sampler, (i_lo + 0.5) / 362).r;
  float hi = texture(ocio_reach_m_table_0Sampler, (i_hi + 0.5) / 362).r;
  float t = h - i_base;
  return mix(lo, hi, t);
}
float ocio_tonescale_fwd0(float J)
{
  float A = 0.0323680267 * pow(abs(J) * 0.00999999978, 0.879464149);
  float Y = pow(( 27.1299992 * A) / (1.0f - A), 2.3809523809523809);
  float f = 1.04710376 * pow(Y / (Y + 0.73009213709383403), 1.14999998);
  float Y_ts = max(0.0, f * f / (f + 0.0399999991));
  float F_L_Y = pow(0.79370057210326195 * Y_ts, 0.42);
  float J_ts = 100. * pow((F_L_Y / ( 27.1299992 + F_L_Y)) * 30.8946857, 1.13705599);
  return sign(J) * J_ts;
}
float ocio_toe_fwd0(float x, float limit, float k1_in, float k2_in)
{
  float k2 = max(k2_in, 0.001);
  float k1 = sqrt(k1_in * k1_in + k2 * k2);
  float k3 = (limit + k1) / (limit + k2);
  return (x > limit) ? x : 0.5 * (k3 * x - k1 + sqrt((k3 * x - k1) * (k3 * x - k1) + 4.0 * k2 * k3 * x));
}
const float ocio_gamut_cusp_table_0_hues_array[362] = float[362](-1.01858521, 0., 0.999435902, 1.9988718, 2.9983077, 3.99774361, 4.99717951, 5.99661541, 6.99605131, 7.99548721, 8.99492264, 9.99435902, 10.9937954, 11.9932308, 12.9926662, 13.9921026, 14.991539, 15.9909744, 16.9904099, 17.9898453, 18.9892826, 19.988718, 20.9881535, 21.9875908, 22.9870262, 23.9864616, 24.9858971, 26.2510033, 27.2489777, 28.2469521, 29.2449265, 30.2429008, 31.2408752, 32.2388496, 33.236824, 34.2347984, 35.2327728, 36.2307434, 37.2287216, 38.2266922, 39.2246704, 40.222641, 41.2206154, 42.2185898, 43.2165642, 44.2145386, 45.212513, 46.2104874, 47.2084618, 48.2064362, 49.2044106, 50.2023849, 51.2003593, 52.1983337, 53.1963043, 54.1942825, 55.1922531, 56.1902313, 57.1882019, 58.1861801, 59.1841507, 60.1821251, 61.1800995, 62.1780739, 63.1760483, 64.1740265, 65.1719971, 66.1699677, 67.1679459, 68.1659241, 69.1638947, 70.1618652, 71.1598434, 72.1578217, 73.1557922, 74.1537628, 75.151741, 76.1497192, 77.1476898, 78.1456604, 79.1436386, 80.1416092, 81.1395874, 82.137558, 83.1355286, 84.1335068, 85.131485, 86.1294556, 87.1274261, 88.1254044, 89.1233826, 90.1213531, 91.1193237, 92.1172943, 93.1152802, 94.1132507, 95.1112213, 96.1091919, 97.1071777, 98.1051483, 99.1031189, 100.101089, 101.099075, 102.097046, 103.095016, 104.092987, 105.090973, 106.088936, 106.548775, 107.571564, 108.594353, 109.617142, 110.639931, 111.66272, 112.685509, 113.708298, 114.731087, 115.753876, 116.776665, 117.799454, 118.822243, 119.845032, 120.867821, 121.89061, 122.913399, 123.936188, 124.958977, 125.981766, 127.004555, 128.027344, 129.05014, 130.072922, 131.095703, 132.1185, 133.141296, 134.164078, 135.186859, 136.209656, 137.232452, 138.255234, 139.278015, 140.300812, 141.287491, 142.27417, 143.260849, 144.247528, 145.234207, 146.220886, 147.207565, 148.207993, 149.20842, 150.208847, 151.209274, 152.209702, 153.210129, 154.210556, 155.210983, 156.211411, 157.211853, 158.21228, 159.212708, 160.213135, 161.213562, 162.213989, 163.214417, 164.214844, 165.215271, 166.215698, 167.216125, 168.216553, 169.21698, 170.217407, 171.217834, 172.218262, 173.218689, 174.219116, 175.219543, 176.219971, 177.220398, 178.220825, 179.221252, 180.22168, 181.222107, 182.222549, 183.222977, 184.223404, 185.223831, 186.224258, 187.224686, 188.225113, 189.22554, 190.225967, 191.226395, 192.226822, 193.12616, 194.025482, 194.92482, 195.824158, 196.830383, 197.836609, 198.842819, 199.849045, 200.85527, 201.861496, 202.867706, 203.873932, 204.880157, 205.886383, 206.892609, 207.898819, 208.905045, 209.91127, 210.917496, 211.923706, 212.929932, 213.936157, 214.942383, 215.948608, 216.954819, 217.961044, 218.96727, 219.973495, 220.979706, 221.985931, 222.992157, 223.998383, 225.004608, 226.010834, 227.017044, 228.02327, 229.029495, 230.035706, 231.041931, 232.048157, 233.054382, 234.060608, 235.066833, 236.073044, 237.079269, 238.085495, 239.091705, 240.097931, 241.104156, 242.110382, 243.116608, 244.122833, 245.129044, 246.135269, 247.141495, 248.147705, 249.153931, 250.160156, 251.166382, 252.172607, 253.178833, 254.185043, 255.191269, 256.19751, 257.203705, 258.20993, 259.216156, 260.222382, 261.228607, 262.234833, 263.241058, 264.247253, 265.253479, 266.259705, 267.26593, 268.272156, 269.270721, 270.269287, 271.267853, 272.266418, 273.264984, 274.26355, 275.262115, 276.260681, 277.249512, 278.238342, 279.227203, 280.216034, 281.204865, 282.193695, 283.182556, 284.171387, 285.160217, 286.149048, 287.137878, 288.12674, 289.11557, 290.104401, 291.093231, 292.082092, 293.070923, 294.059753, 295.048584, 296.037445, 297.026276, 298.015106, 299.003937, 299.992798, 300.981628, 301.970459, 302.95929, 303.94812, 304.936981, 305.925812, 306.914642, 307.903473, 308.892334, 309.881165, 310.869995, 311.858826, 312.847656, 313.836517, 314.825348, 315.814178, 316.803009, 317.79187, 318.780701, 319.769531, 320.758362, 321.747192, 322.736053, 323.724884, 324.713715, 325.702545, 326.691406, 327.680237, 328.669067, 329.657898, 330.646759, 331.63559, 332.62442, 333.628967, 334.633514, 335.638062, 336.642609, 337.647186, 338.651733, 339.656281, 340.660828, 341.665375, 342.68396, 343.702545, 344.72113, 345.739746, 346.758331, 347.776917, 348.795502, 349.814087, 350.832703, 351.851288, 352.869873, 353.888458, 354.907043, 355.925629, 356.944244, 357.96283, 358.981415, 360.);
vec3 ocio_gamut_cusp_table_0_sample(float h)
{
  int i = int(h) + 1;
  int i_lo = int(max(float(0), float(i + 0)));
  int i_hi = int(min(float(361), float(i + 2)));
  while (i_lo + 1 < i_hi)
  {
    float hcur = ocio_gamut_cusp_table_0_hues_array[i];
    if (h > hcur)
    {
      i_lo = i;
    }
    else
    {
      i_hi = i;
    }
    i = (i_lo + i_hi) / 2;
  }
  vec3 lo = texture(ocio_gamut_cusp_table_0Sampler, (i_hi - 1 + 0.5) / 362).rgb;
  vec3 hi = texture(ocio_gamut_cusp_table_0Sampler, (i_hi + 0.5) / 362).rgb;
  float t = (h - ocio_gamut_cusp_table_0_hues_array[i_hi - 1]) / (ocio_gamut_cusp_table_0_hues_array[i_hi] - ocio_gamut_cusp_table_0_hues_array[i_hi - 1]);
  return mix(lo, hi, t);
}
float ocio_get_focus_gain0(float J, float cuspJ)
{
  float thr = mix(cuspJ, 100.000000, 0.300000);
  if (J > thr)
  {
    float gain = ( 100. - thr) / max(0.0001, 100. - J);
    gain = log(gain)/log(10.0);
    return gain * gain + 1.0;
  }
  else
  {
    return 1.0;
  }
}
float ocio_solve_J_intersect0(float J, float M, float focusJ, float slope_gain)
{
  float M_scaled = M / slope_gain;
  float a = M_scaled / focusJ;
  if (J < focusJ)
  {
    float b = 1.0 - M_scaled;
    float c = -J;
    float det =  b * b - 4.f * a * c;
    float root =  sqrt(det);
    return -2.0 * c / (b + root);
  }
  else
  {
    float b = - (1.0 + M_scaled + 100. * a);
    float c = 100. * M_scaled + J;
    float det =  b * b - 4.f * a * c;
    float root =  sqrt(det);
    return -2.0 * c / (b - root);
  }
}
float ocio_find_gamut_boundary_intersection0(vec2 JM_cusp, float gamma_top_inv, float gamma_bottom_inv, float J_intersect_source, float J_intersect_cusp, float slope)
{
  float M_boundary_lower = J_intersect_cusp * pow(J_intersect_source / J_intersect_cusp, gamma_bottom_inv) / (JM_cusp.r / JM_cusp.g - slope);
  float M_boundary_upper = JM_cusp.g * (100. - J_intersect_cusp) * pow((100. - J_intersect_source) / (100. - J_intersect_cusp), gamma_top_inv) / (slope * JM_cusp.g + 100. - JM_cusp.r);
  float smin = 0.0;
  {
    float a = M_boundary_lower;
    float b = M_boundary_upper;
    float s = 0.119999997 * JM_cusp.g;
    float h = max(s - abs(a - b), 0.0) / s;
    smin = min(a, b) - h * h * h * s * 0.16666666666666666;
  }
  return smin;
}
float ocio_remap_M_fwd0(float M, float gamut_boundary_M, float reach_boundary_M)
{
  float boundary_ratio = gamut_boundary_M / reach_boundary_M;
  float proportion = max(boundary_ratio, 0.75);
  float threshold = proportion * gamut_boundary_M;
  if (proportion >= 1.0f || M <= threshold)
  {
    return M;
  }
  float m_offset = M - threshold;
  float gamut_offset = gamut_boundary_M - threshold;
  float reach_offset = reach_boundary_M - threshold;
  float scale = reach_offset / ((reach_offset / gamut_offset) - 1.0f);
  float nd = m_offset / scale;
  return threshold + scale * nd / (1.0f + nd);
}
vec3 ocio_gamut_compress0(vec3 JMh, float Jx, vec3 JMGcusp, float reachMaxM)
{
  float J = JMh.r;
  float M = JMh.g;
  float h = JMh.b;
  if (M <= 0.0 || J > 100.)
  {
    return vec3(J, 0.0, h);
  }
  else
  {
    vec2 JMcusp = JMGcusp.rg;
    float focusJ = mix(JMcusp.r, 34.096539, min(1.0, 1.300000 - (JMcusp.r / 100.000000)));
    float slope_gain = 135. * ocio_get_focus_gain0(Jx, JMcusp.r);
    float J_intersect_source = ocio_solve_J_intersect0(JMh.r, JMh.g, focusJ, slope_gain);
    float gamut_slope = (J_intersect_source < focusJ) ? J_intersect_source : (100. - J_intersect_source);
    gamut_slope = gamut_slope * (J_intersect_source - focusJ) / (focusJ * slope_gain);
    float gamma_top_inv = JMGcusp.b;
    float gamma_bottom_inv = 0.877192974;
    float J_intersect_cusp = ocio_solve_J_intersect0(JMcusp.r, JMcusp.g, focusJ, slope_gain);
    float gamutBoundaryM = ocio_find_gamut_boundary_intersection0(JMcusp, gamma_top_inv, gamma_bottom_inv, J_intersect_source, J_intersect_cusp, gamut_slope);
    if (gamutBoundaryM <= 0.0)
    {
      return vec3(J, 0.0, h);
    }
    float reachBoundaryM = 100. * pow(J_intersect_source / 100.,  0.879464149);
    reachBoundaryM = reachBoundaryM / ((100. / reachMaxM) - gamut_slope);
    float remapped_M = ocio_remap_M_fwd0(M, gamutBoundaryM, reachBoundaryM);
    float remapped_J = J_intersect_source + remapped_M * gamut_slope;
    return vec3(remapped_J, remapped_M, h);
  }
}

// Declaration of the OCIO shader function

vec4 OCIODisplay(vec4 inPixel)
{
  vec4 outColor = inPixel;
  
  // Add Range processing
  
  {
    outColor.rgb = max(vec3(0., 0., 0.), outColor.rgb);
    outColor.rgb = min(vec3(1024., 1024., 1024.), outColor.rgb);
  }
  
  // Add Matrix processing
  
  {
    vec4 res = vec4(outColor.rgb.r, outColor.rgb.g, outColor.rgb.b, outColor.a);
    vec4 tmp = res;
    res = mat4(0.69545224135745176, 0.044794563372037632, -0.0055258825581135443, 0., 0.14067869647029416, 0.85967111845642163, 0.0040252103059786586, 0., 0.16386906217225403, 0.095534318171540358, 1.0015006722521349, 0., 0., 0., 0., 1.) * tmp;
    outColor.rgb = vec3(res.x, res.y, res.z);
    outColor.a = res.w;
  }
  
  // Add FixedFunction 'ACES_OutputTransform20 (Forward)' processing
  
  {
    
    // Add RGB to JMh
    
    vec3 JMh;
    vec3 Aab;
    {
      {
        vec3 lms = mat3(0.445181042, 0.123734146, 0.0117007261, 0.34964928, 0.613643706, 0.0280607939, -0.00112973212, 0.0563228019, 0.753939033) * outColor.rgb;
        vec3 F_L_v = pow(abs(lms), vec3(0.419999987, 0.419999987, 0.419999987));
        vec3 rgb_a = (sign(lms) * F_L_v) / ( 27.1299992 + F_L_v);
        Aab = mat3(20.25881, 15480., 1720., 10.129405, -16887.2734, 1720., 0.506470263, 1407.27271, -3440.) * rgb_a.rgb;
      }
      {
        if (Aab.r <= 0.0)
        {
          JMh.rgb = vec3(0., 0., 0.);
        }
        else
        {
          float J = 100. * pow(Aab.r, 1.13705599);
          float M = (J == 0.0) ? 0.0 : sqrt(Aab.g * Aab.g + Aab.b * Aab.b);
          float h = (Aab.g == 0.0) ? 0.0 : atan(Aab.b, Aab.g) * 57.29577951308238;
          h = h - floor(h / 360.0) * 360.0;
          h = (h < 0.0) ? h + 360.0 : h;
          JMh.rgb = vec3(J, M, h);
        }
      }
      outColor.rgb = JMh;
    }
    float h_rad = outColor.b * 0.0174532924;
    float cos_hr = cos(h_rad);
    float sin_hr = sin(h_rad);
    
    // Add ToneScale and ChromaCompress (fwd)
    
    float J_ts = ocio_tonescale_fwd0(outColor.r);
    // Sample tables (fwd)
    float reachMaxM = ocio_reach_m_table_0_sample(outColor.b);
    
    {
      float J = outColor.r;
      float M = outColor.g;
      float h = outColor.b;
      float M_cp = M;
      if (M != 0.0)
      {
        float nJ = J_ts / 100.;
        float snJ = max(0.0, 1.0 - nJ);
        float Mnorm;
        {
          float cos_hr2 = 2.0 * cos_hr * cos_hr - 1.0;
          float sin_hr2 = 2.0 * cos_hr * sin_hr;
          float cos_hr3 = 4.0 * cos_hr * cos_hr * cos_hr - 3.0 * cos_hr;
          float sin_hr3 = 3.0 * sin_hr - 4.0 * sin_hr * sin_hr * sin_hr;
          vec3 cosines = vec3(cos_hr, cos_hr2, cos_hr3);
          vec3 cosine_weights = vec3(11.341321604032515, 16.469863649185896, 7.8842182208776475);
          vec3 sines = vec3(sin_hr, sin_hr2, sin_hr3);
          vec3 sine_weights = vec3(14.665187919584513, -6.3725780354404442, 9.1941277054452897);
          Mnorm = dot(cosines, cosine_weights) + dot(sines, sine_weights) + 77.133051547393805;
        }
        float limit = pow(nJ, 0.879464149) * reachMaxM / Mnorm;
        M_cp = M * pow(J_ts / J, 0.879464149);
        M_cp = M_cp / Mnorm;
        M_cp = limit - ocio_toe_fwd0(limit - M_cp, limit - 0.001, snJ * 1.29999995, sqrt(nJ * nJ + 0.00499999989));
        M_cp = ocio_toe_fwd0(M_cp, limit, nJ * 2.4000001, snJ);
        M_cp = M_cp * Mnorm;
      }
      outColor.rgb = vec3(J_ts, M_cp, h);
    }
    
    // Add GamutCompress (fwd)
    
    {
      vec3 JMGcusp = ocio_gamut_cusp_table_0_sample(outColor.b);
      outColor.rgb = ocio_gamut_compress0(outColor.rgb, outColor.r, JMGcusp, reachMaxM);
    }
    
    // Add JMh to RGB
    
    {
      vec3 JMh = outColor.rgb;
      vec3 Aab;
      {
        Aab.r = pow(JMh.r * 0.00999999978, 0.879464149);
        Aab.g = JMh.g * cos_hr;
        Aab.b = JMh.g * sin_hr;
      }
      {
        vec3 rgb_a = mat3(0.0323680267, 0.0323680267, 0.0323680267, 2.07657631e-05, -4.10250432e-05, -1.01296409e-05, 1.3260621e-05, -1.20174373e-05, -0.000290076074) * Aab.rgb;
        vec3 rgb_a_lim = min( abs(rgb_a), vec3(0.99000001, 0.99000001, 0.99000001) );
        vec3 lms = sign(rgb_a) * pow( 27.1299992 * rgb_a_lim / (1.0f - rgb_a_lim), vec3(2.38095236, 2.38095236, 2.38095236));
        JMh.rgb = mat3(7.45048571, -1.4750675, 0.0106288502, -6.1301837, 3.11835742, -0.31857267, -0.0603808537, -0.383369029, 1.56786489) * lms;
      }
      outColor.rgb = JMh;
    }
  }
  
  // Add Range processing
  
  {
    outColor.rgb = max(vec3(0., 0., 0.), outColor.rgb);
    outColor.rgb = min(vec3(1., 1., 1.), outColor.rgb);
  }
  
  // Add Gamma 'monCurveMirrorRev' processing
  
  {
    vec4 breakPnt = vec4(0.00303993467, 0.00303993467, 0.00303993467, 1.);
    vec4 slope = vec4(12.9232101, 12.9232101, 12.9232101, 1.);
    vec4 scale = vec4(1.05499995, 1.05499995, 1.05499995, 1.00000095);
    vec4 offset = vec4(0.0549999997, 0.0549999997, 0.0549999997, 9.99999997e-07);
    vec4 gamma = vec4(0.416666657, 0.416666657, 0.416666657, 0.999998987);
    vec4 signcol = sign(outColor);;
    outColor = abs( outColor );
    vec4 isAboveBreak = vec4(greaterThan( outColor, breakPnt));
    vec4 linSeg = outColor * slope;
    vec4 powSeg = pow( outColor, gamma ) * scale - offset;
    vec4 res = isAboveBreak * powSeg + ( vec4(1., 1., 1., 1.) - isAboveBreak ) * linSeg;
    res = signcol * res;
    outColor.rgb = vec3(res.x, res.y, res.z);
    outColor.a = res.w;
  }

  return outColor;
}
