/*
 * This file auto-generated from script. Do not Edit!
 * Original : aero_startup/.templates/aero_hardware_interface/UnusedAngle2Stroke.hh
 * Original : aero_description/{my_robot}/headers/Angle2Stroke.hh
*/
#ifndef AERO_COMMON_UNUSED_ANGLE_TO_STROKE_H_
#define AERO_COMMON_UNUSED_ANGLE_TO_STROKE_H_

#include <vector>
#include <stdint.h>

namespace aero
{
  namespace common
  {

    inline void UnusedAngle2Stroke
    (std::vector<int16_t>& _strokes, const std::vector<bool> _angles)
    {
      if (!_angles[30]) _strokes[27] = 0x7fff;
      if (!_angles[31]) _strokes[26] = 0x7fff;
      if (!_angles[0]) _strokes[25] = 0x7fff;
      if (!_angles[2] && !_angles[1]) _strokes[24] = 0x7fff;
      if (!_angles[2] && !_angles[1]) _strokes[23] = 0x7fff;
      if (!_angles[14]) _strokes[22] = 0x7fff;
      if (!_angles[10]) _strokes[21] = 0x7fff;
      if (!_angles[11]) _strokes[20] = 0x7fff;
      if (!_angles[8] && !_angles[9]) _strokes[19] = 0x7fff;
      if (!_angles[8] && !_angles[9]) _strokes[18] = 0x7fff;
      if (!_angles[7]) _strokes[17] = 0x7fff;
      if (!_angles[6]) _strokes[16] = 0x7fff;
      if (!_angles[5]) _strokes[15] = 0x7fff;
      if (!_angles[4]) _strokes[14] = 0x7fff;
      if (!_angles[3]) _strokes[13] = 0x7fff;
      if (!_angles[29]) _strokes[12] = 0x7fff;
      if (!_angles[25]) _strokes[11] = 0x7fff;
      if (!_angles[26]) _strokes[10] = 0x7fff;
      if (!_angles[23] && !_angles[24]) _strokes[9] = 0x7fff;
      if (!_angles[23] && !_angles[24]) _strokes[8] = 0x7fff;
      if (!_angles[22]) _strokes[7] = 0x7fff;
      if (!_angles[21]) _strokes[6] = 0x7fff;
      if (!_angles[20]) _strokes[5] = 0x7fff;
      if (!_angles[19]) _strokes[4] = 0x7fff;
      if (!_angles[18]) _strokes[3] = 0x7fff;
      if (!_angles[17] && !_angles[16]) _strokes[2] = 0x7fff;
      if (!_angles[17] && !_angles[16]) _strokes[1] = 0x7fff;
      if (!_angles[15]) _strokes[0] = 0x7fff;
      // implement here
    };

  }
}

#endif
