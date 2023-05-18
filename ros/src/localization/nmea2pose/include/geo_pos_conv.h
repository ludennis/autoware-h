/*
 * Copyright (c) 2014, Nagoya University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Autoware nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Based on file:
 *  https://github.com/autowarefoundation/autoware/blob/release/
 *  1.8.1/ros/src/computing/perception/localization/lib/gnss/
 *  include/gnss/geo_pos_conv.hpp
 */

#ifndef __ITRI_GEO_POS_CONV_H__
#define __ITRI_GEO_POS_CONV_H__

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

namespace ITRI
{
    struct GPSReference
    {
        double latitude;
        double longitude;
        double altitude;
    };

    class GeoPosConv
    {
        private:
            GPSReference mOrigin; //the gps of origin of map
            double mLatitude;
            double mLongitude;
            double mAltitude;
            double mX;
            double mY;
            double mZ;

        public:
            double latitude();
            double longitude();
            double altitude();
            double x();
            double y();
            double z();

            void SetGPSReference(int num);

            void SetLLHNmeaDegrees(double latd,double lond, double h);
            void LLHToXYZ(double lat, double lon, double ele);

            void ConvLLH2XYZ(double lat, double lon, double h);
            void ConvXYZ2LLH(double X, double Y, double Z);
    };
}

#endif // __ITRI_GEO_POS_CONV_H__
