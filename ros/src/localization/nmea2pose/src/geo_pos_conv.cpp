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
 *  1.8.1/ros/src/computing/perception/localization/lib/
 *  gnss/src/geo_pos_conv.cpp
 */

#include <geo_pos_conv.h>
#include <trace/utils.h>

#define TRACE_TAG "geo_pos_conv"

namespace ITRI
{
    const GPSReference GPS_REFERENCE[] = {
        {
            //map:itri campus
            24.775084704,
            121.045888961,
            146.593
        },
        {
            //map:artc campus
            24.0603805555556,
            120.384379444444,
            22.33888
        },
        {
            //map:hamura field
            35.7705231,
            139.3241414,
            152.503
        }
    };

    double GeoPosConv::x()
    {
        return mX;
    }

    double GeoPosConv::y()
    {
        return mY;
    }

    double GeoPosConv::z()
    {
        return mZ;
    }

    double GeoPosConv::latitude()
    {
        return mLatitude;
    }

    double GeoPosConv::longitude()
    {
        return mLongitude;
    }

    double GeoPosConv::altitude()
    {
        return mAltitude;
    }

    void GeoPosConv::SetGPSReference(int num)
    {
        TRACE_ASSERT(num >= 0);
        TRACE_ASSERT(num < sizeof(GPS_REFERENCE) / sizeof(GPSReference));
        mOrigin = GPS_REFERENCE[num];
    }

    void GeoPosConv::SetLLHNmeaDegrees(double latd, double lond, double h)
    {
        double lat, lad, lod, lon;
        // 1234.56 -> 12'34.56 -> 12+ 34.56/60

        lad = floor(latd / 100.);
        lat = latd - lad * 100.;
        lod = floor(lond / 100.);
        lon = lond - lod * 100.;

        double temp_lat = (lad + lat / 60.0);
        double temp_lon = (lod + lon / 60.0);
        ConvLLH2XYZ( temp_lat, temp_lon, h);
    }

    void GeoPosConv::LLHToXYZ(double lat, double lon, double ele)
    {
        ConvLLH2XYZ( lat, lon, ele);
    }

    void GeoPosConv::ConvLLH2XYZ(double lat, double lon, double h)
    {
        //type: WGS84
        GeographicLib::Geocentric earth(
            GeographicLib::Constants::WGS84_a(),
            GeographicLib::Constants::WGS84_f());
        GeographicLib::LocalCartesian referencPoint(
            mOrigin.latitude,
            mOrigin.longitude,
            mOrigin.altitude,
            earth);

        double locX, locY, locZ;
      //get relative position in the coordinate which origin is reference point.
        referencPoint.Forward(lat, lon, h, locX, locY, locZ);

        mX = locX;
        mY = locY;
        mZ = locZ;
    }

    void GeoPosConv::ConvXYZ2LLH(double X, double Y, double Z)
    {
        GeographicLib::Geocentric earth(
            GeographicLib::Constants::WGS84_a(),
            GeographicLib::Constants::WGS84_f());
        GeographicLib::LocalCartesian referencPoint(
            mOrigin.latitude,
            mOrigin.longitude,
            mOrigin.altitude,
            earth);

        double lat, lon, h;
        referencPoint.Reverse(X, Y, Z, lat, lon, h);

        mLatitude = lat;
        mLongitude = lon;
        mAltitude = h;
    }
}
