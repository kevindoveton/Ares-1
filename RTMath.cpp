// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib-Arduino
//
//  Copyright (c) 2014-2015, richards-tech
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of
//  this software and associated documentation files (the "Software"), to deal in
//  the Software without restriction, including without limitation the rights to use,
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
//  Software, and to permit persons to whom the Software is furnished to do so,
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include "RTMath.h"
#include <Arduino.h>

RTVector3::RTVector3()
{
    zero();
}


RTVector3::RTVector3(RTFLOAT x, RTFLOAT y, RTFLOAT z)
{
    m_data[0] = x;
    m_data[1] = y;
    m_data[2] = z;
}


RTVector3& RTVector3::operator =(const RTVector3& vec)
{
    if (this == &vec)
        return *this;

    m_data[0] = vec.m_data[0];
    m_data[1] = vec.m_data[1];
    m_data[2] = vec.m_data[2];

    return *this;
}


const RTVector3& RTVector3::operator +=(RTVector3& vec)
{
    for (int i = 0; i < 3; i++)
        m_data[i] += vec.m_data[i];
    return *this;
}

const RTVector3& RTVector3::operator -=(RTVector3& vec)
{
    for (int i = 0; i < 3; i++)
        m_data[i] -= vec.m_data[i];
    return *this;
}

void RTVector3::zero()
{
    for (int i = 0; i < 3; i++)
        m_data[i] = 0;
}
