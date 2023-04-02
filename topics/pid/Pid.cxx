// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*!
 * @file Pid.cpp
 * This source file contains the definition of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifdef _WIN32
// Remove linker warning LNK4221 on Visual Studio
namespace {
char dummy;
}  // namespace
#endif  // _WIN32

#include "Pid.h"
#include <fastcdr/Cdr.h>

#include <fastcdr/exceptions/BadParamException.h>
using namespace eprosima::fastcdr::exception;

#include <utility>

Pid::Pid()
{
    // m_v_1 com.eprosima.idl.parser.typecode.PrimitiveTypeCode@6950e31
    m_v_1 = 0.0;
    // m_v_2 com.eprosima.idl.parser.typecode.PrimitiveTypeCode@b7dd107
    m_v_2 = 0.0;
    // m_v_3 com.eprosima.idl.parser.typecode.PrimitiveTypeCode@42eca56e
    m_v_3 = 0.0;
    // m_v_4 com.eprosima.idl.parser.typecode.PrimitiveTypeCode@52f759d7
    m_v_4 = 0.0;

}

Pid::~Pid()
{




}

Pid::Pid(
        const Pid& x)
{
    m_v_1 = x.m_v_1;
    m_v_2 = x.m_v_2;
    m_v_3 = x.m_v_3;
    m_v_4 = x.m_v_4;
}

Pid::Pid(
        Pid&& x)
{
    m_v_1 = x.m_v_1;
    m_v_2 = x.m_v_2;
    m_v_3 = x.m_v_3;
    m_v_4 = x.m_v_4;
}

Pid& Pid::operator =(
        const Pid& x)
{

    m_v_1 = x.m_v_1;
    m_v_2 = x.m_v_2;
    m_v_3 = x.m_v_3;
    m_v_4 = x.m_v_4;

    return *this;
}

Pid& Pid::operator =(
        Pid&& x)
{

    m_v_1 = x.m_v_1;
    m_v_2 = x.m_v_2;
    m_v_3 = x.m_v_3;
    m_v_4 = x.m_v_4;

    return *this;
}

bool Pid::operator ==(
        const Pid& x) const
{

    return (m_v_1 == x.m_v_1 && m_v_2 == x.m_v_2 && m_v_3 == x.m_v_3 && m_v_4 == x.m_v_4);
}

bool Pid::operator !=(
        const Pid& x) const
{
    return !(*this == x);
}

size_t Pid::getMaxCdrSerializedSize(
        size_t current_alignment)
{
    size_t initial_alignment = current_alignment;


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);



    return current_alignment - initial_alignment;
}

size_t Pid::getCdrSerializedSize(
        const Pid& data,
        size_t current_alignment)
{
    (void)data;
    size_t initial_alignment = current_alignment;


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);



    return current_alignment - initial_alignment;
}

void Pid::serialize(
        eprosima::fastcdr::Cdr& scdr) const
{

    scdr << m_v_1;
    scdr << m_v_2;
    scdr << m_v_3;
    scdr << m_v_4;

}

void Pid::deserialize(
        eprosima::fastcdr::Cdr& dcdr)
{

    dcdr >> m_v_1;
    dcdr >> m_v_2;
    dcdr >> m_v_3;
    dcdr >> m_v_4;
}

/*!
 * @brief This function sets a value in member v_1
 * @param _v_1 New value for member v_1
 */
void Pid::v_1(
        float _v_1)
{
    m_v_1 = _v_1;
}

/*!
 * @brief This function returns the value of member v_1
 * @return Value of member v_1
 */
float Pid::v_1() const
{
    return m_v_1;
}

/*!
 * @brief This function returns a reference to member v_1
 * @return Reference to member v_1
 */
float& Pid::v_1()
{
    return m_v_1;
}

/*!
 * @brief This function sets a value in member v_2
 * @param _v_2 New value for member v_2
 */
void Pid::v_2(
        float _v_2)
{
    m_v_2 = _v_2;
}

/*!
 * @brief This function returns the value of member v_2
 * @return Value of member v_2
 */
float Pid::v_2() const
{
    return m_v_2;
}

/*!
 * @brief This function returns a reference to member v_2
 * @return Reference to member v_2
 */
float& Pid::v_2()
{
    return m_v_2;
}

/*!
 * @brief This function sets a value in member v_3
 * @param _v_3 New value for member v_3
 */
void Pid::v_3(
        float _v_3)
{
    m_v_3 = _v_3;
}

/*!
 * @brief This function returns the value of member v_3
 * @return Value of member v_3
 */
float Pid::v_3() const
{
    return m_v_3;
}

/*!
 * @brief This function returns a reference to member v_3
 * @return Reference to member v_3
 */
float& Pid::v_3()
{
    return m_v_3;
}

/*!
 * @brief This function sets a value in member v_4
 * @param _v_4 New value for member v_4
 */
void Pid::v_4(
        float _v_4)
{
    m_v_4 = _v_4;
}

/*!
 * @brief This function returns the value of member v_4
 * @return Value of member v_4
 */
float Pid::v_4() const
{
    return m_v_4;
}

/*!
 * @brief This function returns a reference to member v_4
 * @return Reference to member v_4
 */
float& Pid::v_4()
{
    return m_v_4;
}


size_t Pid::getKeyMaxCdrSerializedSize(
        size_t current_alignment)
{
    size_t current_align = current_alignment;







    return current_align;
}

bool Pid::isKeyDefined()
{
    return false;
}

void Pid::serializeKey(
        eprosima::fastcdr::Cdr& scdr) const
{
    (void) scdr;
        
}
