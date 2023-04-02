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
 * @file Killswitch.cpp
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

#include "Killswitch.h"
#include <fastcdr/Cdr.h>

#include <fastcdr/exceptions/BadParamException.h>
using namespace eprosima::fastcdr::exception;

#include <utility>

Killswitch::Killswitch()
{
    // m_kills com.eprosima.idl.parser.typecode.PrimitiveTypeCode@4a87761d
    m_kills = 0.0;

}

Killswitch::~Killswitch()
{
}

Killswitch::Killswitch(
        const Killswitch& x)
{
    m_kills = x.m_kills;
}

Killswitch::Killswitch(
        Killswitch&& x)
{
    m_kills = x.m_kills;
}

Killswitch& Killswitch::operator =(
        const Killswitch& x)
{

    m_kills = x.m_kills;

    return *this;
}

Killswitch& Killswitch::operator =(
        Killswitch&& x)
{

    m_kills = x.m_kills;

    return *this;
}

bool Killswitch::operator ==(
        const Killswitch& x) const
{

    return (m_kills == x.m_kills);
}

bool Killswitch::operator !=(
        const Killswitch& x) const
{
    return !(*this == x);
}

size_t Killswitch::getMaxCdrSerializedSize(
        size_t current_alignment)
{
    size_t initial_alignment = current_alignment;


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    return current_alignment - initial_alignment;
}

size_t Killswitch::getCdrSerializedSize(
        const Killswitch& data,
        size_t current_alignment)
{
    (void)data;
    size_t initial_alignment = current_alignment;


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    return current_alignment - initial_alignment;
}

void Killswitch::serialize(
        eprosima::fastcdr::Cdr& scdr) const
{

    scdr << m_kills;

}

void Killswitch::deserialize(
        eprosima::fastcdr::Cdr& dcdr)
{

    dcdr >> m_kills;
}

/*!
 * @brief This function sets a value in member kills
 * @param _kills New value for member kills
 */
void Killswitch::kills(
        float _kills)
{
    m_kills = _kills;
}

/*!
 * @brief This function returns the value of member kills
 * @return Value of member kills
 */
float Killswitch::kills() const
{
    return m_kills;
}

/*!
 * @brief This function returns a reference to member kills
 * @return Reference to member kills
 */
float& Killswitch::kills()
{
    return m_kills;
}


size_t Killswitch::getKeyMaxCdrSerializedSize(
        size_t current_alignment)
{
    size_t current_align = current_alignment;



    return current_align;
}

bool Killswitch::isKeyDefined()
{
    return false;
}

void Killswitch::serializeKey(
        eprosima::fastcdr::Cdr& scdr) const
{
    (void) scdr;
     
}