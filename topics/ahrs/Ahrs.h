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
 * @file Ahrs.h
 * This header file contains the declaration of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifndef _FAST_DDS_GENERATED_AHRS_H_
#define _FAST_DDS_GENERATED_AHRS_H_


#include <fastrtps/utils/fixed_size_string.hpp>

#include <stdint.h>
#include <array>
#include <string>
#include <vector>
#include <map>
#include <bitset>

#if defined(_WIN32)
#if defined(EPROSIMA_USER_DLL_EXPORT)
#define eProsima_user_DllExport __declspec( dllexport )
#else
#define eProsima_user_DllExport
#endif  // EPROSIMA_USER_DLL_EXPORT
#else
#define eProsima_user_DllExport
#endif  // _WIN32

#if defined(_WIN32)
#if defined(EPROSIMA_USER_DLL_EXPORT)
#if defined(Ahrs_SOURCE)
#define Ahrs_DllAPI __declspec( dllexport )
#else
#define Ahrs_DllAPI __declspec( dllimport )
#endif // Ahrs_SOURCE
#else
#define Ahrs_DllAPI
#endif  // EPROSIMA_USER_DLL_EXPORT
#else
#define Ahrs_DllAPI
#endif // _WIN32

namespace eprosima {
namespace fastcdr {
class Cdr;
} // namespace fastcdr
} // namespace eprosima


/*!
 * @brief This class represents the structure Ahrs defined by the user in the IDL file.
 * @ingroup AHRS
 */
class Ahrs
{
public:

    /*!
     * @brief Default constructor.
     */
    eProsima_user_DllExport Ahrs();

    /*!
     * @brief Default destructor.
     */
    eProsima_user_DllExport ~Ahrs();

    /*!
     * @brief Copy constructor.
     * @param x Reference to the object Ahrs that will be copied.
     */
    eProsima_user_DllExport Ahrs(
            const Ahrs& x);

    /*!
     * @brief Move constructor.
     * @param x Reference to the object Ahrs that will be copied.
     */
    eProsima_user_DllExport Ahrs(
            Ahrs&& x);

    /*!
     * @brief Copy assignment.
     * @param x Reference to the object Ahrs that will be copied.
     */
    eProsima_user_DllExport Ahrs& operator =(
            const Ahrs& x);

    /*!
     * @brief Move assignment.
     * @param x Reference to the object Ahrs that will be copied.
     */
    eProsima_user_DllExport Ahrs& operator =(
            Ahrs&& x);

    /*!
     * @brief Comparison operator.
     * @param x Ahrs object to compare.
     */
    eProsima_user_DllExport bool operator ==(
            const Ahrs& x) const;

    /*!
     * @brief Comparison operator.
     * @param x Ahrs object to compare.
     */
    eProsima_user_DllExport bool operator !=(
            const Ahrs& x) const;

    /*!
     * @brief This function sets a value in member roll
     * @param _roll New value for member roll
     */
    eProsima_user_DllExport void roll(
            float _roll);

    /*!
     * @brief This function returns the value of member roll
     * @return Value of member roll
     */
    eProsima_user_DllExport float roll() const;

    /*!
     * @brief This function returns a reference to member roll
     * @return Reference to member roll
     */
    eProsima_user_DllExport float& roll();

    /*!
     * @brief This function sets a value in member pitch
     * @param _pitch New value for member pitch
     */
    eProsima_user_DllExport void pitch(
            float _pitch);

    /*!
     * @brief This function returns the value of member pitch
     * @return Value of member pitch
     */
    eProsima_user_DllExport float pitch() const;

    /*!
     * @brief This function returns a reference to member pitch
     * @return Reference to member pitch
     */
    eProsima_user_DllExport float& pitch();

    /*!
     * @brief This function sets a value in member yaw
     * @param _yaw New value for member yaw
     */
    eProsima_user_DllExport void yaw(
            float _yaw);

    /*!
     * @brief This function returns the value of member yaw
     * @return Value of member yaw
     */
    eProsima_user_DllExport float yaw() const;

    /*!
     * @brief This function returns a reference to member yaw
     * @return Reference to member yaw
     */
    eProsima_user_DllExport float& yaw();


    /*!
     * @brief This function returns the maximum serialized size of an object
     * depending on the buffer alignment.
     * @param current_alignment Buffer alignment.
     * @return Maximum serialized size.
     */
    eProsima_user_DllExport static size_t getMaxCdrSerializedSize(
            size_t current_alignment = 0);

    /*!
     * @brief This function returns the serialized size of a data depending on the buffer alignment.
     * @param data Data which is calculated its serialized size.
     * @param current_alignment Buffer alignment.
     * @return Serialized size.
     */
    eProsima_user_DllExport static size_t getCdrSerializedSize(
            const Ahrs& data,
            size_t current_alignment = 0);


    /*!
     * @brief This function serializes an object using CDR serialization.
     * @param cdr CDR serialization object.
     */
    eProsima_user_DllExport void serialize(
            eprosima::fastcdr::Cdr& cdr) const;

    /*!
     * @brief This function deserializes an object using CDR serialization.
     * @param cdr CDR serialization object.
     */
    eProsima_user_DllExport void deserialize(
            eprosima::fastcdr::Cdr& cdr);



    /*!
     * @brief This function returns the maximum serialized size of the Key of an object
     * depending on the buffer alignment.
     * @param current_alignment Buffer alignment.
     * @return Maximum serialized size.
     */
    eProsima_user_DllExport static size_t getKeyMaxCdrSerializedSize(
            size_t current_alignment = 0);

    /*!
     * @brief This function tells you if the Key has been defined for this type
     */
    eProsima_user_DllExport static bool isKeyDefined();

    /*!
     * @brief This function serializes the key members of an object using CDR serialization.
     * @param cdr CDR serialization object.
     */
    eProsima_user_DllExport void serializeKey(
            eprosima::fastcdr::Cdr& cdr) const;

private:

    float m_roll;
    float m_pitch;
    float m_yaw;
};

#endif // _FAST_DDS_GENERATED_AHRS_H_