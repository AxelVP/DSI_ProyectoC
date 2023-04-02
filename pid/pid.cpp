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

/**
 * @file MandoSubscriber.cpp
 *
 */

#include "../topics/pid/PidPubSubTypes.h"
#include "../topics/ahrs/AhrsPubSubTypes.h"
#include "../topics/mando/MandoPubSubTypes.h"

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <string>
#include <unistd.h>
#include "PIDControl.h"

#include <chrono>
#include <sstream>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <string>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>

#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>

#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>

#define MANDO_MIN 1.0        //valor minimo que se recibe del throttle en el mando
#define MANDO_MAX 181.0       //valor maximo que se recibe del throttle en el mando
#define MOTOR_MAX 1750.0      //valor maximo que recibe el PWM de los motores
#define MOTOR_MIN 1000.0      //valor minimo que recibe el PWM de los motores (con 1000 no se mueven)
#define PID_MAX 180           //valor maximo que se va a recibir del PID (roll, pitch o yaw)

using namespace eprosima::fastdds::dds;
using namespace std::chrono;
 
int* mixer(float roll, float pitch, float yaw, float throttle) {
    static int valores[4];

    /*
    Calculamos el valor de los motores correspondiente al throttle segun una funcion lineal
    Debe cumplir dos requisitos:
    - f(MANDO_MIN) = MOTOR_MIN = 1000
    - f(MANDO_MAX) = MOTOR_MAX = 1400

    f(x) = (MOTOR_MAX-MOTOR_MIN)/(MANDO_MAX - MANDO_MIN)*throttle + (MOTOR_MIN*MANDO_MAX - MOTOR_MAX*MANDO_MIN)/(MANDO_MAX-MANDO_MIN)
    f(x) =  400/(MANDO_MAX-MANDO_MIN)*throttle + (1000*MANDO_MAX - 1400*MANDO_MIN)/(MANDO_MAX - MANDO_MIX) 
    */
    float m = (MOTOR_MAX-MOTOR_MIN)/(MANDO_MAX-MANDO_MIN);
    float n = (MOTOR_MIN*MANDO_MAX - MOTOR_MAX*MANDO_MIN)/(MANDO_MAX-MANDO_MIN);
    float motor_throttle = m*throttle + n;
    std::cout << "f(x) = " << m << "x + " << n << ", Resultado: " << motor_throttle << std::endl;

    /*
    Calculamos los valores del mixer correspondientes
    */
    valores[0] = motor_throttle + roll - pitch + yaw;
    valores[1] = motor_throttle - roll + pitch + yaw;
    valores[2] = motor_throttle - roll - pitch - yaw;
    valores[3] = motor_throttle + roll + pitch - yaw;

    /*
    Nos aseguramos de que la salida no sobrepase los valores limites de los motores
    */
    if(valores[0] > MOTOR_MAX) valores[0] = MOTOR_MAX;
    if(valores[1] > MOTOR_MAX) valores[1] = MOTOR_MAX;
    if(valores[2] > MOTOR_MAX) valores[2] = MOTOR_MAX;
    if(valores[3] > MOTOR_MAX) valores[3] = MOTOR_MAX;

    if(valores[0] < MOTOR_MIN) valores[0] = MOTOR_MIN;
    if(valores[1] < MOTOR_MIN) valores[1] = MOTOR_MIN;
    if(valores[2] < MOTOR_MIN) valores[2] = MOTOR_MIN;
    if(valores[3] < MOTOR_MIN) valores[3] = MOTOR_MIN;

    return valores;
}



Mando mando_;

class MandoSubscriber
{
private:

    DomainParticipant* participant_;

    Subscriber* subscriber_;

    DataReader* reader_;

    Topic* topic_;

    TypeSupport type_;

    class SubListener : public DataReaderListener
    {
    public:

        SubListener()
            : samples_(0)
        {
        }

        ~SubListener() override
        {
        }

        void on_subscription_matched(
                DataReader*,
                const SubscriptionMatchedStatus& info) override
        {
            if (info.current_count_change == 1)
            {
                std::cout << "Subscriber matched." << std::endl;
            }
            else if (info.current_count_change == -1)
            {
                std::cout << "Subscriber unmatched." << std::endl;
            }
            else
            {
                std::cout << info.current_count_change
                        << " is not a valid value for SubscriptionMatchedStatus current count change" << std::endl;
            }
        }

        void on_data_available(
                DataReader* reader) override
        {
            SampleInfo info;
            if (reader->take_next_sample(&mando_, &info) == ReturnCode_t::RETCODE_OK)
            {
                if (info.valid_data)
                {
                    
                    //IMPRIMIR TODOS LOS VALORES
                    //IMPRIMIR TODOS LOS VALORES
                    //IMPRIMIR TODOS LOS VALORES
                    //IMPRIMIR TODOS LOS VALORES
                    
                    /*std::cout   << "j_izq_ver: " << mando_.j_izq_ver() << "\n"
                                << "j_der_ver: " << mando_.j_der_ver() << "\n"
                                << "j_izq_hor: " << mando_.j_izq_hor() << "\n"
                                << "j_der_hor: " << mando_.j_der_hor() << "\n"
                                << "p_izq: " << mando_.p_izq() << "\n"
                                << "p_der: " << mando_.p_der() << "\n"
                                << std::endl;*/
                }
            }
        }

        

        std::atomic_int samples_;

    } listener_;

public:

    MandoSubscriber()
        : participant_(nullptr)
        , subscriber_(nullptr)
        , topic_(nullptr)
        , reader_(nullptr)
        , type_(new MandoPubSubType())
    {
    }

    virtual ~MandoSubscriber()
    {
        if (reader_ != nullptr)
        {
            subscriber_->delete_datareader(reader_);
        }
        if (topic_ != nullptr)
        {
            participant_->delete_topic(topic_);
        }
        if (subscriber_ != nullptr)
        {
            participant_->delete_subscriber(subscriber_);
        }
        DomainParticipantFactory::get_instance()->delete_participant(participant_);
    }

    //!Initialize the subscriber
    bool init()
    {
        DomainParticipantQos participantQos;
        participantQos.name("Participant_subscriber");
        participant_ = DomainParticipantFactory::get_instance()->create_participant(0, participantQos);

        if (participant_ == nullptr)
        {
            return false;
        }

        // Register the Type
        type_.register_type(participant_);

        // Create the subscriptions Topic
        topic_ = participant_->create_topic("MandoTopic", "Mando", TOPIC_QOS_DEFAULT);

        if (topic_ == nullptr)
        {
            return false;
        }

        // Create the Subscriber
        subscriber_ = participant_->create_subscriber(SUBSCRIBER_QOS_DEFAULT, nullptr);

        if (subscriber_ == nullptr)
        {
            return false;
        }

        // Create the DataReader
        reader_ = subscriber_->create_datareader(topic_, DATAREADER_QOS_DEFAULT, &listener_);

        if (reader_ == nullptr)
        {
            return false;
        }

        return true;
    }

    //!Run the Subscriber
    void run()
    {
        while(true)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
};



Ahrs ahrs_;

class AhrsSubscriber
{
private:

    DomainParticipant* participant_;

    Subscriber* subscriber_;

    DataReader* reader_;

    Topic* topic_;

    TypeSupport type_;

    class SubListener : public DataReaderListener
    {
    public:

        SubListener()
            : samples_(0)
        {
        }

        ~SubListener() override
        {
        }

        void on_subscription_matched(
                DataReader*,
                const SubscriptionMatchedStatus& info) override
        {
            if (info.current_count_change == 1)
            {
                std::cout << "Subscriber matched." << std::endl;
            }
            else if (info.current_count_change == -1)
            {
                std::cout << "Subscriber unmatched." << std::endl;
            }
            else
            {
                std::cout << info.current_count_change
                        << " is not a valid value for SubscriptionMatchedStatus current count change" << std::endl;
            }
        }

        void on_data_available(
                DataReader* reader) override
        {
            SampleInfo info;
            if (reader->take_next_sample(&ahrs_, &info) == ReturnCode_t::RETCODE_OK)
            {
                if (info.valid_data)
                {
                    
                    //IMPRIMIR TODOS LOS VALORES
                    //IMPRIMIR TODOS LOS VALORES
                    //IMPRIMIR TODOS LOS VALORES
                    //IMPRIMIR TODOS LOS VALORES
                    
                    /*std::cout   << "roll: " << ahrs_.roll() << "\n"
                                << "pitch: " << ahrs_.pitch() << "\n"
                                << "yaw: " << ahrs_.yaw() << "\n"
                                << std::endl;*/
                }
            }
        }

        

        std::atomic_int samples_;

    } listener_;

public:

    AhrsSubscriber()
        : participant_(nullptr)
        , subscriber_(nullptr)
        , topic_(nullptr)
        , reader_(nullptr)
        , type_(new AhrsPubSubType())
    {
    }

    virtual ~AhrsSubscriber()
    {
        if (reader_ != nullptr)
        {
            subscriber_->delete_datareader(reader_);
        }
        if (topic_ != nullptr)
        {
            participant_->delete_topic(topic_);
        }
        if (subscriber_ != nullptr)
        {
            participant_->delete_subscriber(subscriber_);
        }
        DomainParticipantFactory::get_instance()->delete_participant(participant_);
    }

    //!Initialize the subscriber
    bool init()
    {
        DomainParticipantQos participantQos;
        participantQos.name("Participant_subscriber");
        participant_ = DomainParticipantFactory::get_instance()->create_participant(0, participantQos);

        if (participant_ == nullptr)
        {
            return false;
        }

        // Register the Type
        type_.register_type(participant_);

        // Create the subscriptions Topic
        topic_ = participant_->create_topic("AhrsTopic", "Ahrs", TOPIC_QOS_DEFAULT);

        if (topic_ == nullptr)
        {
            return false;
        }

        // Create the Subscriber
        subscriber_ = participant_->create_subscriber(SUBSCRIBER_QOS_DEFAULT, nullptr);

        if (subscriber_ == nullptr)
        {
            return false;
        }

        // Create the DataReader
        reader_ = subscriber_->create_datareader(topic_, DATAREADER_QOS_DEFAULT, &listener_);

        if (reader_ == nullptr)
        {
            return false;
        }

        return true;
    }

    //!Run the Subscriber
    void run()
    {
        while(true)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
};


Pid pid_;

class PidPublisher
{
private:

    DomainParticipant* participant_;

    Publisher* publisher_;

    Topic* topic_;

    DataWriter* writer_;

    TypeSupport type_;

    class PubListener : public DataWriterListener
    {
    public:

        PubListener()
            : matched_(0)
        {
        }

        ~PubListener() override
        {
        }

        void on_publication_matched(
                DataWriter*,
                const PublicationMatchedStatus& info) override
        {
            if (info.current_count_change == 1)
            {
                matched_ = info.total_count;
                std::cout << "Publisher matched." << std::endl;
            }
            else if (info.current_count_change == -1)
            {
                matched_ = info.total_count;
                std::cout << "Publisher unmatched." << std::endl;
            }
            else
            {
                std::cout << info.current_count_change
                        << " is not a valid value for PublicationMatchedStatus current count change." << std::endl;
            }
        }

        std::atomic_int matched_;

    } listener_;

public:

    PidPublisher()
        : participant_(nullptr)
        , publisher_(nullptr)
        , topic_(nullptr)
        , writer_(nullptr)
        , type_(new PidPubSubType())
    {
    }

    virtual ~PidPublisher()
    {
        if (writer_ != nullptr)
        {
            publisher_->delete_datawriter(writer_);
        }
        if (publisher_ != nullptr)
        {
            participant_->delete_publisher(publisher_);
        }
        if (topic_ != nullptr)
        {
            participant_->delete_topic(topic_);
        }
        DomainParticipantFactory::get_instance()->delete_participant(participant_);
    }

    //!Initialize the publisher
    bool init()
    {

        pid_.v_1(0);
        pid_.v_2(0);
        pid_.v_3(0);
        pid_.v_4(0);

        DomainParticipantQos participantQos;
        participantQos.name("Participant_publisher");
        participant_ = DomainParticipantFactory::get_instance()->create_participant(0, participantQos);

        if (participant_ == nullptr)
        {
            return false;
        }

        // Register the Type
        type_.register_type(participant_);

        // Create the publications Topic
        topic_ = participant_->create_topic("PidTopic", "Pid", TOPIC_QOS_DEFAULT);

        if (topic_ == nullptr)
        {
            return false;
        }

        // Create the Publisher
        publisher_ = participant_->create_publisher(PUBLISHER_QOS_DEFAULT, nullptr);

        if (publisher_ == nullptr)
        {
            return false;
        }

        // Create the DataWriter
        writer_ = publisher_->create_datawriter(topic_, DATAWRITER_QOS_DEFAULT, &listener_);

        if (writer_ == nullptr)
        {
            return false;
        }
        return true;
    }

    //!Send a publication
    bool publish()
    {

        //CODIGO VALORES
        //CODIGO VALORES
        //CODIGO VALORES
        //CODIGO VALORES

        //pid_.v_1(pid_.v_1()+1);
        //pid_.v_2(pid_.v_2()+1);
        //pid_.v_3(pid_.v_3()+1);
        //pid_.v_4(pid_.v_4()+1);

        //CODIGO VALORES
        //CODIGO VALORES
        //CODIGO VALORES
        //CODIGO VALORES

        writer_->write(&pid_);
        return true;

    }

    //!Run the Publisher
    void run()
    {
        
        if (publish())
        {
            //IMPRIMIR TODOS LOS VALORES
            //IMPRIMIR TODOS LOS VALORES
            //IMPRIMIR TODOS LOS VALORES
            //IMPRIMIR TODOS LOS VALORES
            
            /*std::cout   << "VALORES PID: " << "\n"
                        << "v_2: " << pid_.v_2() << "\n"
                        << "v_3: " << pid_.v_3() << "\n"
                        << "v_4: " << pid_.v_4() << "\n"
                        << std::endl;*/
            
        }
    
    }
};





int main(
        int argc,
        char** argv)
{
    std::cout << "Starting subscriber." << std::endl;
    int samples = 10;

    //PID
    std::cout << "Ejecutando el PID... \n";

    std::ifstream indata;
    float valor, K[9];
    int i = 0;

    //Leer el fichero con las constantes del PID
    indata.open("pid.txt"); // opens the file
    if(!indata) { 
        std::cerr << "Error abriendo el fichero" << std::endl;
        exit(1);
    }
    indata >> valor;

    while ( !indata.eof() ) {
        K[i] = valor;
        indata >> valor; 
        i++;        
    }
    std::cout << std::endl;
    indata.close();
    PIDControl<float> pid_roll(K[0], K[1], K[2]);
    PIDControl<float> pid_pitch(K[3], K[4], K[5]);
    PIDControl<float> pid_yaw(K[6], K[7], K[8]);

    std::cout << std::endl;
    indata.close();


    pid_roll.setFeedbackWrapBounds(0.0, 360.0);
    pid_roll.setLastTime(duration_cast< milliseconds >(system_clock::now().time_since_epoch()).count());
    pid_roll.setInputBounded(false);
    pid_roll.setOutputBounded(false);

    pid_pitch.setFeedbackWrapBounds(0.0, 360.0);
    pid_pitch.setLastTime(duration_cast< milliseconds >(system_clock::now().time_since_epoch()).count());
    pid_pitch.setInputBounded(false);
    pid_pitch.setOutputBounded(false);

    pid_yaw.setFeedbackWrapBounds(0.0, 360.0);
    pid_yaw.setLastTime(duration_cast< milliseconds >(system_clock::now().time_since_epoch()).count());
    pid_yaw.setInputBounded(false);
    pid_yaw.setOutputBounded(false);
    usleep(4000);

    float sensor_roll, sensor_pitch, sensor_yaw;
    float mando_roll, mando_pitch, mando_yaw;

    //PID


    MandoSubscriber* mysubMando = new MandoSubscriber();
    AhrsSubscriber* mysubAhrs = new AhrsSubscriber();
    PidPublisher* mypubPid = new PidPublisher();
    mysubMando->init();
    mysubAhrs->init();
    mypubPid->init();

    while(true){
    
        //PID ROLL
        //std::cout << "ROLL" << std::endl;
        pid_roll.tick(ahrs_.roll(), mando_.j_der_hor());
        std::cout << "sensor " << ahrs_.roll() << std::endl;
        std::cout << "mando " << mando_.j_der_hor() << std::endl;
        std::cout << "output " << pid_roll.getOutput() << std::endl;
        std::cout << std::endl;
        
        //PID PITCH
        //std::cout << "PITCH" << std::endl;
        pid_pitch.tick(ahrs_.pitch(), mando_.j_der_ver());
        std::cout << "sensor " << ahrs_.pitch() << std::endl;
        std::cout << "mando " << mando_.j_der_ver() << std::endl;
        std::cout << "output " << pid_pitch.getOutput() << std::endl;
        std::cout << std::endl;

        //PID YAW
        //std::cout << "YAW" << std::endl;
        pid_yaw.tick(ahrs_.yaw(), 180.0);
        std::cout << "sensor: " << ahrs_.yaw() << std::endl;
        std::cout << "mando: " << mando_.j_izq_hor() << std::endl;
        std::cout << "output: " << pid_yaw.getOutput() << std::endl;
        std::cout << std::endl;
        
        ///////////////// FALTA MIXER y PUBLICAR EN DDS
        int* valores = mixer(pid_roll.getOutput(), pid_pitch.getOutput(), pid_yaw.getOutput(), mando_.j_izq_ver());
        pid_.v_1(valores[0]);
        pid_.v_2(valores[1]);
        pid_.v_3(valores[2]);
        pid_.v_4(valores[3]);

        mypubPid->run();
        std::cout << std::endl;

        usleep(3000); 
    }

    delete mysubMando;
    delete mysubAhrs;
    delete mypubPid;
    return 0;
}
