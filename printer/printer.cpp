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
 * @file printer.cpp
 *
 */

#include "../topics/pid/PidPubSubTypes.h"
#include "../topics/ahrs/AhrsPubSubTypes.h"
#include "../topics/mando/MandoPubSubTypes.h"
#include "../topics/killswitch/KillswitchPubSubTypes.h"

#include <chrono>
#include <signal.h>
#include <unistd.h>
#include <sys/types.h>
#include <sstream>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>

#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>

#include <bits/stdc++.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <cstring>

using namespace eprosima::fastdds::dds;



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

        Mando mando_;

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
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
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
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
};

Pid pid_;
int sockfd;
struct sockaddr_in servaddr;

class PidSubscriber
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
            if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
            {
                perror("socket creation failed");
                exit(EXIT_FAILURE);
            }

            memset(&servaddr, 0, sizeof(servaddr));

            // Filling server information
            servaddr.sin_family = AF_INET;
            servaddr.sin_port = htons(1881);
            servaddr.sin_addr.s_addr = inet_addr("192.168.131.48");
        }

        

        

        void on_data_available(
                DataReader* reader) override
        {
            SampleInfo info;
            if (reader->take_next_sample(&pid_, &info) == ReturnCode_t::RETCODE_OK)
            {
                if (info.valid_data)
                {
                    // IMPRIMIR TOD"Hola6" << std::endl;OS LOS VALORES
                    // IMPRIMIR TODOS LOS VALORES
                    // IMPRIMIR TODOS LOS VALORES
                    // IMPRIMIR TODOS LOS VALORES

                    std::cout   << "v_1: " << pid_.v_1() << "\n"
                                << "v_2: " << pid_.v_2() << "\n"
                                << "v_3: " << pid_.v_3() << "\n"
                                << "v_4: " << pid_.v_4() << "\n"
                                << std::endl;

                    

                    

                    std::string motor1 = std::to_string(pid_.v_1());
                    std::string motor2 = std::to_string(pid_.v_2());
                    std::string motor3 = std::to_string(pid_.v_3());
                    std::string motor4 = std::to_string(pid_.v_4());
                    std::string pitch = std::to_string(ahrs_.pitch());
                    std::string roll = std::to_string(ahrs_.roll());
                    std::string yaw = std::to_string(ahrs_.yaw());

                    std::string cadena = motor1 + "," + motor2 + "," + motor3 + "," + motor4 + "," + pitch + "," + roll + "," + yaw;
                    
                    char *motores = new char[cadena.length() + 1];

                    strcpy(motores, cadena.c_str());
                   
                   
                    sendto(sockfd, (const char *)motores, strlen(motores),
                           MSG_CONFIRM, (const struct sockaddr *)&servaddr,
                           sizeof(servaddr));
                }
            }
        }

        

        std::atomic_int samples_;

    } listener_;

public:

    PidSubscriber()
        : participant_(nullptr)
        , subscriber_(nullptr)
        , topic_(nullptr)
        , reader_(nullptr)
        , type_(new PidPubSubType())
    {
    }

    virtual ~PidSubscriber()
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
        topic_ = participant_->create_topic("PidTopic", "Pid", TOPIC_QOS_DEFAULT);

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
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
};




class KillswitchSubscriber
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
            if (reader->take_next_sample(&killswitch_, &info) == ReturnCode_t::RETCODE_OK)
            {
                if (info.valid_data)
                {
                    //IMPRIMIR TODOS LOS VALORES
                    //IMPRIMIR TODOS LOS VALORES
                    //IMPRIMIR TODOS LOS VALORES
                    //IMPRIMIR TODOS LOS VALORES

                    //kill(getpid(),SIGINT);
                    
                    std::cout   << "kills: " << killswitch_.kills() << "\n"
                                << std::endl;
                }
            }
        }

        Killswitch killswitch_;

        std::atomic_int samples_;

    } listener_;

public:

    KillswitchSubscriber()
        : participant_(nullptr)
        , subscriber_(nullptr)
        , topic_(nullptr)
        , reader_(nullptr)
        , type_(new KillswitchPubSubType())
    {
    }

    virtual ~KillswitchSubscriber()
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
        topic_ = participant_->create_topic("KillswitchTopic", "Killswitch", TOPIC_QOS_DEFAULT);

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
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
};






int main(
        int argc,
        char** argv)
{
    std::cout << "Starting subscriber." << std::endl;
    int samples = 10;
    
    MandoSubscriber* mysubMando = new MandoSubscriber();
    AhrsSubscriber* mysubAhrs = new AhrsSubscriber();
    PidSubscriber* mysubPid = new PidSubscriber();
    KillswitchSubscriber* mysubKillswitch = new KillswitchSubscriber();

    mysubPid->init();
    mysubKillswitch->init();
    mysubMando->init();
    mysubAhrs->init();

    while(true){
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    delete mysubMando;
    delete mysubAhrs;
    delete mysubKillswitch;
    delete mysubPid;
    return 0;
}
