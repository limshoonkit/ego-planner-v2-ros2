/**
 * @file reliable_bridge.hpp
 * @brief Reliable bridge for ROS 2 data transfer in unstable network.
 * It will establish the connections as peer to peer mode.
 * It will reconnect each other autonomously.
 * It can guarantee the data transfer to the device in the strict order.
 * It has a queue for sending data asynchronously. 
 * 
 * Note: This program relies on ZMQ and ZMQPP.
 * sudo apt install libzmqpp-dev
 * 
 * Core Idea: It would create the sending and receiving thread for each device and process asynchronously.
 * The index number would correspond to the resource for one device.
 * 
 * @copyright Copyright (c) 2021-2024
 */
#ifndef __RELIABLE_BRIDGE__
#define __RELIABLE_BRIDGE__

#include "zmq.hpp"
#include "stdio.h"
#include "stdlib.h"
#include "time.h"
#include "zmqpp/zmqpp.hpp"
#include <deque>
#include <map>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <boost/shared_array.hpp>
#include <unordered_set>
#include <algorithm>
#include <memory>
#include <functional>
#include <vector>
#include <string>
#include <cassert>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

using namespace std;

namespace ser = rclcpp;

zmqpp::message &operator<<(zmqpp::message &in, rclcpp::SerializedMessage const &msg)
{
    in.add_raw(reinterpret_cast<void const *>(msg.get_rcl_serialized_message().buffer, msg.size()));
    return in;
}

class ReliableBridge
{
#define OUTPUT_MSG 2

#if (OUTPUT_MSG == 1)
#define print_info(...) RCLCPP_INFO(rclcpp::get_logger("ReliableBridge"), __VA_ARGS__)
#define print_warning(...)
#elif (OUTPUT_MSG == 2)
#define print_info(...) RCLCPP_INFO(rclcpp::get_logger("ReliableBridge"), __VA_ARGS__)
#define print_warning(...) RCLCPP_ERROR(rclcpp::get_logger("ReliableBridge"), __VA_ARGS__)
#else
#define print_info(...)
#define print_warning(...)
#endif

private:
    void transport_msg_thread(int index)
    {
        int ind = index;
        while (thread_flag)
        {
            {
                unique_lock<mutex> locker(*sender_mutex.at(ind));
                cond[ind]->wait(locker);
                auto &buffer = send_data[ind];
                locker.unlock();

                while (!buffer->empty() && thread_flag) {
                    auto &data = buffer->front();
                    zmqpp::message send_array;

                    send_array << data.first << data.second.size() << data.second;
                    if (senders[ind]->send(send_array, false))
                    {
                        unique_lock<mutex> locker2(*sender_mutex.at(ind));
                        buffer->pop_front();
                        locker2.unlock();
                    }
                }
            }
        }
    }

    void recv_thread(int index)
    {
        int ind = index;
        int ID = id_list[ind];
        while (thread_flag)
        {
            zmqpp::message recv_array;

            if (receivers[ind]->receive(recv_array, false))
            {
                string topic_name;
                size_t data_len;

                recv_array >> topic_name >> data_len;

                rclcpp::SerializedMessage msg_ser(data_len);
                memcpy(msg_ser.get_rcl_serialized_message().buffer,
                       static_cast<const uint8_t *>(recv_array.raw_data(recv_array.read_cursor())),
                       data_len);
                recv_array.next();
                msg_ser.get_rcl_serialized_message().buffer_length = data_len;

                auto &topic_cb = callback_list[ind];
                const auto &iter = topic_cb->find(topic_name);
                if (iter != topic_cb->end())
                {
                    iter->second(ID, msg_ser);
                }
            }
            usleep(1);
        }
    }

public:
    bool thread_flag;

    void StopThread() {
        thread_flag = false;
        for (size_t i = 0; i < ip_list.size(); i++) {
            senders[i]->close();
        }

        for (std::thread &th : recv_threads) {
            if(th.joinable()) {
                pthread_cancel(th.native_handle());
                std::cout << "rec thread stopped" << std::endl;
            } else {
                std::cout << "can not join rec thread: " << th.get_id() << std::endl;
            }
        }

        for (std::thread &th : send_threads) {
            if(th.joinable()) {
                pthread_cancel(th.native_handle());
                std::cout << "send thread stopped" << std::endl;
            } else {
                std::cout << "can not join send thread: " << th.get_id() << std::endl;
            }
        }
    }

    zmqpp::context_t context;

    vector<unique_ptr<zmqpp::socket>> senders;
    vector<unique_ptr<zmqpp::socket>> receivers;

    vector<unique_ptr<std::deque<pair<string, rclcpp::SerializedMessage>>>> send_data;

    vector<unique_ptr<mutex>> sender_mutex;
    vector<unique_ptr<condition_variable>> cond;
    vector<std::thread> send_threads;
    vector<std::thread> recv_threads;

    vector<unique_ptr<map<string, function<void(int, rclcpp::SerializedMessage &)>>>> callback_list;

    map<int, int> id_remap;
    vector<int> id_list;
    vector<string> ip_list;
    const int self_id;
    const uint queue_size;

    template <typename T>
    static bool containsDuplicate(const vector<T> &v)
    {
        unordered_set<T> s(v.size() * 2);
        for (auto x : v)
            if (!s.insert(x).second)
                return true;
        return false;
    }

    ReliableBridge(const int self_ID, vector<string> &IP_list, vector<int> &ID_list, uint Queue_size = 10000) : 
        self_id(self_ID), ip_list(IP_list), id_list(ID_list), queue_size(Queue_size), thread_flag(true)
    {
        if(ip_list.size() != id_list.size())
        {
            print_warning("[Bridge Warning]: IP doesn't match ID!");
            assert(ip_list.size() == id_list.size());       
        }    
        int maxValue = *max_element(id_list.begin(),id_list.end());
        int minValue = *min_element(id_list.begin(),id_list.end());
        if(id_list.size()  > 100)
        {
            print_warning("[Bridge Warning]: Bridge only supports up to 100 devices!");
            assert(id_list.size()  <= 100);
            return;
        }
        if(maxValue  > 100 || minValue < 0 || self_ID > 100 || self_ID < 0)
        {
            print_warning("[Bridge Warning]: ID is invalid!");
            assert(maxValue  <= 100 && minValue >= 0 && self_ID <= 100 && self_ID >= 0);
            return;
        }
        if(containsDuplicate(id_list) == true)
        {
            print_warning("[Bridge Warning]: ID list has duplicate!");
            assert(containsDuplicate(id_list) == false);
            return;
        }
        const auto &self_pos = std::find(id_list.begin(), id_list.end(), self_id);

        if (self_pos != id_list.end())
        {
            int ind = self_pos - id_list.begin();
            id_list.erase(self_pos);
            ip_list.erase(ip_list.begin() + ind);
        }

        for (size_t i = 0; i < ip_list.size(); i++)
        {
            id_remap.emplace(std::make_pair(id_list[i], i));

            string url = "tcp://*:" + to_string(30000 +self_id*100 + id_list[i]);
            print_info("[Bridge]: Bind: %s", url.c_str());
            unique_ptr<zmqpp::socket> sender(new zmqpp::socket(context, zmqpp::socket_type::push));
            sender->bind(url);
            senders.emplace_back(std::move(sender));

            url = "tcp://" + ip_list[i] + ":" + to_string(30000 + id_list[i]*100 + self_id);
            print_info("[Bridge]: Connect: %s", url.c_str());
            unique_ptr<zmqpp::socket> receiver(new zmqpp::socket(context, zmqpp::socket_type::pull));
            receiver->connect(url);
            receivers.emplace_back(std::move(receiver));

            callback_list.emplace_back(new map<string, function<void(int, rclcpp::SerializedMessage &)>>());

            send_data.emplace_back(new std::deque<pair<string, rclcpp::SerializedMessage>>());
            sender_mutex.emplace_back(new mutex());
            cond.emplace_back(new condition_variable());

            send_threads.emplace_back(std::thread(&ReliableBridge::transport_msg_thread, this, i));
            recv_threads.emplace_back(std::thread(&ReliableBridge::recv_thread, this, i));
        }
    }

    void register_callback(int ID, string topic_name, function<void(int, rclcpp::SerializedMessage &)> callback)
    {   
        int ind;
        try
        {
            ind = id_remap.at(ID);
        }
        catch(const std::exception& e)
        {
            print_warning("ID is not in the list!");
            return;
        }
        callback_list[ind]->emplace(topic_name, callback);
    }

    void register_callback_for_all(string topic_name, function<void(int, rclcpp::SerializedMessage &)> callback)
    {
        for (size_t i = 0; i < id_list.size(); i++)
        {
            register_callback(id_list[i], topic_name, callback);
        }
    }

    template <typename T>
    int send_msg_to_one(int ID, string topic_name, T &msg)
    {
        int ind;
        try
        {
            ind = id_remap.at(ID);
        }
        catch(const std::exception& e)
        {
            print_warning("ID is not in the list!");
            return -2;
        }
        auto &buffer = send_data[ind];
        if (buffer->size() > queue_size)
        {
            print_warning("[Bridge Warning]: ID:%d Send buffer is full", ID);
            return -1;
        }
        {
            unique_lock<mutex> locker(*sender_mutex.at(ind));
            rclcpp::SerializedMessage serialized_msg;
            rclcpp::Serialization<T> serializer;
            serializer.serialize_message(&msg, &serialized_msg);
            buffer->emplace_back(make_pair(topic_name, std::move(serialized_msg)));
            locker.unlock();
            cond[ind]->notify_all();
        }
        return 0;
    }

    template <typename T>
    int send_msg_to_all(string topic_name, T &msg)
    {
        int err_code = 0;
        for (size_t i = 0; i < id_list.size(); i++)
        {
            err_code += send_msg_to_one<T>(id_list[i], topic_name, msg);
        }
        return err_code;
    }
};

#endif
