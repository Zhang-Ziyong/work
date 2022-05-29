#pragma once
#include <rclcpp/rclcpp.hpp>
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "Svar.h"

template <typename T>
T& offset(void* msg){
    return *(T*)((uint8_t*)msg);
}

template <typename T>
T& offset(const void* msg){
    return *(T*)((uint8_t*)msg);
}

template <typename T>
sv::SvarBuffer typebuf(const rosidl_typesupport_introspection_cpp::MessageMember& member,
                       uint8_t* v_ptr, ssize_t size){
    return sv::SvarBuffer((T*)member.get_const_function(v_ptr, 0),std::vector<ssize_t>({size})).clone();
}

template <typename TT,typename ST=TT>
void asign(const rosidl_typesupport_introspection_cpp::MessageMember& member,
           void* msg, const sv::Svar& value){
    if(!member.is_array_){
        offset<TT>(msg) = value.castAs<ST>();
        return;
    }

    if(value.isArray()){
        auto size = value.size();
        member.resize_function(msg,size);
        for(int i=0;i<size;i++)
            offset<TT>(member.get_function(msg,i)) = value[i].castAs<ST>();
    }
    else if(value.is<sv::SvarBuffer>()){
        const sv::SvarBuffer& buf = value.as<sv::SvarBuffer>();
        auto size = buf.size()/sizeof(TT);
        member.resize_function(msg,size);
        memcpy(member.get_function(msg,0),buf.ptr(),buf.size());
    }
    else
        throw sv::SvarExeption("Unkown how to asign");
}

template <typename TT,typename ST=TT>
sv::Svar obtain(const rosidl_typesupport_introspection_cpp::MessageMember& member,
                const void* msg, bool cbor, bool no_array=false){
    if(no_array || ! member.is_array_){
        return (TT)offset<ST>(msg);
    }

    ssize_t size = member.array_size_;
    if(size == 0)
        size = member.size_function(msg);

    if(cbor)
        return sv::SvarBuffer((ST*)member.get_const_function(msg, 0),std::vector<ssize_t>({size})).clone();

    std::vector<sv::Svar> vec;
    vec.reserve(size);
    for(int i=0;i<size;i++)
        vec.push_back(obtain<TT,ST>(member,member.get_const_function(msg,i),cbor,true));
    return vec;
}

inline void set_value(const rosidl_message_type_support_t* support,void* msg,const sv::Svar& v){
    using namespace rosidl_typesupport_introspection_cpp;
    MessageMembers* members = (MessageMembers*)support->data;

    for(int i=0;i<members->member_count_;i++){
        const rosidl_typesupport_introspection_cpp::MessageMember& member = members->members_[i];
        sv::Svar value = v[member.name_];
        if(value.isUndefined()) continue;
        uint32_t skip = member.offset_;
        uint8_t* v_ptr= (uint8_t*)msg + skip;

        switch (member.type_id_) {
        case ROS_TYPE_FLOAT:
            asign<float,double>(member, v_ptr, value);
            break;
        case ROS_TYPE_DOUBLE:
            asign<double,double>(member, v_ptr, value);
            break;
        case ROS_TYPE_LONG_DOUBLE:
            asign<long double,double>(member, v_ptr, value);
            break;
        case ROS_TYPE_CHAR:
            asign<char,int>(member, v_ptr, value);
            break;
        case ROS_TYPE_WCHAR:
            asign<wchar_t,int>(member, v_ptr, value);
            break;
        case ROS_TYPE_BOOLEAN:
            asign<bool,bool>(member, v_ptr, value);
            break;
        case ROS_TYPE_OCTET:
            throw sv::SvarExeption("Unkown type ROS_TYPE_OCTET");
            break;
        case ROS_TYPE_UINT8:
            asign<uint8_t,int>(member, v_ptr, value);
            break;
        case ROS_TYPE_INT8:
            asign<int8_t,int>(member, v_ptr, value);
            break;
        case ROS_TYPE_UINT16:
            asign<uint16_t,int>(member, v_ptr, value);
            break;
        case ROS_TYPE_INT16:
            asign<int16_t,int>(member, v_ptr, value);
            break;
        case ROS_TYPE_UINT32:
            asign<uint32_t,int>(member, v_ptr, value);
            break;
        case ROS_TYPE_INT32:
            asign<int32_t,int>(member, v_ptr, value);
            break;
        case ROS_TYPE_UINT64:
            asign<uint64_t,int>(member, v_ptr, value);
            break;
        case ROS_TYPE_INT64:
            asign<int64_t,int>(member, v_ptr, value);
            break;
        case ROS_TYPE_STRING:
            asign<std::string>(member, v_ptr, value);
            break;
        case ROS_TYPE_WSTRING:
            throw sv::SvarExeption("Unkown type ROS_TYPE_STRING");
            break;
        case ROS_TYPE_MESSAGE:
            if(member.is_array_){
                auto size = member.array_size_;
                if(size == 0)
                    size = member.size_function(v_ptr);
                member.resize_function(v_ptr, value.size());
                for(int i=0;i<value.size();i++){
                    void * item_ptr = member.get_function(v_ptr,i);
                    set_value(member.members_, item_ptr, value[i]);
                }
            }
            else
                set_value(member.members_, v_ptr, value);
            break;
        default:
            break;
        }
    }
}

inline void get_value(const rosidl_message_type_support_t* support,const void* msg,sv::Svar& v,bool cbor=false){
    using namespace rosidl_typesupport_introspection_cpp;
    MessageMembers* members = (MessageMembers*)support->data;

    for(int i=0;i<members->member_count_;i++){
        const rosidl_typesupport_introspection_cpp::MessageMember& member = members->members_[i];
        uint32_t skip = member.offset_;
        uint8_t* v_ptr= (uint8_t*)msg + skip;

        switch (member.type_id_) {
        case ROS_TYPE_FLOAT:
            v[member.name_] = obtain<double,float>(member,v_ptr,cbor);
            break;
        case ROS_TYPE_DOUBLE:
            v[member.name_] = obtain<double,double>(member,v_ptr,cbor);
            break;
        case ROS_TYPE_LONG_DOUBLE:
            v[member.name_] = obtain<double,long double>(member,v_ptr,cbor);
            break;
        case ROS_TYPE_CHAR:
            v[member.name_] = obtain<int,char>(member,v_ptr,cbor);
            break;
        case ROS_TYPE_WCHAR:
            v[member.name_] = obtain<int,wchar_t>(member,v_ptr,cbor);
            break;
        case ROS_TYPE_BOOLEAN:
            v[member.name_] = obtain<bool,bool>(member,v_ptr,cbor);
            break;
        case ROS_TYPE_OCTET:
            throw sv::SvarExeption("Unkown type ROS_TYPE_OCTET");
            break;
        case ROS_TYPE_UINT8:
            v[member.name_] = obtain<int,uint8_t>(member,v_ptr,cbor);
            break;
        case ROS_TYPE_INT8:
            v[member.name_] = obtain<int,int8_t>(member,v_ptr,cbor);
            break;
        case ROS_TYPE_UINT16:
            v[member.name_] = obtain<int,uint16_t>(member,v_ptr,cbor);
            break;
        case ROS_TYPE_INT16:
            v[member.name_] = obtain<int,int16_t>(member,v_ptr,cbor);
            break;
        case ROS_TYPE_UINT32:
            v[member.name_] = obtain<int,uint32_t>(member,v_ptr,cbor);
            break;
        case ROS_TYPE_INT32:
            v[member.name_] = obtain<int,int32_t>(member,v_ptr,cbor);
            break;
        case ROS_TYPE_UINT64:
            v[member.name_] = obtain<int,uint64_t>(member,v_ptr,cbor);
            break;
        case ROS_TYPE_INT64:
            v[member.name_] = obtain<int,int64_t>(member,v_ptr,cbor);
            break;
        case ROS_TYPE_STRING:
            if(!member.is_array_){
                v[member.name_] = offset<std::string>(v_ptr);
            }
            else{
                auto size = member.array_size_;
                if(size == 0)
                    size = member.size_function(v_ptr);

                std::vector<sv::Svar> vec;
                vec.reserve(size);
                for(int i=0;i<size;i++)
                    vec.push_back(offset<std::string>(member.get_const_function(v_ptr,i)));
                v[member.name_] = vec;
            }
            break;
        case ROS_TYPE_WSTRING:
            throw sv::SvarExeption("Unkown type ROS_TYPE_STRING");
            break;
        case ROS_TYPE_MESSAGE:
            if(member.is_array_)
            {
                auto size = member.array_size_;
                if(size == 0)
                    size = member.size_function(v_ptr);
                std::vector<sv::Svar> child;
                child.reserve(size);
                for(int i=0;i<size;i++){
                    const void * item_ptr = member.get_const_function(v_ptr,i);
                    sv::Svar item;
                    get_value(member.members_, item_ptr, item);
                    child.push_back(item);
                }
                v[member.name_] = child;
            }
            else
            {
                sv::Svar child;
                get_value(member.members_, v_ptr, child);
                v[member.name_] = child;
            }
            break;
        default:
            break;
        }
    }
}
