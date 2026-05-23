//
// Created by clemens on 02.07.22.
//

#ifndef SRC_XESC_INTERFACE_H
#define SRC_XESC_INTERFACE_H

#include <xesc_msgs/XescStateStamped.h>


namespace xesc_interface {
    class XescInterface {
    public:
        virtual void getStatus(xesc_msgs::XescStateStamped &state)=0;
        virtual void getStatusBlocking(xesc_msgs::XescStateStamped &state)=0;
        virtual void setDutyCycle(float duty_cycle)=0;
        virtual void setCurrent(float current)=0;
        virtual void setBrake(float brake)=0;
        virtual void setSpeed(float speed)=0;
        virtual void stop()=0;

    };
}

#endif
