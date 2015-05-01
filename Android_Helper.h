//
// Created by tong on 15-5-1.
//

#ifndef SENSORBASEDSTABILIZATION_ANDROID_HELPER_H
#define SENSORBASEDSTABILIZATION_ANDROID_HELPER_H

#include <string>

template<typename T>
inline string to_string(T value) {
    std::ostringstream os;
    os << value;
    return os.str();
}

#endif //SENSORBASEDSTABILIZATION_ANDROID_HELPER_H
