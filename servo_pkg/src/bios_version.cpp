///////////////////////////////////////////////////////////////////////////////////
//   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          //
//                                                                               //
//   Licensed under the Apache License, Version 2.0 (the "License").             //
//   You may not use this file except in compliance with the License.            //
//   You may obtain a copy of the License at                                     //
//                                                                               //
//       http://www.apache.org/licenses/LICENSE-2.0                              //
//                                                                               //
//   Unless required by applicable law or agreed to in writing, software         //
//   distributed under the License is distributed on an "AS IS" BASIS,           //
//   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    //
//   See the License for the specific language governing permissions and         //
//   limitations under the License.                                              //
///////////////////////////////////////////////////////////////////////////////////

#include "servo_pkg/bios_version.hpp"


namespace PWM {
    BiosVersion::BiosVersion(const std::string &biosVersion, rclcpp::Logger logger_)
    :logger_(logger_)
    {
        int ret = std::sscanf(biosVersion.c_str(), "%d.%d.%d", &first_, &second_, &third_);
        if (ret <=0) {
            RCLCPP_WARN(logger_, "Invalid bios version string, reverting to default");
            std::sscanf(BIOS_CUT_OFF.c_str(), "%d.%d.%d", &first_, &second_, &third_);
        }
    }
    bool BiosVersion::operator >= (const BiosVersion &rhs) {
        if (first_ > rhs.first_) {
            return true;
        }
        if (second_ > rhs.second_) {
            return true;
        }
        if (third_ > rhs.third_) {
            return true;
        }
        if (first_ == rhs.first_
            && second_ == rhs.second_ &&
            third_ == rhs.third_) {
            return true;
        }
        return false;
    }
}
