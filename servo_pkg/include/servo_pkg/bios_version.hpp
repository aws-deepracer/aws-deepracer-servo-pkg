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

#ifndef BIOS_VERSION_HPP
#define BIOS_VERSION_HPP

#include "rclcpp/rclcpp.hpp"


namespace PWM {
 /// This is the hardware rev where we need to start checking
 /// calibration file because of a manufacturing polarity issue
 const std::string BIOS_CUT_OFF = "0.0.8";
 class BiosVersion {
    /// Helper class for comparing bios versions
    public:
        BiosVersion(const std::string &biosVersion, rclcpp::Logger logger_);
        bool operator >= (const BiosVersion &rhs);
    private:
        /// Bios version is comprised of three digits first.second.thirds
        /// version comparaon starts with the first digit
        int first_;
        int second_;
        int third_;
        /// ROS Logger object to log messages.
        rclcpp::Logger logger_;
    };
}
#endif
