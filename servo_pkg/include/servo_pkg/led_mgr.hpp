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

#ifndef LED_MGR_HPP
#define LED_MGR_HPP

#include "rclcpp/rclcpp.hpp"
#include "servo_pkg/pwm.hpp"
#include "servo_pkg/utility.hpp"
#include "deepracer_interfaces_pkg/srv/set_led_ctrl_srv.hpp"
#include "deepracer_interfaces_pkg/srv/get_led_ctrl_srv.hpp"

namespace PWM {
    #define SERVO_PERIOD 20000000
    class LedMgr
        {
        public:
            LedMgr(rclcpp::Logger logger_);
            ~LedMgr();
            /// Pointer to the red, green and blue led pwm
            /// handler for the led requests
            bool setLedCtrlHdl(const std::shared_ptr<rmw_request_id_t> request_header,
                               const std::shared_ptr<deepracer_interfaces_pkg::srv::SetLedCtrlSrv::Request> req,
                               std::shared_ptr<deepracer_interfaces_pkg::srv::SetLedCtrlSrv::Response> res);
                           

            /// Handler for get led values request
            bool getLedCtrlHdl(const std::shared_ptr<rmw_request_id_t> request_header,
                               const std::shared_ptr<deepracer_interfaces_pkg::srv::GetLedCtrlSrv::Request> req,
                               std::shared_ptr<deepracer_interfaces_pkg::srv::GetLedCtrlSrv::Response> res);
 
        private:
            /// Reads the LED values from the desired file and stores the values in memory
            /// @param ledValuesMap Map to store the calibrations read from file
            /// @param filePath Full file path of the file containing the calibrations, note the client verifies existence
            void setLedValFromFile(std::unordered_map<std::string, int> &ledValuesMap,
                                const std::string &filePath);

            /// Writes the given led values map to file
            /// @param ledValuesMap Led values to write to file
            /// @param filePath Full path to the desired file for which to write the given led values map.
            void writeLedValJSON(std::unordered_map<std::string, int> &ledValuesMap,
                            const std::string &filePath);

            std::vector<std::shared_ptr< Servo >> ledVec_;
            /// Hash map that stores the pwm values of each color channel.
            std::unordered_map<std::string, int> ledValuesMap_;
            /// ROS Logger object to log messages.
	        rclcpp::Logger logger_;
    	};
}
#endif
