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


#include "servo_pkg/led_mgr.hpp"


namespace PWM {
    // LED defines
    #define LED_OFF 0
    #define LED_ON 10000000

    // Path to LED values file
    const std::string LED_VAL_PATH = "/opt/aws/deepracer/led_values.json";

    // Keys for the LED values file
    const std::string LED_HEADER_KEY = "Led Values";
    const std::string RED_CHANNEL_KEY = "red_pwm";
    const std::string GREEN_CHANNEL_KEY = "green_pwm";
    const std::string BLUE_CHANNEL_KEY = "blue_pwm";


    enum LEDColors
    {
        red,
        green,
        blue,
        totalColor
    };
   
    LedMgr::LedMgr(rclcpp::Logger logger_)
    :logger_(logger_)
    {
        ledVec_ = { (std::make_shared<Servo>(2, logger_)), (std::make_shared<Servo>(3, logger_)), (std::make_shared<Servo>(4, logger_))};
        ledValuesMap_ = {{{RED_CHANNEL_KEY, LED_OFF},{GREEN_CHANNEL_KEY, LED_OFF},
                                    {BLUE_CHANNEL_KEY, LED_ON}}}; // Default

        RCLCPP_INFO(logger_, "LedMgr pwm channel creation");
        if (checkFile(LED_VAL_PATH)) {
            setLedValFromFile(ledValuesMap_, LED_VAL_PATH);
        }
        else {
            writeLedValJSON(ledValuesMap_, LED_VAL_PATH);
        }

        std::string colorVec[3] = {RED_CHANNEL_KEY, GREEN_CHANNEL_KEY, BLUE_CHANNEL_KEY};
        for (size_t color=0; color<ledVec_.size(); ++color) {
            ledVec_[color]->setPeriod(SERVO_PERIOD);
            ledVec_[color]->setDuty(ledValuesMap_[colorVec[color]]);
        }
    }

    LedMgr::~LedMgr()
    {
        for(auto color : ledVec_) {
            color->setDuty(LED_OFF);
        }
    }

    /// Pointer to the red, green and blue led pwm
    /// handler for the led requests
    bool LedMgr::setLedCtrlHdl(const std::shared_ptr<rmw_request_id_t> request_header,
                               const std::shared_ptr<deepracer_interfaces_pkg::srv::SetLedCtrlSrv::Request> req,
                               std::shared_ptr<deepracer_interfaces_pkg::srv::SetLedCtrlSrv::Response> res) {
        (void)request_header;
        RCLCPP_INFO(logger_, "LedMgr pwm channel on R-G-B %d-%d-%d", req->red, req->green, req->blue);
        res->error = 0;
        if (req->red < 0 || req->red > 10000000 ||
                req->green < 0 || req->green > 10000000 ||
                req->blue < 0 || req->blue > 10000000) {
            RCLCPP_ERROR(logger_, "Invalid LED color combination: R-G-B %d-%d-%d", req->red, req->green, req->blue);
            res->error = 1;
            return false;
        }

        ledVec_[red]->setDuty(req->red);
        ledVec_[green]->setDuty(req->green);
        ledVec_[blue]->setDuty(req->blue);

        // Write the updated values in the file
        ledValuesMap_[RED_CHANNEL_KEY] = req->red;
        ledValuesMap_[GREEN_CHANNEL_KEY] = req->green;
        ledValuesMap_[BLUE_CHANNEL_KEY] = req->blue;

        writeLedValJSON(ledValuesMap_, LED_VAL_PATH);

        return true;
    }

    /// Handler for get led values request
    bool LedMgr::getLedCtrlHdl(const std::shared_ptr<rmw_request_id_t> request_header,
                               const std::shared_ptr<deepracer_interfaces_pkg::srv::GetLedCtrlSrv::Request> req,
                               std::shared_ptr<deepracer_interfaces_pkg::srv::GetLedCtrlSrv::Response> res) {

        (void)request_header;
        (void)req;
        if (ledValuesMap_.find(RED_CHANNEL_KEY) == ledValuesMap_.end() || ledValuesMap_.find(GREEN_CHANNEL_KEY) == ledValuesMap_.end() || ledValuesMap_.find(BLUE_CHANNEL_KEY) == ledValuesMap_.end()) {
            RCLCPP_ERROR(logger_, "Invalid channel values type");
            return false;
        }
        res->red = ledValuesMap_[RED_CHANNEL_KEY];
        res->green = ledValuesMap_[GREEN_CHANNEL_KEY];
        res->blue = ledValuesMap_[BLUE_CHANNEL_KEY];
        return true;
    }

    /// Reads the LED values from the desired file and stores the values in memory
    /// @param ledValuesMap Map to store the calibrations read from file
    /// @param filePath Full file path of the file containing the calibrations, note the client verifies existence
    void LedMgr::setLedValFromFile(std::unordered_map<std::string, int> &ledValuesMap,
                        const std::string &filePath) {
        Json::Value ledJsonValue;
        Json::Reader reader;
        std::ifstream ifs(filePath);

        if (!reader.parse(ifs, ledJsonValue)) {
            RCLCPP_ERROR(logger_, "Error parsing led_value.json");
            return;
        }
        if (!ledJsonValue.isMember(LED_HEADER_KEY)) {
            RCLCPP_ERROR(logger_, "Led value file error: No led header");
            return;
        }
        if (!ledJsonValue[LED_HEADER_KEY].isMember(RED_CHANNEL_KEY) || !ledJsonValue[LED_HEADER_KEY].isMember(GREEN_CHANNEL_KEY) || !ledJsonValue[LED_HEADER_KEY].isMember(BLUE_CHANNEL_KEY)) {
            RCLCPP_ERROR(logger_, "Led value file error: Missing channel keys");
            return;
        }
        ledValuesMap[RED_CHANNEL_KEY] = ledJsonValue[LED_HEADER_KEY][RED_CHANNEL_KEY].asInt();
        ledValuesMap[GREEN_CHANNEL_KEY] = ledJsonValue[LED_HEADER_KEY][GREEN_CHANNEL_KEY].asInt();
        ledValuesMap[BLUE_CHANNEL_KEY] = ledJsonValue[LED_HEADER_KEY][BLUE_CHANNEL_KEY].asInt();
    }

    /// Writes the given led values map to file
    /// @param ledValuesMap Led values to write to file
    /// @param filePath Full path to the desired file for which to write the given led values map.
    void LedMgr::writeLedValJSON(std::unordered_map<std::string, int> &ledValuesMap,
                    const std::string &filePath) {
        Json::Value ledValuesJsonValue;

        if (ledValuesMap.find(RED_CHANNEL_KEY) != ledValuesMap.end() && ledValuesMap.find(GREEN_CHANNEL_KEY) != ledValuesMap.end() && ledValuesMap.find(BLUE_CHANNEL_KEY) != ledValuesMap.end()) {
            ledValuesJsonValue[LED_HEADER_KEY][RED_CHANNEL_KEY] = ledValuesMap[RED_CHANNEL_KEY];
            ledValuesJsonValue[LED_HEADER_KEY][GREEN_CHANNEL_KEY] = ledValuesMap[GREEN_CHANNEL_KEY];
            ledValuesJsonValue[LED_HEADER_KEY][BLUE_CHANNEL_KEY] = ledValuesMap[BLUE_CHANNEL_KEY];
        }
        else {
            RCLCPP_ERROR(logger_, "Invalid Led values map");
            return;
        }
        writeJSONToFile(ledValuesJsonValue, filePath);
    }
}
