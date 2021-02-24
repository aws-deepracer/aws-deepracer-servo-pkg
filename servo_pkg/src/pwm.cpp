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

#include "servo_pkg/pwm.hpp"

namespace PWM {
    /// Max size of the character buffer used to concat the file paths.
    #define MAX_BUF 64
    // The following pwm dev id and base path were given by pegatron.
    #define PWMDEV "0000:00:17.0"
    #define BASE_SYS_PATH "/sys/class/pwm/"

    /// Helper method that writes the given value into a specified path.
    /// @param path Path to the file where the value is to be written.
    /// @param value Value to write in the file given by path.
  	void writePWM(const char *path, int value, rclcpp::Logger logger_) {
        int fd, len;
        fd = open(path, O_WRONLY);
        if (fd < 0) {
            RCLCPP_ERROR(logger_, "Failed to open: %s", path);
            return;
        }
        char buf[MAX_BUF];
        len = snprintf(buf, sizeof(buf), "%d", value);
        write(fd, buf, len);
        close(fd);
    }
    /// Rewriting the command to dynamically find the pwmchip%d directory and return the syspath
    /// `ls -al /sys/class/pwm/ | grep "0000:00:17.0" | awk '{ print $9}'`
    /// @returns Syspath pointing to the pwmchip directory.
    const std::string getSysPath(){
        // Set default to pwmchip0 from previous release
        auto syspath = std::string(BASE_SYS_PATH) + std::string("pwmchip0");
        // ls -al /sys/class/pwm/
        for (const auto & entry : std::filesystem::directory_iterator(BASE_SYS_PATH)){
            auto filepath = entry.path();
            if(std::filesystem::exists(filepath) && std::filesystem::is_symlink(filepath)){
                std::string symlinkTarget = std::filesystem::read_symlink(filepath).c_str();
                // grep "0000:00:17.0"
                if (symlinkTarget.find(PWMDEV) != std::string::npos) {
                    auto tmp = symlinkTarget;
                    std::string delimiter = "/";
                    size_t pos = 0;
                    std::string token;
                    size_t tokenCount = 0;
                    // awk '{ print $9}'
                    while ((pos = tmp.find(delimiter)) != std::string::npos && tokenCount < 9) {
                        tmp.erase(0, pos + delimiter.length());
                        tokenCount++;
                    }
                    syspath = std::string(BASE_SYS_PATH) + std::string(tmp);
                }
            }
        }
        return syspath;
    }

    Servo::Servo(int channel, rclcpp::Logger logger_)
      : channel_(channel),
        period_(0),
        duty_(0),
        syspath_(getSysPath()),
	    logger_(logger_)

    {
        RCLCPP_INFO(logger_, "Servo syspath: %s", syspath_.c_str());
        // Export the PWM if it is not already exported.
        char exportPath[MAX_BUF];
        snprintf(exportPath, sizeof(exportPath), (syspath_ + std::string("/pwm%d")).c_str(), channel_);
        struct stat st;
        if (stat(exportPath, &st) == 0) {
            RCLCPP_INFO(logger_, "Servo channel %d has already been exported", channel_);
            return;
        }
        writePWM((syspath_ + std::string("/export")).c_str(), channel_, logger_);
    }

    /// Setter for the PWM period.
    /// @param period Desired period in ms.
    void Servo::setPeriod(int period) {
        char periodPath[MAX_BUF];
        snprintf(periodPath, sizeof(periodPath), (syspath_ + std::string("/pwm%d/period")).c_str(), channel_);
        writePWM(periodPath, period, logger_);
    }

    /// Setter for the duty cycle, this is what determines how much the servo actuates.
    /// @param Desired duty cycle.
    void Servo::setDuty(int duty) {
        char dutyPath[MAX_BUF];
        snprintf(dutyPath, sizeof(dutyPath), (syspath_ + std::string("/pwm%d/duty_cycle")).c_str(), channel_);
        writePWM(dutyPath, duty, logger_);
    }

    /// @returns Current value of the period.
    int Servo::getPeriod() const {
        return period_;
    }

    /// @returns Current value of the duty cycle.
    int Servo::getDuty() const {
        return duty_;
    }
}
