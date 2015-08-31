#ifndef LIDAR_SICK_LMS1XX_HPP
#define LIDAR_SICK_LMS1XX_HPP


#include <iodrivers_base/Driver.hpp>

namespace lidar_sick_lms1xx
{
    /*!
* @class scanCfg
* @brief Structure containing scan configuration.
*
* @author Konrad Banachowicz
*/
typedef struct _scanCfg {
	/*!
	 * @brief Scanning frequency.
	 * 1/100 Hz
	 */
	int scaningFrequency;

	/*!
	 * @brief Scanning resolution.
	 * 1/10000 degree
	 */
	int angleResolution;

	/*!
	 * @brief Start angle.
	 * 1/10000 degree
	 */
	int startAngle;

	/*!
	 * @brief Stop angle.
	 * 1/10000 degree
	 */
	int stopAngle;
} scanCfg;

/*!
* @class scanDataCfg
* @brief Structure containing scan data configuration.
*
* @author Konrad Banachowicz
*/
typedef struct _scanDataCfg {

	/*!
	 * @brief Output channels.
	 * Defines which output channel is activated.
	 */
	int outputChannel;

	/*!
	 * @brief Remission.
	 * Defines whether remission values are output.
	 */
	bool remission;

	/*!
	 * @brief Remission resolution.
	 * Defines whether the remission values are output with 8-bit or 16bit resolution.
	 */
	int resolution;

	/*!
	 * @brief Encoders channels.
	 * Defines which output channel is activated.
	 */
	int encoder;

	/*!
	 * @brief Position.
	 * Defines whether position values are output.
	 */
	bool position;

	/*!
	 * @brief Device name.
	 * Determines whether the device name is to be output.
	 */
	bool deviceName;

	bool timestamp;

	/*!
	 * @brief Output interval.
	 * Defines which scan is output.
	 *
	 * 01 every scan\n
	 * 02 every 2nd scan\n
	 * ...\n
	 * 50000 every 50000th scan
	 */
	int outputInterval;
} scanDataCfg;

/*!
* @class outputRange
* @brief Structure containing scan output range configuration
*
* @author wpd
*/
typedef struct _scanOutputRange {
	/*!
	 * @brief Scanning resolution.
	 * 1/10000 degree
	 */
	int angleResolution;

	/*!
	 * @brief Start angle.
	 * 1/10000 degree
	 */
	int startAngle;

	/*!
	 * @brief Stop angle.
	 * 1/10000 degree
	 */
	int stopAngle;
} scanOutputRange;
/*!
* @class scanData
* @brief Structure containing single scan message.
*
* @author Konrad Banachowicz
*/
typedef struct _scanData {

	/*!
	 * @brief Number of samples in dist1.
	 *
	 */
	int dist_len1;

	/*!
	 * @brief Radial distance for the first reflected pulse
	 *
	 */
	uint16_t dist1[1082];

	/*!
	 * @brief Number of samples in dist2.
	 *
	 */
	int dist_len2;

	/*!
	 * @brief Radial distance for the second reflected pulse
	 *
	 */
	uint16_t dist2[1082];

	/*!
	 * @brief Number of samples in rssi1.
	 *
	 */
	int rssi_len1;

	/*!
	 * @brief Remission values for the first reflected pulse
	 *
	 */
	uint16_t rssi1[1082];

	/*!
	 * @brief Number of samples in rssi2.
	 *
	 */
	int rssi_len2;

	/*!
	 * @brief Remission values for the second reflected pulse
	 *
	 */
	uint16_t rssi2[1082];
} scanData;

typedef enum{
	undefined = 0,
	initialisation = 1,
	configuration = 2,
	idle = 3,
	rotated = 4,
	in_preparation = 5,
	ready = 6,
	ready_for_measurement = 7
} status_t;

/*!
* @class LMS1xx
* @brief Class responsible for communicating with LMS1xx device.
*
* @author Konrad Banachowicz
*/

    
    class Driver : public iodrivers_base::Driver
    {   //std::vector<uint8_t> buffer;
        
        void processPacket(std::vector<std::string>* processedPacket, int packet_size);

        int extractPacket (uint8_t const *buffer, size_t buffer_size) const; 
        public:
            Driver();
            std::vector<uint8_t> buffer;
            void open(std::string const& uri);
            void startMeas();
            void stopMeas();
            status_t queryStatus();
            bool login();
            scanCfg getScanCfg(); 
            bool setScanCfg(const scanCfg &cfg);
            void setScanDataCfg(const scanDataCfg &cfg);
            scanOutputRange getscanOutputRange();
            void scanContinous(int start);
            bool saveConfig();
            bool startDevice();
            void getData(scanData& data);
            



            void read();
    };
}

#endif

