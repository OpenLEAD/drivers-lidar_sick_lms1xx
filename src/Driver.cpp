#include <lidar_sick_lms1xx/Driver.hpp>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include<base/logging.h>
#include <boost/algorithm/string.hpp>
#include <exception>

using namespace lidar_sick_lms1xx;

int stringHextoInt(std::string hex)
{
    unsigned int uint;
    std::stringstream ss;
    ss << std::hex << hex;
    ss >> uint;
    return  static_cast<int>(uint);
}


Driver::Driver(): iodrivers_base::Driver(1000000)
{
    buffer.resize(1000000);
}

void Driver::open(std::string const& uri)
{
    openURI(uri);
}

int Driver::extractPacket (uint8_t const *buffer, size_t buffer_size) const
{
    if (buffer_size==0)
        return 0;

    if(buffer[0]!=0x02)
        return -1;

    for(size_t pkg_size=1; pkg_size < buffer_size; ++pkg_size)
    {
       if(buffer[pkg_size]==0x03)
           return pkg_size+1;
    }
    return 0;
}

void Driver::processPacket(std::vector<std::string>* processedPacket, int packet_size)
{
    //remove stx and etx
    std::string packet_raw(&buffer[1],&buffer[0]+(packet_size-1));
    boost::split(*processedPacket, packet_raw, boost::is_any_of(" "));
}

void Driver::startMeas()
{
   char msg[100];
   int msg_size = snprintf(msg,100,"%c%s%c", 0x02, "sMN LMCstartmeas", 0x03);

    writePacket(reinterpret_cast<uint8_t const*>(msg),msg_size);
    int packet_size = readPacket(&buffer[0], buffer.size());
    
    std::string ans(&buffer[0],&buffer[0]+packet_size);

    if(ans.find("sAN LMCstartmeas 0")!=std::string::npos)
    {   
        LOG_INFO("Starting Measurements!");
        return;
    }
    else 
        LOG_INFO("Start Measurements: Command not allowed, try again.");
}



void Driver::stopMeas()
{
    char msg[100];
    int msg_size = snprintf(msg,100,"%c%s%c",0x02,"sMN LMCstopmeas",0x03);
    
    writePacket(reinterpret_cast<uint8_t const*>(msg),msg_size); 
    int packet_size = readPacket(&buffer[0],buffer.size());

    std::string ans(&buffer[0],&buffer[0]+packet_size);
 

    if(ans.find("sAN LMCstopmeas 0") != std::string::npos)
    {
        LOG_INFO("Measurements Stopped!");
        return;
    }
    else
    {
        LOG_INFO("Stop Measurement: Command not allowed,try again.");
        return;
    }

}


status_t Driver::queryStatus()
{
    char msg[100];
    int msg_size = snprintf(msg,100,"%c%s%c",0x02,"sRN STlms",0x03);
    
    writePacket(reinterpret_cast<uint8_t const*>(msg),msg_size); 
    int packet_size = readPacket(&buffer[0],buffer.size());
    
    std::vector<std::string> processedPacket;
    processPacket(&processedPacket,packet_size);

    if(processedPacket[1]=="STlms")
    {
        int status = std::atoi(processedPacket[2].data());
        return (status_t) status;
    }    
    else
        throw std::runtime_error("Invalid Packet received!");

}

bool Driver::login()
{
    char msg[100];
    
    /* Ask for Acess Mode as Authorized Client (03)
     * with password F4724744
    */
    int msg_size = snprintf(msg,100, "%c%s%c", 0x02, "sMN SetAccessMode 03 F4724744", 0x03);
    writePacket(reinterpret_cast<uint8_t const*>(msg),msg_size); 
    int packet_size = readPacket(&buffer[0],buffer.size());
    
    std::vector<std::string> processedPacket;
    processPacket(&processedPacket,packet_size);

    if(processedPacket[1]+processedPacket[2]=="SetAccessMode1") 
    {
        LOG_INFO("Login Succeeded!");
        return true;
    }
    else    
    {
        LOG_ERROR("Login not succeeded!");
        return false;
    }
}

scanCfg Driver::getScanCfg()
{
    scanCfg cfg;
    char msg[100];
    int msg_size = snprintf(msg,100,"%c%s%c",0x02,"sRN LMPscancfg",0x03);

    writePacket(reinterpret_cast<uint8_t const*>(msg),msg_size);

    int packet_size = readPacket(&buffer[0],buffer.size());
    
    std::vector<std::string> processedPacket;
    processPacket(&processedPacket,packet_size);
    
    if(processedPacket[1]!="LMPscancfg")
    {
        throw std::runtime_error("Invalid Packet received!");
    }
    else
    {
        cfg.scaningFrequency = std::atoi(processedPacket[2].data());
        cfg.angleResolution = std::atoi(processedPacket[4].data());
        cfg.startAngle = stringHextoInt(processedPacket[5]);
        cfg.stopAngle = stringHextoInt(processedPacket[6]);
        return cfg;
    }

}
bool Driver::setScanCfg(const scanCfg &cfg)
{
    char msg[100];

    int msg_size = snprintf(msg,100,"%c%s %X +1 %X %X %X%c",0x02, "sMN mLMPsetscancfg",cfg.scaningFrequency,cfg.angleResolution,cfg.startAngle,cfg.stopAngle, 0x03);

     writePacket(reinterpret_cast<uint8_t const*>(msg),msg_size); 

     int packet_size = readPacket(&buffer[0],buffer.size()); 

     std::vector<std::string> processedPacket;
     processPacket(&processedPacket,packet_size);
     
     if(processedPacket[2]=="0")
     {
         LOG_INFO("Scan Config sucessfully set");
         return true;
     }
     else
     {
         LOG_ERROR("Error occurred during Scan configuration");
         return false;
     }
}

void Driver::setScanDataCfg(const scanDataCfg &cfg)
{
    char msg[100];
    int msg_size = snprintf(msg,100, "%c%s %02X 00 %d %d 0 %02X 00 %d %d 0 %d +%d%c",0x02,
            "sWN LMDscandatacfg", cfg.outputChannel, cfg.remission ? 1 : 0,
            cfg.resolution, cfg.encoder, cfg.position ? 1 : 0, cfg.deviceName ? 1 : 0,
            cfg.timestamp ? 1 : 0, cfg.outputInterval, 0x03);
    
     writePacket(reinterpret_cast<uint8_t const*>(msg),msg_size); 

     int packet_size = readPacket(&buffer[0],buffer.size()); 

     std::vector<std::string> processedPacket;
     processPacket(&processedPacket,packet_size);

     if(processedPacket[2]!="LMDscandatacfg")
         throw std::runtime_error("Invalid Packet received!");

}

scanOutputRange Driver::getscanOutputRange()
{
    scanOutputRange outputRange;
    char msg[100];
    int msg_size = snprintf(msg,100, "%c%s%c", 0x02, "sRN LMPoutputRange", 0x03);

    writePacket(reinterpret_cast<uint8_t const*>(msg),msg_size);

    int packet_size = readPacket(&buffer[0],buffer.size());
     
    std::vector<std::string> processedPacket;
    processPacket(&processedPacket,packet_size);
    
    if(processedPacket[2]!="LMPoutputRange")
    {
        throw std::runtime_error("Invalid Packet received!");
    }
    else
    {
        outputRange.angleResolution = std::atoi(processedPacket[3].data());
        outputRange.startAngle = stringHextoInt(processedPacket[4]);
        outputRange.stopAngle = stringHextoInt(processedPacket[5]);

        return outputRange;
    }    
}

void Driver::scanContinous(int start)
{
    char msg[100];
    int msg_size = snprintf(msg,100, "%c%s %d%c", 0x02, "sEN LMDscandata", start, 0x03);

    writePacket(reinterpret_cast<uint8_t const*>(msg),msg_size);

    int packet_size = readPacket(&buffer[0],buffer.size());
    
     std::vector<std::string> processedPacket;
     processPacket(&processedPacket,packet_size);
     
    if(processedPacket[2]=="0")
    {
        LOG_INFO("Scan Continous: STOP");
    }
    else
    {
        LOG_INFO("Scan Continous: START");
    }



}

bool Driver::saveConfig()
{
    char msg[100];
    int msg_size = snprintf(msg,100, "%c%s%c", 0x02, "sMN mEEwriteall", 0x03);

    writePacket(reinterpret_cast<uint8_t const*>(msg),msg_size);
    int packet_size = readPacket(&buffer[0],buffer.size());

    std::vector<std::string> processedPacket;
    processPacket(&processedPacket,packet_size);

    if(processedPacket[2]=="1")
    {
        LOG_INFO("Configurations permanently saved on device");
        return true;
    }
    else
    {
        LOG_INFO("Error saving configuration permanently");
        return false;
    }


}

bool Driver::startDevice()
{
    char msg[100];
    int msg_size = snprintf(msg,100, "%c%s%c", 0x02, "sMN Run", 0x03);

    writePacket(reinterpret_cast<uint8_t const*>(msg),msg_size);
    int packet_size = readPacket(&buffer[0],buffer.size());

    std::vector<std::string> processedPacket;
    processPacket(&processedPacket,packet_size);

    if(processedPacket[2]=="1")
    {
        LOG_INFO("Device sucessfully started");
        return true;
    }
    else
    {
        LOG_INFO("Error occurred starting device");
        return false;
    }

}

void Driver::getData(scanData& data)
{
    int packet_size = readPacket(&buffer[0],buffer.size());

    std::vector<std::string> processedPacket;
    processPacket(&processedPacket,packet_size);
    int numberEncodersPos = 16; //TODO 
    int numberEncoders = std::atoi(processedPacket[numberEncodersPos].data());
    
    //the number of enconder vary between 0 and 3, so the position of the 
    //number of 16Bit channel information depends on this number. The
    //information is the string after the last encoder info. 
    int numberChannelsXBitPos=numberEncodersPos+2*numberEncoders+1;    
    int numberChannelsXBit = std::atoi(processedPacket[numberChannelsXBitPos].data());
       
    if(numberChannelsXBit>0)
    {
        LOG_DEBUG("Number of channels 16Bits: %d", numberChannelsXBit);
    }
    else
    {   
        //if there is no data in 16 bits,all the content from this channels is
        //blank, so the next information is the number of channels in 8bit.
        numberChannelsXBitPos = numberChannelsXBitPos+1;
        numberChannelsXBit = std::atoi(processedPacket[numberChannelsXBitPos].data());
        LOG_DEBUG("Number of channels 8Bits: %d", numberChannelsXBit);
        if(numberChannelsXBit==0)
            throw  std::runtime_error("There is neither 16Bit nor 8Bit, an error must be occurrred");
    }
    int measDataContentPos = numberChannelsXBitPos+1;
    int measDataPos;   

    for(int i=0; i<numberChannelsXBit;i++)
    {
        std::string measDataContent = processedPacket[measDataContentPos];
        
        //number of measurements
        int numberDataPos = measDataContentPos+5;
        int numberData = stringHextoInt(processedPacket[numberDataPos]);
        
        if(measDataContent=="DIST1"){
            data.dist_len1 = numberData;
        }
        else if(measDataContent=="DIST2"){
            data.dist_len2 = numberData;
        }
        else if(measDataContent=="RSSI1"){
            data.rssi_len1 = numberData;
        } else if(measDataContent=="RSSI2"){
            data.rssi_len2 = numberData;
        } else {
             throw std::runtime_error("Invalid Packet received!");
        }

        for(int j=0; j<numberData; j++)
        {
            measDataPos = numberDataPos+j+1;
            int measData = stringHextoInt(processedPacket[measDataPos]);
             if(measDataContent=="DIST1"){
                data.dist1[j] = measData;
             } else if(measDataContent=="DIST2"){
                data.dist2[j] = measData;
             } else if(measDataContent=="RSSI1"){
                 data.rssi1[j] = measData;
             } else if(measDataContent=="RSSI2"){
                data.rssi2[j] = measData;
             }        
        }
        measDataContentPos = measDataContentPos + numberData + 6;
    }
}























