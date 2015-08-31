#include <lidar_sick_lms1xx/Driver.hpp>
#include <iostream>
#include <string.h>
#include <boost/algorithm/string.hpp>
#include <vector>
#include <sstream>

using namespace lidar_sick_lms1xx;

int stringHextoInt(std::string hex)
{

    unsigned int uint;
    std::stringstream ss;
    ss << std::hex << hex;
    ss >> uint;

    return  static_cast<int>(uint);
}



void processMsg(std::vector<std::string>* msgp, int packet_size)
{
    char msg[100];
    int msg_size = snprintf(msg,100, "%c%s%c", 0x02, "sAN mLMPsetscancfg 1 1388 1 1388 FFF92230 225510", 0x03);
    uint8_t const* buf = reinterpret_cast<uint8_t const*>(msg);
    //remove stx and etx
    std::vector<uint8_t> buffer(&buf[1],&buf[0]+(msg_size-1));
    std::string msg_raw(&buffer[0],&buffer[0]+packet_size);

    std::vector<std::string> msg_processed;

    boost::split(*msgp, msg_raw, boost::is_any_of(" "), boost::token_compress_on );
    //return &msgp;
}


int main(int argc, char const* argv[])
{

    lidar_sick_lms1xx::Driver laser;
    
    char msg[100];
    //int msg_size = snprintf(msg,100,"%c%s%c", 0x02, "sAN LMCstartmeas 0", 0x03);

    int msg_size = snprintf(msg,100, "%c%s%c", 0x02, "sAN mLMPsetscancfg 1 1388 1 1388 FFF92230 225510", 0x03);
    
    //int msg_size = snprintf(msg,100,"%c%s%c",0x02,"sRA STlms 7 0 8 16:36:54 8 17.03.2030 0 0 0",0x03);
    uint8_t const* buf = reinterpret_cast<uint8_t const*>(msg);
   
    std::vector<uint8_t> buffer(&buf[0],&buf[0]+msg_size);
  

    for(int i=0; i<buffer.size(); ++i)
        std::cout << buffer[i];
   
  
    std::string ans(&buffer[0],&buffer[0]+msg_size);
    

    std::vector<std::string> ans_parts;
    processMsg(&ans_parts,msg_size); 
    

    std::cout <<"\n" <<stringHextoInt(ans_parts[6].data()) << std::endl;
    
      
    return 0;
}

