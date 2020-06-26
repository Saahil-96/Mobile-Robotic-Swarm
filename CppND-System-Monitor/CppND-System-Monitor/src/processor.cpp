#include "processor.h"
#include <fstream>
#include <sstream>

// Function called by Utilization Method
auto Processor::ParseData(float arr[])
{
    long value[10],user,nice,system,idle,iowait,irq,softirq,steal;
    std::string line,key ;
    std::ifstream file("/proc/stat");
    if(file.is_open())
    {
        while(std::getline(file,line))
        {
        std::istringstream linestream(line);
linestream>>key>>value[0]>>value[1]>>value[2]>>value[3]>>value[4]>>value[5]>>value[6]>>value[7]>>value[8]>>value[9];
        if (key=="cpu")
        {
            user=value[0];nice=value[1];system=value[2];idle=value[3];iowait=value[4];irq=value[5];softirq=value[6];steal=value[7];
        }
        }
    }

    long totidle=idle+iowait;
    long totnonidle=user+nice+system+irq+softirq+steal;
    arr[0]=totidle;
    arr[1]=totnonidle;
    }

// Parse data about CPU utilization by system
float Processor::Utilization() 
{ 
  float val[2];

  ParseData(val);
  prevnonidletime=val[1]; previdletime=val[0];

  for (int i=0;i<=10000000;i++){} //Delay

  ParseData(val);
  currentnonidletime=val[1];
  currentidletime=val[0];

  currenttotaltime=currentidletime+currentnonidletime;
  prevtotaltime=previdletime+prevnonidletime;

  float Totaltime=((currenttotaltime-prevtotaltime)-(currentidletime-previdletime))/(currenttotaltime-prevtotaltime);
  return Totaltime;
}