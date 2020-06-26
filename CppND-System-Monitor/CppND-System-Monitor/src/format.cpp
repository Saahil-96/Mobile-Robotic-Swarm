#include <string>

#include "format.h"

using std::string;

string Format::ElapsedTime(long seconds) 
{  
   int hours, minutes, secs;
   string Hours, Minutes, Secs;
   hours=seconds/3600;
   minutes=(seconds-(hours*3600))/60;
   secs=(seconds-(hours*3600)-(minutes*60));
   
   Hours=std::to_string(hours);
   Minutes=std::to_string(minutes);
   Secs=std::to_string(secs);
  
//   Add zeros for proper formatting of single digit instances
   if(hours<10){Hours="0"+Hours;}
   if(minutes<10){Minutes="0"+Minutes;}
   if(secs<10){Secs="0"+Secs;}
  
//   Final string
   string time=Hours+":"+Minutes+":"+Secs;
   return time; 
}