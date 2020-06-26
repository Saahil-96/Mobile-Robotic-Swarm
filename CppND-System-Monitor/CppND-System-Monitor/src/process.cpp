#include <unistd.h>
#include <cctype>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>
#include <fstream>
#include <time.h>
#include "process.h"
#include "linux_parser.h"

#include <iostream>

using std::string;
using std::to_string;
using std::vector;

// Constructor
Process::Process(){
user = LinuxParser::User(pid);
command = LinuxParser::Command(pid);
ram = LinuxParser::Ram(pid);
upTime = LinuxParser::UpTime(pid);
}

// Return this process' PID
int Process::Pid() 
{ return pid; }

// Return this process' CPU Utilization
float Process::CpuUtilization() {
 std::string line,key;
 long value,uptime,utime,stime,cutime,cstime,starttime;
 float totaltime, seconds;
 float cpuusage;
 std::ifstream CpuUti("/proc/"+std::to_string(this->pid)+"/stat");
 if(CpuUti.is_open())
 {
   while(getline(CpuUti,line))
   {
     std::istringstream linestream(line);
     uptime=LinuxParser::UpTime();
     linestream>>value;
     linestream>>key;
     linestream>>key;
     for(int i =1; i<=10;i++)
     {linestream>>value;}
     linestream>>utime;
     linestream>>stime;
     linestream>>cutime;
     linestream>>cstime;
     for(int j =1; j<=4;j++)
     {linestream>>value;}
     linestream>>starttime;
   }
 }
 totaltime=utime+stime+cutime+cstime;
 seconds=uptime-(starttime/sysconf(_SC_CLK_TCK));
 cpuusage=100*((totaltime/sysconf(_SC_CLK_TCK))/seconds);
 return cpuusage; 
}

// Return this process' command line
string Process::Command() 
{ return LinuxParser::Command(this->pid); }

// Return this process' RAM
string Process::Ram() 
{ return LinuxParser::Ram(this->pid); }

// Return this process' User
string Process::User() 
{ return LinuxParser::User(this->pid);}

// Return this process' up time
long int Process::UpTime() 
{ return LinuxParser::UpTime(this->pid);}

// Overload the "less than" comparison operator for Process objects
bool Process::operator<(Process const& a) const { return cpuUtil<a.cpuUtil; }