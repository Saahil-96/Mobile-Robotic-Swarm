#include <unistd.h>
#include <cstddef>
#include <set>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>

#include "process.h"
#include "processor.h"
#include "system.h"
#include "linux_parser.h"

using std::set;
using std::size_t;
using std::string;
using std::vector;

// TODO: Return the system's CPU
Processor& System::Cpu() { return cpu_; }

// Return a vector of all the system's processes with each object instantiated by each process ID
vector<Process>& System::Processes() 
{ vector<int> pids=LinuxParser::Pids();
   for (auto i=0;i<(int)pids.size();i++)
   {
     Process process;
     process.pid=pids[i];
     processes_.push_back(process);
   }
  return processes_; 
}

// Return Kernel of system
std::string System::Kernel() 
{ 
  std::string os,line,version,kernel;
   std::ifstream file("/proc/version");
  if(file.is_open())
  {
      std::getline(file,line);
      std::istringstream linestream(line);
      linestream >> os >> version >> kernel;
  }
  return kernel; }

// Return memory utilization of system
float System::MemoryUtilization() 
{ 
  std::string line, key;
  double memtotal,memfree, value;
  float memfin;
  std::ifstream MemUti("/proc/meminfo");
  if (MemUti.is_open()){
  while(std::getline(MemUti,line)){
   std::istringstream linestream(line);
   linestream>>key>>value;
      if (key=="MemTotal:")
       {memtotal=value;}
      else if (key=="MemFree:")
       {memfree=value;}
   }
  }
  memfin=(memtotal-memfree)/memtotal;
  return memfin; }

// Return Operating System
std::string System::OperatingSystem() 
{ return LinuxParser::OperatingSystem(); }

// Return number of running processes
int System::RunningProcesses() 
{ return LinuxParser::RunningProcesses(); }

// Return number of total processes
int System::TotalProcesses() 
{ return LinuxParser::TotalProcesses(); }

// Return system up time
long int System::UpTime() 
{ return LinuxParser::UpTime(); }