#include <dirent.h>
#include <unistd.h>
#include <string>
#include <vector>

#include "linux_parser.h"

using std::stof;
using std::string;
using std::to_string;
using std::vector;

// Parses the name of the Operating System from files
string LinuxParser::OperatingSystem() {
  string line;
  string key;
  string value;
  std::ifstream filestream(kOSPath);
  if (filestream.is_open()) {
    while (std::getline(filestream, line)) {
      std::replace(line.begin(), line.end(), ' ', '_');
      std::replace(line.begin(), line.end(), '=', ' ');
      std::replace(line.begin(), line.end(), '"', ' ');
      std::istringstream linestream(line);
      while (linestream >> key >> value) {
        if (key == "PRETTY_NAME") {
          std::replace(value.begin(), value.end(), '_', ' ');
          return value;
        }
      }
    }
  }
  return value;
}

// Parses the name of the Kernel from files
string LinuxParser::Kernel() {
  string os, kernel;
  string line;
  std::ifstream stream(kProcDirectory + kVersionFilename);
  if (stream.is_open()) {
    std::getline(stream, line);
    std::istringstream linestream(line);
    linestream >> os >> kernel;
  }
  return kernel;
}

// Parses all Process IDs from files and stores them in a vector
vector<int> LinuxParser::Pids() {
  vector<int> pids;
  DIR* directory = opendir(kProcDirectory.c_str());
  struct dirent* file;
  while ((file = readdir(directory)) != nullptr) {
    // Is this a directory?
    if (file->d_type == DT_DIR) {
      // Is every character of the name a digit?
      string filename(file->d_name);
      if (std::all_of(filename.begin(), filename.end(), isdigit)) {
        int pid = stoi(filename);
        pids.push_back(pid);
      }
    }
  }
  closedir(directory);
  return pids;
}

// Parses Memory Utilization data of system from files
float LinuxParser::MemoryUtilization() {  std::string line, key;
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
  return memfin;  
  }

// Parses the up time of the system from files
long LinuxParser::UpTime() { 
  long uptime;
  std::string line;
  std::ifstream stream("/proc/uptime");
  if (stream.is_open()){
    std::getline(stream, line);
    std::istringstream linestream(line);
    linestream >> uptime;
  }
  return uptime;  
}

// Parses the total Jiffies of the system from files
long LinuxParser::Jiffies() { 
vector<string> cpu_util=LinuxParser::CpuUtilization();
long total=stol(cpu_util[3],nullptr,10)+stol(cpu_util[4],nullptr,10)+stol(cpu_util[0],nullptr,10)+stol(cpu_util[1],nullptr,10)+stol(cpu_util[2],nullptr,10)+stol(cpu_util[5],nullptr,10)+stol(cpu_util[6],nullptr,10)+stol(cpu_util[7],nullptr,10);
return total;
}

// Parses the total active Jiffies of the specified process from files
long LinuxParser::ActiveJiffies(int pid) { 
 std::string line,key;
 long value,utime,stime,cutime,cstime;
 long totaltime;
 std::ifstream CpuUti("/proc/"+std::to_string(pid)+"/stat");
 if(CpuUti.is_open())
 {
   while(getline(CpuUti,line))
   {
     std::istringstream linestream(line);
     linestream>>value;
     linestream>>key;
     linestream>>key;
     for(int i =1; i<=10;i++)
     {linestream>>value;}
     linestream>>utime;
     linestream>>stime;
     linestream>>cutime;
     linestream>>cstime;
   }
 }
 totaltime=utime+stime+cutime+cstime;
 return totaltime; 
}

// Parses the total active Jiffies of the system from files
long LinuxParser::ActiveJiffies() { 
  vector<string> cpu_util=LinuxParser::CpuUtilization();
long nonidle=stol(cpu_util[0],nullptr,10)+stol(cpu_util[1],nullptr,10)+stol(cpu_util[2],nullptr,10)+stol(cpu_util[5],nullptr,10)+stol(cpu_util[6],nullptr,10)+stol(cpu_util[7],nullptr,10);
return nonidle;
}

// Parses the total idle Jiffies of the system from files
long LinuxParser::IdleJiffies() {
vector<string> cpu_util=LinuxParser::CpuUtilization();
long idle=stol(cpu_util[3],nullptr,10)+stol(cpu_util[4],nullptr,10);
return idle;
}

// Parses all CPU utilization data from files and stores it in a vector of strings
vector<string> LinuxParser::CpuUtilization() { 
  string key, line; 
  long value[10];
  vector<string> ret_val;
  
  std::ifstream cpuutil("/proc/stat");
  if (cpuutil.is_open())
  {
    while(std::getline(cpuutil,line))
    {
    	std::istringstream linestream(line); linestream>>key>>value[0]>>value[1]>>value[2]>>value[3]>>value[4]>>value[5]>>value[6]>>value[7]>>value[8]>>value[9];
      if(key=="cpu")
      {
        for(int i=0; i<10; i++)
        {ret_val.push_back(to_string(value[i]));}
        return ret_val;
      }
      else
      {
		continue;
      }
    }
  }
  return ret_val; 
}

// Parses number of total processes in system from files
int LinuxParser::TotalProcesses() { 
  std::ifstream TotProc("/proc/stat");
  std::string line, key;
  int value;
  if(TotProc.is_open())
  {
   while(std::getline(TotProc,line))
   {
     std::istringstream linestream(line);
     linestream>>key>>value;
     if(key=="processes")
     {return value;}
   }
  }
  return value; }

// Parses number of total running processes in system from files
int LinuxParser::RunningProcesses() { 
  std::ifstream RunProc("/proc/stat");
  std::string line, key;
  int value;
    if (RunProc.is_open())
    {
      while (std::getline(RunProc, line))
      {
      std::istringstream linestream(line);
        while(linestream>>key>>value)
        {
          if(key=="procs_running")
          {
          return value;
          }
        }
      }
    }
  return value; }

// Parses command that started each process from files
string LinuxParser::Command(int pid) {
  std::string line, fin=" "; 
  std::ifstream cmd("/proc/"+std::to_string(pid)+"/cmdline");
  if(cmd.is_open())
  {
    std::getline(cmd,line);
  }
  return line;
}

// Parses RAM of specified process from files
string LinuxParser::Ram(int pid) { 
  std::string line, key, ret_val;
  double value;
  std::ifstream RAM("/proc/"+std::to_string(pid)+"/status");
  if(RAM.is_open())
  	{
     while(std::getline(RAM,line))
      {
        std::istringstream linestream(line);
        linestream>>key>>value;
        value=value/1000;
        if(key=="VmSize:")
        {
          ret_val=to_string(value);
        }
      }
    }
  
//   Remove excess trailing zeros
  for(int i=ret_val.size()-1;i>=0;i--)
  {
  	if (ret_val[i]=='0')
    {
      ret_val.erase(ret_val.begin()+i);
    }
    else{
      break;
    }
  }
  
  return ret_val; 
}

// Parses user ID of specified process from files
string LinuxParser::Uid(int pid) { 
  std::string line, key;
  int value, user_id=0;
  std::ifstream User("/proc/"+std::to_string(pid)+"/status");
  if(User.is_open())
  	{
     while(std::getline(User,line))
      {
        std::istringstream linestream(line);
        linestream>>key>>value;
        if(key=="Uid:")
        {
          user_id=value;
        }
      }
    } 
  return to_string(user_id);
 }

// Parses user name of specified process from files
string LinuxParser::User(int pid) { 
  std::string line, key, user, user_id, tmp, check;
  int count=0, user_id_num=0, i=0;
  
  user_id=LinuxParser::Uid(pid);
  user_id_num=stoi(user_id);
 std::ifstream User_name("/etc/passwd");
 if(User_name.is_open())
  	{
     while(std::getline(User_name,line))
      {
        std::istringstream linestream(line);
        linestream>>key;
        while(count<=2)
          {
            if(key[i]==58)
            {
              count++;
            }
    		if(count==0)
            {
              tmp=tmp+key[i]; 
            }
            i++;
            if(count==2)
            {
              check=check + key[i]; 
            }
          }
       
       if(check.compare(std::to_string(user_id_num))!=0)
       {
         user=tmp;
         break;
       }
       else
       {
         tmp=" ";
         user_id_num=0;
         check=" ";
         continue;
       }
      }
     }   
  
 return user; 
}

// Parses up time of specified process from files
long LinuxParser::UpTime(int pid) { 
 std::string line,key;
 long value, uptime;
 float starttime;
 std::ifstream CpuUti("/proc/"+std::to_string(pid)+"/stat");
 if(CpuUti.is_open())
 {
   while(getline(CpuUti,line))
   {
     std::istringstream linestream(line);
     uptime=LinuxParser::UpTime();
     linestream>>value;
     linestream>>key;
     linestream>>key;
     for(int i =1; i<=18;i++)
     {linestream>>value;}
     linestream>>starttime;
   }
 }
  uptime=LinuxParser::UpTime()-starttime/sysconf(_SC_CLK_TCK);
 return (long) uptime; 
}