#ifndef PROCESSOR_H
#define PROCESSOR_H

class Processor {
 public:
  float Utilization(); 
  auto ParseData(float arr[]);
  float Currenttotaltime(){return currenttotaltime;}
  float Prevtotaltime(){return prevtotaltime;}
  float Currentidletime(){return currentidletime;}
  float Currentnonidletime(){return currentnonidletime;}
  float Previdletime(){return previdletime;}
  float Prevnonidletime(){return prevnonidletime;}
  
 private:
    float currenttotaltime;
    float prevtotaltime;
    float currentidletime;
    float previdletime;
    float prevnonidletime;
    float currentnonidletime;
};

#endif