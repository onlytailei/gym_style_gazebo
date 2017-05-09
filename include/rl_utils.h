/*************************************************************************
	> File Name: rl_utils.h
	> Author: TAI Lei
	> Mail: lei.tai@my.cityu.edu.hk
	> Created Time: Di 09 Mai 2017 18:39:58 CEST
 ************************************************************************/

#ifndef _RL_UTILS_H
#define _RL_UTILS_H

// TODO build a base class
// with two functions
// rewardCal
// terminalCheck

// TODO build a inherit class to get that

namespace RL{
  class RL_Utils{
    public:
      RL_Utils();
      
      virtual float rewardCalculate() const =0; 
      virtual bool terminalCheck() const =0;
  };

  class DynamicNavi_RL_Utils : public RL_Utils{
    public: 
      DynamicNavi_RL_Utils();

      virtual float rewardCalculate() const;
      virtual bool terminalCheck() const;
  };
}
#endif
