#ifndef DT_ACTION_H
#define DT_ACTION_H

#include <iostream>
#include <string>
#include <utility>
#include <vector>

#ifndef COLOR_UNBOLD
#define COLOR_UNBOLD    "\033[0m"
#endif
#ifndef COLOR_BOLD
#define COLOR_BOLD     "\033[1m"
#endif

class Action
{
public:
  Action(const std::string& action_type, const std::string& action_name = "")
  {
    action_name_ = action_name;
    action_type_ = action_type;
    cpt_ = 0;
  }

  std::string getType() { return action_type_; }
  std::string getName() { return action_name_; }
  void setName(const std::string& action_name) { action_name_ = action_name; }

  void addParameter(const std::string& parameter_type, const std::string& parameter_value)
  {
    parameters_.push_back(std::make_pair(parameter_type, parameter_value));
  }

  void goBegin() { cpt_ = 0; }

  std::pair<std::string, std::string> getParameter()
  {
    return parameters_[cpt_++];
  }

  bool end() { return cpt_ >= parameters_.size(); }

  void display()
  {
    std::cout << COLOR_BOLD << action_name_ << " : " << action_type_ << COLOR_UNBOLD << "(";
    std::string str_parameters;
    for(auto& parameter : parameters_)
    {
      if(str_parameters != "")
        str_parameters += ", ";
      str_parameters += parameter.first + ":" + parameter.second;
    }
    std::cout << str_parameters;
    std::cout << ")" << std::endl;
  }

  std::string toString()
  {
    std::string res;
    res += action_name_ + " : " + action_type_ + "(";
    std::string str_parameters;
    for(auto& parameter : parameters_)
    {
      if(str_parameters != "")
        str_parameters += ", ";
      str_parameters += parameter.first + ":" + parameter.second;
    }
    res += str_parameters + ")";
    return res;
  }


private:
  std::string action_name_;
  std::string action_type_;
  std::vector<std::pair<std::string, std::string>> parameters_;
  size_t cpt_;
};

#endif // DT_ACTION_H
