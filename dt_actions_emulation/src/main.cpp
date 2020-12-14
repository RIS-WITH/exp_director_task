#include "ActionFileReader.h"
#include "Menu.h"
#include "Parameters.h"

#include <ros/ros.h>
#include <iostream>
#include <algorithm>

#include "ontologenius/OntologyManipulator.h"

#include "dt_actions_emulation/MementarAction.h"
#include "std_msgs/String.h"

OntologyManipulator* onto_;
ros::Publisher* timeline_pub_;
size_t cpt;

void sendAction(const std::string& action_name)
{
  dt_actions_emulation::MementarAction msg;
  std::cout << "send action " << action_name << std::endl;
  msg.name = action_name;
  msg.start_stamp = ros::Time::now();
  msg.end_stamp = ros::Time(0);
  timeline_pub_->publish(msg);
  std::cout << "The action has startd, wait a second..." << std::endl;
  ros::Duration(2).sleep();
  msg.name = action_name;
  msg.start_stamp = ros::Time(0);
  msg.end_stamp = ros::Time::now();
  timeline_pub_->publish(msg);
}

std::string findName(const std::string& action_type)
{
  std::string action_name_base = action_type + "_";
  std::transform(action_name_base.begin(), action_name_base.end(), action_name_base.begin(), [](unsigned char c){ return std::tolower(c); });
  for(;;)
  {
    if(onto_->individuals.exist(action_name_base + std::to_string(cpt)))
      cpt++;
    else
      return action_name_base + std::to_string(cpt);

    if(cpt > 100000)
      return "";
  }
}

void callback(Action action)
{
  if(action.getName() == "")
    action.setName(findName(action.getType()));

  onto_->feeder.addInheritage(action.getName(), action.getType());
  action.goBegin();
  while (action.end() == false)
  {
    auto param = action.getParameter();
    auto parameter_property = "hasParameter" + param.first;
    onto_->feeder.addInheritage(parameter_property, "hasParameter");
    onto_->feeder.addProperty(action.getName(), parameter_property, param.second);
  }
  onto_->feeder.waitUpdate(5000);
  sendAction(action.getName());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dt_action_emulation");
  ros::NodeHandle n;

  Parameters params;
  params.insert(Parameter("actions_file", {"-i", "--input"}));
  params.insert(Parameter("onto_name", {"-n", "--name"}, {""}));

  params.set(argc, argv);
  params.display();

  ActionFileReader reader;
  std::vector<Action> actions = reader.getActions(params.at("actions_file").getFirst());
  if(actions.size() == 0)
    return -1;

  onto_ = new OntologyManipulator(params.at("onto_name").getFirst());
  std::string mementar_pub_name = "mementar/insert_action" + ((params.at("onto_name").getFirst() != "") ? ("/" + params.at("onto_name").getFirst()) : "");
  std::cout << mementar_pub_name << std::endl;
  ros::Publisher timeline_pub = n.advertise<dt_actions_emulation::MementarAction>(mementar_pub_name, 1000);
  timeline_pub_ = &timeline_pub;

  Menu menu(callback);
  menu.run(actions);

  delete onto_;

  return 0;
}
