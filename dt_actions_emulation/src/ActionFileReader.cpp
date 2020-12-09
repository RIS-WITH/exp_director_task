#include "ActionFileReader.h"

#include <iostream>
#include <fstream>

std::vector<Action> ActionFileReader::getActions(const std::string& file_path)
{
  std::vector<Action> actions;

  TiXmlDocument tixml_document = getDomainDocument(file_path);
  TiXmlElement* tixml_actions = tixml_document.FirstChildElement("actions");
  if(tixml_actions != nullptr)
  {
    actions = getActionsFromTixml(tixml_actions);
    std::cout << "The actions have been found." << std::endl;
  }
  else
    std::cout << "The actions file has not been read corectly." << std::endl;

  return actions;
}

TiXmlDocument ActionFileReader::getDomainDocument(const std::string& xml_path)
{
  TiXmlDocument tixml_doc;

  std::string doc_str = "";
  std::string ligne_str = "";
  std::ifstream file(xml_path);

  if(!file.is_open())
  {
    std::cout << "Fail to open : " << xml_path << std::endl;
    return tixml_doc;
  }

  while(getline(file,ligne_str))
    doc_str += ligne_str;

  tixml_doc.Parse((const char*)doc_str.c_str(), nullptr, TIXML_ENCODING_UTF8);

  file.close();

  return tixml_doc;
}

std::vector<Action> ActionFileReader::getActionsFromTixml(TiXmlElement* tixml_actions)
{
  std::vector<Action> actions;
  for(TiXmlElement* elem = tixml_actions->FirstChildElement("action"); elem != nullptr; elem = elem->NextSiblingElement("action"))
    actions.push_back(getActionFromTixml(elem));

  return actions;
}

Action ActionFileReader::getActionFromTixml(TiXmlElement* tixml_action)
{
  Action action(getStrAttribute(tixml_action, "type"),
                getStrAttribute(tixml_action, "name"));

  if(action.getType() == "")
    std::cout << "An action without type has been found." << std::endl;

  for(TiXmlElement* elem = tixml_action->FirstChildElement("parameter"); elem != nullptr; elem = elem->NextSiblingElement("parameter"))
    action.addParameter(getStrAttribute(elem, "type"), elem->GetText());

  return action;
}
