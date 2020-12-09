#ifndef DT_ACTIONFILEREADER_H
#define DT_ACTIONFILEREADER_H

#include "Action.h"

#include <tinyxml.h>

class ActionFileReader
{
public:
  std::vector<Action> getActions(const std::string& file_path);

private:
  TiXmlDocument getDomainDocument(const std::string& xml_path);
  std::vector<Action> getActionsFromTixml(TiXmlElement* tixml_actions);
  Action getActionFromTixml(TiXmlElement* tixml_action);

  inline std::string getStrAttribute(TiXmlElement* elem, const std::string& attribute)
  {
    const char* str_attribute = elem->Attribute(attribute.c_str());
    if(str_attribute != NULL)
      return std::string(str_attribute);
    else
      return "";
  }
};

#endif // DT_ACTIONFILEREADER_H
