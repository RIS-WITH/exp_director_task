#ifndef DT_MENU_H
#define DT_MENU_H

#include <functional>

#include "Action.h"

class Menu
{
public:
  Menu(std::function<void(Action)> callback) { callback_ = callback; }

  void run(std::vector<Action> actions);
private:
  std::function<void(Action)> callback_;
};

#endif // DT_MENU_H
