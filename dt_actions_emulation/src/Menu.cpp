#include "Menu.h"

#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#ifndef COLOR_RESET
#define COLOR_RESET    "\033[0m"
#endif
#ifndef COLOR_HIL
#define COLOR_HIL     "\033[107;30m"
#endif

static struct termios term;
static struct termios oterm;

#define cursorForward(x) printf("\033[%dC", (x))
#define cursorBackward(x) printf("\033[%dD", (x))

#define KEY_ESCAPE  0x001b
#define KEY_ENTER   0x000a
#define KEY_UP      0x0105
#define KEY_DOWN    0x0106
#define KEY_LEFT    0x0107
#define KEY_RIGHT   0x0108

static int getch(void);
static int kbhit(void);
static int kbesc(void);
static int kbget(void);

static int getch(void)
{
  int c = 0;

  tcgetattr(0, &oterm);
  memcpy(&term, &oterm, sizeof(term));
  term.c_lflag &= ~(ICANON | ECHO);
  term.c_cc[VMIN] = 1;
  term.c_cc[VTIME] = 0;
  tcsetattr(0, TCSANOW, &term);
  c = getchar();
  tcsetattr(0, TCSANOW, &oterm);
  return c;
}

static int kbhit(void)
{
  int c = 0;

  tcgetattr(0, &oterm);
  memcpy(&term, &oterm, sizeof(term));
  term.c_lflag &= ~(ICANON | ECHO);
  term.c_cc[VMIN] = 0;
  term.c_cc[VTIME] = 1;
  tcsetattr(0, TCSANOW, &term);
  c = getchar();
  tcsetattr(0, TCSANOW, &oterm);
  if (c != -1) ungetc(c, stdin);
  return ((c != -1) ? 1 : 0);
}

static int kbesc(void)
{
  int c;

  if (!kbhit())
    return KEY_ESCAPE;

  c = getch();
  if (c == '[')
  {
    switch (getch())
    {
      case 'A':
          c = KEY_UP;
          break;
      case 'B':
          c = KEY_DOWN;
          break;
      case 'C':
          c = KEY_LEFT;
          break;
      case 'D':
          c = KEY_RIGHT;
          break;
      default:
          c = 0;
          break;
    }
  }
  else
    c = 0;

  if (c == 0) while (kbhit()) getch();
  return c;
}

static int kbget(void)
{
  int c = getch();
  return (c == KEY_ESCAPE) ? kbesc() : c;
}

void Menu::run(std::vector<Action> actions)
{
  size_t index = 0;
  while(true)
  {
    printf("\033[H\033[2J");
    for(size_t i = 0; i < actions.size(); i++)
    {
      if(i == index)
        std::cout << COLOR_HIL << " > " << actions[i].toString() << COLOR_RESET << std::endl;
      else
        actions[i].display();
    }
    std::cout << "-- Press ESC to finish --" << std::endl;

    int input = kbget();
    if((input == KEY_RIGHT) || (input == KEY_DOWN))
    {
      if(index < actions.size() - 1)
        index++;
    }
    else if((input == KEY_LEFT) || (input == KEY_UP))
    {
      if(index > 0)
        index--;
    }
    else if(input == KEY_ENTER)
    {
      callback_(actions[index]);
      index++;
      if(index >= actions.size())
        return;
    }
    else if(KEY_ESCAPE)
      return;
  }
}
