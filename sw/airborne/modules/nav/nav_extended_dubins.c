#include "nav_extended_dubins.h"

bool NotExtendedDubins(DubinsType t)
{
  return t < 8;
}
bool StartExtendedDubins(DubinsType t)
{
  return 8 <= t && t < 2*8;
}
bool EndExtendedDubins(DubinsType t)
{
  return 2*8 <= t && t < 3*8;
}
bool BothExtendedDubins(DubinsType t)
{
  return 3*8 <= t && t < 4*8;
}