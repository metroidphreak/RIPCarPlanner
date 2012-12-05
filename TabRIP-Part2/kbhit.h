#include <fcntl.h>
#include <unistd.h>
#define kbhit(c) (read(0,&c,1)>0)
int initKBHit()
{
  int tem = fcntl(0, F_GETFL, 0);
  fcntl (0, F_SETFL, (tem | O_NDELAY));
}
void closeKBHit(int tem)
{
  fcntl(0, F_SETFL, tem);
}
#if 0
int main()
{
  char c;
  int tem;
  tem = initKBHit();
  while (!kbhit(c)) {
  }
  closeKBHit(tem);
}
#endif
