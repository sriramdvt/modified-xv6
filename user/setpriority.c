#include "kernel/param.h"
#include "kernel/types.h"
#include "kernel/stat.h"
#include "user/user.h"

int
main(int argc, char *argv[])
{

  if(argc < 3 || (argv[1][0] < '0' || argv[1][0] > '9')){
    fprintf(2, "Use setpriority as: setpriority priority pid\n");
    exit(1);
  }

  set_priority(atoi(argv[1]), atoi(argv[2]));
  
  exit(0);
}

