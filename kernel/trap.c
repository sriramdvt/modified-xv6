#include "types.h"
#include "param.h"
#include "memlayout.h"
#include "riscv.h"
#include "spinlock.h"
#include "proc.h"
#include "defs.h"

struct spinlock tickslock;
uint ticks;

extern char trampoline[], uservec[], userret[];

// in kernelvec.S, calls kerneltrap().
void kernelvec();

extern int devintr();

void
trapinit(void)
{
  initlock(&tickslock, "time");
}

// set up to take exceptions and traps while in the kernel.
void
trapinithart(void)
{
  w_stvec((uint64)kernelvec);
}

//
// handle an interrupt, exception, or system call from user space.
// called from trampoline.S
//
void
usertrap(void)
{
  int which_dev = 0;

  if((r_sstatus() & SSTATUS_SPP) != 0)
    panic("usertrap: not from user mode");

  // send interrupts and exceptions to kerneltrap(),
  // since we're now in the kernel.
  w_stvec((uint64)kernelvec);

  struct proc *p = myproc();
  
  // save user program counter.
  p->trapframe->epc = r_sepc();
  
  if(r_scause() == 8){
    // system call

    if(p->killed)
      exit(-1);

    // sepc points to the ecall instruction,
    // but we want to return to the next instruction.
    p->trapframe->epc += 4;

    // an interrupt will change sstatus &c registers,
    // so don't enable until done with those registers.
    intr_on();

    syscall();
  } else if((which_dev = devintr()) != 0){
    // ok
  } else {
    printf("usertrap(): unexpected scause %p pid=%d\n", r_scause(), p->pid);
    printf("            sepc=%p stval=%p\n", r_sepc(), r_stval());
    p->killed = 1;
  }

  if(p->killed)
    exit(-1);


  #if !defined FCFS
  #if !defined PBS

  #ifdef MLFQ

  struct proc *best_proc_q = 0;
  int best_q = -1;
  for(int q_no = 0; q_no < 4; q_no++){
    if(best_proc_q != 0) {
      break;
    }
    for(int p_no = 0; p_no < procs_q[q_no]; p_no++){
      if (mlfq_q[q_no][p_no]->state == RUNNABLE){
        best_proc_q = mlfq_q[q_no][p_no];
        best_q = q_no;
        break;
      }
    }
  }

  if (p != 0){
    best_q = best_q < 0 ? best_q : p->q_num;
    if (best_q < p->q_num){
      yield();
    }
  }

  // if it is a timer interrupt and the process went over the limit, or if there's a process in a better queue
  if (which_dev == 2 && p->rtime_ls >= max_q_ticks[p->q_num]) {
    int prev_q = p->q_num;
    #ifdef Y_DEBUG
      printf("Usertrap Process %d ran for %d in q%d, moving to q%d\n", p->pid, p->rtime_ls, p->q_num, p->q_num+1);
      printf("For usertrap age: ");
      #endif
    remove_p_from_q(p);
    #ifdef Y_DEBUG
      printf("Removed %d from q%d and going to add it to q%d\n", p->pid, prev_q, prev_q+1);
    #endif
    add_p_to_q(p, prev_q+1);
    #ifdef Y_DEBUG
      printf("Finished moving %d in trap\n", p->pid);
    #endif
    yield();
  }
  
  #else
  // give up the CPU if this is a timer interrupt.
  if(which_dev == 2)
    yield();

  #endif
  #endif
  #endif

  usertrapret();
}

//
// return to user space
//
void
usertrapret(void)
{
  struct proc *p = myproc();

  // we're about to switch the destination of traps from
  // kerneltrap() to usertrap(), so turn off interrupts until
  // we're back in user space, where usertrap() is correct.
  intr_off();

  // send syscalls, interrupts, and exceptions to trampoline.S
  w_stvec(TRAMPOLINE + (uservec - trampoline));

  // set up trapframe values that uservec will need when
  // the process next re-enters the kernel.
  p->trapframe->kernel_satp = r_satp();         // kernel page table
  p->trapframe->kernel_sp = p->kstack + PGSIZE; // process's kernel stack
  p->trapframe->kernel_trap = (uint64)usertrap;
  p->trapframe->kernel_hartid = r_tp();         // hartid for cpuid()

  // set up the registers that trampoline.S's sret will use
  // to get to user space.
  
  // set S Previous Privilege mode to User.
  unsigned long x = r_sstatus();
  x &= ~SSTATUS_SPP; // clear SPP to 0 for user mode
  x |= SSTATUS_SPIE; // enable interrupts in user mode
  w_sstatus(x);

  // set S Exception Program Counter to the saved user pc.
  w_sepc(p->trapframe->epc);

  // tell trampoline.S the user page table to switch to.
  uint64 satp = MAKE_SATP(p->pagetable);

  // jump to trampoline.S at the top of memory, which 
  // switches to the user page table, restores user registers,
  // and switches to user mode with sret.
  uint64 fn = TRAMPOLINE + (userret - trampoline);
  ((void (*)(uint64,uint64))fn)(TRAPFRAME, satp);
}

// interrupts and exceptions from kernel code go here via kernelvec,
// on whatever the current kernel stack is.
void 
kerneltrap()
{
  int which_dev = 0;
  uint64 sepc = r_sepc();
  uint64 sstatus = r_sstatus();
  uint64 scause = r_scause();
  
  if((sstatus & SSTATUS_SPP) == 0)
    panic("kerneltrap: not from supervisor mode");
  if(intr_get() != 0)
    panic("kerneltrap: interrupts enabled");

  if((which_dev = devintr()) == 0){
    printf("scause %p\n", scause);
    printf("sepc=%p stval=%p\n", r_sepc(), r_stval());
    panic("kerneltrap");
  }

  #if !defined FCFS
  #if !defined PBS

  #ifdef MLFQ

  struct proc *p = myproc();

  struct proc *best_proc = 0;
  int best_q = -1;
  for(int q_no = 0; q_no < 4; q_no++){
    if(best_proc != 0) {
      break;
    }
    for(int p_no = 0; p_no < procs_q[q_no]; p_no++){
      if (mlfq_q[q_no][p_no]->state == RUNNABLE){
        best_proc = mlfq_q[q_no][p_no];
        best_q = q_no;
        break;
      }
    }
  }

  if (p != 0){
    best_q = best_q < 0 ? best_q : p->q_num;
    if (best_q < p->q_num){
      yield();
    }
  }

  // if it is a timer interrupt and the process went over the limit, or if there's a process in a better queue
  if (which_dev == 2 && p != 0 && p->state == RUNNING
     && p->rtime_ls >= max_q_ticks[p->q_num]) {
    int prev_q = p->q_num;
    #ifdef Y_DEBUG
      printf("Kerneltrap Process %d ran for %d in q%d, moving to q%d\n", p->pid, p->rtime_ls, p->q_num, p->q_num+1);
    printf("For kerneltrap age: ");
    #endif
    remove_p_from_q(p);
    #ifdef Y_DEBUG
      printf("Removed %d from q%d and going to add it to q%d\n", p->pid, prev_q, prev_q+1);
    #endif
    add_p_to_q(p, prev_q+1);
    #ifdef Y_DEBUG
      printf("Finished moving %d in trap\n", p->pid);
    #endif
    yield();
  }
  
  #else
  // give up the CPU if this is a timer interrupt.
  if(which_dev == 2 && myproc() != 0 && myproc()->state == RUNNING)
    yield();

  #endif
  #endif
  #endif



  // the yield() may have caused some traps to occur,
  // so restore trap registers for use by kernelvec.S's sepc instruction.
  w_sepc(sepc);
  w_sstatus(sstatus);
}

void
clockintr()
{
  acquire(&tickslock);
  ticks++;
  update_time();
  wakeup(&ticks);
  release(&tickslock);
}

// check if it's an external interrupt or software interrupt,
// and handle it.
// returns 2 if timer interrupt,
// 1 if other device,
// 0 if not recognized.
int
devintr()
{
  uint64 scause = r_scause();

  if((scause & 0x8000000000000000L) &&
     (scause & 0xff) == 9){
    // this is a supervisor external interrupt, via PLIC.

    // irq indicates which device interrupted.
    int irq = plic_claim();

    if(irq == UART0_IRQ){
      uartintr();
    } else if(irq == VIRTIO0_IRQ){
      virtio_disk_intr();
    } else if(irq){
      printf("unexpected interrupt irq=%d\n", irq);
    }

    // the PLIC allows each device to raise at most one
    // interrupt at a time; tell the PLIC the device is
    // now allowed to interrupt again.
    if(irq)
      plic_complete(irq);

    return 1;
  } else if(scause == 0x8000000000000001L){
    // software interrupt from a machine-mode timer interrupt,
    // forwarded by timervec in kernelvec.S.

    if(cpuid() == 0){
      clockintr();
    }
    
    // acknowledge the software interrupt by clearing
    // the SSIP bit in sip.
    w_sip(r_sip() & ~2);

    return 2;
  } else {
    return 0;
  }
}

