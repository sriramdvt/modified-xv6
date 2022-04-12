#include "types.h"
#include "param.h"
#include "memlayout.h"
#include "riscv.h"
#include "spinlock.h"
#include "proc.h"
#include "defs.h"
#include "stddef.h"

struct cpu cpus[NCPU];

struct proc proc[NPROC];

struct proc *initproc;

int nextpid = 1;
struct spinlock pid_lock;

extern void forkret(void);
static void freeproc(struct proc *p);

extern char trampoline[]; // trampoline.S

// helps ensure that wakeups of wait()ing
// parents are not lost. helps obey the
// memory model when using p->parent.
// must be acquired before any p->lock.
struct spinlock wait_lock;

// Allocate a page for each process's kernel stack.
// Map it high in memory, followed by an invalid
// guard page.
void
proc_mapstacks(pagetable_t kpgtbl) {
  struct proc *p;
  
  for(p = proc; p < &proc[NPROC]; p++) {
    char *pa = kalloc();
    if(pa == 0)
      panic("kalloc");
    uint64 va = KSTACK((int) (p - proc));
    kvmmap(kpgtbl, va, (uint64)pa, PGSIZE, PTE_R | PTE_W);
  }
}

// initialize the proc table at boot time.
void
procinit(void)
{
  struct proc *p;
  
  initlock(&pid_lock, "nextpid");
  initlock(&wait_lock, "wait_lock");
  for(p = proc; p < &proc[NPROC]; p++) {
      initlock(&p->lock, "proc");
      p->kstack = KSTACK((int) (p - proc));
  }
}

// Must be called with interrupts disabled,
// to prevent race with process being moved
// to a different CPU.
int
cpuid()
{
  int id = r_tp();
  return id;
}

// Return this CPU's cpu struct.
// Interrupts must be disabled.
struct cpu*
mycpu(void) {
  int id = cpuid();
  struct cpu *c = &cpus[id];
  return c;
}

// Return the current struct proc *, or zero if none.
struct proc*
myproc(void) {
  push_off();
  struct cpu *c = mycpu();
  struct proc *p = c->proc;
  pop_off();
  return p;
}

int
allocpid() {
  int pid;
  
  acquire(&pid_lock);
  pid = nextpid;
  nextpid = nextpid + 1;
  release(&pid_lock);

  return pid;
}

// Look in the process table for an UNUSED proc.
// If found, initialize state required to run in the kernel,
// and return with p->lock held.
// If there are no free procs, or a memory allocation fails, return 0.
static struct proc*
allocproc(void)
{
  struct proc *p;

  for(p = proc; p < &proc[NPROC]; p++) {
    acquire(&p->lock);
    if(p->state == UNUSED) {
      goto found;
    } else {
      release(&p->lock);
    }
  }
  return 0;

found:
  p->pid = allocpid();
  p->state = USED;
  p->mask = 0;

  // Creation time of the process
  p->ctime = ticks;
  // Run time of the process = 0 since it just started
  p->rtime = 0;
  // Wait time of the process = 0 since it just started
  p->wtime = 0;
  // Exit time of the process = 0 since it didn't exit yet
  p->etime = 0;
  // Default priority of the process
  p->s_priority = 60;
  // Time spent sleeping = 0 since it just started
  p->stime = 0;
  // Time spent running from the last time it was scheduled
  p->rtime_ls = 0;
  // Default nice value is 5
  p->nice = 5;
  // Process hasn't been scheduled yet
  p->num_scheduled = 0;
  // Add it to the top priority queue by default
  p->q_num = 0;
  // Time spent waiting = 0
  p->age_t = 0;
  // The process hasn't spent any time in any of the queues
  for (int i = 0; i < 5; i++){
    p->q_time[i] = 0;
  }

  // Allocate a trapframe page.
  if((p->trapframe = (struct trapframe *)kalloc()) == 0){
    freeproc(p);
    release(&p->lock);
    return 0;
  }

  // An empty user page table.
  p->pagetable = proc_pagetable(p);
  if(p->pagetable == 0){
    freeproc(p);
    release(&p->lock);
    return 0;
  }

  // Set up new context to start executing at forkret,
  // which returns to user space.
  memset(&p->context, 0, sizeof(p->context));
  p->context.ra = (uint64)forkret;
  p->context.sp = p->kstack + PGSIZE;

  return p;
}

// free a proc structure and the data hanging from it,
// including user pages.
// p->lock must be held.
static void
freeproc(struct proc *p)
{
  if(p->trapframe)
    kfree((void*)p->trapframe);
  p->trapframe = 0;
  if(p->pagetable)
    proc_freepagetable(p->pagetable, p->sz);
  p->pagetable = 0;
  p->sz = 0;
  p->pid = 0;
  p->parent = 0;
  p->name[0] = 0;
  p->mask = 0;
  p->chan = 0;
  p->killed = 0;
  p->xstate = 0;
  p->state = UNUSED;
}

// Create a user page table for a given process,
// with no user memory, but with trampoline pages.
pagetable_t
proc_pagetable(struct proc *p)
{
  pagetable_t pagetable;

  // An empty page table.
  pagetable = uvmcreate();
  if(pagetable == 0)
    return 0;

  // map the trampoline code (for system call return)
  // at the highest user virtual address.
  // only the supervisor uses it, on the way
  // to/from user space, so not PTE_U.
  if(mappages(pagetable, TRAMPOLINE, PGSIZE,
              (uint64)trampoline, PTE_R | PTE_X) < 0){
    uvmfree(pagetable, 0);
    return 0;
  }

  // map the trapframe just below TRAMPOLINE, for trampoline.S.
  if(mappages(pagetable, TRAPFRAME, PGSIZE,
              (uint64)(p->trapframe), PTE_R | PTE_W) < 0){
    uvmunmap(pagetable, TRAMPOLINE, 1, 0);
    uvmfree(pagetable, 0);
    return 0;
  }

  return pagetable;
}

// Free a process's page table, and free the
// physical memory it refers to.
void
proc_freepagetable(pagetable_t pagetable, uint64 sz)
{
  uvmunmap(pagetable, TRAMPOLINE, 1, 0);
  uvmunmap(pagetable, TRAPFRAME, 1, 0);
  uvmfree(pagetable, sz);
}

// a user program that calls exec("/init")
// od -t xC initcode
uchar initcode[] = {
  0x17, 0x05, 0x00, 0x00, 0x13, 0x05, 0x45, 0x02,
  0x97, 0x05, 0x00, 0x00, 0x93, 0x85, 0x35, 0x02,
  0x93, 0x08, 0x70, 0x00, 0x73, 0x00, 0x00, 0x00,
  0x93, 0x08, 0x20, 0x00, 0x73, 0x00, 0x00, 0x00,
  0xef, 0xf0, 0x9f, 0xff, 0x2f, 0x69, 0x6e, 0x69,
  0x74, 0x00, 0x00, 0x24, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00
};

// Set up first user process.
void
userinit(void)
{

  for(int i = 0; i < 5; i++){
    for(int j = 0; j < NPROC; j++){
      mlfq_q[i][j] = NULL;
    }
  }

  struct proc *p;

  p = allocproc();
  initproc = p;
  
  // allocate one user page and copy init's instructions
  // and data into it.
  uvminit(p->pagetable, initcode, sizeof(initcode));
  p->sz = PGSIZE;

  // prepare for the very first "return" from kernel to user.
  p->trapframe->epc = 0;      // user program counter
  p->trapframe->sp = PGSIZE;  // user stack pointer

  safestrcpy(p->name, "initcode", sizeof(p->name));
  p->cwd = namei("/");

  p->state = RUNNABLE;

  #ifdef MLFQ
  add_p_to_q(p, 0);
  #endif

  release(&p->lock);
}

// Grow or shrink user memory by n bytes.
// Return 0 on success, -1 on failure.
int
growproc(int n)
{
  uint sz;
  struct proc *p = myproc();

  sz = p->sz;
  if(n > 0){
    if((sz = uvmalloc(p->pagetable, sz, sz + n)) == 0) {
      return -1;
    }
  } else if(n < 0){
    sz = uvmdealloc(p->pagetable, sz, sz + n);
  }
  p->sz = sz;
  return 0;
}

// Create a new process, copying the parent.
// Sets up child kernel stack to return as if from fork() system call.
int
fork(void)
{
  int i, pid;
  struct proc *np;
  struct proc *p = myproc();

  // Allocate process.
  if((np = allocproc()) == 0){
    return -1;
  }

  // Copy user memory from parent to child.
  if(uvmcopy(p->pagetable, np->pagetable, p->sz) < 0){
    freeproc(np);
    release(&np->lock);
    return -1;
  }
  np->sz = p->sz;

  // copy the strace mask to the child
  np->mask = p->mask;

  // copy saved user registers.
  *(np->trapframe) = *(p->trapframe);

  // Cause fork to return 0 in the child.
  np->trapframe->a0 = 0;

  // increment reference counts on open file descriptors.
  for(i = 0; i < NOFILE; i++)
    if(p->ofile[i])
      np->ofile[i] = filedup(p->ofile[i]);
  np->cwd = idup(p->cwd);

  safestrcpy(np->name, p->name, sizeof(p->name));

  pid = np->pid;

  release(&np->lock);

  acquire(&wait_lock);
  np->parent = p;
  release(&wait_lock);

  acquire(&np->lock);
  np->state = RUNNABLE;

  #ifdef MLFQ
  add_p_to_q(np, 0);
  #endif

  release(&np->lock);

  return pid;
}

// Pass p's abandoned children to init.
// Caller must hold wait_lock.
void
reparent(struct proc *p)
{
  struct proc *pp;

  for(pp = proc; pp < &proc[NPROC]; pp++){
    if(pp->parent == p){
      pp->parent = initproc;
      wakeup(initproc);
    }
  }
}

// Exit the current process.  Does not return.
// An exited process remains in the zombie state
// until its parent calls wait().
void
exit(int status)
{
  struct proc *p = myproc();

  if(p == initproc)
    panic("init exiting");

  // Close all open files.
  for(int fd = 0; fd < NOFILE; fd++){
    if(p->ofile[fd]){
      struct file *f = p->ofile[fd];
      fileclose(f);
      p->ofile[fd] = 0;
    }
  }

  begin_op();
  iput(p->cwd);
  end_op();
  p->cwd = 0;

  acquire(&wait_lock);

  // Give any children to init.
  reparent(p);

  // Parent might be sleeping in wait().
  wakeup(p->parent);
  
  acquire(&p->lock);

  p->xstate = status;
  p->state = ZOMBIE;
  p->etime = ticks;

  release(&wait_lock);

  // Jump into the scheduler, never to return.
  sched();
  panic("zombie exit");
}

// Wait for a child process to exit and return its pid.
// Return -1 if this process has no children.
int
wait(uint64 addr)
{
  struct proc *np;
  int havekids, pid;
  struct proc *p = myproc();

  acquire(&wait_lock);

  for(;;){
    // Scan through table looking for exited children.
    havekids = 0;
    for(np = proc; np < &proc[NPROC]; np++){
      if(np->parent == p){
        // make sure the child isn't still in exit() or swtch().
        acquire(&np->lock);

        havekids = 1;
        if(np->state == ZOMBIE){
          // Found one.
          pid = np->pid;
          if(addr != 0 && copyout(p->pagetable, addr, (char *)&np->xstate,
                                  sizeof(np->xstate)) < 0) {
            release(&np->lock);
            release(&wait_lock);
            return -1;
          }
          freeproc(np);
          release(&np->lock);
          release(&wait_lock);
          return pid;
        }
        release(&np->lock);
      }
    }

    // No point waiting if we don't have any children.
    if(!havekids || p->killed){
      release(&wait_lock);
      return -1;
    }
    
    // Wait for a child to exit.
    sleep(p, &wait_lock);  //DOC: wait-sleep
  }
}

// Wait for a child process to exit and return its pid.
// Return -1 if this process has no children.
int
waitx(uint64 addr, uint* wtime, uint* rtime)
{
  struct proc *np;
  int havekids, pid;
  struct proc *p = myproc();

  acquire(&wait_lock);

  for(;;){
    // Scan through table looking for exited children.
    havekids = 0;
    for(np = proc; np < &proc[NPROC]; np++){
      if(np->parent == p){
        // make sure the child isn't still in exit() or swtch().
        acquire(&np->lock);

        havekids = 1;
        if(np->state == ZOMBIE){
          // Found one.
          pid = np->pid;

          *rtime = np->rtime;
          *wtime = np->etime - np->ctime - np->rtime;

          #ifdef Y_DEBUG
          printf("Process %d started:%d, ended:%d, rtime:%d\n",pid, np->ctime, np->etime, np->rtime);
          #endif

          if(addr != 0 && copyout(p->pagetable, addr, (char *)&np->xstate,
                                  sizeof(np->xstate)) < 0) {
            release(&np->lock);
            release(&wait_lock);
            return -1;
          }
          freeproc(np);
          release(&np->lock);
          release(&wait_lock);
          return pid;
        }
        release(&np->lock);
      }
    }

    // No point waiting if we don't have any children.
    if(!havekids || p->killed){
      release(&wait_lock);
      return -1;
    }
    
    // Wait for a child to exit.
    sleep(p, &wait_lock);  //DOC: wait-sleep
  }
}

void
update_time()
{
  struct proc* p;

  for (p = proc; p < &proc[NPROC]; p++) {
    acquire(&p->lock);
    if (p->state == RUNNING) {
      p->rtime++;
      p->rtime_ls++;
    }
    else if (p->state == SLEEPING) {
      p->stime++;
      p->wtime++;
    }
    else if (p->state == RUNNABLE) {
      p->age_t++;
      p->wtime++;
    }

    if(p->state == RUNNING || p->state == RUNNABLE){
      p->q_time[p->q_num]++;
    }

    #ifdef Y_DEBUG
    if((p->state == RUNNING || p->state == RUNNABLE || p->state == SLEEPING) && (p->pid != 1 && p->pid != 2)){
      printf("Process P%d is in Q%d at %d\n", p->pid, p->q_num, ticks);
    }
    #endif

    release(&p->lock); 
  }
}

// Update the statistics of a process
// after it is scheduled
void
update_process(struct proc* p)
{
  if (p->rtime_ls > 0) {
    p->nice = (10 * p->stime)/(p->rtime_ls + p->stime);
  }
  else 
    p->nice = 5;
  
  p->rtime_ls = 0;
  p->age_t = 0;
  p->num_scheduled++;
}

struct proc *mlfq_q[5][NPROC];
uint max_q_ticks[5] = {1,2,4,8,16};
uint procs_q[5] = {0,0,0,0,0};

int remove_p_from_q(struct proc *p){

  int q_no = p->q_num;

  #ifdef Y_DEBUG
  printf("Removing %d from q%d which has %d procs\n", p->pid, q_no, procs_q[q_no]);
  #endif

  int proc_id = -1;

  for(int i = 0; i < procs_q[q_no]; i++){
    if(mlfq_q[q_no][i]->pid == p->pid){
      proc_id = i;
      break;
    }
  }

  if (proc_id == -1) return -1;
  
  for(int i = proc_id+1; i < procs_q[q_no]; i++){
    mlfq_q[q_no][i-1] = mlfq_q[q_no][i];
  }


  p->q_num = -1;
  procs_q[q_no]--;

  #ifdef Y_DEBUG
  printf("Removed %d from q%d\n", p->pid, q_no);
  #endif

  return 1;
}

int add_p_to_q(struct proc *p, int q_no){

  #ifdef Y_DEBUG
  printf("Adding %d to q%d\n", p->pid, q_no);
  #endif

  for (int q_check = 0; q_check < 5; q_check++){
    for(int i = 0; i < procs_q[q_check]; i++){
      // If the process is already in a queue
      if(mlfq_q[q_check][i]->pid == p->pid){
        #ifdef Y_DEBUG
        printf("Process %d is already in q%d\n", p->pid, q_check);
        printf("For already in q: ");
        #endif
        remove_p_from_q(p);
        }
    }
  }

  q_no = q_no <= 4 ? q_no : 4;
  q_no = q_no >= 0 ? q_no : 0;

  mlfq_q[q_no][procs_q[q_no]] = p;
  p->q_num = q_no;
  procs_q[q_no]++;

  p->age_t = 0;

  #ifdef Y_DEBUG
  printf("Added %d to q%d\n", p->pid, q_no);
  #endif

  return 1;

}

// Per-CPU process scheduler.
// Each CPU calls scheduler() after setting itself up.
// Scheduler never returns.  It loops, doing:
//  - choose a process to run.
//  - swtch to start running that process.
//  - eventually that process transfers control
//    via swtch back to the scheduler.
void
scheduler(void)
{
  struct cpu *c = mycpu();

  
  c->proc = 0;
  for(;;){
    // Avoid deadlock by ensuring that devices can interrupt.
    intr_on();

    #ifdef RR
      struct proc *p;
      for(p = &proc[0]; p < &proc[NPROC]; p++) {
        acquire(&p->lock);
        if(p->state == RUNNABLE) {
          // Switch to chosen process.  It is the process's job
          // to release its lock and then reacquire it
          // before jumping back to us.
          p->state = RUNNING;
          update_process(p);
          c->proc = p;
          swtch(&c->context, &p->context);

          // Process is done running for now.
          // It should have changed its p->state before coming back.
          c->proc = 0;
        }
        release(&p->lock);
      }
    #else
      #ifdef FCFS
      struct proc *p;
      struct proc *first_proc = NULL;
      for(p = &proc[0]; p < &proc[NPROC]; p++) {
        if(p->state == RUNNABLE) {

          if(first_proc == NULL) {
            first_proc = p;
          }
          else if (first_proc->ctime > p->ctime) {
            first_proc = p;
          }
        }
      }

      if (first_proc != NULL && first_proc->state == RUNNABLE) {
        acquire(&first_proc->lock);
        if(first_proc->state == RUNNABLE) {
          // Switch to chosen process.  It is the process's job
          // to release its lock and then reacquire it
          // before jumping back to us.
          first_proc->state = RUNNING;
          update_process(first_proc);
          c->proc = first_proc;
          swtch(&c->context, &first_proc->context);

          // Process is done running for now.
          // It should have changed its p->state before coming back.
          c->proc = 0;
        }
        release(&first_proc->lock);
      }

      #else
        #ifdef PBS
        struct proc *p;
        struct proc *highest_p_proc = NULL;
        uint iter_p = 0, highest_p = 0;
        for(p = &proc[0]; p < &proc[NPROC]; p++) {
          if(p->state == RUNNABLE) {

            if(highest_p_proc == NULL) {
              highest_p_proc = p;
            }

            else {
              iter_p = p->s_priority-p->nice+5;
              if (iter_p > 100) iter_p = 100;
              if (iter_p < 0) iter_p = 0;

              highest_p = highest_p_proc->s_priority-highest_p_proc->nice+5;
              if (highest_p > 100) highest_p = 100;
              if (highest_p < 0) highest_p = 0;

              if (highest_p > iter_p){
                highest_p_proc = p;
              }
              else if (highest_p == iter_p) {
                if (highest_p_proc->num_scheduled > p->num_scheduled) highest_p_proc = p;
                else if (highest_p_proc->num_scheduled == p->num_scheduled) {
                  if (highest_p_proc->ctime > p->ctime) highest_p_proc = p;
                }
              }
            }
          }
        }
        if (highest_p_proc != NULL && highest_p_proc->state == RUNNABLE) {
          acquire(&highest_p_proc->lock);
          if(highest_p_proc->state == RUNNABLE) {
            // Switch to chosen process.  It is the process's job
            // to release its lock and then reacquire it
            // before jumping back to us.
            highest_p_proc->state = RUNNING;


            update_process(highest_p_proc);           

            c->proc = highest_p_proc;
            swtch(&c->context, &highest_p_proc->context);

            // Process is done running for now.
            // It should have changed its p->state before coming back.
            c->proc = 0;
          }
          release(&highest_p_proc->lock);
        }

        #else
          #ifdef MLFQ

          // Aging: move process to a higher priority queue if it aged for too long
          for(int q_no = 1; q_no < 5; q_no++){
            for(int p_no = 0; p_no < procs_q[q_no]; p_no++){
              if (mlfq_q[q_no][p_no]->age_t > AGE_LIMIT){
                int prev_q = mlfq_q[q_no][p_no]->q_num;
                #ifdef Y_DEBUG
                printf("For aging: ");
                #endif
                struct proc *aged_proc = mlfq_q[q_no][p_no];
                remove_p_from_q(mlfq_q[q_no][p_no]);
                add_p_to_q(aged_proc,prev_q-1);
              }
            }
          }

          // choose the process from the priority queues to run
          struct proc *best_proc = NULL;
          #ifdef Y_DEBUG
          int found_q = -1;
          #endif
          for(int q_no = 0; q_no < 4; q_no++){
            if(best_proc != NULL) {
              break;
            }
            for(int p_no = 0; p_no < procs_q[q_no]; p_no++){
              if (mlfq_q[q_no][p_no]->state == RUNNABLE){
                best_proc = mlfq_q[q_no][p_no];
                #ifdef Y_DEBUG
                found_q = q_no;
                #endif
                break;
              }
            }
          }

          if (best_proc != NULL && best_proc->state == RUNNABLE) {
            acquire(&best_proc->lock);
            if(best_proc->state == RUNNABLE) {
              // Switch to chosen process.  It is the process's job
              // to release its lock and then reacquire it
              // before jumping back to us.
              best_proc->state = RUNNING;
              update_process(best_proc);
              c->proc = best_proc;

              #ifdef Y_DEBUG
              printf("MLFQ chose %d in q%d\n", best_proc->pid, found_q);
              #endif

              swtch(&c->context, &best_proc->context);

              // Process is done running for now.
              // It should have changed its p->state before coming back.
              c->proc = 0;
            }
            release(&best_proc->lock);
          }


          #endif
        #endif
      #endif
    #endif

  }
}

// Switch to scheduler.  Must hold only p->lock
// and have changed proc->state. Saves and restores
// intena because intena is a property of this
// kernel thread, not this CPU. It should
// be proc->intena and proc->noff, but that would
// break in the few places where a lock is held but
// there's no process.
void
sched(void)
{
  int intena;
  struct proc *p = myproc();

  if(!holding(&p->lock))
    panic("sched p->lock");
  if(mycpu()->noff != 1)
    panic("sched locks");
  if(p->state == RUNNING)
    panic("sched running");
  if(intr_get())
    panic("sched interruptible");

  intena = mycpu()->intena;
  swtch(&p->context, &mycpu()->context);
  mycpu()->intena = intena;
}

// Give up the CPU for one scheduling round.
void
yield(void)
{
  struct proc *p = myproc();
  acquire(&p->lock);
  p->state = RUNNABLE;
  sched();
  release(&p->lock);
}

// Set the static priority of the process
// with the given pid
int
set_priority(int new_priority, int pid)
{

  if (new_priority < 0 || new_priority > 100) {
    printf("Invalid priority\n");
    return -1;
  }

  struct proc *p;
  int old_priority = -1;

  for(p = &proc[0]; p < &proc[NPROC]; p++) {
    if (p->pid == pid) {
      old_priority = p->s_priority;
      p->s_priority = new_priority;
      p->nice = 5;
    }
  }
  if (old_priority > new_priority) yield();

  return old_priority;
}

// A fork child's very first scheduling by scheduler()
// will swtch to forkret.
void
forkret(void)
{
  static int first = 1;

  // Still holding p->lock from scheduler.
  release(&myproc()->lock);

  if (first) {
    // File system initialization must be run in the context of a
    // regular process (e.g., because it calls sleep), and thus cannot
    // be run from main().
    first = 0;
    fsinit(ROOTDEV);
  }

  usertrapret();
}

// Atomically release lock and sleep on chan.
// Reacquires lock when awakened.
void
sleep(void *chan, struct spinlock *lk)
{
  struct proc *p = myproc();
  
  // Must acquire p->lock in order to
  // change p->state and then call sched.
  // Once we hold p->lock, we can be
  // guaranteed that we won't miss any wakeup
  // (wakeup locks p->lock),
  // so it's okay to release lk.

  acquire(&p->lock);  //DOC: sleeplock1
  release(lk);

  // Go to sleep.
  p->chan = chan;
  p->state = SLEEPING;

  p->stime = 0;

  sched();

  // Tidy up.
  p->chan = 0;

  // Reacquire original lock.
  release(&p->lock);
  acquire(lk);
}

// Wake up all processes sleeping on chan.
// Must be called without any p->lock.
void
wakeup(void *chan)
{
  struct proc *p;

  for(p = proc; p < &proc[NPROC]; p++) {
    if(p != myproc()){
      acquire(&p->lock);
      if(p->state == SLEEPING && p->chan == chan) {
        p->state = RUNNABLE;
      }
      release(&p->lock);
    }
  }
}

// Kill the process with the given pid.
// The victim won't exit until it tries to return
// to user space (see usertrap() in trap.c).
int
kill(int pid)
{
  struct proc *p;

  for(p = proc; p < &proc[NPROC]; p++){
    acquire(&p->lock);
    if(p->pid == pid){
      p->killed = 1;
      if(p->state == SLEEPING){
        // Wake process from sleep().
        p->state = RUNNABLE;
        #ifdef MLFQ
        add_p_to_q(p, 0);
        #endif
      }
      release(&p->lock);
      return 0;
    }
    release(&p->lock);
  }
  return -1;
}

// Copy to either a user address, or kernel address,
// depending on usr_dst.
// Returns 0 on success, -1 on error.
int
either_copyout(int user_dst, uint64 dst, void *src, uint64 len)
{
  struct proc *p = myproc();
  if(user_dst){
    return copyout(p->pagetable, dst, src, len);
  } else {
    memmove((char *)dst, src, len);
    return 0;
  }
}

// Copy from either a user address, or kernel address,
// depending on usr_src.
// Returns 0 on success, -1 on error.
int
either_copyin(void *dst, int user_src, uint64 src, uint64 len)
{
  struct proc *p = myproc();
  if(user_src){
    return copyin(p->pagetable, dst, src, len);
  } else {
    memmove(dst, (char*)src, len);
    return 0;
  }
}

// Print a process listing to console.  For debugging.
// Runs when user types ^P on console.
// No lock to avoid wedging a stuck machine further.
void
procdump(void)
{
  static char *states[] = {
  [UNUSED]    "unused   ",
  [SLEEPING]  "sleeping ",
  [RUNNABLE]  "runnable ",
  [RUNNING]   "running  ",
  [ZOMBIE]    "zombie   "
  };
  struct proc *p;
  char *state;

  printf("\n");
  #if defined RR
  printf("PID  State  Name  rtime  wtime  nrun\n");
  #else
  #if defined FCFS
  printf("PID  State  Name  rtime  wtime  nrun\n");
  #else
  #if defined PBS
  printf("PID Priority State rtime wtime rtime_ls  stime_ls  nrun\n");
  #else
  #if defined MLFQ
  printf("PID Priority Q State rtime wtime rtime_ls  stime_ls  nrun q0 q1 q2 q3 q4\n");
  #endif
  #endif
  #endif
  #endif

  for(p = proc; p < &proc[NPROC]; p++){
    if(p->state == UNUSED)
      continue;
    if(p->state >= 0 && p->state < NELEM(states) && states[p->state])
      state = states[p->state];
    else
      state = "???";
    #if defined RR
    printf("%d %s %d %d %d", p->pid, state, p->rtime, p->wtime, p->num_scheduled);
    #else
    #if defined FCFS
    printf("%d %s %d %d %d", p->pid, state, p->rtime, p->wtime, p->num_scheduled);
    #else
    #if defined PBS
    printf("%d   %d   %s   %d      %d    %d        %d         %d", p->pid, p->s_priority, state, p->rtime, p->wtime, p->rtime_ls, p->stime, p->num_scheduled);
    #else
    #if defined MLFQ

    int priority = -1;
    for (int q_check = 0; q_check < 5; q_check++){
      for(int i = 0; i < procs_q[q_check]; i++){
        if (mlfq_q[q_check][i]->pid == p->pid){
          priority = i;
          break;
        }
      }
    }
    printf("%d   %d   %d  %s   %d      %d    %d        %d        %d   %d  %d  %d  %d  %d", p->pid, priority, p->q_num, state, p->rtime, p->wtime, p->rtime_ls, p->stime, p->num_scheduled, p->q_time[0], p->q_time[1], p->q_time[2], p->q_time[3], p->q_time[4]);

    #endif
    #endif
    #endif
    #endif
    printf("\n");
  }
}
