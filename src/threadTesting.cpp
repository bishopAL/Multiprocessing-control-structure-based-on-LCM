#include <iostream>
#include <limits.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <unistd.h>
#include <math.h>
#define PI 3.1415926
#define _JOYSTICK 1
#define THREAD1_ENABLE 1
#define THREAD2_ENABLE 1
#define THREAD3_ENABLE 1
#define THREAD4_ENABLE 1

void *thread1_func(void *data)
{

}
void *thread2_func(void *data)
{
    
}
void *thread3_func(void *data)
{
    
}
void *thread4_func(void *data)
{
    
}

void thread_init()
{
    struct sched_param param1, param2, param3, param4;
    pthread_attr_t attr1, attr2, attr3, attr4;
    pthread_t thread1 ,thread2 ,thread3, thread4;
    int ret;
    
    /* 1.Lock memory */
    if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
        printf("mlockall failed: %m\n");
        exit(-2);
    }
    /* 2. Initialize pthread attributes (default values) */
    ret = pthread_attr_init(&attr1);
    if (ret) {
        printf("init pthread attributes failed\n");
        goto out;
    }
    ret = pthread_attr_init(&attr2);
    if (ret) {
        printf("init pthread attributes failed\n");
        goto out;
    }
    ret = pthread_attr_init(&attr3);
    if (ret) {
        printf("init pthread attributes failed\n");
        goto out;
    }
    ret = pthread_attr_init(&attr4);
    if (ret) {
        printf("init pthread attributes failed\n");
        goto out;
    }
 
    /* 3. Set a specific stack size  */
    ret = pthread_attr_setstacksize(&attr1, PTHREAD_STACK_MIN);
    if (ret) {
        printf("pthread setstacksize failed\n");
        goto out;
    }
    ret = pthread_attr_setstacksize(&attr2, PTHREAD_STACK_MIN);
    if (ret) {
        printf("pthread setstacksize failed\n");
        goto out;
    }
    ret = pthread_attr_setstacksize(&attr3, PTHREAD_STACK_MIN);
    if (ret) {
        printf("pthread setstacksize failed\n");
        goto out;
    }
    ret = pthread_attr_setstacksize(&attr4, PTHREAD_STACK_MIN);
    if (ret) {
        printf("pthread setstacksize failed\n");
        goto out;
    }
 
    /*4. Set scheduler policy and priority of pthread */
    ret = pthread_attr_setschedpolicy(&attr1, SCHED_FIFO);
    if (ret) {
        printf("pthread setschedpolicy failed\n");
        goto out;
    }
    ret = pthread_attr_setschedpolicy(&attr2, SCHED_FIFO);
    if (ret) {
        printf("pthread setschedpolicy failed\n");
        goto out;
    }
    ret = pthread_attr_setschedpolicy(&attr3, SCHED_FIFO);
    if (ret) {
        printf("pthread setschedpolicy failed\n");
        goto out;
    }
    ret = pthread_attr_setschedpolicy(&attr4, SCHED_FIFO);
    if (ret) {
        printf("pthread setschedpolicy failed\n");
        goto out;
    }
    
    param1.sched_priority = 20;
    param2.sched_priority = 1;
    param3.sched_priority = 20;
    param4.sched_priority = 20;

    ret = pthread_attr_setschedparam(&attr1, &param1);
    if (ret) {
            printf("pthread setschedparam failed\n");
            goto out;
    }
    ret = pthread_attr_setschedparam(&attr2, &param2);
    if (ret) {
            printf("pthread setschedparam failed\n");
            goto out;
    }
    ret = pthread_attr_setschedparam(&attr3, &param3);
    if (ret) {
            printf("pthread setschedparam failed\n");
            goto out;
    }
    ret = pthread_attr_setschedparam(&attr4, &param4);
    if (ret) {
            printf("pthread setschedparam failed\n");
            goto out;
    }

    /*5. Use scheduling parameters of attr */
    ret = pthread_attr_setinheritsched(&attr1, PTHREAD_EXPLICIT_SCHED);
    if (ret) {
            printf("pthread setinheritsched failed\n");
            goto out;
    }
    ret = pthread_attr_setinheritsched(&attr2, PTHREAD_EXPLICIT_SCHED);
    if (ret) {
            printf("pthread setinheritsched failed\n");
            goto out;
    }
    ret = pthread_attr_setinheritsched(&attr3, PTHREAD_EXPLICIT_SCHED);
    if (ret) {
            printf("pthread setinheritsched failed\n");
            goto out;
    }
    ret = pthread_attr_setinheritsched(&attr4, PTHREAD_EXPLICIT_SCHED);
    if (ret) {
            printf("pthread setinheritsched failed\n");
            goto out;
    }
 
    /*6. Create a pthread with specified attributes */
    #ifdef THREAD1_ENABLE
    ret = pthread_create(&thread1, &attr1, thread1_func, NULL);
    if (ret) {
            printf("create pthread1 failed\n");
            goto out;
    }
    #endif

    #ifdef THREAD2_ENABLE
    ret = pthread_create(&thread2, &attr2, thread2_func, NULL);
    if (ret) {
            printf("create pthread2 failed\n");
            goto out;
    }
    #endif

    #ifdef THREAD3_ENABLE
    ret = pthread_create(&thread3, &attr3, thread3_func, NULL);
    if (ret) {
            printf("create pthread3 failed\n");
            goto out;
    }
    #endif

    #ifdef THREAD4_ENABLE
    ret = pthread_create(&thread4, &attr4, thread4_func, NULL);
    if (ret) {
            printf("create pthread4 failed\n");
            goto out;
    }
    #endif

    #ifdef THREAD1_ENABLE
    ret = pthread_join(thread1, NULL);
    if (ret)
        printf("join pthread1 failed: %m\n");
    #endif

    #ifdef THREAD2_ENABLE
    ret = pthread_join(thread2, NULL);
    if (ret)
        printf("join pthread2 failed: %m\n");
    #endif

    #ifdef THREAD3_ENABLE
    ret = pthread_join(thread3, NULL);
    if (ret)
        printf("join pthread3 failed: %m\n");
    #endif

    #ifdef THREAD4_ENABLE
    ret = pthread_join(thread4, NULL);
    if (ret)
        printf("join pthread4 failed: %m\n");
    #endif
 
    /*7. Join the thread and wait until it is done */
    
out:
    ret;

}

int main()
{
    thread_init();
    return 1;
}