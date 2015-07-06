/*
 * Copyright (c) 2015
 *
 *     Yuan Mei
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * TMI10ChipBM host port ...
 */

/* waitpid on linux */
#include <sys/types.h>
#include <sys/wait.h>

#include <sys/ioctl.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>
#include <netdb.h>

#include <err.h>
#include <errno.h>
#include <fcntl.h>

#ifdef __linux /* on linux */
#include <pty.h>
#include <utmp.h>
#else /* (__APPLE__ & __MACH__) */
#include <util.h> /* this is for mac or bsd */
#endif

#include <paths.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <pthread.h>

#include "common.h"
#include "hdf5rawWaveformIo.h"
#include "command.h"

static time_t startTime, stopTime;

static unsigned int chMask;
static unsigned int avgMask;
static size_t nCh;
static size_t nEvents;
static struct hdf5io_waveform_file *waveformFile;
static struct hdf5io_waveform_event waveformEvent;
static struct waveform_attribute waveformAttr;

/******************************************************************************/
#define Sleep(x) (usleep((x)*5000))
/******************************************************************************/

#define MAXSLEEP 2
static int connect_retry(int sockfd, const struct sockaddr *addr, socklen_t alen)
{
    int nsec;
    /* Try to connect with exponential backoff. */
    for (nsec = 1; nsec <= MAXSLEEP; nsec <<= 1) {
        if (connect(sockfd, addr, alen) == 0) {
            /* Connection accepted. */
            return(0);
        }
        /*Delay before trying again. */
        if (nsec <= MAXSLEEP/2)
            sleep(nsec);
    }
    return(-1);
}

static int get_socket(char *host, char *port)
{
    int status;
    struct addrinfo addrHint, *addrList, *ap;
    int sockfd, sockopt;

    memset(&addrHint, 0, sizeof(struct addrinfo));
    addrHint.ai_flags = AI_CANONNAME|AI_NUMERICSERV;
    addrHint.ai_family = AF_INET; /* we deal with IPv4 only, for now */
    addrHint.ai_socktype = SOCK_STREAM;
    addrHint.ai_protocol = 0;
    addrHint.ai_addrlen = 0;
    addrHint.ai_canonname = NULL;
    addrHint.ai_addr = NULL;
    addrHint.ai_next = NULL;

    status = getaddrinfo(host, port, &addrHint, &addrList);
    if(status < 0) {
        error_printf("getaddrinfo: %s\n", gai_strerror(status));
        return status;
    }

    for(ap=addrList; ap!=NULL; ap=ap->ai_next) {
        sockfd = socket(ap->ai_family, ap->ai_socktype, ap->ai_protocol);
        if(sockfd < 0) continue;
        sockopt = 1;
        if(setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, (char*)&sockopt, sizeof(sockopt)) == -1) {
            close(sockfd);
            warn("setsockopt TCP_NODELAY");
            continue;
        }
        sockopt = 1;
        if(setsockopt(sockfd, SOL_SOCKET, SO_KEEPALIVE, (char*)&sockopt, sizeof(sockopt)) == -1) {
            close(sockfd);
            warn("setsockopt SO_KEEPALIVE");
            continue;
        }
        if(connect_retry(sockfd, ap->ai_addr, ap->ai_addrlen) < 0) {
            close(sockfd);
            warn("connect");
            continue;
        } else {
            break; /* success */
        }
    }
    if(ap == NULL) { /* No address succeeded */
        error_printf("Could not connect, tried %s:%s\n", host, port);
        return -1;
    }
    freeaddrinfo(addrList);
    return sockfd;
}

static int query_response_with_timeout(int sockfd, char *queryStr, size_t nbytes, char *respStr,
                                       ssize_t nbytes_ret_exp, struct timeval *tv)
{
    int maxfd;
    fd_set rfd;
    int nsel;
    ssize_t nr, nw;
    size_t ret;

    nw = send(sockfd, queryStr, nbytes, 0);
    if(nw<0) {
        warn("send");
        return (int)nw;
    }
    if(nbytes_ret_exp == 0) return nw;

    ret = 0;
    for(;;) {
        FD_ZERO(&rfd);
        FD_SET(sockfd, &rfd);
        maxfd = sockfd;
        nsel = select(maxfd+1, &rfd, NULL, NULL, tv);
        if(nsel < 0 && errno != EINTR) { /* other errors */
            return nsel;
        }
        if(nsel == 0) { /* timed out */
            warn("select");
            break;
        }
        if(nsel>0 && FD_ISSET(sockfd, &rfd)) {
            nr = read(sockfd, respStr+ret, BUFSIZ-ret);
            // debug_printf("nr = %zd\n", nr);
            if(nr>0) {
                ret += nr;
                if(ret >= nbytes_ret_exp && nbytes_ret_exp > 0) break;
            } else {
                break;
            }
        }
    }
    return (int)ret;
}

static int query_response(int sockfd, char *queryStr, size_t nbytes,
                          char *respStr, ssize_t nbytes_ret_exp)
{
    struct timeval tv = {
        .tv_sec = 0,
        .tv_usec = 500000,
    };
    return query_response_with_timeout(sockfd, queryStr, nbytes, respStr, nbytes_ret_exp, &tv);
}

static void atexit_flush_files(void)
{
    hdf5io_flush_file(waveformFile);
    hdf5io_close_file(waveformFile);
}

static void signal_kill_handler(int sig)
{
    printf("\nstart time = %zd\n", startTime);
    printf("stop time  = %zd\n", time(NULL));
    fflush(stdout);
    
    error_printf("Killed, cleaning up...\n");
    atexit(atexit_flush_files);
    exit(EXIT_SUCCESS);
}

static void *send_and_receive_loop(void *arg)
{
    struct timeval tv; /* tv should be re-initialized in the loop since select
                          may change it after each call */
    int sockfd, maxfd, nsel;
    fd_set rfd, wfd;
    char ibuf[BUFSIZ];
    size_t iEvent = 0;
    ssize_t nr, nw, readTotal;
/*
    FILE *fp;
    if((fp=fopen("log.txt", "w"))==NULL) {
        perror("log.txt");
        return (void*)NULL;
    }
*/
    sockfd = *((int*)arg);

    readTotal = 0;
    for(;;) {
        tv.tv_sec = 10;
        tv.tv_usec = 0;
        FD_ZERO(&rfd);
        FD_SET(sockfd, &rfd);
        FD_ZERO(&wfd);
        FD_SET(sockfd, &wfd);
        maxfd = sockfd;
        nsel = select(maxfd+1, &rfd, &wfd, NULL, &tv);
        if(nsel < 0 && errno != EINTR) { /* other errors */
            warn("select");
            break;
        }
        if(nsel == 0) {
            warn("timed out");
        }
        if(nsel>0) {
            if(FD_ISSET(sockfd, &rfd)) {
                nr = read(sockfd, ibuf, sizeof(ibuf));
                debug_printf("nr = %zd\n", nr);
                if(nr < 0) {
                    warn("read");
                    break;
                }
                readTotal += nr;
//            write(fileno(fp), ibuf, nr);
            }
            if(FD_ISSET(sockfd, &wfd)) {
                strlcpy(ibuf, "CURVENext?\n", sizeof(ibuf));
                nw = write(sockfd, ibuf, 2459); // strnlen(ibuf, sizeof(ibuf)));
                debug_printf("nw = %zd\n", nw);
            }
        }
        if(iEvent >= nEvents) {
            goto end;
        }
        iEvent++;
    }
end:
    debug_printf("readTotal = %zd\n", readTotal);

//    fclose(fp);
    return (void*)NULL;
}

/******************************************************************************/

/******************************************************************************/

int genesys_prepare(int sockfd)
{
    char buf[BUFSIZ];
    uint32_t *buf32, val;
    uint64_t retcmd, cid;
    size_t n;
    ssize_t i;

    buf32 = (uint32_t*)buf;
    
    // configure_adc(sockfd, buf);
    /* dac */
    // configure_dac(sockfd, buf);

    /* wr_addr_begin */
    n = cmd_write_register(&buf32, 12, 0x0000);
    n = query_response(sockfd, buf, n, buf, 0);
    n = cmd_write_register(&buf32, 13, 0x0000); /* no wrap_around */
    n = query_response(sockfd, buf, n, buf, 0);
    /* post_trigger */
    n = cmd_write_register(&buf32, 14, 0x0000);
    n = query_response(sockfd, buf, n, buf, 0);
    n = cmd_write_register(&buf32, 15, 0x0000);
    n = query_response(sockfd, buf, n, buf, 0);
    /* rd_addr_end */
    n = cmd_write_register(&buf32, 16, 0x0000); /* low bits */
    n = query_response(sockfd, buf, n, buf, 0);
    /* data path selection */
    n = cmd_write_register(&buf32, 10, 0x0003);
    n = query_response(sockfd, buf, n, buf, 0);

    /* select channel & average */
    switch(nCh) {
    case 1:
        for(val = 0x0000; (((chMask>>val) & (0x0001)) == 0) && (val<16) ; val++) {;}
        break;
    case 4:
        val = 0x0010;
        if(chMask & 0x000f) val |= 0x0000;
        if(chMask & 0x00f0) val |= 0x0001;
        if(chMask & 0x0f00) val |= 0x0002;
        if(chMask & 0xf000) val |= 0x0003;
        break;
    case 8:
        val = 0x0020;
        if(chMask & 0x00ff) val |= 0x0000;
        if(chMask & 0xff00) val |= 0x0001;
        break;
    case 16:
        val = 0x0030; break;
    default:
        val = 0x0000;
        break;
    }
    val |=  (avgMask<<8);
    n = cmd_write_register(&buf32, 14, val);
    n = query_response(sockfd, buf, n, buf, 0);

    /* reference clock frequency division factor (2**n) */
    n = cmd_write_register(&buf32, 15, 3);
    n = query_response(sockfd, buf, n, buf, 0);

    return 1;
}

int genesys_arm_acquire(int sockfd)
{
    char buf[BUFSIZ];
    uint32_t *buf32;
    uint16_t status;
    int finished;
    size_t n;

    buf32 = (uint32_t*)buf;

    /* disable fmc data fifo wren, disallow trigger */
    n = cmd_write_register(&buf32, 17, 0x0400); /* also high bits of rd_addr_end */
    n = query_response(sockfd, buf, n, buf, 0);
    /* reset (clear) fifos */
    n = cmd_send_pulse(&buf32, 0x200); /* pulse_reg(9) */
    n = query_response(sockfd, buf, n, buf, 0); Sleep(2);
    /* sdram ctrl_reset */
    n = cmd_send_pulse(&buf32, 0x40); /* pulse_reg(6) */
    n = query_response(sockfd, buf, n, buf, 0); Sleep(2);
    /* enable fmc data fifo wren */
//    n = cmd_write_register(&buf32, 13, 0x8800); /* also high bits of rd_addr_end */
//    n = query_response(sockfd, buf, n, buf, 0); Sleep(2);
    /* allow trigger */
//    n = cmd_write_register(&buf32, 13, 0xc800); /* also high bits of rd_addr_end */
//    n = query_response(sockfd, buf, n, buf, 0);
    /* software trigger, for testing */
    Sleep(2);
    n = cmd_send_pulse(&buf32, 0x400); /* pulse_reg(10) */
    n = query_response(sockfd, buf, n, buf, 0);
#if 0    
    finished = 0;
    do {
        /* check if we've got a trigger AND completed write */
        /* allow time to fill the front-end memory */
        Sleep(120);        
        /* check status */
        n = cmd_read_status(&buf32, 0);
        n = query_response(sockfd, buf, n, buf, 4);
        printf("Trigger pointer low: 0x%04x\n", ((uint16_t)buf[2]<<8) | (uint16_t)buf[3]);
        n = cmd_read_status(&buf32, 2);
        n = query_response(sockfd, buf, n, buf, 4);
        status = ((uint16_t)buf[2]<<8) | (uint16_t)buf[3];
        printf("0, RD_BUSY, WR_WRAPPED, WR_BUSY, Trigger pointer high: 0x%04hx\n", status);
        finished = status & 0x2000;
    } while(finished == 0);
#endif
    printf("Got trigger and write is done!\n");
    return 1;
}

int genesys_read_save(int sockfd)
{
#define NBASK (4096 * 32)
    char ibuf[NBASK];
    SCOPE_DATA_TYPE *ibufsd;
    char buf[BUFSIZ];
    uint32_t *buf32;
    size_t nb, ncmd;
    ssize_t n, iCh, iP, i, j;
    /**/
    struct timeval tv; /* tv should be re-initialized in the loop since select
                          may change it after each call */
    int maxfd, nsel;
    fd_set rfd;
    ssize_t readTotal;

    ibufsd = (SCOPE_DATA_TYPE *)ibuf;
    buf32 = (uint32_t*)buf;
    
    /* sdram rd_start */
    Sleep(200);
    n = cmd_send_pulse(&buf32, 0x100); /* pulse_reg(8) */
    n = query_response(sockfd, buf, n, buf, 0); Sleep(2);

    /* read data fifo back */
    ncmd = cmd_read_datafifo(&buf32, NBASK/sizeof(int32_t));

    iP = 0;
    nb = 0;
    while(nb < SCOPE_NCH * SCOPE_MEM_LENGTH_MAX * sizeof(SCOPE_DATA_TYPE)) {
        n = query_response(sockfd, buf, ncmd, NULL, 0);

        readTotal = 0;
        for(;;) {
            tv.tv_sec  = 9;
            tv.tv_usec = 500 * 1000;
            FD_ZERO(&rfd);
            FD_SET(sockfd, &rfd);
            maxfd = sockfd;
            nsel = select(maxfd+1, &rfd, NULL, NULL, &tv);
            if(nsel < 0 && errno != EINTR) { /* other errors */
                warn("select");
                break;
            }
            if(nsel == 0) {
                warn("select");
                error_printf("nb = %zd, readTotal = %zd\n\n", nb, readTotal);
                signal_kill_handler(0);
                break;
            }
            if(nsel>0) {
                if(FD_ISSET(sockfd, &rfd)) {
                    n = read(sockfd, ibuf+readTotal, sizeof(ibuf)-readTotal);
                    // debug_printf("n = %zd\n", n);
                    if(n < 0) {
                        warn("read");
                        break;
                    } else if(n == 0) {
                        warn("read: socket closed");
                        break;
                    }
                    readTotal += n;
                }
            }
            if(readTotal >= NBASK) {
                if(readTotal > NBASK) {
                    error_printf("(readTotal = %zd) > (NBASK = %d)\n", readTotal, NBASK);
                }
                break;
            }
        }
        nb += readTotal;
        
        // printf("received bytes: %zd\n", nb);
        /*
        for(i=0; i<n/sizeof(uint16_t); i++) {
            // printf("%6hd ", buf16[i]>>2);
            printf("0x%04hx ", buf16[i]); fflush(stdout);
            if(i%8 == 7) printf("\n");
        }
        */
        /* test data continuity from a counter */ /*
        for(i=0; i<n/sizeof(uint16_t); i+=8) {
            t2 = 0;
            for(k=4; k<8; k++) {
                t3 = buf16[i+k];
                t2 |= (((t3>>8) & 0x00ffL) | ((t3<<8) & 0x00ff00L)) << ((7-k)*16);
            }
            if(t2 - t1 != 1) printf("t1 = 0x%016lx, t2 = 0x%016lx\n", t1, t2);
            t1 = t2;
        }
                                                  */
        /* fill received data into wavBuf */
        i=0;
        while(i<readTotal/sizeof(SCOPE_DATA_TYPE)) {
            j=0;
            for(iCh=0; iCh<SCOPE_NCH; iCh++) {
                if((chMask >> iCh) & 0x01) {
                    waveformEvent.wavBuf[j * waveformAttr.nPt + iP] = (ibufsd[i]);
                                                  /* lowest 2 bits are 0's for 14-bit adc data */
                    j++;
                    i++;
                }
            }
            iP++;
        }
    }
    printf("event %zd, received bytes: %zd, iP = %zd\n", waveformEvent.eventId, nb, iP);

    return hdf5io_write_event(waveformFile, &waveformEvent);
#undef NBASK    
}

/******************************************************************************/

int main(int argc, char **argv)
{
    char *p, *outFileName, *scopeAddress, *scopePort;
    /**/
    int sockfd;
    unsigned int v, c;
    double dac_offset_volt = 0.0;
    pthread_t wTid;
    ssize_t i;
    size_t nWfmPerChunk = 1;

    if(argc<7) {
        error_printf("%s scopeAdddress scopePort outFileName chMask(0x..) avgMask nEvents nWfmPerChunk\n",
                     argv[0]);
        error_printf("avgMask(0x..): high 4-bit is offset, 2**(low 4-bit) is number of points to average\n");
        return EXIT_FAILURE;
    }
    scopeAddress = argv[1];
    scopePort = argv[2];
    outFileName = argv[3];
    errno = 0;
    chMask = strtol(argv[4], &p, 16);
    v = chMask;
    for(c=0; v; c++) v &= v - 1; /* Brian Kernighan's way of counting bits */
    nCh = c;
    if(errno != 0 || *p != 0 || p == argv[4] || chMask <= 0 || nCh>SCOPE_NCH) {
        error_printf("Invalid chMask input: %s\n", argv[4]);
        return EXIT_FAILURE;
    }
    if((nCh!=1) && (nCh!=4) && (nCh!=8) && (nCh!=16)) {
        error_printf("chMask must represent allowed 4,8,16 channel groups\n");
        return EXIT_FAILURE;
    }
    errno = 0;
    avgMask = strtol(argv[5], &p, 16);
    if(errno != 0 || *p != 0 || p == argv[5]) {
        error_printf("Invalid avgMask input: %s\n", argv[5]);
        return EXIT_FAILURE;
    }
    nEvents = atol(argv[6]);
    if(argc>7)
        nWfmPerChunk = atol(argv[7]);
    
    debug_printf("outFileName: %s, chMask: 0x%04x, nCh: %zd, avgMask: 0x%02x, nEvents: %zd, nWfmPerChunk: %zd\n",
                 outFileName, chMask, nCh, avgMask, nEvents, nWfmPerChunk);

    waveformAttr.chMask  = chMask;
    waveformAttr.nPt     = SCOPE_MEM_LENGTH_MAX * (SCOPE_NCH / nCh);
    waveformAttr.nFrames = 0;
    waveformAttr.dt      = 8.0e-9 * (1<<(0xf & avgMask));
    waveformAttr.t0      = 0.0;
    for(i=0; i<SCOPE_NCH; i++) {
        /* 14bit ADC, filled to high bits of 16-bit words, polarity reversed, 2Vpp full scale */
        waveformAttr.ymult[i] = -2.0 / (1<<16);
        /* ADC outputs 2's compliment, so 0 is 0.0V */
        waveformAttr.yoff[i]  = 0.0;
        /* DAC controlled electrical offset at input */
        waveformAttr.yzero[i] = dac_offset_volt;
    }
        
    sockfd = get_socket(scopeAddress, scopePort);
    if(sockfd < 0) {
        error_printf("Failed to establish a socket.\n");
        return EXIT_FAILURE;
    }

    waveformEvent.wavBuf = (SCOPE_DATA_TYPE*)malloc(
        nCh * waveformAttr.nPt * sizeof(SCOPE_DATA_TYPE));
    if(waveformEvent.wavBuf == NULL) {
        error_printf("Failed to allocate waveformEvent.wavBuf\n");
        return EXIT_FAILURE;
    }

    genesys_prepare(sockfd);
    printf("DAC Offset: %g Volts\n", dac_offset_volt);
    
    waveformFile = hdf5io_open_file(outFileName, nWfmPerChunk, nCh);
    hdf5io_write_waveform_attribute_in_file_header(waveformFile, &waveformAttr);
    
    signal(SIGKILL, signal_kill_handler);
    signal(SIGINT, signal_kill_handler);

//    pthread_create(&wTid, NULL, pop_and_save, &sockfd);

    printf("start time = %zd\n", startTime = time(NULL));

//    send_and_receive_loop(&sockfd);

    for(waveformEvent.eventId = 0; waveformEvent.eventId < nEvents; waveformEvent.eventId++) {
        genesys_arm_acquire(sockfd);        
        genesys_read_save(sockfd);
    }
    
    stopTime = time(NULL);
//    pthread_join(wTid, NULL);

    printf("\nstart time = %zd\n", startTime);
    printf("stop time  = %zd\n", stopTime);

    free(waveformEvent.wavBuf);
    close(sockfd);
    atexit_flush_files();
    return EXIT_SUCCESS;
}
