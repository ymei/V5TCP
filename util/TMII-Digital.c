/*
 * Copyright (c) 2013 - 2016
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
 * tcpio host port ...
 */

/* waitpid on linux */
#include <sys/types.h>
#include <sys/wait.h>

#include <sys/ioctl.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>
#include <netinet/in.h>
#include <netdb.h>

#include <err.h>
#include <errno.h>
#include <fcntl.h>

#if defined(__linux) /* on linux */
#include <pty.h>
#include <utmp.h>
#elif defined(__FreeBSD__)
#include <libutil.h>
#elif defined(__APPLE__) && defined(__MACH__)
#include <util.h>
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
#include "command.h"

static time_t startTime, stopTime;
static unsigned int chMask;
static size_t nCh;
static size_t nEvents;

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
    if(nbytes_ret_exp == 0) return 0;

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
    /* hdf5io_flush_file(waveformFile); */
    /* hdf5io_close_file(waveformFile); */
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

int configure_dac(int sockfd, char *buf)
{
    uint32_t *buf32, val;
    size_t n;
    buf32 = (uint32_t*)buf;

    /* DAC8568 for TopmetalII- */    
    /* turn on internal vref = 2.5V, so the output is val/65536.0 * 5.0 [V] */
#define DACVolt(x) ((uint16_t)((double)(x)/5.0 * 65536.0))
    n = cmd_write_register(&buf32, 7, 0x0800);
    n = query_response(sockfd, buf, n, buf, 0);
    n = cmd_send_pulse(&buf32, 0x02); /* pulse_reg(1) */
    n = query_response(sockfd, buf, n, buf, 0);
    n = cmd_write_register(&buf32, 7, 0x0001);
    n = query_response(sockfd, buf, n, buf, 0);
    n = cmd_send_pulse(&buf32, 0x02); /* pulse_reg(1) */
    n = query_response(sockfd, buf, n, buf, 0);
    /* write and update output1 : DAC_EX_VREF */
    val = (0x03<<24) | (0x00 << 20) | (DACVolt(2.5) << 4);
    n = cmd_write_register(&buf32, 7, (val & 0xffff0000)>>16);
    n = query_response(sockfd, buf, n, buf, 0);
    n = cmd_send_pulse(&buf32, 0x02); /* pulse_reg(1) */
    n = query_response(sockfd, buf, n, buf, 0);
    n = cmd_write_register(&buf32, 7, val & 0xffff);
    n = query_response(sockfd, buf, n, buf, 0);
    n = cmd_send_pulse(&buf32, 0x02); /* pulse_reg(1) */
    n = query_response(sockfd, buf, n, buf, 0);
    /* write and update output2 : 4BDAC_IB */
    val = (0x03<<24) | (0x01 << 20) | (DACVolt(0.684) << 4);
    n = cmd_write_register(&buf32, 7, (val & 0xffff0000)>>16);
    n = query_response(sockfd, buf, n, buf, 0);
    n = cmd_send_pulse(&buf32, 0x02); /* pulse_reg(1) */
    n = query_response(sockfd, buf, n, buf, 0);
    n = cmd_write_register(&buf32, 7, val & 0xffff);
    n = query_response(sockfd, buf, n, buf, 0);
    n = cmd_send_pulse(&buf32, 0x02); /* pulse_reg(1) */
    n = query_response(sockfd, buf, n, buf, 0);
    /* write and update output3 : COL_IB */
    val = (0x03<<24) | (0x02 << 20) | (DACVolt(0.9) << 4);
    n = cmd_write_register(&buf32, 7, (val & 0xffff0000)>>16);
    n = query_response(sockfd, buf, n, buf, 0);
    n = cmd_send_pulse(&buf32, 0x02); /* pulse_reg(1) */
    n = query_response(sockfd, buf, n, buf, 0);
    n = cmd_write_register(&buf32, 7, val & 0xffff);
    n = query_response(sockfd, buf, n, buf, 0);
    n = cmd_send_pulse(&buf32, 0x02); /* pulse_reg(1) */
    n = query_response(sockfd, buf, n, buf, 0);
    /* write and update output4 : Gring */
    val = (0x03<<24) | (0x03 << 20) | (DACVolt(0.0) << 4);
    n = cmd_write_register(&buf32, 7, (val & 0xffff0000)>>16);
    n = query_response(sockfd, buf, n, buf, 0);
    n = cmd_send_pulse(&buf32, 0x02); /* pulse_reg(1) */
    n = query_response(sockfd, buf, n, buf, 0);
    n = cmd_write_register(&buf32, 7, val & 0xffff);
    n = query_response(sockfd, buf, n, buf, 0);
    n = cmd_send_pulse(&buf32, 0x02); /* pulse_reg(1) */
    n = query_response(sockfd, buf, n, buf, 0);
    /* TopmetalII- internal DAC */
#if 0
    /*    ARST_VREF    VR8B          CSA_VREF */
    val = (0x4f<<16) | (0xfc << 8) | (0x34);
    n = cmd_write_register(&buf32, 8, (val & 0xffff0000)>>16);
    n = query_response(sockfd, buf, n, buf, 0);
    n = cmd_send_pulse(&buf32, 0x04); /* pulse_reg(2) */
    n = query_response(sockfd, buf, n, buf, 0);
    n = cmd_write_register(&buf32, 8, val & 0xffff);
    n = query_response(sockfd, buf, n, buf, 0);
    n = cmd_send_pulse(&buf32, 0x04); /* pulse_reg(2) */
    n = query_response(sockfd, buf, n, buf, 0);
#endif
    /* use external DAC to set bias */
    /* write and update output6 : VR8B */
    val = (0x03<<24) | (0x05 << 20) | (DACVolt(0.824) << 4);
    n = cmd_write_register(&buf32, 7, (val & 0xffff0000)>>16);
    n = query_response(sockfd, buf, n, buf, 0);
    n = cmd_send_pulse(&buf32, 0x02); /* pulse_reg(1) */
    n = query_response(sockfd, buf, n, buf, 0);
    n = cmd_write_register(&buf32, 7, val & 0xffff);
    n = query_response(sockfd, buf, n, buf, 0);
    n = cmd_send_pulse(&buf32, 0x02); /* pulse_reg(1) */
    n = query_response(sockfd, buf, n, buf, 0);
    /* write and update output7 : ARST_VREF */
    val = (0x03<<24) | (0x06 << 20) | (DACVolt(0.818) << 4);
    n = cmd_write_register(&buf32, 7, (val & 0xffff0000)>>16);
    n = query_response(sockfd, buf, n, buf, 0);
    n = cmd_send_pulse(&buf32, 0x02); /* pulse_reg(1) */
    n = query_response(sockfd, buf, n, buf, 0);
    n = cmd_write_register(&buf32, 7, val & 0xffff);
    n = query_response(sockfd, buf, n, buf, 0);
    n = cmd_send_pulse(&buf32, 0x02); /* pulse_reg(1) */
    n = query_response(sockfd, buf, n, buf, 0);
    /* write and update output5 : CSA_VREF */
    val = (0x03<<24) | (0x04 << 20) | (DACVolt(0.618) << 4);
    n = cmd_write_register(&buf32, 7, (val & 0xffff0000)>>16);
    n = query_response(sockfd, buf, n, buf, 0);
    n = cmd_send_pulse(&buf32, 0x02); /* pulse_reg(1) */
    n = query_response(sockfd, buf, n, buf, 0);
    n = cmd_write_register(&buf32, 7, val & 0xffff);
    n = query_response(sockfd, buf, n, buf, 0);
    n = cmd_send_pulse(&buf32, 0x02); /* pulse_reg(1) */
    n = query_response(sockfd, buf, n, buf, 0);
    /* write and update output8 : AINS */
    val = (0x03<<24) | (0x07 << 20) | (DACVolt(0.000) << 4);
    n = cmd_write_register(&buf32, 7, (val & 0xffff0000)>>16);
    n = query_response(sockfd, buf, n, buf, 0);
    n = cmd_send_pulse(&buf32, 0x02); /* pulse_reg(1) */
    n = query_response(sockfd, buf, n, buf, 0);
    n = cmd_write_register(&buf32, 7, val & 0xffff);
    n = query_response(sockfd, buf, n, buf, 0);
    n = cmd_send_pulse(&buf32, 0x02); /* pulse_reg(1) */
    n = query_response(sockfd, buf, n, buf, 0);
#undef DACVolt
    Sleep(20); /* Wait for shiftreg to finish */

    return 1;
}

int configure_sram(int sockfd, int row, int val)
{
    uint32_t *buf32=NULL, *aval, v;
    char *buf;
    size_t n, nval;
    ssize_t i;

    nval = 1296; /* 72 * 72 / 4 */
    aval = (uint32_t*)calloc(nval, sizeof(uint32_t));

    v=0;
    for(i=0; i<nval; i++) {
        aval[i] = 0;
        if((int)(i/(72/4)) == row) {
            v = 0x10 | (val & 0x0f);
            aval[i] = v<<24 | v<<16 | v<<8 | v;
        }
    }
    
    n = cmd_write_memory(&buf32, 0, aval, nval);
    buf = (char*)buf32;
    n = query_response(sockfd, buf, n, buf, 0);
    
    free(aval);
    free(buf32);
    
    return 1;
}

int configure_topmetal(int sockfd, char *buf)
{
    uint32_t *buf32, val;
    size_t n;

    buf32 = (uint32_t*)buf;
    /* select clock source */
    n = cmd_write_register(&buf32, 6, 0x0000);
    n = query_response(sockfd, buf, n, buf, 0);
    /* trigger rate control, 1 trigger every val frames */
    n = cmd_write_register(&buf32, 5, 0x0001);
    n = query_response(sockfd, buf, n, buf, 0);
    /* trigger delay, trigger_out at val TM_CLK cycles after new frame starts */
    n = cmd_write_register(&buf32, 4, 0x0000);
    n = query_response(sockfd, buf, n, buf, 0);
    /* (bit 3 downto 0) controls TM_CLK = f_CLK/2**(bit 3 downto 0) */
    /* bit 15 sets the output of EX_RST, bit 14 vetos trigger_out */
    n = cmd_write_register(&buf32, 2, 0x4004);
    n = query_response(sockfd, buf, n, buf, 0);
    /* bit 15 enables stop_control, the rest of bits set the stop_address within a frame */
    n = cmd_write_register(&buf32, 3, 0x0a20);
    n = query_response(sockfd, buf, n, buf, 0);
    /* bit 8 [high] resets topmetal_iiminus_analog module */
    n = cmd_write_register(&buf32, 1, 0x0100);
    n = query_response(sockfd, buf, n, buf, 0);
    Sleep(1);
    n = cmd_write_register(&buf32, 1, 0x0000);
    n = query_response(sockfd, buf, n, buf, 0);

    /* allow trigger output since the first trigger out will happen
     * right after sram write.  Pay attention to TM_CLK rate here as well. */
    n = cmd_write_register(&buf32, 2, 0x0004);
    n = query_response(sockfd, buf, n, buf, 0);
    /* load sram data */
    configure_sram(sockfd, 33, 0x0);
    /* write sram */
    n = cmd_send_pulse(&buf32, 0x08); /* pulse_reg(3) */
    n = query_response(sockfd, buf, n, buf, 0);

/* force a trigger */
    // Sleep(100);
    // n = cmd_send_pulse(&buf32, 0x01); /* pulse_reg(0) */
    // n = query_response(sockfd, buf, n, buf, 0);

    /* turn off analog clock */
    n = cmd_write_register(&buf32, 6, 0x0002);
    n = query_response(sockfd, buf, n, buf, 0);
 
    /* digital part */
    /* (bit 3 downto 0) controls digital clock f_CLK/2**(bit 3 downto 0) */
    n = cmd_write_register(&buf32, 8, 0x0006);
    n = query_response(sockfd, buf, n, buf, 0);
   
    Sleep(2); 
    return 1;
}

int tm_digital_read(int sockfd)
{
#define NBASK (4096*4)
    char ibuf[NBASK];
    char buf[BUFSIZ];
    uint32_t *ibuf32, *buf32, v;
    size_t nb, ncmd;
    ssize_t n, iCh, iP, i, j;
    /**/
    struct timeval tv; /* tv should be re-initialized in the loop since select()
                          may change it after each call */
    int maxfd, nsel;
    fd_set rfd;
    ssize_t readTotal;

    buf32 = (uint32_t*)buf;
    ibuf32 = (uint32_t*)ibuf;

    /* reset fifo36: fifo36_trig */
    n = cmd_send_pulse(&buf32, 0x20); /* pulse_reg(5) */
    n = query_response(sockfd, buf, n, buf, 0); Sleep(2);
    Sleep(200);

    /* read data fifo back */
    ncmd = cmd_read_datafifo(&buf32, NBASK/sizeof(uint32_t));

    nb = 0;
    while(nb < NBASK) {
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
                error_printf("select timed out!  nb = %zd, readTotal = %zd\n\n", nb, readTotal);
                // close(sockfd);
                // signal_kill_handler(0);
                readTotal = NBASK;
                break;
            }
            if(nsel>0) {
                if(FD_ISSET(sockfd, &rfd)) {
                    n = read(sockfd, ibuf+readTotal, sizeof(ibuf)-readTotal);
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

        /* test data continuity from a counter */ /*
        for(i=0; i<readTotal/sizeof(uint16_t); i+=8) {
            t2 = 0;
            for(k=4; k<8; k++) {
                t3 = buf16[i+k];
                t2 |= (((t3>>8) & 0x00ffL) | ((t3<<8) & 0x00ff00L)) << ((7-k)*16);
            }
            if(t2 - t1 != 1) printf("t1 = 0x%016lx, t2 = 0x%016lx\n", t1, t2);
            t1 = t2;
        }
                                                  */
    }

    for(i=0; i<nb/sizeof(uint32_t); i++) {
        /* big endian to little endian */
        v = ibuf32[i]>>24 | (ibuf32[i]>>8 & 0x0000ff00) | (ibuf32[i]<<8 & 0x00ff0000)
            | ibuf32[i]<<24;
        printf("0x%04x %d %d 0x%03x 0x%02x\n", v>>19, (v>>18)&0x1, (v>>17)&0x1, (v>>7)&0x1ff, v&0x7f);
    }

    return nb;
#undef NBASK
}


/******************************************************************************/

int main(int argc, char **argv)
{
    char buf[BUFSIZ];
    uint32_t *buf32, val;
    char *p, *outFileName, *scopeAddress, *scopePort;
    unsigned int v, c;
    int sockfd;
    pthread_t wTid;
    ssize_t i;
    size_t n, nWfmPerChunk = 100;

    if(argc<6) {
        error_printf("%s scopeAdddress scopePort outFileName chMask(0x..) nEvents nWfmPerChunk\n",
                     argv[0]);
        return EXIT_FAILURE;
    }
    scopeAddress = argv[1];
    scopePort = argv[2];
    outFileName = argv[3];
    nEvents = atol(argv[5]);

    errno = 0;
    chMask = strtol(argv[4], &p, 16);
    v = chMask;
    for(c=0; v; c++) v &= v - 1; /* Brian Kernighan's way of counting bits */
    nCh = c;
    if(errno != 0 || *p != 0 || p == argv[4] || chMask <= 0 || nCh>SCOPE_NCH) {
        error_printf("Invalid chMask input: %s\n", argv[4]);
        return EXIT_FAILURE;
    }
    if(argc>=7)
        nWfmPerChunk = atol(argv[6]);

    debug_printf("outFileName: %s, chMask: 0x%02x, nCh: %zd, nEvents: %zd, nWfmPerChunk: %zd\n",
                 outFileName, chMask, nCh, nEvents, nWfmPerChunk);

    sockfd = get_socket(scopeAddress, scopePort);
    if(sockfd < 0) {
        error_printf("Failed to establish a socket.\n");
        return EXIT_FAILURE;
    }

    signal(SIGKILL, signal_kill_handler);
    signal(SIGINT, signal_kill_handler);

//    pthread_create(&wTid, NULL, pop_and_save, &sockfd);

    printf("start time = %zd\n", startTime = time(NULL));

//    send_and_receive_loop(&sockfd);

    buf32 = (uint32_t*)buf;

    /* dac value, for usage demo, not in effect */
    // n = cmd_write_register(&buf32, 0, 40632); // 3.1V
    n = cmd_write_register(&buf32, 0, 32768); // 2.5V
    // n = cmd_read_register(&buf32, 3);
    printf("sent: ");
    for(i=0; i<n; i++) {
        printf("%02x ", (unsigned char)buf[i]);
    }
    printf("\n");
    n = query_response(sockfd, buf, n, buf, 0);
    printf("received: ");
    for(i=0; i<n; i++) {
        printf("%02x ", (unsigned char)buf[i]);
    }
    printf("\n");

    configure_dac(sockfd, buf);
    configure_topmetal(sockfd, buf);
    tm_digital_read(sockfd);
    
    stopTime = time(NULL);
//    pthread_join(wTid, NULL);

    printf("\nstart time = %zd\n", startTime);
    printf("stop time  = %zd\n", stopTime);

    close(sockfd);
    atexit_flush_files();
    return EXIT_SUCCESS;
}
