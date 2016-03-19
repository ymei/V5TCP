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
#define TM_NCOL 72
#define TM_NROW 72
static struct config_parameters
{
    int row;
    int col;
    int b4dac_val;
    double b4dac_ib;
    double col_ib;
    double vr8b;
    double arst_vref;
    double csa_vref;
    double ains;
    double addr_grst;
} config_param;

static time_t startTime, stopTime;

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
    val = (0x03<<24) | (0x01 << 20) | (DACVolt(config_param.b4dac_ib) << 4);
    n = cmd_write_register(&buf32, 7, (val & 0xffff0000)>>16);
    n = query_response(sockfd, buf, n, buf, 0);
    n = cmd_send_pulse(&buf32, 0x02); /* pulse_reg(1) */
    n = query_response(sockfd, buf, n, buf, 0);
    n = cmd_write_register(&buf32, 7, val & 0xffff);
    n = query_response(sockfd, buf, n, buf, 0);
    n = cmd_send_pulse(&buf32, 0x02); /* pulse_reg(1) */
    n = query_response(sockfd, buf, n, buf, 0);
    /* write and update output3 : COL_IB */
    val = (0x03<<24) | (0x02 << 20) | (DACVolt(config_param.col_ib) << 4);
    n = cmd_write_register(&buf32, 7, (val & 0xffff0000)>>16);
    n = query_response(sockfd, buf, n, buf, 0);
    n = cmd_send_pulse(&buf32, 0x02); /* pulse_reg(1) */
    n = query_response(sockfd, buf, n, buf, 0);
    n = cmd_write_register(&buf32, 7, val & 0xffff);
    n = query_response(sockfd, buf, n, buf, 0);
    n = cmd_send_pulse(&buf32, 0x02); /* pulse_reg(1) */
    n = query_response(sockfd, buf, n, buf, 0);
    /* write and update output4 : Gring */
    val = (0x03<<24) | (0x03 << 20) | (DACVolt(config_param.addr_grst) << 4);
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
    val = (0x03<<24) | (0x05 << 20) | (DACVolt(config_param.vr8b) << 4);
    n = cmd_write_register(&buf32, 7, (val & 0xffff0000)>>16);
    n = query_response(sockfd, buf, n, buf, 0);
    n = cmd_send_pulse(&buf32, 0x02); /* pulse_reg(1) */
    n = query_response(sockfd, buf, n, buf, 0);
    n = cmd_write_register(&buf32, 7, val & 0xffff);
    n = query_response(sockfd, buf, n, buf, 0);
    n = cmd_send_pulse(&buf32, 0x02); /* pulse_reg(1) */
    n = query_response(sockfd, buf, n, buf, 0);
    /* write and update output7 : ARST_VREF */
    val = (0x03<<24) | (0x06 << 20) | (DACVolt(config_param.arst_vref) << 4);
    n = cmd_write_register(&buf32, 7, (val & 0xffff0000)>>16);
    n = query_response(sockfd, buf, n, buf, 0);
    n = cmd_send_pulse(&buf32, 0x02); /* pulse_reg(1) */
    n = query_response(sockfd, buf, n, buf, 0);
    n = cmd_write_register(&buf32, 7, val & 0xffff);
    n = query_response(sockfd, buf, n, buf, 0);
    n = cmd_send_pulse(&buf32, 0x02); /* pulse_reg(1) */
    n = query_response(sockfd, buf, n, buf, 0);
    /* write and update output5 : CSA_VREF */
    val = (0x03<<24) | (0x04 << 20) | (DACVolt(config_param.csa_vref) << 4);
    n = cmd_write_register(&buf32, 7, (val & 0xffff0000)>>16);
    n = query_response(sockfd, buf, n, buf, 0);
    n = cmd_send_pulse(&buf32, 0x02); /* pulse_reg(1) */
    n = query_response(sockfd, buf, n, buf, 0);
    n = cmd_write_register(&buf32, 7, val & 0xffff);
    n = query_response(sockfd, buf, n, buf, 0);
    n = cmd_send_pulse(&buf32, 0x02); /* pulse_reg(1) */
    n = query_response(sockfd, buf, n, buf, 0);
    /* write and update output8 : AINS */
    val = (0x03<<24) | (0x07 << 20) | (DACVolt(config_param.ains) << 4);
    n = cmd_write_register(&buf32, 7, (val & 0xffff0000)>>16);
    n = query_response(sockfd, buf, n, buf, 0);
    n = cmd_send_pulse(&buf32, 0x02); /* pulse_reg(1) */
    n = query_response(sockfd, buf, n, buf, 0);
    n = cmd_write_register(&buf32, 7, val & 0xffff);
    n = query_response(sockfd, buf, n, buf, 0);
    n = cmd_send_pulse(&buf32, 0x02); /* pulse_reg(1) */
#undef DACVolt
    Sleep(20); /* Wait for shiftreg to finish */

    return 1;
}

int configure_sram(int sockfd, int row, int col, int val)
{
    uint32_t *buf32=NULL, *aval, v;
    char *buf;
    size_t n, nval;
    ssize_t i;

    nval = 1296; /* 72 * 72 / 4 */
    aval = (uint32_t*)calloc(nval, sizeof(uint32_t));

    v = 0x10 | (val & 0x0f);
    for(i=0; i<nval; i++) {
        aval[i] = 0;
        if((int)(i/(TM_NCOL/4)) == row) {
            aval[i] = v<<24 | v<<16 | v<<8 | v;
        }
    }
    // hack
    // aval[18*row+(col/4)] = v<<((col%4)*8);
    
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
    configure_sram(sockfd, config_param.row, config_param.col, config_param.b4dac_val);
    /* write sram */
    n = cmd_send_pulse(&buf32, 0x08); /* pulse_reg(3) */
    n = query_response(sockfd, buf, n, buf, 0);

/* force a trigger */
    // Sleep(100);
    // n = cmd_send_pulse(&buf32, 0x01); /* pulse_reg(0) */
    // n = query_response(sockfd, buf, n, buf, 0);

    /* stop on a pixel */
    /* bit 15 enables stop_control, the rest of bits set the stop_address within a frame */

    n = cmd_write_register(&buf32, 3, 0x8000 | (config_param.row * TM_NCOL + config_param.col));
    n = query_response(sockfd, buf, n, buf, 0);
 
    /* digital part */
    /* (bit 3 downto 0) controls digital clock f_CLK/2**(bit 3 downto 0) */
    n = cmd_write_register(&buf32, 8, 0x0007);
    n = query_response(sockfd, buf, n, buf, 0);

    Sleep(2); 
    return 1;
}

int analyze_data(const char *buf, size_t n, size_t *hits, size_t *nframe)
{
    uint32_t *ibuf32, v, cnti, cntd, markerd, readyd, timed, addrd;
    // size_t nframe=0, hits[TM_NCOL] = {0};
    ssize_t i, icol;

    ibuf32 = (uint32_t*)buf;

    icol = -1;
    // nframe = 0;
    for(i=0; i<n/sizeof(uint32_t); i++) {
        /* big endian to little endian */
        v = ibuf32[i]>>24 | (ibuf32[i]>>8 & 0x0000ff00) |
            (ibuf32[i]<<8 & 0x00ff0000) | ibuf32[i]<<24;
        cntd    = v>>19;
        markerd = (v>>18) & 0x1;
        readyd  = (v>>17) & 0x01;
        timed   = (v>>7)  & 0x1ff;
        addrd    = v      & 0x7f;
        //printf("0x%04x %d %d 0x%03x 0x%02x\n", v>>19, (v>>18)&0x1, (v>>17)&0x1, (v>>7)&0x1ff, v&0x7f);
        if(markerd) {
            icol = 0; cnti = cntd;
            (*nframe)++;
        }
        if(icol < 0) { /* looking for the first markerd */
            continue;
        }
        if(cntd < cnti) { /* counter wrapped around */
            icol = ((1<<13) | cntd) - cnti;
        } else {
            icol = cntd - cnti;
        }
        if(icol < 0 || icol >= TM_NCOL) {
            error_printf("icol = %zd\n", icol);
            break;
        }
        if(readyd)
            hits[icol]++;
    }
    return 1;
}

int tm_digital_read(int sockfd, size_t nframemax, FILE *fp)
{
#define NBASK (4096*4)
    char ibuf[NBASK];
    char buf[BUFSIZ];
    uint32_t *ibuf32, *buf32, v;
    size_t nb, ncmd;
    /* for analysis */
    size_t nframe=0, hits[TM_NCOL] = {0};
    ssize_t n, iCh, iP, i, j;
    /**/
    struct timeval tv; /* tv should be re-initialized in the loop since select()
                          may change it after each call */
    int maxfd, nsel;
    fd_set rfd;
    ssize_t readTotal;

    buf32 = (uint32_t*)buf;
    ibuf32 = (uint32_t*)ibuf;

    while(nframe < nframemax) {
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
                tv.tv_usec = 100 * 1000;
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
               if(t2 - t1 != 1)jprintf("t1 = 0x%016lx, t2 = 0x%016lx\n", t1, t2);
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
        analyze_data(ibuf, nb, hits, &nframe);
    }

    for(i=0; i<TM_NCOL; i++) {
        fprintf(fp, "%-2zd %zd %zd\n", i, hits[i], nframe);
    }
    
    return nframe;
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
    size_t nframemax;
    FILE *fp;

    config_param.row = 0;
    config_param.col = 0;
    config_param.b4dac_val = 0x0;
    config_param.b4dac_ib = 0.695;
    config_param.col_ib = 0.987;
    config_param.vr8b = 0.630;
    config_param.arst_vref = 0.818;
    config_param.csa_vref = 0.618;
    config_param.ains = 0.0;
    config_param.addr_grst = 0.0;

    if(argc<5 || (argc>5 && argc<15)) {
        error_printf("%s scopeAdddress scopePort outFileName nframemax\n"
                     "[row col 4bdac_val 4bdac_ib col_ib vr8b arst_vref csa_vref ains addr_grst]\n",
                     argv[0]);
        return EXIT_FAILURE;
    }
    scopeAddress = argv[1];
    scopePort = argv[2];
    outFileName = argv[3];
    nframemax = atol(argv[4]);

    if(argc>5) {
        config_param.row       = strtol(argv[5], &p, 10);
        config_param.col       = strtol(argv[6], &p, 10);
        config_param.b4dac_val = strtol(argv[7], &p, 16);
        config_param.b4dac_ib  = strtod(argv[8], &p);
        config_param.col_ib    = strtod(argv[9], &p);
        config_param.vr8b      = strtod(argv[10], &p);
        config_param.arst_vref = strtod(argv[11], &p);
        config_param.csa_vref  = strtod(argv[12], &p);
        config_param.ains      = strtod(argv[13], &p);
        config_param.addr_grst = strtod(argv[14], &p);
    }
    if(errno) {
        error_printf("Value interpretation error.\n");
        return EXIT_FAILURE;
    }
    
    sockfd = get_socket(scopeAddress, scopePort);
    if(sockfd < 0) {
        error_printf("Failed to establish a socket.\n");
        return EXIT_FAILURE;
    }

    signal(SIGKILL, signal_kill_handler);
    signal(SIGINT, signal_kill_handler);


    if((fp = fopen(outFileName, "w")) == NULL) {
        perror(outFileName);
        return EXIT_FAILURE;
    }
    
//    pthread_create(&wTid, NULL, pop_and_save, &sockfd);

    fprintf(stderr, "start time = %zd\n", startTime = time(NULL));

//    send_and_receive_loop(&sockfd);

    buf32 = (uint32_t*)buf;

    configure_dac(sockfd, buf);
    configure_topmetal(sockfd, buf);
    tm_digital_read(sockfd, nframemax, fp);
    
    stopTime = time(NULL);
//    pthread_join(wTid, NULL);

    fprintf(stderr, "\nstart time = %zd\n", startTime);
    fprintf(stderr, "stop time  = %zd\n", stopTime);

    close(sockfd);
    fclose(fp);
    atexit_flush_files();
    return EXIT_SUCCESS;
}
