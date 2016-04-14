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
 * TMII-Analog host port ...
 * For setting analog operation conditions for Topmetal-II-
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
#include <getopt.h>

#include "common.h"
#include "command.h"

#define TM_NCOL 72
#define TM_NROW 72
struct config_parameters
{
    char *scopeAddress;
    char *scopePort;
    int stop_row;
    int stop_col;
    uint16_t clk_src;
    uint16_t clk_div;
    double b4dac_ib;
    double col_ib;
    double vr8b;
    double arst_vref;
    double csa_vref;
    double ains;
    double gring;
};
struct config_parameters config_param,
    config_param_defaults={"192.168.2.3", "1024", -1, -1, 0x0000, 0x0001, 0.684, 0.9, 3.078, 0.8, 0.6, 0.0, 0.0};
static time_t startTime, stopTime;

static int parse_opts(int argc, char* const *argv)
{
    char *p, *pname;
    ssize_t i;
    int ch;
    double val;
    long ival;
    
    /* options descriptor */
    static struct option longopts[] = {
        { "b4dac_ib",  required_argument, NULL, 'd' },
        { "col_ib",    required_argument, NULL, 'c' },
        { "vr8b",      required_argument, NULL, 'e' },
        { "arst_vref", required_argument, NULL, 'r' },
        { "csa_vref",  required_argument, NULL, 'b' },
        { "ains",      required_argument, NULL, 'i' },
        { "gring",     required_argument, NULL, 'g' },
        { "stop_row",  required_argument, NULL, 'y' },
        { "stop_col",  required_argument, NULL, 'x' },
        { "clk_src",   required_argument, NULL, 's' },
        { "clk_div",   required_argument, NULL, 'v' },        
        { NULL, 0, NULL, 0 }
    };
    memcpy(&config_param, &config_param_defaults, sizeof(config_param));
    pname = argv[0];
    while((ch = getopt_long(argc, argv, "d:c:e:r:b:i:g:y:x:s:v:", longopts, NULL)) != -1) {
        switch(ch) {
        case 'd':
            val = strtod(optarg, &p);
            if(p == optarg) {
                error_printf("Double float conversion error.\n");
            } else {
                config_param.b4dac_ib = val;
            }
            break;
        case 'c':
            val = strtod(optarg, &p);
            if(p == optarg) {
                error_printf("Double float conversion error.\n");
            } else {
                config_param.col_ib = val;
            }
            break;
        case 'e':
            val = strtod(optarg, &p);
            if(p == optarg) {
                error_printf("Double float conversion error.\n");
            } else {
                config_param.vr8b = val;
            }
            break;
        case 'r':
            val = strtod(optarg, &p);
            if(p == optarg) {
                error_printf("Double float conversion error.\n");
            } else {
                config_param.arst_vref = val;
            }
            break;
        case 'b':
            val = strtod(optarg, &p);
            if(p == optarg) {
                error_printf("Double float conversion error.\n");
            } else {
                config_param.csa_vref = val;
            }
            break;
        case 'i':
            val = strtod(optarg, &p);
            if(p == optarg) {
                error_printf("Double float conversion error.\n");
            } else {
                config_param.ains = val;
            }
            break;
        case 'g':
            val = strtod(optarg, &p);
            if(p == optarg) {
                error_printf("Double float conversion error.\n");
            } else {
                config_param.gring = val;
            }
            break;
        case 'y':
            ival = strtol(optarg, &p, 10);
            if(p == optarg) {
                error_printf("Integer conversion error.\n");
            } else {
                config_param.stop_row = ival;
            }
            break;
        case 'x':
            ival = strtol(optarg, &p, 10);
            if(p == optarg) {
                error_printf("Integer conversion error.\n");
            } else {
                config_param.stop_col = ival;
            }
            break;
        case 's':
            ival = strtol(optarg, &p, 16);
            if(p == optarg) {
                error_printf("Integer conversion error.\n");
            } else {
                config_param.clk_src = ival;
            }
            break;
        case 'v':
            ival = strtol(optarg, &p, 16);
            if(p == optarg) {
                error_printf("Integer conversion error.\n");
            } else {
                config_param.clk_div = ival;
            }
            break;
        case '?':
        case 'h':
        default:
            goto usage;
        }
    }
    argc -= optind;
    argv += optind;
    if(argc < 2) {
    usage:
        error_printf("%s [\n", pname);
        i = 0;
        while(longopts[i].name) {
            error_printf("    --%-20s or -%c\n", longopts[i].name, longopts[i].val);
            i++;
        }
        error_printf("] ip port\n");
        return 0;
    }

    config_param.scopeAddress = argv[0];
    config_param.scopePort = argv[1];
    
    return argc;
}

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
    val = (0x03<<24) | (0x03 << 20) | (DACVolt(config_param.gring) << 4);
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
    n = query_response(sockfd, buf, n, buf, 0);
#undef DACVolt
    Sleep(20); /* Wait for shiftreg to finish */

    return 1;
}

int configure_topmetal(int sockfd, char *buf)
{
    uint32_t *buf32;
    size_t n;

    buf32 = (uint32_t*)buf;
    /* select clock source */
    n = cmd_write_register(&buf32, 6, config_param.clk_src);
    n = query_response(sockfd, buf, n, buf, 0);
    /* trigger rate control, 1 trigger every val frames */
    n = cmd_write_register(&buf32, 5, 0x0001);
    n = query_response(sockfd, buf, n, buf, 0);
    /* trigger delay, trigger_out at val TM_CLK cycles after new frame starts */
    n = cmd_write_register(&buf32, 4, 0x0000);
    n = query_response(sockfd, buf, n, buf, 0);
    /* (bit 3 downto 0) controls TM_CLK = f_CLK/2**(bit 3 downto 0) */
    /* bit 15 sets the output of EX_RST, bit 14 vetos trigger_out */
    n = cmd_write_register(&buf32, 2, 0x4000 | config_param.clk_div);
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
    n = cmd_write_register(&buf32, 2, config_param.clk_div);
    n = query_response(sockfd, buf, n, buf, 0);
    /* write sram */
    n = cmd_send_pulse(&buf32, 0x08); /* pulse_reg(3) */
    n = query_response(sockfd, buf, n, buf, 0);

/* force a trigger */
    // Sleep(100);
    // n = cmd_send_pulse(&buf32, 0x01); /* pulse_reg(0) */
    // n = query_response(sockfd, buf, n, buf, 0);

    /* stop on a pixel */
    /* bit 15 enables stop_control, the rest of bits set the stop_address within a frame */
    if(config_param.stop_row >= 0 && config_param.stop_col >= 0) {
        n = cmd_write_register(&buf32, 3, 0x8000 | (config_param.stop_row * TM_NCOL + config_param.stop_col));
        n = query_response(sockfd, buf, n, buf, 0);
    }

    return 1;
}


int main(int argc, char **argv)
{
    char buf[BUFSIZ];
    uint32_t *buf32;
    int sockfd;
    // pthread_t wTid;

    if(parse_opts(argc, argv)==0) {
        return EXIT_FAILURE;
    }

    printf("Settings: \n");
    printf("scopeAddress: %s\n",  config_param.scopeAddress);
    printf("scopePort: %s\n",     config_param.scopePort);
    printf("stop_row:  %d\n",     config_param.stop_row);
    printf("stop_col:  %d\n",     config_param.stop_col);
    printf("clk_src:   0x%04x\n", config_param.clk_src);
    printf("clk_div:   0x%04x\n", config_param.clk_div);
    printf("b4dac_ib:  %g\n",     config_param.b4dac_ib);
    printf("col_ib:    %g\n",     config_param.col_ib);
    printf("vr8b:      %g\n",     config_param.vr8b);
    printf("arst_vref: %g\n",     config_param.arst_vref);
    printf("csa_vref:  %g\n",     config_param.csa_vref);
    printf("ains:      %g\n",     config_param.ains);
    printf("gring:     %g\n",     config_param.gring);

    sockfd = get_socket(config_param.scopeAddress, config_param.scopePort);
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

    configure_dac(sockfd, buf);
    configure_topmetal(sockfd, buf);

    stopTime = time(NULL);
//    pthread_join(wTid, NULL);

    printf("\nstart time = %zd\n", startTime);
    printf("stop time  = %zd\n", stopTime);

    close(sockfd);
    atexit_flush_files();
    return EXIT_SUCCESS;
}
