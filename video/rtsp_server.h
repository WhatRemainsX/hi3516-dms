/*
 * 【说明】rtsp_server：RTSP 服务器的对外接口，只有两个函数：
 *      rtsp_server_start(554)  → 启动，返回句柄
 *      rtsp_server_stop(句柄)   → 停止并释放
 *   rtsp_server 是"不透明类型"（外部看不到内部字段），写法同 rtp_sender。
 *   具体实现和 RTSP 协议的解释见 rtsp_server.c。
 */

#ifndef RTSP_SERVER_H
#define RTSP_SERVER_H

typedef struct rtsp_server rtsp_server;     /* 不透明类型，和 rtp_sender 一样 */

rtsp_server *rtsp_server_start(unsigned short port);
void rtsp_server_stop(rtsp_server *srv);

#endif
