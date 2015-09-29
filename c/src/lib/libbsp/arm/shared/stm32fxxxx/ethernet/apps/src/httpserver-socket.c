/**
  ******************************************************************************
  * @file    LwIP/LwIP_HTTP_Server_Socket_RTOS/Src/httpserver-socket.c
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    25-May-2015  
  * @brief   Basic http server implementation using LwIP socket API   
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */
  
#include "lwip/opt.h"
#include "lwip/arch.h"
#include "lwip/api.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"

#include "string.h"
#include "cmsis_os.h"
#include <httpserver-socket.h>
#include <hal-ethernetif.h>
#include <cmsis_os.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define WEBSERVER_THREAD_PRIO    ( tskIDLE_PRIORITY + 4 )

u32_t nPageHits = 0;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static   portCHAR PAGE_HEADER_DYNAMIC[512] = "<!DOCTYPE html PUBLIC \"-//W3C//DTD HTML 4.01//EN" "http://www.w3.org/TR/html4/strict.dtd\"><html><head><title>Next Gen Control Platform Task List</title><meta http-equiv=\"Content-Type\"content=\"text/html; charset=windows-1252\"><meta http-equiv=\"refresh\" content=\"1\"><style =\"font-weight: normal; font-family: Verdana;\"></style></head><body><h1>List of NextGen Tasks</h1><a href=\"/\">Main Page</a><p>";
static   portCHAR DYNAMIC_PAGE_CONTENT[2048];
static   portCHAR main_page[4096] = "<html><head><meta http-equiv=\"content-type\" content=\"text/html; charset=utf-8\"><title>Next Gen Main Page</title></head><body lang=\"en-US\" dir=\"ltr\" style=\"background: transparent\"><h1><font face=\"Courier 10 Pitch\">Vecna NextGen Platform Web Server</font></h1><p><a href=\"/list_of_tasks.html\">List of tasks</a></body></html>";
static   portCHAR error_page[4096] = "<html><head><meta http-equiv=\"content-type\" content=\"text/html; charset=utf-8\"><title></title></head><body lang=\"en-US\" dir=\"ltr\" style=\"background: transparent\"><h1><font face=\"Courier 10 Pitch\">404: Requested page does not exist</font></h1><p><br><br></p><a href=\"/\">Main page</a></body></html>";



static osThreadDef_t web_server_task;


__attribute__ ((weak)) bool http_server_server_app_specific(char* recv_buffer, portCHAR** output_buffer)
{
  // override this function in the application to process any non-default
  // pages.
  return false;
}

/**
  * @brief serve tcp connection  
  * @param conn: connection sockestm32f_ethernet_get_num_rx_msgt
  * @retval None
  */
__attribute__ ((weak))  void http_server_serve(int conn)
{
  int buflen = 1500;
  int ret;
  char recv_buffer[1500];
  portCHAR* pageContent;
				
  // Read in the request
  ret = read(conn, recv_buffer, buflen);

  if(ret < 0) return;

  if(strncmp(recv_buffer, "GET /list_of_tasks.html", 23) == 0 )
  {
    // load dynamic page with list RTEMS tasks
    pageContent = http_generate_platform_stats_page();
  }
  else if(strncmp(recv_buffer, "GET / ", 6) == 0)
  {
    // load main page
    pageContent = http_generate_main_page();
  }
  else if(http_server_server_app_specific(recv_buffer, &pageContent) == false)
  {
    pageContent = http_generate_error_page();
  }

  write(conn, pageContent, strlen(pageContent));

  // close connection socket
  close(conn);
}

/**
  * @brief  http server thread 
  * @param arg: pointer on argument(not used here) 
  * @retval None
  */
void http_server_socket_thread(void *arg)
{
  int sock, newconn, size;
  struct sockaddr_in address, remotehost;

 /* create a TCP socket */
  if ((sock = socket(AF_INET, SOCK_STREAM, 0 )) < 0)
  {
    return;
  }
  
  /* bind to port 80 at any interface */
  address.sin_family = AF_INET;
  address.sin_port = htons(80);
  address.sin_addr.s_addr = INADDR_ANY;

  if (bind(sock, (struct sockaddr *)&address, sizeof (address)) < 0)
  {
    return;
  }
  
  /* listen for incoming connections (TCP listen backlog = 5) */
  listen(sock, 5);
  
  size = sizeof(remotehost);
  
  while (1) 
  {
    newconn = accept(sock, (struct sockaddr *)&remotehost, (socklen_t *)&size);
    http_server_serve(newconn);
  }
}

/**
  * @brief  Initialize the HTTP server (start its thread) 
  * @param  none
  * @retval None
  */
void http_server_socket_init()
{

#if 0
 web_server_task.pthread   = http_server_socket_thread;
 web_server_task.tpriority = osPriorityNormal;
 web_server_task.instances = 1;
 web_server_task.stacksize = 10*1024;

 osThreadCreate (&web_server_task, NULL);
#else
 sys_thread_new("HTTP", http_server_socket_thread, NULL, 0, WEBSERVER_THREAD_PRIO);
#endif
}

portCHAR* http_generate_platform_stats_page(void){
  portCHAR dynamic_text[64] = {0};
  portCHAR headerRow[128];
  memset(DYNAMIC_PAGE_CONTENT, 0, sizeof(DYNAMIC_PAGE_CONTENT));

  /* Update the hit count */
  nPageHits++;
  strcat(DYNAMIC_PAGE_CONTENT, PAGE_HEADER_DYNAMIC);
  sprintf(dynamic_text, "Page refresh count %d", (int)nPageHits);
  strcat(DYNAMIC_PAGE_CONTENT, dynamic_text);
  sprintf(dynamic_text, "<p>Number of Ethernet Packets (RX: %lu, TX %lu, TX DMA buffer err: %lu)", stm32f_ethernet_get_num_rx_msg(), stm32f_ethernet_get_num_tx_msg(), stm32f_ethernet_get_num_tx_dma_buffer_unavailable());
  strcat(DYNAMIC_PAGE_CONTENT, dynamic_text);
  snprintf(headerRow, sizeof(headerRow), "<pre><br>%4s\t%16s\t%8s\t%10s\t%8s", "Name", "Task State", "Stk Addr", "Stk Sz", "rtems_id");
  strcat((char *)DYNAMIC_PAGE_CONTENT, headerRow);
  strcat((char *)DYNAMIC_PAGE_CONTENT, "<br>------------------------------------------------------------------------------<br>");

  /* The list of tasks and their status */
  osThreadList((unsigned char *)(DYNAMIC_PAGE_CONTENT + strlen(DYNAMIC_PAGE_CONTENT)));
  strcat((char *)DYNAMIC_PAGE_CONTENT, "<br>--------------------------------------------------------------------------");

  return (portCHAR*) DYNAMIC_PAGE_CONTENT;
}

portCHAR* http_generate_main_page(void){
  return (portCHAR*) main_page;
}

portCHAR* http_generate_error_page(void){
  return (portCHAR*) error_page;
}



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
