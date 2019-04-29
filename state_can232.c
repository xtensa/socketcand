#include "config.h"
#include "socketcand.h"
#include "statistics.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#include <sys/types.h>
#include <sys/wait.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/uio.h>
#include <net/if.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <syslog.h>

#include <linux/can.h>

int can232_socket;
struct ifreq ifr;
struct sockaddr_can addr;
fd_set readfds;
struct msghdr msg;
struct can_frame frame;
struct iovec iov;
char ctrlmsg[CMSG_SPACE(sizeof(struct timeval)) + CMSG_SPACE(sizeof(__u32))];
struct timeval tv;
struct cmsghdr *cmsg;
int channel_open = 0;
int listen_only_mode = 0;

#define LW232_LAWICEL_VERSION_STR     "V1013"
#define LW232_LAWICEL_SERIAL_NUM      "NA123"
#define LW232_RET_ASCII_OK             0x0D
#define LW232_RET_ASCII_ERROR          0x07
#define LW232_CR    '\r'

void state_can232() 
{
	char buf[MAXLEN];
	char word[9]; // 8 hex symbols + EOL
	int i, ret;
	int add_timestamp = 0;

	if(previous_state != STATE_CAN232) 
	{

		if((can232_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) 
		{
			PRINT_ERROR("Error while creating CAN232 socket %s\n", strerror(errno));
			state = STATE_SHUTDOWN;
			return;
		}

		strcpy(ifr.ifr_name, bus_name);
		if(ioctl(can232_socket, SIOCGIFINDEX, &ifr) < 0) 
		{
			PRINT_ERROR("Error while searching for bus %s\n", strerror(errno));
			state = STATE_SHUTDOWN;
			return;
		}

		addr.can_family = AF_CAN;
		addr.can_ifindex = ifr.ifr_ifindex;

		const int timestamp_on = 1;
		if(setsockopt( can232_socket, SOL_SOCKET, SO_TIMESTAMP, &timestamp_on, sizeof(timestamp_on)) < 0) 
		{
			PRINT_ERROR("Could not enable CAN timestamps\n");
			state = STATE_SHUTDOWN;
			return;
		}

		if(bind(can232_socket, (struct sockaddr *) &addr, sizeof(addr)) < 0) 
		{
			PRINT_ERROR("Error while binding CAN232 socket %s\n", strerror(errno));
			state = STATE_SHUTDOWN;
			return;
		}

		iov.iov_base = &frame;
		msg.msg_name = &addr;
		msg.msg_iov = &iov;
		msg.msg_iovlen = 1;
		msg.msg_control = &ctrlmsg;

		previous_state = STATE_CAN232;
	}

	FD_ZERO(&readfds);
	FD_SET(can232_socket, &readfds);
	FD_SET(client_socket, &readfds);

	/*
	 * Check if there are more elements in the element buffer before calling select() and
	 * blocking for new packets.
	 */
	if(more_elements) 
	{
		FD_CLR(can232_socket, &readfds);
	} 
	else 
	{
		ret = select((can232_socket > client_socket)?can232_socket+1:client_socket+1, &readfds, NULL, NULL, NULL);

		if(ret < 0) 
		{
			PRINT_ERROR("Error in select()\n")
			state = STATE_SHUTDOWN;
			return;
		}
	}

	if(FD_ISSET(can232_socket, &readfds)) 
	{
		iov.iov_len = sizeof(frame);
		msg.msg_namelen = sizeof(addr);
		msg.msg_flags = 0;
		msg.msg_controllen = sizeof(ctrlmsg);

		ret = recvmsg(can232_socket, &msg, 0);
		if(ret < sizeof(struct can_frame)) 
		{
			PRINT_ERROR("Error reading frame from CAN232 socket\n")
		} 
		else if(channel_open) 
		{
			/* read timestamp data */
			for (cmsg = CMSG_FIRSTHDR(&msg);
			     cmsg && (cmsg->cmsg_level == SOL_SOCKET);
			     cmsg = CMSG_NXTHDR(&msg,cmsg)) 
			{
				if (cmsg->cmsg_type == SO_TIMESTAMP) 
				{
					tv = *(struct timeval *)CMSG_DATA(cmsg);
				}
			}

			if(frame.can_id & CAN_ERR_FLAG) 
			{
				canid_t class = frame.can_id  & CAN_EFF_MASK;
				ret = sprintf(buf, "< error %03X %ld.%06ld >", class, tv.tv_sec, tv.tv_usec);
				send(client_socket, buf, strlen(buf), 0);
			} 
			else if(frame.can_id & CAN_RTR_FLAG) 
			{
				/* TODO implement */
			} 
			else 
			{
				if(frame.can_id & CAN_EFF_FLAG) // 29-bit address 
				{
					ret = sprintf(buf, "T%08X", frame.can_id & CAN_EFF_MASK);
				} 
				else // 11-bit address
				{
					ret = sprintf(buf, "t%03X", frame.can_id & CAN_SFF_MASK);
				}

				// print data length
				ret += sprintf(buf+ret, "%d", frame.can_dlc);
				// print data itself
				for(i=0;i<frame.can_dlc;i++) 
				{
					ret += sprintf(buf+ret, "%02X", frame.data[i]);
				}

				if(add_timestamp == 1)
				{
					int timestamp = (tv.tv_sec%60)*1000 + tv.tv_usec/1000;
					ret += sprintf(buf+ret, "%04X", timestamp);
				}

				ret += sprintf(buf+ret, "%c", LW232_CR);
				send(client_socket, buf, strlen(buf), 0);
			}
		}
	}

	if(FD_ISSET(client_socket, &readfds)) 
	{
		ret = receive_command_can232(client_socket, (char *) &buf);

		if(ret == 0) 
		{

			if (state_changed(buf, state)) 
			{
				close(can232_socket);
				strcpy(buf, "< ok >");
				send(client_socket, buf, strlen(buf), 0);
				return;
			}

			switch(buf[0])
			{
				int id_size;

				/* Send a single frame */
				case 't':
				case 'T':
				case 'r':
				case 'R': 
					if(buf[0]=='T' || buf[0]=='R') id_size=8;
					else id_size=3;

					if(!channel_open || listen_only_mode)
					{
						sprintf(buf, "%c", LW232_RET_ASCII_ERROR);
						break;
					}

					// copy ID
					strncpy( word, buf+1, id_size);
					word[id_size]='\0';

					frame.can_id=(int)strtol(word,NULL,16);
					if( id_size==8) frame.can_id |= CAN_EFF_FLAG;
					if( buf[0]=='r' || buf[0]=='R') frame.can_id |= CAN_RTR_FLAG;

					// copy DLC
					word[0] = buf[id_size+1]; 
					word[1] = '\0';
					frame.can_dlc = strtol(word, NULL, 16); 
					if(frame.can_dlc > 8)
					{
						PRINT_ERROR("ERROR: data size cannot be greater then 8, read DLC=%d\n", frame.can_dlc);
						PRINT_ERROR("buffer='%s', id_size=%d\n", buf, id_size);
						sprintf(buf, "%c", LW232_RET_ASCII_ERROR);
						return;
					}

					for(int i=0;i<frame.can_dlc; i++)
					{
						word[0] = buf[id_size + 2 + i*2];
						word[1] = buf[id_size + 2 + i*2+1];
						word[2] = 0;
						frame.data[i] = strtol(word,NULL,16); 
					}

					ret = send(can232_socket, &frame, sizeof(struct can_frame), 0);
					if(ret==-1) 
					{
						PRINT_ERROR("ERROR: cannot send frame to the socket\n");
						state = STATE_SHUTDOWN;
						return;
					}
					// z for auto poll mode
					sprintf(buf, "z%c", LW232_RET_ASCII_OK);
					break;
				case 'L':
					if(!channel_open)
					{
						listen_only_mode = 1;
						channel_open = 1;
						sprintf(buf, "%c", LW232_RET_ASCII_OK);
					}
					else
					{
						sprintf(buf, "%c", LW232_RET_ASCII_ERROR);
					}	
					break;
				case 'O':
					if(!channel_open)
					{
						listen_only_mode = 0;
						channel_open = 1;
						sprintf(buf, "%c", LW232_RET_ASCII_OK);
					}
					else
					{
						sprintf(buf, "%c", LW232_RET_ASCII_ERROR);
					}
					break;
				case 'C':
					if(channel_open)
					{
						channel_open = 0;
						sprintf(buf, "%c", LW232_RET_ASCII_OK);
					}
					else
					{
						sprintf(buf, "%c", LW232_RET_ASCII_ERROR);
					}
					break;
				case 'S':
					sprintf(buf, "%c", LW232_RET_ASCII_OK);
					break;
				case 'P': // buffer poll single - not needed, reply BELL
				case 'A': // buffer poll all
					sprintf(buf, "%c", LW232_RET_ASCII_ERROR);
					break;
				case 'F': // get error status, always return no errors
					sprintf(buf, "F00%c", LW232_RET_ASCII_OK);
					break;
				case 'X': // set auto poll ON/OFF. For us always ON
					if(channel_open)
						sprintf(buf, "%c", LW232_RET_ASCII_ERROR);
					else
						sprintf(buf, "%c", LW232_RET_ASCII_OK);
					break;
				case 'v':
				case 'V':
					sprintf(buf, "%s%c", LW232_LAWICEL_VERSION_STR, LW232_RET_ASCII_OK);
					break;
				case 'N':
					sprintf(buf, "%s%c", LW232_LAWICEL_SERIAL_NUM, LW232_RET_ASCII_OK);
					break;
				case 'Z': // set timestamp ON/OFF
					add_timestamp = buf[1];
					break;
				case 's': // commands that not supported
				case 'm':
				case 'M':
				case 'W':
				case 'U':
				case 'Q':
					PRINT_ERROR("Unsupported command received: %c\n", buf[0]);
					sprintf(buf, "%c", LW232_RET_ASCII_ERROR);
					break;
				default:
					PRINT_ERROR("unknown command '%s'\n", buf);
					strcpy(buf, "< error unknown command >\n");
			}
			send(client_socket, buf, strlen(buf), 0);
		} 
		else 
		{
			PRINT_VERBOSE("Normal switch to SHUTDOWN mode\n");
			state = STATE_SHUTDOWN;
			return;
		}
	} 
	else 
	{
		ret = read(client_socket, &buf, 0);
		if(ret==-1) 
		{
			state = STATE_SHUTDOWN;
			return;
		}
	}
}
