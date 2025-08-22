#pragma once
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/ioctl.h>

#define MAX_CAN_CH		2

class CCAN_If
{
public:
	CCAN_If();
	virtual ~CCAN_If();

protected:
	const char* m_CAN_IfN;			// CAN Interface Name
	struct ifreq m_CAN_IfR;     	// CAN Interface Requirement
	struct sockaddr_can m_CAN_SA;   // Can Socket Address

protected:
	int m_CAN_C;					// CAN Channel
	int m_CAN_F;					// CAN File descriptor
	int m_CAN_S;				  	// CAN Status

public:
	void Initialize();
	void Finalize();

	void Set_CAN_If_Ch(int Ch){m_CAN_C = Ch;}
	int Get_CAN_If_Ch(){return m_CAN_C;}

	int Create_CAN_If();
	void Delete_CAN_If(){close(m_CAN_F);}

	struct can_frame Read_CAN();
	int Write_CAN(can_frame Frame);
};