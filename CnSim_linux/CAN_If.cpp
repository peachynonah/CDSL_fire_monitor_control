#define DEBUG false

#include "CAN_If.h"

CCAN_If::CCAN_If()
{
}
CCAN_If::~CCAN_If()
{
}

void CCAN_If::Initialize()
{
	if (Create_CAN_If() == -1)
	{
		if (DEBUG)
			printf(" : CAN[%d] isn't initialized. \r\n\n", m_CAN_C);
	}
	else
	{
		if (DEBUG)
			printf(" : CAN[%d] is initialized. \r\n\n", m_CAN_C);
	}
}

void CCAN_If::Finalize()
{
	Delete_CAN_If();

	if (DEBUG)
		printf(" : CAN[%d] is finalized. \r\n\n", m_CAN_C);
}

int CCAN_If::Create_CAN_If()
{
	// Open SocketCAN //
	m_CAN_F = socket(PF_CAN, SOCK_RAW, CAN_RAW);

	if (m_CAN_F == -1)
	{
		if (DEBUG)
			printf(" : SocketCAN Channel[%d] isn't opened. \r\n\n", m_CAN_C);

		m_CAN_S = m_CAN_F;
	}
	else
	{
		if (DEBUG)
			printf(" : SocketCAN Channel[%d] is opened. \r\n\n", m_CAN_C);

		m_CAN_S = m_CAN_F;

		// Set Interface SocketCAN //
		switch (m_CAN_C)
		{
		default:
		case 0:
			m_CAN_IfN = "can0";
			break;
		case 1:
			m_CAN_IfN = "can1";
			break;
		}

		strcpy(m_CAN_IfR.ifr_name, m_CAN_IfN);
		ioctl(m_CAN_F, SIOCGIFINDEX, &m_CAN_IfR);

		m_CAN_SA.can_family = AF_CAN;
		m_CAN_SA.can_ifindex = m_CAN_IfR.ifr_ifindex;

		// Bind SocketCAN //
		m_CAN_S = bind(m_CAN_F, (struct sockaddr *)&m_CAN_SA, sizeof(m_CAN_SA));

		if (m_CAN_S == -1)
		{
			if (DEBUG)
				printf(" : SocketCAN Channel[%d] isn't bound. \r\n\n", m_CAN_C);
		}
		else
		{
			if (DEBUG)
				printf(" : SocketCAN Channel[%d] is bound. \r\n\n", m_CAN_C);
		}
	}
	return m_CAN_S;
}

struct can_frame CCAN_If::Read_CAN()
{
	struct timeval timeout;
	timeout.tv_sec = 3;
	timeout.tv_usec = 0;

	fd_set m_CAN_rF;

	FD_ZERO(&m_CAN_rF);
	FD_SET(m_CAN_F, &m_CAN_rF);
	select(m_CAN_F + 1, &m_CAN_rF, NULL, NULL, &timeout);

	// Initialize CAN Frame
	ssize_t Len = 0;
	struct can_frame Frame;

	memset(&Frame, 0, sizeof(Frame));

	// Read CAN
	if (FD_ISSET(m_CAN_F, &m_CAN_rF))
	{
		Len = read(m_CAN_F, &Frame, sizeof(Frame));

		if (Len < 0)
		{
			if (DEBUG)
				printf(" : SocketCAN Channel[%d] read error.\r\n\n", m_CAN_C);
		}
		else
		{
			if (DEBUG)
				printf(" : GET ID:%4x | LEN:%1x | DATA %02x %02x %02x %02x %02x %02x %02x %02x | \r\n\n",
					   (int)Frame.can_id, (int)Frame.can_dlc, (int)Frame.data[0], (int)Frame.data[1], (int)Frame.data[2], (int)Frame.data[3],
					   (int)Frame.data[4], (int)Frame.data[5], (int)Frame.data[6], (int)Frame.data[7]);
		}
	}
	else
	{
		printf(" : SocketCAN [%d] Read Time out Error! \r\n\n");
	}
	return Frame;
}

int CCAN_If::Write_CAN(can_frame Frame)
{
	ssize_t Len = 0;

	if (write(m_CAN_F, &Frame, sizeof(Frame)) != sizeof(Frame))
	{
		if (DEBUG)
			printf(" : SocketCAN Channel[%d] write error.\r\n\n", m_CAN_C);
		return -1;
	}
	else
	{
		if (DEBUG)
			printf(" : PUT ID %4x | LEN %1x | DATA %02x %02x %02x %02x %02x %02x %02x %02x | \r\n\n",
				   (int)Frame.can_id, (int)Frame.can_dlc, (int)Frame.data[0], (int)Frame.data[1], (int)Frame.data[2], (int)Frame.data[3],
				   (int)Frame.data[4], (int)Frame.data[5], (int)Frame.data[6], (int)Frame.data[7]);
		return 0;
	}
}