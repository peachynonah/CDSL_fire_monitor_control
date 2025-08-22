#pragma once

#include "CAN_If.h"
#include "CAN_Dev.h"

#include <time.h>
#include <sys/sysinfo.h>

#define MAX_PDO_NUM		4
#define MAX_SET_PDO_NUM 1

#define TxPDO 0
#define RxPDO 1

#define POSITION_MODE 0
#define TORQUE_MODE 1

class CCAN_Manager
{
public:
	CCAN_Manager();
	virtual ~CCAN_Manager();

public:
	CCAN_If m_If[MAX_CAN_CH];
	CCAN_Dev m_Dev[MAX_DEV_NUM];

public:
	int m_UsedCh[MAX_CAN_CH]; // Used Channel
	int m_NoUsedCh; // Number of Used Channel
	void Set_IftoDev();

protected:
	void Update_CAN_Channel(int NewCh);

public:
	void Initialize(int ServoCmdType);
	void Finalize();

	void Rqst_PDO_Data();
	void Recv_PDO_Data();
	void Set_PDO_Config(UINT8 DevIdx, UINT8 TRX, UINT8 PDO_N, UINT8 Trans_Type, UINT8 Mode);
	void Send_PDO_Data(UINT8 DevIdx);
};