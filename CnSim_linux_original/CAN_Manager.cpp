#define DEBUG false

#include "CAN_Manager.h"

CCAN_Manager::CCAN_Manager()
{
	// Set CAN Channel to Interface
	for (int i = 0; i < MAX_CAN_CH; i++)
	{
		m_If[i].Set_CAN_If_Ch(i);
	}
	// Set Dev ID to Dev
	for (int i = 0; i < MAX_DEV_NUM; i++)
	{
		m_Dev[i].m_Dev_ID = i + 1;
	}
}

CCAN_Manager::~CCAN_Manager()
{
}

void CCAN_Manager::Set_IftoDev()
{
	m_NoUsedCh = 0;

	m_Dev[0].Connect_IftoDev(&m_If[0]);
	Update_CAN_Channel(0);
	m_Dev[1].Connect_IftoDev(&m_If[0]);
	Update_CAN_Channel(0);
}

void CCAN_Manager::Update_CAN_Channel(int NewCh)
{
	// Check whether NewCh is registered
	for (int i = 0; i < m_NoUsedCh; i++)
	{
		if (m_UsedCh[i] == NewCh)
		{
			return;
		}
	}
	m_UsedCh[m_NoUsedCh++] = NewCh;
}

void CCAN_Manager::Initialize(int ServoCmdType)
{
	UINT32 CtrlMode = ServoCmdType;

	if (DEBUG)
		printf(" : Initialize CAN Device \r\n\n");

	// Set Interface to Device
	Set_IftoDev();

	for (int i = 0; i < m_NoUsedCh; i++)
	{
		m_If[i].Initialize();
	}

	for (int i = 0; i < MAX_DEV_NUM; i++)
	{
		if (DEBUG)
			printf("=========================================================== \r\n\n");

		// Step 0. Device Initialize
		m_Dev[i].Initialize();

		// Step 1. Clear Fault
		while (m_Dev[i].m_CtrlWord != 0x0080)
		{
			m_Dev[i].m_CtrlWord = m_Dev[i].ClearFault();
			usleep(1000);
		}

		if (DEBUG)
			printf(" : Step 1. Clear Fault[%x] \r\n\n", m_Dev[i].m_CtrlWord);

		// Step 2. Set NMT Pre-Operational Mode
		while (m_Dev[i].m_StatWord != STAT_SWT_ON_DIS)
		{
			m_Dev[i].Set_NMT_Mode(NMT_CS_PRE_OP);
			m_Dev[i].m_StatWord = m_Dev[i].ReadObject(STAT_I, STAT_S).Value;
		}

		if (DEBUG)
			printf(" : Step 2. Set NMT Pre-Operational Mode[%x] \r\n\n", m_Dev[i].m_StatWord);

		// Step 3. Get Device Control Mode
		m_Dev[i].m_MoOpDis = m_Dev[i].ReadObject(MODIS_I, MODIS_S).Value;

		switch (m_Dev[i].m_MoOpDis)
		{
		case PPM:
			if (DEBUG)
				printf(" : Step 3. Current Control Mode -> PPM \r\n\n");
			break;
		case PVM:
			if (DEBUG)
				printf(" : Step 3. Current Control Mode -> PVM \r\n\n");
			break;
		case HMM:
			if (DEBUG)
				printf(" : Step 3. Current Control Mode -> HMM \r\n\n");
			break;
		case CSP:
			if (DEBUG)
				printf(" : Step 3. Current Control Mode -> CSP \r\n\n");
			break;
		case CSV:
			if (DEBUG)
				printf(" : Step 3. Current Control Mode -> CST \r\n\n");
			break;
		case CST:
			if (DEBUG)
				printf(" : Step 3. Current Control Mode -> CST \r\n\n");
			break;
		}

		switch (ServoCmdType)
		{
		case POSITION_MODE:
			if (DEBUG)
				printf(" : Step 3-1. User Control Mode -> CSP \r\n\n");
			CtrlMode = CSP;
			break;

		case TORQUE_MODE:
			if (DEBUG)
				printf(" : Step 3-1. User Control Mode -> CST \r\n\n");
			CtrlMode = CST;
			break;
		}

		if (DEBUG)
			printf(" : Step 3-2. Set Control Mode \r\n\n");

		m_Dev[i].WriteObject(MO_I, MO_S, CtrlMode);
		m_Dev[i].m_MoOp = m_Dev[i].ReadObject(MO_I, MO_S).Value;

		// Step 3. Define PDO Offset
		for (int j = 0; j < MAX_PDO_NUM; j++)
		{
			m_Dev[i].m_TxPDO_Offset[j] = 0x180 + (j * 0x100); // m_TxPDO_Offset = {0x180,0x280,0x380,0x480}
			m_Dev[i].m_RxPDO_Offset[j] = 0x200 + (j * 0x100); // m_RxPDO_Offset = {0x200,0x300,0x400,0x500}
		}

		if (DEBUG)
			printf(" : Step 4. Define PDO Offset for PDO COB-ID [Tx : %x %x %x %x Rx: %x %x %x %x] \r\n\n",
				   m_Dev[i].m_TxPDO_Offset[0], m_Dev[i].m_TxPDO_Offset[1], m_Dev[i].m_TxPDO_Offset[2], m_Dev[i].m_TxPDO_Offset[3],
				   m_Dev[i].m_RxPDO_Offset[0], m_Dev[i].m_RxPDO_Offset[1], m_Dev[i].m_RxPDO_Offset[2], m_Dev[i].m_RxPDO_Offset[3]);

		// Step 5. Tx & Rx PDO Configuration
		for (int j = 1; j < (MAX_SET_PDO_NUM + 1); j++)
		{
			Set_PDO_Config(i, TxPDO, j, PDO_TT_ASYNC, CtrlMode); // Tx PDO
			Set_PDO_Config(i, RxPDO, j, PDO_TT_SYNC, CtrlMode);	 // Rx PDO
		}

		if (DEBUG)
			printf(" : Step 11. Tx & Rx PDO Configuration Done \r\n\n");

		// Step 5. Switch ON
		if (m_Dev[i].m_StatWord == STAT_SWT_ON_DIS) // Switch ON Disable
		{
			if (DEBUG)
				printf(" : Step 12. Switch ON Disable[%x] \r\n\n", m_Dev[i].m_StatWord);

			m_Dev[i].WriteObject(CTRL_I, CTRL_S, (CTRL_QUICK_STOP | CTRL_ENA_VOLT));
			m_Dev[i].m_CtrlWord = m_Dev[i].ReadObject(CTRL_I, CTRL_S).Value;

			m_Dev[i].m_StatWord = m_Dev[i].ReadObject(STAT_I, STAT_S).Value;

			if ((m_Dev[i].m_StatWord == (STAT_QUICK_STOP | STAT_RDY_TO_SWT_ON))) // Ready to Switch ON
			{
				if (DEBUG)
					printf(" : Step 13. Ready to Switch ON[%x] \r\n\n", m_Dev[i].m_StatWord);

				m_Dev[i].WriteObject(CTRL_I, CTRL_S, (CTRL_QUICK_STOP | CTRL_ENA_VOLT | CTRL_SWT_ON));
				m_Dev[i].m_CtrlWord = m_Dev[i].ReadObject(CTRL_I, CTRL_S).Value;

				while (m_Dev[i].m_StatWord != (STAT_QUICK_STOP | STAT_SWT_ON | STAT_RDY_TO_SWT_ON))
				{
					m_Dev[i].m_StatWord = m_Dev[i].ReadObject(STAT_I, STAT_S).Value;
				}

				if (DEBUG)
					printf(" : Step 14. Switch ON[%x] \r\n\n", m_Dev[i].m_StatWord);

				while (m_Dev[i].m_StatWord != (STAT_REMOTE | STAT_QUICK_STOP | STAT_SWT_ON | STAT_RDY_TO_SWT_ON))
				{
					m_Dev[i].Set_NMT_Mode(NMT_CS_OP);
					m_Dev[i].m_StatWord = m_Dev[i].ReadObject(STAT_I, STAT_S).Value;
				}

				if (DEBUG)
					printf(" : Step 15. Set NMT Operational Mode[%x] \r\n\n", m_Dev[i].m_StatWord);

				if (DEBUG)
					printf("=========================================================== \r\n\n");
			}
		}
	}
	if (DEBUG)
		printf(" : End Initilalze \r\n\n");

	printf(" : CAN manager is initilalzed \r\n\n");
}

void CCAN_Manager::Finalize()
{
	for (int i = 0; i < MAX_CAN_CH; i++)
	{
		m_If[i].Finalize();
	}
}
void CCAN_Manager::Rqst_PDO_Data()
{
	// Initialize CAN Frame
	can_frame cFrame;
	memset(&cFrame, 0, sizeof(cFrame));

	// Only send COB_ID_SYNC(0x80)
	cFrame.can_id = COB_ID_SYNC;
	cFrame.can_dlc = 8;
	cFrame.data[0] = LD_CCS_W | LD_TT;

	for (int i = 0; i < m_NoUsedCh; i++)
	{
		m_If[i].Write_CAN(cFrame);
	}
}

void CCAN_Manager::Send_PDO_Data(UINT8 DevIdx)
{
	can_frame cFrame;
	memset(&cFrame, 0, sizeof(cFrame));
	TxPDO_Frame sFrame;
	memset(&sFrame, 0, sizeof(sFrame));

	sFrame.COB_ID = m_Dev[DevIdx].m_RxPDO_Offset[0] + m_Dev[DevIdx].m_Dev_ID;
	sFrame.ControlWord = m_Dev[DevIdx].m_CtrlWord;
	sFrame.MoOp = m_Dev[DevIdx].m_MoOp;

	switch (m_Dev[DevIdx].m_MoOp)
	{
	case CSP:
		sFrame.TargetValue = m_Dev[DevIdx].m_TargetPos;
		break;
	case CSV:
		sFrame.TargetValue = m_Dev[DevIdx].m_TargetVel;
		break;
	case CST:
		sFrame.TargetValue = m_Dev[DevIdx].m_TargetTrq;
		break;
	}

	cFrame.can_id = sFrame.COB_ID;
	cFrame.can_dlc = 8;
	cFrame.data[0] = (sFrame.ControlWord & 0x00FF);
	cFrame.data[1] = (sFrame.ControlWord & 0xFF00) >> 8;
	cFrame.data[2] = sFrame.MoOp;
	cFrame.data[3] = (sFrame.TargetValue & 0x000000FF);
	cFrame.data[4] = (sFrame.TargetValue & 0x0000FF00) >> 8;
	cFrame.data[5] = (sFrame.TargetValue & 0x00FF0000) >> 16;
	cFrame.data[6] = (sFrame.TargetValue & 0xFF000000) >> 24;

	for (int i = 0; i < m_NoUsedCh; i++)
	{
		m_If[i].Write_CAN(cFrame);
	}
	// memset(&cFrame, 0, sizeof(cFrame));
	// memset(&sFrame, 0, sizeof(sFrame));

	// sFrame.COB_ID = m_Dev[DevIdx].m_RxPDO_Offset[1] + m_Dev[DevIdx].m_Dev_ID;
	// sFrame.DigOutCmd = m_Dev[DevIdx].m_DigOutCmd;

	// cFrame.can_id = sFrame.COB_ID;
	// cFrame.can_dlc = 4;
	// cFrame.data[0] = (sFrame.DigOutCmd & 0x000000FF);
	// cFrame.data[1] = (sFrame.DigOutCmd & 0x0000FF00) >> 8;
	// cFrame.data[2] = (sFrame.DigOutCmd & 0x00FF0000) >> 16;
	// cFrame.data[3] = (sFrame.DigOutCmd & 0xFF000000) >> 24;

	// m_Dev[DevIdx].Get_CAN_If()->Put_CAN(cFrame);
}

void CCAN_Manager::Recv_PDO_Data()
{
	Rqst_PDO_Data();

	for (int i = 0; i < MAX_SET_PDO_NUM; i++)
	{
		for (int j = 0; j < MAX_DEV_NUM; j++)
		{
			can_frame cFrame;
			memset(&cFrame, 0, sizeof(cFrame));
			RxPDO_Frame rFrame;
			memset(&rFrame, 0, sizeof(rFrame));

			for (int i = 0; i < m_NoUsedCh; i++)
			{
				cFrame = m_If[i].Read_CAN();
			}

			rFrame.COB_ID = cFrame.can_id;
			rFrame.StatusWord = cFrame.data[1] << 8 | cFrame.data[0];
			rFrame.MoOpDis = cFrame.data[2];
			rFrame.ActualValue = cFrame.data[6] << 24 | cFrame.data[5] << 16 | cFrame.data[4] << 8 | cFrame.data[3];

			for (int k = 0; k < MAX_DEV_NUM; k++)
			{
				if (rFrame.COB_ID == m_Dev[k].m_TxPDO_Offset[0] + m_Dev[k].m_Dev_ID)
				{
					m_Dev[k].m_StatWord = rFrame.StatusWord;
					m_Dev[k].m_MoOpDis = rFrame.MoOpDis;

					switch (m_Dev[k].m_MoOpDis)
					{
					case PPM:
						if (m_Dev[k].m_StatWord & STAT_TARGET_REACHED) // Target Reached
						{
							if (DEBUG)
								printf(" : [PPM] Target Reached. \r\n\n");
						}
						if (m_Dev[k].m_StatWord & STAT_INT_LIM_ACT) // Internal limit active
						{
							if (DEBUG)
								printf(" : [PPM] Internal Limit Active. \r\n\n");
						}
						if (m_Dev[k].m_StatWord & STAT_SET_POINT_ACK) // Set Point Acknowledge
						{
							if (DEBUG)
								printf(" : [PPM] Set Point Acknowledge. \r\n\n");
						}
						if (m_Dev[k].m_StatWord & STAT_FOLLOW_ERR) // Following Error
						{
							if (DEBUG)
								printf(" : [PPM] Following Error. \r\n\n");
						}
						break;

					case PVM:
						if (m_Dev[k].m_StatWord & STAT_TARGET_REACHED) // Target Reached
						{
							if (DEBUG)
								printf(" : [PVM] Internal Limit Active. \r\n\n");
						}

						if (m_Dev[k].m_StatWord & STAT_SPD_LIM) // Speed is limited
						{
							if (DEBUG)
								printf(" : [PVM] Internal Limit Active. \r\n\n");
						}
						if (m_Dev[k].m_StatWord & STAT_SPD) // Speed
							\
							{
								if (DEBUG)
									printf(" : [PVM] Internal Limit Active. \r\n\n");
							}
						if (m_Dev[k].m_StatWord & STAT_NOT_USED) // Not Used
						{
							if (DEBUG)
								printf(" : [PVM] Internal Limit Active. \r\n\n");
						}
						break;

					case HMM:
						if (m_Dev[k].m_StatWord & STAT_TARGET_REACHED) // Target Reached
						{
							if (DEBUG)
								printf(" : [HMM] Internal Limit Active. \r\n\n");
						}

						if (m_Dev[k].m_StatWord & STAT_INT_LIM_ACT) // Internal limit active
						{
							if (DEBUG)
								printf(" : [HMM] Internal Limit Active. \r\n\n");
						}
						if (m_Dev[k].m_StatWord & STAT_HOMING_ATTAINED) // Homming attained
						{
							if (DEBUG)
								printf(" : [HMM] Internal Limit Active. \r\n\n");
						}

						if (m_Dev[k].m_StatWord & STAT_HOMMING_ERR) // Homming error
						{
							if (DEBUG)
								printf(" : [HMM] Internal Limit Active. \r\n\n");
						}
						break;

					case CSP:
						if (m_Dev[k].m_StatWord & STAT_RESERVED_2) // Reserved
						{
							if (DEBUG)
								printf(" : [CSP] Reserved. \r\n\n");
						}

						if (m_Dev[k].m_StatWord & STAT_INT_LIM_ACT) // Internal limit active
						{
							if (DEBUG)
								printf(" : [CSP] Internal Limit Active. \r\n\n");
						}

						if (m_Dev[k].m_StatWord & STAT_FOLLOW_CMD_VAL) // Drive follows command value
						{
							if (DEBUG)
								printf(" : [CSP] Drive follows command value. \r\n\n");
						}
						if (m_Dev[k].m_StatWord & STAT_FOLLOW_ERR) // Following Error
						{
							if (!DEBUG)
								printf(" : [CSP] Following Error. \r\n\n");
						}
						m_Dev[k].m_ActPos = rFrame.ActualValue;
						if (DEBUG)
							printf(" : [%d]POS : %d \r\n\n", m_Dev[0].m_Dev_ID, m_Dev[k].m_ActPos);
						break;

					case CSV:
						if (m_Dev[k].m_StatWord & STAT_RESERVED_2) // Reserved
						{
							if (DEBUG)
								printf(" : [CSV] Reserved. \r\n\n");
						}
						if (m_Dev[k].m_StatWord & STAT_INT_LIM_ACT) // Internal limit active
						{
							if (DEBUG)
								printf(" : [CSV] Internal Limit Active. \r\n\n");
						}
						if (m_Dev[k].m_StatWord & STAT_FOLLOW_CMD_VAL) // Drive follows command value
						{
							if (DEBUG)
								printf(" : [CSV] Drive follows command value. \r\n\n");
						}

						if (m_Dev[k].m_StatWord & STAT_NOT_USED) // Not used
						{
							if (DEBUG)
								printf(" : [CSV] Not used. \r\n\n");
						}
						m_Dev[k].m_ActVel = rFrame.ActualValue;
						if (DEBUG)
							printf(" : [%d]RPM : %d \r\n\n", m_Dev[k].m_Dev_ID, m_Dev[k].m_ActVel);
						break;

					case CST:
						if (m_Dev[k].m_StatWord & STAT_RESERVED_2) // Reserved
						{
							if (DEBUG)
							printf(" : [CST] Reserved. \r\n\n\n");
						}
						if (m_Dev[k].m_StatWord & STAT_INT_LIM_ACT) // Internal limit active
						{
							if (DEBUG)
							printf(" : [CST] Internal Limit Active. \r\n\n");
						}
						if (m_Dev[k].m_StatWord & STAT_FOLLOW_CMD_VAL) // Drive follows command value
						{
							if (DEBUG)
							printf(" : [CST] Drive follows command value. \r\n\n");
						}
						if (m_Dev[k].m_StatWord & STAT_NOT_USED)
						{
							if (DEBUG)
							printf(" : [CST] Not used. \r\n\n");
						}
						//m_Dev[k].m_ActTrq = rFrame.ActualValue;
						m_Dev[k].m_ActPos = rFrame.ActualValue;
						if (DEBUG)
							printf(" : [%d]POSITION -> %d \r\n\n", m_Dev[k].m_Dev_ID, m_Dev[k].m_ActPos);
							//printf(" : [%d]TORQUE -> %d \r\n\n", m_Dev[k].m_Dev_ID, m_Dev[k].m_ActTrq);
						break;
					}

					if (m_Dev[k].m_StatWord & STAT_RDY_TO_SWT_ON) // Ready to switch ON
					{
						if (m_Dev[k].m_StatWord & STAT_SWT_ON) // Switched ON
						{
							if (m_Dev[k].m_StatWord & STAT_OP_ENA) // Operation enabled
							{
								if (DEBUG)
									printf(" : [%d] Operating enabled. \r\n\n", j);
							}
							else
							{
								if (DEBUG)
									printf(" : [%d] Switched ON. \r\n\n", j);
							}
						}
						else
						{
							if (DEBUG)
								printf(" : [%d] Ready to switch ON. \r\n\n", j);
						}
					}

					if (m_Dev[k].m_StatWord & STAT_FAULT) // Fault
					{
						if (DEBUG)
							printf(" : [%d] Fault. \r\n\n", j);
					}

					if (m_Dev[k].m_StatWord & STAT_VOLT_ENA) // PowerON
					{
						if (DEBUG)
							printf(" : [%d] Power ON. \r\n\n", j);
					}

					if (m_Dev[k].m_StatWord & STAT_QUICK_STOP) // Quick Stop
					{
						if (DEBUG)
							printf(" : [%d] Quick Stop. \r\n\n", j);
					}

					if (m_Dev[k].m_StatWord & STAT_SWT_ON_DIS) // Switch ON disabled
					{
						if (DEBUG)
							printf(" : [%d] Switched ON disabled. \r\n\n", j);
					}

					if (m_Dev[k].m_StatWord & STAT_WARNING) // Warning
					{
						if (DEBUG)
							printf(" : [%d] Warning. \r\n\n", j);
					}

					if (m_Dev[k].m_StatWord & STAT_RESERVED_1)
					{
						if (DEBUG)
							printf(" : [%d] Reserved(1). \r\n\n", j);
					}

					if (m_Dev[k].m_StatWord & STAT_REMOTE) // Remote
					{
						if (DEBUG)
							printf(" : [%d] Remote. \r\n\n", j);
					}

					if (m_Dev[k].m_StatWord & STAT_RESERVED_3)
					{
						if (DEBUG)
							printf(" : [%d] Reserved(3). \r\n\n", j);
					}

					if (m_Dev[k].m_StatWord & STAT_POS_REF_TO_HOME_POS) // Position referenced to home position
					{
						if (DEBUG)
							printf(" : [%d] Position referenced to home position. \r\n\n", j);
					}
				}

				if (rFrame.COB_ID == m_Dev[k].m_TxPDO_Offset[1] + m_Dev[k].m_Dev_ID)
				{
					rFrame.DigOutState = cFrame.data[3] << 24 | cFrame.data[2] << 16 | cFrame.data[1] << 8 | cFrame.data[0];
					m_Dev[k].m_DigOutStat = rFrame.DigOutState;
					if (DEBUG)
						printf(" : [%d] Digital Ouput State : %d \r\n\n", j, m_Dev[k].m_DigOutStat);
				}

				if (rFrame.COB_ID == m_Dev[k].m_TxPDO_Offset[2] + m_Dev[k].m_Dev_ID)
				{
				}

				if (rFrame.COB_ID == m_Dev[k].m_TxPDO_Offset[3] + m_Dev[k].m_Dev_ID)
				{
				}
			}
		}
	}
}

void CCAN_Manager::Set_PDO_Config(UINT8 DevIdx, UINT8 TRX, UINT8 PDO_N, UINT8 Trans_Type, UINT8 Mode)
{
	SDO_Frame Config_Frame;
	memset(&Config_Frame, 0, sizeof(Config_Frame));

	// Step 4. Configure COB-ID
	Config_Frame.COB_ID = SDO_W_ID + m_Dev[DevIdx].m_Dev_ID;
	Config_Frame.SubIndex = PDO_COB_ID_S;

	// Step 4-2. Set COB_ID
	if (TRX)
	{
		switch (PDO_N)
		{
		default:
		case 1:
			Config_Frame.Index = TxPDO_I;
			Config_Frame.Value = m_Dev[DevIdx].m_TxPDO_Offset[0] + m_Dev[DevIdx].m_Dev_ID;
			break;
		case 2:
			Config_Frame.Index = TxPDO_I + 1;
			Config_Frame.Value = m_Dev[DevIdx].m_TxPDO_Offset[1] + m_Dev[DevIdx].m_Dev_ID;
			break;
		case 3:
			Config_Frame.Index = TxPDO_I + 2;
			Config_Frame.Value = m_Dev[DevIdx].m_TxPDO_Offset[2] + m_Dev[DevIdx].m_Dev_ID;
			break;
		case 4:
			Config_Frame.Index = TxPDO_I + 3;
			Config_Frame.Value = m_Dev[DevIdx].m_TxPDO_Offset[3] + m_Dev[DevIdx].m_Dev_ID;
			break;
		}
	}
	else
	{
		switch (PDO_N)
		{
		default:
		case 1:
			Config_Frame.Index = RxPDO_I;
			Config_Frame.Value = m_Dev[DevIdx].m_RxPDO_Offset[0] + m_Dev[DevIdx].m_Dev_ID;
			break;
		case 2:
			Config_Frame.Index = RxPDO_I + 1;
			Config_Frame.Value = m_Dev[DevIdx].m_RxPDO_Offset[1] + m_Dev[DevIdx].m_Dev_ID;
			break;
		case 3:
			Config_Frame.Index = RxPDO_I + 2;
			Config_Frame.Value = m_Dev[DevIdx].m_RxPDO_Offset[2] + m_Dev[DevIdx].m_Dev_ID;
			break;
		case 4:
			Config_Frame.Index = RxPDO_I + 3;
			Config_Frame.Value = m_Dev[DevIdx].m_RxPDO_Offset[3] + m_Dev[DevIdx].m_Dev_ID;
			break;
		}
	}

	// Step 4-3. Write Object
	if (DEBUG)
		printf(" : Step 5. Configure COB-ID -> %x \r\n\n", Config_Frame.Value);

	m_Dev[DevIdx].WriteObject(Config_Frame.Index, Config_Frame.SubIndex, Config_Frame.Value);

	// Step 5. Set Transmission Type
	// Step 5-1. Select Transmission Type
	Config_Frame.SubIndex = PDO_TT_S;

	switch (Trans_Type)
	{
	case PDO_TT_SYNC:
		Config_Frame.Value = PDO_TT_SYNC;
		if (DEBUG)
			printf(" : Step 6. Set Transmission Type -> SYNC \r\n\n");
		break;
	case PDO_TT_RTR:
		Config_Frame.Value = PDO_TT_RTR;
		if (DEBUG)
			printf(" : Step 6. Set Transmission Type -> RTR \r\n\n");
		break;
	default:
	case PDO_TT_ASYNC:
		Config_Frame.Value = PDO_TT_ASYNC;
		if (DEBUG)
			printf(" : Step 6. Set Transmission Type -> ASYNC \r\n\n");
		break;
	}

	// Step 5-2. Write Object
	m_Dev[DevIdx].WriteObject(Config_Frame.Index, Config_Frame.SubIndex, Config_Frame.Value);

	// Step 6. Disable Number of Mapped Application Objects
	Config_Frame.Index = Config_Frame.Index + 0x200;
	Config_Frame.SubIndex = PDO_S_S;
	Config_Frame.Value = 0x00;

	if (DEBUG)
		printf(" : Step 7. Disable Number of Mapped Application Objects[%d]. \r\n\n", Config_Frame.Value);

	m_Dev[DevIdx].WriteObject(Config_Frame.Index, Config_Frame.SubIndex, Config_Frame.Value);

	// Step 4. Mapping Objects
	UINT8 NMO = 1; // Number of Mapping Object

	if (TRX)
	{
		switch (PDO_N)
		{
		default:
		case 1:
			// TxPDO_1 Mapping Object
			Config_Frame.SubIndex = NMO;
			Config_Frame.Value = 0x60410010; // Statusword, 16bit(2Byte)

			if (DEBUG)
				printf(" : Step 8. Mapping Object[%d] -> %x \r\n\n", Config_Frame.SubIndex, Config_Frame.Value);

			m_Dev[DevIdx].WriteObject(Config_Frame.Index, Config_Frame.SubIndex, Config_Frame.Value);
			NMO++;

			Config_Frame.SubIndex = NMO;
			Config_Frame.Value = 0x60610008; // Modes of Operation Display, 8bit(1Byte)

			if (DEBUG)
				printf(" : Step 8. Mapping Object[%d] -> %x \r\n\n", Config_Frame.SubIndex, Config_Frame.Value);

			m_Dev[DevIdx].WriteObject(Config_Frame.Index, Config_Frame.SubIndex, Config_Frame.Value);
			NMO++;

			Config_Frame.SubIndex = NMO;

			switch (Mode)
			{
			case CSP:
				Config_Frame.Value = 0x60640020; // Position Actual Value, 32bit(4Byte)

				if (DEBUG)
					printf(" : Step 8. Mapping Object[%d] -> %x \r\n\n", Config_Frame.SubIndex, Config_Frame.Value);

				m_Dev[DevIdx].WriteObject(Config_Frame.Index, Config_Frame.SubIndex, Config_Frame.Value);
				break;
			case CSV:
				Config_Frame.Value = 0x606C0020; // Velocity Actual Value, 32bit(4Byte)

				if (DEBUG)
					printf(" : Step 8. Mapping Object[%d] -> %x \r\n\n", Config_Frame.SubIndex, Config_Frame.Value);

				m_Dev[DevIdx].WriteObject(Config_Frame.Index, Config_Frame.SubIndex, Config_Frame.Value);
				break;
			case CST:
				// Config_Frame.Value = 0x60770010; // Torque Actual Value, 16bit(2Byte)
				Config_Frame.Value = 0x60640020; // Position Actual Value, 32bit(4Byte)

				if (DEBUG)
					printf(" : Step 8. Mapping Object[%d] -> %x \r\n\n", Config_Frame.SubIndex, Config_Frame.Value);

				m_Dev[DevIdx].WriteObject(Config_Frame.Index, Config_Frame.SubIndex, Config_Frame.Value);
				break;
			}
			break;
		case 2:
			Config_Frame.SubIndex = NMO;
			Config_Frame.Value = 0x60FD0020; // Digital Inputs, 32bit(4Byte)

			if (DEBUG)
				printf(" : Step 8. Mapping Object[%d] -> %x \r\n\n", Config_Frame.SubIndex, Config_Frame.Value);

			m_Dev[DevIdx].WriteObject(Config_Frame.Index, Config_Frame.SubIndex, Config_Frame.Value);
			break;
		case 3:
			Config_Frame.SubIndex = NMO;
			Config_Frame.Value = 0x60410010; // Statusword, 16bit(2Byte)

			if (DEBUG)
				printf(" : Step 8. Mapping Object[%d] -> %x \r\n\n", Config_Frame.SubIndex, Config_Frame.Value);

			m_Dev[DevIdx].WriteObject(Config_Frame.Index, Config_Frame.SubIndex, Config_Frame.Value);
			break;
		case 4:
			Config_Frame.SubIndex = NMO;
			Config_Frame.Value = 0x60410010; // Statusword, 16bit(2Byte)

			if (DEBUG)
				printf(" : Step 8. Mapping Object[%d] -> %x \r\n\n", Config_Frame.SubIndex, Config_Frame.Value);

			m_Dev[DevIdx].WriteObject(Config_Frame.Index, Config_Frame.SubIndex, Config_Frame.Value);
			break;
		}
	}
	else
	{
		switch (PDO_N)
		{
		default:
		case 1:
			Config_Frame.SubIndex = NMO;
			Config_Frame.Value = 0x60400010; // Control World, 16bit(2Byte)

			if (DEBUG)
				printf(" : Step 8. Mapping Object[%d] -> %x \r\n\n", Config_Frame.SubIndex, Config_Frame.Value);

			m_Dev[DevIdx].WriteObject(Config_Frame.Index, Config_Frame.SubIndex, Config_Frame.Value);
			NMO++;

			Config_Frame.SubIndex = NMO;
			Config_Frame.Value = 0x60600008; // Mode of Operation, 8bit(1Byte)

			if (DEBUG)
				printf(" : Step 8. Mapping Object[%d] -> %x \r\n\n", Config_Frame.SubIndex, Config_Frame.Value);

			m_Dev[DevIdx].WriteObject(Config_Frame.Index, Config_Frame.SubIndex, Config_Frame.Value);
			NMO++;

			Config_Frame.SubIndex = NMO;
			switch (Mode)
			{
			case CSP:
				Config_Frame.Value = 0x607A0020; // Target Position, 32bit(4Byte)

				if (DEBUG)
					printf(" : Step 8. Mapping Object[%d] -> %x \r\n\n", Config_Frame.SubIndex, Config_Frame.Value);

				m_Dev[DevIdx].WriteObject(Config_Frame.Index, Config_Frame.SubIndex, Config_Frame.Value);
				break;
			case CSV:
				Config_Frame.Value = 0x60FF0020; // Target Velocity, 32bit(4Byte)

				if (DEBUG)
					printf(" : Step 8. Mapping Object[%d] -> %x \r\n\n", Config_Frame.SubIndex, Config_Frame.Value);

				m_Dev[DevIdx].WriteObject(Config_Frame.Index, Config_Frame.SubIndex, Config_Frame.Value);
				break;
			case CST:
				Config_Frame.Value = 0x60710010; // Target Torque, 16bit(2Byte)

				if (DEBUG)
					printf(" : Step 8. Mapping Object[%d] -> %x \r\n\n", Config_Frame.SubIndex, Config_Frame.Value);

				m_Dev[DevIdx].WriteObject(Config_Frame.Index, Config_Frame.SubIndex, Config_Frame.Value);
				break;
			}
			break;
		case 2:
			Config_Frame.SubIndex = NMO;
			Config_Frame.Value = 0x60FE0120; // Digital Outputs, 32bit(4Byte)

			if (DEBUG)
				printf(" : Step 8. Mapping Object[%d] -> %x \r\n\n", Config_Frame.SubIndex, Config_Frame.Value);

			m_Dev[DevIdx].WriteObject(Config_Frame.Index, Config_Frame.SubIndex, Config_Frame.Value);
			break;
		case 3:
			Config_Frame.SubIndex = NMO;
			Config_Frame.Value = 0x60400010; // Control World, 16bit(2Byte)

			if (DEBUG)
				printf(" : Step 8. Mapping Object[%d] -> %x \r\n\n", Config_Frame.SubIndex, Config_Frame.Value);

			m_Dev[DevIdx].WriteObject(Config_Frame.Index, Config_Frame.SubIndex, Config_Frame.Value);
			break;
		case 4:
			Config_Frame.SubIndex = NMO;
			Config_Frame.Value = 0x60400010; // Control World, 16bit(2Byte)

			if (DEBUG)
				printf(" : Step 8. Mapping Object[%d] -> %x \r\n\n", Config_Frame.SubIndex, Config_Frame.Value);

			m_Dev[DevIdx].WriteObject(Config_Frame.Index, Config_Frame.SubIndex, Config_Frame.Value);
			break;
		}
	}

	// Step 5: Enable Number of mapped Application Objects
	Config_Frame.SubIndex = PDO_S_S;
	Config_Frame.Value = NMO;

	if (DEBUG)
		printf(" : Step 9. Enable Number of mapped Application Objects -> %x \r\n\n", Config_Frame.Value);

	m_Dev[DevIdx].WriteObject(Config_Frame.Index, Config_Frame.SubIndex, Config_Frame.Value);

	// Step 6: Activate
	if (DEBUG)
		printf(" : Step 10. Changes will directly activated. \r\n\n");
}