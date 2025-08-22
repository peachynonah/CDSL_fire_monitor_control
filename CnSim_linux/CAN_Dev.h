#pragma once
#include "CAN_If.h"
#include "CAN_DevDef.h"

#define MAX_DEV_NUM		2

// PDO Mapping Frame
typedef struct {
	UINT32 COB_ID;
	UINT16 ControlWord;
	UINT8 MoOp;
	INT32 TargetValue;
	UINT32 DigOutCmd;
}TxPDO_Frame;

typedef struct {
	UINT32 COB_ID;
	UINT16 StatusWord;
	UINT8 MoOpDis;
	INT32 ActualValue;
	UINT32 DigOutState;
}RxPDO_Frame;

typedef struct {
	INT32 Raw;
	INT32 Pre_Raw;
	UINT64 CNT;
	INT16 DIR;
	INT16 Offset;
}ENC_Info;

class CCAN_Dev
{
public:
	CCAN_Dev();
	virtual ~CCAN_Dev();

protected:
	CCAN_If* m_pCAN_If;
	UINT8   m_CAN_Ch;			// CAN Channel

public:
	void Connect_IftoDev(CCAN_If* pCAN_If);
	//CCAN_If* Get_CAN_If(){return m_pCAN_If;}

public: // Mechanical Charateristic
	double  m_Gear_Ratio;		// Gear Ratio
	ENC_Info m_ENC;
	

public:
	UINT8	m_Dev_ID;			// Device ID
	UINT8	m_NMT_S;			// NMT State
	bool	m_PWR_S;			// Power State
	UINT8	m_RST_S;	    	// Reset State

	// CAN PDO Data
	// PC -> Controller
	UINT16	m_CtrlWord; 		// 0x6040
	INT8	m_MoOp; 			// 0x6060
	INT16	m_TargetTrq; 		// 0x6071
	INT32	m_TargetVel; 		// 0x60FF
	INT32	m_TargetPos; 		// 0x607A
	UINT32	m_DigOutCmd;	 	// 0x60FE
	
	// Controller -> PC
	UINT16	m_StatWord; 		// 0x6041
	INT8	m_MoOpDis; 			// 0x6061
	INT16	m_ActTrq; 			// 0x6077
	INT32	m_ActVel;			// 0x606C	
	INT32	m_ActPos;			// 0x6064
	UINT16  m_DigOutStat;		// 0x3150

public:
	// CAN PDO Offset
	UINT16 m_RxPDO_Offset[4] = {0,};
	UINT16 m_TxPDO_Offset[4] = {0,};

public:
	void WriteObject(UINT16 Index, UINT8 SubIndex, UINT32 Value);
	SDO_Frame ReadObject(UINT16 Index, UINT8 SubIndex);

public:
	void Initialize();
	UINT16 Power(bool State);
	UINT16 ClearFault();

public:
	void Set_NMT_Mode(UINT8 MODE);
	UINT32 Get_NMT_State();
	void Set_HB_Time(UINT8 ms_Time);

public:
	UINT16 ErrCode();
	void AbtMsg(UINT32 AbortCodes);
	void ErrMsg(UINT16 ErrCode);
};