#define DEBUG false

#include "CAN_Dev.h"

CCAN_Dev::CCAN_Dev()
{
}

CCAN_Dev::~CCAN_Dev()
{
}

void CCAN_Dev::Connect_IftoDev(CCAN_If* pCAN_If)
{
	m_pCAN_If = pCAN_If;
	m_CAN_Ch = pCAN_If->Get_CAN_If_Ch();
}

void CCAN_Dev::Initialize()
{

}

void CCAN_Dev::WriteObject(UINT16 Index, UINT8 SubIndex, UINT32 Value)
{
	// Initialize Frame
	can_frame cFrame;
	memset(&cFrame, 0, sizeof(cFrame));
	SDO_Frame rFrame;
	memset(&rFrame, 0, sizeof(rFrame));

	int VLC = 0; // Value Length Code

	// if(Value <= 0xFF)
	// 	VLC = 1;

	// if(Value > 0xFF && Value <= 0xFFFF)
	// 	VLC = 2;

	// if(Value > 0xFFFF && Value <= 0xFFFFFF)
	// 	VLC = 3;

	// if(Value > 0xFFFFFF && Value <= 0xFFFFFFFF)
	// 	VLC = 4;

	VLC = 4; // Force Allocation

	// Send to Write Object
	cFrame.can_id = SDO_W_ID + m_Dev_ID;
	cFrame.can_dlc = 4 + VLC;
	cFrame.data[0] = (LD_CCS_W | LD_TT); // Write Object,
	cFrame.data[1] = (Index & 0x00FF);
	cFrame.data[2] = (Index & 0xFF00) >> 8;
	cFrame.data[3] = SubIndex;

	 for (int i = 0; i < VLC; i++)
	 {
	 	cFrame.data[4 + i] = Value >> (8 * i);
	 }

	m_pCAN_If->Write_CAN(cFrame);

	// Initialize Frame
	memset(&cFrame, 0, sizeof(cFrame));
	cFrame = m_pCAN_If->Read_CAN();

	// Recv to Write Object
	rFrame.COB_ID = cFrame.can_id;
	rFrame.LD = cFrame.data[0];
	rFrame.Index = cFrame.data[2] << 8 | cFrame.data[1];
	rFrame.SubIndex = cFrame.data[3];
	rFrame.Value = cFrame.data[7] << 24 | cFrame.data[6] << 16 | cFrame.data[5] << 8 | cFrame.data[4];

	switch (rFrame.LD)
	{
	case LD_CS:
		if(DEBUG)
			printf(" : Dev[%d] SDO abort transfer. \r\n\n", m_Dev_ID);

		AbtMsg(rFrame.Value);
		break;

	case LD_SCS_W:
		if(DEBUG)
			printf(" : Dev[%d] SDO update data. \r\n\n", m_Dev_ID);
		break;
	}
}

SDO_Frame CCAN_Dev::ReadObject(UINT16 Index, UINT8 SubIndex)
{
	// Initialize Frame
	can_frame cFrame;
	memset(&cFrame, 0, sizeof(cFrame));
	SDO_Frame rFrame;
	memset(&rFrame, 0, sizeof(rFrame));

	// Send to Read Object
	cFrame.can_id = SDO_W_ID + m_Dev_ID;
	cFrame.can_dlc = 8;
	cFrame.data[0] = LD_SCS_R;
	cFrame.data[1] = (Index & 0x00FF);
	cFrame.data[2] = (Index & 0xFF00) >> 8;
	cFrame.data[3] = SubIndex;
	m_pCAN_If->Write_CAN(cFrame);

	// Recv to Read Object
	memset(&cFrame, 0, sizeof(cFrame));
	cFrame = m_pCAN_If->Read_CAN();
	
	// Matching CAN RX Frame with SDO Recv Frame
	rFrame.COB_ID = cFrame.can_id;
	rFrame.LD = cFrame.data[0];
	rFrame.Index = cFrame.data[2] << 8 | cFrame.data[1];
	rFrame.SubIndex = cFrame.data[3];

	switch (rFrame.LD)
	{
	case LD_CS:
		if (DEBUG)
			printf(" : Dev[%d] SDO abort transfer. \r\n\n", m_Dev_ID);
		rFrame.Value = cFrame.data[7] << 24 | cFrame.data[6] << 16 | cFrame.data[5] << 8 | cFrame.data[4];
		AbtMsg(rFrame.Value);
		break;

	case LD_SCS_R | LD_N_1B | LD_TT | LD_SI:
		if (DEBUG)
			printf(" : Dev[%d] SDO read 1 byte. \r\n\n", m_Dev_ID);
		rFrame.Value = cFrame.data[4];
		break;

	case LD_SCS_R | LD_N_2B | LD_TT | LD_SI:
		if (DEBUG)
			printf(" : Dev[%d] SDO read 2 byte. \r\n\n", m_Dev_ID);
		rFrame.Value = cFrame.data[5] << 8 | cFrame.data[4];
		break;

	case LD_SCS_R | LD_TT | LD_SI:
		if (DEBUG)
			printf(" : Dev[%d] SDO read 3 byte. \r\n\n", m_Dev_ID);
		rFrame.Value = cFrame.data[6] << 16 | cFrame.data[5] << 8 | cFrame.data[4];
		break;

	case LD_SCS_R:
		if (DEBUG)
			printf(" : Dev[%d] SDO read 4 byte. \r\n\n", m_Dev_ID);
		rFrame.Value = cFrame.data[7] << 24 | cFrame.data[6] << 16 | cFrame.data[5] << 8 | cFrame.data[4];
		break;
	}
	return rFrame;
}

UINT16 CCAN_Dev::Power(bool State)
{
	UINT16 _CtrlWord = 0;
	
	switch(State)
	{
		case false:
			//_CtrlWord = m_CtrlWord | CTRL_QUICK_STOP | CTRL_ENA_VOLT | CTRL_SWT_ON;
			_CtrlWord = 0x07;
		break;
		
		case true:
			//_CtrlWord = m_CtrlWord | CTRL_ENA_OP | CTRL_QUICK_STOP | CTRL_ENA_VOLT | CTRL_SWT_ON;
			_CtrlWord = 0x0f;
		break;
	}
	return _CtrlWord;
}

UINT16 CCAN_Dev::ClearFault()
{
	UINT16 _CtrlWord = 0;

	WriteObject(CTRL_I, CTRL_S, 0x0000);
	WriteObject(CTRL_I, CTRL_S, 0x0080);
	_CtrlWord = ReadObject(CTRL_I, CTRL_S).Value;

	return _CtrlWord;
}

void CCAN_Dev::Set_NMT_Mode(UINT8 MODE)
{
	can_frame cFrame;
	memset(&cFrame, 0, sizeof(cFrame));

	cFrame.can_id = NMT_COB_ID;
	cFrame.can_dlc = 2;
	cFrame.data[0] = MODE;
	cFrame.data[1] = m_Dev_ID; // 0x00 : All Node, N : N Node

	m_pCAN_If->Write_CAN(cFrame);
}

UINT32 CCAN_Dev::Get_NMT_State()
{
	UINT32 State = 0;

	can_frame cFrame;
	memset(&cFrame, 0, sizeof(cFrame));

	cFrame = m_pCAN_If->Read_CAN();

	if (cFrame.can_id == NMT_HS_ID + m_Dev_ID)
	{
		State = cFrame.data[7] << 24 | cFrame.data[6] << 16 | cFrame.data[5] << 8 | cFrame.data[4];
		
		switch (State)
		{
		case NMT_HS_BOOT_UP:
			m_NMT_S = NMT_HS_BOOT_UP;
			break;
		case NMT_HS_PRE_OP:
			m_NMT_S = NMT_HS_PRE_OP;
			break;
		case NMT_HS_OP:
			m_NMT_S = NMT_HS_OP;
			break;
		case NMT_HS_STOP:
			m_NMT_S = NMT_HS_STOP;
			break;
		}
	}
	return State;
}

void CCAN_Dev::Set_HB_Time(UINT8 ms_Time)
{
	WriteObject(NMT_HB_Time_I, NMT_HB_Time_S, ms_Time);
}

void CCAN_Dev::AbtMsg(UINT32 Value)
{
	switch (Value)
	{
	case 0x05030000:
		printf(" : Toggle error -> Toggle bit not alternated. \r\n\n");
		break;
	case 0x05040000:
		printf(" : SDO timeout -> SDO protocol timed out. \r\n\n");
		break;
	case 0x05040001:
		printf(" : Command unknown -> Command specifier unknown. \r\n\n");
		break;
	case 0x05040004:
		printf(" : CRC error -> CRC check failed. \r\n\n");
		break;
	case 0x06010000:
		printf(" : Access error -> Unsupported access to an object. \r\n\n");
		break;
	case 0x06010001:
		printf(" : Write only error -> Read command to a write only object. \r\n\n");
		break;
	case 0x06010002:
		printf(" : Read only error -> Write command to a read only object. \r\n\n");
		break;
	case 0x06010003:
		printf(" : Subindex can not be written -> Subindex can not be written, subindex 0 must be “0” (zero) for write access. \r\n\n");
		break;
	case 0x06020000:
		printf(" : Object does not exist error -> Last read or write command had wrong object index or subindex. \r\n\n");
		break;
	case 0x06040041:
		printf(" : PDO mapping error -> Object is not mappable to the PDO. \r\n\n");
		break;
	case 0x06040042:
		printf(" : PDO length error -> Number and length of objects to be mapped would exceed PDO length. \r\n\n");
		break;
	case 0x06040043:
		printf(" : General parameter error -> General parameter incompatibility. \r\n\n");
		break;
	case 0x06040047:
		printf(" : General internal incompatibility error -> General internal incompatibility in device. \r\n\n");
		break;
	case 0x06060000:
		printf(" : Hardware error -> Access failed due to hardware error. \r\n\n");
		break;
	case 0x06070010:
		printf(" : Service parameter error -> Data type does not match, length or service parameter do not match. \r\n\n");
		break;
	case 0x06070013:
		printf(" : Service parameter too short error -> Data type does not match, length of service parameter too long. \r\n\n");
		break;
	case 0x06090011:
		printf(" : Subindex error -> Last read or write command had wrong object subindex. \r\n\n");
		break;
	case 0x06090030:
		printf(" : Value range error -> Value range of parameter exceeded. \r\n\n");
		break;
	case 0x08000000:
		printf(" : General error -> General error. \r\n\n");
		break;
	case 0x08000020:
		printf(" : Transfer or store error -> Data cannot be transferred or stored. \r\n\n");
		break;
	case 0x08000022:
		printf(" : Wrong device state error -> Data cannot be transferred or stored to application because of present device state. \r\n\n");
		break;
	case 0x0F00FFBE:
		printf(" : Password error -> Password is incorrect. \r\n\n");
		break;
	case 0x0F00FFBF:
		printf(" : Illegal command error -> Command code is illegal (does not exist). \r\n\n");
		break;
	case 0x0F00FFC0:
		printf(" : Wrong NMT state error -> Device is in wrong NMT state. \r\n\n");
		break;
	}
}
// Error Code(ECODE -> Unsigned Int)

UINT16 CCAN_Dev::ErrCode()
{
	return ReadObject(ErrCode_I, ErrCode_S).Value;
}

// 에러뜨면 일단 Drive Error 
void CCAN_Dev::ErrMsg(UINT16 ErrCode)
{
	switch (ErrCode)
	{
	case 0x0000:
		printf(" : No error \r\n\n");
		break;
	case 0x1000:
		printf(" : Generic error -> Unspecific error occurred \r\n\n");
		break;
	case 0x1080: case 0x1081: case 0x1082: case 0x1083:
		printf(" : Generic initialization error -> Critical error occurred during boot-up \r\n\n");
		break;
	case 0x1090:
		printf(" : Firmware incompatibility error -> Incompatible extension firmware version detected \r\n\n");
		break;
	case 0x2310:
		printf(" : Overcurrent error -> Short circuit in motor winding. Controller gains too high and/or deceleration too high. Damaged power stage. \r\n\n");
		break;
	case 0x2320:
		printf(" : Power state protection error -> Short circuit of motor winding against ground. Short circuit of motor winding against operating voltage Vcc. Damaged power stage. \r\n\n");
		break;
	case 0x3210:
		printf(" : Overvoltage error -> Power supply voltage too high \r\n\n");
		break;
	case 0x3220:
		printf(" : Undervoltage error -> Supply voltage is too low for operation. Power supply cannot supply required acceleration current. \r\n\n");
		break;
	case 0x4210:
		printf(" : Thermal overload error -> Temperature at device’s power stage too high \r\n\n");
		break;
	case 0x4380:
		printf(" : Thermal motor overload error -> Temperature at motor too high or sensor not connected \r\n\n");
		break;
	case 0x5113:
		printf(" : Logic supply voltage too low error -> Logic supply voltage is too low for operation \r\n\n");
		break;
	case 0x5280:
		printf(" : Hardware defect error -> Hardware problem detected \r\n\n");
		break;
	case 0x5281:
		printf(" : Hardware incompatibility error -> An incompatible hardware combination was detected \r\n\n");
		break;
	case 0x5480: case 0x5481: case 0x5482: case 0x5483:
		printf(" : Hardware error -> A hardware problem was detected \r\n\n");
		break;
	case 0x6080:
		printf(" : Sign of life error -> Problem with connection to extension 1 : Overload situation or Extension hardware failure \r\n\n");
		break;
	case 0x6081:
		printf(" : Extension 1 watchdog error -> Connection loss to extension 1 : Overload situation or Extension hardware failure \r\n\n");
		break;
	case 0x6180: case 0x6181: case 0x6182: case 0x6183:
		printf(" : Internal software error -> An internal software error occurred \r\n\n");
		break;
	case 0x6320:
		printf(" : Software parameter error -> Corrupt parameter detected \r\n\n");
		break;
	case 0x6380:
		printf(" : Persistent parameter corrupt error -> Persistent parameters are corrupt or inconsistent (wrong CRC) \r\n\n");
		break;
	case 0x7320:
		printf(" : Position sensor error -> Detected position of position sensor is no longer valid \r\n\n");
		break;
	case 0x7380:
		printf(" : Position sensor breach error -> Position sensor supervision has detected a bad working condition \r\n\n");
		break;
	case 0x7381:
		printf(" : Position sensor resolution error -> Encoder pulses counted between the first two index pulses do not fit the resolution. Setting of encoder resolution is wrong. \r\n\n");
		break;
	case 0x7382:
		printf(" : Position sensor index error -> Encoder index signal was not found within two turns at start-up \r\n\n");
		break;
	case 0x7388:
		printf(" : Hall sensor error -> Motor Hall sensors report an impossible signal combination \r\n\n");
		break;
	case 0x7389:
		printf(" : Hall sensor not found error -> No Hall sensor 3 edge found within first motor turn \r\n\n");
		break;
	case 0x738A:
		printf(" : Hall angle detection error -> Angle difference measured between encoder and Hall sensors is too high \r\n\n");
		break;
	case 0x738C:
		printf(" : SSI sensor error -> SSI sensor driver could not sample position data \r\n\n");
		break;
	case 0x738D:
		printf(" : SSI sensor frame error \r\n\n");
		break;
	case 0x7390:
		printf(" : Missing main sensor error -> No main sensor available. \r\n\n");
		break;
	case 0x7391:
		printf(" : Missing commutation sensor error -> No commutation sensor available \r\n\n");
		break;
	case 0x7392:
		printf(" : Main sensor direction error -> Position sensor supervision has detected a turn-away of the motor in the opposite direction \r\n\n");
		break;
	case 0x8110:
		printf(" : CAN overrun error (object lost) -> One of the CAN mail boxes experienced an overflow caused by too high communication rate \r\n\n");
		break;
	case 0x8111:
		printf(" : CAN overrun error -> Execution of CAN communication had an overrun caused by too high communication rate \r\n\n");
		break;
	case 0x8120:
		printf(" : CAN passive mode error -> Device changed to CAN passive mode \r\n\n");
		break;
	case 0x8130:
		printf(" : CAN heartbeat error -> CANopen Heartbeat Consumer Procedure or Life Guarding have detected a timeout \r\n\n");
		break;
	case 0x8150:
		printf(" : CAN PDO COB-ID collision \r\n\n");
		break;
	case 0x8180:
		printf(" : EtherCAT communication error -> EtherCAT communication error during operation enable (link lost) \r\n\n");
		break;
	case 0x8181:
		printf(" : EtherCAT initialization error -> Initialization of the Ethernet module has failed \r\n\n");
		break;
	case 0x8182:
		printf(" : EtherCAT Rx queue overflow -> The EtherCAT receive queue had an overrun caused by too high communication rate \r\n\n");
		break;
	case 0x8183:
		printf(" : EtherCAT communication error (internal) -> Internal communication of the EtherCAT module has failed \r\n\n");
		break;
	case 0x81FD:
		printf(" : CAN bus turned off -> CAN controller has entered CAN bus off state \r\n\n");
		break;
	case 0x81FE:
		printf(" : CAN Rx queue overflow -> One of the CAN receive queues had an overrun caused by too high communication rate \r\n\n");
		break;
	case 0x81FF:
		printf(" : CAN Tx queue overflow -> One of the CAN transmit queues had an overrun caused by too high communication rate \r\n\n");
		break;
	case 0x8210:
		printf(" : CAN PDO length error -> Received PDO was not processed due to length error (too short) \r\n\n");
		break;
	case 0x8250:
		printf(" : RPDO timeout -> Interpolation aborted in cyclic mode due to no PDO received after elapsed interpolation time period. \r\n\n");
		break;
	case 0x8280:
		printf(" : EtherCAT PDO communication error -> EtherCAT module detected an error at Process Data (PDO) communication \r\n\n");
		break;
	case 0x8281:
		printf(" : EtherCAT SDO communication error -> EtherCAT module detected an error at Service Data (SDO) communication \r\n\n");
		break;
	case 0x8611:
		printf(" : Following error \r\n\n");
		break;
	case 0x8A80:
		printf(" : Negative limit switch error -> Negative limit switch was/is active \r\n\n");
		break;
	case 0x8A81:
		printf(" : Positive limit switch error -> Positive limit switch was/is active \r\n\n");
		break;
	case 0x8A82:
		printf(" : Software position limit error -> Movement commanded or actual position runs out of software position limit \r\n\n");
		break;
	case 0x8A88:
		printf(" : STO error -> Error when STO is not active. STO functionality was triggered while power stage was enabled. \r\n\n");
		break;
	case 0xFF01:
		printf(" : System overloaded error -> Device has not enough free resources to process new commands \r\n\n");
		break;
	case 0xFF02:
		printf(" : Watchdog error -> Cyclic monitoring has detected an invalid device status \r\n\n");
		break;
	case 0xFF0B:
		printf(" : System peak overloaded error -> The device has not enough free resources to provide proper regulation\r\n\n");
		break;
	case 0xFF10:
		printf(" : Controller gain error -> Control function not possible due to bad controller gains \r\n\n");
		break;
	}
}